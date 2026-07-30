#define _POSIX_C_SOURCE 200809L

#include "smod.h"

#include <errno.h>
#include <getopt.h>
#include <libgen.h>
#include <math.h>
#include <openssl/sha.h>
#include <portaudio.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <zlib.h>

#define PI 3.14159265358979323846

typedef struct {
    uint8_t *data;
    size_t size;
} Buffer;

typedef struct {
    Buffer *items;
    size_t count;
} Frames;

typedef struct {
    int baud;
    size_t chunk;
    int repeats;
    float volume;
    double gap;
    const char *device;
    const char *wav;
    int no_play;
} Options;

static void fail(void) {
    exit(1);
}

static Buffer read_file(const char *path) {
    FILE *file = fopen(path, "rb");
    struct stat st;
    Buffer result = {0};
    if (!file || fstat(fileno(file), &st) || st.st_size < 0)
        fail();
    if (!S_ISREG(st.st_mode))
        fail();
    result.size = (size_t)st.st_size;
    result.data = smod_alloc(result.size);
    if (result.size && fread(result.data, 1, result.size, file) != result.size)
        fail();
    fclose(file);
    return result;
}

static Buffer make_frame(int type, const uint8_t sid[8], uint32_t sequence,
                         uint32_t total, const uint8_t *data, size_t size) {
    Buffer frame;
    size_t body_offset = SMOD_PREAMBLE_SIZE + SMOD_SYNC_SIZE;
    size_t body_size = SMOD_FRAME_HEADER_SIZE + size;
    if (size > UINT16_MAX)
        fail();
    frame.size = body_offset + body_size + SMOD_CRC_SIZE;
    frame.data = smod_alloc(frame.size);
    memset(frame.data, 0x55, SMOD_PREAMBLE_SIZE);
    memcpy(frame.data + SMOD_PREAMBLE_SIZE, SMOD_SYNC, SMOD_SYNC_SIZE);
    uint8_t *body = frame.data + body_offset;
    body[0] = SMOD_VERSION;
    body[1] = (uint8_t)type;
    memcpy(body + 2, sid, 8);
    smod_put_be32(body + 10, sequence);
    smod_put_be32(body + 14, total);
    smod_put_be16(body + 18, (uint16_t)size);
    memcpy(body + SMOD_FRAME_HEADER_SIZE, data, size);
    smod_put_be32(body + body_size, (uint32_t)crc32(0, body, body_size));
    return frame;
}

static Frames build_frames(const char *path, const Options *options) {
    Buffer raw = read_file(path);
    uint8_t digest[SHA256_DIGEST_LENGTH];
    SHA256(raw.data, raw.size, digest);

    uLongf compressed_size = compressBound(raw.size);
    uint8_t *compressed = smod_alloc(compressed_size);
    if (compress2(compressed, &compressed_size, raw.data, raw.size, 9) != Z_OK)
        fail();
    uint8_t *payload = raw.data;
    size_t payload_len = raw.size;
    int codec = SMOD_RAW;
    if (compressed_size < raw.size) {
        payload = compressed;
        payload_len = compressed_size;
        codec = SMOD_ZLIB;
    }

    size_t chunks = payload_len ? (payload_len + options->chunk - 1) / options->chunk : 1;
    size_t meta_repeats = options->repeats + 1 > 3 ? options->repeats + 1 : 3;
    Frames frames = {
        .count = meta_repeats + chunks * (size_t)options->repeats,
        .items = smod_alloc((meta_repeats + chunks * (size_t)options->repeats) *
                            sizeof(Buffer)),
    };

    char *copy = strdup(path);
    if (!copy)
        fail();
    const char *name = basename(copy);
    size_t name_len = strlen(name);
    if (name_len > 4096)
        fail();
    size_t metadata_size = SMOD_META_HEADER_SIZE + name_len;
    uint8_t *metadata = smod_alloc(metadata_size);
    smod_put_be64(metadata, raw.size);
    smod_put_be64(metadata + 8, payload_len);
    metadata[16] = (uint8_t)codec;
    memcpy(metadata + 17, digest, 32);
    smod_put_be16(metadata + 49, (uint16_t)name_len);
    memcpy(metadata + 51, name, name_len);

    Buffer meta = make_frame(SMOD_META, digest, 0, (uint32_t)chunks,
                             metadata, metadata_size);
    size_t at = 0;
    for (size_t i = 0; i < meta_repeats; ++i) {
        frames.items[at].size = meta.size;
        frames.items[at].data = smod_alloc(meta.size);
        memcpy(frames.items[at++].data, meta.data, meta.size);
    }
    for (size_t i = 0; i < chunks; ++i) {
        size_t offset = i * options->chunk;
        size_t length = payload_len > offset ? payload_len - offset : 0;
        if (length > options->chunk)
            length = options->chunk;
        Buffer data = make_frame(SMOD_DATA, digest, (uint32_t)i,
                                 (uint32_t)chunks, payload + offset, length);
        for (int r = 0; r < options->repeats; ++r) {
            frames.items[at].size = data.size;
            frames.items[at].data = smod_alloc(data.size);
            memcpy(frames.items[at++].data, data.data, data.size);
        }
        free(data.data);
    }
    free(meta.data);
    free(metadata);
    free(copy);
    free(compressed);
    free(raw.data);
    return frames;
}

static float *modulate(const Buffer *wire, const Options *options, size_t *count) {
    int spb = SMOD_RATE / options->baud;
    *count = wire->size * 8 * (size_t)spb;
    float *samples = smod_alloc(*count * sizeof(float));
    for (size_t bit = 0; bit < wire->size * 8; ++bit) {
        int value = (wire->data[bit / 8] >> (7 - bit % 8)) & 1;
        double frequency = (value ? 4.0 : 2.0) * options->baud;
        for (int sample = 0; sample < spb; ++sample)
            samples[bit * spb + sample] =
                options->volume * sin(2.0 * PI * frequency * sample / SMOD_RATE);
    }
    size_t fade = spb / 4 > 4 ? (size_t)spb / 4 : 4;
    if (fade > *count / 2)
        fade = *count / 2;
    for (size_t i = 0; i < fade; ++i) {
        samples[i] *= (float)i / fade;
        samples[*count - 1 - i] *= (float)i / fade;
    }
    return samples;
}

static void put_wav_header(FILE *file, uint32_t samples) {
    uint8_t h[44] = {0};
    memcpy(h, "RIFF", 4);
    smod_put_le32(h + 4, 36 + samples * 2);
    memcpy(h + 8, "WAVEfmt ", 8);
    smod_put_le32(h + 16, 16);
    smod_put_le16(h + 20, 1);
    smod_put_le16(h + 22, 1);
    smod_put_le32(h + 24, SMOD_RATE);
    smod_put_le32(h + 28, SMOD_RATE * 2);
    smod_put_le16(h + 32, 2);
    smod_put_le16(h + 34, 16);
    memcpy(h + 36, "data", 4);
    smod_put_le32(h + 40, samples * 2);
    fwrite(h, 1, sizeof(h), file);
}

static void write_pcm(FILE *file, const float *samples, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        float value = samples[i] > 1 ? 1 : samples[i] < -1 ? -1 : samples[i];
        int16_t pcm = (int16_t)(value * 32767);
        uint8_t bytes[2];
        smod_put_le16(bytes, (uint16_t)pcm);
        fwrite(bytes, 1, 2, file);
    }
}

static void create_parent_directories(const char *path) {
    char *copy = strdup(path);
    if (!copy)
        fail();
    for (char *slash = copy + 1; (slash = strchr(slash, '/')); ++slash) {
        *slash = 0;
        if (mkdir(copy, 0777) && errno != EEXIST)
            fail();
        *slash = '/';
    }
    free(copy);
}

static void write_wav(const char *path, const Frames *frames,
                      const Options *options) {
    create_parent_directories(path);
    FILE *file = fopen(path, "wb");
    if (!file)
        fail();
    size_t gap_count = (size_t)llround(SMOD_RATE * options->gap);
    uint64_t samples = gap_count * (frames->count + 2);
    for (size_t i = 0; i < frames->count; ++i)
        samples += frames->items[i].size * 8 * (SMOD_RATE / options->baud);
    if (samples > (UINT32_MAX - 36) / 2)
        fail();
    put_wav_header(file, (uint32_t)samples);
    float *gap = calloc(gap_count, sizeof(float));
    write_pcm(file, gap, gap_count);
    for (size_t i = 0; i < frames->count; ++i) {
        size_t count;
        float *samples_out = modulate(&frames->items[i], options, &count);
        write_pcm(file, samples_out, count);
        write_pcm(file, gap, gap_count);
        free(samples_out);
    }
    write_pcm(file, gap, gap_count);
    free(gap);
    if (fclose(file))
        fail();
}

static PaDeviceIndex choose_device(const char *requested) {
    if (!requested)
        return Pa_GetDefaultOutputDevice();
    char *end;
    long index = strtol(requested, &end, 10);
    if (*requested && !*end)
        return (PaDeviceIndex)index;
    int count = Pa_GetDeviceCount();
    for (int i = 0; i < count; ++i)
        if (strstr(Pa_GetDeviceInfo(i)->name, requested))
            return i;
    fail();
    return paNoDevice;
}

static void play(const Frames *frames, const Options *options) {
    PaError error = Pa_Initialize();
    if (error != paNoError)
        fail();
    PaDeviceIndex device = choose_device(options->device);
    if (device == paNoDevice)
        fail();
    const PaDeviceInfo *info = Pa_GetDeviceInfo(device);
    PaStreamParameters output = {
        .device = device, .channelCount = 1, .sampleFormat = paFloat32,
        .suggestedLatency = info->defaultHighOutputLatency, .hostApiSpecificStreamInfo = NULL,
    };
    PaStream *stream = NULL;
    error = Pa_OpenStream(&stream, NULL, &output, SMOD_RATE, paFramesPerBufferUnspecified,
                          paNoFlag, NULL, NULL);
    if (error == paNoError)
        error = Pa_StartStream(stream);
    size_t gap_count = (size_t)llround(SMOD_RATE * options->gap);
    float *gap = calloc(gap_count, sizeof(float));
    if (error == paNoError)
        error = Pa_WriteStream(stream, gap, gap_count);
    for (size_t i = 0; error == paNoError && i < frames->count; ++i) {
        size_t count;
        float *samples = modulate(&frames->items[i], options, &count);
        error = Pa_WriteStream(stream, samples, count);
        if (error == paNoError)
            error = Pa_WriteStream(stream, gap, gap_count);
        free(samples);
    }
    if (error == paNoError)
        error = Pa_WriteStream(stream, gap, gap_count);
    free(gap);
    if (stream) {
        Pa_StopStream(stream);
        Pa_CloseStream(stream);
    }
    Pa_Terminate();
    if (error != paNoError)
        fail();
}

static Options parse_options(int argc, char **argv, const char **path) {
    Options o = {.baud = 4800, .chunk = 160, .repeats = 2,
                 .volume = .35f, .gap = .10};
    static const struct option long_options[] = {
        {"baud", required_argument, 0, 'b'}, {"chunk-size", required_argument, 0, 'c'},
        {"repeats", required_argument, 0, 'r'}, {"volume", required_argument, 0, 'v'},
        {"gap", required_argument, 0, 'g'}, {"device", required_argument, 0, 'd'},
        {"wav", required_argument, 0, 'w'}, {"no-play", no_argument, 0, 'n'},
        {"help", no_argument, 0, 'h'}, {0, 0, 0, 0},
    };
    opterr = 0;
    int option;
    while ((option = getopt_long(argc, argv, "b:c:r:v:g:d:w:nh", long_options, NULL)) != -1) {
        switch (option) {
        case 'b': o.baud = atoi(optarg); break;
        case 'c': o.chunk = strtoul(optarg, NULL, 10); break;
        case 'r': o.repeats = atoi(optarg); break;
        case 'v': o.volume = strtof(optarg, NULL); break;
        case 'g': o.gap = strtod(optarg, NULL); break;
        case 'd': o.device = optarg; break;
        case 'w': o.wav = optarg; break;
        case 'n': o.no_play = 1; break;
        default:
            exit(option == 'h' ? 0 : 2);
        }
    }
    if (optind + 1 != argc)
        fail();
    *path = argv[optind];
    if (!(o.baud == 600 || o.baud == 1200 || o.baud == 2400 || o.baud == 4800))
        fail();
    if (o.chunk < 16 || o.chunk > 1024)
        fail();
    if (o.repeats < 1 || o.repeats > 8)
        fail();
    if (o.volume < .01 || o.volume > 1)
        fail();
    if (o.gap < .05 || o.gap > .50)
        fail();
    if (o.no_play && !o.wav)
        fail();
    return o;
}

int main(int argc, char **argv) {
    const char *path;
    Options options = parse_options(argc, argv, &path);
    Frames frames = build_frames(path, &options);
    if (options.wav) {
        write_wav(options.wav, &frames, &options);
    }
    if (!options.no_play) {
        play(&frames, &options);
    }
    for (size_t i = 0; i < frames.count; ++i)
        free(frames.items[i].data);
    free(frames.items);
    return 0;
}
