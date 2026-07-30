#define _POSIX_C_SOURCE 200809L

#include "smod.h"

#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <openssl/sha.h>
#include <portaudio.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <zlib.h>

#define PI 3.14159265358979323846

typedef struct {
    float *data;
    size_t count;
    int rate;
} Samples;

typedef struct {
    uint8_t type;
    uint8_t sid[8];
    uint32_t sequence;
    uint32_t total;
    uint8_t *data;
    size_t size;
} Frame;

typedef struct {
    Frame *items;
    size_t count;
    size_t capacity;
} Frames;

typedef struct {
    uint8_t sid[8];
    char *filename;
    uint64_t original_size;
    uint64_t payload_size;
    uint8_t codec;
    uint8_t digest[32];
    uint32_t chunks;
} Metadata;

typedef struct {
    const char *output_dir;
    const char *input_wav;
    const char *device;
    int baud;
    double gap;
    double threshold;
    double calibrate;
    double start_hold;
    double silence;
    double timeout;
    int overwrite;
} Options;

typedef struct {
    Metadata metadata;
    uint8_t *seen;
    size_t valid_frames;
    size_t received;
    uint64_t payload_bytes;
    struct timespec started;
    int cannot_complete;
    uint32_t missing_sequence;
    uint32_t observed_sequence;
} ReceiveProgress;

static void fail(const char *message) {
    fprintf(stderr, "error: %s\n", message);
    exit(1);
}

static uint32_t le32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 |
           (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static uint16_t le16(const uint8_t *p) {
    return (uint16_t)(p[0] | p[1] << 8);
}

static uint8_t *read_all(const char *path, size_t *size) {
    FILE *file = fopen(path, "rb");
    struct stat st;
    if (!file || fstat(fileno(file), &st) || st.st_size < 0)
        fail(strerror(errno));
    *size = (size_t)st.st_size;
    uint8_t *data = smod_alloc(*size);
    if (*size && fread(data, 1, *size, file) != *size)
        fail("could not read input file");
    fclose(file);
    return data;
}

static Samples read_wav(const char *path) {
    size_t size;
    uint8_t *file = read_all(path, &size);
    if (size < 12 || memcmp(file, "RIFF", 4) || memcmp(file + 8, "WAVE", 4))
        fail("input is not a RIFF/WAVE file");
    uint16_t format = 0, channels = 0, bits = 0;
    uint32_t rate = 0;
    const uint8_t *pcm = NULL;
    size_t pcm_size = 0;
    for (size_t at = 12; at + 8 <= size;) {
        uint32_t length = le32(file + at + 4);
        size_t next = at + 8 + length + (length & 1);
        if (next > size)
            fail("truncated WAV chunk");
        if (!memcmp(file + at, "fmt ", 4) && length >= 16) {
            format = le16(file + at + 8);
            channels = le16(file + at + 10);
            rate = le32(file + at + 12);
            bits = le16(file + at + 22);
        } else if (!memcmp(file + at, "data", 4)) {
            pcm = file + at + 8;
            pcm_size = length;
        }
        at = next;
    }
    if (format != 1 || !channels || !pcm || !(bits == 8 || bits == 16 || bits == 32))
        fail("unsupported WAV format");
    size_t width = bits / 8;
    size_t frames = pcm_size / (width * channels);
    Samples result = {.data = smod_alloc(frames * sizeof(float)),
                      .count = frames, .rate = (int)rate};
    for (size_t i = 0; i < frames; ++i) {
        double sum = 0;
        for (uint16_t c = 0; c < channels; ++c) {
            const uint8_t *p = pcm + (i * channels + c) * width;
            if (bits == 8)
                sum += ((int)p[0] - 128) / 128.0;
            else if (bits == 16)
                sum += (int16_t)le16(p) / 32768.0;
            else
                sum += (int32_t)le32(p) / 2147483648.0;
        }
        result.data[i] = (float)(sum / channels);
    }
    free(file);
    return result;
}

static int compare_float(const void *a, const void *b) {
    float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

static float percentile(const float *values, size_t count, double fraction) {
    float *copy = smod_alloc(count * sizeof(float));
    memcpy(copy, values, count * sizeof(float));
    qsort(copy, count, sizeof(float), compare_float);
    size_t index = (size_t)floor((count - 1) * fraction);
    float result = copy[index];
    free(copy);
    return result;
}

static Frame parse_frame(const uint8_t *bytes, size_t size, size_t sync) {
    Frame invalid = {0};
    size_t start = sync + SMOD_SYNC_SIZE;
    if (start + SMOD_FRAME_HEADER_SIZE + SMOD_CRC_SIZE > size)
        return invalid;
    const uint8_t *body = bytes + start;
    if (body[0] != SMOD_VERSION || !(body[1] == SMOD_META || body[1] == SMOD_DATA))
        return invalid;
    size_t data_size = smod_be16(body + 18);
    size_t body_size = SMOD_FRAME_HEADER_SIZE + data_size;
    if (start + body_size + SMOD_CRC_SIZE > size)
        return invalid;
    if ((uint32_t)crc32(0, body, body_size) != smod_be32(body + body_size))
        return invalid;
    Frame frame = {.type = body[1], .sequence = smod_be32(body + 10),
                   .total = smod_be32(body + 14), .size = data_size};
    memcpy(frame.sid, body + 2, 8);
    frame.data = smod_alloc(data_size);
    memcpy(frame.data, body + SMOD_FRAME_HEADER_SIZE, data_size);
    return frame;
}

static uint8_t *demodulate(const float *samples, size_t count, int baud,
                           int phase, size_t *bit_count) {
    int spb = SMOD_RATE / baud;
    *bit_count = count > (size_t)phase ? (count - phase) / spb : 0;
    uint8_t *bits = smod_alloc(*bit_count);
    for (size_t bit = 0; bit < *bit_count; ++bit) {
        const float *data = samples + phase + bit * spb;
        double mean = 0;
        for (int i = 0; i < spb; ++i)
            mean += data[i];
        mean /= spb;
        double e[2] = {0, 0};
        for (int tone = 0; tone < 2; ++tone) {
            double cosine = 0, sine = 0;
            double frequency = (tone ? 4.0 : 2.0) * baud;
            for (int i = 0; i < spb; ++i) {
                double window = spb == 1 ? 1 :
                    .5 - .5 * cos(2 * PI * i / (spb - 1));
                double value = (data[i] - mean) * window;
                cosine += value * cos(2 * PI * frequency * i / SMOD_RATE);
                sine += value * sin(2 * PI * frequency * i / SMOD_RATE);
            }
            e[tone] = cosine * cosine + sine * sine;
        }
        bits[bit] = e[1] > e[0];
    }
    return bits;
}

static Frame decode_segment(const float *samples, size_t count, int baud) {
    int spb = SMOD_RATE / baud;
    for (int phase = 0; phase < spb; ++phase) {
        size_t bit_count;
        uint8_t *bits = demodulate(samples, count, baud, phase, &bit_count);
        for (int alignment = 0; alignment < 8; ++alignment) {
            size_t byte_count = bit_count > (size_t)alignment ?
                                (bit_count - alignment) / 8 : 0;
            uint8_t *bytes = smod_alloc(byte_count);
            for (size_t byte = 0; byte < byte_count; ++byte) {
                uint8_t value = 0;
                for (int bit = 0; bit < 8; ++bit)
                    value = (uint8_t)(value << 1 |
                        bits[alignment + byte * 8 + bit]);
                bytes[byte] = value;
            }
            for (size_t at = 0; at + SMOD_SYNC_SIZE <= byte_count; ++at) {
                if (!memcmp(bytes + at, SMOD_SYNC, SMOD_SYNC_SIZE)) {
                    Frame frame = parse_frame(bytes, byte_count, at);
                    if (frame.type) {
                        free(bytes);
                        free(bits);
                        return frame;
                    }
                }
            }
            free(bytes);
        }
        free(bits);
    }
    Frame invalid = {0};
    return invalid;
}

static void append_frame(Frames *frames, Frame frame) {
    if (frames->count == frames->capacity) {
        frames->capacity = frames->capacity ? frames->capacity * 2 : 16;
        Frame *items = realloc(frames->items, frames->capacity * sizeof(Frame));
        if (!items)
            fail("out of memory");
        frames->items = items;
    }
    frames->items[frames->count++] = frame;
}

static Frames decode_capture(const Samples *samples, int baud, double expected_gap) {
    if (samples->rate != SMOD_RATE)
        fail("recording must use a 48000 Hz sample rate");
    size_t block_size = (size_t)llround(SMOD_RATE * .005);
    size_t blocks = samples->count / block_size;
    if (blocks < 2)
        fail("captured signal is too short");
    float *rms = smod_alloc(blocks * sizeof(float));
    uint8_t *active = calloc(blocks, 1);
    for (size_t block = 0; block < blocks; ++block) {
        double energy = 1e-12;
        for (size_t i = 0; i < block_size; ++i) {
            double value = samples->data[block * block_size + i];
            energy += value * value;
        }
        rms[block] = (float)sqrt(energy / block_size);
    }
    float noise = percentile(rms, blocks, .01);
    float signal = percentile(rms, blocks, .90);
    if (signal <= noise * 1.5)
        fail("no clear modem signal found in recording");
    float threshold = noise + .22f * (signal - noise);
    for (size_t i = 0; i < blocks; ++i)
        active[i] = rms[i] > threshold;
    for (size_t i = 1; i + 1 < blocks;) {
        if (active[i]) {
            ++i;
            continue;
        }
        size_t start = i;
        while (i < blocks && !active[i])
            ++i;
        if (active[start - 1] && i < blocks && i - start <= 4)
            memset(active + start, 1, i - start);
    }

    size_t minimum_bytes = SMOD_PREAMBLE_SIZE + SMOD_SYNC_SIZE +
                           SMOD_FRAME_HEADER_SIZE + SMOD_CRC_SIZE;
    size_t minimum_blocks = (size_t)floor(minimum_bytes * 8.0 / baud * .75 / .005);
    size_t padding = (size_t)llround(fmin(expected_gap * .20, .020) / .005);
    if (!padding)
        padding = 1;
    Frames frames = {0};
    for (size_t i = 0; i < blocks;) {
        while (i < blocks && !active[i])
            ++i;
        size_t start = i;
        while (i < blocks && active[i])
            ++i;
        size_t end = i;
        if (end > start && end - start >= minimum_blocks) {
            size_t first = start > padding ? start - padding : 0;
            size_t last = end + padding < blocks ? end + padding : blocks;
            Frame frame = decode_segment(samples->data + first * block_size,
                                         (last - first) * block_size, baud);
            if (frame.type)
                append_frame(&frames, frame);
        }
    }
    free(active);
    free(rms);
    if (!frames.count)
        fail("no valid frames decoded");
    return frames;
}

static int metadata_from_frame(const Frame *frame, Metadata *metadata) {
    if (frame->type != SMOD_META || frame->size < SMOD_META_HEADER_SIZE)
        return 0;
    uint16_t name_len = smod_be16(frame->data + 49);
    if (frame->data[16] > SMOD_ZLIB ||
        (size_t)SMOD_META_HEADER_SIZE + name_len != frame->size ||
        memcmp(frame->sid, frame->data + 17, 8) || !frame->total)
        return 0;
    metadata->original_size = smod_be64(frame->data);
    metadata->payload_size = smod_be64(frame->data + 8);
    metadata->codec = frame->data[16];
    memcpy(metadata->digest, frame->data + 17, 32);
    memcpy(metadata->sid, frame->sid, 8);
    metadata->chunks = frame->total;
    metadata->filename = smod_alloc(name_len + 1);
    memcpy(metadata->filename, frame->data + 51, name_len);
    metadata->filename[name_len] = 0;
    char *slash = strrchr(metadata->filename, '/');
    char *backslash = strrchr(metadata->filename, '\\');
    char *base = slash > backslash ? slash : backslash;
    if (base) {
        char *safe = strdup(base + 1);
        free(metadata->filename);
        metadata->filename = safe;
    }
    if (!metadata->filename[0] || !strcmp(metadata->filename, ".") ||
        !strcmp(metadata->filename, "..")) {
        free(metadata->filename);
        metadata->filename = smod_alloc(30);
        snprintf(metadata->filename, 30, "received-%02x%02x%02x%02x.bin",
                 frame->sid[0], frame->sid[1], frame->sid[2], frame->sid[3]);
    }
    return 1;
}

static uint8_t *reconstruct(const Frames *frames, Metadata *metadata) {
    int found = 0;
    for (size_t i = 0; i < frames->count && !found; ++i)
        found = metadata_from_frame(&frames->items[i], metadata);
    if (!found)
        fail("no valid metadata frame was received");
    if (metadata->original_size > SIZE_MAX || metadata->payload_size > SIZE_MAX)
        fail("received file is too large");
    uint8_t **chunks = calloc(metadata->chunks, sizeof(uint8_t *));
    size_t *sizes = calloc(metadata->chunks, sizeof(size_t));
    for (size_t i = 0; i < frames->count; ++i) {
        Frame *frame = &frames->items[i];
        if (frame->type == SMOD_DATA && !memcmp(frame->sid, metadata->sid, 8) &&
            frame->total == metadata->chunks && frame->sequence < metadata->chunks &&
            !chunks[frame->sequence]) {
            chunks[frame->sequence] = frame->data;
            sizes[frame->sequence] = frame->size;
        }
    }
    size_t payload_size = 0;
    for (uint32_t i = 0; i < metadata->chunks; ++i) {
        if (!chunks[i])
            fail("one or more data chunks are missing");
        payload_size += sizes[i];
    }
    if (payload_size != metadata->payload_size)
        fail("payload length mismatch");
    uint8_t *payload = smod_alloc(payload_size);
    size_t at = 0;
    for (uint32_t i = 0; i < metadata->chunks; ++i) {
        memcpy(payload + at, chunks[i], sizes[i]);
        at += sizes[i];
    }
    uint8_t *raw = smod_alloc((size_t)metadata->original_size);
    if (metadata->codec == SMOD_ZLIB) {
        uLongf raw_size = (uLongf)metadata->original_size;
        if (uncompress(raw, &raw_size, payload, payload_size) != Z_OK ||
            raw_size != metadata->original_size)
            fail("received compressed data could not be decompressed");
    } else {
        if (payload_size != metadata->original_size)
            fail("file length mismatch");
        memcpy(raw, payload, payload_size);
    }
    uint8_t digest[SHA256_DIGEST_LENGTH];
    SHA256(raw, (size_t)metadata->original_size, digest);
    if (memcmp(digest, metadata->digest, 32))
        fail("final SHA-256 verification failed");
    free(payload);
    free(chunks);
    free(sizes);
    return raw;
}

static double elapsed_since(const struct timespec *started) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now.tv_sec - started->tv_sec +
           (now.tv_nsec - started->tv_nsec) / 1000000000.0;
}

static void display_progress(const ReceiveProgress *progress, int finished) {
    if (!isatty(fileno(stderr)) && !finished)
        return;
    double fraction = progress->metadata.chunks ?
        (double)progress->received / progress->metadata.chunks : 0;
    int filled = (int)llround(24 * fraction);
    char bar[25];
    for (int i = 0; i < 24; ++i)
        bar[i] = i < filled ? '#' : '-';
    bar[24] = 0;

    double elapsed = fmax(.001, elapsed_since(&progress->started));
    double rate = progress->payload_bytes / elapsed;
    double remaining = progress->metadata.payload_size > progress->payload_bytes ?
        (double)(progress->metadata.payload_size - progress->payload_bytes) : 0;
    char rate_text[32], eta_text[32];
    if (rate >= 1024)
        snprintf(rate_text, sizeof(rate_text), "%.1f KiB/s", rate / 1024);
    else if (rate > 0)
        snprintf(rate_text, sizeof(rate_text), "%.0f B/s", rate);
    else
        strcpy(rate_text, "-- B/s");
    if (rate > 0)
        snprintf(eta_text, sizeof(eta_text), "ETA %.0fs", ceil(remaining / rate));
    else
        strcpy(eta_text, "ETA --");

    if (isatty(fileno(stderr)))
        fprintf(stderr, "\r\033[2KRX [%s] %6.1f%% [%s] [%s]%s",
                bar, fraction * 100, rate_text, eta_text, finished ? "\n" : "");
    else
        fprintf(stderr, "RX [%s] %6.1f%% [%s] [%s]\n",
                bar, fraction * 100, rate_text, eta_text);
    fflush(stderr);
}

static void process_progress_segment(ReceiveProgress *progress,
                                     const float *samples, size_t count,
                                     int baud) {
    Frame frame = decode_segment(samples, count, baud);
    if (!frame.type)
        return;
    ++progress->valid_frames;
    if (!progress->metadata.chunks) {
        Metadata metadata = {0};
        if (metadata_from_frame(&frame, &metadata)) {
            progress->metadata = metadata;
            progress->seen = calloc(metadata.chunks, 1);
            if (!progress->seen)
                fail("out of memory");
        }
    } else if (frame.type == SMOD_DATA &&
               !memcmp(frame.sid, progress->metadata.sid, 8) &&
               frame.total == progress->metadata.chunks &&
               frame.sequence < progress->metadata.chunks) {
        for (uint32_t sequence = 0; sequence < frame.sequence; ++sequence) {
            if (!progress->seen[sequence]) {
                progress->cannot_complete = 1;
                progress->missing_sequence = sequence;
                progress->observed_sequence = frame.sequence;
                break;
            }
        }
        if (!progress->cannot_complete && !progress->seen[frame.sequence]) {
            progress->seen[frame.sequence] = 1;
            ++progress->received;
            progress->payload_bytes += frame.size;
        }
    }
    free(frame.data);
    display_progress(progress, 0);
}

static PaDeviceIndex choose_input(const char *requested) {
    if (!requested) {
        PaDeviceIndex fallback = Pa_GetDefaultInputDevice();
        const PaDeviceInfo *fallback_info = Pa_GetDeviceInfo(fallback);
        if (!fallback_info || fallback_info->maxInputChannels < 32)
            return fallback;
        PaDeviceIndex best = paNoDevice;
        int best_score = -1;
        int count = Pa_GetDeviceCount();
        for (int i = 0; i < count; ++i) {
            const PaDeviceInfo *info = Pa_GetDeviceInfo(i);
            if (!strstr(info->name, "(hw:") || info->maxInputChannels < 1)
                continue;
            int score = (strstr(info->name, "Analog") != NULL) * 2 +
                        (info->maxOutputChannels > 0);
            if (score >= best_score) {
                best = i;
                best_score = score;
            }
        }
        if (best != paNoDevice)
            fprintf(stderr, "Using audio input device %d: %s\n", best,
                    Pa_GetDeviceInfo(best)->name);
        return best == paNoDevice ? fallback : best;
    }
    char *end;
    long index = strtol(requested, &end, 10);
    if (*requested && !*end)
        return (PaDeviceIndex)index;
    int count = Pa_GetDeviceCount();
    for (int i = 0; i < count; ++i)
        if (strstr(Pa_GetDeviceInfo(i)->name, requested))
            return i;
    fail("audio input device not found");
    return paNoDevice;
}

static double block_rms(const float *data, size_t count) {
    double mean = 0, energy = 1e-12;
    for (size_t i = 0; i < count; ++i)
        mean += data[i];
    mean /= count;
    for (size_t i = 0; i < count; ++i) {
        double value = data[i] - mean;
        energy += value * value;
    }
    return sqrt(energy / count);
}

static Samples record_audio(const Options *o) {
    PaError error = Pa_Initialize();
    if (error != paNoError)
        fail(Pa_GetErrorText(error));
    PaDeviceIndex device = choose_input(o->device);
    if (device == paNoDevice)
        fail("no default audio input device");
    const PaDeviceInfo *info = Pa_GetDeviceInfo(device);
    PaStreamParameters input = {
        .device = device, .channelCount = 1, .sampleFormat = paFloat32,
        .suggestedLatency = info->defaultHighInputLatency, .hostApiSpecificStreamInfo = NULL,
    };
    PaStream *stream = NULL;
    size_t block = (size_t)llround(SMOD_RATE * .02);
    error = Pa_OpenStream(&stream, &input, NULL, SMOD_RATE, block, paNoFlag, NULL, NULL);
    if (error == paNoError)
        error = Pa_StartStream(stream);
    if (error != paNoError)
        fail(Pa_GetErrorText(error));
    float *buffer = smod_alloc(block * sizeof(float));
    int calibration_blocks = (int)llround(o->calibrate / .02);
    if (calibration_blocks < 1)
        calibration_blocks = 1;
    float *calibration = smod_alloc(calibration_blocks * sizeof(float));
    printf("Calibrating input for %.1f seconds; keep TX stopped.\n", o->calibrate);
    fflush(stdout);
    for (int i = 0; i < 20; ++i)
        Pa_ReadStream(stream, buffer, block);
    for (int i = 0; i < calibration_blocks; ++i) {
        Pa_ReadStream(stream, buffer, block);
        calibration[i] = (float)block_rms(buffer, block);
    }
    double noise = percentile(calibration, calibration_blocks, .95);
    double trigger = fmax(o->threshold, noise * 2.5);
    printf("Listening. Noise RMS %.4f; trigger %.4f. Start the transmitter now.\n",
           noise, trigger);
    fflush(stdout);
    size_t maximum_idle_blocks = (size_t)llround(o->timeout / .02);
    size_t capacity = SMOD_RATE;
    float *captured = smod_alloc(capacity * sizeof(float));
    size_t count = 0;
    size_t segment_start = 0;
    int started = 0, loud = 0, quiet = 0;
    int loud_needed = (int)llround(o->start_hold / .02);
    int quiet_needed = (int)llround(o->silence / .02);
    if (loud_needed < 1)
        loud_needed = 1;
    if (quiet_needed < 1)
        quiet_needed = 1;
    size_t pre_blocks = 25, pre_count = 0, pre_next = 0;
    float *pre_roll = smod_alloc(pre_blocks * block * sizeof(float));
    ReceiveProgress progress = {0};
    size_t idle_blocks = 0;
    int inactivity_timeout = 0;
    for (;;) {
        if (++idle_blocks > maximum_idle_blocks) {
            inactivity_timeout = 1;
            break;
        }
        error = Pa_ReadStream(stream, buffer, block);
        if (error != paNoError && error != paInputOverflowed)
            fail(Pa_GetErrorText(error));
        double rms = block_rms(buffer, block);
        if (!started) {
            memcpy(pre_roll + pre_next * block, buffer, block * sizeof(float));
            pre_next = (pre_next + 1) % pre_blocks;
            if (pre_count < pre_blocks)
                ++pre_count;
            loud = rms >= trigger ? loud + 1 : 0;
            if (loud < loud_needed)
                continue;
            started = 1;
            idle_blocks = 0;
            clock_gettime(CLOCK_MONOTONIC, &progress.started);
            puts("Modem signal detected.");
            fflush(stdout);
            size_t first = pre_count == pre_blocks ? pre_next : 0;
            for (size_t i = 0; i < pre_count; ++i) {
                size_t source = (first + i) % pre_blocks;
                memcpy(captured + count, pre_roll + source * block,
                       block * sizeof(float));
                count += block;
            }
            continue;
        }
        if (count + block > capacity) {
            capacity *= 2;
            float *grown = realloc(captured, capacity * sizeof(float));
            if (!grown)
                fail("out of memory");
            captured = grown;
        }
        memcpy(captured + count, buffer, block * sizeof(float));
        count += block;
        quiet = rms < trigger * .55 ? quiet + 1 : 0;
        if (quiet == 3) {
            size_t valid_frames = progress.valid_frames;
            process_progress_segment(&progress, captured + segment_start,
                                     count - segment_start, o->baud);
            segment_start = count;
            if (progress.valid_frames > valid_frames)
                idle_blocks = 0;
            if (progress.cannot_complete)
                break;
        }
        if (quiet >= quiet_needed)
            break;
    }
    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();
    free(calibration);
    free(buffer);
    free(pre_roll);
    if (!started)
        fail("no modem signal detected before the receive timeout");
    display_progress(&progress, 1);
    free(progress.seen);
    free(progress.metadata.filename);
    if (progress.cannot_complete) {
        char message[160];
        snprintf(message, sizeof(message),
                 "chunk %u is missing; chunk %u has already arrived, "
                 "so this transmission cannot complete",
                 progress.missing_sequence, progress.observed_sequence);
        free(captured);
        fail(message);
    }
    if (inactivity_timeout) {
        free(captured);
        fail("receiver timed out waiting for another valid frame");
    }
    Samples result = {.data = captured, .count = count, .rate = SMOD_RATE};
    return result;
}

static char *output_path(const Options *o, const char *filename) {
    char *directory = strdup(o->output_dir);
    if (!directory)
        fail("out of memory");
    for (char *slash = directory + 1;; ++slash) {
        slash = strchr(slash, '/');
        if (slash)
            *slash = 0;
        if (mkdir(directory, 0777) && errno != EEXIST)
            fail(strerror(errno));
        if (!slash)
            break;
        *slash = '/';
    }
    free(directory);
    size_t length = strlen(o->output_dir) + strlen(filename) + 32;
    char *path = smod_alloc(length);
    snprintf(path, length, "%s/%s", o->output_dir, filename);
    if (o->overwrite || access(path, F_OK))
        return path;
    for (unsigned counter = 1;; ++counter) {
        snprintf(path, length, "%s/%s.%u", o->output_dir, filename, counter);
        if (access(path, F_OK))
            return path;
    }
}

static Options parse_options(int argc, char **argv) {
    Options o = {.output_dir = ".", .baud = 4800, .gap = .10,
                 .threshold = .0005, .calibrate = 1, .start_hold = .12,
                 .silence = 1, .timeout = 600};
    static const struct option options[] = {
        {"baud", required_argument, 0, 'b'}, {"gap", required_argument, 0, 'g'},
        {"device", required_argument, 0, 'd'}, {"threshold", required_argument, 0, 't'},
        {"calibrate", required_argument, 0, 'c'}, {"start-hold", required_argument, 0, 's'},
        {"silence", required_argument, 0, 'q'}, {"timeout", required_argument, 0, 'T'},
        {"input-wav", required_argument, 0, 'i'}, {"overwrite", no_argument, 0, 'o'},
        {"help", no_argument, 0, 'h'}, {0, 0, 0, 0},
    };
    int option;
    while ((option = getopt_long(argc, argv, "b:g:d:t:c:s:q:T:i:oh", options, NULL)) != -1) {
        switch (option) {
        case 'b': o.baud = atoi(optarg); break;
        case 'g': o.gap = strtod(optarg, NULL); break;
        case 'd': o.device = optarg; break;
        case 't': o.threshold = strtod(optarg, NULL); break;
        case 'c': o.calibrate = strtod(optarg, NULL); break;
        case 's': o.start_hold = strtod(optarg, NULL); break;
        case 'q': o.silence = strtod(optarg, NULL); break;
        case 'T': o.timeout = strtod(optarg, NULL); break;
        case 'i': o.input_wav = optarg; break;
        case 'o': o.overwrite = 1; break;
        default:
            printf("usage: rx [DESTINATION_DIR] [--baud RATE] [--gap SEC]\n"
                   "          [--device DEVICE] [--threshold N] [--calibrate SEC]\n"
                   "          [--start-hold SEC] [--silence SEC] [--timeout SEC]\n"
                   "          [--input-wav FILE] [--overwrite]\n");
            exit(option == 'h' ? 0 : 2);
        }
    }
    if (optind < argc)
        o.output_dir = argv[optind++];
    if (optind != argc)
        fail("too many destination directories");
    if (!(o.baud == 600 || o.baud == 1200 || o.baud == 2400 || o.baud == 4800))
        fail("--baud must be 600, 1200, 2400, or 4800");
    if (o.gap <= 0 || o.threshold < 0 || o.calibrate <= 0 ||
        o.start_hold <= 0 || o.silence <= 0 || o.timeout <= 0)
        fail("timing and threshold options must be positive");
    return o;
}

int main(int argc, char **argv) {
    Options options = parse_options(argc, argv);
    Samples samples = options.input_wav ? read_wav(options.input_wav) :
                                         record_audio(&options);
    if (options.input_wav)
        printf("Decoding %s\n", options.input_wav);
    Frames frames = decode_capture(&samples, options.baud, options.gap);
    Metadata metadata = {0};
    uint8_t *raw = reconstruct(&frames, &metadata);
    char *path = output_path(&options, metadata.filename);
    FILE *file = fopen(path, "wb");
    if (!file || (metadata.original_size &&
        fwrite(raw, 1, (size_t)metadata.original_size, file) != metadata.original_size) ||
        fclose(file))
        fail("could not write received file");
    printf("Received %s (%llu bytes); SHA-256 verified. Decoded %zu valid frames.\n",
           path, (unsigned long long)metadata.original_size, frames.count);
    free(path);
    free(raw);
    free(metadata.filename);
    free(samples.data);
    for (size_t i = 0; i < frames.count; ++i)
        free(frames.items[i].data);
    free(frames.items);
    return 0;
}
