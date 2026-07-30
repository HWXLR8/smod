#ifndef SMOD_H
#define SMOD_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define SMOD_VERSION 1
#define SMOD_RATE 48000
#define SMOD_META 1
#define SMOD_DATA 2
#define SMOD_RAW 0
#define SMOD_ZLIB 1
#define SMOD_PREAMBLE_SIZE 12
#define SMOD_SYNC_SIZE 8
#define SMOD_FRAME_HEADER_SIZE 20
#define SMOD_META_HEADER_SIZE 51
#define SMOD_CRC_SIZE 4

static const uint8_t SMOD_SYNC[SMOD_SYNC_SIZE] =
    {0xd3, 0x91, 0xc5, 0xa7, 0x5a, 0x6e, 0x2b, 0xf0};

static inline uint16_t smod_be16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] << 8 | p[1]);
}

static inline uint32_t smod_be32(const uint8_t *p) {
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 |
           (uint32_t)p[2] << 8 | p[3];
}

static inline uint64_t smod_be64(const uint8_t *p) {
    return (uint64_t)smod_be32(p) << 32 | smod_be32(p + 4);
}

static inline void smod_put_be16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static inline void smod_put_be32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static inline void smod_put_be64(uint8_t *p, uint64_t v) {
    smod_put_be32(p, (uint32_t)(v >> 32));
    smod_put_be32(p + 4, (uint32_t)v);
}

static inline void smod_put_le16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static inline void smod_put_le32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static inline void *smod_alloc(size_t size) {
    void *p = malloc(size ? size : 1);
    if (!p) {
        fputs("error: out of memory\n", stderr);
        exit(1);
    }
    return p;
}

#endif
