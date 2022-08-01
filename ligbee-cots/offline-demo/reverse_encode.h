#ifndef PROJC_REVERSE_ENCODE_H
#define PROJC_REVERSE_ENCODE_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

struct ReverseEncoder{
    int d_sf;
    int d_cr;
    int d_len;
    int d_packet_len;
    int d_payload_len;
    int d_num_bytes;
    int d_num_symbols;

    int d_symbols_done;
    int d_interleave_offset;
    int d_encode_offset;
    int d_packet_offset;

    uint8_t  *d_packet;
    uint8_t *d_encoded;
    int *d_symbols;
};

void init_encode(struct ReverseEncoder *decoder, int *array, int sf, int cr, int len);

int reverse_encode(struct ReverseEncoder *decoder);

int deinterleave(struct ReverseEncoder *decoder, uint8_t reduce_rate);

void deshuffle(struct ReverseEncoder *decoder);

void dewhiten(struct ReverseEncoder *decoder);

void dehamming(struct ReverseEncoder *decoder);

void release(struct ReverseEncoder *decoder);

void save_to_file(struct ReverseEncoder *decoder, FILE *fp);

/* util functions */
uint32_t rotl(uint32_t bits, uint32_t count, const uint32_t size);
uint8_t gray_encode(uint8_t symbol);
uint8_t hamming_decode_soft_byte(uint8_t v);
uint8_t bit(const uint8_t v, const uint8_t i);
uint8_t pack_nibble(const uint8_t a, const uint8_t b, const uint8_t c, const uint8_t d);

/* for debug*/
void print_value(const uint8_t *array, size_t len);

#endif //PROJC_REVERSE_ENCODE_H
