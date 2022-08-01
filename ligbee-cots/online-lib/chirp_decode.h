#ifndef PROJC_CHIRP_DECODE_H
#define PROJC_CHIRP_DECODE_H

#include <stdio.h>

// struct for reverse_encode options
struct ChirpDecoder {
    int d_decim;      //  d_decim:    decim mode, whether use oversampling recovery
    int d_gradient;  //  d_gradient:    gradient mode, whether use refinement

    int d_num_chirp_packet;
    int d_num_of_bins;
    int d_samples_per_symbol;
    int d_decim_factor;

    int d_sf;
    int d_cr;
    int d_len;

    int d_dbg_chip;
    int d_dbg_max;

    FILE *write_fp;
};

void chirp_decode(struct ChirpDecoder *chirp_decoder, const unsigned char *samples);


#endif //PROJC_CHIRP_DECODE_H
