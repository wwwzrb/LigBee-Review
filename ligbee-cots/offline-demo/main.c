#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <sys/time.h>
#include <math.h>

#include "chirp_decode.h"

int decim_factor,
        sampling_rate,
        bandwidth,
        sf,
        cr,
        len,
        d_num_byte_packet,
        d_num_chirp_packet,
        d_samples_per_second,
        d_bw,
        d_sf,
        d_cr,
        d_samples_per_symbol,
        d_num_of_bins,
        d_decim_factor,
        d_dbg_max,
        d_dbg_read_trace,
        d_dbg_chirp;

FILE *write_fp = NULL;      // for write file


/*
 * for testing，read packet trace
 *  samples：    packet trace bitarray，HEX
 *  str:        file name
 *  packet_id:  id of line, each line is a packet
 */
void read_packet_trace(unsigned char samples[], const char *str, int packet_id) {
    FILE *fp = NULL;

    if (NULL == (fp = fopen(str, "r"))) {
        printf("read file error\n");
        exit(1);
    }

    char line[120000];   // at least 20000 chars for each line
    int cnt = 0;
    char s[3] = "00";
    while (fscanf(fp, "%[^\n]", line) != EOF) {
        fgetc(fp);
        if (line[0] == '#') continue;
        else if (packet_id > 0) {
            packet_id--;
            if (d_dbg_read_trace){
                printf("id: %d, ", packet_id);
            }
        }
        else {
            if (d_dbg_read_trace){
                printf("\nChirp Readline: %s\n", line);
                printf("\nRead Samples:\n");
            }

            for (int i = 0; line[i] != '\0'; i++) {
                if (!isalnum(line[i])) continue;
                if (line[i] == '0' && (line[i + 1] == 'x' || line[i + 1] == 'X')) i += 2;
                else continue;
                if (isalnum(line[i + 1])) {
                    s[0] = line[i];
                    s[1] = line[i + 1];
                    i += 2;
                    samples[cnt++] = (unsigned char) strtol(s, NULL, 16);
                    if (d_dbg_read_trace){
                        printf("%x ", samples[cnt-1]);
                    }
                } else {
                    s[0] = line[i];
                    s[1] = '\0';
                    i += 2;
                    samples[cnt++] = (unsigned char) strtol(s, NULL, 16);
                    if (d_dbg_read_trace){
                        printf("%x ", samples[cnt-1]);
                    }
                }
            }
            if (d_dbg_read_trace){
                printf("\n");
            }
            break;
        }
    }
    fclose(fp);
}

int decode_trace()
{
    unsigned char samples[22000];
    double elapsed_list[100];
    int elapsed_total = 0;
    int cnt_total = 0;
    double elapsed_us;
    struct timeval tv_start, tv_end;

    // init
    sampling_rate   = 1000000;       // 1M
    bandwidth       = 125000;        // 125e3

    sf = 7;
    cr = 4;
    len = 4;
    d_num_byte_packet = len;
    d_num_chirp_packet = (int)(8 + (4 + cr) * ceil(len * 2.0 / sf));

    d_samples_per_second = sampling_rate;
    d_bw = bandwidth;
    d_sf = sf;
    d_cr = 4;

    // some constants
    d_samples_per_symbol    = (1 << d_sf) * d_samples_per_second / d_bw;
    d_num_of_bins           = 1 << d_sf;
    d_decim_factor          = d_samples_per_symbol / d_num_of_bins;

    // decoding settings
    int d_decim = 1, d_gradient = 1;
    d_dbg_max           = 0;                      // whether print max correlation
    d_dbg_read_trace    = 0;                 // whether print read packet info
    d_dbg_chirp         = 0;                      //  whether print chirp bitarray info

//    char write_path[40] = "../RealTrace/";
//    char read_path[40] = "../RealTrace/";
//    const char *name = "201911161100.txt";
//    strcat(write_path, "Result");
//    strcat(write_path, name);
//    strcat(read_path, "LoRaTrace");
//    strcat(read_path, name);

    char write_path[40] = "../SynthesisTrace/result_";
    char read_path[40] = "../SynthesisTrace/chip_seq_";
    char post_fix[10];
   snprintf( post_fix, 4, "%d", len);
    strcat(write_path, post_fix);
    strcat(read_path, post_fix);

    if (NULL == (write_fp = fopen(write_path, "w"))) {
        printf("new writefile error\n");
        exit(1);
    }

    struct ChirpDecoder *chirp_decoder = calloc(1, sizeof(struct ChirpDecoder));
    chirp_decoder->d_decim = d_decim;
    chirp_decoder->d_gradient = d_gradient;
    chirp_decoder->d_num_chirp_packet = d_num_chirp_packet;
    chirp_decoder->d_num_of_bins = d_num_of_bins;
    chirp_decoder->d_samples_per_symbol = d_samples_per_symbol ;
    chirp_decoder->d_decim_factor = d_decim_factor;
    chirp_decoder->d_sf = sf;
    chirp_decoder->d_cr = cr;
    chirp_decoder->d_len = len;
    chirp_decoder->d_dbg_chip = d_dbg_chirp;
    chirp_decoder->d_dbg_max = d_dbg_max;
    chirp_decoder->write_fp = write_fp;

    for (int r=0; r<1; r++){
        for (int i=0;i<100; i++){
            memset(samples, 0, sizeof(samples));
//            printf("\n--------------\nreading packet %d:\n", i);
            read_packet_trace(samples, read_path, i);
//            printf("\n--------------\ndecoding packet %d:\n", i);
            gettimeofday(&tv_start, NULL);
            chirp_decode(chirp_decoder, samples);
            gettimeofday(&tv_end, NULL);
            elapsed_us = (double) (tv_end.tv_usec - tv_start.tv_usec) / 1000000 + (double) (tv_end.tv_sec - tv_start.tv_sec);
//            printf("\n--------------Time elapsed %lf:\n", elapsed_us);
            elapsed_list[i] = elapsed_us;
            if ((int)(elapsed_us * 1e6)){
                elapsed_total += (int)(elapsed_us * 1e6);
                cnt_total ++;
            }
        }
        printf("%f ", (float)elapsed_total/cnt_total);
    }
    printf("\n");

    free(chirp_decoder);
    fclose(write_fp);

    return 0;
}

int main() {
    decode_trace();
    return 0;
}

