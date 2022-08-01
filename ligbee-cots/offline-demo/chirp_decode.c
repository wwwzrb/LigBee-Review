#include "queue.h"
#include "util.h"
#include "chirp_decode.h"
#include "reverse_encode.h"

/*
 * params
 * chirp_decoder: ChirpDecoder  Struct parameters
 *  samples:    bitarray
 */
void chirp_decode(struct ChirpDecoder *chirp_decoder, const unsigned char *samples) {
    /* Initialize chirp_decoder parameters */
    int decim = chirp_decoder->d_decim;
    int gradient = chirp_decoder->d_gradient;
    int num_chirp_packet = chirp_decoder->d_num_chirp_packet;
    int num_of_bins= chirp_decoder->d_num_of_bins;
    int samples_per_symbol = chirp_decoder->d_samples_per_symbol;
    int decim_factor = chirp_decoder->d_decim_factor;
    int sf = chirp_decoder->d_sf;
    int cr = chirp_decoder->d_cr;
    int len = chirp_decoder->d_len;
    int dbg_chirp = chirp_decoder->d_dbg_chip;
    int dbg_max = chirp_decoder->d_dbg_max;

    FILE *write_fp = chirp_decoder->write_fp;

    // intermediate chirp_decoder result
    int corr_idx = 0, corr_max = 0;
    // save chirp_decoder result
    struct Queue *indexes = createQueue((unsigned) num_chirp_packet),
            *maxes = createQueue((unsigned) num_chirp_packet);

    // control variables
    int   shift_idx = 0,      // preamble offset
            curr_chirp_start,   // start of a chirp bit
            start_bit,
            end_bit,
            one_vote_count,
            zero_vote_count;

    // store raw chips:
    // decode after oversampling recovery, curr_chirp_decim
    // decode directly, curr_chirp
    unsigned char curr_chirp_decim[num_of_bins];
    unsigned char curr_chirp[samples_per_symbol / decim_factor];
    memset(curr_chirp_decim, 0, sizeof(curr_chirp_decim));
    memset(curr_chirp, 0, sizeof(curr_chirp));

//     //  for RealTrace
//    //   skip fine sync, use statistics of shift_idx
//    int sync_len = 32,          // zigbee sync bit length
//    pre_len = 0;        // lora preamble length
//    shift_idx = (int) ((pre_len - 1 + 4.25) * samples_per_symbol - sync_len);

    // only for SynthesisTrace
    shift_idx = 0;
    for (int i = 0; i < num_chirp_packet; i++) {
        if (decim) {
            // curr_chirp_target
            curr_chirp_start = shift_idx + i * samples_per_symbol;
            memset(curr_chirp_decim, 0, sizeof(curr_chirp_decim));

            // oversampling recovery
            for (int j = 0; j < num_of_bins; j++) {
                start_bit = curr_chirp_start + j * decim_factor;
                end_bit = curr_chirp_start + (j + 1) * decim_factor;

                one_vote_count = 0;

                for (int k = start_bit; k < end_bit; k++) {
                    if (is_kth_bit_set_for_bitarray(samples, k)) {
                        one_vote_count++;
                    }
                }
                zero_vote_count = end_bit - start_bit - one_vote_count;

                // biased estimation
                if (one_vote_count >= zero_vote_count) {
                    set_kth_bit_for_bitarray(curr_chirp_decim, j);
                }
            }

            // show chirp
            if (dbg_chirp){
                printf("\nchirp %d bitarray:\n", i);
                print_chirp(curr_chirp_decim, num_of_bins / 8);
                printf("\n");
            }

            get_max_correlation_idx(curr_chirp_decim, num_of_bins, &corr_idx, &corr_max, gradient);
        } else {
            // curr_chirp
            memcpy(curr_chirp, samples + (shift_idx + i * samples_per_symbol) / 8,
                   sizeof(curr_chirp));

            // show chirp
            if (dbg_chirp){
                printf("\nchirp %d bitarray:\n", i);
                print_chirp(curr_chirp, samples_per_symbol / 8);
                printf("\n");
            }

            get_max_correlation_idx(curr_chirp, samples_per_symbol, &corr_idx, &corr_max, gradient);
            corr_idx = corr_idx / decim_factor;
        }
        enqueue(indexes, corr_idx);
        enqueue(maxes, corr_max);
    }

    /* reverse encoding */
    struct ReverseEncoder *reverse_encoder =  calloc(1, sizeof(struct ReverseEncoder));
    init_encode(reverse_encoder, indexes->array, sf, cr, len);
    reverse_encode(reverse_encoder);
//    printf("--------------Decoding Results for Bytes: \n");
    save_to_file(reverse_encoder, write_fp);
    release(reverse_encoder);

//    printf("--------------Decoding Results for Chirps: \n");
//    save_queue_to_file(indexes, write_fp);
//    print_queue(indexes);
    if (dbg_max){
        printf("--------------Zero/One Count Max: \n");
        print_queue(maxes);
    }
    release_queue(indexes);
}

