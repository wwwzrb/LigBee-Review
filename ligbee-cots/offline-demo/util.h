#ifndef PROJC_UTIL_H
#define PROJC_UTIL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ----------------------------------------------
// utils func
void print_chirp(unsigned char chirp[], int size) {
    printf("\n");
    for (int i = 0; i < size; i++) {
        printf("%d ", chirp[i]);
    }
    printf("\n");
}

int count_one_num(const unsigned char val[], int size) {
    /*
     * params:
     * val: variable denoted by byte
     * size: number of bytes in val
     *
     * return:
     * The number of bit 1s in val.
     */
    int num = 0;

    for (int i = 0; i < size; i++) {
        unsigned char temp = val[i];
        while (temp) {
            temp &= (temp - 1);
            ++num;
        }
    }
    return num;
}


int is_kth_bit_set_for_bitarray(const unsigned char src[], int k) {
    /*
     * Check whether the k-th bit is set
     * */

    int byte_num = k / 8;
    int bit_num = k % 8;
    unsigned char opnum = (unsigned char) (1 << (7 - bit_num));

    // Big endian at the left, however, count from left to the right.
    if (src[byte_num] & opnum)
        return 1;
    else
        return 0;
}


void set_kth_bit_for_bitarray(unsigned char src[], int k) {
    /*
     * set the k-th bit to 1
     */
    int byte_num = k / 8;
    // Big endian at the left, however, count from left to the right.
    int bit_num = k % 8;

    src[byte_num] |= (unsigned char)(1 << (7 - bit_num));
}


// -------------
// decoding a chirp
void get_max_correlation_idx(unsigned char chirp_bitarray[], int chirp_bitnum, int *corr_idx_p, int *corr_max_p, int gradient) {
    /*
     * params:
     *  chirp bitarray:  chirp数组
     *  chirp bitnum:    chirp 数组 bit 数目
     *  gradient:        是否使用gradient方式解包
     *
     * return:
     *  corr idx:        解包结果
     *  corr max:        统计的最大的1的数目
     */
    *corr_idx_p = 0;
    *corr_max_p = 0;

    int corr_one_max = 0, corr_zero_max = 0;
    int window_halflen = chirp_bitnum / 2;

    // 开始统计1, 0
    int one_cnt[window_halflen + 1];
    memset(one_cnt, 0, sizeof(one_cnt));
    one_cnt[0] = count_one_num(chirp_bitarray, window_halflen / 8);
    int one_whole_cnt[chirp_bitnum];
    memset(one_whole_cnt, 0, sizeof(one_whole_cnt));
    one_whole_cnt[window_halflen - 1] = one_cnt[0];

    int left_set = 0, right_set = 0;

    // 原来那个python里面为什么是(0, window+1)? 64- 127 也是64个额。。
    for (int i = 0; i < window_halflen; ++i) {

        left_set = is_kth_bit_set_for_bitarray(chirp_bitarray, i);
        right_set = is_kth_bit_set_for_bitarray(chirp_bitarray, i + window_halflen);

        // 窗口向右移动，左侧减一，右侧加1
        one_cnt[i + 1] = one_cnt[i] + (right_set - left_set);
        one_whole_cnt[i + window_halflen] = one_whole_cnt[i - 1 + window_halflen] + right_set;

        if (one_cnt[i + 1] > corr_one_max) {
            corr_one_max = one_cnt[i + 1];
        }
        if (corr_zero_max < window_halflen - one_cnt[i + 1]) {
            corr_zero_max = window_halflen - one_cnt[i + 1];
        }
    }

    // 处理特殊情况
    if (one_whole_cnt[chirp_bitnum - 1] < window_halflen * 1 / 5) {
        *corr_idx_p = -1;
        *corr_max_p = 128;
        return;
    }
    // 若有多个最大值则进行平均
    int idxes_sum_max = 0, idxes_cnt_max = 0, idxes_sum_min = 0, idxes_cnt_min = 0;
    for (int i = 0; i <= window_halflen; i++) {
        if (corr_one_max == one_cnt[i]) {
            idxes_sum_max += i;
            idxes_cnt_max++;
        }

        if (corr_zero_max == window_halflen - one_cnt[i]) {
            idxes_sum_min += i;
            idxes_cnt_min++;
        }
    }
    int corr_one_idx = idxes_sum_max / idxes_cnt_max;
    int corr_zero_idx = idxes_sum_min / idxes_cnt_min;

    int set=0;
    int pre_post_zero, pre_post_one;
    pre_post_zero=pre_post_one=0;
    int pre_post_len = 4;
    for (int i=0; i <pre_post_len; i++){
        set = is_kth_bit_set_for_bitarray(chirp_bitarray, i);
        if (set){
            pre_post_one += 1;
        }
        else{
            pre_post_zero += 1;
        }
    }

    for (int i=chirp_bitnum-pre_post_len; i <chirp_bitnum; i++){
        set = is_kth_bit_set_for_bitarray(chirp_bitarray, i);
        if (set){
            pre_post_one += 1;
        }
        else{
            pre_post_zero += 1;
        }
    }

    if (gradient) {

        // 用梯度信息
        if (pre_post_zero >= pre_post_one) {
            /*
            int idx = corr_one_idx + corr_one_max;
            while (idx < 2 * window_halflen && !is_kth_bit_set_for_bitarray(chirp_bitarray, idx)) idx++;

            if (idx < 2 * window_halflen) *corr_idx_p = idx + 1;
            *corr_max_p = corr_zero_max;
             */

            for (int idx = corr_one_idx + corr_one_max; idx < 2 * window_halflen; idx++) {
                if (!is_kth_bit_set_for_bitarray(chirp_bitarray, idx)) {
                    *corr_idx_p = idx + 1;
                    break;
                }
            }
            *corr_max_p = corr_one_max;

        } else {
            /*
            int idx = corr_zero_idx;
            while (idx > 0 && is_kth_bit_set_for_bitarray(chirp_bitarray, idx)) idx--;

            if (idx > 0) *corr_idx_p = idx + 2;
            *corr_max_p = corr_zero_max;
             * */
            for (int idx = corr_zero_idx; idx >= 0; idx--) {
                if (is_kth_bit_set_for_bitarray(chirp_bitarray, idx)) {
                    *corr_idx_p = idx + 2;
                    break;
                }
            }

            *corr_max_p = corr_zero_max;

        }

        *corr_idx_p = (chirp_bitnum - *corr_idx_p) % chirp_bitnum; // chirp_bitnum equals to d_num_of_bins

    } else {
        // one max > zero max
        if (corr_one_max >= corr_zero_max) {
            float temp = (float) (window_halflen - corr_one_idx);
            *corr_idx_p = temp > 0 ? (int) temp : 0;
            *corr_max_p = corr_one_max;
        } else {
            float temp = (float) (window_halflen * 2 - corr_zero_idx);
            *corr_idx_p = temp > 0 ? (int) temp : 0;
            *corr_max_p = corr_zero_max;
        }

        *corr_idx_p = (*corr_idx_p + chirp_bitnum) % chirp_bitnum; // chirp_bitnum equals to d_samples_per_symbol
    }

}

#endif //PROJC_UTIL_H
