import scipy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

font = FontProperties()
font.set_size(20)
font.set_name('Times New Roman')
font.set_weight('regular')

def show(content, name):
    fig = plt.figure(figsize=(4, 4))
    axes = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    x = [i for i in range(len(content))]
    axes.plot(x, content, 'k-')
    plt.savefig(name, bbox_inches='tight', format='svg')


def show_chirp(content, name):
    fig = plt.figure(figsize=(4, 2))
    axes = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    x = [i for i in range(len(content))]
    axes.plot(x, content, '-', color='#000000', linewidth=0.5)
    axes.set_xlabel('Index', fontproperties=font)
    axes.set_ylabel('Amplitude', fontproperties=font)
    plt.savefig(name, bbox_inches='tight', format='svg')


def show_freq(content, name):
    fig = plt.figure(figsize=(4, 2))
    axes = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    x = [i for i in range(len(content))]
    axes.plot(x, content, '.', color='#000000', linewidth=0.5)
    axes.set_xlabel('Index', fontproperties=font)
    x_tick = len(content) // 4
    axes.set_xticks([tmp for tmp in range(0, len(content) + x_tick, x_tick)])
    axes.set_xticklabels([tmp for tmp in range(0, len(content) + x_tick, x_tick)], fontproperties=font)
    axes.set_ylabel('Phase shift', fontproperties=font)
    plt.savefig(name, bbox_inches='tight', format='svg')


def show_binary(content, name):
    fig = plt.figure(figsize=(4, 2))
    axes = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    x = [i for i in range(len(content))]
    axes.plot(x, content, '.', color='#000000', linewidth=0.5)
    axes.set_xlabel('Index', fontproperties=font)
    x_tick = len(content) // 4
    axes.set_xticks([tmp for tmp in range(0, len(content) + x_tick, x_tick)])
    axes.set_xticklabels([tmp for tmp in range(0, len(content) + x_tick, x_tick)], fontproperties=font)
    axes.set_ylabel('Chip value', fontproperties=font)
    plt.savefig(name, bbox_inches='tight', format='svg')


def show_corr(content, name):
    global d_sf
    fig = plt.figure(figsize=(4, 2))
    axes = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    x = [i for i in range(len(content))]
    axes.plot(x, content, '.', color='#000000', linewidth=0.5)
    axes.set_xlabel('Index', fontproperties=font)
    axes.set_ylabel('Correlation', fontproperties=font)
    axes.set_yticks([tmp for tmp in range(0, (1 << d_sf) + 32, 32)])
    axes.set_yticklabels([tmp for tmp in range(0, (1 << d_sf) + 32, 32)], fontproperties=font)
    plt.savefig(name, bbox_inches='tight', format='svg')

def max_correlation_idx(content, window, isheader):
    temp_zeros = np.zeros(window, dtype=np.uint8)
    temp_ones = np.ones(window, dtype=np.uint8)
    temp = np.concatenate((temp_zeros, temp_ones))
    lsb_zero = np.uint8(0)
    lsb_one = np.uint8(1)
    zero_cnt = 0
    one_cnt = 0
    sum_cnt = 0
    corr_zero = []
    corr_one = []
    corr_sum = []

    for i in range(2*window):
        for j in range(2*window):
            idx = (i+j) % (2*window)
            if content[idx] == temp[j]:
                sum_cnt += 1

        corr_sum.append(sum_cnt)
        sum_cnt = 0

        for j in range(window):
            if i+j >= 2*window:
                break
            if content[i + j] == lsb_zero:
                zero_cnt += 1
            else:
                one_cnt += 1

        corr_one.append(one_cnt)
        corr_zero.append(zero_cnt)

        zero_cnt = 0
        one_cnt = 0

    corr_zero = np.array(corr_zero)
    corr_one = np.array(corr_one)
    corr_sum = np.array(corr_sum)

    # if np.max(corr_zero) > np.max(corr_one):
    #     zero_color = 'r'
    #     one_color = 'k'
    #     zero_marker = '.'
    #     one_marker = '.'
    # else:
    #     zero_color = 'k'
    #     one_color = 'r'
    #     zero_marker = '.'
    #     one_marker = '.'
    # fig = plt.figure(figsize=(4, 2))
    # axes = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    # x = [i for i in range(len(content))]
    # axes.plot(x, corr_zero, 'k.--')
    # axes.plot(x, corr_one, 'r.--')
    # plt.savefig('corr.png', bbox_inches='tight')

    return corr_sum


packet_cnt = 2
d_decim_factor = 8
path_prefix = "../oversample/" + str(d_decim_factor) + "/"
data = scipy.fromfile(path_prefix+"data_"+str(packet_cnt), scipy.complex64)
data_ifreq = scipy.fromfile(path_prefix+"data_ifreq_"+str(packet_cnt), scipy.float32)
data_ifreq_avg = scipy.fromfile(path_prefix+"data_ifreq_avg_"+str(packet_cnt), scipy.float32)
data_binary = scipy.fromfile(path_prefix+"data_binary_"+str(packet_cnt), scipy.uint8)

d_sf = 7
d_number_of_bins = (1 << d_sf)
d_samples_per_symbol = d_number_of_bins*d_decim_factor
d_half_number_of_bins = int(d_number_of_bins/2)
d_half_samples_per_symbol = int(d_samples_per_symbol/2)
chirp_cnt = int(len(data)/d_samples_per_symbol)
chirp_symbol = np.array([124, 60, 96, 112, 24, 108, 48, 100,
                         1, 18, 67, 90, 66, 15, 31, 38,
                         0, 42, 117, 34, 7, 75, 114, 57], dtype=scipy.uint8)

chirp_index = [0, 3, 4]

chirps = []
chirp_ifreqs = []
chirp_ifreq_avgs = []
chirp_binarys = []
chirp_binary_avgs = []
chirp_corrs = []

for i in range(len(chirp_index)):
    index = chirp_index[i]
    begin_index = index * d_samples_per_symbol
    end_index = (index + 1) * d_samples_per_symbol
    decim_bgn_index = index * d_number_of_bins
    decim_end_index = (index + 1) * d_number_of_bins

    chirp = data[begin_index:end_index]
    chirp_ifreq = data_ifreq[begin_index:end_index]
    chirp_ifreq_avg = data_ifreq_avg[decim_bgn_index:decim_end_index]
    chirp_binary = data_binary[begin_index:end_index]

    avg_factor = 8
    chirp_binary_avg = np.add.reduceat(chirp_binary, np.arange(0, len(chirp_binary), avg_factor))
    chirp_binary_avg = np.array([round(b / d_decim_factor) for b in chirp_binary_avg], np.uint8)

    window = int(d_half_samples_per_symbol / avg_factor)

    chirp_corr = max_correlation_idx(chirp_binary_avg, window, True)

    chirps.append(chirp)
    chirp_ifreqs.append(chirp_ifreq)
    chirp_ifreq_avgs.append(chirp_ifreq_avg)
    chirp_binarys.append(chirp_binary)
    chirp_corrs.append(chirp_corr)

    show_chirp(chirp, 'chirp_' + str(chirp_symbol[index]) + '.svg')
    show_freq(chirp_ifreq, 'freq_' + str(chirp_symbol[index]) + '.svg')
    show_freq(chirp_ifreq_avg, 'freq_avg_' + str(chirp_symbol[index]) + '.svg')
    show_binary(chirp_binary, 'binary_' + str(chirp_symbol[index]) + '.svg')
    show_binary(chirp_binary_avg, 'binary_avg_' + str(chirp_symbol[index]) + '.svg')
    show_corr(chirp_corr, 'chirp_corr_' + str(chirp_symbol[index]) + '.svg')




