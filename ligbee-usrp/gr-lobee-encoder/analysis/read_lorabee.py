import scipy
import numpy as np
import matplotlib.pyplot as plt
from cmath import phase


def show(content):
    x = [i for i in range(len(content))]
    plt.plot(x, content, 'b.')
    plt.show()


def max_gradient_idx(content):
    max_gradient = 0.1
    max_index = 0
    for i in range(1, d_number_of_bins):
        gradient = content[i-1] - content[i]
        if gradient > max_gradient:
            max_gradient = gradient
            max_index = i+1
    return (d_number_of_bins - max_index) % d_number_of_bins


def max_correlation_idx(content, window, isheader):
    lsb_zero = np.uint8(0)
    lsb_one = np.uint8(1)
    zero_cnt = 0
    one_cnt = 0
    corr_zero = []
    corr_one = []

    for i in range(window + 1):
        for j in range(window):
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
    corr_zero_max = np.max(corr_zero)
    corr_one_max = np.max(corr_one)

    corr_zero_index = np.where(corr_zero == corr_zero_max)[0]
    corr_one_index = np.where(corr_one == corr_one_max)[0]

    # print("O: corr_zero_index: ", corr_zero_index, " corr_one_index: ", corr_one_index)

    # corr_zero_index = math.floor(np.average(corr_zero_index)+0.5)
    # corr_one_index = math.floor(np.average(corr_one_index)+0.5)

    if isheader:
        index_error = 0
    else:
        index_error = d_index_error

    decim_factor = int(window / d_half_number_of_bins)
    corr_zero_index = round(np.average(corr_zero_index))
    corr_one_index = round(np.average(corr_one_index))

    # according to correlation info
    # if corr_zero_max > corr_one_max:
    #     curr_index = int(max(round((window * 2 - corr_zero_index + index_error * decim_factor) / decim_factor), 0.0))
    #     flag = 0
    # else:
    #     curr_index = int(max(round((window - corr_one_index + index_error * decim_factor) / decim_factor), 0.0))
    #     flag = 1
    # curr_index = ((curr_index + d_number_of_bins) % d_number_of_bins)

    # according to correlation and frequent gradient info
    # corr_zero_index = corr_zero_index[0]
    # corr_one_index = corr_one_index[-1]
    curr_index = 0
    if corr_zero_max > corr_one_max:
        flag = 0
        for i in range(int(corr_zero_index), -1, -1):
            if content[i] == 1:
                curr_index = round(i/decim_factor) + 2
                break
    else:
        flag = 1
        for i in range(int(corr_one_index+corr_one_max), int(2*window)):
            if content[i] == 0:
                curr_index = round(i/decim_factor) + 1
                break
    curr_index = ((d_number_of_bins - curr_index) % d_number_of_bins)

    energy = corr_one_index / window
    print("C: flag=", flag, "; corr_zero_index: ", corr_zero_index, " corr_zero_max: ", corr_zero_max,
          "; corr_one_index: ", corr_one_index, " corr_one_max: ", corr_one_max,
          " curr_index: ", curr_index, "; index_error ", index_error)

    return curr_index


def get_error_idx(content, begin):
    error_cnt = []
    for i in range(len(content)):
        error_cnt.append(bin(content[i] ^ chirp_symbol[i + begin]).count('1'))
    return error_cnt


packet_cnt = 2
d_decim_factor = 2
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

gradient_result = []
corr_result = []
d_index_error = []

chirp_binary_res = []

for i in range(chirp_cnt):
    index = i
    begin_index = index * d_samples_per_symbol
    end_index = (index + 1) * d_samples_per_symbol
    decim_bgn_index = index * d_number_of_bins
    decim_end_index = (index + 1) * d_number_of_bins

    chirp = data[begin_index:end_index]
    chirp_phase = [phase(sample) for sample in chirp]
    chirp_ifreq = data_ifreq[begin_index:end_index]
    chirp_ifreq_avg = data_ifreq_avg[decim_bgn_index:decim_end_index]
    chirp_binary = data_binary[begin_index:end_index]

    gradient_result.append(scipy.uint8(max_gradient_idx(chirp_ifreq_avg)))

    avg_factor = d_decim_factor
    chirp_binary_avg = np.add.reduceat(chirp_binary, np.arange(0, len(chirp_binary), avg_factor))
    chirp_binary_avg = np.array([round(b/d_decim_factor) for b in chirp_binary_avg], np.uint8)

    if index in chirp_index:
        show_phase = chirp_phase
        show_ifreq = chirp_ifreq
        show_ifreq_avg = chirp_ifreq_avg
        show_binary = chirp_binary
        show_binary_avg = chirp_binary_avg
        show(chirp_phase)
        show(chirp_ifreq)
        show(chirp_ifreq_avg)
        show(chirp_binary)
        show(chirp_binary_avg)

    chirp_binary = chirp_binary_avg
    window = int(d_half_samples_per_symbol/avg_factor)
    if i < 8:
        corr_idx = max_correlation_idx(chirp_binary, window, True)
        d_index_error.append(4*round(corr_idx/4)-corr_idx)
    else:
        d_index_error = round(np.average(d_index_error))
        corr_idx = max_correlation_idx(chirp_binary, window, False)
    corr_result.append(scipy.uint8(corr_idx))


gradient_error = get_error_idx(gradient_result, 0)
corr_error = get_error_idx(corr_result, 0)

print("gradient_error: ", np.average(gradient_error), "; corr_error: ", np.average(corr_error))



