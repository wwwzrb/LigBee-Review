import numpy as np
from LoRa import LoRa
import matplotlib.pyplot as plt
import pandas as pd
from sklearn.preprocessing import minmax_scale
# from scipy.fftpack import fft
from matplotlib.font_manager import FontProperties
from cmath import phase

font = FontProperties()
font.set_size(20)
font.set_name('Times New Roman')
font.set_weight('regular')

# Generate FFT bins
sf = 7
lora_bw = 125e3
decim_factor = 1

lora = LoRa(sf)

lora_symbol = [124]
chirp_cnt = len(lora_symbol)
lora_samples = lora.mod(lora_symbol)
lora_phases = []
lora_freqs = []
lora_avg_freqs = []
lora_binarys = []
lora_avg_binarys = []
noise = np.random.normal(0, 0.15, (2, lora.mod_len))
# lora_samples = np.add(lora_samples, noise)

lora_result, lora_fft, lora_multi = lora.de_mod(lora_samples)

lora_mean = np.mean(lora_fft)
lora_var = np.var(lora_fft)

lora_fft = np.divide(np.subtract(lora_fft, lora_mean), lora_var)

for i in range(chirp_cnt):
    lora_phases.append([phase(sample) for sample in lora_samples[i]])

    lora_freqs.append(lora.get_freq(lora_samples, i))
    lora_freqs[i].append(0)  # append 0 as freqs are calculated from adjacent samples

    # index = (1 << sf) - lora_symbol[i] - 1
    # lora_freqs[i][index+1] = 0.0545
    # lora_freqs[i].append(lora_freqs[i][-1])
    # tmp = lora_freqs[i][index-1]
    # lora_freqs[i][index-1] = lora_freqs[i][index]
    # lora_freqs[i][index] = tmp

    lora.show_samples(lora_phases, i)
    lora.show_samples(lora_freqs, i)
    lora.show_samples(lora_fft, i)

for i in range(chirp_cnt):
    if decim_factor == 1:
        break

    lora_avg_freq = []
    for j in range((1 << sf) // decim_factor):
        lora_avg_freq.append((lora_freqs[i][decim_factor*j]+lora_freqs[i][decim_factor*j+1])/2)
    lora_avg_freqs.append(lora_avg_freq)

# plot phase shift + binary
# for i in range(chirp_cnt):
#     lora_binary = [1 if j > 0 else 0 for j in lora_freqs[i]]
#     lora_binarys.append(lora_binary)
#
#     lora_avg_binary = [1 if j > 0 else 0 for j in lora_avg_freqs[i]]
#     lora_avg_binarys.append(lora_avg_binary)
#
#     fig = plt.figure(figsize=(4, 2))
#     axes = fig.add_axes([0.1, 0.1, 0.8, 0.8])
#     x = [j for j in range(len(lora_freqs[i]))]
#     axes.plot(x, lora_freqs[i], '.', color='#000000', linewidth=0.5)
#     x_tick = (1 << (sf - 2))
#     axes.set_xlabel('Index', fontproperties=font)
#     axes.set_xticks([tmp for tmp in range(0, (1 << sf) + x_tick, x_tick)])
#     axes.set_xticklabels([tmp for tmp in range(0, (1 << sf) + x_tick, x_tick)], fontproperties=font)
#     axes.set_ylabel('Phase shift', fontproperties=font)
#     axes.set_ylim(-2.2, 2.2)
#     plt.savefig('freq_os_' + str(lora_symbol[i]) + '.svg', bbox_inches='tight', format='svg')
#
#     fig = plt.figure(figsize=(4, 2))
#     axes = fig.add_axes([0.1, 0.1, 0.8, 0.8])
#     x = [j for j in range(len(lora_binarys[i]))]
#     axes.plot(x, lora_binarys[i], '.', color='#000000', linewidth=0.5)
#     axes.set_xlabel('Index', fontproperties=font)
#     x_tick = (1 << (sf - 2))
#     axes.set_xticks([tmp for tmp in range(0, (1 << sf) + x_tick, x_tick)])
#     axes.set_xticklabels([tmp for tmp in range(0, (1 << sf) + x_tick, x_tick)], fontproperties=font)
#     axes.set_ylabel('Chip value', fontproperties=font)
#     axes.set_yticks([0, 1])
#     axes.set_yticklabels([0, 1])
#     axes.set_yticklabels(['0', '1'], fontproperties=font)
#     plt.savefig('binary_os_' + str(lora_symbol[i]) + '.svg', bbox_inches='tight', format='svg')
#
#
#     fig = plt.figure(figsize=(4, 2))
#     axes = fig.add_axes([0.1, 0.1, 0.8, 0.8])
#     x = [j for j in range(len(lora_avg_binarys[i]))]
#     axes.plot(x, lora_avg_binarys[i], '.', color='#000000', linewidth=0.5)
#     axes.set_xlabel('Index', fontproperties=font)
#     x_tick = (1 << (sf - 2)) // decim_factor
#     axes.set_xticks([tmp for tmp in range(0, (1 << sf) // decim_factor + x_tick, x_tick)])
#     axes.set_xticklabels([tmp for tmp in range(0, (1 << sf) // decim_factor + x_tick, x_tick)], fontproperties=font)
#     axes.set_ylabel('Chip value', fontproperties=font)
#     axes.set_yticks([0, 1])
#     axes.set_yticklabels([0, 1])
#     axes.set_yticklabels(['0', '1'], fontproperties=font)
#     plt.savefig('binary_avg_os_' + str(lora_symbol[i]) + '.svg', bbox_inches='tight', format='svg')