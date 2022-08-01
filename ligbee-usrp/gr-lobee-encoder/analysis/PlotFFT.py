import numpy as np
from LoRa import LoRa
import matplotlib.pyplot as plt
import pandas as pd
from sklearn.preprocessing import minmax_scale
# from scipy.fftpack import fft
from matplotlib.font_manager import FontProperties

font = FontProperties()
font.set_size(20)
font.set_name('Times New Roman')

# Generate FFT bins
sf = 6
lora_bw = 125e3

lora = LoRa(sf)

lora_symbol = [30, 48]
lora_samples = lora.mod(lora_symbol)
lora_freqs = []
lora_binarys = []
# noise = np.random.normal(0, 0.2, (2, lora.mod_len))
# lora_samples = np.add(lora_samples, noise)

lora_result, lora_fft, lora_multi = lora.de_mod(lora_samples)

lora_mean = np.mean(lora_fft)
lora_var = np.var(lora_fft)

lora_fft = np.divide(np.subtract(lora_fft, lora_mean), lora_var)

for i in range(2):
    lora_freqs.append(lora.get_freq(lora_samples, i))
    lora.show_samples(lora_freqs, i)
    lora.show_samples(lora_fft, i)

# plot fft
fig = plt.figure(figsize=(4, 2))
axes = fig.add_axes([0.1, 0.1, 0.8, 0.8])
x = [j for j in range(lora.mod_len)]
axes.plot(x, lora_fft[0], '-', color='#000000', linewidth=2)
axes.plot(x, lora_fft[1], '-', color='#C00000', linewidth=2)
axes.set_xlabel('FFT bin', fontproperties=font)
axes.set_xticklabels([tmp for tmp in range(-20, 140, 20)], fontproperties=font)
axes.set_ylabel('Amplitude', fontproperties=font)
axes.set_ylim(-0.1, 1.1)
axes.set_yticks([0, 1])
axes.set_yticklabels([0, 1])
axes.set_yticklabels(['0', '1'], fontproperties=font)
plt.savefig('fft'+str(lora_symbol[0])+'+'+str(lora_symbol[1])+'.svg', bbox_inches='tight', format='svg')

# plot phase shift + binary
for i in range(2):
    # index = (1 << sf) - lora_symbol[i] - 1
    #
    # tmp = lora_freqs[i][index]
    # lora_freqs[i][index] = lora_freqs[i][index + 1]
    # lora_freqs[i][index + 1] = tmp

    lora_binary = [1 if j > 0 else 0 for j in lora_freqs[i]]
    lora_binary.append(lora_binary[-1])
    lora_binarys.append(lora_binary)

    fig = plt.figure(figsize=(4, 2))
    axes = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    x = [j for j in range(len(lora_freqs[i]))]
    axes.plot(x, lora_freqs[i], '.', color='#000000', linewidth=0.5)
    axes.set_xlabel('Index', fontproperties=font)
    axes.set_xticks([tmp for tmp in range(0, (1 << sf) + 16, 16)])
    axes.set_xticklabels([tmp for tmp in range(0, (1 << sf) + 16, 16)], fontproperties=font)
    axes.set_ylabel('Phase shift', fontproperties=font)
    axes.set_yticks([-3, 0, 3])
    axes.set_yticklabels([-3, 0, 3])
    axes.set_yticklabels(['-3', '0', '3'], fontproperties=font)
    plt.savefig('freq_std_' + str(lora_symbol[i]) + '.svg', bbox_inches='tight', format='svg')

    fig = plt.figure(figsize=(4, 2))
    axes = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    x = [j for j in range(len(lora_binarys[i]))]
    axes.plot(x, lora_binarys[i], '.', color='#000000', linewidth=0.5)
    axes.set_xlabel('Index', fontproperties=font)
    axes.set_xticks([tmp for tmp in range(0, (1 << sf) + 16, 16)])
    axes.set_xticklabels([tmp for tmp in range(0, (1 << sf) + 16, 16)], fontproperties=font)
    axes.set_ylabel('Chip value', fontproperties=font)
    axes.set_yticks([0, 1])
    axes.set_yticklabels([0, 1])
    axes.set_yticklabels(['0', '1'], fontproperties=font)
    plt.savefig('binary_std_' + str(lora_symbol[i]) + '.svg', bbox_inches='tight', format='svg')