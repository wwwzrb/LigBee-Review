import matplotlib.pyplot as plt
import scipy
import numpy as np
from math import sin, pi,cos, atan2
import time
from cmath import exp
import cmath

#output_file = "./3/packet"
gt_file = "./ground_truth_bit"
CHIP_PER_SYMBOL = 32
Tc = 0.8e-5
Fc = 1/0.8e-5
samples_per_chip, chips_per_symbol, symbol_length = 32,32,68


def get_gt_chips():
    return np.load('chip_stream.npy')


def get_correlation_samples(length, samples_per_chip):
    res = []
    chips = get_gt_chips()
    if length == -1:
        length = len(chips)
    for i in range(length):
        sig = chips[i]
        for j in range(samples_per_chip):
            res.append(sig*sin(pi*j/32))
    return np.array(res)


def get_input(input):
    return scipy.fromfile(input, dtype=scipy.complex64)


def calculate_corrcoef(a, b):
    return np.corrcoef(a,b)[0][1]


def get_sin_samples():
    #32 is the samples per chip
    res = []
    for i in range(32):
        res.append(sin(pi * i / 32))
    return res


def get_chip_from_eof(eof):
    if eof > 0.01:
        return 1
    elif eof < -0.01:
        return -1
    else:
        return 0


# slide from $header-interval to $header+interval
def zigzag_correlation(samples, gt_samples, header,interval):
    N = 2*interval

    gt_samples_conj = [x.conjugate() for x in gt_samples]
    samples_per_packet = samples_per_chip * chips_per_symbol * symbol_length
    # sliding window offset
    sumlist = []
    for i in range(N):
        sum = 0
        for j in range(len(gt_samples)):
            sum += gt_samples_conj[j] * samples[i+j]
        sumlist.append(sum)
    sum_samples = sumlist[:len(gt_samples)]
    idx = np.where(sum_samples == np.max(sum_samples))[0]
    idx = idx - interval + header
    # print(np.where(sum_samples == np.max(sum_samples)))
    t = [i for i in range(len(gt_samples))]
    print("Packet idx is %d\n"%(idx))
    return idx
    # plt.plot(t, sum_samples)
    # plt.show()


def split_packet_corse(input_stream, begin_idx):
    idx_list = []
    b_time = time.time()
    last_idx = 0
    ic = 0
    sin_samples = get_sin_samples()
    preamble_chip_stream = get_gt_chips()[:320]
    preamble_chip_stream = [int(x) for x in preamble_chip_stream]

    samples_per_chip = 32
    preamble_chip_num = 32*10
    threshold = 10*10
    #chip loop
    i, ct = begin_idx, 0
    flag = 0
    while i < len(input_stream)-32*2176:
        error_chip_num = 0
        for j in range(preamble_chip_num):
            slice = input_stream[i+j*samples_per_chip:i+(j+1)*samples_per_chip]
            eof = calculate_corrcoef(slice,sin_samples)
            if abs(eof) < 0.5:
                error_chip_num += 1
        if error_chip_num < threshold:
            idx_list.append(i)
            last_idx = i
            i += 32*32*68 #samples_per_chip * chip_per_symbol* symbol_num = samples_per_packet
            flag = 0
            print("The %dth, error chip number is %.d.\n" % (i,error_chip_num))
        else:
            i += 1
            flag += 1
            if flag >= 10:
                i = i-flag + 32*32*68
                flag = 0
        if i / 100000 > ct:
            e_time = time.time()
            runtime = e_time-b_time
            print("The %dth 100k iterations, %d in list, the last is %d, time consumption=%.3fs.\n" % (ct, len(idx_list),last_idx, runtime))
            ct += 1

    f = open(dir+'packet_idx','w')
    for idx in idx_list:
        f.write(str(idx)+'\n')
    f.close()
    return idx_list


def split_packet_refinement(input_stream,packet_list,gt_preamble_samples):
    samples_per_packet = samples_per_chip * chips_per_symbol * symbol_length
    new_packet_list = []
    #for each packet
    for idx in packet_list:
        #for each possible header
        maxeof, maxidx = 0, 0
        for i in range(idx-10,idx+10):
            sumeof = 0
            for j in range(0,chips_per_symbol*8, 32):
                stardard = gt_preamble_samples[j:j+32]
                slice = input_stream[i+j:i+j+32]
                eof = np.corrcoef(slice,stardard)[1][0]
                sumeof += abs(eof)
            if sumeof > maxeof:
                maxidx = i
                maxeof = sumeof
        new_packet_list.append(maxidx)
        #print("Packet idx is %d\n"%(maxidx))
    f = open(dir+'packet_idx_refine','w')
    for ele in new_packet_list:
        f.write(str(ele)+'\n')
    f.close()
    return new_packet_list


def load_packet_idx(file):
    f = open(file,'r')
    packet_idx_list = list(f.readlines())
    packet_idx_list = [int(idx) for idx in packet_idx_list]
    return packet_idx_list


def load_freq(file):
    f = open(file,'r')
    freq = list(f.readlines())
    freq = [float(idx) for idx in freq]
    return freq

def get_packet_samples(primitive_data, packet_idx, sample_length_per_packet):
    res = []
    for idx in packet_idx:
        res.append(primitive_data[idx:idx+sample_length_per_packet])
    return np.array(res)


def calculate_freq_offset(preamble_samples, gt_preamble_samples):
    samples_conj, gt_samples_conj = np.copy(preamble_samples), np.copy(gt_preamble_samples)
    for i in range(len(samples_conj)):
        samples_conj[i] = samples_conj[i].conjugate()
        gt_samples_conj[i] = gt_samples_conj[i].conjugate()

    offset = 1024
    #delayed samples are conjugate
    samples_nondelay, samples_delay_conj = preamble_samples[offset:],samples_conj[:-offset]
    #non-delayed gt samples are conjugate
    gt_samples_conj, gt_samples_delay = gt_samples_conj[offset:], gt_preamble_samples[:-offset]
    res = np.zeros(len(preamble_samples)-1,dtype=complex)
    angle =  []
    for i in range(len(preamble_samples)-offset):
        tmp = samples_nondelay[i] * samples_delay_conj[i] * gt_samples_conj[i] * gt_samples_delay[i]
        res[i] = tmp
        if not (abs(tmp.real) < 1e-3 and abs(tmp.imag) < 1e-3):
            angle.append(atan2(tmp.imag,tmp.real))
    # t = [i for i in range(len(angle))]
    # plt.plot(t, angle,'.')
    # plt.show()
    sum,ct  = 0, 0
    for i in range(128,len(angle)):
        #if abs(angle[i]-pi/2) > 1e-3 and abs(angle[i]+pi/2) > 1e-3:
        sum += angle[i]
        ct += 1
    chip_offset = offset//32
    df = sum / ct/2/pi*Fc/chip_offset

    print(df)
    return df

def compensate_phase_offset(samples,gt_samples):
    # compensate in sample level:
    df = calculate_freq_offset(samples[0:8192], gt_samples[0:8192])
    com_samples = np.zeros(len(samples),dtype=complex)
    Tc = 8e-6
    #print(x_1,y_1,x_2,y_2)
    for i in range(0,len(samples),32):
        for j in range(32):
            #compensate for each chip

            com_samples[i+j] = samples[i+j]*exp(-1j*2*cmath.pi*df*(i//32)*Tc)
            #compensate for each sample
            #com_samples[i + j] = samples[i + j] * exp(-1j * 2 * cmath.pi * df / 32 * (i+j)  * Tc)
    # angle = angle[:1024]
    # t = [i for i in range(len(angle))]
    # plt.plot(t, angle)
    # plt.show()
    return com_samples, df

def test_figure(input_stream):
    begin, slice = 508507, 32 * 32
    begin = begin
    interval = 32 * 32
    data = input_stream[begin:begin + slice]
    data2 = input_stream[begin + interval:begin + slice + interval]
    data3 = input_stream[begin + 2 * interval:begin + slice + 2 * interval]
    data4 = input_stream[begin + 3 * interval:begin + slice + 3 * interval]
    data5 = input_stream[begin + 4 * interval:begin + slice + 4 * interval]
    data6 = input_stream[begin + 5 * interval:begin + slice + 5 * interval]

    dT = 1 / 4e6
    t = [i * dT for i in range(len(data))]
    plt.subplot(6, 1, 1)
    plt.plot(t, data)

    plt.subplot(6, 1, 2)
    plt.plot(t, data2)

    plt.subplot(6, 1, 3)
    plt.plot(t, data3)

    plt.subplot(6, 1, 4)
    plt.plot(t, data4)

    plt.subplot(6, 1, 5)
    plt.plot(t, data5)

    plt.subplot(6, 1, 6)
    plt.plot(t, data6)
    plt.show()



def run_preprocess(input_stream, packet_idx_list=[], is_split_packet=1, split_ptr = 0, debug = 0, debug_idx = 0):
    if is_split_packet:
        idx_list = split_packet_corse(input_stream,split_ptr)
    else:
        gt_chips = get_gt_chips()
        gt_samples = list(get_correlation_samples(2176, 32))

        #packet_idx_list = load_packet_idx(packet_idx_file)
        sample_length_per_packet = 32 * len(gt_chips)
        #1-d:packek_idx 2d: packet length
        packet_samples = get_packet_samples(input_stream, packet_idx_list , sample_length_per_packet)
        packet_samples_com = []
        df_list = []
        #df = -142
        # compensate for each packet
        if debug == 1:
            packet = packet_samples[debug_idx]
            #test_fig_multiplot(packet, 32)
            packet_com, df = compensate_phase_offset(packet, gt_samples)
            packet_samples_com.append(packet_com)
            test_fig_multiplot(packet_com,56)
            print("Packet %d Finished!\n" % (debug_idx))
        else:
            for i in range(len(packet_samples)):
                packet = packet_samples[i]
                packet_com, df = compensate_phase_offset(packet, gt_samples)
                packet_samples_com.append(packet_com)
                df_list.append(df)
                print("Packet %d Finished!\n" % (i))
            f = open(dir + 'compensated_freq', 'w')
            for i in range(len(df_list)):
                f.writelines(str(df_list[i])+'\n')
            f.close()
        #packet_samples_com = np.array(packet_samples_com)
        return packet_samples_com

        #test_fig_multiplot()

def test_fig_multiplot(packet1_com,chip_header):

    #packet1_com = packet1
    slice,k = 1024,chip_header
    data = packet1_com[slice*k:slice*(k+1)]
    data2 = packet1_com[slice*(k+1):slice*(k+2)]
    data3 = packet1_com[slice*(k+2):slice*(k+3)]
    data4 = packet1_com[slice*(k+3):slice*(k+4)]
    data5 = packet1_com[slice*(k+4):slice*(k+5)]
    data6 = packet1_com[slice*(k+5):slice*(k+6)]
    data7 = packet1_com[slice * (k+6):slice * (k+7)]
    data8 = packet1_com[slice * (k+7):slice * (k+8)]
    dT = 1/4e6
    t = [dT * i for i in range(len(data))]
    subnum = 8
    plt.subplot(subnum,1,1)
    plt.plot(t,data)
    plt.subplot(subnum,1,2)
    plt.plot(t,data2)
    plt.subplot(subnum,1,3)
    plt.plot(t,data3)
    plt.subplot(subnum,1,4)
    plt.plot(t,data4)
    plt.subplot(subnum,1,5)
    plt.plot(t,data5)
    plt.subplot(subnum,1,6)
    plt.plot(t,data6)
    plt.subplot(subnum,1,7)
    plt.plot(t,data7)
    plt.subplot(subnum,1,8)
    plt.plot(t,data8)
    plt.show()

#detect the first two chips
def detect_signal(sample_stream):
    samples_per_chip, chip_length = 32,2
    base = get_sin_samples() * 2
    for i in range(len(sample_stream)):
        tmpeof = calculate_corrcoef(sample_stream[i:i+ samples_per_chip* chip_length], base)
        if abs(tmpeof) > 0.6:
            return i

def autorun_mulple_freq_offset_removal():
    dir = 'F:/data/zigbee/'
    i = 1
    non_file = [1,2, 7, 8]
    while i <= 22:
        if i not in non_file:
            realdir = dir + str(i)+'/'
            input_file = realdir + "sensor2"
            packet_idx_file = realdir + "packet_idx"
            input_stream = get_input(input_file)
            packet_idx_list = load_packet_idx(packet_idx_file)
            packet_samples_com = run_preprocess(input_stream = input_stream, packet_idx_list=packet_idx_list, is_split_packet=0)

            #save
            output = realdir + 'packet_samples.npy'
            np.save(output,packet_samples_com)
            print("%d Finished!\n"%(i))
        i += 1

def run_find_packet(input_stream):
    # i = detect_signal(input_stream)
    # print(i)
    #run_preprocess(input_stream = input_stream, is_split_packet=1, split_ptr=22852)

    #test_fig_multiplot(input_stream[22852:],0)
    #
    packet_list = load_packet_idx(dir+'packet_idx')
    idx = packet_list[2785]
    test_fig_multiplot(input_stream[idx:],0)

def detect_packet_refine(input_stream):
    gt_preamble_samples = np.load('sample_stream.npy')[:samples_per_chip * chips_per_symbol * 8]
    idx_list = load_packet_idx(dir + 'packet_idx')
    split_packet_refinement(input_stream, idx_list, gt_preamble_samples)
# input_file = dir+"sensor2"
# packet_idx_file = dir + "packet_idx"
# input_stream = get_input()
# gt_chips = list(get_gt_chips())
# gt_sample_stream = list(get_correlation_samples(32*10,32))

#autorun_mulple_freq_offset_removal()

#observe signal samples after frequency removal
def observe_packet(input_stream):
    #dir ='F:/data/zigbee/10/'
    '''test'''
    # packets = np.load(dir+'packet_samples.npy')
    # packet = packets[2]
    # test_fig_multiplot(packet,56)
    # input_stream = get_input(dir+'sensor2')
    # packet_list = load_packet_idx(dir+'packet_idx')
    # packet_com = run_preprocess(input_stream=input_stream, packet_idx_list=packet_list, is_split_packet=0, debug=1,
    #                             debug_idx=24)
    '''correction'''
    #input_stream = get_input(dir+'sensor2')
    packet_list = load_packet_idx(dir+'packet_idx_refine')
    packet_com = run_preprocess(input_stream = input_stream, packet_idx_list=packet_list,is_split_packet=0, debug=0, debug_idx=432)
    slice = 1000

    np.save(dir+ 'packet_samples.npy',packet_com)
    # np.save('packet_samples.npy',np.array(packet_com[:1000]))
    # np.save('F:/data/zigbee/24/packet_samples.npy',np.array(packet_com[1000:2000]))
    # np.save('F:/data/zigbee/25/packet_samples.npy',np.array(packet_com[2000:3000]))
    # np.save('F:/data/zigbee/26/packet_samples.npy',np.array(packet_com[3000:]))

def main_multiple():
    global dir
    # operate with multiple files
    tmpdir = dir
    for file in file_idx_list:
        dir = tmpdir + str(file) + '/'
        input_stream = get_input(dir + 'sensor2')
        '''step 1'''
        # run_find_packet(input_stream)
        '''step 2'''
        detect_packet_refine(input_stream)
        '''step 3'''
        observe_packet(input_stream)

def main_single():
    global dir
    # operate with single file
    file = '23/'
    dir = dir + file
    input_stream = get_input(dir + 'sensor2')
    run_find_packet(input_stream)

def get_uncompensated_packets():
    for file in file_idx_list:
        realdir = dir + str(file) + '/'
        packet_list = load_packet_idx(realdir + 'packet_idx_refine')
        input_stream = get_input(realdir+'sensor2')
        packets = []
        for idx in packet_list:
            packets.append(input_stream[idx:idx+32*32*68])
        # packets = np.array(packets)
        np.save(realdir+'packets_uncompensated.npy',packets)
        print(str(file) + " Finished\n")

def detect_QPSK_preamble(input_stream):
    for i in range(20000,100000):
        if input_stream[i] > 0.2:
            return i




file_idx_list = [3,4,9,10,11,12,13,14,15,17,18]
file_idx_list = [4]

dir ='./'
input_stream  = get_input(dir+str(4)+'/sensor2')
#header = detect_QPSK_preamble(input_stream)
#test_fig_multiplot(input_stream[22272:],0)



