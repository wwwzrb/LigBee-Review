import scipy
import numpy as np
from RandomPacket import RandomPacket
import pickle as pickle


def read_as_object(file):
    with open(file, 'rb') as input_obj:
        obj = pickle.load(input_obj)

    print('Read Object')
    return obj


def save_as_object(obj, file_name):
    with open(file_name, 'wb') as output:
        pickle.dump(obj, output, pickle.HIGHEST_PROTOCOL)


result = scipy.fromfile(open('../data/lora_result'), dtype=scipy.uint8)
chirp_result = scipy.fromfile(open('../data/lora_chirp_result'), dtype=scipy.uint32)

packet_header = np.array([4, 128, 240], dtype=np.uint8)

# Repeat 100
# postfix = "201911241850.pkl"
# postfix = "201911241855.pkl"

# Repeat 200
# postfix = "201911260915.pkl"
# postfix = "201911260916.pkl"
# postfix = "201911260917.pkl"
# postfix = "201911260918.pkl"
# postfix = "201911260919.pkl"
# postfix = "201911260920.pkl"
# postfix = "201911260921.pkl"
# postfix = "201911260922.pkl"
# postfix = "201911260923.pkl"
postfix = "201911260924.pkl"
load_name = "./RandomByte/RandomPacket" + postfix
randomPacket = read_as_object(load_name)

num_packets = len(randomPacket.packet_strs)
num_payload_bytes = randomPacket.num_payload_bytes
num_packet_bytes = randomPacket.num_packet_bytes
num_packet_chirps = randomPacket.num_packet_chirps

error_cnt = []
header_err_cnt = []
for i in range(num_packets):
    comp_result = []
    header_result = []
    packet_byte = np.array(randomPacket.packet_lists[i], np.uint8)
    for j in range(0, 3):
        header_result.append(bin(packet_byte[j] ^ result[i*num_packet_bytes+j]).count('1'))
    for j in range(0, num_packet_bytes):
        comp_result.append(bin(packet_byte[j] ^ result[i*num_packet_bytes+j]).count('1'))

    error_cnt.append(np.sum(comp_result))
    header_err_cnt.append(np.sum(header_result))

    packet_symbol = chirp_result[i*num_packet_chirps:(i+1)*num_packet_chirps]
    randomPacket.packet_symbols.append(packet_symbol)

print("Bit Error Rate: ", np.sum(error_cnt) / 8 / len(result))
print("Packet Reception Ratio: ", (len(error_cnt)-len(np.nonzero(header_err_cnt)[0]))/num_packets)
print("Frame Reception Ratio: ", (len(error_cnt)-len(np.nonzero(error_cnt)[0]))/num_packets)

save_name = "./RandomByte/RandomPacketSymbol"+postfix
save_as_object(randomPacket, save_name)

randomPacketSymbol = read_as_object(save_name)
