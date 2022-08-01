import numpy as np
import random
import pickle as pickle
from lora_encode import LoRaEncode

class RandomPacket:
    def __init__(self, repeat, spread_factor, code_rate, payload_length):
        self.d_sf = spread_factor
        self.d_cr = code_rate
        self.num_payload_bytes = payload_length
        self.num_packet_bytes = 3 + payload_length
        self.num_packet_chirps = int(8 + (4 + self.d_cr) * np.ceil(payload_length * 2.0 / spread_factor))

        self.packet_num = repeat
        self.packet_len = self.num_packet_chirps

        self.packet_header = [4, 128, 240]
        self.packet_list = None
        self.packet_hex = None
        self.packet_symbol = None
        self.packet_str = None

        self.packet_lists = []
        self.packet_hexs = []
        self.packet_symbols = []
        self.packet_strs = []


    def generate_packet(self):
        self.packet_list = self.packet_header.copy()
        for i in range(3, self.num_packet_bytes):
            self.packet_list.append(random.randint(0, 255))

        self.packet_hex = [hex(x) for x in self.packet_list]

        self.packet_str = ''.join('{:02X}'.format(self.packet_list[i]) for i in range(3,len(self.packet_list)))

        self.packet_lists.append(self.packet_list)
        self.packet_hexs.append(self.packet_hex)
        self.packet_strs.append(self.packet_str)

def save_as_object(obj, file_name):
    with open(file_name, 'wb') as output:
        pickle.dump(obj, output, pickle.HIGHEST_PROTOCOL)

if __name__ == "__main__":
    rpt=200
    sf = 7
    cr = 4
    pl = 4

    randomPacket = RandomPacket(rpt, sf, cr, pl)

    for idx in range(rpt):
        randomPacket.generate_packet()

    file_prefix = "./RandomByte/"
    file_path = file_prefix + "RandomPacket201911260915.pkl"
    # file_path = file_prefix + "RandomPacket201911260916.pkl"
    # file_path = file_prefix + "RandomPacket201911260917.pkl"
    # file_path = file_prefix + "RandomPacket201911260918.pkl"
    # file_path = file_prefix + "RandomPacket201911260919.pkl"
    # file_path = file_prefix + "RandomPacket201911260920.pkl"
    # file_path = file_prefix + "RandomPacket201911260921.pkl"
    # file_path = file_prefix + "RandomPacket201911260922.pkl"
    # file_path = file_prefix + "RandomPacket201911260923.pkl"
    # file_path = file_prefix + "RandomPacket201911260924.pkl"
    save_as_object(randomPacket, file_path)
