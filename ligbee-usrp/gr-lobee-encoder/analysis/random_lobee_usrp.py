from loranode import RN2483Controller
from loraconfig import LoRaConfig
from RandomPacket import RandomPacket
import pickle as pickle
import time


def read_as_object(file):
    with open(file, 'rb') as input_obj:
        obj = pickle.load(input_obj)

    print('Read Object')
    return obj


def transmit_data(device, file):
    pre_delay = 0.150
    post_delay = 1.0
    intra_delay = 0.1  # OS=8

    # basic_seq_0 = ["12", "34", "56", "78"]
    # basic_seq_1 = ["21", "43", "65", "87"]
    # basic_seq_2 = ["87", "65", "43", "21"]
    # basic_seq = [basic_seq_0, basic_seq_1, basic_seq_2]
    #
    # seq_len = 7
    # seq_idx = 1
    # seq = ""
    #
    # for i in range(seq_len):
    #     seq += basic_seq[seq_idx][i % 4]

    # seq = "12345678"  # 4 Byte
    # seq = "87654321"  # 4 Byte
    # seq = "1234567812345678"  # 8 Byte
    # seq = "12345678123456781234567812345678"  # 16 Byte
    # seq = "FFFEFCF8F0E1C2"
    # seq = "21346587"
    # seq_0 = "0123456789ABCD"
    # seq_1 = "1032547698BADC"

    randomPacket = read_as_object(file)

    # time.sleep(pre_delay)
    for i in range(0, len(randomPacket.packet_strs)):
        payload = randomPacket.packet_strs[i]
        print(payload)
        device.send_p2p(payload)
        time.sleep(intra_delay)

    # time.sleep(post_delay)


# Initialize lora configuration
config = LoRaConfig(freq=868.1e6, sf=7, cr="4/8", bw=125*1e3, prlen=8, crc=False, implicit=False)
print(config.string_repr())

# Configure transmitter
c = RN2483Controller("/dev/ttyACM0")
try:
    c.set_freq(int(config.freq))
    c.set_sf(config.sf)
    c.set_cr(config.cr)
    c.set_bw(int(config.bw // 1000))
    c.set_prlen(str(config.prlen))
    c.set_crc("on" if config.crc else "off")
    # c.set_implicit("on" if config.implicit else "off")
    c.set_sync('34')
    c.set_pwr(14)
    print("Freq: " + c.get_freq())
    print("Pre length: " + c.get_prlen())
    print("Bandwidth : " + c.get_bw())
    print("Sync words: " + c.get_sync())
    print("Power     : " + c.get_pwr())
except Exception as e:
    print(e)
    exit(1)

# Transmit random data
# 100 Repeat
# load_prefix = "./RandomByte/RandomPacket20191124"
# load_name = load_prefix + "1850.pkl"
# load_name = load_prefix + "1855.pkl"

# 200 Repeat
load_prefix = "./RandomByte/RandomPacket20191126"
load_name = load_prefix + "0915.pkl"
# load_name = load_prefix + "0916.pkl"
# load_name = load_prefix + "0917.pkl"
# load_name = load_prefix + "0918.pkl"
# load_name = load_prefix + "0919.pkl"
# load_name = load_prefix + "0920.pkl"
# load_name = load_prefix + "0921.pkl"
# load_name = load_prefix + "0922.pkl"
# load_name = load_prefix + "0923.pkl"
# load_name = load_prefix + "0924.pkl"


transmit_data(c, load_name)
