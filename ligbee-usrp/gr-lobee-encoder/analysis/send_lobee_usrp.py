from loranode import RN2483Controller
from loraconfig import LoRaConfig
import collections
import time

Test = collections.namedtuple('Test', ['payload', 'times'])


def transmit_data(device):
    pre_delay = 0.150
    post_delay = 1.0
    intra_delay = 0.1

    basic_seq_0 = "12345678123456"
    basic_seq_1 = "21436587214365"
    basic_seq_2 = "87654321876543"
    basic_seq = [basic_seq_0, basic_seq_1, basic_seq_2]

    seq_len = 7
    basic_len = 7
    seq_idx = 1
    seq = ""

    for i in range(seq_len//basic_len):
        seq += basic_seq[seq_idx][:2*seq_len]

    # seq = "12345678"  # 4 Byte
    # seq = "87654321"  # 4 Byte
    # seq = "1234567812345678"  # 8 Byte
    # seq = "12345678123456781234567812345678"  # 16 Byte
    # seq = "FFFEFCF8F0E1C2"
    seq = "21436587"  # 4 Byte
    # seq_0 = "0123456789ABCD"
    # seq_1 = "1032547698BADC"
    test = Test(payload=seq, times=10)
    print(test.payload)

    # time.sleep(pre_delay)

    start_timestamp = time.time()
    for i in range(0, test.times):
        device.send_p2p(test.payload)
        time.sleep(intra_delay)

    # time.sleep(post_delay)

    return start_timestamp


# Initialize lora configuration
config = LoRaConfig(freq=868.1e6, sf=7, cr="4/8", bw=500*1e3, prlen=8, crc=False, implicit=False)
print(config.string_repr())

# Configure transmitter
# c = RN2483Controller("/dev/ttyACM0")
c = RN2483Controller("COM6")  # windows path
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

start_time = transmit_data(c)
print("--- %s seconds ---" % (time.time() - start_time))
