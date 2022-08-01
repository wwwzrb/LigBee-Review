from loranode import RN2483Controller
from loraconfig import LoRaConfig
import collections
import time

Test = collections.namedtuple('Test', ['payload', 'times'])


def receive_data(device):
    pre_delay = 0.150

    time.sleep(pre_delay)

    while True:
        rec = device.recv_p2p()
        print("Rec"+rec)


# Initialize lora configuration
config = LoRaConfig(freq=868.1e6, sf=7, cr="4/8", bw=125e3, crc=False, implicit=False)
print(config.string_repr())

# Configure transmitter
c = RN2483Controller("/dev/ttyACM3")
try:
    # self.lc.set_freq(config.freq)
    c.set_sf(config.sf)
    c.set_cr(config.cr)
    c.set_bw(config.bw / 1e3)
    c.set_prlen(str(config.prlen))
    c.set_crc("on" if config.crc else "off")
    # c.set_implicit("on" if config.implicit else "off")
    c.set_pwr(14)
    # print(c.get_sync())
except Exception as e:
    print(e)
    exit(1)

receive_data(c)
