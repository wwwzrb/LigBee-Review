from loranode import RN2483Controller
from loranode.rpyutils import printd, Level, Color, clr, set_debug_level
from loraconfig import LoRaConfig
import collections
from threading import Thread, Lock
import time

Test = collections.namedtuple('Test', ['payload', 'times'])


def set_device(device, config):
    c = device
    try:
        # c.set_freq(int(config.freq))
        c.set_sf(config.sf)
        c.set_cr(config.cr)
        c.set_bw(int(config.bw // 1000))
        c.set_prlen(config.prlen)
        c.set_crc("on" if config.crc else "off")
        # # c.set_implicit("on" if config.implicit else "off")
        c.set_sync('11')
        c.set_pwr(14)
        c.set_wdt(config.wdt)
        print("Freq: " + c.get_freq())
        print("SF:   " + c.get_sf())
        print("CR:   " + c.get_cr())
        print("Pre length: " + c.get_prlen())
        print("Bandwidth : " + c.get_bw())
        print("Sync words: " + c.get_sync())
        print("Power     : " + c.get_pwr())
        print("RX timeout: " + c.get_wdt())
    except Exception as e:
        print(e)
        exit(1)


class TransmitterThread(Thread):
    def __init__(self, mutex, name, config):
        Thread.__init__(self)
        self.setDaemon(True)
        self.mutex = mutex
        self.device = RN2483Controller(name)
        set_device(self.device, config)
        self.counter = 0
        self.time_begin = None

    def run(self):
        self.mutex.acquire()
        self.time_begin = time.time()
        print("--- begin to send ---", self.time_begin)
        self.mutex.release()
        while True:
            if self.counter >= repeat:
                return

            data = "21436587"

            self.mutex.acquire()
            printd(clr(Color.GREEN, "--- Num: {0} --- SEND: {1} --- TIME: {2}".format(self.counter, data, time.time())), Level.INFO)
            self.mutex.release()

            self.device.send_p2p(data)
            self.counter += 1
            time.sleep(0.05)


class ReceiverThread(Thread):
    def __init__(self, mutex, name, config):
        Thread.__init__(self)
        self.setDaemon(True)
        self.mutex = mutex
        self.device = RN2483Controller(name)
        set_device(self.device, config)
        self.counter = 0
        self.time_begin = None

    def run(self):
        self.mutex.acquire()
        self.time_begin = time.time()
        print("--- begin to recv ---", self.time_begin)
        self.mutex.release()
        time.sleep(0.5)
        self.mutex.acquire()
        print("--- actual begin  ---", time.time())
        self.mutex.release()
        while True:
            if self.counter >= repeat:
                return

            # receive n symbols with wdt timeout in ms
            self.device.set_wdt(1000)
            data = self.device.recv_symbol(1000)

            # continuous receive
            # data = self.device.recv_p2p()

            self.mutex.acquire()
            printd(clr(Color.BLUE, "--- Num: {0} --- RECV: {1} --- TIME: {2}".format(self.counter, data, time.time())), Level.INFO)
            self.mutex.release()

            self.counter += 1


if __name__ == "__main__":
    # set_debug_level(Level.DEBUG)

    # Initialize lora configuration
    configure = LoRaConfig(freq=868.1e6, sf=7, cr="4/8", bw=500 * 1e3, prlen=8, wdt=0, crc=False, implicit=False)
    print(configure.string_repr())
    repeat = 10

    # Initialize T/RX thread
    m = Lock()
    tx_path = "/dev/ttyACM0"
    rx_path = "/dev/ttyACM1"
    r = None
    t = None
    # t = "Disable"

    if not r:
        r = ReceiverThread(m, rx_path, configure)
        r.start()

    if not t:
        t = TransmitterThread(m, tx_path, configure)
        t.start()

    if t is not "Disable":
        print("--- %s seconds ---" % (t.time_begin - r.time_begin))

    if r:
        r.join()

    if t is not "Disable":
        t.join()
