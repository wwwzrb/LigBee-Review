import numpy as np
from Stack import Stack, State
from gnuradio import gr
import pmt

class my_lobee_test_bp(gr.basic_block):
    """
    docstring for block my_lobee_test_bp
    """

    def __init__(self, spreading_factor, scale, threshold):
        self.spreading_factor = spreading_factor
        self.scale = scale
        self.threshold = threshold
        self.mod_len = 256
        self.quad_len = self.scale * self.mod_len
        self.half_quad_len = int(self.quad_len / 2)
        self.SFD = int(self.quad_len / 4)
        self.up_binary = [ bool(x) for x in np.zeros(self.half_quad_len) ] + [ bool(x) for x in np.ones(self.half_quad_len)
                                                                             ]
        self.zeros_binary = self.up_binary[0:self.half_quad_len]
        self.ones_binary = self.up_binary[self.half_quad_len:]
        gr.basic_block.__init__(self, name='my_lobee_test_bp', in_sig=[
         np.uint8], out_sig=None)
        self.message_port_register_out(pmt.intern('out'))
        self.debug_verbose = 2
        self.debug_info = 1
        self.debug = 2
        return

    def general_work(self, input_items, output_items):
        in0 = input_items[0]
        symbols = []
        preamble_chirps = 6
        preamble_tolerance = 1
        sfd_chirps = 4
        sfd_tolerance = 1
        sfd_seq = [2, 4, 0, 0]
        sfd_sum = int(self.SFD * 3 / 4)
        corr_sum = int(self.half_quad_len * 3 / 4)
        corr_history = Stack()
        d_state = State.S_RESET
        i = 0
        num_out = 0
        while i <= len(in0) - self.quad_len:
            if d_state == State.S_RESET:
                corr_history.clear()
                d_state = State.S_PREFILL
                if self.debug >= self.debug_verbose:
                    print 'S_PREFILL'
            if d_state == State.S_READ_PAYLOAD:
                chirp_binary = in0[i:i + self.quad_len]
                curr_max, curr_index, flag = self.chirp_demod(chirp_binary)
                if self.debug >= self.debug_info:
                    print num_out + 12, 'curr_index: ', curr_index, 'curr_max', curr_max, 'flag', flag
                if curr_max >= corr_sum + self.threshold:
                    symbols.append(curr_index)
                    num_out += 1
                    i += self.quad_len
                    continue
                else:
                    output = pmt.init_u16vector(num_out, symbols)
                    msg_pair = pmt.cons(pmt.make_dict(), output)
                    self.message_port_pub(pmt.intern('out'), msg_pair)
                    symbols = []
                    d_state = State.S_RESET
                    num_out = 0
                    break
            chirp_binary = in0[i:i + self.quad_len]
            curr_max, curr_index = self.chirp_correlation(chirp_binary)
            corr_history.push(curr_index)
            if corr_history.size() > preamble_chirps:
                corr_history.pop_back()
            if d_state == State.S_PREFILL:
                if corr_history.size() >= preamble_chirps:
                    d_state = State.S_DETECT_PREAMBLE
                    if self.debug >= self.debug_verbose:
                        print 'S_DETECT_PREAMBLE'
                else:
                    i += self.quad_len
                    if i > len(in0) - self.quad_len:
                        break
                    continue
            if d_state == State.S_DETECT_PREAMBLE:
                preamble_index = corr_history.bottom()
                preamble_found = True
                for j in range(1, preamble_chirps):
                    if abs(preamble_index - corr_history.get_i(j)) > preamble_tolerance:
                        preamble_found = False
                        break

                if preamble_found:
                    d_state = State.S_SFD_SYNC
                    if self.debug_verbose:
                        print 'S_SFD_SYNC'
                else:
                    i += self.quad_len
                    if i > len(in0) - self.quad_len:
                        break
                    continue
            if d_state == State.S_SFD_SYNC:
                sfd_index = [
                 curr_index]
                sfd_found = True
                for j in range(1, sfd_chirps + 1):
                    if abs(sfd_index[j - 1] - sfd_seq[j - 1]) > sfd_tolerance:
                        sfd_found = False
                        break
                    elif len(sfd_index) < sfd_chirps:
                        if i + (j + 1) * self.quad_len > len(in0) - self.quad_len:
                            sfd_found = False
                            break
                        chirp_binary = in0[i + j * self.quad_len:i + (j + 1) * self.quad_len]
                        curr_max, curr_index = self.chirp_correlation(chirp_binary)
                        sfd_index.append(curr_index)

                sfd_corr = np.sum(in0[i + sfd_chirps * self.quad_len:i + sfd_chirps * self.quad_len + self.SFD])
                if sfd_found and np.sum(np.abs(np.subtract(sfd_index, sfd_seq))) < sfd_tolerance + 1 and sfd_corr >= sfd_sum:
                    d_state = State.S_READ_PAYLOAD
                    if self.debug >= self.debug_info:
                        print 'S_READ_PAYLOAD'
                        print 'sfd_index', sfd_index, 'sfd_corr', sfd_corr
                    i += sfd_chirps * self.quad_len + self.SFD
                else:
                    i += 1
                    if i >= len(in0) - self.quad_len - 1:
                        break
                    continue

        if i > len(in0):
            num_consumed = len(in0)
        else:
            num_consumed = i
        self.consume(0, num_consumed)
        return 0

    def chirp_correlation(self, chirp_binary):
        corr_zero = []
        corr_one = []
        for j in range(0, self.half_quad_len + 1):
            corr_zero.append(np.sum(np.logical_not(np.logical_xor(chirp_binary[j:j + self.half_quad_len], self.zeros_binary))))
            corr_one.append(np.sum(np.logical_not(np.logical_xor(chirp_binary[j:j + self.half_quad_len], self.ones_binary))))

        corr_zero_max = np.amax(corr_zero)
        corr_zero_index = np.where(corr_zero[:] == corr_zero_max)[0]
        corr_zero_index = np.sum(corr_zero_index) / len(corr_zero_index)
        corr_zero_index = int(round(corr_zero_index))
        corr_one_max = np.amax(corr_one)
        corr_one_index = np.where(corr_one[:] == corr_one_max)[0]
        corr_one_index = np.sum(corr_one_index) / len(corr_one_index)
        corr_one_index = int(round(corr_one_index))
        if corr_zero_max >= corr_one_max:
            curr_index = round((self.quad_len - corr_zero_index) / self.scale)
            curr_max = corr_zero_max
        else:
            curr_index = round((self.half_quad_len - corr_one_index) / self.scale)
            curr_max = corr_one_max
        curr_index = int(round(curr_index % 128 / 4))
        return (
         curr_max, curr_index)

    def chirp_demod(self, chirp_binary):
        corr_zero = []
        corr_one = []
        for j in range(0, self.half_quad_len + 1):
            corr_zero.append(np.sum(np.logical_not(np.logical_xor(chirp_binary[j:j + self.half_quad_len], self.zeros_binary))))
            corr_one.append(np.sum(np.logical_not(np.logical_xor(chirp_binary[j:j + self.half_quad_len], self.ones_binary))))

        corr_zero_max = np.amax(corr_zero)
        corr_zero_index = np.where(corr_zero[:] == corr_zero_max)[0]
        corr_zero_index = np.sum(corr_zero_index) / len(corr_zero_index)
        corr_zero_index = int(round(corr_zero_index))
        corr_one_max = np.amax(corr_one)
        corr_one_index = np.where(corr_one[:] == corr_one_max)[0]
        corr_one_index = np.sum(corr_one_index) / len(corr_one_index)
        corr_one_index = int(round(corr_one_index))
        if corr_zero_max >= corr_one_max:
            curr_index = round((self.quad_len - corr_zero_index) / self.scale)
            curr_max = corr_zero_max
            flag = 0
        else:
            curr_index = round((self.half_quad_len - corr_one_index) / self.scale)
            curr_max = corr_one_max
            flag = 1
        curr_index = int(round((curr_index - 64) % 256 / 4))
        if corr_one_max < self.threshold:
            curr_max = corr_one_max
        return (
         curr_max, curr_index, flag)

