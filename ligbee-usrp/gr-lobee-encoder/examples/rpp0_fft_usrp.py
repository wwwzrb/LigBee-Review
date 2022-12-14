#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Rpp0 Fft Usrp
# Generated: Tue Mar 23 18:59:31 2021
##################################################


if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"

from gnuradio import eng_notation
from gnuradio import gr
from gnuradio import uhd
from gnuradio import wxgui
from gnuradio.eng_option import eng_option
from gnuradio.fft import window
from gnuradio.filter import firdes
from gnuradio.wxgui import fftsink2
from gnuradio.wxgui import forms
from grc_gnuradio import wxgui as grc_wxgui
from optparse import OptionParser
import lora
import time
import wx


class rpp0_fft_usrp(grc_wxgui.top_block_gui):

    def __init__(self):
        grc_wxgui.top_block_gui.__init__(self, title="Rpp0 Fft Usrp")
        _icon_path = "/usr/share/icons/hicolor/32x32/apps/gnuradio-grc.png"
        self.SetIcon(wx.Icon(_icon_path, wx.BITMAP_TYPE_ANY))

        ##################################################
        # Variables
        ##################################################
        self.target_freq_sx1272 = target_freq_sx1272 = 868.1e6
        self.target_freq_rn2483 = target_freq_rn2483 = 868.08e6
        self.target_freq = target_freq = target_freq_sx1272
        self.sf = sf = 7
        self.samp_rate = samp_rate = 1e6
        self.rx_gain = rx_gain = 1
        self.noise_pow = noise_pow = 0
        self.downlink = downlink = False
        self.decimation = decimation = 1
        self.capture_freq = capture_freq = 868.1e6
        self.bw = bw = 500000

        ##################################################
        # Blocks
        ##################################################
        _rx_gain_sizer = wx.BoxSizer(wx.VERTICAL)
        self._rx_gain_text_box = forms.text_box(
        	parent=self.GetWin(),
        	sizer=_rx_gain_sizer,
        	value=self.rx_gain,
        	callback=self.set_rx_gain,
        	label='rx_gain',
        	converter=forms.float_converter(),
        	proportion=0,
        )
        self._rx_gain_slider = forms.slider(
        	parent=self.GetWin(),
        	sizer=_rx_gain_sizer,
        	value=self.rx_gain,
        	callback=self.set_rx_gain,
        	minimum=0,
        	maximum=1,
        	num_steps=100,
        	style=wx.SL_HORIZONTAL,
        	cast=float,
        	proportion=1,
        )
        self.Add(_rx_gain_sizer)
        self.wxgui_fftsink2_1 = fftsink2.fft_sink_c(
        	self.GetWin(),
        	baseband_freq=target_freq,
        	y_per_div=10,
        	y_divs=10,
        	ref_level=0,
        	ref_scale=2.0,
        	sample_rate=samp_rate,
        	fft_size=1024,
        	fft_rate=15,
        	average=False,
        	avg_alpha=None,
        	title='FFT Plot',
        	peak_hold=False,
        )
        self.Add(self.wxgui_fftsink2_1.win)
        self.uhd_usrp_source_0 = uhd.usrp_source(
        	",".join(("addr=192.168.10.6", "")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		channels=range(1),
        	),
        )
        self.uhd_usrp_source_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0.set_center_freq(target_freq, 0)
        self.uhd_usrp_source_0.set_normalized_gain(rx_gain, 0)
        self.lora_message_socket_sink_0 = lora.message_socket_sink('127.0.0.1', 40868, 0)
        self.lora_fft_receiver_0 = lora.fft_receiver(samp_rate, bw, sf, False, 4, True, False, 4, 7, '/home/wangzhe/sjtu/USRP/MLoRa/online/')

        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.lora_fft_receiver_0, 'frames'), (self.lora_message_socket_sink_0, 'in'))
        self.connect((self.uhd_usrp_source_0, 0), (self.lora_fft_receiver_0, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.wxgui_fftsink2_1, 0))

    def get_target_freq_sx1272(self):
        return self.target_freq_sx1272

    def set_target_freq_sx1272(self, target_freq_sx1272):
        self.target_freq_sx1272 = target_freq_sx1272
        self.set_target_freq(self.target_freq_sx1272)

    def get_target_freq_rn2483(self):
        return self.target_freq_rn2483

    def set_target_freq_rn2483(self, target_freq_rn2483):
        self.target_freq_rn2483 = target_freq_rn2483

    def get_target_freq(self):
        return self.target_freq

    def set_target_freq(self, target_freq):
        self.target_freq = target_freq
        self.wxgui_fftsink2_1.set_baseband_freq(self.target_freq)
        self.uhd_usrp_source_0.set_center_freq(self.target_freq, 0)

    def get_sf(self):
        return self.sf

    def set_sf(self, sf):
        self.sf = sf

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.wxgui_fftsink2_1.set_sample_rate(self.samp_rate)
        self.uhd_usrp_source_0.set_samp_rate(self.samp_rate)

    def get_rx_gain(self):
        return self.rx_gain

    def set_rx_gain(self, rx_gain):
        self.rx_gain = rx_gain
        self._rx_gain_slider.set_value(self.rx_gain)
        self._rx_gain_text_box.set_value(self.rx_gain)
        self.uhd_usrp_source_0.set_normalized_gain(self.rx_gain, 0)


    def get_noise_pow(self):
        return self.noise_pow

    def set_noise_pow(self, noise_pow):
        self.noise_pow = noise_pow

    def get_downlink(self):
        return self.downlink

    def set_downlink(self, downlink):
        self.downlink = downlink

    def get_decimation(self):
        return self.decimation

    def set_decimation(self, decimation):
        self.decimation = decimation

    def get_capture_freq(self):
        return self.capture_freq

    def set_capture_freq(self, capture_freq):
        self.capture_freq = capture_freq

    def get_bw(self):
        return self.bw

    def set_bw(self, bw):
        self.bw = bw


def main(top_block_cls=rpp0_fft_usrp, options=None):

    tb = top_block_cls()
    tb.Start(True)
    tb.Wait()


if __name__ == '__main__':
    main()
