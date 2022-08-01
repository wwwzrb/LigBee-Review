#!/usr/bin/env python
# -*- coding: utf-8 -*-

from gnuradio import gr
import gnuradio
import lora

class lora_decoder(gr.hier_block2):
    """
    docstring for block lora_decoder
    """
    def __init__(self, samp_rate, center_freq, bandwidth, sf, implicit, cr, crc, reduced_rate=False, conj=False, disable_drift_correction=False, path=""):
        gr.hier_block2.__init__(self,
            "lora_decoder",
            gr.io_signature(1, 1, gr.sizeof_gr_complex),  # Input signature
            gr.io_signature(0, 0, 0)) # Output signature
        
        # Parameters
        self.samp_rate     = samp_rate
        self.center_freq   = center_freq
        # self.channel_list  = channel_list
        self.bandwidth     = bandwidth
        self.sf            = sf
        self.implicit      = implicit
        self.cr            = cr
        self.crc           = crc
        # self.decimation    = decimation
        self.reduced_rate  = reduced_rate
        self.conj          = conj
        # self.disable_channelization = disable_channelization
        self.disable_drift_correction = disable_drift_correction        

        # Define blocks
        self.block_conj = gnuradio.blocks.conjugate_cc()
        self.decoder = lora.decoder(samp_rate, bandwidth, sf, implicit, cr, crc, reduced_rate, disable_drift_correction, path)

        # Messages
        self.message_port_register_hier_out('frames')        

        # Connect blocks
        self._connect_conj_block_if_enabled(self, self.decoder)

        self.msg_connect((self.decoder, 'frames'), (self, 'frames'))
        
    def _connect_conj_block_if_enabled(self, source, dest):
        if self.conj:
            self.connect((source, 0), (self.block_conj, 0))
            self.connect((self.block_conj, 0), (dest, 0))
        else:
            self.connect((source, 0), (dest, 0))

    def get_sf(self):
        return self.sf

    def set_sf(self, sf):
        self.sf = sf
        self.decoder.set_sf(self.sf)

    def get_center_freq(self):
        return self.center_freq

    def set_center_freq(self, center_freq):
        self.center_freq = center_freq
