/* -*- c++ -*- */
/*
 * Copyright 2018 Pieter Robyns.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef LORACONF_H
#define LORACONF_H

#include <lora/loratap.h>
#include <lora/loraphy.h>

typedef struct __attribute__((__packed__)) loraconf {
    loratap_header_t tap;
    loraphy_header_t phy;
} loraconf_t;

void print_loraconf(std::ostream& out, loraconf_t& conf) {
    out << "*** TAP" << std::endl;
    out << "\tVersion: " << conf.tap.lt_version << std::endl;
    out << "\tPadding: " << conf.tap.lt_padding << std::endl;
    out << "\tLength: " << conf.tap.lt_length << std::endl;

    out << "\tFrequency: " << conf.tap.channel.frequency << std::endl;
    out << "\tBandwidth: " << (int)conf.tap.channel.bandwidth << std::endl;
    out << "\tSF: " << (int)conf.tap.channel.sf << std::endl;

    out << "\tPacket RSSI: " << conf.tap.rssi.packet_rssi << std::endl;
    out << "\tMax RSSI: " << conf.tap.rssi.max_rssi << std::endl;
    out << "\tCurrent RSSI: " << conf.tap.rssi.current_rssi << std::endl;
    out << "\tSNR: " << conf.tap.rssi.snr << std::endl;

    out << "\tSync: " << conf.tap.sync_word << std::endl;

    out << "*** PHY" << std::endl;
    out << "\tLength: " << (int)conf.phy.length << std::endl;
    out << "\tCR: " << (int)conf.phy.cr << std::endl;
    out << "\tHas CRC: " << (bool)conf.phy.has_mac_crc << std::endl;
    uint8_t crc = (conf.phy.crc_msn << 4) | conf.phy.crc_lsn;
    out << "\tCRC: " << (int)crc << std::endl;
    out << "\tReserved: " << (int)conf.phy.reserved << std::endl;
}

#endif
