/* -*- c++ -*- */
/* 
 * Copyright 2018 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_LOBEE_LOBEE_CPP_BP_IMPL_H
#define INCLUDED_LOBEE_LOBEE_CPP_BP_IMPL_H

#include <cmath>
#include <cstdlib>
#include <vector>
#include <queue>
#include <complex>
#include <lobee/lobee_cpp_bp.h>

namespace gr {
  namespace lobee {

    class lobee_cpp_bp_impl : public lobee_cpp_bp
    {
     private:
      pmt::pmt_t d_out_port;

      demod_state_t d_state;
      unsigned short d_sf;
      unsigned short d_scale;
      unsigned short d_threshold;
      unsigned short d_mod_len;
      unsigned short d_half_mod_len;
      unsigned short d_quad_len;
      unsigned short d_half_quad_len;
      unsigned short d_SFD;

      std::vector<unsigned char> d_up_binary;  
      std::vector<unsigned char> d_zeros_binary;
      std::vector<unsigned char> d_ones_binary;
      std::vector<unsigned char> d_chirp;

      unsigned short d_preamble_idx;
      std::vector<unsigned short> d_corr_history;
      unsigned short d_corr_sum=0;
      unsigned short d_corr_max=0;
      unsigned short d_flag=0;
      
      unsigned short d_sync_recovery_cnt;
      unsigned short d_overlaps;
      unsigned short d_offset = 0;
      unsigned short d_sfd_history[4] = {0,0,0,0};
      unsigned short d_sfd_seq[4] = {2,4,0,0};
      unsigned short d_sfd_sum;

      const float d_lora_factor = 4;
      
      unsigned short d_symbol_cnt;
      std::vector<unsigned short> d_symbols;

     public:
      lobee_cpp_bp_impl(unsigned short spreading_factor, unsigned short scale, unsigned short threshold);
      ~lobee_cpp_bp_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
      
      unsigned short chirp_correlation(std::vector<unsigned char> chirp, unsigned short &corr_max, unsigned short &flag);
    };

  } // namespace lobee
} // namespace gr

#endif /* INCLUDED_LOBEE_LOBEE_CPP_BP_IMPL_H */

