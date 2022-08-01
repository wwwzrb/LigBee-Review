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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "lobee_cpp_bp_impl.h"

#define DEBUG_OFF     1
#define DEBUG_INFO    1
#define DEBUG_VERBOSE 2
#define DEBUG         DEBUG_OFF

#define OVERLAP_DEFAULT 1
#define OVERLAP_FACTOR  1

namespace gr {
  namespace lobee {

    lobee_cpp_bp::sptr
    lobee_cpp_bp::make(unsigned short spreading_factor, unsigned short scale, unsigned short threshold)
    {
      return gnuradio::get_initial_sptr
        (new lobee_cpp_bp_impl(spreading_factor, scale, threshold));
    }

    /*
     * The private constructor
     */
    lobee_cpp_bp_impl::lobee_cpp_bp_impl(unsigned short spreading_factor, unsigned short scale, unsigned short threshold)
      : gr::block("lobee_cpp_bp",
              gr::io_signature::make(1, 1, sizeof(char)),
              gr::io_signature::make(0, 0, 0)),
        d_sf(spreading_factor),
        d_scale(scale),
        d_threshold(threshold)
    {
      d_out_port = pmt::mp("out");
      message_port_register_out(d_out_port);

      d_state = S_RESET;

      d_mod_len = (1<<d_sf);
      d_half_mod_len = (unsigned short)(d_mod_len/2);
      d_quad_len = (unsigned short)d_scale*d_mod_len;
      d_half_quad_len = (unsigned short)(d_quad_len/2);
      d_SFD = (unsigned short)(d_quad_len/4);
      d_sfd_sum = (unsigned short)(d_SFD*3/4);
      d_corr_sum = (unsigned short)(d_half_quad_len * 3 / 4);
      d_overlaps = OVERLAP_DEFAULT;
      d_offset = 0;

      std::vector<unsigned char> zeros_binary(d_half_quad_len, 0);
      std::vector<unsigned char> ones_binary(d_half_quad_len, 1);
      d_zeros_binary = zeros_binary;
      d_ones_binary = ones_binary;
      d_up_binary=zeros_binary;
      d_up_binary.insert(d_up_binary.end(),d_ones_binary.begin(), d_ones_binary.end());

    }

    /*
     * Our virtual destructor.
     */
    lobee_cpp_bp_impl::~lobee_cpp_bp_impl()
    {
    }

    void
    lobee_cpp_bp_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      ninput_items_required[0] = d_quad_len*noutput_items;
    }

    int
    lobee_cpp_bp_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const unsigned char *in = (const unsigned char *) input_items[0];
      unsigned char *out = (unsigned char *) output_items[0];

      unsigned short num_consumed = d_quad_len;
      unsigned short curr_index = 0;
      bool preamble_found = false;
      bool sfd_found      = false;
      unsigned short sfd_begin = 0;
      unsigned short sfd_error =0;
      unsigned short sfd_corr = 0;

      d_chirp.clear();
      d_chirp.resize(d_quad_len);
      std::copy(&in[0], &in[d_quad_len], d_chirp.begin());

      curr_index=chirp_correlation(d_chirp,d_corr_max,d_flag);
      
      if(d_state==S_READ_PAYLOAD)
      {
        curr_index=(unsigned short)round(((curr_index-d_mod_len/4 +d_mod_len) % d_mod_len)/d_lora_factor);
      }
      else
      {
        curr_index= (unsigned short)round((curr_index%d_half_mod_len) / d_lora_factor);
      }

      d_corr_history.insert(d_corr_history.begin(), curr_index);
      if(d_corr_history.size()>REQUIRED_PREAMBLE_CHIRPS)
      {
        d_corr_history.pop_back();
      }

      switch(d_state){
      case S_RESET:
      {
        d_overlaps = OVERLAP_DEFAULT;
        d_offset = 0;
        d_symbols.clear();
        d_corr_history.clear();
        d_sync_recovery_cnt = 0;
        curr_index = 0;
        preamble_found = false;
        sfd_found      = false;
        sfd_begin = 0;
        sfd_error =0;
        sfd_corr = 0;
        d_symbol_cnt = 0;

        d_state = S_PREFILL;

        #if DEBUG >= DEBUG_INFO
          std::cout << "Next state: S_PREFILL" << std::endl;
        #endif

        break;
      
      case S_PREFILL:
        if(d_corr_history.size()>=REQUIRED_PREAMBLE_CHIRPS)
        {
          d_state = S_DETECT_PREAMBLE;

          #if DEBUG >= DEBUG_INFO
            std::cout << "Next state: S_DETECT_PREAMBLE" << std::endl;
          #endif
        }

        break;

      }

      
      case S_DETECT_PREAMBLE:
      {
        d_preamble_idx = d_corr_history[0];

        #if DEBUG >= DEBUG_VERBOSE
          std::cout << "PREAMBLE " << d_corr_history[0] << std::endl;
        #endif

        // Check for discontinuities that exceed some tolerance
        preamble_found = true;
        for (int i = 1; i < REQUIRED_PREAMBLE_CHIRPS; i++)
        {
          if (abs(int(d_preamble_idx) - int(d_corr_history[i])) > LORA_PREAMBLE_TOLERANCE)
          {
            preamble_found = false;
          }
        }

        if (preamble_found && d_corr_max>=d_corr_sum+d_threshold)
        {
          d_state = S_SFD_SYNC;

          #if DEBUG >= DEBUG_INFO
            std::cout << "Next state: S_SFD_SYNC" << std::endl;
          #endif
        }
        break;

      }

      
      case S_SFD_SYNC:
      {
        d_overlaps = OVERLAP_FACTOR;

        if (d_sync_recovery_cnt++ > (unsigned short)(d_quad_len/d_overlaps))
        {
          d_state = S_RESET;
          d_overlaps = OVERLAP_DEFAULT;

          #if DEBUG >= DEBUG_INFO
            std::cout << "Bailing out of sync loop"   << std::endl;
            std::cout << "Next state: S_RESET" << std::endl;
          #endif
        }

        for(int i=0;i<(d_quad_len/d_overlaps);i++)
        {
          sfd_found = true;
          d_offset = (unsigned short)(i*d_overlaps);
          sfd_error =0 ;

          for(int j=0;j<4;j++)
          {
            d_chirp.clear();
            d_chirp.resize(d_quad_len);
            std::copy(&in[d_offset+j*d_quad_len], &in[d_offset+(j+1)*d_quad_len], d_chirp.begin());
            curr_index=chirp_correlation(d_chirp,d_corr_max,d_flag);
            curr_index= (unsigned short)round((curr_index%d_half_mod_len) / d_lora_factor);

            if(abs((int)curr_index-(int)d_sfd_seq[j])<LORA_SFD_TOLERANCE+1)
            {
              sfd_error+=abs((int)curr_index-(int)d_sfd_seq[j]);

              # if DEBUG >= DEBUG_VERBOSE
               std::cout << "sync_word " << curr_index << " corr_max: "<< d_corr_max << " flag: " << d_flag << " sfd_error: " << sfd_error << std::endl;
              # endif
            }
            else
            {
              sfd_error=0;
              sfd_found =false;
              break;
            }
          }

          if(sfd_found&& sfd_error<LORA_SFD_TOLERANCE+1)
          {
            sfd_begin = d_offset + 4*d_quad_len;
            for(int j=0;j<d_SFD;j++)
            {
              if(in[sfd_begin+j])
              {
                sfd_corr++;
              }
            }
            if(sfd_corr>=d_sfd_sum+d_threshold)
            {
              num_consumed = sfd_begin + d_SFD;

              d_state = S_READ_PAYLOAD;
              d_overlaps = OVERLAP_DEFAULT;

              #if DEBUG >= DEBUG_INFO
                std::cout << "Next state: S_READ_PAYLOAD" << std::endl;
              #endif

              break;             
            }

          }

        }

        break;

      }
       
      case S_READ_PAYLOAD:
      {
        if (d_corr_max < d_corr_sum+d_threshold)
        {
          d_state = S_OUT;

          #if DEBUG >= DEBUG_INFO
            std::cout << "Exit d_corr_sum: " << d_corr_sum+d_threshold << " Current d_corr_max: " << d_corr_max << std::endl; 
            std::cout << "Next state: S_OUT" << std::endl;
          #endif
          
          break;
        }

        #if DEBUG >= DEBUG_VERBOSE
          std::cout << "symbol " << 12+d_symbol_cnt << " curr_index: " << curr_index << " corr_max: " << d_corr_max << " flag: " << d_flag  << std::endl;
        #endif

        d_symbol_cnt++;
        d_symbols.push_back(curr_index);

        break;

      }

      
      case S_OUT:
      {
        pmt::pmt_t output = pmt::init_u16vector(d_symbols.size(), d_symbols);
        pmt::pmt_t msg_pair = pmt::cons(pmt::make_dict(), output);
        message_port_pub(d_out_port, msg_pair);

        d_state = S_RESET;

        #if DEBUG >= DEBUG_INFO
          std::cout << "Next state: S_RESET" << std::endl;
          //std::cout << "Demodulated Symbols: " << std::endl;
          //print_bitwise_u16(d_symbols);
        #endif

        break;
      }

      default:
        break;

    }


      consume_each (num_consumed);

      return noutput_items;
    }

    unsigned short
    lobee_cpp_bp_impl::chirp_correlation(std::vector<unsigned char> chirp, unsigned short &corr_max, unsigned short &flag)
    {
      unsigned short zero_cnt=0;
      unsigned short one_cnt=0;
      std::vector<unsigned short> corr_zero;
      std::vector<unsigned short> corr_one;
      unsigned short corr_zero_max;
      unsigned short corr_one_max;
      unsigned short corr_zero_index;
      unsigned short corr_one_index;
      unsigned short curr_index=0;

      for(decltype(d_zeros_binary.size()) i=0;i<=(d_zeros_binary.size());i++)
      {
        for(decltype(d_zeros_binary.size()) j=0;j<(d_zeros_binary.size());j++)
        {
          if(chirp[i+j]==d_zeros_binary[j])
          {
            zero_cnt++;
          }
          if(chirp[i+j]==d_ones_binary[j])
          {
            one_cnt++;
          }
        }
        corr_zero.push_back(zero_cnt);
        corr_one.push_back(one_cnt);

        zero_cnt=0;
        one_cnt =0;

      }

      auto corr_zero_iter = std::max_element(corr_zero.begin(), corr_zero.end());
      auto corr_one_iter = std::max_element(corr_one.begin(), corr_one.end());
      corr_zero_max = *corr_zero_iter;
      corr_one_max = *corr_one_iter;

      corr_zero_index = std::distance(corr_zero.begin(), corr_zero_iter);
      corr_one_index = std::distance(corr_one.begin(), corr_one_iter);

      zero_cnt=1;
      one_cnt =1;
      for(decltype(corr_zero.size()) i=corr_zero_index+1;i<(corr_zero.size());i++)
      {
        if(corr_zero[i]==corr_zero_max){
          corr_zero_index+=i;
          zero_cnt++;
        }
      }
      for(decltype(corr_one.size()) i=corr_one_index+1;i<(corr_one.size());i++)
      {
        if(corr_one[i]==corr_one_max){
          corr_one_index+=i;
          one_cnt++;
        }
      }
      
      corr_zero_index = (unsigned short)round((float)corr_zero_index/zero_cnt);
      corr_one_index = (unsigned short)round((float)corr_one_index/one_cnt);
    

      if(corr_zero_max>=corr_one_max){
        curr_index = (unsigned short)round(float(d_quad_len-corr_zero_index)/d_scale);
        corr_max = corr_zero_max;
        flag=0;
      }
      else
      {
        curr_index = (unsigned short)round(float(d_half_quad_len-corr_one_index)/d_scale);
        corr_max = corr_one_max;
        flag=1;
      }
      if(corr_one_max<d_threshold)
      {
        corr_max = corr_one_max;
      }
      return curr_index;
    }

  } /* namespace lobee */
} /* namespace gr */

