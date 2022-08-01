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


#ifndef INCLUDED_LOBEE_LOBEE_CPP_BP_H
#define INCLUDED_LOBEE_LOBEE_CPP_BP_H

#include <lobee/api.h>
#include <gnuradio/block.h>

#define REQUIRED_PREAMBLE_CHIRPS   7
#define REQUIRED_SFD_CHIRPS        2
#define LORA_SFD_TOLERANCE         1
#define LORA_PREAMBLE_TOLERANCE    1

namespace gr {
  namespace lobee {

    enum demod_state_t {
      S_RESET,
      S_PREFILL,
      S_DETECT_PREAMBLE,
      S_SFD_SYNC,
      S_READ_PAYLOAD,
      S_OUT
    };

    /*!
     * \brief <+description of block+>
     * \ingroup lobee
     *
     */
    class LOBEE_API lobee_cpp_bp : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<lobee_cpp_bp> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of lobee::lobee_cpp_bp.
       *
       * To avoid accidental use of raw pointers, lobee::lobee_cpp_bp's
       * constructor is in a private implementation
       * class. lobee::lobee_cpp_bp::make is the public interface for
       * creating new instances.
       */
      static sptr make(unsigned short spreading_factor, unsigned short scale, unsigned short threshold);
    };

  } // namespace lobee
} // namespace gr

#endif /* INCLUDED_LOBEE_LOBEE_CPP_BP_H */

