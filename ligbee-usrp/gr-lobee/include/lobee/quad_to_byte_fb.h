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


#ifndef INCLUDED_LOBEE_QUAD_TO_BYTE_FB_H
#define INCLUDED_LOBEE_QUAD_TO_BYTE_FB_H

#include <lobee/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace lobee {

    /*!
     * \brief <+description of block+>
     * \ingroup lobee
     *
     */
    class LOBEE_API quad_to_byte_fb : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<quad_to_byte_fb> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of lobee::quad_to_byte_fb.
       *
       * To avoid accidental use of raw pointers, lobee::quad_to_byte_fb's
       * constructor is in a private implementation
       * class. lobee::quad_to_byte_fb::make is the public interface for
       * creating new instances.
       */
      static sptr make();
    };

  } // namespace lobee
} // namespace gr

#endif /* INCLUDED_LOBEE_QUAD_TO_BYTE_FB_H */

