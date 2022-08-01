/* -*- c++ -*- */

#ifndef INCLUDED_LORA_LOBEE_RECEIVER_H
#define INCLUDED_LORA_LOBEE_RECEIVER_H

#include <lora/api.h>
#include <gnuradio/sync_block.h>
#include <string>

#define REQUIRED_PREAMBLE_CHIRPS   6
#define LORA_PREAMBLE_TOLERANCE    2
#define REQUIRED_SYNC_WORDS        2
#define REQUIRED_SFD_CHIRPS        2
#define LORA_SFD_COARSE            2
#define LORA_SFD_TOLERANCE         4
#define LORA_SFD_ERROR             10

namespace gr {
  namespace lora {

    /*!
     * \brief <+description of block+>
     * \ingroup lora
     *
     */
    class LORA_API lobee_receiver : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<lobee_receiver> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of lora::lobee_receiver.
       *
       * To avoid accidental use of raw pointers, lora::lobee_receiver's
       * constructor is in a private implementation
       * class. lora::lobee_receiver::make is the public interface for
       * creating new instances.
       */
      static sptr make(float samp_rate, uint32_t bandwidth, uint8_t sf, bool implicit, uint8_t cr, bool crc, bool reduced_rate, bool disable_drift_correction, int32_t sync_shift, uint8_t packet_len, std::string path);

      virtual void set_sf(uint8_t sf) = 0;
      virtual void set_samp_rate(float samp_rate) = 0;
    };

  } // namespace lora
} // namespace gr

#endif /* INCLUDED_LORA_LOBEE_RECEIVER_H */

