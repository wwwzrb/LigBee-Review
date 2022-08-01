#ifndef INCLUDED_LORA_FFT_RECEIVER_H
#define INCLUDED_LORA_FFT_RECEIVER_H

#include <lora/api.h>
#include <gnuradio/sync_block.h>
#include <string>

#define FFT_REQUIRED_PREAMBLE_CHIRPS   4   // Required preamble chirps.
#define FFT_PREAMBLE_TOLERANCE         2   // Error tolerance of preamble sync.
#define FFT_SYNC_TOLERANCE             2   // Error tolerance after preamble sync.
#define FFT_SYNC_ATTEMPTS              12  // Attempts to detect first syncword according to required peamble chirps.
#define FFT_REQUIRED_SYNC_WORDS        2   // Required sync words in first detection.
#define FFT_REQUIRED_SFD_CHIRPS        2   // Required chirps in fine synchronization.
#define FFT_SFD_COARSE                 8   // To detect first syncword.
#define FFT_SFD_FINE                   4   // To synchronize with syncword.
#define FFT_SFD_TOTAL                  10   // Total errors tolerance in syncword.
#define FFT_PWR_QUEUE_SIZE             8   // Power queue length.

namespace gr {
  namespace lora {

    /*!
     * \brief <+description of block+>
     * \ingroup lora
     *
     */
    class LORA_API fft_receiver : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<fft_receiver> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of lora::fft_receiver.
       *
       * To avoid accidental use of raw pointers, lora::fft_receiver's
       * constructor is in a private implementation
       * class. lora::fft_receiver::make is the public interface for
       * creating new instances.
       */
      static sptr make(float samp_rate, uint32_t bandwidth, uint8_t sf, bool implicit, uint8_t cr, bool crc, bool reduced_rate, int32_t sync_shift, uint8_t packet_len, std::string path);

      virtual void set_sf(uint8_t sf) = 0;
      virtual void set_samp_rate(float samp_rate) = 0;

    };

  } // namespace lora
} // namespace gr

#endif /* INCLUDED_LORA_FFT_RECEIVER_H */

