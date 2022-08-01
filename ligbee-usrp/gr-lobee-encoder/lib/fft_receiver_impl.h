#ifndef INCLUDED_LORA_FFT_RECEIVER_IMPL_H
#define INCLUDED_LORA_FFT_RECEIVER_IMPL_H

#include <liquid/liquid.h>
#include <lora/fft_receiver.h>
#include <vector>
#include <fstream>
#include <lora/debugger.h>
#include <volk/volk.h>
#include <lora/loraphy.h>
#include <boost/circular_buffer.hpp>


namespace gr {
  namespace lora {

    /**
     *  \brief  **DecoderState** : Each state the FFT decoder can be in.
     */
    enum class DecoderState {
        RESET,
        DETECT,
        SYNC,
        FIND_SFD,
        PAUSE,
        DECODE_HEADER,
        DECODE_PAYLOAD,
        STOP
    };

    class fft_receiver_impl : public fft_receiver
    {
     private:
      debugger                d_dbg;              ///< Debugger for plotting samples, printing output, etc.
      DecoderState            d_state;            ///< Holds the current state of the decoder (state machine).

      std::vector<gr_complex> d_downchirp;        ///< The complex ideal downchirp.
      std::vector<gr_complex> d_downchirp_zero;   ///< The ideal downchirp begin from frequency 0. 
      std::vector<float>      d_downchirp_ifreq;  ///< The instantaneous frequency of the ideal downchirp.
      std::vector<gr_complex> d_upchirp;          ///< The complex ideal upchirp.
      std::vector<gr_complex> d_upchirp_zero;     ///< The ideal upchirp begin from frequency 0.
      std::vector<float>      d_upchirp_ifreq;    ///< The instantaneous frequency of the ideal upchirp.

      uint32_t                   d_preamble_idx;             ///< Preamble max fft bin.
      uint32_t                   d_sync_idx;                 ///< Synced preamble max fft bin.
      uint32_t                   d_window;                   ///< FFT window.
      std::vector<uint32_t>      d_idx_history;              ///< Stack of preamble max fft bin index.
      float                      d_bin_threshold;            ///< Least fft energy threshold in peak bin.
      float                      d_energy_threshold;         ///< The absolute threshold to distinguish signal from noise.
      int32_t                    d_first_syncword;           ///< First sync word before fine sync.
      bool                       d_enable_fine_sync;         ///< Enable drift correction
      int32_t                    d_fine_sync;                ///< Amount of drift correction to apply for next symbol 
      int32_t                    d_syncword[4]={24,32,0,0};  ///< Sync word 0x34
      int32_t                    d_sync_shift;               ///< Downchirp position shift
      uint32_t                   d_sync_fails;               ///< Indicates how many times the sync process failed. After some tries, the state will revert to `DecoderState::RESET`.
      int32_t                    d_sync_bgn;                 ///< Fine sync begin index;
      int32_t                    d_sync_end;                 ///< Fine sync end index; 
      int32_t                    d_step;                     ///< Step length   

      uint32_t                d_curr_idx;         ///< Current max fft bin index;
      uint32_t                d_curr_symbol;      ///< Current symbol index;
      float                   d_curr_bin;         ///< Current fft energy in peak bin;
      float                   d_curr_energy;      ///< Current average signal energy;
      int32_t                 d_idx_error;        ///< Header index_error;
      int32_t                 d_num_header;       ///< The number of header symbols;   

      bool             d_implicit;                ///< Implicit header mode.
      bool             d_reduced_rate;            ///< Use reduced rate (only configurable in implicit header mode).
      uint8_t          d_sf;                      ///< The Spreading Factor.
      uint32_t         d_bw;                      ///< The receiver bandwidth (fixed to `125kHz`).
      loraphy_header_t d_phdr;                    ///< LoRa PHY header.
      uint16_t         d_mac_crc;                 ///< The MAC CRC.
      double           d_bits_per_second;         ///< Indicator of how many bits are transferred each second.
      uint32_t         d_delay_after_sync;        ///< The number of samples to skip in `DecoderState::PAUSE`.
      uint32_t         d_samples_per_second;      ///< The number of samples taken per second by GNU Radio.
      double           d_symbols_per_second;      ///< Indicator of how many symbols (read: chirps) are transferred each second.
      double           d_bits_per_symbol;         ///< The number of bits each of the symbols contain.
      uint32_t         d_samples_per_symbol;      ///< The number of samples in one symbol.
      uint32_t         d_halfsamp_per_symbol;     ///< Half of the number of samples in one symbol.
      uint32_t         d_quadsamp_per_symbol;     ///< Quad of the number of samples in one symbol.
      double           d_period;                  ///< Period of the symbol.
      uint32_t         d_number_of_bins;          ///< Indicates in how many parts or bins a symbol is decimated, i.e. the max value to decode out of one payload symbol.
      uint32_t         d_halfnum_of_bins;         ///< Half of the number of bins.
      uint32_t         d_number_of_bins_hdr;      ///< Indicates in how many parts or bins a HDR symbol is decimated, i.e. the max value to decode out of one HDR symbol.
       int32_t         d_payload_symbols;         ///< The number of symbols needed to decode the payload. Calculated from an indicator in the HDR.
      uint32_t         d_payload_length;          ///< The number of words after decoding the HDR or payload. Calculated from an indicator in the HDR.
      const uint8_t*   d_whitening_sequence;      ///< A pointer to the whitening sequence to be used in decoding. Determined by the SF in the ctor.

      

      float            d_snr;                     ///< Signal to noise ratio
      float            d_pwr_signal;              ///< Signal pow
      float            d_pwr_noise;               ///< Noise pow
      std::vector<float> d_pow_signal;            ///< Save curr signal pow
      std::vector<float> d_pow_noise;             ///< Save curr noise pow
      std::vector<float> d_snr_pow;               ///< Save curr snr in pow
      std::vector<unsigned char>    d_snr_db;     ///< Save curr snr in db
      boost::circular_buffer<float> d_pwr_queue;  ///< Queue holding symbol power values
      boost::circular_buffer<float> d_bin_queue;  ///< Stack of preamble max fft bin.
      uint8_t                       d_packet_len; ///< Know packet length to abandon packets with header error (Fail of Header CRC)

      std::vector<uint32_t> d_words;              ///< Vector containing the demodulated words.
      std::vector<uint8_t>  d_demodulated;        ///< Vector containing the words after deinterleaving.
      std::vector<uint8_t>  d_words_deshuffled;   ///< Vector containing the words after deshuffling.
      std::vector<uint8_t>  d_words_dewhitened;   ///< Vector containing the words after dewhitening.
      std::vector<uint8_t>  d_decoded;            ///< Vector containing the words after Hamming decode or the final decoded words.
      
      std::string             d_path;             ///< File save path
      uint32_t                d_packet_cnt;       ///< Index for packet
      std::vector<gr_complex> d_data_shr;         ///< Save packet samples with sync header 
      std::vector<gr_complex> d_data;             ///< Save packet samples
      std::vector<gr_complex> d_data_mult;        ///< Save packet samples after multiplying downchirp
      std::vector<gr_complex> d_data_fft;         ///< Save fft result
      std::vector<gr_complex> d_data_decim_fft;   ///< Save fft bins after decimator(previous and back half bins, abandon middle bins)

      std::ofstream d_debug_samples;              ///< Debug utputstream for complex values.
      std::ofstream d_debug;                      ///< Outputstream for the debug log.

      std::vector<gr_complex> d_fft;              ///< Vector containing the FFT resuls.
      std::vector<gr_complex> d_mult_hf;          ///< Vector containing the multiplication.
      std::vector<gr_complex> d_tmp;              ///< Vector containing the FFT decimation.
      std::vector<gr_complex> d_positive;        ///< Vector containing positive frequency bin.
      std::vector<gr_complex> d_negative;         ///< Vector containing negative frequency bin.
      fftplan d_q;                                ///< The LiquidDSP::FFT_Plan.
      fftplan d_qr;                               ///< The LiquidDSP::FFT_Plan in reverse.
      fec     d_h48_fec;                          ///< LiquidDSP Hamming 4/8 FEC.

      uint32_t      d_decim_factor;               ///< The number of samples (data points) in each bin.
      float         d_cfo_estimation;             ///< An estimation for the current Center Frequency Offset.
      double        d_dt;                         ///< Indicates how fast the frequency changes in a symbol (chirp).     

      /**
       * \brief TODO: Determin the spreading factor automatically!  
       */
      uint8_t detect_sf_autocorr(const gr_complex *samples, uint32_t window);        

      /**
       *  \brief  TODO
       */
      float experimental_determine_cfo(const gr_complex *samples, uint32_t window);

      /**
       *  \brief  Generate the ideal up- and downchirps.
       */
      void build_ideal_chirps(void);

      /**
        *  \brief  Debug method to dump the given values array to a file in textual format.
        *
        *  \param  path
        *          The path to the file to dump to.
        *  \param  v
        *          The values array to dump.
        *  \param  length
        *          Length of said array.
        *  \param  ppm
        *          PPM value of the data.
        */
      void values_to_file(const std::string path, const unsigned char *v, const uint32_t length, const uint32_t ppm);

      /**
       *  \brief  Debug method to dump the given complex array to the given file in binary format.
       *
       *  \param  path
       *          The path to the file to dump to.
       *  \param  v
       *          The complex array to dump.
       *  \param  length
       *          Length of said array.
       *  \param  elem_size
       *          `sizeof` the data in the array.
       */
      void samples_to_file(const std::string path, const gr_complex *v, const uint32_t length, const uint32_t elem_size);


      /**
       *  \brief  Debug method to dump the given byte array to the given file in binary format.
       *
       *  \param  path
       *          The path to the file to dump to.
       *  \param  v
       *          The bytes array to dump.
       *  \param  length
       *          Length of said array.
       *  \param  elem_size
       *          `sizeof` the data in the array.
       */
      void bytes_to_file(const std::string path, const unsigned char *v, const uint32_t length, const uint32_t elem_size, bool app);

      /**
       *  \brief  Debug method to dump the given float array to the given file in binary format.
       *
       *  \param  path
       *          The path to the file to dump to.
       *  \param  v
       *          The floats array to dump.
       *  \param  length
       *          Length of said array.
       *  \param  elem_size
       *          `sizeof` the data in the array.
       */
      void floats_to_file(const std::string path, const float *v, const uint32_t length, const uint32_t elem_size, bool app);

      /**
       *  \brief  Write the given complex array to the debug outputstream.
       *
       *  \param  v
       *          The complex array.
       *  \param  length
       *          Length of said complex array.
       */
      void samples_debug(const gr_complex *v, const uint32_t length);
      
      /**
       *  \brief  Write the lora sync header to the debug outputstream.
       *
       *  \param  v
       *          Pointer of the beginning of SHR.
       *  \param  pre_len
       *          The number of saved preambles in SHR.
       */      
      void save_shr(const gr_complex *v, const uint32_t pre_len);

      /**
       * \brief Detect the first sync word!
       */
      bool detect_first_syncword(const gr_complex *samples, int32_t &first_syncword);

      /**
       * \brief Detect sync word and fine sync with preamble!
       */
      bool detect_syncword(const gr_complex *samples);    

      /**
       *  \brief  Returns the index of the max frequency bin
       *
       *  \param  samples
       *          The complex samples array to analyze.
       *  \param  curr_bin
       *          Return max fft bin values by reference
       *  \param  up
       *          upchirp or downchirp of the received signal
       */      
      uint32_t get_max_freq_bin(const gr_complex *samples, float &curr_bin, bool up);

      /**
       *  \brief  Returns the index of the max fft bin
       *
       *  \param  samples
       *          The complex samples array to analyze.
       */
      uint32_t get_max_fft_bin(const gr_complex *samples, float &curr_bin);

      /**
       *  \brief  Determine the energy of a symbol.
       *
       *  \param  samples
       *          The complex symbol to analyse.
       */
      float determine_energy(const gr_complex *samples);

      /**
       *  \brief  Determine the SNR
       */
      void determine_snr();      

      /**
       *  \brief  Demodulate the given symbol and return true if all expected symbols have been parsed.
       *
       *  \param  samples
       *          The samples to demodulate.
       *  \param  is_header
       *          Whether the demodulated words were from the HDR.
       */
      bool demodulate(const gr_complex *samples, const bool reduced_rate);

      /**
       *  \brief  Deinterleave the raw demodulated words by reversing the interleave pattern.
       *
       *  \param  ppm
       *          The number of words that zere interleaved. Depends on `SF`.
       */
      void deinterleave(const uint32_t ppm);

      /**
       *  \brief  The process of decoding the demodulated words to get the actual payload.
       *          <br/>1. Deshuffle the words
       *          <br/>2. Dewhiten the words
       *          <br/>3. Hamming decoding
       *
       *  \param  is_header
       *          Whether the demodulated words were from the HDR.
       */
      void decode(const bool is_header);

      /**
       *  \brief  Deshuffle the demodulated words by the given pattern.
       *
       *  \param  shuffle_pattern
       *          The order in which the bits appear.
       *  \param  is_header
       *          Whether the demodulated words were from the HDR.
       */
      void deshuffle(const uint8_t *shuffle_pattern, const bool is_header);

      /**
       *  \brief  Dewhiten the deshuffled words by XORing with the whitening sequence.
       *
       *  \param  prng
       *          The whitening sequence to XOR with.
       */
      void dewhiten(const uint8_t *prng);

      /**
       *  \brief  Use Hamming to decode the dewhitened words.
       *          <br/>- CR 4 or 3: Hamming(8,4) or Hamming(7,4) with parity correction
       *          <br/>- CR 2 or 1: Extract data only (can only find parity errors, not correct them)
       *
       *  \param  is_header
       *          Decoding for the header?
       */
      void hamming_decode(bool is_header);

      /**
       *  \brief  Hamming(8,4) decoding by calling `hamming_decode_soft_byte` on each byte.
       *          <BR>Each byte is decoded in pairs, the first one becoming the LSB nibble
       *          <BR>and the second one the MSB nibble (if even; else just zeroes).
       *
       *  \param  is_header
       *          Decoding for the header?
       */
      void hamming_decode_soft(bool is_header);

      /**
       *  \brief  Extract only the data in the given bytes.
       *
       *  \param  is_header
       *          Decoding for the header?
       */
      void extract_data_only(bool is_header);

      /**
       *  \brief  Pub the received frame to next block.
       */
      void msg_lora_frame(void); 

      /**
       *  \brief  Calculate the instantaneous phase for the given complex symbol.
       *
       *  \param  in_samples
       *          The complex array to calculate the instantaneous phase for.
       *  \param  out_iphase
       *          The output `float` array containing the instantaneous phase.
       *  \param  window
       *          The size of said arrays.
       */
      inline void instantaneous_phase(const gr_complex *in_samples, float *out_iphase, const uint32_t window);

      /**
       *  \brief  Calculate the instantaneous frequency for the given complex symbol.
       *
       *  \param  in_samples
       *          The complex array to calculate the instantaneous frequency for.
       *  \param  out_ifreq
       *          The output `float` array containing the instantaneous frequency.
       *  \param  window
       *          The size of said arrays.
       */
      inline void instantaneous_frequency(const gr_complex *in_samples, float *out_ifreq, const uint32_t window);

     public:
      
      /**
       *  \brief  Default constructor.
       *
       *  \param  samp_rate
       *          The sample rate of the input signal given to `work` later.
       *  \param  sf
       *          The expected spreqding factor.
       */      
      fft_receiver_impl(float samp_rate, uint32_t bandwidth, uint8_t sf, bool implicit, uint8_t cr, bool crc, bool reduced_rate, int32_t sync_shift, uint8_t packet_len, std::string path);
      
      /**
       *  Default destructor.
       */ 
      ~fft_receiver_impl();

      /**
      *   \brief  The main method called by GNU Radio to perform tasks on the given input.
      *
      *   \param  noutput_items
      *           The requested amoutn of output items.
      *   \param  input_items
      *           An array with samples to process.
      *   \param  output_items
      *           An array to return processed samples.
      *   \return Returns the number of output items generated.
      */
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);


      /**
       *  \brief  Set th current spreading factor.
       *          <br/>**Currently not supported, restart GNU Radio with different settings instead.**
       *  \param  sf
       *          The new spreading factor.
       */
      virtual void set_sf(const uint8_t sf);

      /**
       *  \brief  Set the current sample rate.
       *          <br/>**Currently not supported, restart GNU Radio with different settings instead.**
       *
       *  \param  samp_rate
       *          The new sample rate.
       */
      virtual void set_samp_rate(const float samp_rate);

    };

  } // namespace lora
} // namespace gr

#endif /* INCLUDED_LORA_FFT_RECEIVER_IMPL_H */

