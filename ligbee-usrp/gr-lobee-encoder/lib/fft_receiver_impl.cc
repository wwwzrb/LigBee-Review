#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "fft_receiver_impl.h"
#include <gnuradio/expj.h>
#include <liquid/liquid.h>
#include <numeric>
#include <algorithm>
#include <lora/loratap.h>
#include <lora/utilities.h>
#include "tables.h"

#include "dbugr.hpp"

#define FFT_DEBUG_OFF     1
#define FFT_DEBUG_INFO    1
#define FFT_DEBUG_VERBOSE 2
#define FFT_DEBUG_ON      FFT_DEBUG_OFF

#define FFT_INDEX_CORRECTION 1

namespace gr {
  namespace lora {

    fft_receiver::sptr
    fft_receiver::make(float samp_rate, uint32_t bandwidth, uint8_t sf, bool implicit, uint8_t cr, bool crc, bool reduced_rate, int32_t sync_shift, uint8_t packet_len, std::string path)
    {
      return gnuradio::get_initial_sptr
        (new fft_receiver_impl(samp_rate, bandwidth, sf, implicit, cr, crc, reduced_rate, sync_shift, packet_len, path));
    }

    /*
     * The private constructor
     */
    fft_receiver_impl::fft_receiver_impl(float samp_rate, uint32_t bandwidth, uint8_t sf, bool implicit, uint8_t cr, bool crc, bool reduced_rate, int32_t sync_shift, uint8_t packet_len, std::string path)
      : gr::sync_block("fft_receiver",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(0, 0, 0)),
      d_pwr_queue(FFT_PWR_QUEUE_SIZE),
      d_bin_queue(FFT_PWR_QUEUE_SIZE),
      d_packet_len(packet_len),
      d_path(path)
    {
      // Raido config
      d_state = gr::lora::DecoderState::DETECT;

      if (sf < 6 || sf > 13) {
          std::cerr << "[LoRa Decoder] ERROR : Spreading factor should be between 6 and 12 (inclusive)!" << std::endl
                    << "                       Other values are currently not supported." << std::endl;
          exit(1);
      }

      #if FFT_DEBUG_ON
          d_debug_samples.open(d_path+"/data/grfft_debug", std::ios::out | std::ios::binary);
          d_debug.open(d_path+"/data/grfft_debug_txt", std::ios::out);
          d_dbg.attach();
      #endif    

      d_bw                 = bandwidth;
      d_implicit           = implicit;
      d_reduced_rate       = reduced_rate;
      d_phdr.cr            = cr;
      d_phdr.has_mac_crc   = crc;
      d_samples_per_second = samp_rate;
      d_payload_symbols    = 0;
      d_cfo_estimation     = 0.0f;
      d_dt                 = 1.0f / d_samples_per_second;
      d_sf                 = sf;
      d_bits_per_second    = (double)d_sf * (double)(4.0 / (4.0 + d_phdr.cr)) / (1u << d_sf) * d_bw;
      d_symbols_per_second = (double)d_bw / (1u << d_sf);
      d_period             = 1.0f / (double)d_symbols_per_second;
      d_bits_per_symbol    = (double)(d_bits_per_second    / d_symbols_per_second);
      d_samples_per_symbol = (uint32_t)(d_samples_per_second / d_symbols_per_second);
      d_halfsamp_per_symbol= (uint32_t)(d_samples_per_symbol / 2);
      d_quadsamp_per_symbol= (uint32_t)(d_samples_per_symbol / 4);
      d_delay_after_sync   = d_samples_per_symbol / 4u;
      d_number_of_bins     = (uint32_t)(1u << d_sf);
      d_halfnum_of_bins    = (uint32_t)(d_number_of_bins/2);  
      d_number_of_bins_hdr = (uint32_t)(1u << (d_sf-2));
      d_decim_factor       = d_samples_per_symbol / d_number_of_bins;
      d_energy_threshold   = 0.0;
      d_fine_sync          = 0;
      d_packet_cnt         = 0;

      d_window             = d_samples_per_symbol;
      d_bin_threshold      = 0.0;
      d_curr_bin           = 0.0;

    //   // get_max_fft_bin max_frequency_gradient_idx max_corrleation_idx 
    //   // detect max fft bin at index=d_halfnum_of_bins for downchirp
    //   // get_max_freq_bin detect max fft bin at index=64 for downchirp multiplying upchirp_zero
      d_syncword[2]        = d_halfnum_of_bins;
      d_syncword[3]        = d_syncword[2]; 
    
      d_sync_idx           = 0;
      d_sync_end           = 0;
      d_sync_shift         = sync_shift;

      // FFT_INDEX_CORRECTION
      d_idx_error          = 0;
      d_curr_idx           = 0;
      d_curr_symbol        = 0;
      d_num_header         = 0;

      set_output_multiple(16*d_samples_per_symbol);

      std::cout << "Bits (nominal) per symbol: \t"      << d_bits_per_symbol    << std::endl;
      std::cout << "Bins per symbol: \t"      << d_number_of_bins     << std::endl;
      std::cout << "Samples per symbol: \t"   << d_samples_per_symbol << std::endl;
      std::cout << "Decimation: \t\t"         << d_decim_factor       << std::endl;

      if(d_implicit) {
          std::cout << "CR: \t\t"         << (int)d_phdr.cr       << std::endl;
          std::cout << "CRC: \t\t"         << (int)d_phdr.has_mac_crc       << std::endl;
      }      

      // Locally generated chirps
      build_ideal_chirps();

      // FFT decoding preparations
      d_fft.resize(d_samples_per_symbol);
      d_mult_hf.resize(d_samples_per_symbol);
      d_tmp.resize(d_number_of_bins);
      d_positive.resize(d_number_of_bins);
      d_negative.resize(d_number_of_bins);
      d_q  = fft_create_plan(d_samples_per_symbol, &d_mult_hf[0], &d_fft[0],     LIQUID_FFT_FORWARD, 0);
      d_qr = fft_create_plan(d_number_of_bins,     &d_tmp[0],     &d_mult_hf[0], LIQUID_FFT_BACKWARD, 0);

      // Hamming coding
      fec_scheme fs = LIQUID_FEC_HAMMING84;
      d_h48_fec = fec_create(fs, NULL);

      // Register gnuradio ports
      message_port_register_out(pmt::mp("frames"));        
    }

    /*
     * Our virtual destructor.
     */
    fft_receiver_impl::~fft_receiver_impl()
    {
      #if FFT_DEBUG_ON
          if (d_debug_samples.is_open())
              d_debug_samples.close();

          if (d_debug.is_open())
              d_debug.close();
      #endif

      fft_destroy_plan(d_q);
      fft_destroy_plan(d_qr);
      fec_destroy(d_h48_fec);
    }

    uint8_t fft_receiver_impl::detect_sf_autocorr(const gr_complex *samples, uint32_t window)
    {
        return 7;
    }

    /**
     * New method to determine CFO.
     */
    float fft_receiver_impl::experimental_determine_cfo(const gr_complex *samples, uint32_t window) {
        gr_complex mult[window];
        float mult_ifreq[window];

        volk_32fc_x2_multiply_32fc(mult, samples, &d_downchirp[0], window);
        instantaneous_frequency(mult, mult_ifreq, window);

        float sum=0;
        for(uint32_t i=0;i<window;i++){
            sum+=mult_ifreq[i];
        }
        //return mult_ifreq[256] / (2.0 * M_PI) * d_samples_per_second;
        return sum/window;
    }

    void fft_receiver_impl::build_ideal_chirps(void) {
        d_downchirp.resize(d_samples_per_symbol);
        d_downchirp_zero.resize(d_samples_per_symbol);
        d_upchirp.resize(d_samples_per_symbol);
        d_upchirp_zero.resize(d_samples_per_symbol);
        d_downchirp_ifreq.resize(d_samples_per_symbol);
        d_upchirp_ifreq.resize(d_samples_per_symbol);

        const double T       = -0.5 * d_bw * d_symbols_per_second;
        const double f0      = (d_bw / 2.0);
        const double pre_dir = 2.0 * M_PI;
        double t;
        gr_complex cmx       = gr_complex(1.0f, 1.0f);

        for (uint32_t i = 0u; i < d_samples_per_symbol; i++) {
            // Width in number of samples = samples_per_symbol
            // See https://en.wikipedia.org/wiki/Chirp#Linear
            t = d_dt * i;
            d_downchirp[i] = cmx * gr_expj(pre_dir * t * (f0 + T * t));
            d_upchirp[i]   = cmx * gr_expj(pre_dir * t * (f0 + T * t) * -1.0f);
        }

        // Generating ideal down/upchirp begin from frequency 0.
        memcpy(&d_upchirp_zero[0], &d_upchirp[d_halfsamp_per_symbol], d_halfsamp_per_symbol*sizeof(gr_complex));
        memcpy(&d_upchirp_zero[d_halfsamp_per_symbol], &d_upchirp[0], d_halfsamp_per_symbol*sizeof(gr_complex));

        memcpy(&d_downchirp_zero[0], &d_downchirp[d_halfsamp_per_symbol], d_halfsamp_per_symbol*sizeof(gr_complex));
        memcpy(&d_downchirp_zero[d_halfsamp_per_symbol], &d_downchirp[0], d_halfsamp_per_symbol*sizeof(gr_complex));

        // Store instantaneous frequency
        instantaneous_frequency(&d_downchirp[0], &d_downchirp_ifreq[0], d_samples_per_symbol);
        instantaneous_frequency(&d_upchirp[0],   &d_upchirp_ifreq[0],   d_samples_per_symbol);

        samples_to_file(d_path+"/data/downchirp", &d_downchirp[0], d_downchirp.size(), sizeof(gr_complex));
        samples_to_file(d_path+"/data/upchirp",   &d_upchirp[0],   d_upchirp.size(),   sizeof(gr_complex));
    }

    void fft_receiver_impl::values_to_file(const std::string path, const unsigned char *v, const uint32_t length, const uint32_t ppm) {
        std::ofstream out_file;
        out_file.open(path.c_str(), std::ios::out | std::ios::app);

        for (uint32_t i = 0u; i < length; i++) {
            std::string tmp = gr::lora::to_bin(v[i], ppm);
            out_file.write(tmp.c_str(), tmp.length());
            out_file.write(" ", 1);
        }
        out_file.write("\n", 1);

        out_file.close();
    }

    void fft_receiver_impl::samples_to_file(const std::string path, const gr_complex *v, const uint32_t length, const uint32_t elem_size) {
        #if FFT_DEBUG_ON
            std::ofstream out_file;
            out_file.open(path.c_str(), std::ios::out | std::ios::binary);

            //for(std::vector<gr_complex>::const_iterator it = v.begin(); it != v.end(); ++it) {
            for (uint32_t i = 0u; i < length; i++) {
                out_file.write(reinterpret_cast<const char *>(&v[i]), elem_size);
            }

            out_file.close();
        #else
            (void) path;
            (void) v;
            (void) length;
            (void) elem_size;
        #endif
    }

    void fft_receiver_impl::bytes_to_file(const std::string path, const unsigned char *v, const uint32_t length, const uint32_t elem_size, bool app) {
        #if FFT_DEBUG_ON
            std::ofstream out_file;

            if(app){
                out_file.open(path.c_str(),std::ios::out|std::ios::binary|std::ios::app);
            }
            else{
                out_file.open(path.c_str(), std::ios::out | std::ios::binary);
            }

            //for(std::vector<gr_complex>::const_iterator it = v.begin(); it != v.end(); ++it) {
            for (uint32_t i = 0u; i < length; i++) {
                out_file.write(reinterpret_cast<const char *>(&v[i]), elem_size);
            }

            out_file.close();
        #else
            (void) path;
            (void) v;
            (void) length;
            (void) elem_size;
        #endif
    }

    void fft_receiver_impl::floats_to_file(const std::string path, const float *v, const uint32_t length, const uint32_t elem_size, bool app){
        #if FFT_DEBUG_ON
            std::ofstream out_file;

            if(app){
                out_file.open(path.c_str(),std::ios::out|std::ios::binary|std::ios::app);
            }
            else{
                out_file.open(path.c_str(), std::ios::out | std::ios::binary);
            }

            //for(std::vector<gr_complex>::const_iterator it = v.begin(); it != v.end(); ++it) {
            for (uint32_t i = 0u; i < length; i++) {
                out_file.write(reinterpret_cast<const char *>(&v[i]), elem_size);
            }

            out_file.close();
        #else
            (void) path;
            (void) v;
            (void) length;
            (void) elem_size;
        #endif
    }

    void fft_receiver_impl::samples_debug(const gr_complex *v, const uint32_t length) {
        #if FFT_DEBUG_ON
            gr_complex start_indicator(0.0f, 32.0f);
            d_debug_samples.write(reinterpret_cast<const char *>(&start_indicator), sizeof(gr_complex));

            for (uint32_t i = 1u; i < length; i++) {
                d_debug_samples.write(reinterpret_cast<const char *>(&v[i]), sizeof(gr_complex));
            }
        #else
            (void) v;
            (void) length;
        #endif
    }

    void fft_receiver_impl::save_shr(const gr_complex *v, const uint32_t pre_len){
        uint32_t shr_length=(pre_len+4)*d_samples_per_symbol + d_quadsamp_per_symbol;

        std::copy(v, v+shr_length, std::back_inserter(d_data_shr));
    }

    bool fft_receiver_impl::detect_first_syncword(const gr_complex *samples, int32_t &first_syncword){
      uint32_t chirp_index[FFT_REQUIRED_SYNC_WORDS];

      for(int32_t i=0;i<FFT_REQUIRED_SYNC_WORDS;i++){
        chirp_index[i]=get_max_fft_bin(samples+i*d_samples_per_symbol, d_curr_bin);
      }

      #if FFT_DEBUG_ON
          for(int32_t i=0;i<FFT_REQUIRED_SYNC_WORDS;i++){
            d_debug << " chirp_index[" << i << "]: "<< chirp_index[i];
          }
          d_debug << "; d_fine_sync: " << d_fine_sync << "; d_sync_fails: " << d_sync_fails << std::endl;
      #endif

      if(abs((int)chirp_index[0]-d_syncword[0])<=FFT_SFD_COARSE&&abs((int)chirp_index[1]-d_syncword[1])<=FFT_SFD_COARSE){
          first_syncword=chirp_index[0];
          d_fine_sync=0;
          return true;
      }
      else{
          return false;   
      }  

    }

    bool fft_receiver_impl::detect_syncword(const gr_complex *samples){
      uint32_t chirp_index[FFT_REQUIRED_SFD_CHIRPS];
      uint32_t up_chirp_index[FFT_REQUIRED_SFD_CHIRPS];

      int32_t sfd_error=0;
      bool sfd_found=true;
      for(int32_t i=0;i<FFT_REQUIRED_SFD_CHIRPS;i++){
          chirp_index[i]=get_max_freq_bin(samples+i*d_samples_per_symbol, d_curr_bin,false);
          up_chirp_index[i]=get_max_fft_bin(samples+i*d_samples_per_symbol, d_curr_bin);
          #if FFT_DEBUG_ON
              d_debug << "down_chirp_index" << i << ": " << chirp_index[i] << std::endl;
            //   d_debug << "  up_chirp_index" << i << ": " << up_chirp_index[i] << std::endl; 
          #endif
          if(abs((int)chirp_index[i]-d_syncword[i+FFT_REQUIRED_SYNC_WORDS])>d_sync_shift){
              sfd_error=0;
              sfd_found=false;
              break;
          }
          else{
              sfd_error+=abs((int)chirp_index[i]-d_syncword[i+FFT_REQUIRED_SYNC_WORDS]);
          }
      }       
      if(sfd_found&&sfd_error<=2*d_sync_shift){
          return true;
      }
      else{
          return false;
      }      
    }

    uint32_t fft_receiver_impl::get_max_freq_bin(const gr_complex *samples, float &curr_bin, bool up){
      float fft_mag[d_number_of_bins];

      //samples_to_file(d_path+"/data/data", &samples[0], d_samples_per_symbol, sizeof(gr_complex));

      // Multiply with ideal downchirp
      for (uint32_t i = 0u; i < d_samples_per_symbol; i++) {
          if(up){
            d_mult_hf[i] = samples[i] * d_downchirp_zero[i];
          }
          else{
            d_mult_hf[i] = samples[i] * d_upchirp_zero[i];              
          }
      }

      // Perform FFT
      fft_execute(d_q);

      // Decimate. Note: assumes fft size is multiple of decimation factor and number of bins is even
      // This decimation should be identical to numpy's approach
    //   const uint32_t N = d_number_of_bins;
    //   memcpy(&d_tmp[0],               &d_fft[0],                                     (N + 1u) / 2u * sizeof(gr_complex));
    //   memcpy(&d_tmp[ (N + 1u) / 2u ], &d_fft[d_samples_per_symbol - (N / 2u)],        N / 2u * sizeof(gr_complex));
    //   d_tmp[N / 2u] += d_fft[N / 2u];
      
      // Get postive and negative frequency bins.
      const uint32_t N = d_number_of_bins;
      memcpy(&d_positive[0], &d_fft[0],                      N*sizeof(gr_complex));
      memcpy(&d_negative[0], &d_fft[d_samples_per_symbol-N], N*sizeof(gr_complex));

      // Get magnitude
      for (uint32_t i = 0u; i < d_number_of_bins; i++) {
          d_tmp[i] = d_positive[i] + d_negative[i];
          fft_mag[i] = std::abs(d_tmp[i]);
      }

      uint32_t curr_idx = 0;
      curr_idx = std::max_element(fft_mag, fft_mag + d_number_of_bins) - fft_mag;
      curr_bin = fft_mag[curr_idx];

      #if FFT_DEBUG_ON
          //std::cout << std::endl;
           if(d_state!=gr::lora::DecoderState::DETECT){
                  d_debug << "O: " << " curr_bin= " << curr_bin << "; curr_index: " << curr_idx << std::endl;
           }
          //std::cout << std::endl;
      #endif

      int32_t idx_error = 0;
      #if FFT_INDEX_CORRECTION
          if(d_state==gr::lora::DecoderState::DECODE_PAYLOAD){
            idx_error=round((float)d_idx_error/d_num_header);
          }
          else{
            idx_error=0;
          }
      #endif      

      curr_idx = std::max((int32_t)curr_idx + idx_error, 0);
      curr_bin = fft_mag[curr_idx];
      #if FFT_DEBUG_ON
          //std::cout << std::endl;
          if(d_state!=gr::lora::DecoderState::DETECT){
                  d_debug << "C: " << " curr_bin= " << curr_bin << "; curr_index: " << curr_idx << std::endl;
          }
          //std::cout << std::endl;
      #endif

      //samples_to_file(d_path+"/data/fft", &d_tmp[0], d_number_of_bins, sizeof(gr_complex));
      
      // Backward FFT for debugging
      // fft_execute(d_qr); 
      // samples_to_file(d_path+"/data/resampled", &d_mult_hf[0], d_number_of_bins, sizeof(gr_complex));

      // Return argmax here
      return curr_idx; 

    }

    uint32_t fft_receiver_impl::get_max_fft_bin(const gr_complex *samples, float &curr_bin){
      float fft_mag[d_number_of_bins];

      //samples_to_file(d_path+"/data/data", &samples[0], d_samples_per_symbol, sizeof(gr_complex));

      // Multiply with ideal downchirp
      for (uint32_t i = 0u; i < d_samples_per_symbol; i++) {
          d_mult_hf[i] = samples[i] * d_downchirp[i];
      }

      //samples_to_file(d_path+"/data/mult", &d_mult_hf[0], d_samples_per_symbol, sizeof(gr_complex));

      // Perform FFT
      fft_execute(d_q);

      // Decimate. Note: assumes fft size is multiple of decimation factor and number of bins is even
      // This decimation should be identical to numpy's approach
      const uint32_t N = d_number_of_bins;
      memcpy(&d_tmp[0],               &d_fft[0],                                     (N + 1u) / 2u * sizeof(gr_complex));
      memcpy(&d_tmp[ (N + 1u) / 2u ], &d_fft[d_samples_per_symbol - (N / 2u)],        N / 2u * sizeof(gr_complex));
      // d_tmp[N / 2u] += d_fft[N / 2u];

      // Get magnitude
      for (uint32_t i = 0u; i < d_number_of_bins; i++) {
          fft_mag[i] = std::abs(d_tmp[i]);
      }

    //   // Get postive and negative frequency bins.
    //   const uint32_t N = d_number_of_bins;
    //   memcpy(&d_positive[0], &d_fft[0],                      N*sizeof(gr_complex));
    //   memcpy(&d_negative[0], &d_fft[d_samples_per_symbol-N], N*sizeof(gr_complex));

    //   // Get magnitude
    //   for (uint32_t i = 0u; i < d_number_of_bins; i++) {
    //       d_tmp[i] = d_positive[i] + d_negative[i];
    //       fft_mag[i] = std::abs(d_tmp[i]);
    //   }
      
      // Copy samples, mult_hf, fft, tmp for debug
      if(d_state==gr::lora::DecoderState::DECODE_HEADER||d_state==gr::lora::DecoderState::DECODE_PAYLOAD){
        std::copy(samples, samples+d_samples_per_symbol, std::back_inserter(d_data_shr));          
        std::copy(samples, samples+d_samples_per_symbol, std::back_inserter(d_data));
        std::copy(&d_mult_hf[0], &d_mult_hf[0]+d_samples_per_symbol, std::back_inserter(d_data_mult));
        std::copy(&d_fft[0], &d_fft[0]+d_samples_per_symbol, std::back_inserter(d_data_fft));
        std::copy(&d_tmp[0], &d_tmp[0]+d_number_of_bins, std::back_inserter(d_data_decim_fft));
      }

      uint32_t curr_idx = 0;
      curr_idx = std::max_element(fft_mag, fft_mag + d_number_of_bins) - fft_mag;
      curr_bin = fft_mag[curr_idx];

      #if FFT_DEBUG_ON
          //std::cout << std::endl;
           if(d_state!=gr::lora::DecoderState::DETECT){
                  d_debug << "O: " << " curr_bin= " << curr_bin << "; curr_index: " << curr_idx << std::endl;
           }
          //std::cout << std::endl;
      #endif

      int32_t idx_error = 0;
      #if FFT_INDEX_CORRECTION
          if(d_state==gr::lora::DecoderState::DECODE_PAYLOAD){
            idx_error=round((float)d_idx_error/d_num_header);
          }
          else{
            idx_error=0;
          }
      #endif      
      
      curr_idx = ((int32_t)curr_idx +idx_error+d_number_of_bins) % d_number_of_bins;
      curr_idx = std::max((int32_t)curr_idx, 0);
      curr_bin = fft_mag[curr_idx];
      #if FFT_DEBUG_ON
          //std::cout << std::endl;
          if(d_state!=gr::lora::DecoderState::DETECT){
                  d_debug << "C: " << " curr_bin= " << curr_bin << "; curr_index: " << curr_idx << std::endl;
          }
          //std::cout << std::endl;
      #endif

      //samples_to_file(d_path+"/data/fft", &d_tmp[0], d_number_of_bins, sizeof(gr_complex));
      
      // Backward FFT for debugging
      // fft_execute(d_qr); 
      // samples_to_file(d_path+"/data/resampled", &d_mult_hf[0], d_number_of_bins, sizeof(gr_complex));

      // Return argmax here
      return curr_idx;      
    }

    float fft_receiver_impl::determine_energy(const gr_complex *samples) {
        float magsq_chirp[d_samples_per_symbol];
        float energy_chirp = 0;
        volk_32fc_magnitude_squared_32f(magsq_chirp, samples, d_samples_per_symbol);
        volk_32f_accumulator_s32f(&energy_chirp, magsq_chirp, d_samples_per_symbol);

        return energy_chirp/d_samples_per_symbol;
    }

    void fft_receiver_impl::determine_snr() {
        if(d_pwr_queue.size() >= 2) {
            float pwr_noise = d_pwr_queue[0];
            float pwr_signal = d_pwr_queue[d_pwr_queue.size()-1];
            d_snr = pwr_signal / pwr_noise;
            d_pwr_signal = pwr_signal;
            d_pwr_noise = pwr_noise;
        }
    }    

    bool fft_receiver_impl::demodulate(const gr_complex *samples, const bool reduced_rate) {
        // DBGR_TIME_MEASUREMENT_TO_FILE("SFxx_method");
        bool is_first =  d_implicit && (d_demodulated.size()==0);
        // DBGR_START_TIME_MEASUREMENT(false, "only");

        uint32_t bin_idx = get_max_fft_bin(samples, d_curr_bin);
        //uint32_t bin_idx = get_shift_fft(samples);


        // if(d_enable_fine_sync)
        //     fine_sync(samples, bin_idx, std::max(d_decim_factor / 4u, 2u));

        // DBGR_INTERMEDIATE_TIME_MEASUREMENT();

        // Save current index for index correction 
        d_curr_idx = bin_idx;

        // Header has additional redundancy
        if (reduced_rate || d_sf > 10) {
            bin_idx = std::lround(bin_idx / 4.0f) % d_number_of_bins_hdr;
        }

        d_curr_symbol = bin_idx;


        // Get the sum of header index error
        if(d_state==gr::lora::DecoderState::DECODE_HEADER){
            d_idx_error+=(int32_t)d_curr_symbol*4-(int32_t)d_curr_idx;
            d_num_header++; 
        }

        // Decode (actually gray encode) the bin to get the symbol value
        const uint32_t word = bin_idx ^ (bin_idx >> 1u);

        #if FFT_DEBUG_ON
            d_debug << gr::lora::to_bin(word, reduced_rate ? d_sf - 2u : d_sf) << " " << word << " (bin " << bin_idx << ")"  << std::endl;
        #endif
        d_words.push_back(word);

        // Look for 4+cr symbols and stop
        // wz:First block always use 4+4 since it's header no matter in implicit or explicit mode
        // wz:Second block will use 4+cr
        if (d_words.size() == (4u + (is_first ? 4 : d_phdr.cr))) {
            // Deinterleave
            deinterleave((reduced_rate||is_first || d_sf > 10) ? d_sf - 2u : d_sf);

            return true; // Signal that a block is ready for decoding
        }

        return false; // We need more words in order to decode a block
    }

    /**
     *  Correct the interleaving by extracting each column of bits after rotating to the left.
     *  <br/>(The words were interleaved diagonally, by rotating we make them straight into columns.)
     */
    void fft_receiver_impl::deinterleave(const uint32_t ppm) {
        const uint32_t bits_per_word = d_words.size();
        const uint32_t offset_start  = ppm - 1u;

        std::vector<uint8_t> words_deinterleaved(ppm, 0u);

        if (bits_per_word > 8u) {
            // Not sure if this can ever occur. It would imply coding rate high than 4/8 e.g. 4/9.
            std::cerr << "[LoRa Decoder] WARNING : Deinterleaver: More than 8 bits per word. uint8_t will not be sufficient!\nBytes need to be stored in intermediate array and then packed into words_deinterleaved!" << std::endl;
            exit(1);
        }

        for (uint32_t i = 0u; i < bits_per_word; i++) {
            const uint32_t word = gr::lora::rotl(d_words[i], i, ppm);

            for (uint32_t j = (1u << offset_start), x = offset_start; j; j >>= 1u, x--) {
                words_deinterleaved[x] |= !!(word & j) << i;
            }
        }

        #if FFT_DEBUG_ON
            print_interleave_matrix(d_debug, d_words, ppm);
            print_vector_bin(d_debug, words_deinterleaved, "D", sizeof(uint8_t) * 8u);
        #endif

        // Add to demodulated data
        d_demodulated.insert(d_demodulated.end(), words_deinterleaved.begin(), words_deinterleaved.end());

        // Cleanup
        d_words.clear();
    }

    void fft_receiver_impl::decode(const bool is_header) {
        static const uint8_t shuffle_pattern[] = {5, 0, 1, 2, 4, 3, 6, 7};

        // For determining shuffle pattern
        //if (!is_header)
        //    values_to_file("/tmp/before_deshuffle", &d_demodulated[0], d_demodulated.size(), 8);

        deshuffle(shuffle_pattern, is_header);

        // For determining whitening sequence
        //if (!is_header)
        //    values_to_file("/tmp/after_deshuffle", &d_words_deshuffled[0], d_words_deshuffled.size(), 8);
        if (d_implicit && ! d_reduced_rate) {
            dewhiten(is_header ? gr::lora::prng_header :
                        (d_phdr.cr ==4) ? gr::lora::prng_payload_cr8_implicit_fullrate:
                        (d_phdr.cr ==3) ? gr::lora::prng_payload_cr7_implicit_fullrate:
                        (d_phdr.cr ==2) ? gr::lora::prng_payload_cr6_implicit_fullrate :
                        gr::lora::prng_payload_cr5_implicit_fullrate);
        } else {
            dewhiten(is_header ? gr::lora::prng_header :
                (d_phdr.cr <=2) ? gr::lora::prng_payload_cr56 : gr::lora::prng_payload_cr78);
        }

        //if (!is_header)
        //    values_to_file("/tmp/after_dewhiten", &d_words_dewhitened[0], d_words_dewhitened.size(), 8);

        hamming_decode(is_header);
    }

    void fft_receiver_impl::deshuffle(const uint8_t *shuffle_pattern, const bool is_header) {
        const uint32_t to_decode = is_header ? 5u : d_demodulated.size();
        const uint32_t len       = sizeof(shuffle_pattern) / sizeof(uint8_t);
        uint8_t result;

        for (uint32_t i = 0u; i < to_decode; i++) {
            result = 0u;

            for (uint32_t j = 0u; j < len; j++) {
                result |= !!(d_demodulated[i] & (1u << shuffle_pattern[j])) << j;
            }

            d_words_deshuffled.push_back(result);
        }

        #if FFT_DEBUG_ON
            print_vector_bin(d_debug, d_words_deshuffled, "S", sizeof(uint8_t)*8);
        #endif

        // We're done with these words
        if (is_header){
            d_demodulated.erase(d_demodulated.begin(), d_demodulated.begin() + 5u);
            d_words_deshuffled.push_back(0);
        } else {
            d_demodulated.clear();
        }
    }

    void fft_receiver_impl::dewhiten(const uint8_t *prng) {
        const uint32_t len = d_words_deshuffled.size();

        for (uint32_t i = 0u; i < len; i++) {
            uint8_t xor_b = d_words_deshuffled[i] ^ prng[i];
            d_words_dewhitened.push_back(xor_b);
        }

        #if FFT_DEBUG_ON
            print_vector_bin(d_debug, d_words_dewhitened, "W", sizeof(uint8_t) * 8);
        #endif

        d_words_deshuffled.clear();
    }

    void fft_receiver_impl::hamming_decode(bool is_header) {
        switch(d_phdr.cr) {
            case 4: case 3: { // Hamming(8,4) or Hamming(7,4)
                //hamming_decode_soft(is_header);
                uint32_t n = ceil(d_words_dewhitened.size() * 4.0f / (4.0f + d_phdr.cr));
                uint8_t decoded[n];

                fec_decode(d_h48_fec, n, &d_words_dewhitened[0], decoded);

                if(!is_header)
                    swap_nibbles(decoded, n);
                d_decoded.assign(decoded, decoded+n);

                #if FFT_DEBUG_ON
                    print_vector_bin(d_debug, d_decoded, "F", sizeof(uint8_t) * 8);
                #endif
                
                break;
            }
            case 2: case 1: { // Hamming(6,4) or Hamming(5,4)
                // TODO: Report parity error to the user
                extract_data_only(is_header);
                break;
            }
        }

        d_words_dewhitened.clear();
    }

    /**
     * Deprecated
     */
    void fft_receiver_impl::hamming_decode_soft(bool is_header) {
        uint32_t len = d_words_dewhitened.size();
        for (uint32_t i = 0u; i < len; i += 2u) {
            const uint8_t d2 = (i + 1u < len) ? hamming_decode_soft_byte(d_words_dewhitened[i + 1u]) : 0u;
            const uint8_t d1 = hamming_decode_soft_byte(d_words_dewhitened[i]);

            if(is_header)
                d_decoded.push_back((d1 << 4u) | d2);
            else
                d_decoded.push_back((d2 << 4u) | d1);
        }
    }

    void fft_receiver_impl::extract_data_only(bool is_header) {
        static const uint8_t data_indices[4] = {1, 2, 3, 5};
        uint32_t len = d_words_dewhitened.size();

        for (uint32_t i = 0u; i < len; i += 2u) {
            const uint8_t d2 = (i + 1u < len) ? select_bits(d_words_dewhitened[i + 1u], data_indices, 4u) & 0xFF : 0u;
            const uint8_t d1 = (select_bits(d_words_dewhitened[i], data_indices, 4u) & 0xFF);

            if(is_header)
                d_decoded.push_back((d1 << 4u) | d2);
            else
                d_decoded.push_back((d2 << 4u) | d1);
        }
    }

    void fft_receiver_impl::msg_lora_frame(void) {
        uint32_t len = sizeof(loratap_header_t) + sizeof(loraphy_header_t) + d_payload_length;
        uint32_t offset = 0;
        uint8_t buffer[len];
        loratap_header_t loratap_header;

        memset(buffer, 0, sizeof(uint8_t) * len);
        memset(&loratap_header, 0, sizeof(loratap_header));
        
        determine_snr();
        loratap_header.rssi.snr = (uint8_t)(10.0f * log10(d_snr) + 0.5);

        d_snr_pow.push_back(d_snr);
        d_snr_db.push_back((uint8_t)(10.0f * log10(d_snr) + 0.5));
        d_pow_noise.push_back(d_pwr_noise);
        d_pow_signal.push_back(d_pwr_signal);

        if(d_snr_pow.size()>=10){
            floats_to_file(d_path+"/data/fft_signal_pow", &d_pow_signal[0], d_pow_signal.size(),sizeof(float),true);
            floats_to_file(d_path+"/data/fft_noise_pow", &d_pow_noise[0], d_pow_noise.size(),sizeof(float),true);
            floats_to_file(d_path+"/data/fft_snr_pow", &d_snr_pow[0],d_snr_pow.size(),sizeof(float),true);
            bytes_to_file(d_path+"/data/fft_snr_db", &d_snr_db[0],d_snr_db.size(),sizeof(char),true);
            d_snr_pow.clear();
            d_snr_db.clear();
            d_pow_noise.clear();
            d_pow_signal.clear();
        }

        // Copy samples, mult_hf, fft, tmp for debug
        std::string path_prefix=d_path+"/data/";
        samples_to_file(path_prefix+"data_shr_"+std::to_string(d_packet_cnt),&d_data_shr[0], d_data_shr.size(), sizeof(gr_complex));
        samples_to_file(path_prefix+"data_"+std::to_string(d_packet_cnt),&d_data[0], d_data.size(), sizeof(gr_complex));
        // samples_to_file(path_prefix+"data_mult_"+std::to_string(d_packet_cnt),&d_data_mult[0], d_data_mult.size(), sizeof(gr_complex));
        // samples_to_file(path_prefix+"data_fft_"+std::to_string(d_packet_cnt),&d_data_fft[0], d_data_fft.size(), sizeof(gr_complex));
        // samples_to_file(path_prefix+"data_decim_fft_"+std::to_string(d_packet_cnt),&d_data_decim_fft[0], d_data_decim_fft.size(), sizeof(gr_complex));        
        d_packet_cnt++;

        offset = gr::lora::build_packet(buffer, offset, &loratap_header, sizeof(loratap_header_t));
        offset = gr::lora::build_packet(buffer, offset, &d_phdr, sizeof(loraphy_header_t));
        offset = gr::lora::build_packet(buffer, offset, &d_decoded[0], d_payload_length);
        if(offset != len) {
            std::cerr << "fft_receiver_impl::msg_lora_frame: invalid write" << std::endl;
            exit(1);
        }

        pmt::pmt_t payload_blob = pmt::make_blob(buffer, sizeof(uint8_t)*len);
        message_port_pub(pmt::mp("frames"), payload_blob);
    }

    inline void fft_receiver_impl::instantaneous_phase(const gr_complex *in_samples, float *out_iphase, const uint32_t window) {
        out_iphase[0] = std::arg(in_samples[0]);

        for (uint32_t i = 1u; i < window; i++) {
            out_iphase[i] = std::arg(in_samples[i]);
            // = the same as atan2(imag(in_samples[i]),real(in_samples[i]));

            // Unwrapped loops from liquid_unwrap_phase
            while ( (out_iphase[i] - out_iphase[i-1]) >  M_PI ) out_iphase[i] -= 2.0f*M_PI;
            while ( (out_iphase[i] - out_iphase[i-1]) < -M_PI ) out_iphase[i] += 2.0f*M_PI;
        }
    }

    inline void fft_receiver_impl::instantaneous_frequency(const gr_complex *in_samples, float *out_ifreq, const uint32_t window) {
        if (window < 2u) {
            std::cerr << "[LoRa Decoder] WARNING : window size < 2 !" << std::endl;
            return;
        }

        /* instantaneous_phase */
        for (uint32_t i = 1u; i < window; i++) {
            const float iphase_1 = std::arg(in_samples[i - 1]);
                  float iphase_2 = std::arg(in_samples[i]);

            // Unwrapped loops from liquid_unwrap_phase
            while ( (iphase_2 - iphase_1) >  M_PI ) iphase_2 -= 2.0f*M_PI;
            while ( (iphase_2 - iphase_1) < -M_PI ) iphase_2 += 2.0f*M_PI;

            out_ifreq[i - 1] = iphase_2 - iphase_1;
        }

        // Make sure there is no strong gradient if this value is accessed by mistake
        out_ifreq[window - 1] = out_ifreq[window - 2];
    }

    int
    fft_receiver_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      (void) noutput_items;
      (void) output_items; 

      const gr_complex *input = (const gr_complex *) input_items[0];

      // uint32_t curr_idx    = 0;
      // uint32_t curr_symbol = 0;
      // float    curr_bin    = 0.0;
      // float    curr_energy = 0.0;
      bool preamble_found  = false;
      bool shr_save   = true;

      d_fine_sync         = 0; // Always reset fine sync 

      switch (d_state) {
          case gr::lora::DecoderState::RESET:{
              d_idx_history.clear();
              d_bin_queue.clear();
              d_pwr_queue.clear();
              preamble_found   = false;
              shr_save    = true;
              d_first_syncword = 0;
              d_sync_fails     = 0;
              d_curr_bin       = 0.0;
              d_curr_energy    = 0.0;

              // PHASE_BINARY_INDEX_CORRECTION
              d_idx_error   = 0;
              d_curr_idx    = 0;
              d_curr_symbol = 0;
              d_num_header  = 0;

              d_data_shr.clear();
              d_data.clear();
              d_data_mult.clear();
              d_data_fft.clear();
              d_data_decim_fft.clear();

              d_decoded.clear();
              d_words.clear();
              d_words_dewhitened.clear();
              d_words_deshuffled.clear();
              d_demodulated.clear();

              d_state = gr::lora::DecoderState::DETECT;

              #if FFT_DEBUG_ON
                d_debug << "Next state: DETECT" << std::endl;
              #endif

              break;
          }

          case gr::lora::DecoderState::DETECT: {
              d_curr_idx = get_max_fft_bin(input, d_curr_bin);
              d_idx_history.insert(d_idx_history.begin(), d_curr_idx);
              if(d_idx_history.size()>FFT_REQUIRED_PREAMBLE_CHIRPS){
                d_idx_history.pop_back();
              }

              d_curr_energy = determine_energy(input);
              d_pwr_queue.push_back(d_curr_energy);
              d_bin_queue.push_back(d_curr_bin);

              if(d_idx_history.size()>=FFT_REQUIRED_PREAMBLE_CHIRPS){
                d_preamble_idx = d_idx_history[0];

                // Check for discontinuities that exceed some tolerance
                preamble_found = true;
                for (uint32_t i = 1; i < FFT_REQUIRED_PREAMBLE_CHIRPS; i++){
                    if (abs(int(d_preamble_idx) - int(d_idx_history[i])) > FFT_PREAMBLE_TOLERANCE){
                        preamble_found = false;
                    }
                }

                if (preamble_found && d_curr_bin>=d_bin_threshold && d_curr_energy>=d_energy_threshold){
                    d_fine_sync=-(int)d_decim_factor*(int)d_curr_idx;                   

                    #if FFT_DEBUG_ON 
                        d_debug << "D: " << " bin= " << d_curr_bin << "; energy= " << std::setprecision(3) << d_curr_energy;
                        d_debug << "; d_fine_sync= " << d_fine_sync << "; d_curr_idx: " << d_curr_idx << std::endl;
                    #endif

                    d_sync_idx = get_max_fft_bin(input+d_fine_sync, d_curr_bin);

                    #if FFT_DEBUG_ON 
                        d_debug << "S: " << " bin= " << d_curr_bin << "; energy= " << std::setprecision(3) << d_curr_energy;
                        d_debug << "; d_fine_sync= " << d_fine_sync << "; d_sync_idx: " << d_sync_idx << std::endl;
                    #endif

                    if(d_sync_idx>FFT_SYNC_TOLERANCE){
                        d_state = gr::lora::DecoderState::RESET;
                        #if FFT_DEBUG_ON
                                d_debug << "Next state: RESET" << std::endl;
                        #endif
                    }else{
                        d_state = gr::lora::DecoderState::SYNC;
                        #if FFT_DEBUG_ON
                                d_debug << "Next state: SYNC" << std::endl;
                        #endif
                    }
                }                
              }

              consume_each((int)d_samples_per_symbol+d_fine_sync);

              break;
          }
          
          //Syncword is 0x00
          case gr::lora::DecoderState::SYNC: {
              if(d_sync_fails>FFT_SYNC_ATTEMPTS+2){
                d_state = gr::lora::DecoderState::RESET;
              }
              uint32_t index_shift = d_sync_fails * d_samples_per_symbol;
              if(detect_syncword(input+index_shift)){
                if(int32_t(d_sync_fails-10)>=0){
                    save_shr(&input[(d_sync_fails-10)*d_samples_per_symbol],8);
                }

                #if FFT_DEBUG_ON
                    d_debug << "Next state: Pause"<< "; d_sync_fails: " << d_sync_fails << std::endl;
                #endif
                
                d_state = gr::lora::DecoderState::PAUSE;
                consume_each(index_shift); 
              }
              else{
                  d_sync_fails++;
              }

              break;
          }
          
        //   // Syncword is 0x34
        //   case gr::lora::DecoderState::SYNC: {
        //       if(d_sync_fails>FFT_SYNC_ATTEMPTS){
        //         d_state = gr::lora::DecoderState::RESET; 
        //       } 
        //       uint32_t index_shift = d_sync_fails * d_samples_per_symbol;
        //       if(detect_first_syncword(&input[index_shift], d_first_syncword)){
        //         //samples_to_file(d_path+"/data/fft_detect",  &input[0], 8*d_samples_per_symbol, sizeof(gr_complex));
        //         if(int32_t(d_sync_fails-8)>=0){
        //             save_shr(&input[(d_sync_fails-8)*d_samples_per_symbol],8);
        //         }
        //         #if FFT_DEBUG_ON
        //             d_debug << "Next state: FIND_SFD "<< "; d_sync_fails: " << d_sync_fails << std::endl;
        //         #endif
        //         d_state = gr::lora::DecoderState::FIND_SFD;
        //       }else{
        //         d_sync_fails++;
        //       }  
        //     //   Old method to print SHR  
        //     //   if(shr_save){
        //     //     save_shr(input, 8);
        //     //     shr_save = false;   
        //     //   }

        //     //   d_sync_fails++;
        //     //   if(d_sync_fails>=FFT_SYNC_ATTEMPTS){
        //     //     d_state = gr::lora::DecoderState::RESET; 
        //     //   }
        //     //   if(detect_first_syncword(input, d_first_syncword)){
        //     //     samples_to_file(d_path+"/data/fft_detect",  &input[0], 8*d_samples_per_symbol, sizeof(gr_complex));
        //     //     #if FFT_DEBUG_ON
        //     //         d_debug << "Next state: FIND_SFD "<< std::endl;
        //     //     #endif
        //     //     d_state = gr::lora::DecoderState::FIND_SFD;
        //     //   }
        //     //   else{
        //     //     consume_each((int)d_samples_per_symbol+d_fine_sync);
        //     //   }

        //     // int32_t step_length=2;
        //     // int32_t steps=(4*(int)d_number_of_bins)/step_length;
        //     // int32_t shift=(int)d_samples_per_symbol-(int)(d_decim_factor*d_curr_idx)-(int)d_number_of_bins;

        //     // int32_t i;
        //     // for(i=1;i<=steps;i++)
        //     // {
        //     //     #if FFT_DEBUG_ON
        //     //         d_debug << "steps: "<< i << "; shift: " << shift+i*step_length << std::endl;
        //     //     #endif
        //     //     if(detect_syncword(input+shift+i*step_length)){
        //     //         d_fine_sync=shift+i*step_length;
        //     //         d_state=gr::lora::DecoderState::FIND_SFD;
        //     //         break;
        //     //     }
        //     // }
        //     // if(i==steps+1){
        //     //     d_state = gr::lora::DecoderState::RESET;
        //     //     break;                
        //     // }

        //     // consume_each((int32_t)d_samples_per_symbol*2+d_fine_sync);

        //     //   float cfo = experimental_determine_cfo(&input[i], d_samples_per_symbol);
        //     //   pmt::pmt_t kv = pmt::cons(pmt::intern(std::string("cfo")), pmt::from_double(cfo));
        //     //   message_port_pub(pmt::mp("control"), kv);

        //     break;
        //   }

        //   case gr::lora::DecoderState::FIND_SFD: {
        //       int32_t shift=(d_syncword[0]-d_first_syncword)*(int32_t)d_decim_factor;
        //       shift = 0;
        //       d_fine_sync=(int32_t)d_samples_per_symbol*(d_sync_fails+2)+shift;
              
        //       #if FFT_DEBUG_ON
        //         d_debug << "d_fine_sync= " << d_fine_sync << "; shift= " << shift << std::endl;
        //       #endif

        //       if(detect_syncword(input+d_fine_sync)){
        //         d_state = gr::lora::DecoderState::PAUSE;
        //       }else{
        //         d_state = gr::lora::DecoderState::RESET;
        //       }

        //       consume_each(d_fine_sync); 

        //       break;
        //   }

          case gr::lora::DecoderState::PAUSE: {
              samples_to_file(d_path+"/data/fft_sync", input, 2*d_samples_per_symbol, sizeof(gr_complex));
              get_max_freq_bin(input, d_curr_bin, false);
              get_max_freq_bin(input+d_samples_per_symbol, d_curr_bin, false);
              if(d_implicit){
                  d_state = gr::lora::DecoderState::DECODE_PAYLOAD;
                  d_payload_symbols = 1;
              } else {
                  d_state = gr::lora::DecoderState::DECODE_HEADER;
              }
              consume_each((int32_t)d_samples_per_symbol*2 + d_delay_after_sync);
              break;
          }

          case gr::lora::DecoderState::DECODE_HEADER: {
              d_phdr.cr = 4u;

              if (demodulate(input, true)) {
                decode(true);

                gr::lora::print_vector_hex(std::cout, &d_decoded[0], d_decoded.size(), false);
                memcpy(&d_phdr, &d_decoded[0], sizeof(loraphy_header_t));
                /* wz: Note MSB and LSB when memcpy into struct!
                std::cout<<" d_phdr.length: " <<(int)d_phdr.length << std::endl;
                std::cout<<" d_phdr.crc_msn: " <<(int)d_phdr.crc_msn << std::endl;
                std::cout<<" d_phdr.has_mac_crc: " <<(int)d_phdr.has_mac_crc << std::endl;
                std::cout<<" d_phdr.cr: " <<(int)d_phdr.cr << std::endl;
                std::cout<<" d_phdr.crc_lsn: " <<(int)d_phdr.crc_lsn << std::endl;
                std::cout<<" d_phdr.reserved: " <<(int)d_phdr.reserved << std::endl;
                */
                if (d_phdr.cr > 4)
                    d_phdr.cr = 4;
                
                if(d_phdr.length!=d_packet_len){
                  d_state = gr::lora::DecoderState::RESET;
                  break;
                }

                bytes_to_file(d_path+"/data/fft_result", &d_decoded[0], d_decoded.size(),sizeof(char), true);                  
                d_decoded.clear();

                d_payload_length = d_phdr.length + MAC_CRC_SIZE * d_phdr.has_mac_crc;
                //d_phy_crc = SM(decoded[1], 4, 0xf0) | MS(decoded[2], 0xf0, 4);

                // Calculate number of payload symbols needed
                uint8_t redundancy = ((d_sf > 10 || d_reduced_rate) ? 2 : 0);
                const int symbols_per_block = d_phdr.cr + 4u;
                const float bits_needed     = float(d_payload_length) * 8.0f;
                const float symbols_needed  = bits_needed * (symbols_per_block / 4.0f) / float(d_sf - redundancy);
                const int blocks_needed     = (int)std::ceil(symbols_needed / symbols_per_block);
                d_payload_symbols     = blocks_needed * symbols_per_block;

                #if FFT_DEBUG_ON
                    d_debug << "LEN: " << d_payload_length << " (" << d_payload_symbols << " symbols)" << std::endl;
                #endif
                
                //   std::cout << "Header Decoded, Break!" << std::endl;
                d_state = gr::lora::DecoderState::DECODE_PAYLOAD;
              }

              consume_each((int32_t)d_samples_per_symbol+d_fine_sync);
              break;
          }

          case gr::lora::DecoderState::DECODE_PAYLOAD: {
              // TODO: Energy Comparsion to Support Implicit Mode!
              if (d_implicit) {
                  d_payload_symbols = 0;
                  //d_demodulated.erase(d_demodulated.begin(), d_demodulated.begin() + 7u); // Test for SF 8 with header
                  d_payload_length = (int32_t)(d_demodulated.size() / 2);
              } else if (demodulate(input, d_implicit || d_reduced_rate)) {
                  if(!d_implicit)
                      d_payload_symbols -= (4u + d_phdr.cr);
              }

              if (d_payload_symbols <= 0) {
                  decode(false);
                  gr::lora::print_vector_hex(std::cout, &d_decoded[0], d_payload_length, true);
                  bytes_to_file(d_path+"/data/fft_result", &d_decoded[0], d_payload_length,sizeof(char), true);
                  msg_lora_frame();

                  d_state = gr::lora::DecoderState::RESET;
              }

              consume_each((int32_t)d_samples_per_symbol+d_fine_sync);

              break;
          }

          case gr::lora::DecoderState::STOP: {
              consume_each(d_samples_per_symbol);
              break;
          }

          default: {
              std::cerr << "[LoRa Decoder] WARNING : No state! Shouldn't happen\n";
              break;
          }
      }      

      // Tell runtime system how many output items we produced.
      return 0;
    }

    void fft_receiver_impl::set_sf(const uint8_t sf) {
        (void) sf;
        std::cerr << "[LoRa Decoder] WARNING : Setting the spreading factor during execution is currently not supported." << std::endl
                  << "Nothing set, kept SF of " << d_sf << "." << std::endl;
    }

    void fft_receiver_impl::set_samp_rate(const float samp_rate) {
        (void) samp_rate;
        std::cerr << "[LoRa Decoder] WARNING : Setting the sample rate during execution is currently not supported." << std::endl
                  << "Nothing set, kept SR of " << d_samples_per_second << "." << std::endl;
    }  

  } /* namespace lora */
} /* namespace gr */

