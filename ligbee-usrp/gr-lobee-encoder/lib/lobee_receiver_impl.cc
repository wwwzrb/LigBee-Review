/* -*- c++ -*- */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/expj.h>
#include <liquid/liquid.h>
#include <numeric>
#include <algorithm>
#include <lora/loratap.h>
#include <lora/utilities.h>
#include "lobee_receiver_impl.h"
#include "tables.h"

#include "dbugr.hpp"

#define DEBUG_OFF     1
#define DEBUG_INFO    1
#define DEBUG_VERBOSE 2
#define DEBUG_ON      DEBUG_OFF

#define BINARY_GRADIENT 1
#define PHASE_BINARY_INDEX_CORRECTION 1
#define CORRELATION_IDX 0

namespace gr {
  namespace lora {

    lobee_receiver::sptr
    lobee_receiver::make(float samp_rate, uint32_t bandwidth, uint8_t sf, bool implicit, uint8_t cr, bool crc, bool reduced_rate, bool disable_drift_correction, int32_t sync_shift, uint8_t packet_len, std::string path)
    {
      return gnuradio::get_initial_sptr
        (new lobee_receiver_impl(samp_rate, bandwidth, sf, implicit, cr, crc, reduced_rate, disable_drift_correction, sync_shift, packet_len, path));
    }

    /*
     * The private constructor
     */
    lobee_receiver_impl::lobee_receiver_impl(float samp_rate, uint32_t bandwidth, uint8_t sf, bool implicit, uint8_t cr, bool crc, bool reduced_rate, bool disable_drift_correction, int32_t sync_shift, uint8_t packet_len, std::string path)
      : gr::sync_block("lobee_receiver",
              gr::io_signature::make(1, 1, sizeof(char)),
              gr::io_signature::make(0, 0, 0)),
      d_pwr_queue(MAX_PWR_QUEUE_SIZE),
      d_packet_len(packet_len),
      d_path(path)
    {
     // Radio config
      d_state = gr::lora::DecoderState::RESET;

      if (sf < 6 || sf > 13) {
          std::cerr << "[LoRa Decoder] ERROR : Spreading factor should be between 6 and 12 (inclusive)!" << std::endl
                    << "                       Other values are currently not supported." << std::endl;
          exit(1);
      }

      #if DEBUG_ON
          d_debug_samples.open(d_path+"/data/grlobee_debug", std::ios::out | std::ios::binary);
          d_debug.open(d_path+"/data/grlobee_debug_txt", std::ios::out);
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
      d_delay_after_sync   = d_samples_per_symbol / 4u;
      d_number_of_bins     = (uint32_t)(1u << d_sf);
      d_halfnum_of_bins    = (uint32_t)(d_number_of_bins/2);  
      d_number_of_bins_hdr = (uint32_t)(1u << (d_sf-2));
      d_decim_factor       = d_samples_per_symbol / d_number_of_bins;
      d_energy_threshold   = 0.0f;
    
      #if BINARY_GRADIENT
        d_window = d_halfnum_of_bins;
      #else
        d_window = d_halfsamp_per_symbol;
      #endif

      d_threshold          = 3*d_window/4;
      d_energy             = 0;
      d_syncword[2]        = (int32_t)d_halfnum_of_bins;
      d_syncword[3]        = d_syncword[2];
      d_sync_index         = 0;
      d_sync_end           = 0;
      d_sync_shift         = sync_shift;

      // PHASE_BINARY_INDEX_CORRECTION
      d_index_error=0;
      d_curr_bin_index=0;
      d_curr_index=0;
      d_num_header=0;

      d_fine_sync = 0;
      d_enable_fine_sync = !disable_drift_correction;
      set_output_multiple(8 * d_samples_per_symbol);

      std::cout << "Bits (nominal) per symbol: \t"      << d_bits_per_symbol    << std::endl;
      std::cout << "Bins per symbol: \t"      << d_number_of_bins     << std::endl;
      std::cout << "Samples per symbol: \t"   << d_samples_per_symbol << std::endl;
      std::cout << "Decimation: \t\t"         << d_decim_factor       << std::endl;

      if(!d_enable_fine_sync) {
          std::cout << "Warning: clock drift correction disabled" << std::endl;
      }
      if(d_implicit) {
          std::cout << "CR: \t\t"         << (int)d_phdr.cr       << std::endl;
          std::cout << "CRC: \t\t"         << (int)d_phdr.has_mac_crc       << std::endl;
      }

      // Locally generated chirps
      build_ideal_chirps();

      // Hamming coding
      fec_scheme fs = LIQUID_FEC_HAMMING84;
      d_h48_fec = fec_create(fs, NULL);

      // Register gnuradio ports
      message_port_register_out(pmt::mp("frames"));      
    }

    /*
     * Our virtual destructor.
     */
    lobee_receiver_impl::~lobee_receiver_impl()
    {
      #if DEBUG_ON
          if (d_debug_samples.is_open())
              d_debug_samples.close();

          if (d_debug.is_open())
              d_debug.close();
      #endif

      fec_destroy(d_h48_fec);
    }
    
    uint8_t lobee_receiver_impl::detect_sf_autocorr(const unsigned char *samples_binary, uint32_t window)
    {
        return 7;
    }

    void lobee_receiver_impl::build_ideal_chirps(void) {
        std::vector<unsigned char> zeros_binary(d_halfsamp_per_symbol, 0);
        std::vector<unsigned char> ones_binary(d_halfsamp_per_symbol, 1);

        d_zeros_binary = zeros_binary;
        d_ones_binary = ones_binary;

        d_upchirp_binary=zeros_binary;
        d_downchirp_binary=ones_binary;

        d_upchirp_binary.insert(d_upchirp_binary.end(),d_ones_binary.begin(), d_ones_binary.end());      
        d_downchirp_binary.insert(d_downchirp_binary.end(),d_zeros_binary.begin(),d_zeros_binary.end());

        d_upchirp_binary=d_upchirp_binary;
        d_upchirp_binary_v.insert(d_upchirp_binary_v.end(),d_upchirp_binary.begin(), d_upchirp_binary.end());
        d_upchirp_binary_v.insert(d_upchirp_binary_v.end(),d_upchirp_binary.begin(), d_upchirp_binary.end());

        d_downchirp.resize(d_samples_per_symbol);
        d_upchirp.resize(d_samples_per_symbol);
        d_downchirp_ifreq.resize(d_samples_per_symbol);
        d_upchirp_ifreq.resize(d_samples_per_symbol);
        d_upchirp_ifreq_v.resize(d_samples_per_symbol*3);
        gr_complex tmp[d_samples_per_symbol*3];

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

        // Store instantaneous frequency
        instantaneous_frequency(&d_downchirp[0], &d_downchirp_ifreq[0], d_samples_per_symbol);
        instantaneous_frequency(&d_upchirp[0],   &d_upchirp_ifreq[0],   d_samples_per_symbol);

        samples_to_file(d_path+"/data/lobee_downchirp", &d_downchirp_binary[0], d_downchirp_binary.size(), sizeof(char),false);
        samples_to_file(d_path+"/data/lobee_upchirp",   &d_upchirp_binary[0],   d_upchirp_binary.size(),   sizeof(char),false);

        // Upchirp sequence
        memcpy(tmp, &d_upchirp[0], sizeof(gr_complex) * d_samples_per_symbol);
        memcpy(tmp+d_samples_per_symbol, &d_upchirp[0], sizeof(gr_complex) * d_samples_per_symbol);
        memcpy(tmp+d_samples_per_symbol*2, &d_upchirp[0], sizeof(gr_complex) * d_samples_per_symbol);
        instantaneous_frequency(tmp, &d_upchirp_ifreq_v[0], d_samples_per_symbol*3);
    }   

    inline void lobee_receiver_impl::instantaneous_frequency(const gr_complex *in_samples, float *out_ifreq, const uint32_t window) {
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

    inline void lobee_receiver_impl::instantaneous_phase(const gr_complex *in_samples, float *out_iphase, const uint32_t window) {
        out_iphase[0] = std::arg(in_samples[0]);

        for (uint32_t i = 1u; i < window; i++) {
            out_iphase[i] = std::arg(in_samples[i]);
            // = the same as atan2(imag(in_samples[i]),real(in_samples[i]));

            // Unwrapped loops from liquid_unwrap_phase
            while ( (out_iphase[i] - out_iphase[i-1]) >  M_PI ) out_iphase[i] -= 2.0f*M_PI;
            while ( (out_iphase[i] - out_iphase[i-1]) < -M_PI ) out_iphase[i] += 2.0f*M_PI;
        }
    }

    void lobee_receiver_impl::values_to_file(const std::string path, const unsigned char *v, const uint32_t length, const uint32_t ppm) {
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

    void lobee_receiver_impl::samples_to_file(const std::string path, const unsigned char *v, const uint32_t length, const uint32_t elem_size, bool app) {
        #if DEBUG_ON
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

    void lobee_receiver_impl::samples_debug(const unsigned char *v, const uint32_t length) {
        #if DEBUG_ON
            gr_complex start_indicator(0.0f, 32.0f);
            d_debug_samples.write(reinterpret_cast<const char *>(&start_indicator), sizeof(char));

            for (uint32_t i = 1u; i < length; i++) {
                d_debug_samples.write(reinterpret_cast<const char *>(&v[i]), sizeof(char));
            }
        #else
            (void) v;
            (void) length;
        #endif
    }

    void lobee_receiver_impl::fine_sync(const unsigned char* samples_binary, uint32_t bin_idx, int32_t search_space)
    {
        int32_t shift_ref = (bin_idx+1) * d_decim_factor;
        float max_correlation = 0.0f;
        int32_t lag = 0;

        for(int32_t i = -search_space+1; i < search_space; i++) {
            float c =cross_correlate_binary_shift(samples_binary, shift_ref+i);
            // wz: the begin position of d_upchirp_binary_v is shift_ref!
            if(c > max_correlation) {
                    max_correlation = c;
                    lag = i;
                }
        }

        #if DEBUG_ON
            d_debug << "FINE: " << -lag << std::endl;
        #endif

        d_fine_sync = -lag;       
    }

    float lobee_receiver_impl::detect_preamble_autocorr(const unsigned char *samples_binary, const uint32_t window) {
        const unsigned char* chirp1 = samples_binary;
        const unsigned char* chirp2 = samples_binary + d_samples_per_symbol;

        uint32_t energy_chirp1 = 0;
        uint32_t energy_chirp2 = 0;
        float autocorr = 0;

        unsigned char flag_chirp1;
        unsigned char flag_chirp2;
        uint32_t index_chirp1;
        uint32_t index_chirp2;
        uint32_t corr_window = (uint32_t)(window/2);

        index_chirp1=max_correlation_idx(chirp1, energy_chirp1,flag_chirp1,corr_window,d_corr_index);
        index_chirp2=max_correlation_idx(chirp2, energy_chirp2,flag_chirp2,corr_window,d_corr_index);

        // When using implicit mode, stop when energy is halved.
        d_energy_threshold = energy_chirp2 / 2.0f;

        // For implicit mode compartion
        d_pwr_queue.push_back(energy_chirp1);

        // Autocorr value
        // for(uint32_t i=0;i<d_samples_per_symbol;i++)
        // {
        //     autocorr+=(float)(1-2*(chirp1[i]^chirp2[i]));
        // }
        // autocorr = abs(autocorr/d_samples_per_symbol);

        if(abs(int(index_chirp1-index_chirp2))<=2){
          autocorr=1;
        }
        else
        {
          autocorr=0;
        }
          
        return autocorr;
    }

    bool lobee_receiver_impl::detect_first_syncword(const unsigned char *samples_binary, int32_t &first_syncword){
        uint32_t index_chirp[REQUIRED_SYNC_WORDS];

        for(int32_t i=0;i<REQUIRED_SYNC_WORDS;i++){
            index_chirp[i]=max_binary_gradient_idx(samples_binary+i*d_samples_per_symbol, d_window);
        }

        #if DEBUG_ON
            for(int32_t i=0;i<REQUIRED_SYNC_WORDS;i++){
                d_debug << " index_chirp[" << i << "]: "<< index_chirp[i];
            }
            d_debug << "; d_fine_sync: " << d_fine_sync << std::endl;
        #endif

        // if(flag_chirp1==1){
        //     d_fine_sync=-(int)d_corr_index[1];
        // }
        // else{
        //     d_fine_sync=(int)d_corr_index[0]-(int)d_samples_per_symbol;
        // }
        // d_fine_sync=-(int)d_decim_factor*(int)index_chirp1;
        

        if(abs((int)index_chirp[0]-d_syncword[0])<=LORA_SFD_COARSE&&abs((int)index_chirp[1]-d_syncword[1])<=LORA_SFD_COARSE){
            first_syncword=index_chirp[0];
            d_fine_sync=0;
            return true;
        }
        else{
            return false;   
        }      
    }

    bool lobee_receiver_impl::detect_syncword(const unsigned char *samples_binary){
        uint32_t index_chirp[REQUIRED_SFD_CHIRPS];

        int32_t sfd_error=0;
        bool sfd_found=true;
        for(int32_t i=0;i<REQUIRED_SFD_CHIRPS;i++){
            index_chirp[i]=max_binary_gradient_idx(samples_binary+i*d_samples_per_symbol,d_window);
            #if DEBUG_ON
                d_debug << " index_chirp" << i << ": "<< index_chirp[i] << std::endl;
            #endif
            if(abs((int)index_chirp[i]-d_syncword[i+REQUIRED_SYNC_WORDS])>d_sync_shift){
                sfd_error=0;
                sfd_found=false;
                break;
            }
            else{
                sfd_error+=abs((int)index_chirp[i]-d_syncword[i+REQUIRED_SYNC_WORDS]);
            }
        }       
        if(sfd_found&&sfd_error<=2*d_sync_shift){
            return true;
        }
        else{
            return false;
        }

    }

    float lobee_receiver_impl::detect_downchirp(const unsigned char *samples_binary, const uint32_t window) {
        return cross_correlate_binary(samples_binary, false);
    }    

    float lobee_receiver_impl::detect_upchirp(const unsigned char *samples_binary, const uint32_t window, int32_t *index){
        return sliding_norm_cross_correlate_upchirp(samples_binary, window, index);
    }

    float lobee_receiver_impl::sliding_norm_cross_correlate_upchirp(const unsigned char *samples_binary, const uint32_t window, int32_t *index){
        float max_correlation = 0;

        // Cross correlate
        for (uint32_t i = 0; i < window; i++) {
            const float max_corr = abs(cross_correlate_binary(samples_binary, true));

            if (max_corr > max_correlation) {
                *index = i;
                max_correlation = max_corr;
            }
        }

        return max_correlation;
    }

    float lobee_receiver_impl::cross_correlate_binary_shift(const unsigned char *samples_binary, const uint32_t shift){
        float autocorr=0;
        // Autocorr value
        for(uint32_t i=0;i<d_samples_per_symbol;i++)
        {
            autocorr+=(float)(1-2*(samples_binary[i]^d_upchirp_binary_v[shift+i]));
        }
        autocorr = abs(autocorr/d_samples_per_symbol);

        return autocorr;           
    }

    float lobee_receiver_impl::cross_correlate_binary(const unsigned char *samples_binary, bool up){
        float autocorr=0;
        // Autocorr value
        for(uint32_t i=0;i<d_samples_per_symbol;i++)
        {
            if(up){
                autocorr+=(float)(1-2*(samples_binary[i]^d_upchirp_binary[i]));
            }
            else{
                autocorr+=(float)(1-2*(samples_binary[i]^d_downchirp_binary[i]));
            }
            
        }
        autocorr = autocorr/d_samples_per_symbol;

        return autocorr;        
    }

    uint32_t lobee_receiver_impl::max_binary_gradient_idx(const unsigned char *samples_binary, uint32_t window){
        uint32_t decim_factor = (d_halfsamp_per_symbol/window);
        unsigned char samples_binary_corr[2*window];

        if(window == d_halfsamp_per_symbol){
            return max_correlation_idx(samples_binary, d_energy, d_flag,window,d_corr_index);
        }else{
            // Dump binary_ifreq
            float sum;
            for(uint32_t i=0;i<d_samples_per_symbol;i++)
            {
                if((i+1)%decim_factor==0){
                    sum=0.0;
                    for(uint32_t j=i+1-decim_factor;j<=i;j++){
                        sum+=(float)samples_binary[j];
                    }
                    samples_binary_corr[i/decim_factor] = (unsigned char)(round(sum/decim_factor));
                }
            }
            return max_correlation_idx(samples_binary_corr, d_energy, d_flag,window,d_corr_index);   
        }
    }

    uint32_t lobee_receiver_impl::max_correlation_idx(const unsigned char *samples_binary, uint32_t &energy, unsigned char &flag, uint32_t window, uint32_t *corr_index){
        const unsigned char lsb_zero=0;
        const unsigned char lsb_one=1;
        uint32_t curr_index=0;
        uint32_t zero_cnt = 0;
        uint32_t one_cnt = 0;  
        std::vector<uint32_t> corr_zero;
        std::vector<uint32_t> corr_one;

        //samples_to_file(d_path+"/data/lobee_data", &samples_binary[0], window, sizeof(char), true);

        uint32_t corr_zero_max;
        uint32_t corr_one_max;
        std::vector<uint32_t> corr_zero_index_vec;
        std::vector<uint32_t> corr_one_index_vec;
        uint32_t corr_zero_index;
        uint32_t corr_one_index;

        for(uint32_t i=0;i<=window;i++){
            for(uint32_t j=0;j<window;j++){
                if(samples_binary[i+j]==lsb_zero)
                {
                    zero_cnt++;
                }
                if(samples_binary[i+j]==lsb_one)
                {
                    one_cnt++;
                }
            }
            corr_zero.push_back(zero_cnt);
            corr_one.push_back(one_cnt);

            zero_cnt=0;
            one_cnt=0;
        }

        auto corr_zero_iter = std::max_element(corr_zero.begin(), corr_zero.end());
        auto corr_one_iter = std::max_element(corr_one.begin(), corr_one.end());
        corr_zero_max = *corr_zero_iter;
        corr_one_max = *corr_one_iter;

        corr_zero_index = std::distance(corr_zero.begin(), corr_zero_iter);
        corr_one_index = std::distance(corr_one.begin(), corr_one_iter);

        // #if DEBUG
        //     std::cout << "Before Averaging:" << std::endl;
        //     std::cout << " corr_zero_index: " << corr_zero_index << " corr_zero_max: " << corr_zero_max << std::endl;
        //     std::cout << " corr_one_index: " << corr_one_index << " corr_one_max: " << corr_one_max << std::endl;
        // #endif

        zero_cnt=0;
        one_cnt =0;
        for(decltype(corr_zero.size()) i=corr_zero_index;i<(corr_zero.size());i++)
        {
            if(corr_zero[i]==corr_zero_max){
                corr_zero_index_vec.push_back(i);
                zero_cnt++;
            }
        }
        for(decltype(corr_one.size()) i=corr_one_index;i<(corr_one.size());i++)
        {
            if(corr_one[i]==corr_one_max){
                corr_one_index_vec.push_back(i);
                one_cnt++;
            }
        }
        
        uint32_t decim_factor = window/d_halfnum_of_bins;

        int32_t index_error=0;
        #if PHASE_BINARY_INDEX_CORRECTION
            if(d_state==gr::lora::DecoderState::DECODE_PAYLOAD){
                index_error=round((float)d_index_error/d_num_header);
            }
            else{
                index_error=0;
            }
        #endif

        corr_zero_index = round((float)std::accumulate(corr_zero_index_vec.begin(), corr_zero_index_vec.end(), 0)/zero_cnt);
        corr_one_index = round((float)std::accumulate(corr_one_index_vec.begin(), corr_one_index_vec.end(), 0)/one_cnt);

        #if BINARY_GRADIENT
            // according to correlation and gradient frequency info
            curr_index = 0;
            // corr_zero_index = corr_zero_index_vec.front();
            // corr_one_index = corr_one_index_vec.back();

            if(corr_zero_max > corr_one_max){
                flag = 0;
                if(corr_one_max>=(window/2)){
                    energy = corr_zero_max;
                } 
                for(int32_t i=corr_zero_index;i>=0;i--){
                    if(samples_binary[i]==1){
                        curr_index = round((float)i/decim_factor)+2;
                        break;
                    }
                }
                // for(int32_t i=corr_zero_index;i>=0;i--){
                //     if(samples_binary[i]==1){
                //         curr_index+=1;
                //     }
                // } 
                // curr_index = round((float)curr_index/decim_factor)+2;
            }
            else{
                flag = 1;
                energy = corr_one_max;
                for(int32_t i=corr_one_index+corr_one_max;i<(int32_t)(2*window);i++){
                    if(samples_binary[i]==0){
                        curr_index = round((float)i/decim_factor)+1;
                        break;
                    }
                }
                // for(int32_t i=corr_one_index+corr_one_max;i<(int32_t)(2*window);i++){
                //     if(samples_binary[i]==0){
                //         curr_index += 1;
                //     }
                // }                
                // curr_index=round((float)curr_index/decim_factor)+1;
            }
            curr_index = (d_number_of_bins - curr_index + index_error)%d_number_of_bins;
        #else
            // according to correlation info
            if(corr_zero_max>=corr_one_max){
                curr_index = std::max(floor(float((int32_t)window*2-(int32_t)corr_zero_index+index_error*decim_factor)/decim_factor),0.0);
                flag=0;
                if(corr_one_max>=(window/2)){
                    energy = corr_zero_max;
                }          
            }
            else{
                curr_index = std::max(floor(float((int32_t)window-(int32_t)corr_one_index+index_error*decim_factor)/decim_factor),0.0);
                flag=1; 
                energy = corr_one_max;       
            }
            curr_index=((curr_index+d_number_of_bins)%d_number_of_bins);
        #endif

        corr_index[0]=corr_zero_index;
        corr_index[1]=corr_one_index;
        corr_index[2]=corr_zero_max;
        corr_index[3]=corr_one_max;

        #ifdef DEBUG
            //std::cout << std::endl;
            if(d_state!=gr::lora::DecoderState::DETECT){
                    d_debug << "C: " << " flag=" << (uint32_t)flag << "; corr_zero_index: " << corr_zero_index << " corr_zero_max: " << corr_zero_max << "; corr_one_index: " << corr_one_index << " corr_one_max: " << corr_one_max;
                    d_debug << " curr_index: " << curr_index << "; index_error " << index_error << std::endl;
            }
            //std::cout << std::endl;
        #endif


        return curr_index;
    }

    bool lobee_receiver_impl::demodulate(const unsigned char *samples_binary, const bool reduced_rate) {
        // DBGR_TIME_MEASUREMENT_TO_FILE("SFxx_method");
        bool is_first =  d_implicit && (d_demodulated.size()==0);
        // DBGR_START_TIME_MEASUREMENT(false, "only");

        // uint32_t bin_idx = max_correlation_idx(samples_binary, energy_chirp, flag_chirp, d_halfsamp_per_symbol,d_corr_index);
        uint32_t bin_idx = (int32_t)max_binary_gradient_idx(samples_binary, d_window);
        //uint32_t bin_idx = get_shift_fft(samples);


        // if(d_enable_fine_sync)
        //     fine_sync(samples_binary, bin_idx, std::max(d_decim_factor / 4u, 2u));

        // DBGR_INTERMEDIATE_TIME_MEASUREMENT();

        // Save current index for index correction 
        d_curr_index = bin_idx;

        // Header has additional redundancy
        if (reduced_rate || d_sf > 10) {
            bin_idx = std::lround(bin_idx / 4.0f) % d_number_of_bins_hdr;
        }

        d_curr_bin_index = bin_idx;


        // Get the sum of header index error
        if(d_state==gr::lora::DecoderState::DECODE_HEADER){
            d_index_error+=(int32_t)d_curr_bin_index*4-(int32_t)d_curr_index;
            d_num_header++; 
        }

        // Decode (actually gray encode) the bin to get the symbol value
        const uint32_t word = bin_idx ^ (bin_idx >> 1u);

        #if DEBUG_ON
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
    void lobee_receiver_impl::deinterleave(const uint32_t ppm) {
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

        #if DEBUG_ON
            print_interleave_matrix(d_debug, d_words, ppm);
            print_vector_bin(d_debug, words_deinterleaved, "D", sizeof(uint8_t) * 8u);
        #endif

        // Add to demodulated data
        d_demodulated.insert(d_demodulated.end(), words_deinterleaved.begin(), words_deinterleaved.end());

        // Cleanup
        d_words.clear();
    }

    void lobee_receiver_impl::decode(const bool is_header) {
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

    void lobee_receiver_impl::deshuffle(const uint8_t *shuffle_pattern, const bool is_header) {
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

        #if DEBUG_ON
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

    void lobee_receiver_impl::dewhiten(const uint8_t *prng) {
        const uint32_t len = d_words_deshuffled.size();

        for (uint32_t i = 0u; i < len; i++) {
            uint8_t xor_b = d_words_deshuffled[i] ^ prng[i];
            d_words_dewhitened.push_back(xor_b);
        }

        #if DEBUG_ON
            print_vector_bin(d_debug, d_words_dewhitened, "W", sizeof(uint8_t) * 8);
        #endif

        d_words_deshuffled.clear();
    }

    void lobee_receiver_impl::hamming_decode(bool is_header) {
        switch(d_phdr.cr) {
            case 4: case 3: { // Hamming(8,4) or Hamming(7,4)
                //hamming_decode_soft(is_header);
                uint32_t n = ceil(d_words_dewhitened.size() * 4.0f / (4.0f + d_phdr.cr));
                uint8_t decoded[n];

                fec_decode(d_h48_fec, n, &d_words_dewhitened[0], decoded);

                if(!is_header)
                    swap_nibbles(decoded, n);
                d_decoded.assign(decoded, decoded+n);

                #if DEBUG_ON
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
    void lobee_receiver_impl::hamming_decode_soft(bool is_header) {
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

    void lobee_receiver_impl::extract_data_only(bool is_header) {
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

    void lobee_receiver_impl::msg_lora_frame(void) {
        uint32_t len = sizeof(loratap_header_t) + sizeof(loraphy_header_t) + d_payload_length;
        uint32_t offset = 0;
        uint8_t buffer[len];
        loratap_header_t loratap_header;

        memset(buffer, 0, sizeof(uint8_t) * len);
        memset(&loratap_header, 0, sizeof(loratap_header));

        loratap_header.rssi.snr = (uint8_t)(10.0f * log10(d_snr) + 0.5);

        offset = gr::lora::build_packet(buffer, offset, &loratap_header, sizeof(loratap_header_t));
        offset = gr::lora::build_packet(buffer, offset, &d_phdr, sizeof(loraphy_header_t));
        offset = gr::lora::build_packet(buffer, offset, &d_decoded[0], d_payload_length);
        if(offset != len) {
            std::cerr << "lobee_receiver_impl::msg_lora_frame: invalid write" << std::endl;
            exit(1);
        }

        pmt::pmt_t payload_blob = pmt::make_blob(buffer, sizeof(uint8_t)*len);
        message_port_pub(pmt::mp("frames"), payload_blob);
    }

    int lobee_receiver_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      (void) noutput_items;
      (void) output_items;

      const unsigned char *input = (unsigned char *) input_items[0];
      
      uint32_t curr_index  = 0; 
      bool preamble_found  = false;

      d_fine_sync         = 0; // Always reset fine sync 

      switch (d_state) {
          case gr::lora::DecoderState::RESET:{
              d_corr_history.clear();
              preamble_found=false;
              d_first_syncword=0;
              d_corr_fails=0;
              d_energy=0;

              // PHASE_BINARY_INDEX_CORRECTION
              d_index_error=0;
              d_curr_bin_index=0;
              d_curr_index=0;
              d_num_header=0;

              d_state = gr::lora::DecoderState::DETECT;

              #if DEBUG_ON
                d_debug << "Next state: DETECT" << std::endl;
              #endif

              break;
          }

          case gr::lora::DecoderState::DETECT: {
              curr_index = max_binary_gradient_idx(input, d_window);
              d_corr_history.insert(d_corr_history.begin(), curr_index);
              if(d_corr_history.size()>REQUIRED_PREAMBLE_CHIRPS){
                d_corr_history.pop_back();
              }

              if(d_corr_history.size()>=REQUIRED_PREAMBLE_CHIRPS){
                d_preamble_idx = d_corr_history[0];

                // Check for discontinuities that exceed some tolerance
                preamble_found = true;
                for (uint32_t i = 1; i < REQUIRED_PREAMBLE_CHIRPS; i++){
                    if (abs(int(d_preamble_idx) - int(d_corr_history[i])) > LORA_PREAMBLE_TOLERANCE){
                        preamble_found = false;
                    }
                }

                if (preamble_found && d_energy>=d_threshold){
                    d_state = gr::lora::DecoderState::SYNC;

                    // if(d_flag==1){
                    //     d_fine_sync=-(int)d_corr_index[1];
                    // }
                    // else{
                    //     d_fine_sync=(int)d_corr_index[0]-(int)d_samples_per_symbol;
                    // }
                    d_fine_sync=-(int)d_decim_factor*(int)curr_index;                   

                    #if DEBUG_ON 
                        d_debug << "D: " << " flag=" << (uint32_t)d_flag << "; corr_zero_index: " << d_corr_index[0] << " corr_zero_max: " << d_corr_index[2] << "; corr_one_index: " << d_corr_index[1] << " corr_one_max: " << d_corr_index[3];
                        d_debug << "; d_fine_sync= " << d_fine_sync << "; curr_index: " << curr_index;
                        d_debug << "; energy= " << std::setprecision(3) << float(d_energy)/d_halfsamp_per_symbol << std::endl;
                        d_debug << "Next state: SYNC" << std::endl;
                    #endif
                }                
              }

              consume_each((int)d_samples_per_symbol+d_fine_sync);

              break;
          }

          case gr::lora::DecoderState::SYNC: {
              d_corr_fails++;
              if(d_corr_fails>=4){
                 d_state = gr::lora::DecoderState::RESET; 
              }
              if(detect_first_syncword(input, d_first_syncword)){
                samples_to_file(d_path+"/data/lobee_detect",  &input[0], 8*d_samples_per_symbol, sizeof(char),false);
                #if DEBUG_ON
                    d_debug << "Next state: FIND_SFD "<< std::endl;
                #endif
                d_state = gr::lora::DecoderState::FIND_SFD;
              }
              else{
                consume_each((int)d_samples_per_symbol+d_fine_sync);
              }

            // int32_t step_length=2;
            // int32_t steps=(4*(int)d_number_of_bins)/step_length;
            // int32_t shift=(int)d_samples_per_symbol-(int)(d_decim_factor*curr_index)-(int)d_number_of_bins;

            // int32_t i;
            // for(i=1;i<=steps;i++)
            // {
            //     #if DEBUG_ON
            //         d_debug << "steps: "<< i << "; shift: " << shift+i*step_length << std::endl;
            //     #endif
            //     if(detect_syncword(input+shift+i*step_length)){
            //         d_fine_sync=shift+i*step_length;
            //         d_state=gr::lora::DecoderState::FIND_SFD;
            //         break;
            //     }
            // }
            // if(i==steps+1){
            //     d_state = gr::lora::DecoderState::RESET;
            //     break;                
            // }

            // consume_each((int32_t)d_samples_per_symbol*2+d_fine_sync);

            //   float cfo = experimental_determine_cfo(&input[i], d_samples_per_symbol);
            //   pmt::pmt_t kv = pmt::cons(pmt::intern(std::string("cfo")), pmt::from_double(cfo));
            //   message_port_pub(pmt::mp("control"), kv);

            break;
          }

          case gr::lora::DecoderState::FIND_SFD: {
              int32_t shift=(d_syncword[0]-d_first_syncword)*(int32_t)d_decim_factor;
              d_fine_sync=(int32_t)d_samples_per_symbol*2+shift;
              
              #if DEBUG_ON
                d_debug << "d_fine_sync= " << d_fine_sync << "; shift= " << shift << std::endl;
              #endif

              if(detect_syncword(input+d_fine_sync)){
                d_state = gr::lora::DecoderState::PAUSE;
              }else{
                d_state = gr::lora::DecoderState::RESET;
              }

              consume_each(d_fine_sync); 

              break;
          }

          case gr::lora::DecoderState::PAUSE: {
              samples_to_file(d_path+"/data/lobee_sync", input, d_samples_per_symbol, sizeof(char),false);
              max_binary_gradient_idx(input, d_window);
              max_binary_gradient_idx(input+d_samples_per_symbol, d_window);
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
                     d_decoded.clear();
                     d_words.clear();
                     d_words_dewhitened.clear();
                     d_words_deshuffled.clear();
                     d_demodulated.clear();
                     break;
                  }

                  samples_to_file(d_path+"/data/lobee_result", &d_decoded[0], d_decoded.size(),sizeof(char),true);
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

                  #if DEBUG_ON
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
                  samples_to_file(d_path+"/data/lobee_result", &d_decoded[0], d_payload_length,sizeof(char),true);
                  msg_lora_frame();

                  d_state = gr::lora::DecoderState::RESET;
                  d_decoded.clear();
                  d_words.clear();
                  d_words_dewhitened.clear();
                  d_words_deshuffled.clear();
                  d_demodulated.clear();
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

    void lobee_receiver_impl::set_sf(const uint8_t sf) {
        (void) sf;
        std::cerr << "[LoRa Decoder] WARNING : Setting the spreading factor during execution is currently not supported." << std::endl
                  << "Nothing set, kept SF of " << d_sf << "." << std::endl;
    }

    void lobee_receiver_impl::set_samp_rate(const float samp_rate) {
        (void) samp_rate;
        std::cerr << "[LoRa Decoder] WARNING : Setting the sample rate during execution is currently not supported." << std::endl
                  << "Nothing set, kept SR of " << d_samples_per_second << "." << std::endl;
    }    

  } /* namespace lora */
} /* namespace gr */

