# LigBee
This repository holds the artifact of the paper "LigBee: Symbol-Level Cross-Technology Communication from LoRa to ZigBee", published on INFOCOM 2023.

------

## Overall
This repo contains the artifact of the LigBee paper, which mainly consists of two parts, LigBee-USRP and LigBee-COTS implementation. 

## System Specification
### Environment
Our development environment mainly include several laptops running Ubuntu18.04 (for USRP) or  Win10  (for COTS) . 

### Hardware 
The hardware used include USRP N210 with SBX daughterboard and WSTK6061B plus EFR32FG14 proprietary radio board manufactured by Silicon Labs.

### Software 
For USRP implementation, we use GNURadio. For COTS implementation, we adopt SimplicityStudio provided by Silicon Labs.

### Top-layer Layout
```
├── README.md            # Introduction of artifact
├── ligbee-usrp          # USRP implementation
├── ligbee-cots          # COTS implementation
└── figures
```

## LigBee-USRP
### Introduction
We implement the LigBee module at `gr-lobee-encoder`. 
The GNU Radio Companion (GRC) is shown below:
<img src="/figures/usrp/ligbee-rx.png" alt="ligbee-usrp-rx">
As a comparison, the standard ZigBee RX works as follows:
<img src="/figures/usrp/zigbee-rx.png" alt="ligbee-usrp-rx">
It can be observed that LigBee takes chip sequence as input and output the decoded LoRa packet. The only difference is that LigBee uses `quad-to-byte` to covert phase shift to chip sequence, while ZigBee RX takes phase shift as input directly.

### Layout
```
└── ligbee-usrp         
    ├── gr-lobee          # Convert phase shift to chip sequence.
    │   ├── include
    │   │   ├── quad_to_byte_fb.h
    │   └── lib
    │       ├── quad_to_byte_fb_impl.h
    │       └── quad_to_byte_fb_impl.cc
    ├── gr-lobee-encoder  # Core of LigBee module, input: chip, output: LoRa packet.
    │   ├── analysis      # Python tool for manipulating LoRa node and analyze results.
    │   ├── examples      # GRC with configured parameter for LigBee test.
    |   |   └── rpp0_lorabee_usrp.grc
    │   ├── include
    │   │   ├── lobee_receiver.h
    │   └── lib
    │       ├── lobee_receiver_impl.h
    │       └── lobee_receiver_impl.cc
    └── python-loranode  # Python lib to manipulate COTS LoRa node
```

### Running 
The following dependencies are required: `python2-numpy`, `python2-scipy`, `swig`, `cppunit`, `fftw`, `gnuradio`, `libvolk`, `log4cpp`, `cmake`, `wx`, `UHD`, and [`liquid-dsp`](https://github.com/jgaeddert/liquid-dsp).

One needs to install the above two modules `gr-lobee` and `gr-lobee-encoder` to run the above GRC. The installed module will be shown as following:
<img src="/figures/usrp/ligbee-module.png" alt="ligbee-usrp-module">

Beside, our python script to manipulate LoRa node relies on the crafted `python-loranode`.

## LigBee-COTS

### Introduction
We implement the LigBee module at `online-lib`. Since COTS implementation is device specific, we present LigBee as the simplest lib here.
We also provide `offline-demo` to show how to incorporate LigBee module using offline Trace. 

### Layout
```
└── ligbee-cots         
    ├── offline-demo          # Offline demo of LigBee
    │   ├── chirp_decode.h 
    │   ├── chirp_decode.c    # Core of LigBee module, chirp decoding
    │   ├── reverse_decode.h  
    │   ├── reverse_decode.c  # Core of LigBee module, reverse encoding
    │   ├── util.h
    │   ├── queue.h
    │   └── main.c            # For offline test only 
    └── online-lib            # Online libiary of COTS deployment
        ├── chirp_decode.h 
        ├── chirp_decode.c    
        ├── reverse_decode.h  
        ├── reverse_decode.c  
        ├── util.h
        └── queue.h
```

### Running

In the `offfline-demo`, we present how `online-lib` decode chip sequency using offline trace.

Next, we focus on how to incorporate `online-lib` into devices, taking `SimplicityStudio` and `WSTK6061B` as an example:

- One needs to copy `online-lib` into your project directory and adds `online-lib` to the project compiling list as well:

<img src="/figures/cots/ligbee-module.png" alt="ligbee-cots-module">  

- After that, the implementation of `initializeDecode` and `decodePacket` is required using `online-lib`, which is device specific.

- As per each packet is receiver, the `decodePacket` is invoked to decode LoRa packet from chip sequence.

A demonstration code is shown as below:

```
  // Initializing packet decoding
  initializeDecode();

  printf("BOOTING....\n");

  while (1) {
    if (packetTx) {
      packetTx = false;
      txLength = TX_PACKET_LENGTH;
      printf("TX(%d): ", txLength);
//      printBuffer(txPtr, txLength);
      printf("\n");
      sendPacket(txLength);
    }
    if (packetRx) {
      packetRx = false;
      if (packetReceived){
//    	  const unsigned char * packetSamples = (const unsigned char *) rxPtr;
    	  decodePacket(rxPtr, d_decim, d_gradient);
    	  packetReceived = false;
      }
      startReceive();
    }
  }
```

## Acknowledgement

Very grateful to the authors of the following repos, from which the USRP implementation is developed:

- Pieter Robyns [gr-lora](https://github.com/rpp0/gr-lora) [python-loranode](https://github.com/rpp0/python-loranode)
- Matt Knight [gr-lora](https://github.com/BastilleResearch/gr-lora)

Thanks to their great efforts to develop effective tools to manipulate COTS LoRa and reverse engineering LoRa.

We also thank the technique supporters from Silicon Labs, who provide valuable suggestions to our COTS implementation.





