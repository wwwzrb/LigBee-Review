# LigBee-Review
Anonymous repo for the peer review of INFOCOM23. 

------

## Overall
This repo contains the artifact of the LigBee paper, which mainly consists of two parts, LigBee-USRP and LigBee-COTS implementation. 

## System Specification
### Environment
Our development environment mainly includes several laptops running Ubuntu18.04 (for USRP) or  Win10  (for COTS) . 

### Hardware 
The hardware used includes USRP N210 with SBX daughterboard and WSTK6061B plus EFR32FG14 proprietary radio board manufactured by Silicon Labs.

### Software 
For USRP implementation, we use GNURadio. For COTS implementation, we adopt SimplicityStudio provided by Silicon Labs.

### Top-layer Layout
```
├── README.md            # Introduction of artifact
├── ligbee-usrp          # USRP implementation
├── ligbee-cots          # COTS implementation
└── figures              # Please refer to '/figures' for all figures in README, since anonymous github may not support displaying figures.
```

## LigBee-USRP
### Introduction
We implement the LigBee module at `gr-lobee-encoder`. 

The GNU Radio Companion (GRC) of LigBee is shown below:
<img src="/figures/usrp/ligbee-rx.png" alt="/figures/usrp/ligbee-rx.png">



As a comparison, the standard ZigBee RX works as follows:
<img src="/figures/usrp/zigbee-rx.png" alt="/figures/usrp/zigbee-rx.png">



It can be observed that LigBee takes chip sequence as input and outputs the decoded LoRa packet. The only difference is that LigBee uses `quad-to-byte` to covert phase shift to chip sequence, while ZigBee RX takes phase shift as input and outputs the decoded ZigBee packet.

### Layout
```
└── ligbee-usrp         
    ├── gr-lobee          # Convert phase shift to chip sequence.
    │   ├── include/lobee
    │   │   ├── quad_to_byte_fb.h
    │   └── lib
    │       ├── quad_to_byte_fb_impl.h
    │       └── quad_to_byte_fb_impl.cc
    ├── gr-lobee-encoder  # Core of LigBee module, input: chip, output: LoRa packet.
    │   ├── analysis      # Python tool for manipulating LoRa node and analyze results.
    │   ├── examples      # GRC with configured parameter for LigBee test.
    |   |   └── rpp0_lorabee_usrp.grc
    │   ├── include/lora
    │   │   ├── lobee_receiver.h
    │   └── lib
    │       ├── lobee_receiver_impl.h
    │       └── lobee_receiver_impl.cc
    └── python-loranode  # Python lib to manipulate COTS LoRa node
```

### Running 
The following dependencies are required: `python2-numpy`, `python2-scipy`, `swig`, `cppunit`, `fftw`, `gnuradio`, `libvolk`, `log4cpp`, `cmake`, `wx`, `UHD`, and [`liquid-dsp`](https://github.com/jgaeddert/liquid-dsp).

One needs to install the `gr-lobee` and `gr-lobee-encoder` modules  to run the above GRC. The installed module will be shown as following:
<img src="/figures/usrp/ligbee-module.png" alt="/figures/usrp/ligbee-module.png">



Beside, our python script to manipulate LoRa node relies on `python-loranode`.

## LigBee-COTS

### Introduction
We implement the LigBee module at `online-lib`. 

Since COTS implementation is device specific, we present LigBee as the simplest lib here.

We also provide `offline-demo` to present how to incorporate LigBee module using offline Trace. 

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

In the `offline-demo`, we present how `online-lib` decode chip sequence using offline trace.

Next, we focus on how to incorporate `online-lib` into devices, taking `SimplicityStudio` and `WSTK6061B` as an example:

- One needs to copy `online-lib` into your project directory and adds `online-lib` to the project compiling list as well:

<img src="/figures/cots/ligbee-module.png" alt="/figures/cots/ligbee-module.png">  



- After that, the implementation of `initializeDecode` and `decodePacket` is required using `online-lib`, which is device specific.

- As per each packet is received, the `decodePacket` is invoked to decode LoRa packet from chip sequence.

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

The USRP implementation referred to the following repos:

- Pieter Robyns [gr-lora](https://github.com/rpp0/gr-lora) [python-loranode](https://github.com/rpp0/python-loranode)
- Matt Knight [gr-lora](https://github.com/BastilleResearch/gr-lora)

Very grateful to the authors of the above repos! Thanks for their great efforts to reverse engineering LoRa and develop effective tools to manipulate COTS LoRa. 

We also thank the technique supporters from Silicon Labs, who provide valuable suggestions to our COTS implementation.







