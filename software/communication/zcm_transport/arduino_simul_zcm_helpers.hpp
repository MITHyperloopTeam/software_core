
#ifndef ARDUINO_SIMUL_ZCM_HELPERS
#define ARDUINO_SIMUL_ZCM_HELPERS

#ifdef COMPILE_FOR_ARDUINO
#include "arduino_cobs_serial_transport.hpp"
#endif
#ifdef COMPILE_FOR_SIMUL
#include "zcm/zcm.h"
#endif

#ifdef COMPILE_FOR_ARDUINO
  #define ARDUINO_SIMUL_CREATE_ZCM(ZCM_NAME, PORT) \
        ZCM_NAME = zcm_create_trans(arduino_transport_create(PORT));

  #define ARDUINO_SIMUL_ZCM_START(ZCM_NAME)

  #define ARDUINO_SIMUL_ZCM_HANDLE(ZCM_NAME) \
        zcm_handle_nonblock(ZCM_NAME);

#endif

#ifdef COMPILE_FOR_SIMUL
    // note: I don't think, according to wireshark, the port is being
    // used properly. It actually is addressed to port 62237.
    // for now I'm gonna be OK with that...
    #define ARDUINO_SIMUL_CREATE_ZCM(ZCM_NAME, PORT) \
        ZCM_NAME = zcm_create("udpm://239.255.76.67:7667?ttl=0");

    #define ARDUINO_SIMUL_ZCM_START(ZCM_NAME) {zcm_start(ZCM_NAME);}

    #define ARDUINO_SIMUL_ZCM_HANDLE(ZCM_NAME) {;}
#endif  

#endif