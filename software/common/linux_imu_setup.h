#ifndef IMU_SETUP_H
#define IMU_SETUP_H

#include "SerialStream.h"
#include <stdio.h>

int serial_setup(SerialStream* stream, int baud_rate, char port[]) {

  serial->Open(port);

  if ( ! serial->good() ) {
    printf("failed to open port\n");
    return 0;
  }

  serial->SetBaudRate( SerialStreamBuf::BAUD_115200 );
  if ( ! serial->good() ) {
    printf("Error: Could not set the baud rate.\n");
    return 0;
  }

  // Set the number of data bits.
  serial->SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
  if ( ! serial->good() ) 
  {
    printf("Error: Could not set the character size.\n");
    return 0;
  }
  //
  // Disable parity.
  //
  serial->SetParity( SerialStreamBuf::PARITY_NONE ) ;
  if ( ! serial->good() ) 
  {
    printf("Error: Could not disable the parity.\n");
    return 0;
  }
  //
  // Set the number of stop bits.
  //
  serial->SetNumOfStopBits( 1 ) ;
  if ( ! serial->good() ) 
  {
    printf("Error: Could not set the number of stop bits.\n");
    return 0;
  }
  return 1;
}

#endif // IMU_SETUP_H