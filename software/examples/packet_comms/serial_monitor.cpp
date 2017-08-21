/** 
**/

#include "../../externals/PacketSerial/src/PacketSerialLinux.h"
#include <stdint.h>

PacketSerial_<COBS, 0, 256> serial;

typedef struct temp_report_ {
   float temp;
} temp_report;

// This is our packet callback.
// The buffer is delivered already decoded.
void onPacket(const uint8_t* buffer, size_t size, void * extra)
{
  // Make a temporary buffer.
  char tmp[100];
  // Send a string "I don't know what's going on"
  printf("Received:[%s] -> %f\n", buffer, ((temp_report*)buffer)->temp);
}

int
main( int    argc,
      char** argv  )
{
    serial.setPacketHandler(&onPacket, 0);
    serial.begin(115200, "/dev/ttyACM0");
    while (1)
      serial.update();
}
