#ifndef IMU_PARSE_H
#define IMU_PARSE_H

#define IMU_PACKET_NUM_BYTES 29

#include <stdint.h>
#include <string.h>

class IMUParse {

private:

  bool collecting_packet_;
  int buf_ptr_;
  bool have_good_packet_;
  unsigned char good_packet_store_[IMU_PACKET_NUM_BYTES];

  uint16_t calc_checksum(unsigned char data[], uint16_t length);

public:
  IMUParse();
  bool new_byte(unsigned char byte); // adds byte to buffer
  bool ready() {return have_good_packet_;}; // returns true when 
  bool parse(float return_data[], int num_fields);  // returns 1 on success, 0 on fail

  unsigned char packet_buf_[IMU_PACKET_NUM_BYTES];
};


#endif // IMU_PARSE_H