#include "IMUParse.h"

IMUParse::IMUParse() {
  buf_ptr_ = 0;
  collecting_packet_ = true;
  have_good_packet_ = false;

  for (int i=0; i<IMU_PACKET_NUM_BYTES; i++)
    packet_buf_[i] = 0;
  memcpy(good_packet_store_, packet_buf_, IMU_PACKET_NUM_BYTES*sizeof(unsigned char));
}

bool IMUParse::new_byte(unsigned char byte){

  // find the start of a packet
  if(!collecting_packet_){
    if(byte == 0xFA){
      collecting_packet_ = true;
      buf_ptr_ = 0;
    }
  }

  // fill up our buffer
  else if(collecting_packet_){
    packet_buf_[buf_ptr_] = byte;
    buf_ptr_ += 1;

    // Filled buffer! Try to parse.
    if (buf_ptr_ == IMU_PACKET_NUM_BYTES){
      collecting_packet_ = false;
      buf_ptr_ = 0;
    
      if(calc_checksum(packet_buf_, IMU_PACKET_NUM_BYTES) == 0){
        have_good_packet_ = true;
        memcpy(good_packet_store_, packet_buf_, IMU_PACKET_NUM_BYTES*sizeof(unsigned char));
      } else {
        return false;
      }
    }
  }

  return true;
}

bool IMUParse::parse(float return_data[], int num_fields){
  union {
    char c[4];
    float f;
  } u;

  // start at 3 b/c first 3 bytes are setup info
  int ptr = 3;
  for (int ii=0; ii < 6; ii++){

    for (int jj=0; jj < 4; jj++)
      u.c[jj] = good_packet_store_[ptr++];

    return_data[ii] = u.f;
  }

  have_good_packet_ = false;
  
  return true;
}

uint16_t IMUParse::calc_checksum(unsigned char data[], uint16_t length)
{
  unsigned int i;
  unsigned short crc = 0;
  for(i=0; i<length; i++){
    crc = (unsigned char)(crc >> 8) | (crc << 8);
    crc ^= data[i];
    crc ^= (unsigned char)(crc & 0xff) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0x00ff) << 5;
  }
  return crc;
}