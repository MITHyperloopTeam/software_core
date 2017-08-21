#ifndef __FIDUCIAL_UTILS_H_
#define __FIDUCIAL_UTILS_H_

#include "zcmgen_c/mithl_fiducial_config_t.h"
#include "zcmgen_c/mithl_fiducial_color_row_t.h"
#include "zcmgen_c/mithl_fiducial_teach_table_t.h"

namespace mithl_fid {

  const uint16_t kSyncWordSend = 0x0055;
  const uint16_t kSyncWordRecv = 0x00AA;

  static const uint8_t crc8Table[] = {
      0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65, 
      157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220, 
      35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98, 
      190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255, 
      70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7, 
      219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154, 
      101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36, 
      248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185, 
      140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205, 
      17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80, 
      175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238, 
      50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115, 
      202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139, 
      87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22, 
      233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168, 
      116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53 };

  /**
   * Compute the checksum on the given array of data
   *
   * See spec, pseudocode: http://www.micro-epsilon.com/download/manuals/man--colorCONTROL-S--en.pdf
   */
  static inline uint8_t calcCRC8(uint8_t * data, size_t length, const uint8_t * table) {
    uint8_t crc8 = 0xAA;
    for (size_t i=0; i<length; i++) {
      uint8_t idx = crc8 ^ data[i];
      crc8 = table[idx];
    }
    return crc8;
  }

  /**
   * Writes a 16-bit word to a 16-bit location using the proper
   * endian ordering (this exists so that it can be changed
   * to support whichever of the two byte-level endiannesses turns
   * out to be needed).
   */
  static inline void write_word(uint16_t word, uint8_t * locationFirst) {
    uint8_t * location8 = (uint8_t *) locationFirst;

    location8[0] = (uint8_t)(word);
    location8[1] = (uint8_t)(word >> 8);
  }

  /**
   * Reads a 16-bit word from a 16-bit location using the proper
   * endian ordering (this exists so that it can be changed
   * to support whichever of the two byte-level endiannesses turns
   * out to be needed).
   */
  static uint16_t read_word(uint16_t * location16) {
    uint8_t * location8 = (uint8_t *) location16;

    uint16_t word = ((uint16_t)(location8[1])) << 8;
    word += location8[0];

    return word;
  }

  static void write_command_buffer_header(uint8_t order, uint16_t arg, uint16_t len, uint8_t dataCRC8, uint8_t * buf) {
    buf[0] = 0x55;
    buf[1] = order;
    buf[2] = (uint8_t)(arg);
    buf[3] = (uint8_t)(arg >> 8);
    buf[4] = (uint8_t)(len);
    buf[5] = (uint8_t)(len >> 8);
    buf[6] = dataCRC8;

    // Calc CRC8 out of header
    buf[7] = calcCRC8(buf, 7, crc8Table);
  }

  static void write_config_command_buffer(mithl_fiducial_config_t * config, uint8_t * buf42Char, unsigned int set_no) {
    // set parameters
    // (generated this automatically with the convenient python list:)
    write_word( config->power         , &(buf42Char[8 + 2*0]) );
    write_word( config->powermode     , &(buf42Char[8 + 2*1]) );
    write_word( config->average       , &(buf42Char[8 + 2*2]) );
    write_word( config->evalmode      , &(buf42Char[8 + 2*3]) );
    write_word( config->hold          , &(buf42Char[8 + 2*4]) );
    write_word( config->intlim        , &(buf42Char[8 + 2*5]) );
    write_word( config->maxcolno      , &(buf42Char[8 + 2*6]) );
    write_word( config->outmode       , &(buf42Char[8 + 2*7]) );
    write_word( config->trigger       , &(buf42Char[8 + 2*8]) );
    write_word( config->exteach       , &(buf42Char[8 + 2*9]) );
    write_word( config->calcmode      , &(buf42Char[8 + 2*10]) );
    write_word( config->dyn_win_lo    , &(buf42Char[8 + 2*11]) );
    write_word( config->dyn_win_hi    , &(buf42Char[8 + 2*12]) );
    write_word( config->color_groups  , &(buf42Char[8 + 2*13]) );
    write_word( config->ledmode       , &(buf42Char[8 + 2*14]) );
    write_word( config->gain          , &(buf42Char[8 + 2*15]) );
    write_word( config->integral      , &(buf42Char[8 + 2*16]) );

    uint8_t dataCRC8 = calcCRC8(&(buf42Char[8]), 34, crc8Table);

    // With dataCRC8 in hand, we can create the header
    write_command_buffer_header(1, set_no, 34, dataCRC8, buf42Char);
    // Explanation of header:
    //   order 1 <--> set config buffer
    //   arg 0 or 1 <--> config set 0 or config set 1 (set_no should have one of these values)
    //   data size is 34 chars
  }

  static void write_teach_table_command_buffer(mithl_fiducial_teach_table_t * teach_table, uint8_t * buf256Char, unsigned int set_no) {
    // write teach table rows
    for (int i=0; i<31; i++) {
      mithl_fiducial_color_row_t row = teach_table->rows[i];

      write_word( row.color_x,    &(buf256Char[8 + (i*16) + 2*0]) );
      write_word( row.color_y,    &(buf256Char[8 + (i*16) + 2*1]) );
      write_word( row.cto,        &(buf256Char[8 + (i*16) + 2*2]) );
      write_word( row.color_int,  &(buf256Char[8 + (i*16) + 2*3]) );
      write_word( row.ito,        &(buf256Char[8 + (i*16) + 2*4]) );
      write_word( row.group,      &(buf256Char[8 + (i*16) + 2*5]) );
      write_word( row.hold,       &(buf256Char[8 + (i*16) + 2*6]) );
      write_word( 0,              &(buf256Char[8 + (i*16) + 2*7]) ); // Dummy 0
    }

    uint8_t dataCRC8 = calcCRC8(&(buf256Char[8]), (31*16), crc8Table);

    // With dataCRC8 in hand, we can create the header
    write_command_buffer_header(1, 2+set_no, (31*16), dataCRC8, buf256Char);
    // Explanation of header:
    //   order 1 <--> set config buffer
    //   arg 2 or 3 <--> teach table set 0 or teach table set 1 (set_no should be 0 or 1)
    //   data size is 8 + (31*8) chars
  }


  /**
   * Returns true if received buffer header (only) matches expected format
   * (0x55 head byte, header CRC8 check) and false otherwise.
   *
   * @param buf data frame to read; 8 bytes header.
   * @param buffer_max_len maximum length of the buffer array we are holding
   *    the bytes in, to prevent bad memory access.
   * @return returns true/false success state of parsing.
   */
  bool is_valid_receive_header(uint8_t * buf, size_t buffer_max_len) {
    // check at least 8 bytes long (header size) before we try to access header values
    if (buffer_max_len < 8) {
      return false;
    }

    // check for valid start char
    if (buf[0] != 0x55) {
      return false;
    }

    uint8_t order = buf[1];
    uint16_t arg = (((uint16_t)buf[3]) << 8) + ((uint16_t)buf[2]);
    uint16_t len = (((uint16_t)buf[5]) << 8) + ((uint16_t)buf[4]);
    uint8_t dataCRC8 = buf[6];
    uint8_t headerCRC8 = buf[7];

    // check header CRC
    uint8_t true_headerCRC8 = calcCRC8(buf, 7, crc8Table);
    if (headerCRC8 != true_headerCRC8) {
      return false;
    }

    // check reported length against buffer length before we try to access data
    if (len + 8 > buffer_max_len) {
      return false;
    }

    // CRC checks out, we are all-clear
    return true;
  }

  /**
   * Returns true if received buffer matches expected format
   * (0x55 head byte, CRC8 checks on header AND data) and false otherwise.
   *
   * @param buf data frame to read, 8 bytes header + len bytes data
   * @param buffer_max_len maximum length of the buffer array we are holding
   *    the bytes in, to prevent bad memory access.
   * @return returns true/false success state of parsing
   */
  bool is_valid_receive_buffer(uint8_t * buf, size_t buffer_max_len) {
    if (!is_valid_receive_header(buf,buffer_max_len)) {
      return false;
    }

    uint16_t len = (((uint16_t)buf[5]) << 8) + ((uint16_t)buf[4]);
    uint8_t dataCRC8 = buf[6];

    // check data CRC
    uint8_t true_dataCRC8 = calcCRC8(&(buf[8]), len, crc8Table);
    if (dataCRC8 != true_dataCRC8) {
      return false;
    }

    // CRCs check out, we are all-clear
    return true;
  }

  /**
   * Returns true if buffer matches expected format (0x00AA header, etc.) and parsing
   * was successful, false otherwise. Note that it won't check the integrity of the
   * actual config parameters received, just that of the surrounding metadata.
   *
   * If true, then config will be populated with the parsed configuration.
   * 
   * If false, then no guarantees are placed on the final state of config.
   * 
   * @param buf36Char data frame of 36 bytes to be read. Assume all 36 locations will
   *    be accessed.
   * @param config pointer to a config stuct which will be filled appropriately
   * @return returns true/false success state of parsing
   */
  static bool parse_config_command_buffer(uint8_t * buf36Char, mithl_fiducial_config_t * config) {
    return false;
  /*
    // switch to 16-bit version of buffer. Macros will ensure
    // proper endianness.
    uint16_t * buf18Word = (uint16_t *)buf36Char;

    // check some basic formatting of the buffer. If problem detected, no need to
    // continue with parsing.
    if (read_word(&(buf18Word[0])) != 0x00AA) return false;
    if (!(read_word(&(buf18Word[1])) == 0x0001 || read_word(&(buf18Word[1])) == 0x0002))
        return false;
    if (read_word(&(buf18Word[17])) != 0x0000) return false;

    // set parameters
    // (generated this automatically with the convenient python list:)
    // config->A = read_word( &(buf18Word[2+B]) );
    config->power =        read_word( &(buf18Word[2+0]) );
    config->powermode =    (int8_t)read_word( &(buf18Word[2+1]) );
    config->average =      read_word( &(buf18Word[2+2]) );
    config->evalmode =     (int8_t)read_word( &(buf18Word[2+3]) );
    config->hold =         (int8_t)read_word( &(buf18Word[2+4]) );
    config->intlim =       read_word( &(buf18Word[2+5]) );
    config->maxcolno =     (int8_t)read_word( &(buf18Word[2+6]) );
    config->outmode =      (int8_t)read_word( &(buf18Word[2+7]) );
    config->trigger =      (int8_t)read_word( &(buf18Word[2+8]) );
    config->exteach =      (int8_t)read_word( &(buf18Word[2+9]) );
    config->calcmode =     (int8_t)read_word( &(buf18Word[2+10]) );
    config->dyn_win_lo =   read_word( &(buf18Word[2+11]) );
    config->dyn_win_hi =   read_word( &(buf18Word[2+12]) );
    config->color_groups = (int8_t)read_word( &(buf18Word[2+13]) );
    config->integral =     read_word( &(buf18Word[2+14]) );

    // parse was successful. Return true.
    return true; */
  }

  /**
   * Check whether the given mithl_fiducial_config_t has valid
   * values according to the spec.
   *
   * @param config  pointer to a config stuct which will check
   * @return returns true/false validity of config
   */
  static bool is_valid_config(mithl_fiducial_config_t * config) {
    if (config->power < 0 || config->power > 1000) return false;
    if (config->powermode < 0 || config->powermode > 1) return false;

    // "AVERAGE": ensure config->average is a power of 2
    //uint16_t uaverage = (uint16_t)config->average; // TODO: Test this and use bithack; until then, switch..case
    //if (uaverage==0 || !(((uaverage & (uaverage-1)) == 0))) return false;
    switch (config->average) {
      case 1:
      case 2:
      case 4:
      case 8:
      case 16:
      case 32:
      case 64:
      case 128:
      case 256:
      case 512:
      case 1024:
      case 2048:
      case 4096:
      case 8192:
      case 16384:
      case 32768:
      break;
      default:
      return false;
    }

    if (config->evalmode < 0 || config->evalmode > 3) return false;    
    if (config->hold < 0 || config->hold > 100) return false;
    if (config->intlim < 0 || config->intlim > 4095) return false;
    if (config->maxcolno < 1 || config->maxcolno > 31) return false;
    if (config->outmode < 0 || config->outmode > 2) return false;
    if (config->trigger < 0 || config->trigger > 6) return false;
    if (config->exteach < 0 || config->exteach > 3) return false;
    if (config->calcmode < 0 || config->calcmode > 3) return false;
    if (config->dyn_win_lo < 0 || config->dyn_win_lo > 4095) return false;
    if (config->dyn_win_hi < 0 || config->dyn_win_hi > 4095) return false;
    if (config->dyn_win_lo > config->dyn_win_hi) return false;
    if (config->color_groups < 0 || config->color_groups > 1) return false;
    if (config->ledmode < 0 || config->ledmode > 3) return false;
    if (config->gain < 1 || config->gain > 8) return false;
    if (config->integral < 1 || config->integral > 250) return false;

    return true;
  }

  static bool equal_sensor_configs(mithl_fiducial_config_t * config1, mithl_fiducial_config_t * config2) {
    bool are_equal = true;
    are_equal &= ( config1->power == config2->power);
    are_equal &= ( config1->powermode == config2->powermode);
    are_equal &= ( config1->average == config2->average);
    are_equal &= ( config1->evalmode == config2->evalmode);
    are_equal &= ( config1->hold == config2->hold);
    are_equal &= ( config1->intlim == config2->intlim);
    are_equal &= ( config1->maxcolno == config2->maxcolno);
    are_equal &= ( config1->outmode == config2->outmode);
    are_equal &= ( config1->trigger == config2->trigger);
    are_equal &= ( config1->exteach == config2->exteach);
    are_equal &= ( config1->calcmode == config2->calcmode);
    are_equal &= ( config1->dyn_win_lo == config2->dyn_win_lo);
    are_equal &= ( config1->dyn_win_hi == config2->dyn_win_hi);
    are_equal &= ( config1->color_groups == config2->color_groups);
    are_equal &= ( config1->ledmode == config2->ledmode);
    are_equal &= ( config1->gain == config2->gain);
    are_equal &= ( config1->integral == config2->integral);

    return are_equal;
  }

  static bool equal_teach_tables(mithl_fiducial_teach_table_t * table1, mithl_fiducial_teach_table_t * table2, int maxcolno) {
    bool are_equal = true;

    for (int i=0; i<maxcolno; i++) {
      mithl_fiducial_color_row_t row1 = table1->rows[i];
      mithl_fiducial_color_row_t row2 = table2->rows[i];

      are_equal &= (row1.color_x == row2.color_x);
      are_equal &= (row1.color_y == row2.color_y);
      are_equal &= (row1.cto == row2.cto);
      are_equal &= (row1.color_int == row2.color_int);
      are_equal &= (row1.ito == row2.ito);
      are_equal &= (row1.group == row2.group);
    }

    return are_equal;
  }
}

#endif // __FIDUCIAL_UTILS_H_

