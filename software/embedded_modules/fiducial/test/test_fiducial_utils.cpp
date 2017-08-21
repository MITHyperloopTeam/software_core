/*
 * Tests fiducial/fiducial_utils.hpp methods.
 *
 * Assert-based testing (ie. useless if run with NDEBUG=1)
 */

#include <assert.h>

#include "../fiducial_utils.hpp"
#include "zcmgen_c/mithl_fiducial_config_t.h"

char MODULE_NAME[] = "FD_TEST_NOTREALLYSIM";
int myid = 99989;

static void test_write_config_msg() {
  // ['power', 'powermode', 'average', 'evalmode', 'hold', 'intlim', 'maxcolno', 'outmode', 'trigger', 'exteach', 'calcmode', 'dyn_win_lo', 'dyn_win_hi', 'color_groups', 'integral']
/*  mithl_fiducial_config_t sample_config;

  sample_config.power = 0x0200 + 1;
  sample_config.powermode = 2;
  sample_config.average = 3;
  sample_config.evalmode = 4;
  sample_config.hold = 5;
  sample_config.intlim = 6;
  sample_config.maxcolno = 7;
  sample_config.outmode = 8;
  sample_config.trigger = 9;
  sample_config.exteach = 10;
  sample_config.calcmode = 11;
  sample_config.dyn_win_lo = 12;
  sample_config.dyn_win_hi = 13;
  sample_config.color_groups = 14;
  sample_config.integral = 15;

  uint16_t buf[18];
  mithl_fid::write_config_command_buffer(&sample_config, buf);
  uint8_t * buf8 = (uint8_t*)buf;

  // Debug prints:
  for (int i=0; i<36; i++)
    printf("%02x", buf8[i]);
  printf("\n");

  // Check values
  assert(buf8[2*0 + 0] == 0x00);
  assert(buf8[2*0 + 1] == 0x55);
  assert(buf8[2*1 + 0] == 0x00);
  assert(buf8[2*1 + 1] == 0x01);
  assert(buf8[2*2 + 0] == 0x02);
  assert(buf8[2*2 + 1] == 0x01);
  for (int i=3; i<15; i++) {
    assert(buf8[2*i + 0] == 0);
    assert(buf8[2*i + 1] == i-1);
  } */
}

static void test_read_config_msg() {
  // ['power', 'powermode', 'average', 'evalmode', 'hold', 'intlim', 'maxcolno', 'outmode', 'trigger', 'exteach', 'calcmode', 'dyn_win_lo', 'dyn_win_hi', 'color_groups', 'integral']
/*  uint8_t buf8[36];
  buf8[0] = 0x00;
  buf8[1] = 0xAA;
  buf8[2] = 0x00;
  buf8[3] = 0x01;
  buf8[4] = 0x02;
  buf8[5] = 0x01;
  for (int i=3; i<18; i++) {
    buf8[2*i + 0] = 0;
    buf8[2*i + 1] = (i-1) % 2;
  }
  buf8[34] = 0x00;
  buf8[35] = 0x00;

  // Debug prints:
  for (int i=0; i<36; i++)
    printf("%02x", buf8[i]);
  printf("\n");

  // Check values:  
  mithl_fiducial_config_t sample_config;

  bool result = mithl_fid::parse_config_command_buffer( buf8, &sample_config );

  assert(result);
  assert(sample_config.power == 0x0200 + 1);
  assert(sample_config.powermode == 2 % 2);
  assert(sample_config.average == 3 % 2);
  assert(sample_config.evalmode == 4 % 2);
  assert(sample_config.hold == 5 % 2);
  assert(sample_config.intlim == 6 % 2);
  assert(sample_config.maxcolno == 7 % 2);
  assert(sample_config.outmode == 8 % 2);
  assert(sample_config.trigger == 9 % 2);
  assert(sample_config.exteach == 10 % 2);
  assert(sample_config.calcmode == 11 % 2);
  assert(sample_config.dyn_win_lo == 12 % 2);
  assert(sample_config.dyn_win_hi == 13 % 2);
  assert(sample_config.color_groups == 14 % 2);
  assert(sample_config.integral == 15 % 2);

  // ORDER==2 is okay, too
  buf8[2*1 + 0] = 0x00;
  buf8[2*1 + 1] = 0x02;
  bool success_with_2 = mithl_fid::parse_config_command_buffer( buf8, &sample_config );
  assert(success_with_2); */
}

static void test_checksum_maker() {
  {
    uint8_t A6_4_1Header[] = { 85, 1, 0, 0, 34, 0, 162 };
    uint8_t chksum = mithl_fid::calcCRC8(A6_4_1Header, 7, mithl_fid::crc8Table);
    printf(" %d", chksum);
    assert(chksum == 249);
    printf(" OK\n");
  }

  {
    uint8_t A6_4_2Header[] = { 85, 2, 0, 0, 0, 0, 170 };
    uint8_t chksum = mithl_fid::calcCRC8(A6_4_2Header, 7, mithl_fid::crc8Table);
    printf(" %d", chksum);
    assert(chksum == 185);
    printf(" OK\n");
  }

  {
    uint8_t A6_4_1Data[] = { 244, 1, 0, 0, 1, 0, 1, 0, 10, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 2, 0, 128, 12, 228, 12, 0, 0, 1, 0, 8, 0, 1, 0 };
    uint8_t chksum = mithl_fid::calcCRC8(A6_4_1Data, 34, mithl_fid::crc8Table);
    printf(" %d", chksum);
    assert(chksum == 162);
    printf(" OK\n");
  }

  {
    uint8_t A6_4_1Response[] = { 85, 1, 0, 0, 0, 0, 170 };
    uint8_t chksum = mithl_fid::calcCRC8(A6_4_1Response, 7, mithl_fid::crc8Table);
    printf(" %d", chksum);
    assert(chksum == 224);
    printf(" OK\n");
  }
}

static void test_config_checker() {
  printf("TODO: IMPLEMENT test_config_checker. Especially test config->average checks.\n");
}

static void test_receive_checker() {
  size_t max_len = 400;
  uint8_t buf[max_len];

  { // #1
    uint16_t len = 50;

    for (int i=0; i<len && i<max_len; i++) {
      buf[8+i] = (uint8_t)i;
    }
    uint8_t dataCRC8 = mithl_fid::calcCRC8(&(buf[8]), len, mithl_fid::crc8Table);
    mithl_fid::write_command_buffer_header(1, 1, len, dataCRC8, buf);

    bool is_valid = mithl_fid::is_valid_receive_buffer(buf, max_len);
    bool correct = (is_valid == true);

    assert(correct);
    if (correct) {
      printf("  OK\n");
    } else {
      printf("  FAILED\n");
    }
  }

  { // #2
    uint16_t len = 500; // Choose impossibly long length

    for (int i=0; i<len && i<max_len; i++) {
      buf[8+i] = (uint8_t)i;
    }
    uint8_t dataCRC8 = mithl_fid::calcCRC8(&(buf[8]), len, mithl_fid::crc8Table);
    mithl_fid::write_command_buffer_header(1, 1, len, dataCRC8, buf);

    bool is_valid = mithl_fid::is_valid_receive_buffer(buf, max_len);
    is_valid = is_valid || mithl_fid::is_valid_receive_header(buf, max_len);
    bool correct = (is_valid == false);

    assert(correct);
    if (correct) {
      printf("  OK\n");
    } else {
      printf("  FAILED\n");
    }
  }

  { // #3
    uint16_t len = 50;

    for (int i=0; i<len && i<max_len; i++) {
      buf[8+i] = (uint8_t)i;
    }
    uint8_t dataCRC8 = mithl_fid::calcCRC8(&(buf[8]), len, mithl_fid::crc8Table);
    mithl_fid::write_command_buffer_header(1, 1, len, dataCRC8, buf);

    // Mess with CRC8 for header
    buf[7] = buf[7] + 1;

    bool is_valid = mithl_fid::is_valid_receive_buffer(buf, max_len);
    is_valid = is_valid || mithl_fid::is_valid_receive_header(buf, max_len);
    bool correct = (is_valid == false);

    assert(correct);
    if (correct) {
      printf("  OK\n");
    } else {
      printf("  FAILED\n");
    }
  }

  { // #4
    uint16_t len = 50;

    for (int i=0; i<len && i<max_len; i++) {
      buf[8+i] = (uint8_t)i;
    }
    uint8_t dataCRC8 = mithl_fid::calcCRC8(&(buf[8]), len, mithl_fid::crc8Table);
    mithl_fid::write_command_buffer_header(1, 1, len, dataCRC8, buf);

    // Mess with CRC8 for data
    buf[6] = buf[6] + 1;

    bool is_valid = mithl_fid::is_valid_receive_buffer(buf, max_len);
    is_valid = is_valid || mithl_fid::is_valid_receive_header(buf, max_len);
    bool correct = (is_valid == false);

    assert(correct);
    if (correct) {
      printf("  OK\n");
    } else {
      printf("  FAILED\n");
    }
  }

  { // #5
    // Try to incur segfault with massive length
    buf[0] = 0x55;
    buf[1] = 1;
    buf[2] =  1;
    buf[3] = 0;
    buf[4] =  254;
    buf[5] = 254; // corresponds to len = 254*256 + 254 = 65278
    buf[6] = 0;

    // Calc CRC8 out of header
    buf[7] = mithl_fid::calcCRC8(buf, 7, mithl_fid::crc8Table);

    bool is_valid = mithl_fid::is_valid_receive_buffer(buf, max_len);
    is_valid = is_valid || mithl_fid::is_valid_receive_header(buf, max_len);
    bool correct = (is_valid == false);

    assert(correct);
    if (correct) {
      printf("  OK\n");
    } else {
      printf("  FAILED\n");
    }
  }

  { // #6
    uint16_t len = 50;

    for (int i=0; i<len && i<max_len; i++) {
      buf[8+i] = (uint8_t)i;
    }
    uint8_t dataCRC8 = mithl_fid::calcCRC8(&(buf[8]), len, mithl_fid::crc8Table);
    mithl_fid::write_command_buffer_header(1, 1, len, dataCRC8, buf);

    // Mess with the data that is being CRC8 checked
    buf[8+3] = buf[8+3] + 1;

    bool is_valid = mithl_fid::is_valid_receive_buffer(buf, max_len);
    is_valid = is_valid || !(mithl_fid::is_valid_receive_header(buf, max_len)); // header should be fine, though
    bool correct = (is_valid == false);

    assert(correct);
    if (correct) {
      printf("  OK\n");
    } else {
      printf("  FAILED\n");
    }
  }

  { // #7
    uint16_t len = 0; // len==0 is perfectly valid

    uint8_t dataCRC8 = mithl_fid::calcCRC8(&(buf[8]), len, mithl_fid::crc8Table);
    mithl_fid::write_command_buffer_header(1, 1, len, dataCRC8, buf);

    bool is_valid = mithl_fid::is_valid_receive_buffer(buf, max_len);
    is_valid = is_valid && mithl_fid::is_valid_receive_header(buf, max_len);
    bool correct = (is_valid == true);

    assert(correct);
    if (correct) {
      printf("  OK\n");
    } else {
      printf("  FAILED\n");
    }
  }

  { // #8
    uint16_t len = max_len - 7; // len==max_len-7 is invalid by one entry

    uint8_t dataCRC8 = mithl_fid::calcCRC8(&(buf[8]), len, mithl_fid::crc8Table);
    mithl_fid::write_command_buffer_header(1, 1, len, dataCRC8, buf);

    bool is_valid = mithl_fid::is_valid_receive_buffer(buf, max_len);
    is_valid = is_valid || mithl_fid::is_valid_receive_header(buf, max_len);
    bool correct = (is_valid == false);

    assert(correct);
    if (correct) {
      printf("  OK\n");
    } else {
      printf("  FAILED\n");
    }
  }

}

int main() {
  printf("Testing checksum...\n");
  test_checksum_maker();

  printf("Testing receive buffer checker...\n");
  test_receive_checker();

  return 0; // all tests passed!
}
