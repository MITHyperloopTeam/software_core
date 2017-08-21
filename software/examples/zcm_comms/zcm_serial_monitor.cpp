/** 
**/

#include "linux_cobs_serial_transport.hpp"
#include "zcmgen_c/mithl_vectorXf_t.h"
#include "zcmgen_c/mithl_string_t.h"

#include <stdint.h>

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

static double getUnixTime(void)
{
    struct timespec tv;

    if(clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

    return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}

static void vector_handler(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_vectorXf_t *msg, void *user)
{
  printf("\n***********************\n");
  printf("mithl_vector_t recv on %s:\n", channel);
  printf("\t utime: %d, rows: %d. Vals: [", msg->utime, msg->rows);
  for (int i=0; i<msg->rows; i++)
    printf("%f ", msg->data[i]);
  printf("]\n");
  printf("***********************\n");
}


static void string_handler(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_string_t *msg, void *user)
{
  printf("\n***********************\n");
  printf("mithl_string_t recv on %s:\n", channel);
  printf("\t utime: %d, data: %s\n", msg->utime, msg->data);
  printf("***********************\n");
}


int
main( int    argc,
      char** argv  )
{
  char * serial_port;
  if (argc == 2)
    serial_port = argv[1];
  else
    serial_port = "/dev/ttyACM0";

  printf("Listening on %s\n", serial_port);

  zcm_trans_t * linux_cobs_serial_transport = linux_cobs_serial_transport_create(serial_port);
  zcm_t * zcm = zcm_create_trans(linux_cobs_serial_transport);
  mithl_vectorXf_t_subscribe(zcm, "VEC", &vector_handler, NULL);
  mithl_string_t_subscribe(zcm, "STR", &string_handler, NULL);
  
  double now = getUnixTime();
  double last_spam = getUnixTime();

  while (1) {
    zcm_handle_nonblock(zcm);
    if (getUnixTime() - now > 1.0){
      now = getUnixTime();
    }
    if (getUnixTime() - last_spam > 0.0001){
      mithl_string_t strmsg;
      char buf[100];  sprintf(buf, "Time: %f\n", getUnixTime());
      strmsg.utime = getUnixTime()*1E6,
      strmsg.data = buf;
      mithl_string_t_publish(zcm, "INC", &strmsg);
      last_spam = getUnixTime();
    }
  }
}
