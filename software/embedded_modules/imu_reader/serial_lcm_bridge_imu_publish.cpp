/** 
**/

#include "linux_cobs_serial_transport.hpp"
#include "zcmgen_c/mithl_vectorXf_t.h"
#include "zcmgen_c/mithl_string_t.h"

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>


zcm_t * zcm_udp;
// updater for udp zcm
void *zcmUDPUpater(void *arg)
{
  while(1)
  {
    zcm_handle(zcm_udp);
  }
}


static double getUnixTime(void)
{
    struct timespec tv;

    if(clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

    return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}

static void output_handler(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_vectorXf_t *msg, void *user)
{
  printf("Msg %s: ", channel);
  for (int i=0; i < msg->rows; i++)
    printf("%2.2f,", msg->data[i]);
  printf("\n");
  mithl_vectorXf_t_publish(zcm_udp, channel, msg);
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

  zcm_trans_t * linux_cobs_serial_transport = linux_cobs_serial_transport_create(serial_port);
  zcm_t * zcm_serial = zcm_create_trans(linux_cobs_serial_transport);
  zcm_udp = zcm_create("udpm://239.255.76.67:7667?ttl=0");

  mithl_vectorXf_t_subscribe(zcm_serial, "_IMU_1_TEST", &output_handler, NULL);
  mithl_vectorXf_t_subscribe(zcm_serial, "_IMU_2_TEST", &output_handler, NULL);
  
  double now = getUnixTime();

  pthread_t pth;
  pthread_create(&pth,NULL,zcmUDPUpater, NULL);

  while (1) {
    zcm_handle_nonblock(zcm_serial);
    if (getUnixTime() - now > 1.0){
      now = getUnixTime();
      printf("Elapsed: %f\n", now);
    }
  }
}
