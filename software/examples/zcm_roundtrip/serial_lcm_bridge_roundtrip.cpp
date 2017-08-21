/** 
**/

#include "linux_cobs_serial_transport.hpp"
#include "zcmgen_c/mithl_string_t.h"

#include <stdint.h>

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>


zcm_t * zcm_udp;
zcm_t * zcm_ss;
zcm_t * zcm_sr;

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

static void string_handler_down(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_string_t *msg, void *user)
{
  printf("\n***********************\n");
  printf("mithl_string_t recv on %s:\n", channel);
  printf("\t utime: %d, string: %s\n", msg->utime, msg->data);
  printf("***********************\n");

  mithl_string_t_publish(zcm_ss, channel, msg);
}

static void string_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_string_t *msg, void *user)
{
  printf("\n***********************\n");
  printf("mithl_string_t recv on %s:\n", channel);
  printf("\t utime: %d, string: %s\n", msg->utime, msg->data);
  printf("***********************\n");

  mithl_string_t_publish(zcm_udp, channel, msg);
}


int
main( int    argc,
      char** argv  )
{
  if (argc != 3){
    printf("Please supply two args: serial port for sender board, and serial port for receiver board.\n");
    exit(0);
  }

  char * serial_port_s = argv[1];
  char * serial_port_r = argv[2];

  printf("Sending on %s, receiving on %s\n", serial_port_s, serial_port_r);

  zcm_trans_t * linux_cobs_serial_transport_s = linux_cobs_serial_transport_create(serial_port_s);
  zcm_trans_t * linux_cobs_serial_transport_r = linux_cobs_serial_transport_create(serial_port_r);
  zcm_ss = zcm_create_trans(linux_cobs_serial_transport_s);
  zcm_sr = zcm_create_trans(linux_cobs_serial_transport_r);
  zcm_udp = zcm_create("udpm://239.255.76.67:7667?ttl=0");

  mithl_string_t_subscribe(zcm_udp, "STR_SEND", &string_handler_down, NULL);
  mithl_string_t_subscribe(zcm_sr, "STR_REC", &string_handler_up, NULL);
  
  double now = getUnixTime();

  pthread_t pth;
  pthread_create(&pth,NULL,zcmUDPUpater, NULL);

  while (1) {
    zcm_handle_nonblock(zcm_ss);
    zcm_handle_nonblock(zcm_sr);
    if (getUnixTime() - now > 1.0){
      now = getUnixTime();
      //printf("Elapsed: %f\n", now);
    }
  }
}
