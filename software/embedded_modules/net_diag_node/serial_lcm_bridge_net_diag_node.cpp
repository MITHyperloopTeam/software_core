/** 
**/

#include "linux_cobs_serial_transport.hpp"

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#include "zcmgen_c/mithl_net_diag_t.h"
#include "zcmgen_c/mithl_net_health_t.h"


zcm_t * zcm_udp;
zcm_t * zcm_serial;

static double getUnixTime(void)
{
    struct timespec tv;

    if(clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

    return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}

// updater for udp zcm
void *zcmUDPUpater(void *arg)
{
  double last_print = getUnixTime();
  while(1)
  {
    if (getUnixTime() - last_print > 1.0){
      printf("ZCM handling thread going\n");
      last_print = getUnixTime();
    }
    zcm_handle(zcm_udp);
    usleep(1);
  }
}

// up only
static void net_health_handler(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_net_health_t *msg, void *user)
{
  printf("\n***********************\n");
  printf("mithl_health_t recv on %s:\n", channel);
  printf("***********************\n");

  mithl_net_health_t_publish(zcm_udp, channel, msg);
}

static void net_diag_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_net_diag_t *msg, void *user)
{
  printf("\n***** mithl_net_diag_t recv on %s going up\n", channel);
  mithl_net_diag_t_publish(zcm_udp, channel, msg);
}
static void net_diag_handler_down(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_net_diag_t *msg, void *user)
{
  printf("\n***** mithl_net_diag_t recv on %s going down\n", channel);
  mithl_net_diag_t_publish(zcm_serial, channel, msg);
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
  zcm_serial = zcm_create_trans(linux_cobs_serial_transport);
  zcm_udp = zcm_create("udpm://239.255.76.67:7667?ttl=0");

  mithl_net_diag_t_subscribe(zcm_udp, "_NREP", &net_diag_handler_down, NULL);
  mithl_net_diag_t_subscribe(zcm_udp, "_NREQ", &net_diag_handler_down, NULL);
  mithl_net_diag_t_subscribe(zcm_serial, "_NREP", &net_diag_handler_up, NULL);
  mithl_net_diag_t_subscribe(zcm_serial, "_NREQ", &net_diag_handler_up, NULL);
  mithl_net_health_t_subscribe(zcm_serial, "_NSUM", &net_health_handler, NULL);
  
  double now = getUnixTime();

  pthread_t pth;
  pthread_create(&pth,NULL,zcmUDPUpater, NULL);

  while (1) {
    zcm_handle_nonblock(zcm_serial);
    if (getUnixTime() - now > 1.0){
      now = getUnixTime();
      printf("Elapsed: %f\n", now);
    }
    usleep(1);
  }
}
