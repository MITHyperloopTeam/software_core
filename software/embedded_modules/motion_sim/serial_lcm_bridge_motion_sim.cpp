/** 
**/

#include "linux_cobs_serial_transport.hpp"
#include "zcmgen_c/mithl_net_diag_t.h"
#include "zcmgen_c/mithl_net_health_t.h"
#include "zcmgen_c/mithl_vectorXf_t.h"

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

int myid = 10;

zcm_t * zcm_udp;
zcm_t * zcm_serial;

/* Todo: finish this 
struct ChannelInfo {
  int msgs_up;
  int msgs_down;
}
std::map<std::string, ChannelInfo> channel_msgs;
*/

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


// up only
static void net_health_handler(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_net_health_t *msg, void *user)
{
  mithl_net_health_t_publish(zcm_udp, channel, msg);
}

static void net_diag_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_net_diag_t *msg, void *user)
{
  mithl_net_diag_t_publish(zcm_udp, channel, msg);
}
static void net_diag_handler_down(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_net_diag_t *msg, void *user)
{
  // every arduino has an ID >= 50; don't send those down
  if (msg->origin_id < 50 || msg->dest_id == myid)
    mithl_net_diag_t_publish(zcm_serial, channel, msg);
}

static void mithl_vectorXf_t_handler(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_vectorXf_t *msg, void *user)
{
  mithl_vectorXf_t_publish(zcm_serial, channel, msg);
}


int
main( int    argc,
      char** argv  )
{
  char * serial_port;
  if (argc == 2)
    serial_port = argv[1];
  else
    serial_port = "/dev/mithl_due_5";

  zcm_trans_t * linux_cobs_serial_transport = linux_cobs_serial_transport_create(serial_port);
  zcm_serial = zcm_create_trans(linux_cobs_serial_transport);
  zcm_udp = zcm_create("udpm://239.255.76.67:7667?ttl=0");


  mithl_net_diag_t_subscribe(zcm_udp, "_NREP", &net_diag_handler_down, NULL);
  mithl_net_diag_t_subscribe(zcm_udp, "_NREQ", &net_diag_handler_down, NULL);
  mithl_net_diag_t_subscribe(zcm_serial, "_NREP", &net_diag_handler_up, NULL);
  mithl_net_diag_t_subscribe(zcm_serial, "_NREQ", &net_diag_handler_up, NULL);
  mithl_net_health_t_subscribe(zcm_serial, "_NSUM", &net_health_handler, NULL);

  mithl_vectorXf_t_subscribe(zcm_udp, "MOTION_SIM_CMD", &mithl_vectorXf_t_handler, NULL);
  
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
