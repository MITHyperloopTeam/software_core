/** 
**/

#include "linux_cobs_serial_transport.hpp"
#include "zcmgen_c/mithl_trigger_t.h"
#include "zcmgen_c/mithl_fiducial_t.h"
#include "zcmgen_c/mithl_fiducial_config_t.h"
#include "zcmgen_c/mithl_fiducial_teach_table_t.h"
#include "zcmgen_c/mithl_fiducial_color_dataval_t.h"
#include "zcmgen_c/mithl_fiducial_low_rate_t.h"
#include "zcmgen_c/mithl_string_t.h"
#include "zcmgen_c/mithl_net_diag_t.h"
#include "zcmgen_c/mithl_net_health_t.h"

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

int myid = 52;

zcm_t * zcm_udp;
zcm_t * zcm_serial;

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


static void fiducial_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_fiducial_t *msg, void *user)
{
  mithl_fiducial_t_publish(zcm_udp, channel, msg);
}

static void string_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_string_t *msg, void *user)
{
  mithl_string_t_publish(zcm_udp, channel, msg);
}

static void fiducial_dataval_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_fiducial_color_dataval_t *msg, void *user)
{
  mithl_fiducial_color_dataval_t_publish(zcm_udp, channel, msg);
}

static void fiducial_config_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_fiducial_config_t *msg, void *user)
{
  mithl_fiducial_config_t_publish(zcm_udp, channel, msg);
}

static void fiducial_config_handler_down(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_fiducial_config_t *msg, void *user)
{
  mithl_fiducial_config_t_publish(zcm_serial, channel, msg);
}

static void fiducial_teach_table_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_fiducial_teach_table_t *msg, void *user)
{
  mithl_fiducial_teach_table_t_publish(zcm_udp, channel, msg);
}

static void fiducial_teach_table_handler_down(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_fiducial_teach_table_t *msg, void *user)
{
  printf("\n****FIDUCIAL MODULE:****\n");
  printf("mithl_fiducial_teach_table_t recv on %s:\n", channel);
  printf("\t utime: %d\n", msg->utime);
  printf("***********************\n");

  mithl_fiducial_teach_table_t_publish(zcm_serial, channel, msg);
}

static void fiducial_low_rate_t_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_fiducial_low_rate_t *msg, void *user)
{
  mithl_fiducial_low_rate_t_publish(zcm_udp, channel, msg);
}

int
main( int    argc,
      char** argv  )
{
  char * serial_port;
  if (argc == 2)
    serial_port = argv[1];
  else
    serial_port = "/dev/mithl_fiducial";

  zcm_trans_t * linux_cobs_serial_transport = linux_cobs_serial_transport_create(serial_port);
  zcm_serial = zcm_create_trans(linux_cobs_serial_transport);
  zcm_udp = zcm_create("udpm://239.255.76.67:7667?ttl=0");


  mithl_net_diag_t_subscribe(zcm_udp, "_NREP", &net_diag_handler_down, NULL);
  mithl_net_diag_t_subscribe(zcm_udp, "_NREQ", &net_diag_handler_down, NULL);
  mithl_net_diag_t_subscribe(zcm_serial, "_NREP", &net_diag_handler_up, NULL);
  mithl_net_diag_t_subscribe(zcm_serial, "_NREQ", &net_diag_handler_up, NULL);
  mithl_net_health_t_subscribe(zcm_serial, "_NSUM", &net_health_handler, NULL);

  mithl_fiducial_t_subscribe(zcm_serial, "_FD_O", &fiducial_handler_up, NULL);
  mithl_fiducial_low_rate_t_subscribe(zcm_serial, "_FD_OL", &fiducial_low_rate_t_handler_up, NULL);
  mithl_fiducial_color_dataval_t_subscribe(zcm_serial, "_FIDUCIAL_INCOMING_DATAVAL", &fiducial_dataval_handler_up, NULL);
  mithl_fiducial_config_t_subscribe(zcm_serial, "_FIDUCIAL_CUR_CONFIG", &fiducial_config_handler_up, NULL);
  mithl_fiducial_config_t_subscribe(zcm_serial, "_FIDUCIAL_INCOMING_CONFIG", &fiducial_config_handler_up, NULL);
  mithl_fiducial_config_t_subscribe(zcm_serial, "_FIDUCIAL_DESIRED_CONFIG", &fiducial_config_handler_up, NULL);
  mithl_fiducial_teach_table_t_subscribe(zcm_serial, "_FIDUCIAL_CUR_TEACH_TABLE", &fiducial_teach_table_handler_up, NULL);
  mithl_fiducial_teach_table_t_subscribe(zcm_serial, "_FIDUCIAL_INCOMING_TEACH_TABLE", &fiducial_teach_table_handler_up, NULL);
  mithl_fiducial_teach_table_t_subscribe(zcm_serial, "_FIDUCIAL_DESIRED_TEACH_TABLE", &fiducial_teach_table_handler_up, NULL);
  mithl_fiducial_config_t_subscribe(zcm_udp, "_FIDUCIAL_SET_CONFIG", &fiducial_config_handler_down, NULL);
  mithl_fiducial_teach_table_t_subscribe(zcm_udp, "_FIDUCIAL_SET_TEACH_TABLE", &fiducial_teach_table_handler_down, NULL);
  // mithl_string_t_subscribe(zcm_serial, "_FIDUCIAL_ARD_MESSAGE", &string_handler_up, NULL); // TODO: Heartbeat
  mithl_string_t_subscribe(zcm_serial, "_FIDUCIAL_WARN_MESSAGE", &string_handler_up, NULL);

  // DB DB DB
  mithl_string_t_subscribe(zcm_serial, "COMM_BUFFER_SUBMIT", &string_handler_up, NULL); //DB
//  mithl_string_t_subscribe(zcm_serial, "RECV_BUFFER_SINGLE", &string_handler_up, NULL); //DB
//  mithl_string_t_subscribe(zcm_serial, "RECV_BUFFER_BCAST_COUNTER", &string_handler_up, NULL); //DB
//  mithl_string_t_subscribe(zcm_serial, "RECV_BUFFER_ANY", &string_handler_up, NULL); //DB
//  mithl_string_t_subscribe(zcm_serial, "RECV_BUFFER_00AAX", &string_handler_up, NULL); //DB
  mithl_string_t_subscribe(zcm_serial, "_FIDUCIAL_ACK_TIMEOUT", &string_handler_up, NULL); //DB
  mithl_string_t_subscribe(zcm_serial, "_FIDUCIAL_WARN2_MESSAGE", &string_handler_up, NULL); //DB
  // /DB /DB /DB





//  mithl_low_speed_config_t_subscribe(zcm_serial, "_LS_CUR_CONFIG", &config_handler_up, NULL);
//  mithl_low_speed_status_t_subscribe(zcm_serial, "_LS_CUR_STATUS", &status_handler_up, NULL);
//  mithl_low_speed_config_t_subscribe(zcm_udp, "_LS_SET_CONFIG", &config_handler_down, NULL);

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
