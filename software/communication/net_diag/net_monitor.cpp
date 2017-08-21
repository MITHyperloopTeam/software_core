#include "net_monitor.hpp"
#include "arduino_simul_zcm_helpers.hpp"


// this is silly af but need it for compressing
// multiple types into a void *
typedef struct IntegerPlusNetMonitor_ {
  int integer;
  NetMonitor * netMonitor;
} IntegerPlusNetMonitor;
IntegerPlusNetMonitor integersPlusNetMonitors[MAX_NUM_TRANSPORTS];

static void req_handler(const zcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_net_diag_t * msg, void * user){
  IntegerPlusNetMonitor * integerPlusNetMonitor = (IntegerPlusNetMonitor *) user;
  integerPlusNetMonitor->netMonitor->handleReq(msg, integerPlusNetMonitor->integer);
}
static void rep_handler(const zcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_net_diag_t * msg, void * user){
  IntegerPlusNetMonitor * integerPlusNetMonitor = (IntegerPlusNetMonitor *) user;
  integerPlusNetMonitor->netMonitor->handleRep(msg, integerPlusNetMonitor->integer);
}

NetMonitor::NetMonitor(TransportInfo * transports, int num_transports, int id, const char * name, double (*timeGetter)(void)){
  timeGetter_ = timeGetter;
  strncpy(my_name_, name, MAX_NAME_LEN);
  my_id_ = id;
  last_summary_send_time_ = timeGetter_();

  // pull in as many transports as we have room for
  if (num_transports <= MAX_NUM_TRANSPORTS)
    num_transports_ = num_transports;
  else
    num_transports_ = MAX_NUM_TRANSPORTS;
  // TODO(gizatt): This is a silent fail if too many are passed in...

  for (int i=0; i < num_transports_; i++){
    memcpy(&(transports_[i].transportInfo), &(transports[i]), sizeof(TransportInfo));
    resetTransportInfo(i);

    // Gotta compress multiple data types 
    integersPlusNetMonitors[i].integer = i;
    integersPlusNetMonitors[i].netMonitor = this;
    mithl_net_diag_t_subscribe(transports_[i].transportInfo.zcm, "_NREQ", &req_handler, &(integersPlusNetMonitors[i]));
    mithl_net_diag_t_subscribe(transports_[i].transportInfo.zcm, "_NREP", &rep_handler, &(integersPlusNetMonitors[i]));
  }
}

void NetMonitor::resetTransportInfo(int transport_info_num){
  transports_[transport_info_num].next_msg_id = 0;
  transports_[transport_info_num].send_info_buffer_head = 0;
  transports_[transport_info_num].send_info_buffer_tail = 0;
  for (int i=0; i < MESSAGE_HISTORY_LEN; i++){
    transports_[transport_info_num].send_time_buffer[i] = 0;
    transports_[transport_info_num].send_id_buffer[i] = -1; // haven't sent anything
  }
  for (int i=0; i < MESSAGE_HISTORY_LEN * MAX_NUM_HOSTS; i++){
    transports_[transport_info_num].response_received[i] = false;
  }
  for (int i=0; i < MAX_NUM_HOSTS; i++){
    transports_[transport_info_num].host_id_array[i] = -1;
    transports_[transport_info_num].est_drop_rate[i] = 0.0;
    transports_[transport_info_num].est_stale_rate[i] = 0.0;
    transports_[transport_info_num].est_rtt[i] = 0.0;
  }
  transports_[transport_info_num].num_mystery_packets = 0;

  transports_[transport_info_num].last_connectivity_send_time = timeGetter_();
}

void NetMonitor::update() {   
  for (int i=0; i<num_transports_; i++){
    double now = timeGetter_();
    if (now - transports_[i].last_connectivity_send_time >= CONNECTIVITY_SEND_PERIOD){
      transports_[i].last_connectivity_send_time = now;
      doConnectivityBroadcast(&(transports_[i]), now);
    }
  }

  double now = timeGetter_();
  if (now - last_summary_send_time_ >= SUMMARY_SEND_PERIOD){
    last_summary_send_time_ = now;
    doSummaryBroadcast(now);
  }

  for (int i=0; i<num_transports_; i++){
    ARDUINO_SIMUL_ZCM_HANDLE(transports_[i].transportInfo.zcm);
  }
}

void NetMonitor::clearBufferSpace(TransportTestInfo * ti) {
  ti->handle_mtx_.lock();

  // a message has timed out if a response hasn't been found 
  if (ti->send_info_buffer_head == ti->send_info_buffer_tail){
    for (int i=0; i<MAX_NUM_HOSTS; i++){
      if (!ti->response_received[ti->send_info_buffer_tail*MAX_NUM_HOSTS + i]){
        // this packet must have been dropped :(
        ti->est_drop_rate[i] = ALPHA_DROP * ti->est_drop_rate[i] + (1.-ALPHA_DROP);
      }
    }
    // invalidate the send id buffer
    ti->send_id_buffer[ti->send_info_buffer_tail] = -1;
    // and free up space in circ buffer
    ti->send_info_buffer_tail = (ti->send_info_buffer_tail + 1) % MESSAGE_HISTORY_LEN;
  }

  ti->handle_mtx_.unlock();
}
void NetMonitor::doConnectivityBroadcast(TransportTestInfo * ti, double now) {
  mithl_net_diag_t broadcastmsg;
  broadcastmsg.utime = now * 1000 * 1000;
  broadcastmsg.msg_id = ti->next_msg_id;
  broadcastmsg.origin_id = my_id_;
  broadcastmsg.dest_id = -1;

  clearBufferSpace(ti);

  // and update buffers so we can handle response
  ti->handle_mtx_.lock();
  for (int i=0; i <MAX_NUM_HOSTS; i++){
    ti->response_received[ti->send_info_buffer_head*MAX_NUM_HOSTS + i] = false;
  }
  ti->send_time_buffer[ti->send_info_buffer_head] = now;
  ti->send_id_buffer[ti->send_info_buffer_head] = ti->next_msg_id;

  // ready for next go
  ti->send_info_buffer_head = (ti->send_info_buffer_head + 1) % MESSAGE_HISTORY_LEN;
  ti->next_msg_id++;
  ti->handle_mtx_.unlock();

  // send the thing
  mithl_net_diag_t_publish(ti->transportInfo.zcm, "_NREQ", &broadcastmsg);
}

void NetMonitor::doSummaryBroadcast(double now) {
  mithl_net_health_t summarymsg;
  summarymsg.utime = now * 1000 * 1000;

  // collapse list of hosts to only good hosts
  int8_t _host_a[MAX_NUM_TRANSPORTS*MAX_NUM_HOSTS];
  int8_t _host_b[MAX_NUM_TRANSPORTS*MAX_NUM_HOSTS];
  int8_t _pair_transport_id[MAX_NUM_TRANSPORTS*MAX_NUM_HOSTS];
  float _est_rtt[MAX_NUM_TRANSPORTS*MAX_NUM_HOSTS];
  float _est_drop_rate[MAX_NUM_TRANSPORTS*MAX_NUM_HOSTS];
  float _est_stale_rate[MAX_NUM_TRANSPORTS*MAX_NUM_HOSTS];
  int32_t _num_mystery_packets[MAX_NUM_TRANSPORTS*MAX_NUM_HOSTS];
  int ir = 0;
  for (int transport_id = 0; transport_id < num_transports_; transport_id++){
    for (int i=0; i < MAX_NUM_HOSTS; i++){
      if (transports_[transport_id].host_id_array[i] >= 0){
        _host_a[ir] = my_id_;
        _host_b[ir] = transports_[transport_id].host_id_array[i];
        _pair_transport_id[ir] = transport_id;
        _est_rtt[ir] = transports_[transport_id].est_rtt[i];
        _est_drop_rate[ir] = transports_[transport_id].est_drop_rate[i];
        _est_stale_rate[ir] = transports_[transport_id].est_stale_rate[i];
        _num_mystery_packets[ir] = transports_[transport_id].num_mystery_packets;
        ir++;
      }
    }   
  } 
  summarymsg.num_pairs = ir;
  summarymsg.num_transports = num_transports_;
  summarymsg.pair_transport_id = _pair_transport_id;
  summarymsg.host_a = _host_a;
  summarymsg.host_b = _host_b;
  summarymsg.est_rtt = _est_rtt;
  summarymsg.est_drop_rate = _est_drop_rate;
  summarymsg.est_stale_rate = _est_stale_rate;
  summarymsg.num_mystery_packets = _num_mystery_packets;
  // send the thing on all transports we have
  for (int transport_id = 0; transport_id < num_transports_; transport_id++){
    mithl_net_health_t_publish(transports_[transport_id].transportInfo.zcm, "_NSUM", &summarymsg); 
  }
}

// Respond to any echoes that come in
void NetMonitor::handleReq(const mithl_net_diag_t * msg, int transport_info_num) {
  if (msg->origin_id != my_id_ && (msg->dest_id == -1 || msg->dest_id == my_id_)) {
    mithl_net_diag_t msg_back;
    msg_back.utime = msg->utime; // echo back the same time...
    msg_back.msg_id = msg->msg_id;
    msg_back.origin_id = my_id_;
    msg_back.dest_id = msg->origin_id;
    mithl_net_diag_t_publish(transports_[transport_info_num].transportInfo.zcm, "_NREP", &msg_back);
  }
  // don't republish any further.
}

// Observe echo responses
void NetMonitor::handleRep(const mithl_net_diag_t * msg, int transport_info_num) { 
  if (msg->dest_id == my_id_){ // broadcast response is not a thing we want to worry about

    TransportTestInfo * ti = &transports_[transport_info_num];

    // do we even know who this is?
    int i;
    for (i=0; i<MAX_NUM_HOSTS; i++){
      if (ti->host_id_array[i] == -1){
        ti->host_id_array[i] = msg->origin_id;
        break;
      }
      else if (msg->origin_id == ti->host_id_array[i]){
        break;
      }
    }

    // we know who this is
    if (i < MAX_NUM_HOSTS){
      ti->handle_mtx_.lock();
      int j;
      for (j=0; j<MESSAGE_HISTORY_LEN; j++){
        if (msg->msg_id == ti->send_id_buffer[j]){
          break;
        }
      }
      // this corresponds to a sent message
      if (j < MESSAGE_HISTORY_LEN){
        double new_rtt = timeGetter_() - ti->send_time_buffer[j];
        ti->est_rtt[i] = ALPHA * ti->est_rtt[i] + (1. - ALPHA)*new_rtt;
        ti->est_drop_rate[i] = ALPHA_DROP * ti->est_drop_rate[i] + 0;
        ti->est_stale_rate[i] = ALPHA_STALE * ti->est_stale_rate[i] + 0;
        ti->response_received[j*MAX_NUM_HOSTS + i] = true;
      } else { // does not correspond to a sent message
        ti->est_stale_rate[i] = ALPHA_STALE * ti->est_stale_rate[i] + (1. - ALPHA_STALE);
      }
      ti->handle_mtx_.unlock();

    } else { // we don't know who this came from
      // skip mutex as increment should be atomic here
      ti->num_mystery_packets++;
    }
  }
}