#ifndef NET_MONITOR_H
#define NET_MONITOR_H

#include "zcmgen_c/mithl_net_diag_t.h"
#include "zcmgen_c/mithl_net_health_t.h"
#include "arduino_fake_mutex.hpp"

/* Network Health Monitoring Utility for a mesh network
   of ZCM nodes that can see each other through some number
   of transports.

   On construction, is handed a list of ZCM transports, along
   with a human-readable list of labels for them.

   Supports a couple of test modes:
     - Connectivity and latency: Everyone sends REQuests 
       periodically on all media. Anyone who 
       receives sends a REsPonse on the same media
       (but doesn't forward to any other media -- this
       is for detecting connections on hops of length 1
       only.) The sender maintains a table of recently
       sent ARPs and checks ACKs against those to
       determine connectivity, latency, and 
       rough drop rates.
     - Bandwidth: On a trigger, node A and B initiate
       an N-second bandwidth-detection test on a specified
       media. Starting at 0 bits per second (bps), A sends
       packets to B, interspersed with ping reqs. B checks
       packet health and respond to requests. When packet
       drop rate / latency reaches threshold, bandwidth
       has been detected and results are added to the
       connectivity table.

    And a couple of utilities:
     - Every node maintains a table of who its connected
       to on what transport. This is broadcast periodically
       in a dedicated message type on all media, and all other
       nodes that hear it (from any media) broadcast it ONCE
       to all other media if they haven't heard it before. This
       way, everyone can reconstruct the full graph if they're
       interested.
     - Bandwidth test triggers find their way to their destination
       using the same mechanism of re-broadcast.


   Periodically broadcasts to ZCM a response-request
   message on channel _NREQ, expecting ASAP response
   on _NREP from anyone else running an instance
   of this class. */

#define MESSAGE_HISTORY_LEN 20
#define MAX_NUM_HOSTS 10
#define ALPHA 0.5
#define ALPHA_DROP 0.9
#define ALPHA_STALE 0.9
#define CONNECTIVITY_SEND_PERIOD 0.5
#define SUMMARY_SEND_PERIOD 1.0
#define MAX_NAME_LEN 100
#define MAX_TRANSPORT_NAME_LEN 15
#define MAX_NUM_TRANSPORTS 3

class NetMonitor { 
  public:
    typedef struct TransportInfo_ {
        zcm_t * zcm;
        char * transport_name;
    } TransportInfo;

  public: // public for debug reasons only... todo(gizatt) write accessors 

    // Media-specific information

    typedef struct TransportTestInfo_ {
        TransportInfo transportInfo;

        // the time at which the last MESSAGE_HISTORY_LEN
        // ARPs were sent
        double send_time_buffer[MESSAGE_HISTORY_LEN];
        // and the ID of that ARP request, to differentiate
        // it from others that might arrive out of order
        int32_t send_id_buffer[MESSAGE_HISTORY_LEN]; // -1 for data invalid
        int32_t next_msg_id;
        // circular buffer management
        int send_info_buffer_head;
        int send_info_buffer_tail;

        // for each host, have we received each packet?
        bool response_received[MESSAGE_HISTORY_LEN*MAX_NUM_HOSTS];


        // map of hosts to current estimated RTTs and dropped packet frequency
        int8_t host_id_array[MAX_NUM_HOSTS]; // -1 for unknown
        float est_rtt[MAX_NUM_HOSTS];
        float est_drop_rate[MAX_NUM_HOSTS]; 
        float est_stale_rate[MAX_NUM_HOSTS];

        int num_mystery_packets;

        double last_connectivity_send_time;

        Mutex handle_mtx_;
    } TransportTestInfo;

    TransportTestInfo transports_[MAX_NUM_TRANSPORTS];
    int num_transports_;

    double last_summary_send_time_;
    int8_t my_id_;
    char my_name_[MAX_NAME_LEN];

    double (* timeGetter_)();

  public: 

    NetMonitor(TransportInfo * transports, int num_transports, int id, const char * name, double (*timeGetter_)(void));
    void resetTransportInfo(int transport_info_num); // initialization helper

    void update();

    // Connectivity test methods
    void clearBufferSpace(TransportTestInfo * ti);
    void doConnectivityBroadcast(TransportTestInfo * ti, double now);
    // Respond to any echoes that come in
    void handleReq(const mithl_net_diag_t * msg, int transport_info_num);
    // Observe echo responses
    void handleRep(const mithl_net_diag_t * msg, int transport_info_num);

    // Utilities
    void doSummaryBroadcast(double now);
}; 

#endif