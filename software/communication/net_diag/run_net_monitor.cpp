#include <iostream> // for standard I/O
#include <string>   // for strings
#include <unistd.h> // for sleep
#include "arduino_simul_zcm_helpers.hpp"
#include "net_monitor.hpp"
#include "common_utils.h"

// For simulation this must be defined, even though
// this is a sim-only module.
char MODULE_NAME[] = "NET_MONITOR";

using namespace std;

int main(int argc, char** argv)
{
    if (argc != 2){
        printf("Usage: run_net_monitor <id num>\n");
        exit (0);
    }
    int myid = atoi(argv[1]);
    printf("Net Monitor initializing with id %d\n", myid);

    zcm_t * zcm_udp = zcm_create("udpm://239.255.76.67:7667?ttl=0");
    NetMonitor::TransportInfo trans_udp[1];
    trans_udp[0].zcm = zcm_udp;
    trans_udp[0].transport_name = "UDP";

    NetMonitor netmon(trans_udp, 1, myid, "C++ Net Mon", &getUnixTime);
    zcm_start(zcm_udp);

    double last_print_time = getUnixTime();
    while (1){
        netmon.update();
        if (getUnixTime() - last_print_time > 1.0){
            last_print_time = getUnixTime();
            printf("[Trans, HostID, RTT, DROP, STALE, MYST] ");
            for (int tid=0; tid<netmon.num_transports_; tid++){
                NetMonitor::TransportTestInfo * ti = &(netmon.transports_[tid]);
                for (int i=0; i<MAX_NUM_HOSTS; i++){
                    if (ti->host_id_array[i] >= 0){
                        printf("(%2d, %2d, %0.4f, %4f, %4f)", tid, ti->host_id_array[i], ti->est_rtt[i], ti->est_drop_rate[i], ti->est_stale_rate[i]);
                    }
                }
                printf("[Trans %d had  %d mystery]\n", tid, ti->num_mystery_packets);
            }
        }
        usleep(100);
    }
    return 0;
}
