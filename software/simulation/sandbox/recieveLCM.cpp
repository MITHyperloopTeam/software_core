#include <iostream>
#include <lcm/lcm.h>
#include "../../communication/lcmtypes/built_types/lcmgen_c/mithl_vectorXf_t.h"

using namespace std;

static void
msg_handler(const lcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_vectorXf_t * msg, void * user){

  cout << msg->utime << "\t\t" << msg->data[0] << endl;
}

int main() {
  lcm_t * lcm = lcm_create(NULL);

  mithl_vectorXf_t_subscribe(lcm, "SIM", &msg_handler, NULL);
  while (1)
    lcm_handle(lcm);
}