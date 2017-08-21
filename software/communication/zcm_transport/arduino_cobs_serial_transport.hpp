/* Implements ZCM Transport Layer using 
   Arduino + COBS from PacketLib */
#ifndef LINUX_COBS_SERIAL_TRANSPORT_H
#define LINUX_COBS_SERIAL_TRANSPORT_H

#define ZCM_EMBEDDED
#include "zcm/transport.h"
#include "Arduino.h"
#include "../../externals/PacketSerial/src/PacketSerialArduino.h"

// Define this the class name you want
#define MTU (1<<10)
#define QUEUE_NUM 20

// Buffer contains the actual MTU, plus the channel name, plus a terminator,
// plus 4 bytes from CRC32
#define BUFFER_SIZE (ZCM_CHANNEL_MAXLEN + 1 + MTU + 4)
#define QUEUE_SIZE (BUFFER_SIZE*QUEUE_NUM)


typedef struct
{
    // Required by ZCM transport definition
    enum zcm_type trans_type;
    zcm_trans_methods_t *vtbl;
    PacketSerial_<COBS, 0, BUFFER_SIZE> serial;

    uint8_t send_buffer[BUFFER_SIZE];
    uint8_t recv_queue[QUEUE_SIZE];
    uint8_t recv_buffer_head; // push to head
    uint8_t recv_buffer_tail; // pop from tail

    size_t recv_queue_total_sizes[QUEUE_NUM];

} arduino_transport_t;

/* Returns Maximum Transmission Unit supported */
size_t arduino_transport_get_mtu(zcm_trans_t *zt);
int arduino_transport_sendmsg(zcm_trans_t *zt, zcm_msg_t msg);
int arduino_transport_recvmsg_enable(zcm_trans_t *zt, const char *channel, bool enable);

int arduino_transport_recvmsg(zcm_trans_t *zt, zcm_msg_t *msg, int timeout);

int arduino_transport_update(zcm_trans_t *zt);

void arduino_transport_destroy(zcm_trans_t *zt);

static zcm_trans_methods_t methods = {
    arduino_transport_get_mtu,
    arduino_transport_sendmsg,
    arduino_transport_recvmsg_enable,
    arduino_transport_recvmsg,
    arduino_transport_update,
    arduino_transport_destroy
};

// put packet in our cicular incoming-message-buffer
void arduino_transport_on_packet(const uint8_t* buffer, size_t size, void * extra);

zcm_trans_t *arduino_transport_create(int serial_num);

#endif
