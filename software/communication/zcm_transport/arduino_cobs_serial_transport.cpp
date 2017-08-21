/* Implements ZCM Transport Layer using 
   Arduino + COBS from PacketLib */

#include "arduino_cobs_serial_transport.hpp"
#include "crc32.h"

/* Returns Maximum Transmission Unit supported */
size_t arduino_transport_get_mtu(zcm_trans_t *zt){
    return MTU;
}

int arduino_transport_sendmsg(zcm_trans_t *zt, zcm_msg_t msg){
    // transmit buffer layout:
    //  [channelName[0]
    //  ...
    //   channelName[channelNameLen-1]
    //   null terminator
    //   <data buf>
    arduino_transport_t * t = (arduino_transport_t *) zt;

    size_t channelNameLen = 0;
    while (msg.channel[channelNameLen] != 0) {
        t->send_buffer[channelNameLen] = (uint8_t) msg.channel[channelNameLen];
        channelNameLen+=1;
        if (channelNameLen > 255) // force to fit in one byte with null term
            return ZCM_EINVALID;
    }
    if (msg.len > MTU)
        return ZCM_EINVALID;

    // put channel name and data in one buffer
    size_t index = 0;
    t->send_buffer[channelNameLen] = 0;
    memcpy(t->send_buffer + channelNameLen + 1, msg.buf, msg.len);
    // Calculate crc on the total buffer and append that
    uint32_t crc = crc32(CRC32_REASONABLE_SEED, t->send_buffer, 1 + channelNameLen + msg.len);
    *((uint32_t *)(&t->send_buffer[1 + channelNameLen + msg.len])) = crc;

    // send send send
    t->serial.send(t->send_buffer, 1 + channelNameLen + msg.len + 4);
    
    // no way to detect failure in PacketSerial so far so report success
    return ZCM_EOK;
}

int arduino_transport_recvmsg_enable(zcm_trans_t *zt, const char *channel, bool enable){
    // register 
    return ZCM_EOK;
}

int arduino_transport_recvmsg(zcm_trans_t *zt, zcm_msg_t *msg, int timeout){
    arduino_transport_t * t = (arduino_transport_t *) zt;

    if (t->recv_buffer_tail != t->recv_buffer_head){
        // decode message from this buffer position
        uint8_t * recv_buffer = t->recv_queue + t->recv_buffer_tail * BUFFER_SIZE;

        // First check CRC32
        uint32_t total_size = t->recv_queue_total_sizes[t->recv_buffer_tail];
        uint32_t crc_recv = * ((uint32_t *)(recv_buffer +  total_size - 4));
        uint32_t crc_calc = crc32(CRC32_REASONABLE_SEED, recv_buffer, total_size - 4);
        if (crc_calc != crc_recv){
            // Free up buffer space and fail
            t->recv_buffer_tail  = (t->recv_buffer_tail + 1) % QUEUE_NUM;
            return ZCM_EAGAIN;
        }

        msg->channel = (char *)recv_buffer;
        
        // step forward past first null term for the msg buffer itself
        uint16_t channelNameLen = 0;
        while (recv_buffer[channelNameLen] != 0) {
            channelNameLen++;
            if (channelNameLen > 255){
                // Free up buffer space and fail
                t->recv_buffer_tail  = (t->recv_buffer_tail + 1) % QUEUE_NUM;
                return ZCM_EAGAIN;
            }
        }
        // This should seem scary to you! 
        // The way nonblocking transports work, ZCM will call recv
        // once, and if we spit out a message, immediately dispatch it to the
        // right callbacks. Further calls to recvmsg or transport_update
        // do not until after that callback is done.
        msg->buf = (char *) (recv_buffer + channelNameLen + 1); 
        msg->len = total_size - 1 - channelNameLen - 4;
        
        t->recv_buffer_tail  = (t->recv_buffer_tail + 1) % QUEUE_NUM;
        return ZCM_EOK;
    }
    return ZCM_EAGAIN;
}

int arduino_transport_update(zcm_trans_t *zt){
    arduino_transport_t * t = (arduino_transport_t *) zt;
    t->serial.update();
    return ZCM_EOK;
}

void arduino_transport_destroy(zcm_trans_t *zt){
    ;
}

// put packet in our cicular incoming-message-buffer
void arduino_transport_on_packet(const uint8_t* buffer, size_t size, void * extra)
{
    if (size > 0){
        arduino_transport_t * t = (arduino_transport_t *) extra;
        uint8_t next_recv_buffer_head = (t->recv_buffer_head + 1) % QUEUE_NUM;
        // if it intersects with tail we have a problem so drop the packet
        // (if this happens we're falling behind on handling messages!)    
        if (next_recv_buffer_head != t->recv_buffer_tail && size <= BUFFER_SIZE){
            memcpy(t->recv_queue + t->recv_buffer_head * BUFFER_SIZE, buffer, size);
            t->recv_queue_total_sizes[t->recv_buffer_head] = size;

            // push head forward
            t->recv_buffer_head = next_recv_buffer_head;
        }
    }
}

zcm_trans_t *arduino_transport_create(int serial_num)
{
    arduino_transport_t *trans = new arduino_transport_t();
    trans->recv_buffer_tail = 0;
    trans->recv_buffer_head = 0;
    /* construct trans here */

    trans->serial.setPacketHandler(&arduino_transport_on_packet, trans);
    trans->serial.begin(115200, serial_num);

    trans->trans_type = ZCM_NONBLOCKING;
    trans->vtbl = &methods;            /* setting the virtual-table defined above */

    return (zcm_trans_t *) trans;
}