#ifndef _CAN_NAZA_H_
#define _CAN_NAZA_H_

#include <stdio.h>
#include <net/if.h>
#include <linux/can.h>
#include <time.h> // for printing to file

#include "libcannaza.h"


struct can_naza_info_t
{
    const char *dump_file;
    const char *can_interface;
    int canfd_on;           // CAN flexible data mode (FD mode)
    int can_rcvbuf_size;    // CAN rx buffer size, zero means default
    struct sockaddr_can can_addr;
    struct ifreq can_ifr;
    int can_sockfd;
    struct timespec prev_process_time;
    int dump_count;
    int print_count;
    int running;
    int process_interval_ms;
    struct can_naza_decoder_info_t nz_dec;
};


int can_naza_init(struct can_naza_info_t *can_nz, const char *can_interface, const char *dump_file);
void can_naza_close(struct can_naza_info_t *can_nz);
void can_naza_terminate(struct can_naza_info_t *can_nz);
void can_naza_rx_frames(struct can_naza_info_t *can_nz, int verbose);
void can_naza_process_messages(struct can_naza_info_t *can_nz);


#endif // _CAN_NAZA_H_
