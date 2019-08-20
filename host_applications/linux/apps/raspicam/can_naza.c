/*
 * can_naza.c
 *
 * Copyright (c) 2015 DiUS
 * All rights reserved.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#include <sys/time.h>
#include <sys/ioctl.h>
//#include <net/if.h>

//#include <linux/can.h>
#include <linux/can/raw.h>

#include "can_naza.h"


#define NAZA_PROCESS_INTERVAL_MS    500
#define DUMP_FILE_DIVIDER           1
#define VERBOSE_PRINT_DIVIDER       2


int can_naza_init(struct can_naza_info_t *can_nz, const char *can_interface, const char *dump_file)
{
    int ret = 0;

    can_nz->dump_file = dump_file;
    can_nz->process_interval_ms = NAZA_PROCESS_INTERVAL_MS;
    can_nz->dump_count = 0;
    can_nz->print_count = 0;
    can_nz->can_interface = can_interface;
    can_nz->canfd_on = 1;        // CAN flexible data mode (FD mode)
    can_nz->can_rcvbuf_size = 0; // CAN rx buffer size, zero means default
    can_nz->running = 1;
    can_nz->can_sockfd = -1;

    can_nz->can_sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_nz->can_sockfd < 0)
    {
        perror("CAN socket");
        ret = -1;
        goto can_error;
    }

    can_nz->can_addr.can_family = AF_CAN;

    memset(&can_nz->can_ifr, 0, sizeof(can_nz->can_ifr));
    strncpy(can_nz->can_ifr.ifr_name, can_nz->can_interface, strlen(can_nz->can_interface));

    if (ioctl(can_nz->can_sockfd, SIOCGIFINDEX, &can_nz->can_ifr) < 0)
    {
        perror("CAN SIOCGIFINDEX");
        ret = -1;
        goto can_error;
    }
    can_nz->can_addr.can_ifindex = can_nz->can_ifr.ifr_ifindex;

    // setup CAN filter
    struct can_filter can_rfilter[] =
    {
        {.can_id = 0x090, .can_mask = CAN_SFF_MASK},
        {.can_id = 0x108, .can_mask = CAN_SFF_MASK},
        {.can_id = 0x7F8, .can_mask = CAN_SFF_MASK},
        //{.can_id = 0x000, .can_mask = 0}, // receive data frames and error frames
    };
    can_err_mask_t can_err_mask = 0xFFFFFFFF;

    setsockopt(can_nz->can_sockfd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
               &can_err_mask, sizeof(can_err_mask));

    setsockopt(can_nz->can_sockfd, SOL_CAN_RAW, CAN_RAW_FILTER,
               can_rfilter, sizeof(can_rfilter));

    // try to switch the socket into CAN FD mode
    setsockopt(can_nz->can_sockfd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
               &can_nz->canfd_on, sizeof(can_nz->canfd_on));

    // set CAN rx buffer size
    if (can_nz->can_rcvbuf_size)
    {
        int curr_rcvbuf_size;
        socklen_t curr_rcvbuf_size_len = sizeof(curr_rcvbuf_size);

        // try SO_RCVBUFFORCE first, if we run with CAP_NET_ADMIN
        if (setsockopt(can_nz->can_sockfd, SOL_SOCKET, SO_RCVBUFFORCE,
                       &can_nz->can_rcvbuf_size, sizeof(can_nz->can_rcvbuf_size)) < 0)
        {
            fprintf(stderr, "CAN SO_RCVBUFFORCE failed so try SO_RCVBUF ...\n");
            if (setsockopt(can_nz->can_sockfd, SOL_SOCKET, SO_RCVBUF,
                           &can_nz->can_rcvbuf_size, sizeof(can_nz->can_rcvbuf_size)) < 0)
            {
                perror("CAN setsockopt SO_RCVBUF");
                ret = -1;
                goto can_error;
            }

            if (getsockopt(can_nz->can_sockfd, SOL_SOCKET, SO_RCVBUF,
                           &curr_rcvbuf_size, &curr_rcvbuf_size_len) < 0)
            {
                perror("CAN getsockopt SO_RCVBUF");
                ret = -1;
                goto can_error;
            }

            // n.b.: The wanted size is doubled in Linux in net/sore/sock.c
            if (curr_rcvbuf_size < can_nz->can_rcvbuf_size*2)
                fprintf(stderr, "CAN socket receive buffer size was adjusted due to /proc/sys/net/core/rmem_max.\n");
        }
    }

    if (bind(can_nz->can_sockfd, (struct sockaddr *)&can_nz->can_addr, sizeof(can_nz->can_addr)) < 0)
    {
        perror("CAN bind");
        ret = -1;
        goto can_error;
    }

    libcannaza_init(&can_nz->nz_dec);

    return ret;

can_error:
    if (can_nz->can_sockfd >= 0)
    {
        close(can_nz->can_sockfd);
        can_nz->can_sockfd = -1;
    }

    return ret;
}


void can_naza_close(struct can_naza_info_t *can_nz)
{
    if (can_nz->can_sockfd >= 0)
    {
        close(can_nz->can_sockfd);
        can_nz->can_sockfd = -1;
    }
    libcannaza_uninit(&can_nz->nz_dec);
}


void can_naza_terminate(struct can_naza_info_t *can_nz)
{
    can_nz->running = 0;
}


void can_naza_rx_frames(struct can_naza_info_t *can_nz, int verbose)
{
    struct timespec now;
    struct timeval timeout = {.tv_sec = 0, .tv_usec = 250000};
    fd_set read_set;
    int status;
    struct iovec iov;
    struct canfd_frame frame;
    struct msghdr msg;
    struct cmsghdr *cmsg;
    int nbytes;
    int maxdlen;
    __u32 dropcnt = 0;
    __u32 last_dropcnt = 0;
    char ctrlmsg[CMSG_SPACE(sizeof(struct timeval)) + CMSG_SPACE(sizeof(__u32))];

    iov.iov_base = &frame;
    msg.msg_name = &can_nz->can_addr;
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = &ctrlmsg;

    if ((can_nz->dump_file) || (verbose))
        clock_gettime(CLOCK_MONOTONIC, &can_nz->prev_process_time);

    while (can_nz->running)
    {
        FD_ZERO(&read_set);
        FD_SET(can_nz->can_sockfd, &read_set);

        status = select(can_nz->can_sockfd + 1, &read_set, NULL, NULL, &timeout);
        if (status < 0)
        {
            int error_code = errno;
            // select was interrupted by a signal
            if (error_code != EINTR)
            {
                fprintf(stderr, "Error: select() returned error: %s\n", strerror(error_code));
                return;
            }
        }
        else
        {
            if (FD_ISSET(can_nz->can_sockfd, &read_set))
            {
                // these settings may be modified by recvmsg()
                iov.iov_len = sizeof(frame);
                msg.msg_namelen = sizeof(struct sockaddr_can);
                msg.msg_controllen = sizeof(ctrlmsg);
                msg.msg_flags = 0;

                nbytes = recvmsg(can_nz->can_sockfd, &msg, 0);
                if (nbytes < 0) {
                    perror("CAN read error");
                    return;
                }

                if ((size_t)nbytes == CAN_MTU)
                    maxdlen = CAN_MAX_DLEN;
                else if ((size_t)nbytes == CANFD_MTU)
                    maxdlen = CANFD_MAX_DLEN;
                else {
                    fprintf(stderr, "CAN read: incomplete CAN frame\n");
                    return;
                }

                for (cmsg = CMSG_FIRSTHDR(&msg);
                     cmsg && (cmsg->cmsg_level == SOL_SOCKET);
                     cmsg = CMSG_NXTHDR(&msg,cmsg))
                {
                    if (cmsg->cmsg_type == SO_RXQ_OVFL)
                        memcpy(CMSG_DATA(cmsg), &dropcnt, sizeof(__u32));
                }

                // check for (unlikely) dropped frames on this specific socket
                if (dropcnt != last_dropcnt)
                {
                    __u32 frames = dropcnt - last_dropcnt;
                    fprintf(stderr, "CAN DROPCOUNT: dropped %d CAN frame%s (total drops %d)\n",
                            frames, (frames > 1)?"s":"", dropcnt);
                    last_dropcnt = dropcnt;
                }

                // process can frame
                libcannaza_process_can_frame(&can_nz->nz_dec, &frame, maxdlen);

                if (clock_gettime(CLOCK_MONOTONIC, &now) == 0)
                {
                    if (((now.tv_sec - can_nz->prev_process_time.tv_sec) * 1000000000 +
                         (now.tv_nsec - can_nz->prev_process_time.tv_nsec)) >= can_nz->process_interval_ms * 1000000)
                    {
                        can_nz->prev_process_time = now;
                        can_naza_process_messages(can_nz);

                        if (can_nz->dump_file)
                        {
                            can_nz->dump_count++;
                            if (can_nz->dump_count >= DUMP_FILE_DIVIDER)
                            {
                                can_nz->dump_count = 0;
                                naza_dump_binary(can_nz->dump_file, &can_nz->nz_dec.nz);
                            }
                        }
                        if (verbose)
                        {
                            can_nz->print_count++;
                            if (can_nz->print_count >= VERBOSE_PRINT_DIVIDER)
                            {
                                can_nz->print_count = 0;
                                naza_print_ascii(stderr, &can_nz->nz_dec.nz);
                            }
                        }
                    }
                }
            }
        }
    }
}


void can_naza_process_messages(struct can_naza_info_t *can_nz)
{
    libcannaza_process_msg_1002(&can_nz->nz_dec);
    libcannaza_process_msg_1003(&can_nz->nz_dec);
    libcannaza_process_msg_1009(&can_nz->nz_dec);
#ifdef GET_SMART_BATTERY_DATA
    libcannaza_process_msg_0926(&can_nz->nz_dec);
#endif
}
