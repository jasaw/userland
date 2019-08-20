#ifndef _TEENSY_NAZA_H_
#define _TEENSY_NAZA_H_

#include <stdio.h>
#include <time.h> // for printing to file

#include "libteensynaza.h"


typedef struct __attribute__((packed))
{
    naza_msg1002_t msg;
    uint16_t crc;
} naza_msg1002_frame_t;

typedef struct __attribute__((packed))
{
    naza_msg1003_t msg;
    uint16_t crc;
} naza_msg1003_frame_t;

typedef struct __attribute__((packed))
{
    naza_msg1009_t msg;
    uint16_t crc;
} naza_msg1009_frame_t;

typedef struct __attribute__((packed))
{
    naza_msg0926_t msg;
    uint16_t crc;
} naza_msg0926_frame_t;

typedef union
{
    naza_msg_header_t header;
    naza_msg1002_frame_t frame1002;
    naza_msg1003_frame_t frame1003;
    naza_msg1009_frame_t frame1009;
    naza_msg0926_frame_t frame0926;
} naza_frame_t;


struct teensy_naza_t
{
    const char *dump_file;
    const char *device_path;
    int serialfd;
    struct timespec prev_print_time;
    int dump_count;
    int print_count;
    int running;
    int print_interval_ms;
    struct ts_naza_decoder_info_t nz_dec;
    naza_frame_t rx_frame;
    int rx_frame_idx;
    int rx_frame_start_count;
};


int teensy_naza_init(struct teensy_naza_t *tnz, const char *serial_interface, const char *dump_file);
void teensy_naza_close(struct teensy_naza_t *tnz);
void teensy_naza_terminate(struct teensy_naza_t *tnz);
void teensy_naza_rx_frames(struct teensy_naza_t *tnz, int verbose);
void teensy_naza_process_messages(struct teensy_naza_t *tnz);


#endif // _CAN_NAZA_H_
