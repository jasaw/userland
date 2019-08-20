/*
 *    Filename: teensy_naza.c
 *
 * Copyright (c) 2015 DiUS
 *
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

#include "libteensynaza.h"
#include "serial.h"
#include "crc16ansi.h"
#include "teensy_naza.h"


#define NAZA_PRINT_INTERVAL_MS      500
#define DUMP_FILE_DIVIDER           1
#define VERBOSE_PRINT_DIVIDER       2

#define SERIAL_BAUD_RATE            115200
#define SLEEP_DURATION_MS           100
#define SERIAL_READ_BUFFER_SIZE     512
#define NUM_START_CHAR              2

#define SERIAL_COMMS_FRAME_START  0xDA
#define SERIAL_COMMS_ESCAPE_CHAR  0xE5


int teensy_naza_init(struct teensy_naza_t *tnz, const char *serial_interface, const char *dump_file)
{
    tnz->dump_file = dump_file;
    tnz->device_path = serial_interface;
    tnz->dump_count = 0;
    tnz->print_count = 0;
    tnz->running = 1;
    tnz->serialfd = -1;
    tnz->print_interval_ms = NAZA_PRINT_INTERVAL_MS;
    tnz->rx_frame_idx = 0;
    tnz->rx_frame_start_count = 0;

    if (serial_init(tnz->device_path, SERIAL_BAUD_RATE))
        return -1;
    tnz->serialfd = serial_get_fd();

    libteensynaza_init(&tnz->nz_dec);

    return 0;
}


void teensy_naza_close(struct teensy_naza_t *tnz)
{
    serial_close();
    tnz->serialfd = serial_get_fd();
    libteensynaza_uninit(&tnz->nz_dec);
}


void teensy_naza_terminate(struct teensy_naza_t *tnz)
{
    tnz->running = 0;
}


static void teensy_process_rxed_frame(struct teensy_naza_t *tnz)
{
    if (tnz->rx_frame_idx >= sizeof(naza_msg_header_t))
    {
        int msg_len;
        uint16_t crc;
        uint16_t msg_crc;
        uint16_t msg_id;
        memcpy(&msg_id, &tnz->rx_frame.header.id, sizeof(msg_id));
        switch (msg_id)
        {
            case NAZA_MESSAGE_MSG1002:
                msg_len = sizeof(naza_msg1002_frame_t);
                crc = crc16(CRC16_INIT, &tnz->rx_frame.frame1002.msg, sizeof(tnz->rx_frame.frame1002.msg));
                memcpy(&msg_crc, &tnz->rx_frame.frame1002.crc, sizeof(msg_crc));
                if ((tnz->rx_frame_idx == msg_len) && (crc == msg_crc))
                    libteensynaza_cache_msg_1002(&tnz->nz_dec, &tnz->rx_frame.frame1002.msg);
                break;
            case NAZA_MESSAGE_MSG1003:
                msg_len = sizeof(naza_msg1003_frame_t);
                crc = crc16(CRC16_INIT, &tnz->rx_frame.frame1003.msg, sizeof(tnz->rx_frame.frame1003.msg));
                memcpy(&msg_crc, &tnz->rx_frame.frame1003.crc, sizeof(msg_crc));
                if ((tnz->rx_frame_idx == msg_len) && (crc == msg_crc))
                    libteensynaza_cache_msg_1003(&tnz->nz_dec, &tnz->rx_frame.frame1003.msg);
                break;
            case NAZA_MESSAGE_MSG1009:
                msg_len = sizeof(naza_msg1009_frame_t);
                crc = crc16(CRC16_INIT, &tnz->rx_frame.frame1009.msg, sizeof(tnz->rx_frame.frame1009.msg));
                memcpy(&msg_crc, &tnz->rx_frame.frame1009.crc, sizeof(msg_crc));
                if ((tnz->rx_frame_idx == msg_len) && (crc == msg_crc))
                    libteensynaza_cache_msg_1009(&tnz->nz_dec, &tnz->rx_frame.frame1009.msg);
                break;
            case NAZA_MESSAGE_MSG0926:
                msg_len = sizeof(naza_msg0926_frame_t);
                crc = crc16(CRC16_INIT, &tnz->rx_frame.frame0926.msg, sizeof(tnz->rx_frame.frame0926.msg));
                memcpy(&msg_crc, &tnz->rx_frame.frame0926.crc, sizeof(msg_crc));
                if ((tnz->rx_frame_idx == msg_len) && (crc == msg_crc))
                    libteensynaza_cache_msg_0926(&tnz->nz_dec, &tnz->rx_frame.frame0926.msg);
                break;
            default:
                return;
        }
    }
}


void teensy_naza_rx_frames(struct teensy_naza_t *tnz, int verbose)
{
    struct timespec now;
    int escape = 0;
    int reset_count = 0;
    int ret;
    int i;
    uint8_t byte;
    uint8_t *rx_buf = (uint8_t *)(&tnz->rx_frame);
    uint8_t buffer[SERIAL_READ_BUFFER_SIZE];

    if ((tnz->dump_file) || (verbose))
        clock_gettime(CLOCK_MONOTONIC, &tnz->prev_print_time);

    while (tnz->running)
    {
        ret = read(tnz->serialfd, buffer, sizeof(buffer));
        if (ret > 0)
        {
            for (i = 0; i < ret; i++)
            {
                byte = buffer[i];
                if (tnz->rx_frame_start_count < NUM_START_CHAR)
                {
                    if (!escape)
                    {
                        switch (byte)
                        {
                            case SERIAL_COMMS_ESCAPE_CHAR:
                                escape = 1;
                                break;
                            case SERIAL_COMMS_FRAME_START:
                                tnz->rx_frame_start_count++;
                                break;
                            default:
                                tnz->rx_frame_start_count = 0;
                                tnz->rx_frame_idx = 0;
                                reset_count = 0;
                        }
                    }
                    else
                    {
                        escape = 0;
                        tnz->rx_frame_start_count = 0;
                        tnz->rx_frame_idx = 0;
                        reset_count = 0;
                    }
                }
                else
                {
                    if ((!escape) && (byte == SERIAL_COMMS_ESCAPE_CHAR))
                        escape = 1;
                    else if ((!escape) && (byte == SERIAL_COMMS_FRAME_START))
                    {
                        reset_count++;
                        if (reset_count >= NUM_START_CHAR)
                        {
                            tnz->rx_frame_start_count = 0;
                            tnz->rx_frame_idx = 0;
                            reset_count = 0;
                        }
                    }
                    else
                    {
                        escape = 0;
                        reset_count = 0;
                        // add byte to rx buffer
                        if (tnz->rx_frame_idx < sizeof(naza_frame_t))
                        {
                            int msg_len = sizeof(naza_msg_header_t);
                            rx_buf[tnz->rx_frame_idx++] = byte;
                            if (tnz->rx_frame_idx >= sizeof(naza_msg_header_t))
                            {
                                uint16_t msg_id;
                                memcpy(&msg_id, &tnz->rx_frame.header.id, sizeof(msg_id));
                                switch (msg_id)
                                {
                                    case NAZA_MESSAGE_MSG1002: msg_len = sizeof(naza_msg1002_frame_t); break;
                                    case NAZA_MESSAGE_MSG1003: msg_len = sizeof(naza_msg1003_frame_t); break;
                                    case NAZA_MESSAGE_MSG1009: msg_len = sizeof(naza_msg1009_frame_t); break;
                                    case NAZA_MESSAGE_MSG0926: msg_len = sizeof(naza_msg0926_frame_t); break;
                                }
                            }
                            if (tnz->rx_frame_idx >= msg_len)
                            {
                                // received a complete frame, process it...
                                teensy_process_rxed_frame(tnz);

                                tnz->rx_frame_start_count = 0;
                                tnz->rx_frame_idx = 0;
                            }
                        }
                        else
                        {
                            // overflow, reset...
                            tnz->rx_frame_start_count = 0;
                            tnz->rx_frame_idx = 0;
                        }
                    }
                }
            }
            // print...
            if (clock_gettime(CLOCK_MONOTONIC, &now) == 0)
            {
                if (((now.tv_sec - tnz->prev_print_time.tv_sec) * 1000000000 +
                     (now.tv_nsec - tnz->prev_print_time.tv_nsec)) >= tnz->print_interval_ms * 1000000)
                {
                    tnz->prev_print_time = now;
                    teensy_naza_process_messages(tnz);

                    if (tnz->dump_file)
                    {
                        tnz->dump_count++;
                        if (tnz->dump_count >= DUMP_FILE_DIVIDER)
                        {
                            tnz->dump_count = 0;
                            naza_dump_binary(tnz->dump_file, &tnz->nz_dec.nz);
                        }
                    }
                    if (verbose)
                    {
                        tnz->print_count++;
                        if (tnz->print_count >= VERBOSE_PRINT_DIVIDER)
                        {
                            tnz->print_count = 0;
                            naza_print_ascii(stderr, &tnz->nz_dec.nz);
                        }
                    }
                }
            }
        }
        usleep(SLEEP_DURATION_MS * 1000);
    }
}


void teensy_naza_process_messages(struct teensy_naza_t *tnz)
{
    libteensynaza_process_msg_1002(&tnz->nz_dec);
    libteensynaza_process_msg_1003(&tnz->nz_dec);
    libteensynaza_process_msg_1009(&tnz->nz_dec);
#ifdef GET_SMART_BATTERY_DATA
    libteensynaza_process_msg_0926(&tnz->nz_dec);
#endif
}

