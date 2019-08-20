//#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include <linux/can/error.h>

#include "libcannaza.h"


#define NAZA_MAX_NUM_SATELLITES             32
#define NAZA_MAX_GPS_LAT_CHANGE_DEGREES     0.01
#define NAZA_MAX_GPS_LON_CHANGE_DEGREES     0.01
#define NAZA_MAX_ALT_CHANGE_METERS          50.0
#define NAZA_MAX_INVALID_MSG                3
#define NAZA_MAX_SEQ_NUM_INCREMENT          25


void libcannaza_init(struct can_naza_decoder_info_t *nz_dec)
{
    int i;
    memset(nz_dec, 0, sizeof(struct can_naza_decoder_info_t));
    for (i = 0; i < NAZA_MESSAGE_COUNT; i++)
        nz_dec->msgFooterState[i] = FOOTER_66_1;
    pthread_mutex_init(&nz_dec->msg1002mutex, NULL);
    pthread_mutex_init(&nz_dec->msg1003mutex, NULL);
    pthread_mutex_init(&nz_dec->msg1009mutex, NULL);
#ifdef GET_SMART_BATTERY_DATA
    pthread_mutex_init(&nz_dec->msg0926mutex, NULL);
#endif
}


void libcannaza_uninit(struct can_naza_decoder_info_t *nz_dec)
{
    pthread_mutex_destroy(&nz_dec->msg1002mutex);
    pthread_mutex_destroy(&nz_dec->msg1003mutex);
    pthread_mutex_destroy(&nz_dec->msg1009mutex);
#ifdef GET_SMART_BATTERY_DATA
    pthread_mutex_destroy(&nz_dec->msg0926mutex);
#endif
}


inline static void reset_decoder_fsm(struct can_naza_decoder_info_t *nz_dec, int canMsgIdIdx)
{
    nz_dec->decoderState[canMsgIdIdx] = HEADER_55_1;
    nz_dec->msgFooterState[canMsgIdIdx] = FOOTER_66_1;
    nz_dec->msgBuf[canMsgIdIdx].header.id = 0;
    nz_dec->msgBuf[canMsgIdIdx].header.len = 0;
}


static int msg_1002_is_valid(naza_msg1002_t *new_msg1002, naza_msg1002_t *cached_msg1002)
{
    // teleportation not allowed
    if ((new_msg1002->numSat > NAZA_MAX_NUM_SATELLITES) ||
        (fabs(new_msg1002->altBaro - cached_msg1002->altBaro) > NAZA_MAX_ALT_CHANGE_METERS) ||
        (fabs(new_msg1002->lat - cached_msg1002->lat) > NAZA_MAX_GPS_LAT_CHANGE_DEGREES) ||
        (fabs(new_msg1002->lon - cached_msg1002->lon) > NAZA_MAX_GPS_LON_CHANGE_DEGREES) ||
        (fabs(new_msg1002->altGps - cached_msg1002->altGps) > NAZA_MAX_ALT_CHANGE_METERS) ||
        (new_msg1002->seqNum < cached_msg1002->seqNum) ||
        (abs(new_msg1002->seqNum - cached_msg1002->seqNum) > NAZA_MAX_SEQ_NUM_INCREMENT))
        return 0;
    return 1;
}


void libcannaza_process_msg_1002(struct can_naza_decoder_info_t *nz_dec)
{
    naza_msg1002_t *msg1002 = &nz_dec->cacheMsg1002;
    if (msg1002->header.id != NAZA_MESSAGE_MSG1002)
        return;

    pthread_mutex_lock(&nz_dec->msg1002mutex);

    nz_dec->nz.headingNc = -atan2(msg1002->magCalY, msg1002->magCalX) / M_PI * 180.0;
    if (nz_dec->nz.headingNc < 0)
        nz_dec->nz.headingNc += 360.0;
    nz_dec->nz.heading = atan2(msg1002->headCompY, msg1002->headCompX) / M_PI * 180.0;
    if (nz_dec->nz.heading < 0)
        nz_dec->nz.heading += 360.0;
    nz_dec->nz.sat = msg1002->numSat;
    nz_dec->nz.gpsAlt = msg1002->altGps;
    nz_dec->nz.lat = msg1002->lat / M_PI * 180.0;
    nz_dec->nz.lon = msg1002->lon / M_PI * 180.0;
    nz_dec->nz.alt = msg1002->altBaro;
    float nVel = msg1002->northVelocity;
    float eVel = msg1002->eastVelocity;
    nz_dec->nz.spd = sqrt(nVel * nVel + eVel * eVel);
    nz_dec->nz.cog = atan2(eVel, nVel) / M_PI * 180;
    if (nz_dec->nz.cog < 0)
        nz_dec->nz.cog += 360.0;
    nz_dec->nz.vsi = -msg1002->downVelocity;

    // Phantom 2 only
    nz_dec->nz.roll = (int)(msg1002->p2roll*100);
    nz_dec->nz.pitch = (int)(msg1002->p2pitch*100);
    nz_dec->nz.rollRad = (float)nz_dec->nz.roll * (M_PI / 180.0);
    nz_dec->nz.pitchRad = (float)nz_dec->nz.pitch * (M_PI / 180.0);

    pthread_mutex_unlock(&nz_dec->msg1002mutex);
}


static int msg_1003_is_valid(naza_msg1003_t *new_msg1003, naza_msg1003_t *cached_msg1003)
{
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;

    uint32_t dateTime = new_msg1003->dateTime;
    second = dateTime & 0b00111111; dateTime >>= 6;
    minute = dateTime & 0b00111111; dateTime >>= 6;
    hour = dateTime & 0b00001111; dateTime >>= 4;
    day = dateTime & 0b00011111; dateTime >>= 5; if (hour > 7) day++;
    month = dateTime & 0b00001111; dateTime >>= 4;
    year = dateTime & 0b01111111;

    if ((second >= 60) ||
        (minute >= 60) ||
        (month > 12) ||
        (year < 15) ||
        (new_msg1003->seqNum < cached_msg1003->seqNum) ||
        (abs(new_msg1003->seqNum - cached_msg1003->seqNum) > NAZA_MAX_SEQ_NUM_INCREMENT))
        return 0;
    return 1;
}


void libcannaza_process_msg_1003(struct can_naza_decoder_info_t *nz_dec)
{
    naza_msg1003_t *msg1003 = &nz_dec->cacheMsg1003;
    if (msg1003->header.id != NAZA_MESSAGE_MSG1003)
        return;

    pthread_mutex_lock(&nz_dec->msg1003mutex);

    uint32_t dateTime = msg1003->dateTime;
    nz_dec->nz.second = dateTime & 0b00111111; dateTime >>= 6;
    nz_dec->nz.minute = dateTime & 0b00111111; dateTime >>= 6;
    nz_dec->nz.hour = dateTime & 0b00001111; dateTime >>= 4;
    nz_dec->nz.day = dateTime & 0b00011111; dateTime >>= 5; if (nz_dec->nz.hour > 7) nz_dec->nz.day++;
    nz_dec->nz.month = dateTime & 0b00001111; dateTime >>= 4;
    nz_dec->nz.year = dateTime & 0b01111111;
    nz_dec->nz.gpsVsi = -msg1003->downVelocity;
    nz_dec->nz.vdop = (double)msg1003->vdop / 100;
    double ndop = (double)msg1003->ndop / 100;
    double edop = (double)msg1003->edop / 100;
    nz_dec->nz.hdop = sqrt(ndop * ndop + edop * edop);
    uint8_t fixType = msg1003->fixType;
    uint8_t fixFlags = msg1003->fixStatus;
    switch (fixType)
    {
        case 2 : nz_dec->nz.fix = FIX_2D; break;
        case 3 : nz_dec->nz.fix = FIX_3D; break;
        default: nz_dec->nz.fix = NO_FIX; break;
    }
    if ((nz_dec->nz.fix != NO_FIX) && (fixFlags & 0x02))
        nz_dec->nz.fix = FIX_DGPS;

    pthread_mutex_unlock(&nz_dec->msg1003mutex);
}


static int msg_1009_is_valid(naza_msg1009_t *new_msg1009, naza_msg1009_t *cached_msg1009)
{

    if ((new_msg1009->seqNum < cached_msg1009->seqNum) ||
        (abs(new_msg1009->seqNum - cached_msg1009->seqNum) > NAZA_MAX_SEQ_NUM_INCREMENT))
        return 0;
    return 1;
}


void libcannaza_process_msg_1009(struct can_naza_decoder_info_t *nz_dec)
{
    naza_msg1009_t *msg1009 = &nz_dec->cacheMsg1009;
    if (msg1009->header.id != NAZA_MESSAGE_MSG1009)
        return;

    pthread_mutex_lock(&nz_dec->msg1009mutex);

    int i;
    for (i = 0; i < 8; i++)
      nz_dec->nz.motorOut[i] = msg1009->motorOut[i];
    for (i = 0; i < 10; i++)
      nz_dec->nz.rcIn[i] = msg1009->rcIn[i];
#ifndef GET_SMART_BATTERY_DATA
    nz_dec->nz.battery = msg1009->batVolt;
#endif
    // Phantom 1 only
    //nz_dec->nz.rollRad = msg1009->p1roll;
    //nz_dec->nz.pitchRad = msg1009->p1pitch;
    //nz_dec->nz.roll = (int)(nz_dec->nz.rollRad * 180.0 / M_PI);
    //nz_dec->nz.pitch = (int)(nz_dec->nz.pitchRad * 180.0 / M_PI);
    nz_dec->nz.mode = (flight_mode_t)msg1009->flightMode;

    pthread_mutex_unlock(&nz_dec->msg1009mutex);
}


#ifdef GET_SMART_BATTERY_DATA
void libcannaza_process_msg_0926(struct can_naza_decoder_info_t *nz_dec)
{
    naza_msg0926_t *msg0926 = &nz_dec->cacheMsg0926;
    if (msg0926->header.id != NAZA_MESSAGE_MSG0926)
        return;

    pthread_mutex_lock(&nz_dec->msg0926mutex);

    int i;
    nz_dec->nz.battery = msg0926->voltage;
    nz_dec->nz.batteryPercent = msg0926->chargePercent;
    for (i = 0; i < 3; i++)
        nz_dec->nz.batteryCell[i] = msg0926->cellVoltage[i];

    pthread_mutex_unlock(&nz_dec->msg0926mutex);
}
#endif


void libcannaza_reset_stats(struct can_naza_decoder_info_t *nz_dec)
{
    memset(&nz_dec->stats, 0, sizeof(nz_dec->stats));
}


void libcannaza_process_can_frame(struct can_naza_decoder_info_t *nz_dec, struct canfd_frame *frame, int maxdlen)
{
    int canMsgIdIdx;
    int i;
	int len = (frame->len > maxdlen)? maxdlen : frame->len;

	if (frame->can_id & CAN_ERR_FLAG)
    {
        // error frame
        nz_dec->stats.canFrameError++;
        return;
    }
    else if (frame->can_id & CAN_EFF_FLAG)
    {
        // extended frame format not supported
        return;
    }
    else if (frame->can_id & CAN_RTR_FLAG)
    {
        // remote request not supported
        return;
    }

    switch (frame->can_id & CAN_SFF_MASK)
    {
        case 0x090: canMsgIdIdx = 0; break;
        case 0x108: canMsgIdIdx = 1; break;
        case 0x7F8: canMsgIdIdx = 2; break;
        default: return; // message not supported
    }

    for (i = 0; i < len; i++)
    {
        switch (nz_dec->decoderState[canMsgIdIdx])
        {
            case HEADER_55_1:
            case HEADER_55_2:
                if (frame->data[i] == 0x55)
                    nz_dec->decoderState[canMsgIdIdx]++;
                else
                    reset_decoder_fsm(nz_dec, canMsgIdIdx);
                break;

            case HEADER_AA_1:
            case HEADER_AA_2:
                if (frame->data[i] == 0xAA)
                {
                    nz_dec->decoderState[canMsgIdIdx]++;
                    nz_dec->msgBufIdx[canMsgIdIdx] = 0;
                }
                else
                    reset_decoder_fsm(nz_dec, canMsgIdIdx);
                break;

            case MSG_ID_1:
            case MSG_ID_2:
            case MSG_LEN_1:
            case MSG_LEN_2:
                if (nz_dec->msgBufIdx[canMsgIdIdx] < sizeof(nz_dec->msgBuf[canMsgIdIdx].bytes))
                    nz_dec->msgBuf[canMsgIdIdx].bytes[nz_dec->msgBufIdx[canMsgIdIdx]++] = frame->data[i];
                nz_dec->decoderState[canMsgIdIdx]++;
                break;

            case PAYLOAD:
                if (nz_dec->msgBuf[canMsgIdIdx].header.len > sizeof(nz_dec->msgBuf[canMsgIdIdx].bytes) - 4)
                {
                    // invalid message length
                    reset_decoder_fsm(nz_dec, canMsgIdIdx);
                }
                if (nz_dec->msgBufIdx[canMsgIdIdx] < sizeof(nz_dec->msgBuf[canMsgIdIdx].bytes))
                    nz_dec->msgBuf[canMsgIdIdx].bytes[nz_dec->msgBufIdx[canMsgIdIdx]++] = frame->data[i];
                if (nz_dec->msgBufIdx[canMsgIdIdx] >= nz_dec->msgBuf[canMsgIdIdx].header.len + 4)
                    nz_dec->decoderState[canMsgIdIdx]++;
                break;

            case FOOTER_66_1:
            case FOOTER_66_2:
                if (frame->data[i] == 0x66)
                    nz_dec->decoderState[canMsgIdIdx]++;
                else
                    reset_decoder_fsm(nz_dec, canMsgIdIdx);
                break;

            case FOOTER_CC_1:
            case FOOTER_CC_2:
                if (frame->data[i] == 0xCC)
                    nz_dec->decoderState[canMsgIdIdx]++;
                else
                    reset_decoder_fsm(nz_dec, canMsgIdIdx);
                break;
            
            default:
                reset_decoder_fsm(nz_dec, canMsgIdIdx);
        }
        
        // look for possible early message termination due to dropped packets
        switch (nz_dec->msgFooterState[canMsgIdIdx])
        {
            case FOOTER_66_1:
            case FOOTER_66_2:
                if (frame->data[i] == 0x66)
                    nz_dec->msgFooterState[canMsgIdIdx]++;
                else
                    nz_dec->msgFooterState[canMsgIdIdx] = FOOTER_66_1;
                break;
            case FOOTER_CC_1:
            case FOOTER_CC_2:
                if (frame->data[i] == 0xCC)
                    nz_dec->msgFooterState[canMsgIdIdx]++;
                else
                    nz_dec->msgFooterState[canMsgIdIdx] = FOOTER_66_1;
                break;
            default:
                nz_dec->msgFooterState[canMsgIdIdx] = FOOTER_66_1;
        }

        if ((nz_dec->decoderState[canMsgIdIdx] == MSG_RX_DONE) ||
            (nz_dec->msgFooterState[canMsgIdIdx] == MSG_RX_DONE))
        {
            if (nz_dec->decoderState[canMsgIdIdx] != nz_dec->msgFooterState[canMsgIdIdx])
            {
                // unexpected termination of message
                reset_decoder_fsm(nz_dec, canMsgIdIdx);
                continue;
            }
            // process received message
            switch (nz_dec->msgBuf[canMsgIdIdx].header.id)
            {
                case NAZA_MESSAGE_MSG1002:
                    if ((nz_dec->msg1002RejectCount >= NAZA_MAX_INVALID_MSG) ||
                        (msg_1002_is_valid(&nz_dec->msgBuf[canMsgIdIdx].msg1002, &nz_dec->cacheMsg1002)))
                    {
                        nz_dec->msg1002RejectCount = 0;
                        pthread_mutex_lock(&nz_dec->msg1002mutex);
                        memcpy(&nz_dec->cacheMsg1002, &nz_dec->msgBuf[canMsgIdIdx].msg1002, sizeof(nz_dec->cacheMsg1002));
                        pthread_mutex_unlock(&nz_dec->msg1002mutex);
                        nz_dec->stats.numMsg1002++;
                    }
                    else
                        nz_dec->msg1002RejectCount++;
                    break;
                case NAZA_MESSAGE_MSG1003:
                    if ((nz_dec->msg1003RejectCount >= NAZA_MAX_INVALID_MSG) ||
                        (msg_1003_is_valid(&nz_dec->msgBuf[canMsgIdIdx].msg1003, &nz_dec->cacheMsg1003)))
                    {
                        nz_dec->msg1003RejectCount = 0;
                        pthread_mutex_lock(&nz_dec->msg1003mutex);
                        memcpy(&nz_dec->cacheMsg1003, &nz_dec->msgBuf[canMsgIdIdx].msg1003, sizeof(nz_dec->cacheMsg1003));
                        pthread_mutex_unlock(&nz_dec->msg1003mutex);
                        nz_dec->stats.numMsg1003++;
                    }
                    else
                        nz_dec->msg1003RejectCount++;
                    break;
                case NAZA_MESSAGE_MSG1009:
                    if ((nz_dec->msg1009RejectCount >= NAZA_MAX_INVALID_MSG) ||
                        (msg_1009_is_valid(&nz_dec->msgBuf[canMsgIdIdx].msg1009, &nz_dec->cacheMsg1009)))
                    {
                        nz_dec->msg1009RejectCount = 0;
                        pthread_mutex_lock(&nz_dec->msg1009mutex);
                        memcpy(&nz_dec->cacheMsg1009, &nz_dec->msgBuf[canMsgIdIdx].msg1009, sizeof(nz_dec->cacheMsg1009));
                        pthread_mutex_unlock(&nz_dec->msg1009mutex);
                        nz_dec->stats.numMsg1009++;
                    }
                    else
                        nz_dec->msg1009RejectCount++;
                    break;
#ifdef GET_SMART_BATTERY_DATA
                case NAZA_MESSAGE_MSG0926:
                    pthread_mutex_lock(&nz_dec->msg0926mutex);
                    memcpy(&nz_dec->cacheMsg0926, &nz_dec->msgBuf[canMsgIdIdx].msg0926, sizeof(nz_dec->cacheMsg0926));
                    pthread_mutex_unlock(&nz_dec->msg0926mutex);
                    nz_dec->stats.numMsg0926++;
                    break;
#endif
            }
            reset_decoder_fsm(nz_dec, canMsgIdIdx);
        }
    }


//    // HACK: print info
//    static struct timespec prev_print_time = {.tv_sec = 0};
//    struct timespec now;
//    if ((clock_gettime(CLOCK_MONOTONIC, &now) == 0) &&
//        (now.tv_sec - prev_print_time.tv_sec >= 1))
//    {
//        prev_print_time = now;
//
//        libcannaza_process_msg_1002(nz_dec);
//        libcannaza_process_msg_1003(nz_dec);
//        libcannaza_process_msg_1009(nz_dec);
//#ifdef GET_SMART_BATTERY_DATA
//        libcannaza_process_msg_0926(nz_dec);
//#endif
//
//        const char *gps_fix_str = "";
//        switch (nz_dec->nz.fix)
//        {
//            case NO_FIX:   gps_fix_str = "NO  "; break;
//            case FIX_2D:   gps_fix_str = "2D  "; break;
//            case FIX_3D:   gps_fix_str = "3D  "; break;
//            case FIX_DGPS: gps_fix_str = "DGPS"; break;
//        }
//
//        fprintf(stdout, "AHRS: roll % 3d pitch % 3d heading(% 7.2f,% 7.2f,% 7.2f) "
//                "alt% 7.2f gpsAlt% 7.2f latlon(% 7.2f,% 7.2f) gpsFix %s "
//                "gpsDateTime %04d-%02d-%02d %02d:%02d:%02d batt %fV (%d%%)\n",
//                nz_dec->nz.roll, nz_dec->nz.pitch, nz_dec->nz.headingNc,
//                nz_dec->nz.heading, nz_dec->nz.cog, nz_dec->nz.alt, nz_dec->nz.gpsAlt,
//                nz_dec->nz.lat, nz_dec->nz.lon, gps_fix_str,
//                nz_dec->nz.year, nz_dec->nz.month, nz_dec->nz.day,
//                nz_dec->nz.hour, nz_dec->nz.minute, nz_dec->nz.second,
//                ((float)nz_dec->nz.battery)/1000, nz_dec->nz.batteryPercent);
//        fprintf(stdout, "Motor: %d, %d, %d, %d, %d, %d, %d, %d\n",
//                nz_dec->nz.motorOut[0], nz_dec->nz.motorOut[1], nz_dec->nz.motorOut[2], nz_dec->nz.motorOut[3],
//                nz_dec->nz.motorOut[4], nz_dec->nz.motorOut[5], nz_dec->nz.motorOut[6], nz_dec->nz.motorOut[7]);
//        fprintf(stdout, "Stats : error frames = %d,    msg 1002 = %d,    "
//                "msg 1003 = %d,    msg 1009 = %d,    msg 0926 = %d\n",
//                nz_dec->stats.canFrameError, nz_dec->stats.numMsg1002,
//                nz_dec->stats.numMsg1003, nz_dec->stats.numMsg1009, nz_dec->stats.numMsg0926);
//        libcannaza_reset_stats(nz_dec);
//    }
}
