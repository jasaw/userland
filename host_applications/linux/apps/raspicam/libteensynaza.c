//#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "libteensynaza.h"


void libteensynaza_init(struct ts_naza_decoder_info_t *nz_dec)
{
    memset(nz_dec, 0, sizeof(struct ts_naza_decoder_info_t));
    pthread_mutex_init(&nz_dec->msg1002mutex, NULL);
    pthread_mutex_init(&nz_dec->msg1003mutex, NULL);
    pthread_mutex_init(&nz_dec->msg1009mutex, NULL);
#ifdef GET_SMART_BATTERY_DATA
    pthread_mutex_init(&nz_dec->msg0926mutex, NULL);
#endif
}


void libteensynaza_uninit(struct ts_naza_decoder_info_t *nz_dec)
{
    pthread_mutex_destroy(&nz_dec->msg1002mutex);
    pthread_mutex_destroy(&nz_dec->msg1003mutex);
    pthread_mutex_destroy(&nz_dec->msg1009mutex);
#ifdef GET_SMART_BATTERY_DATA
    pthread_mutex_destroy(&nz_dec->msg0926mutex);
#endif
}


void libteensynaza_process_msg_1002(struct ts_naza_decoder_info_t *nz_dec)
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


void libteensynaza_process_msg_1003(struct ts_naza_decoder_info_t *nz_dec)
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


void libteensynaza_process_msg_1009(struct ts_naza_decoder_info_t *nz_dec)
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
void libteensynaza_process_msg_0926(struct ts_naza_decoder_info_t *nz_dec)
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


void libteensynaza_cache_msg_1002(struct ts_naza_decoder_info_t *nz_dec, naza_msg1002_t *msg)
{
    pthread_mutex_lock(&nz_dec->msg1002mutex);
    memcpy(&nz_dec->cacheMsg1002, msg, sizeof(nz_dec->cacheMsg1002));
    pthread_mutex_unlock(&nz_dec->msg1002mutex);
    nz_dec->stats.numMsg1002++;
}


void libteensynaza_cache_msg_1003(struct ts_naza_decoder_info_t *nz_dec, naza_msg1003_t *msg)
{
    pthread_mutex_lock(&nz_dec->msg1003mutex);
    memcpy(&nz_dec->cacheMsg1003, msg, sizeof(nz_dec->cacheMsg1003));
    pthread_mutex_unlock(&nz_dec->msg1003mutex);
    nz_dec->stats.numMsg1003++;
}


void libteensynaza_cache_msg_1009(struct ts_naza_decoder_info_t *nz_dec, naza_msg1009_t *msg)
{
    pthread_mutex_lock(&nz_dec->msg1009mutex);
    memcpy(&nz_dec->cacheMsg1009, msg, sizeof(nz_dec->cacheMsg1009));
    pthread_mutex_unlock(&nz_dec->msg1009mutex);
    nz_dec->stats.numMsg1009++;
}


#ifdef GET_SMART_BATTERY_DATA
void libteensynaza_cache_msg_0926(struct ts_naza_decoder_info_t *nz_dec, naza_msg0926_t *msg)
{
    pthread_mutex_lock(&nz_dec->msg0926mutex);
    memcpy(&nz_dec->cacheMsg0926, msg, sizeof(nz_dec->cacheMsg0926));
    pthread_mutex_unlock(&nz_dec->msg0926mutex);
    nz_dec->stats.numMsg0926++;
}
#endif


void libteensynaza_reset_stats(struct ts_naza_decoder_info_t *nz_dec)
{
    memset(&nz_dec->stats, 0, sizeof(nz_dec->stats));
}
