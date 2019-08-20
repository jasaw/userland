/*
 *    Filename: naza.c
 *
 * Copyright (c) 2015 DiUS
 *
 */

#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>

#include "naza.h"


int naza_dump_binary(const char *filename, struct naza_info_t *nz)
{
    // FIXME: dump a well defined struct rather than raw memory.
    int fd = open(filename, O_WRONLY | O_CREAT | O_APPEND, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    if (fd < 0)
    {
        fprintf(stderr, "Error: failed to open %s\n", filename);
        return -1;
    }
    int ret = write(fd, nz, sizeof(struct naza_info_t));
    if (ret != sizeof(struct naza_info_t))
    {
        int errorcode = errno;
        fprintf(stderr, "Error: write error (%d): %s\n", errorcode, strerror(errorcode));
    }
    close(fd);
    return 0;
}


void naza_print_ascii(FILE *fp, struct naza_info_t *nz)
{
    const char *gps_fix_str = "";
    switch (nz->fix)
    {
        case NO_FIX:   gps_fix_str = "NO"; break;
        case FIX_2D:   gps_fix_str = "2D"; break;
        case FIX_3D:   gps_fix_str = "3D"; break;
        case FIX_DGPS: gps_fix_str = "DGPS"; break;
    }
    // TODO: sanitize output if no GPS fix
    struct tm naza_timeinfo;
    struct tm *timeinfo;
    char time_buf[64];
    naza_timeinfo.tm_sec   = nz->second;
    naza_timeinfo.tm_min   = nz->minute;
    naza_timeinfo.tm_hour  = nz->hour;
    naza_timeinfo.tm_mday  = nz->day;   // 1-31
    naza_timeinfo.tm_mon   = nz->month - 1; // 0-11
    naza_timeinfo.tm_year  = nz->year + 100;  // since 1900
    naza_timeinfo.tm_isdst = -1; // Is DST on? 1 = yes, 0 = no, -1 = unknown
    time_t tmptime = timegm(&naza_timeinfo);
    timeinfo = localtime(&tmptime);
    strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", timeinfo);
    fprintf(fp, "NAZA_INFO: roll:%d, pitch:%d, heading:%.2f, "
            "alt:%.1f, gpsFix:%s, gpsDateTime:%s, gpsLat:%f, "
            "gpsLon:%f, gpsAlt:%.1f, gpsSpeed:%.2f, gpsTrack:%.2f, "
            "gpsNumSat:%d, batt:%.2fV(%d%%), motor:%d:%d:%d:%d:%d:%d:%d:%d\n",
            nz->roll, nz->pitch, nz->heading, nz->alt, gps_fix_str,
            time_buf, nz->lat, nz->lon, nz->gpsAlt, nz->spd, nz->cog,
            nz->sat, ((float)nz->battery)/1000, nz->batteryPercent,
            nz->motorOut[0], nz->motorOut[1], nz->motorOut[2], nz->motorOut[3],
            nz->motorOut[4], nz->motorOut[5], nz->motorOut[6], nz->motorOut[7]);
}
