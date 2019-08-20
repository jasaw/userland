#ifndef _LIBTEENSYNAZA_H_
#define _LIBTEENSYNAZA_H_

#include <pthread.h>
#include "naza.h"


struct ts_naza_stats_t
{
    unsigned short numMsg1002;
    unsigned short numMsg1003;
    unsigned short numMsg1009;
    unsigned short numMsg0926;
};


struct ts_naza_decoder_info_t
{
    struct naza_info_t nz;
    struct ts_naza_stats_t stats;

    naza_msg1002_t cacheMsg1002;
    naza_msg1003_t cacheMsg1003;
    naza_msg1009_t cacheMsg1009;
#ifdef GET_SMART_BATTERY_DATA
    naza_msg0926_t cacheMsg0926;
#endif

    pthread_mutex_t msg1002mutex;
    pthread_mutex_t msg1003mutex;
    pthread_mutex_t msg1009mutex;
#ifdef GET_SMART_BATTERY_DATA
    pthread_mutex_t msg0926mutex;
#endif
};



void libteensynaza_init(struct ts_naza_decoder_info_t *nz_dec);
void libteensynaza_uninit(struct ts_naza_decoder_info_t *nz_dec);

void libteensynaza_cache_msg_1002(struct ts_naza_decoder_info_t *nz_dec, naza_msg1002_t *msg);
void libteensynaza_cache_msg_1003(struct ts_naza_decoder_info_t *nz_dec, naza_msg1003_t *msg);
void libteensynaza_cache_msg_1009(struct ts_naza_decoder_info_t *nz_dec, naza_msg1009_t *msg);
#ifdef GET_SMART_BATTERY_DATA
void libteensynaza_cache_msg_0926(struct ts_naza_decoder_info_t *nz_dec, naza_msg0926_t *msg);
#endif

void libteensynaza_process_msg_1002(struct ts_naza_decoder_info_t *nz_dec);
void libteensynaza_process_msg_1003(struct ts_naza_decoder_info_t *nz_dec);
void libteensynaza_process_msg_1009(struct ts_naza_decoder_info_t *nz_dec);
#ifdef GET_SMART_BATTERY_DATA
void libteensynaza_process_msg_0926(struct ts_naza_decoder_info_t *nz_dec);
#endif

void libteensynaza_reset_stats(struct ts_naza_decoder_info_t *nz_dec);


#endif // _LIBTEENSYNAZA_H_
