#ifndef _LIBCANNAZA_H_
#define _LIBCANNAZA_H_

#include <linux/can.h>
#include <pthread.h>
#include "naza.h"


typedef enum
{ // order is important
    HEADER_55_1 = 0,
    HEADER_AA_1,
    HEADER_55_2,
    HEADER_AA_2,
    MSG_ID_1,
    MSG_ID_2,
    MSG_LEN_1,
    MSG_LEN_2,
    PAYLOAD,
    FOOTER_66_1,
    FOOTER_CC_1,
    FOOTER_66_2,
    FOOTER_CC_2,
    MSG_RX_DONE,
    MAX_STATE
} decoder_state_t;


struct can_naza_stats_t
{
    unsigned short canFrameError;
    unsigned short numMsg1002;
    unsigned short numMsg1003;
    unsigned short numMsg1009;
    unsigned short numMsg0926;
};


struct can_naza_decoder_info_t
{
    struct naza_info_t nz;
    struct can_naza_stats_t stats;

    int msg1002RejectCount;
    int msg1003RejectCount;
    int msg1009RejectCount;

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

    naza_msg_t msgBuf[NAZA_MESSAGE_COUNT];
    decoder_state_t decoderState[NAZA_MESSAGE_COUNT];
    decoder_state_t msgFooterState[NAZA_MESSAGE_COUNT];
    int msgBufIdx[NAZA_MESSAGE_COUNT];
};



void libcannaza_init(struct can_naza_decoder_info_t *nz_dec);
void libcannaza_uninit(struct can_naza_decoder_info_t *nz_dec);
void libcannaza_reset_stats(struct can_naza_decoder_info_t *nz_dec);
void libcannaza_process_can_frame(struct can_naza_decoder_info_t *nz_dec, struct canfd_frame *frame, int maxdlen);

void libcannaza_process_msg_1002(struct can_naza_decoder_info_t *nz_dec);
void libcannaza_process_msg_1003(struct can_naza_decoder_info_t *nz_dec);
void libcannaza_process_msg_1009(struct can_naza_decoder_info_t *nz_dec);
#ifdef GET_SMART_BATTERY_DATA
void libcannaza_process_msg_0926(struct can_naza_decoder_info_t *nz_dec);
#endif


#endif // _LIBCANNAZA_H_
