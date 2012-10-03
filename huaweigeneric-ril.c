/*
 **
 ** Copyright 2006, The Android Open Source Project
 ** Copyright 2012 Eduardo Jos[e Tagle <ejtagle@tutopia.com>
 **
 ** Licensed under the Apache License, Version 2.0 (the "License");
 ** you may not use this file except in compliance with the License.
 ** You may obtain a copy of the License at
 **
 **     http://www.apache.org/licenses/LICENSE-2.0
 **
 ** Unless required by applicable law or agreed to in writing, software
 ** distributed under the License is distributed on an "AS IS" BASIS,
 ** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 ** See the License for the specific language governing permissions and
 ** limitations under the License.
 */

//#define ORG_DIAL 0
//#define BYPASS_AUDIO_CHECK 1


/*
The status updates from the second port are prefixed with a caret and are of these forms:

^MODE:3,2 indicates GPRS
^MODE:3,3 indicates EDGE
^MODE:5,4 indicates 3G
^MODE:5,5 indicates HSDPA

^DSFLOWRPT:n,n,n,n,n,n,n
n1 is the duration of the connection in seconds
n2 is transmit (upload) speed in bytes per second (n2 *8 / 1000 will give you kbps)
n3 is receive (download) speed in bytes per second (n3 *8 / 1000 will give you kbps)
n4 is the total bytes transmitted during this session
n5 is the total bytes transmitted during this session
n6 no idea, but I always get 0x1F40 (8000) here
n7 no idea, but I always get 0x7530 (30000) here

You can reset the connection statistics by sending AT^DSFLOWCLR.

---
If anyone is interested, sending DTMF signals in modem-mode (+FCLASS=0) is possible. But it is device-dependent feature.
For Huawei E1550 it is AT^DTMF=<n> command.

----

AT^CVOICE=0 Enable Voice
AT^CVOICE=1 Disable Voice
AT^CVOICE=? Check status

AT^CVOICE? [len=10]
        [len=0]
        <LF>^CVOICE:0,8000,16,20 [len=21] This answer tells you voice parameters (8 khz,16 bits, 20 ms)
        <LF>

----
That need to be tested but voice activation should work with:
AT^U2DIAG=5

then the modem become 0x12d1:0x140c a flash drive.

then I plug in on my Linux Ubuntu computer, use minicom and type AT^U2DIAG=0 to deactivate the damn flash drive.

Maybe it's better to first type AT^U2DIAG=0 and then AT^U2DIAG=5 ...

Regards, Phil.


-----
1)with unaccepted sim
-> AT^CARDLOCK?
<-^CARDLOCK: 1,10,0 ............ locked
<-^CARDLOCK: 2,10,0 ............ unlocked
Quote:
Originally Posted by sergeymkl 
4. second number of reply to CARDLOCK inquiry.
<-^CARDLOCK: 1,10,0 ............ locked
<-^CARDLOCK: 2,10,0 ............ unlocked
<-^CARDLOCK: A,B,C ............
A = is to verify if locked or unlocked
B = attempts left? or total given attempts?
C = is for?

*/

#define LOG_NDEBUG 0
#define LOG_TAG "RIL"
#include <utils/Log.h>

#include <telephony/ril.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <alloca.h>
#include "atchannel.h"
#include "at_tok.h"
#include "misc.h"
#include "net-utils.h"
#include "gsm.h"
#include "requestdatahandler.h"
#include "fcp_parser.h"
#include "audiochannel.h"
#include <getopt.h>
#include <sys/socket.h>
#include <cutils/sockets.h>
#include <termios.h>
#include <cutils/properties.h>
#include <cutils/logger.h>
#include <cutils/logd.h>
#include <stdbool.h>
#include <stddef.h>

#include <linux/if.h>
#include <linux/sockios.h>
#include <linux/route.h>
#include <linux/wireless.h> 

#define D ALOGI

#define RIL_VERSION_STRING "Huawei-ril 1.0.0.8 (Built on " __DATE__" " __TIME__ ")"

#define timespec_cmp(a, b, op)         \
        ((a).tv_sec == (b).tv_sec    \
        ? (a).tv_nsec op (b).tv_nsec \
        : (a).tv_sec op (b).tv_sec)

#define TIMEOUT_SEARCH_FOR_TTY 5 /* Poll every Xs for the port*/
#define TIMEOUT_EMRDY 10 /* Module should respond at least within 10s */
#define MAX_BUF 1024

/** declarations */
static const char *getVersion();
static void signalCloseQueues(void);
static void onRequest (int request, void *data, size_t datalen, RIL_Token t);
static int onSupports (int requestCode);
static void onCancel (RIL_Token t);
extern const char *requestToString(int request);

static RIL_RadioState getRadioState();
static int isRadioOn();
static int getCardStatus(RIL_CardStatus_v6 **pp_card_status);
static void freeCardStatus(RIL_CardStatus_v6 *p_card_status);
static void onDataCallListChanged(void *param);
static int killConn(const char * cid);
static int wait_for_property(const char *name, const char *desired_value, int maxwait, int allowempty);
static void checkMessageStorageReady(void *p);
static int pppSupported(void);
static void onSIMReady(void *p);
static void pollSIMState(void *param);
static void checkMessageStorageReady(void *p);

extern const char * requestToString(int request);

/*** Static Variables ***/
static const RIL_RadioFunctions s_callbacks = {
    RIL_VERSION,
    onRequest,
    getRadioState,
    onSupports,
    onCancel,
    getVersion
};

static char* rndis_iface_dev = "/dev/ttyUSB0";
static char* ppp_iface_dev   = "/dev/ttyUSB0";
static int using_rndis = 0;

/* Last call index status */
static int lastCallIdx = 0;

/* Last call fail cause */
static int lastCallFailCause = CALL_FAIL_NORMAL;

#define RNDIS_IFACE "rmnet0"
#define PPP_IFACE   "ppp0"

static RIL_RadioState sState = RADIO_STATE_UNAVAILABLE;
static pthread_mutex_t s_state_mutex = PTHREAD_MUTEX_INITIALIZER;
static const struct timespec TIMEVAL_SIMPOLL = {1,0};


#ifdef RIL_SHLIB
static const struct RIL_Env *s_rilenv;
#define RIL_onRequestComplete(t, e, response, responselen) s_rilenv->OnRequestComplete(t,e, response, responselen)
#define RIL_onUnsolicitedResponse(a,b,c) s_rilenv->OnUnsolicitedResponse(a,b,c)
#define RIL_requestTimedCallback(a,b,c) s_rilenv->RequestTimedCallback(a,b,c)
#endif

typedef struct RILRequest {
    int request;
    void *data;
    size_t datalen;
    RIL_Token token;
    struct RILRequest *next;
} RILRequest;

typedef struct RILEvent {
    void (*eventCallback) (void *param);
    void *param;
    struct timespec abstime;
    struct RILEvent *next;
    struct RILEvent *prev;
} RILEvent;

typedef struct RequestQueue {
    pthread_mutex_t queueMutex;
    pthread_cond_t cond;
    RILRequest *requestList;
    RILEvent *eventList;
    char enabled;
    char closed;
} RequestQueue;

static RequestQueue s_requestQueue = {
    PTHREAD_MUTEX_INITIALIZER,
    PTHREAD_COND_INITIALIZER,
    NULL,
    NULL,
    1,
    1
};

static RequestQueue *s_requestQueues[] = {
    &s_requestQueue
};

/* The Audio channel */
static struct GsmAudioTunnel sAudioChannel;
static char  sAudioDevice[64] = {0};

static int audioTunnelMuted = 0; // If audio tunnel is muted

/* Starts the audio tunnel channel */
static void startAudioTunnel(void* param)
{
    ATResponse *atResponse = NULL;
    int err;
    char *line;
    int voice_disabled = 0, sampling_rate = 8000,
        bits_per_sample = 16, framesize_ms = 20;

    if (gsm_audio_tunnel_running(&sAudioChannel))
        return;

    // Enable voice function - Some firmwares fail this command, but 
	// nevertheless, it works. Leave validation for the next one.
    err = at_send_command("AT^CVOICE=0");

#ifndef BYPASS_AUDIO_CHECK
    // Check status - This is the check that matters!
    err = at_send_command_singleline("AT^CVOICE?", "^CVOICE:", &atResponse);
    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;
    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &voice_disabled);
    if (err < 0) goto error;

	// Check if voice was really enabled
	if (voice_disabled != 0) goto error;

    err = at_tok_nextint(&line, &sampling_rate);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &bits_per_sample);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &framesize_ms);
    if (err < 0) goto error;
#else
	sampling_rate = 8000;
	bits_per_sample = 16;
	framesize_ms = 20;
#endif
	
	/* Increase volume to maximum. Some modems do not set the volume if not
	  actually changing values.... And some of them do not understand the 
	  command at all. Try to switch volume to maximum - DerArtem */
	err = at_send_command("AT+CLVL=1"); /* Lo volume */
	err = at_send_command("AT+CLVL=5"); /* Maximum volume */
	
    /* Start the audio channel */
    err = gsm_audio_tunnel_start(&sAudioChannel,sAudioDevice,
                sampling_rate, (sampling_rate * framesize_ms) / 1000, bits_per_sample );
    if (err)
        goto error;

	// Mute/Ummute audioTunnel as requested
	gsm_audio_tunnel_mute(&sAudioChannel,audioTunnelMuted);

	ALOGD("Audio Tunnel enabled");
    at_response_free(atResponse);
    return;

error:
	ALOGE("Failed to start audio tunnel");

    at_response_free(atResponse);
    return;
}

/* Stops the audio tunnel channel */
static void stopAudioTunnel(void* param)
{
	if (!gsm_audio_tunnel_running(&sAudioChannel))
        return;

	// Stop the audio tunnel
    gsm_audio_tunnel_stop(&sAudioChannel);

    ALOGD("Audio Tunnel disabled");
}

static void muteAudioTunnel(int muteit)
{
	audioTunnelMuted = muteit;
		
    ALOGD("Audio Tunnel %smuted",muteit?"":"un");

	if (!gsm_audio_tunnel_running(&sAudioChannel))
        return;

	// Mute/Ummute audioTunnel if requested
	gsm_audio_tunnel_mute(&sAudioChannel,audioTunnelMuted);
}

/**
 * Enqueue a RILEvent to the request queue.
 */
static void enqueueRILEvent(void (*callback) (void *param),
                     void *param, const struct timespec *relativeTime)
{
    int err;
    struct timespec ts;
    char done = 0;
    RequestQueue *q = NULL;

    RILEvent *e = (RILEvent *) malloc(sizeof(RILEvent));
    memset(e, 0, sizeof(RILEvent));

    e->eventCallback = callback;
    e->param = param;

    if (relativeTime == NULL) {
        relativeTime = (const struct timespec *) alloca(sizeof(struct timespec));
        memset((struct timespec *) relativeTime, 0, sizeof(struct timespec));
    }

    clock_gettime(CLOCK_MONOTONIC, &ts);

    e->abstime.tv_sec = ts.tv_sec + relativeTime->tv_sec;
    e->abstime.tv_nsec = ts.tv_nsec + relativeTime->tv_nsec;

    if (e->abstime.tv_nsec > 1000000000) {
        e->abstime.tv_sec++;
        e->abstime.tv_nsec -= 1000000000;
    }

    q = &s_requestQueue;

again:
    if ((err = pthread_mutex_lock(&q->queueMutex)) != 0)
        ALOGE("%s() failed to take queue mutex: %s!", __func__, strerror(err));

    if (q->eventList == NULL) {
        q->eventList = e;
    } else {
        if (timespec_cmp(q->eventList->abstime, e->abstime, > )) {
            e->next = q->eventList;
            q->eventList->prev = e;
            q->eventList = e;
        } else {
            RILEvent *tmp = q->eventList;
            do {
                if (timespec_cmp(tmp->abstime, e->abstime, > )) {
                    tmp->prev->next = e;
                    e->prev = tmp->prev;
                    tmp->prev = e;
                    e->next = tmp;
                    break;
                } else if (tmp->next == NULL) {
                    tmp->next = e;
                    e->prev = tmp;
                    break;
                }
                tmp = tmp->next;
            } while (tmp);
        }
    }

    if ((err = pthread_cond_broadcast(&q->cond)) != 0)
        ALOGE("%s() failed to take broadcast queue update: %s!",
            __func__, strerror(err));

    if ((err = pthread_mutex_unlock(&q->queueMutex)) != 0)
        ALOGE("%s() failed to release queue mutex: %s!",
            __func__, strerror(err));

}


/** returns 1 if on, 0 if off, and -1 on error */
static int isRadioOn()
{
    ATResponse *atResponse = NULL;
    int err;
    char *line;
    char ret1;

	err = at_send_command_singleline("AT+CFUN?", "+CFUN:", &atResponse);
	if (err != AT_NOERROR) {
		// assume radio is off
		goto error;
	}
	
	line = atResponse->p_intermediates->line;

	err = at_tok_start(&line);
	if (err < 0) goto error;

	err = at_tok_nextbool(&line, &ret1);
	if (err < 0) goto error;

	at_response_free(atResponse);

	ALOGD("Radio is %s", (ret1 == 1) ? "On" : "Off");
	
	/* 1=means online, all the others, mean offline */
	return (ret1 == 1) ? 1 : 0;

error:
    at_response_free(atResponse);
    return -1;
}


static const char *radioStateToString(RIL_RadioState radioState)
{
    const char *state;

    switch (radioState) {
    case RADIO_STATE_OFF:
        state = "RADIO_STATE_OFF";
        break;
    case RADIO_STATE_UNAVAILABLE:
        state = "RADIO_STATE_UNAVAILABLE";
        break;
    case RADIO_STATE_SIM_NOT_READY:
        state = "RADIO_STATE_SIM_NOT_READY";
        break;
    case RADIO_STATE_SIM_LOCKED_OR_ABSENT:
        state = "RADIO_STATE_SIM_LOCKED_OR_ABSENT";
        break;
    case RADIO_STATE_SIM_READY:
        state = "RADIO_STATE_SIM_READY";
        break;
    case RADIO_STATE_RUIM_NOT_READY:
        state = "RADIO_STATE_RUIM_NOT_READY";
        break;
    case RADIO_STATE_RUIM_READY:
        state = "RADIO_STATE_RUIM_READY";
        break;
    case RADIO_STATE_RUIM_LOCKED_OR_ABSENT:
        state = "RADIO_STATE_RUIM_READY";
        break;
    case RADIO_STATE_NV_NOT_READY:
        state = "RADIO_STATE_NV_NOT_READY";
        break;
    case RADIO_STATE_NV_READY:
        state = "RADIO_STATE_NV_READY";
        break;
    default:
        state = "RADIO_STATE_<> Unknown!";
        break;
    }

    return state;
}


/**
 * Synchronous call from the RIL to us to return current radio state.
 * RADIO_STATE_UNAVAILABLE should be the initial state.
 */
static RIL_RadioState getRadioState()
{
    return sState;
}

static void setRadioState(RIL_RadioState newState)
{
    RIL_RadioState oldState;
    int err;

    if ((err = pthread_mutex_lock(&s_state_mutex)) != 0)
        ALOGE("%s() failed to take state mutex: %s!", __func__, strerror(err));

    oldState = sState;

    ALOGI("%s() oldState=%s newState=%s", __func__, radioStateToString(oldState),
         radioStateToString(newState));

    sState = newState;

    if ((err = pthread_mutex_unlock(&s_state_mutex)) != 0)
        ALOGE("%s() failed to release state mutex: %s!", __func__, strerror(err));

    /* Do these outside of the mutex. */
    if (sState != oldState || sState == RADIO_STATE_SIM_LOCKED_OR_ABSENT) {
        RIL_onUnsolicitedResponse(RIL_UNSOL_RESPONSE_RADIO_STATE_CHANGED,
                                  NULL, 0);

        if (sState == RADIO_STATE_SIM_READY) {
            enqueueRILEvent(checkMessageStorageReady, NULL, NULL);
            enqueueRILEvent(onSIMReady, NULL, NULL);
        } else if (sState == RADIO_STATE_SIM_NOT_READY)
            enqueueRILEvent(pollSIMState, NULL, NULL);
    }
}

/*
-> AT^CARDLOCK?
<-^CARDLOCK: 1,10,0 ............ locked to an operator: Can be unlocked by code
<-^CARDLOCK: 2,10,0 ............ unlocked
<-^CARDLOCK: 3,10,0 ............ locked: Datacard is locked forever to an operator
<-^CARDLOCK: A,B,C ............
A = is to verify if locked or unlocked
B = attempts left? or total given attempts?
C = is for?
*/

static int getSimLockStatus(int* status, int* times)
{
    ATResponse *atResponse = NULL;
    int err;
    char *line;

    err = at_send_command_singleline("AT^CARDLOCK?", "^CARDLOCK:", &atResponse);
	
	if (err != AT_NOERROR) {
        // assume locked
        goto error;
    }

    line = atResponse->p_intermediates->line;

    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, status);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, times);
    if (err < 0) goto error;

    at_response_free(atResponse);

	ALOGD("Card %s, times: %d", *status == 1 ? "Locked" : "Unlocked" , *times);
    return 0;

error:
    at_response_free(atResponse);
    return -1;
}


typedef enum {
    SIM_ABSENT = 0,                     /* SIM card is not inserted */
    SIM_NOT_READY = 1,                  /* SIM card is not ready */
    SIM_READY = 2,                      /* radiostate = RADIO_STATE_SIM_READY */
    SIM_PIN = 3,                        /* SIM PIN code lock */
    SIM_PUK = 4,                        /* SIM PUK code lock */
    SIM_NETWORK_PERSO = 5,              /* Network Personalization lock */
    SIM_PIN2 = 6,                       /* SIM PIN2 lock */
    SIM_PUK2 = 7,                       /* SIM PUK2 lock */
    SIM_NETWORK_SUBSET_PERSO = 8,       /* Network Subset Personalization */
    SIM_SERVICE_PROVIDER_PERSO = 9,     /* Service Provider Personalization */
    SIM_CORPORATE_PERSO = 10,           /* Corporate Personalization */
    SIM_SIM_PERSO = 11,                 /* SIM/USIM Personalization */
    SIM_STERICSSON_LOCK = 12,           /* ST-Ericsson Extended SIM */
    SIM_BLOCKED = 13,                   /* SIM card is blocked */
    SIM_PERM_BLOCKED = 14,              /* SIM card is permanently blocked */
    SIM_NETWORK_PERSO_PUK = 15,         /* Network Personalization PUK */
    SIM_NETWORK_SUBSET_PERSO_PUK = 16,  /* Network Subset Perso. PUK */
    SIM_SERVICE_PROVIDER_PERSO_PUK = 17,/* Service Provider Perso. PUK */
    SIM_CORPORATE_PERSO_PUK = 18,       /* Corporate Personalization PUK */
    SIM_SIM_PERSO_PUK = 19,             /* SIM Personalization PUK (unused) */
    SIM_PUK2_PERM_BLOCKED = 20          /* PUK2 is permanently blocked */
} SIM_Status;

typedef enum {
    UICC_TYPE_UNKNOWN,
    UICC_TYPE_SIM,
    UICC_TYPE_USIM,
    UICC_TYPE_UIM,
} UICC_Type;



/** Returns one of SIM_*. Returns SIM_NOT_READY on error. */
static SIM_Status getSIMStatus(void)
{
    ATResponse *atResponse = NULL;
    int err;
    SIM_Status ret = SIM_ABSENT;
    char *cpinLine;
    char *cpinResult;
	int status, times;

    if (getRadioState() == RADIO_STATE_OFF ||
        getRadioState() == RADIO_STATE_UNAVAILABLE) {
        return SIM_NOT_READY;
    }

    err = at_send_command_singleline("AT+CPIN?", "+CPIN:", &atResponse);

    if (err != AT_NOERROR) {
        if (at_get_error_type(err) == AT_ERROR) {
            ret = SIM_NOT_READY;
            goto done;
        }

        switch (at_get_cme_error(err)) {
        case CME_SIM_NOT_INSERTED:
            ret = SIM_ABSENT;
            break;
        case CME_SIM_PIN_REQUIRED:
            ret = SIM_PIN;
            break;
        case CME_SIM_PUK_REQUIRED:
            ret = SIM_PUK;
            break;
        case CME_SIM_PIN2_REQUIRED:
            ret = SIM_PIN2;
            break;
        case CME_SIM_PUK2_REQUIRED:
            ret = SIM_PUK2;
            break;
        case CME_NETWORK_PERSONALIZATION_PIN_REQUIRED:
            ret = SIM_NETWORK_PERSO;
            break;
        case CME_NETWORK_PERSONALIZATION_PUK_REQUIRED:
            ret = SIM_NETWORK_PERSO_PUK;
            break;
        case CME_NETWORK_SUBSET_PERSONALIZATION_PIN_REQUIRED:
            ret = SIM_NETWORK_SUBSET_PERSO;
            break;
        case CME_NETWORK_SUBSET_PERSONALIZATION_PUK_REQUIRED:
            ret = SIM_NETWORK_SUBSET_PERSO_PUK;
            break;
        case CME_SERVICE_PROVIDER_PERSONALIZATION_PIN_REQUIRED:
            ret = SIM_SERVICE_PROVIDER_PERSO;
            break;
        case CME_SERVICE_PROVIDER_PERSONALIZATION_PUK_REQUIRED:
            ret = SIM_SERVICE_PROVIDER_PERSO_PUK;
            break;
        case CME_PH_SIMLOCK_PIN_REQUIRED: /* PUK not in use by modem */
            ret = SIM_SIM_PERSO;
            break;
        case CME_CORPORATE_PERSONALIZATION_PIN_REQUIRED:
            ret = SIM_CORPORATE_PERSO;
            break;
        case CME_CORPORATE_PERSONALIZATION_PUK_REQUIRED:
            ret = SIM_CORPORATE_PERSO_PUK;
            break;
        default:
            ret = SIM_NOT_READY;
            break;
        }
        return ret;
    }

    /* CPIN? has succeeded, now look at the result. */

    cpinLine = atResponse->p_intermediates->line;
    err = at_tok_start(&cpinLine);

    if (err < 0) {
        ret = SIM_NOT_READY;
        goto done;
    }

    err = at_tok_nextstr(&cpinLine, &cpinResult);

    if (err < 0) {
        ret = SIM_NOT_READY;
        goto done;
    }

    if (0 == strcmp(cpinResult, "READY")) {
		ret = SIM_READY;
		if (getSimLockStatus(&status,&times) < 0)
			goto done;
		if (status == 1 || status == 3) {
			ret = SIM_NETWORK_PERSO;
		}
    } else if (0 == strcmp(cpinResult, "SIM PIN")) {
        ret = SIM_PIN;
    } else if (0 == strcmp(cpinResult, "SIM PUK")) {
        ret = SIM_PUK;
    } else if (0 == strcmp(cpinResult, "SIM PIN2")) {
        ret = SIM_PIN2;
    } else if (0 == strcmp(cpinResult, "SIM PUK2")) {
        ret = SIM_PUK2;
    } else if (0 == strcmp(cpinResult, "PH-NET PIN")) {
        ret = SIM_NETWORK_PERSO;
    } else if (0 == strcmp(cpinResult, "PH-NETSUB PIN")) {
        ret = SIM_NETWORK_SUBSET_PERSO;
    } else if (0 == strcmp(cpinResult, "PH-SP PIN")) {
        ret = SIM_SERVICE_PROVIDER_PERSO;
    } else if (0 == strcmp(cpinResult, "PH-CORP PIN")) {
        ret = SIM_CORPORATE_PERSO;
    } else if (0 == strcmp(cpinResult, "PH-SIMLOCK PIN")) {
        ret = SIM_SIM_PERSO;
    } else if (0 == strcmp(cpinResult, "PH-ESL PIN")) {
        ret = SIM_STERICSSON_LOCK;
    } else if (0 == strcmp(cpinResult, "BLOCKED")) {
        int numRetries = 3;
        if (numRetries == -1 || numRetries == 0)
            ret = SIM_PERM_BLOCKED;
        else
            ret = SIM_PUK2_PERM_BLOCKED;
    } else if (0 == strcmp(cpinResult, "PH-SIM PIN")) {
        /*
         * Should not happen since lock must first be set from the phone.
         * Setting this lock is not supported by Android.
         */
        ret = SIM_BLOCKED;
    } else {
        /* Unknown locks should not exist. Defaulting to "sim absent" */
        ret = SIM_ABSENT;
    }
done:
    at_response_free(atResponse);
    return ret;
}


/**
 * SIM ready means any commands that access the SIM will work, including:
 *  AT+CPIN, AT+CSMS, AT+CNMI, AT+CRSM
 *  (all SMS-related commands).
 */
static void pollSIMState(void *param)
{
    if (((int) param) != 1 &&
        getRadioState() != RADIO_STATE_SIM_NOT_READY &&
        getRadioState() != RADIO_STATE_SIM_LOCKED_OR_ABSENT)
        /* No longer valid to poll. */
        return;

    switch (getSIMStatus()) {
    case SIM_NOT_READY:
        ALOGI("SIM_NOT_READY, poll for sim state.");
        enqueueRILEvent(pollSIMState, NULL, &TIMEVAL_SIMPOLL);
        return;

    case SIM_PIN2:
    case SIM_PUK2:
    case SIM_PUK2_PERM_BLOCKED:
    case SIM_READY:
        setRadioState(RADIO_STATE_SIM_READY);
        return;
    case SIM_ABSENT:
    case SIM_PIN:
    case SIM_PUK:
    case SIM_NETWORK_PERSO:
    case SIM_NETWORK_SUBSET_PERSO:
    case SIM_SERVICE_PROVIDER_PERSO:
    case SIM_CORPORATE_PERSO:
    case SIM_SIM_PERSO:
    case SIM_STERICSSON_LOCK:
    case SIM_BLOCKED:
    case SIM_PERM_BLOCKED:
    case SIM_NETWORK_PERSO_PUK:
    case SIM_NETWORK_SUBSET_PERSO_PUK:
    case SIM_SERVICE_PROVIDER_PERSO_PUK:
    case SIM_CORPORATE_PERSO_PUK:
    /* pass through, do not break */
    default:
        setRadioState(RADIO_STATE_SIM_LOCKED_OR_ABSENT);
        return;
    }
}

static void sendCallStateChanged(void *param)
{
    RIL_onUnsolicitedResponse (
            RIL_UNSOL_RESPONSE_CALL_STATE_CHANGED,
            NULL, 0);
}


static int clccStateToRILState(int state, RIL_CallState *p_state)
{
    switch(state) {
        case 0: *p_state = RIL_CALL_ACTIVE;   return 0;
        case 1: *p_state = RIL_CALL_HOLDING;  return 0;
        case 2: *p_state = RIL_CALL_DIALING;  return 0;
        case 3: *p_state = RIL_CALL_ALERTING; return 0;
        case 4: *p_state = RIL_CALL_INCOMING; return 0;
        case 5: *p_state = RIL_CALL_WAITING;  return 0;
        default:
			ALOGE("Unsupported call state: %d\n",state);
            return -1;
    }
}

/**
 * Note: directly modified line and has *p_call point directly into
 * modified line
 */
static int callFromCLCCLine(char *line, RIL_Call *p_call)
{
    //+CLCC: 1,0,2,0,0,\"+18005551212\",145
    //     index,isMT,state,mode,isMpty(,number,TOA)?

    int err;
    int state;
    int mode;

    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &(p_call->index));
    if (err < 0) goto error;

    err = at_tok_nextbool(&line, &(p_call->isMT));
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &state);
    if (err < 0) goto error;

    err = clccStateToRILState(state, &(p_call->state));
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &mode);
    if (err < 0) goto error;

    p_call->isVoice = (mode == 0);

    err = at_tok_nextbool(&line, &(p_call->isMpty));
    if (err < 0) goto error;

    if (at_tok_hasmore(&line)) {
        err = at_tok_nextstr(&line, &(p_call->number));

        /* tolerate null here */
        if (err < 0) return 0;

        // Some lame implementations return strings
        // like "NOT AVAILABLE" in the CLCC line
        if (p_call->number != NULL
                && 0 == strspn(p_call->number, "+0123456789")
           ) {
            p_call->number = NULL;
        }

        err = at_tok_nextint(&line, &p_call->toa);
        if (err < 0) goto error;
    }

    return 0;

error:
	ALOGE("invalid CLCC line\n");
    return -1;
}

static const struct timespec TIMEVAL_CALLSTATEPOLL = {0,500000};

static void requestGetCurrentCalls(void *data, size_t datalen, RIL_Token t)
{
    int err,fd;
    ATResponse *atResponse;
    ATLine *p_cur;
    int countCalls;
    RIL_Call *p_calls;
    RIL_Call **pp_calls;
    int i;
    char status[1];
    int needRepoll = 0;
    int countValidCalls=0;


    if(getRadioState() != RADIO_STATE_SIM_READY){
        /* Might be waiting for SIM PIN */
        RIL_onRequestComplete(t, RIL_E_RADIO_NOT_AVAILABLE, NULL, 0);
    }

    err = at_send_command_multiline ("AT+CLCC", "+CLCC:", &atResponse);
    if (err != AT_NOERROR) {
        RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
        at_response_free(atResponse);
        return;
    }

    /* count the calls */
    for (countCalls = 0, p_cur = atResponse->p_intermediates
            ; p_cur != NULL
            ; p_cur = p_cur->p_next
        ) {
        countCalls++;
    }

    /* yes, there's an array of pointers and then an array of structures */
    pp_calls = (RIL_Call **)alloca(countCalls * sizeof(RIL_Call *));
    p_calls = (RIL_Call *)alloca(countCalls * sizeof(RIL_Call));
    memset (p_calls, 0, countCalls * sizeof(RIL_Call));

    /* init the pointer array */
    for(i = 0; i < countCalls ; i++) {
        pp_calls[i] = &(p_calls[i]);
    }

    for (countValidCalls = 0, p_cur = atResponse->p_intermediates
            ; p_cur != NULL
            ; p_cur = p_cur->p_next
        ) {
        err = callFromCLCCLine(p_cur->line, p_calls + countValidCalls);

        if (err != 0) {
            continue;
        }

        if (p_calls[countValidCalls].state != RIL_CALL_ACTIVE
                && p_calls[countValidCalls].state != RIL_CALL_HOLDING
           ) {
            needRepoll = 1;
        }
        if(p_calls[countValidCalls].isVoice) // only count voice calls
            countValidCalls++;
    }

	ALOGI("Calls=%d,Valid=%d\n",countCalls,countValidCalls);

    RIL_onRequestComplete(t, RIL_E_SUCCESS, pp_calls,
            countValidCalls * sizeof (RIL_Call *));

	/* Start/stop audio tunnel if a valid voice call is detected -
	   This will restart the audiotunnel when we preventively shut it 
	   down after user hangs call (but should be restarted if in conference
	   Stopping should be handled by the connection end notification, but,
	   just in case, we also handle it here*/
	if (countValidCalls) {
		startAudioTunnel(NULL);
	} else {
		stopAudioTunnel(NULL);
	}
			
    at_response_free(atResponse);

    if (needRepoll) {
        enqueueRILEvent(sendCallStateChanged, NULL, &TIMEVAL_CALLSTATEPOLL);
    }
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    at_response_free(atResponse);
}

static void requestScreenState(void *data, size_t datalen, RIL_Token t)
{
    int err, screenState;

    assert (datalen >= sizeof(int *));
    screenState = ((int*)data)[0];

    if (screenState == 1) {
        /* Screen is on - be sure to enable all unsolicited notifications again */

        /* Enable proactive network registration notifications */
        err = at_send_command("AT+CREG=2");
        if (err != AT_NOERROR) goto error;

        /* Enable proactive network registration notifications */
        err = at_send_command("AT+CGREG=2");
        if (err != AT_NOERROR) goto error;

        /* Enable unsolicited reports */
        err = at_send_command("AT^CURC=1");
        if (err != AT_NOERROR) goto error;

        /* Enable GPRS reporting */
        err = at_send_command("AT+CGEREP=1,0");
        if (err != AT_NOERROR) goto error;



    } else if (screenState == 0) {

        /* Screen is off - disable all unsolicited notifications */
        err = at_send_command("AT+CREG=0");
        if (err != AT_NOERROR) goto error;
        err = at_send_command("AT+CGREG=0");
        if (err != AT_NOERROR) goto error;
        err = at_send_command("AT^CURC=0");
        if (err != AT_NOERROR) goto error;
        err = at_send_command("AT+CGEREP=0,0");
        if (err != AT_NOERROR) goto error;


    } else {
        /* Not a defined value - error */
        goto error;
    }

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
    return;

error:
	ALOGE("ERROR: requestScreenState failed");
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
}


/* wait for a propertyvalue change */
static int wait_for_property(const char *name, const char *desired_value, int maxwait, int allowempty)
{
    char value[PROPERTY_VALUE_MAX] = {'\0'};
    int maxnaps = maxwait;

    if (maxnaps < 1) {
        maxnaps = 1;
    }

    while (maxnaps-- > 0) {
        if (property_get(name, value, NULL)) {
			if (allowempty || (!allowempty && value[0]!=0)) {
				if (desired_value == NULL ||
						strcmp(value, desired_value) == 0) {
					return 0;
				}
			}
        }
		sleep(1);
    }
    return -1; /* failure */
}

/* Wait for pending output to be written on FD.  */
static int tcdrain (int fd)
{
	/* The TIOCSETP control waits for pending output to be written before
	affecting its changes, so we use that without changing anything.  */
	struct sgttyb b;
	if (ioctl (fd, TIOCGETP, (void *) &b) < 0 ||
		ioctl (fd, TIOCSETP, (void *) &b) < 0)
			return -1;
	return 0;
}

#ifdef ORG_DIAL
/* Write an string to the modem */
static int dial_at_modem(const char* cmd, int skipanswerwait)
{
	int i, n, ret;
	int fd = -1;
	char buf[64+1];
	
	ALOGD("dial_at_modem: opening modem %s", ppp_iface_dev);
    do {
        fd = open( ppp_iface_dev, O_RDWR | O_NOCTTY | O_NDELAY); /* nonblocking */
    } while (fd < 0 && errno == EINTR);

    if (fd < 0) {
        ALOGE("could not open modem %s: %s", ppp_iface_dev, strerror(errno) );
        return -1;
    }

    // disable echo on serial lines and set a 10 second timeout
    if ( isatty( fd ) ) {
        struct termios  ios;
        tcgetattr( fd, &ios );
        ios.c_lflag = 0;  /* disable ECHO, ICANON, etc... */
        ios.c_oflag &= (~ONLCR); /* Stop \n -> \r\n translation on output */
        ios.c_iflag &= (~(ICRNL | INLCR)); /* Stop \r -> \n & \n -> \r translation on input */
        ios.c_iflag |= (IGNCR | IXOFF);  /* Ignore \r & XON/XOFF on input */
		ios.c_cc[VTIME] = 10; /* Timeout in 1/10 of a second */
		ios.c_cc[VMIN] = 0; /* Minimum number of chars returned: 0*/ 
        tcsetattr( fd, TCSANOW, &ios );
    }
	
	// Discard all pending data */
	tcflush(fd, TCIOFLUSH);
	
	// If hangup sequence, wait a bit...
	if (!strcmp(cmd,"+++")) {
		sleep(1);
	}
	
	// Write command to modem
	ALOGD("dial_at_modem: writing command: %s", cmd);
	i = strlen(cmd);
	n = 0;
	do {
		ret = write(fd, cmd + n, i - n);
		if (ret < 0 && errno == EINTR)
			continue;
		if (ret < 0) {
			ALOGE("Error writing to modem: %s", strerror(errno) );
			close(fd);
			return -1;
		}
		n += ret;
	} while (n < i);
	
	// Force all the data to be transmitted
	tcdrain(fd);                  

	// If hangup sequence, wait a bit... and we are done. Modem won't answer it
	if (!strcmp(cmd,"+++")) {
		sleep(1);
		
		// Leave line ready */
		write(fd, "\r\n", 2);
		tcdrain(fd);          
        
		close(fd);
		return 0;
	}

	// If we must skip waiting for an answer
	if (skipanswerwait) {
		close(fd);
		return 0;
	}
	
	// Read answer with timeout
	ALOGD("dial_at_modem: waiting for response");
	do {
	
		/* Receive a line with timeout */
		int waitc = 5;
		n = 0;
		do {
			ret = read(fd, &buf[n], 1);
			if (ret < 0 && errno == EINTR)
				continue;
		
			if (ret < 0) {
				ALOGE("dial_at_modem: Error reading from serial port: %d",errno);
				close(fd);
				return -1;
			}
			
			if (ret == 0) {
				waitc --;
				if (waitc <= 0) {
					ALOGE("dial_at_modem: No answer from modem");
					close(fd);
					return -1;
				}
				
				// not timed out yet, sleep a second
				sleep(1);  
			} else {
				/* Something received, restart timeout */
				waitc = 5;
			}
			
			// Count of read chars
			n += ret;
		} while (n < 64 && buf[n-1] != '\n');
		
		/*buf[n] = 0; ALOGD("dial_at_modem: read %d bytes ['%s']",n,&buf[0]);*/
		
		// Remove the trailing spaces
		while (n > 0 && buf[n-1] <= ' '  && buf[n-1] != 0)
			n--;
		buf[n] = 0;
	  
		// Remove the leading spaces
		n = 0;
		while (buf[n]<= ' '  && buf[n] != 0)
			n++;

	} while (buf[0] == 0); // Ignore blank lines
		
	close(fd);
	
	ALOGD("dial_at_modem: got answer : '%s'",&buf[n]);
	return (!strcmp(&buf[n],"OK") || !strcmp(&buf[n],"CONNECT")) ? 0 : -1;
} 
#endif

static int killConn(const char* cididx)
{
	/* Leave NDIS mode */
	if (!pppSupported())
		at_send_command("AT^NDISDUP=%s,0",cididx);
	
	/* Leave Data context mode */
    at_send_command("AT+CGACT=0,%s", cididx);

	/* Hang up */
    at_send_command("ATH");

	/* Kill pppd daemon (just in case)*/
	system("killall pppd");
	
#ifdef ORG_DIAL
	/* Hang up modem, if needed */
	dial_at_modem("+++",1);
	dial_at_modem("ATH\r\n",1);
#endif
	
	/* Bring down all interfaces */
	if (ifc_init() == 0) {
		ifc_down(RNDIS_IFACE);
		ifc_down(PPP_IFACE);
		ifc_close();
    } 
	
    return 0;
}

#define LOG_FILE_DIR    "/dev/log/"

struct lc_entry {
    union {
        unsigned char buf[LOGGER_ENTRY_MAX_LEN + 1] __attribute__((aligned(4)));
        struct logger_entry entry __attribute__((aligned(4)));
    };
};

/* Get the connection data used by pppd from the android logcat */
int get_pppd_info(struct timeval* from, char* local_ip, char* dns1, char* dns2, char* gw)
{
	struct lc_entry* lce;

	const char mainlog[] = LOG_FILE_DIR "main";
	const char pppd_str[] = "pppd";
	const char local_ip_str[] = "local  IP address ";
	const char remote_ip_str[] = "remote IP address ";
	const char primary_dns_str[] = "primary   DNS address ";
	const char secondary_dns_str[] = "secondary DNS address ";

	int fd;
    int result;
	fd_set readset;
	int search_tmout = 500; //
	
	local_ip[0] = 0;
	dns1[0] = 0;
	dns2[0] = 0;
	gw[0] = 0;
	
	lce = malloc(sizeof(struct lc_entry));
	if (!lce) {
		ALOGE("Failed to allocate logcat entry buffer");
		return -1;
	}
	
	fd = open(mainlog, O_RDONLY);
	if (fd < 0) {
		ALOGE("Unable to open log device '%s': %s\n", mainlog, strerror(errno));
		free(lce);
		return -1;
	}

	/*ALOGD("filter: sec:%d, usec:%d", from->tv_sec, from->tv_usec);*/
	
	while (1) {
	
        do {
            struct timeval timeout = { 0, 10000 /* 10ms */ }; 
            FD_ZERO(&readset);
            FD_SET(fd, &readset);
            result = select(fd + 1, &readset, NULL, NULL, &timeout);
        } while (result < 0 && errno == EINTR);
		
		// Wait until we find something...
		if (result == 0) {
			if (search_tmout > 0) {
				search_tmout --;
				continue;
			} else
				break;
		}
			
		if (result > 0 && FD_ISSET(fd, &readset)) {
		
			/* NOTE: driver guarantees we read exactly one full entry */
			int ret = read(fd, lce, LOGGER_ENTRY_MAX_LEN);
			if (ret < 0) {
				if (errno == EINTR) {
					continue;
				}
				if (errno == EAGAIN) {
					continue;
				}
				ALOGE("logcat read");
				continue;
			}
			else if (!ret) {
				ALOGE("read: Unexpected EOF!\n");
				continue;
			}
			else if (lce->entry.len != ret - sizeof(struct logger_entry)) {
				ALOGE("read: unexpected length. Expected %d, got %d\n",
						lce->entry.len, ret - sizeof(struct logger_entry));
				continue;
			}
			
			lce->entry.msg[lce->entry.len] = '\0';
			
			/*
			 * format: <priority:1><tag:N>\0<message:N>\0
			 *
			 * tag str
			 *   starts at buf->msg+1
			 * msg
			 *   starts at buf->msg+1+len(tag)+1
			 *
			 * The message may have been truncated by the kernel log driver.
			 * When that happens, we must null-terminate the message ourselves.
			 */
			 
			/*ALOGD("entry: tag:%d[%c], sec:%d, usec:%d, n:%s, m:%s", lce->entry.msg[0], lce->entry.msg[0], lce->entry.sec, lce->entry.nsec/1000, &lce->entry.msg[1],(&lce->entry.msg[1])+strlen(&lce->entry.msg[1])+1); */
			
			// We are interested in pppd entries with I priority newer than specified
			if ((lce->entry.msg[0] == 'i' || lce->entry.msg[0] == 'I' || lce->entry.msg[0] == 4) &&
				(lce->entry.sec > from->tv_sec || 
					(lce->entry.sec == from->tv_sec && (lce->entry.nsec/1000) >= from->tv_usec)) &&
				!strcmp(&lce->entry.msg[1],pppd_str)) {
				char * pos = NULL;
				
				// Dealing with PPPD entries in logcat - Get a pointer to the message
				char * msg = (&lce->entry.msg[1]) + strlen(&lce->entry.msg[1]) + 1;
								
				// Strip leading spaces
				while (*msg != 0 && *msg <= ' ') msg++;
				
				// And also strip trailing spaces...
				pos = msg + strlen(msg);
				while (pos != msg && pos[-1] <= ' ') pos--;
				*pos = 0;
				
				// An entry we are interested in ?
				if ((pos = strstr(msg,local_ip_str)) != NULL) {
					strcpy(local_ip,msg + sizeof(local_ip_str) - 1);
					strcpy(gw,msg + sizeof(local_ip_str) - 1);
				} else
				if ((pos = strstr(msg,remote_ip_str)) != NULL) {
					// We found the remote_ip address ... Give no more than 3 seconds for the next entry to appear
					// (those entries are not mandatory)
					search_tmout = 300;
				} else
				if ((pos = strstr(msg,primary_dns_str)) != NULL) {
					strcpy(dns1,msg + sizeof(primary_dns_str) - 1);
				} else
				if ((pos = strstr(msg,secondary_dns_str)) != NULL) {
					strcpy(dns2,msg + sizeof(secondary_dns_str) - 1);
				}
			}
			
			// If we have all required data, break now!
			if (local_ip[0] && gw[0] && dns1[0] && dns2[0]) {
				break;
			}
		}
	}
	
	close(fd);
	free(lce);
	
	// Return if we succeeded or not
	return local_ip[0] ? 0 : -1;
}


/* Setup connection using PPP */
static int setupPPP(RIL_Token t,const char* ctxid,const char* user,const char* pass)
{
	int err;
	char* cmd = NULL;
	const char* fmt;
	in_addr_t addr;
	in_addr_t mask;
	unsigned flags = 0;
	int ctr = 10;
	struct timeval from_tm;

    RIL_Data_Call_Response_v6 responses;
	char ppp_ifname[PROPERTY_VALUE_MAX] = {'\0'};
    char ppp_local_ip[PROPERTY_VALUE_MAX] = {'\0'};
    char ppp_dns1[PROPERTY_VALUE_MAX] = {'\0'};
    char ppp_dns2[PROPERTY_VALUE_MAX] = {'\0'};
	char ppp_dnses[(PROPERTY_VALUE_MAX * 2) + 3] = {'\0'};
    char ppp_gw[PROPERTY_VALUE_MAX] = {'\0'};
	char pbuf[32] = {'\0'};


	ALOGD("Trying to setup PPP connnection...");
	
#ifdef ORG_DIAL
	/* PDP context activation */
	err = at_send_command("AT+CGACT=%s",ctxid);
	
    /* Enter data state using context #1 as PPP */
	asprintf(&cmd,"AT+CGDATA=\"PPP\",%s\r\n",ctxid);
    err = dial_at_modem(cmd,0);
	free(cmd);

	/* If failure, fall back to Start data on PDP context 1 */	
	if (err) {
		ALOGD("Failed to activate data context - Resorting to dial");
		err = at_send_command("ATD*99***%s#",ctxid);
		if (err != AT_NOERROR) {
			return -1;
		}
	}
#else
	/* Dial data */
	err = at_send_command("ATD*99***%s#",ctxid);
	if (err != AT_NOERROR) {
		/* If failed, retry just with data context activation */
		err = at_send_command("AT+CGDATA=\"PPP\",%s",ctxid);
		if (err != AT_NOERROR) {
			return -1;
		}
	}
	
	/* Wait for the modem to finish */
	sleep(2);
#endif
	
	/* original options
	"nodetach debug noauth defaultroute usepeerdns "
	"connect-delay 1000 "
	"user NotUsed@nobody.com password NotUsed "
	"crtscts lcp-echo-failure 0 lcp-echo-interval 0 ipcp-max-configure 30 "
	"ipcp-max-failure 30 ipcp-max-terminate 10 novj linkname ril";
	*/
	
	/* start the gprs pppd - pppd must be suid root */
	if (user == NULL || user[0] == 0 ||
		pass == NULL || pass[0] == 0 ) {
		
		fmt = "/system/bin/pppd %s "
		"115200 "
		"crtscts modem "
		"linkname ril%s "
		"noauth "
		"user %s password %s "		
		"defaultroute usepeerdns noipdefault "
		"novj novjccomp nobsdcomp "
		"ipcp-accept-remote ipcp-accept-local "
		"dump debug "
		"lcp-echo-failure 0 lcp-echo-interval 0 ipcp-max-configure 30 "
		"ipcp-max-failure 30 ipcp-max-terminate 10";
		
	} else {
	
		fmt = "/system/bin/pppd %s "
		"115200 "
		"crtscts modem "
		"linkname ril%s "
		"user %s password %s "
		"defaultroute usepeerdns noipdefault "
		"novj novjccomp nobsdcomp "
		"ipcp-accept-remote ipcp-accept-local "
		"dump debug "
		"lcp-echo-failure 0 lcp-echo-interval 0 ipcp-max-configure 30 "
		"ipcp-max-failure 30 ipcp-max-terminate 10";
	}
	
	asprintf(&cmd,fmt,ppp_iface_dev,ctxid, ((user && user[0])? user: "guest"), ((pass && pass[0])? pass: "guest") );
	
	ALOGD("Starting pppd w/command line: '%s'",cmd);
	
	// Get current time
	gettimeofday(&from_tm,NULL);
	
	// Start PPPD
    system(cmd);
	free(cmd);
	
#if 1
	/* Try not to depend on ip-up/down scripts ... */

	// Wait until network interface is up and running 
	if (ifc_init() < 0) {
		ALOGE("Failed initialization of net ifc");
		return -1;
	}
	
	ALOGD("Waiting until net ifc %s is up", PPP_IFACE);
	ctr = 20;
	do {
		sleep(1); // wait one second...
		flags = 0;
		ifc_get_info(PPP_IFACE, &addr, &mask, &flags);
	} while (--ctr > 0 && !(flags & IFF_UP));
	ifc_close();
	
	if (ctr <= 0) {
		ALOGE("Net ifc %s was never upped!", PPP_IFACE);
		return -1;
	}
	
	/* Get PPP information by reading and parsing the PPPD log */
	if (get_pppd_info(&from_tm, ppp_local_ip, ppp_dns1, ppp_dns2, ppp_gw) < 0) {
		ALOGE("Unable to get dns/gw");
		return -1;
	}
	
	strcpy(ppp_ifname,PPP_IFACE);
	
#else
	// Wait until ppp ip-up script has completely executed...
	sprintf(pbuf,"net.ril%s.if",ctxid);
    if (wait_for_property(pbuf, NULL, 20, 0) < 0) {
		ALOGE("Timeout waiting %s - giving up!\n",pbuf);
        return -1;
    }
	property_get(pbuf, ppp_ifname, NULL);
	
	// Get local IP
	sprintf(pbuf,"net.ril%s.local-ip",ctxid);
    property_get(pbuf, ppp_local_ip, NULL);
	
	sprintf(pbuf,"net.ril%s.dns1",ctxid);	
    property_get(pbuf, ppp_dns1, NULL);

	sprintf(pbuf,"net.ril%s.dns2",ctxid);	
    property_get(pbuf, ppp_dns2, NULL);

	sprintf(pbuf,"net.ril%s.gw",ctxid);	
    property_get(pbuf, ppp_gw, NULL);
	
#endif

    sprintf(ppp_dnses, "%s %s", ppp_dns1, ppp_dns2);

	ALOGI("Got ifname: %s, local-ip: %s, dns1: %s, dns2: %s, gw: %s\n",
		ppp_ifname, ppp_local_ip, ppp_dns1, ppp_dns2, ppp_gw);

    responses.status = 0;
    responses.suggestedRetryTime = -1;
    responses.cid = (ctxid[0] - '0');
    responses.active = 2;
    responses.type = (char*)"PPP";
    responses.ifname 	= ppp_ifname;
    responses.addresses = ppp_local_ip;
    responses.dnses 	= ppp_dnses;
    responses.gateways 	= ppp_gw;
	
	using_rndis = 0;
	
    RIL_onRequestComplete(t, RIL_E_SUCCESS, &responses, sizeof(RIL_Data_Call_Response_v6));
    return 0;
}

/* Converts a hex encoded addr to an std addr */
static void str2addr(char* addr,const char* str)
{
	unsigned int val = (unsigned int) strtoul(str, NULL, 16);
	sprintf(addr,"%u.%u.%u.%u",
			(val & 0x000000ffU),
			(val & 0x0000ff00U) >> 8U,
			(val & 0x00ff0000U) >> 16U,
			(val & 0xff000000U) >> 24U);
}

/* Setup connection using NDIS */
static int setupNDIS(RIL_Token t,const char* ctxid)
{
    ATResponse *atResponse = NULL;
    RIL_Data_Call_Response_v6 responses;
    char rndis_dnses[64 + 3] = {'\0'};
    char rndis_local_ip[32] = {'\0'};
    char rndis_dns1[32] = {'\0'};
    char rndis_dns2[32] = {'\0'};
    char rndis_gw[32] = {'\0'};
	
	in_addr_t in_addr;
    in_addr_t in_gateway;
 	
    int err;
    char *line;
	char *ip = NULL;
	char *netmask = NULL;
	char *gateway = NULL;
	char *unk = NULL;
	char *dns1 = NULL;
	char *dns2 = NULL; 	

	ALOGD("Trying to setup NDIS connnection...");
	
	/* Enter data state using context #1 as NDIS */
    err = at_send_command("AT^NDISDUP=%s,1",ctxid);
	if (err != AT_NOERROR) 
		return -1;

	/* Leave 10 seconds for startup */
	sleep(10);
		
    /* Check DHCP status */
    err = at_send_command_singleline("AT^DHCP?", "^DHCP:", &atResponse);
    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;
    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextstr(&line, &ip);
    if (err < 0) goto error;

    err = at_tok_nextstr(&line, &netmask);
    if (err < 0) goto error;

    err = at_tok_nextstr(&line, &gateway);
    if (err < 0) goto error;

    err = at_tok_nextstr(&line, &unk);
    if (err < 0) goto error;

    err = at_tok_nextstr(&line, &dns1);
    if (err < 0) goto error;

    err = at_tok_nextstr(&line, &dns2);
    if (err < 0) goto error;
	
	ALOGD("IP: %s, netmask: %s, gateway: %s, dns1:%s, dns2:%s",ip,netmask,gateway,dns1,dns2);
	
    str2addr(rndis_local_ip, ip);
	if (inet_pton(AF_INET, rndis_local_ip, &in_addr) <= 0) {
		ALOGE("%s() inet_pton() failed for %s!", __func__, rndis_local_ip);
		goto error;
	} 
	
	str2addr(rndis_dns1, dns1);
	str2addr(rndis_dns2, dns2);
	sprintf(rndis_dnses, "%s %s", rndis_dns1, rndis_dns2);
	
	str2addr(rndis_gw, gateway);
	if (inet_pton(AF_INET, rndis_gw, &in_gateway) <= 0) {
		ALOGE("%s() inet_pton() failed for %s!", __func__, rndis_gw);
		goto error;
	} 
	
    at_response_free(atResponse);
	
    responses.status = 0;
    responses.suggestedRetryTime = -1;
    responses.cid = 1;
    responses.active = 2;
    responses.type = (char*)"RNDIS";
    responses.ifname = (char*)RNDIS_IFACE;
    responses.addresses = rndis_local_ip;
    responses.dnses 	= rndis_dnses;
    responses.gateways 	= rndis_gw;
	
    /* Don't use android netutils. We use our own and get the routing correct.
     * Carl Nordbeck */
    if (ifc_configure(RNDIS_IFACE, in_addr, in_gateway))
        ALOGE("%s() Failed to configure the interface %s", __func__, RNDIS_IFACE); 	
	
	using_rndis = 1;
	
    RIL_onRequestComplete(t, RIL_E_SUCCESS, &responses, sizeof(RIL_Data_Call_Response_v6));
    return 0;

error:
	ALOGE("Failed to start NDIS mode");

    at_response_free(atResponse);
    return -1;
}

/* Query if modem supports PPP. Otherwise, RNDIS will be used */
static int pppSupported(void)
{
	ATResponse *atResponse = NULL;
 	
    int err;
	int supp_mode = 0; // PPP
    char *line;

	ALOGD("Identifying modem capabilities...");
	
    /* Check Modem capabilities */
    err = at_send_command_singleline("AT^DIALMODE?", "^DIALMODE:", &atResponse);
    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;
    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &supp_mode);
    if (err < 0) goto error;

	ALOGD("Modem mode: %d [0=ppp,1=ndis,2=ppp&ndis",supp_mode);
	
    at_response_free(atResponse);
	
	return supp_mode != 1; // ppp supported
	
error:
	ALOGD("Assuming PPP mode");
    at_response_free(atResponse);
    return 1;
}

static void requestSetupDefaultPDP(void *data, size_t datalen, RIL_Token t)
{
    const char *apn;
    const char *user = NULL;
    const char *pass = NULL;
    int err;
	const char* ctxid = "1";
	int use_ppp;

    apn = ((const char **)data)[2];
    user = ((char **)data)[3];
    if (user == NULL || strlen(user) < 2) {
        user = "dummy";
    }

    pass = ((char **)data)[4];
    if (pass == NULL || strlen(pass) < 2) {
        pass = "dummy";
    }

	/* Check if PPP is supported */
	use_ppp = pppSupported();
	
    D("requesting data connection to APN '%s'\n", apn);

    // Make sure there is no existing connection
    killConn(ctxid);


    /* packet-domain event reporting */
    err = at_send_command("AT+CGEREP=1,0");

    /* Define the PDP context #1*/
    err = at_send_command("AT+CGDCONT=%s,\"IP\",\"%s\"",ctxid, apn);

    /* Set required QoS params to default */
    err = at_send_command("AT+CGQREQ=%s",ctxid);

    /* Set minimum QoS params to default */
    err = at_send_command("AT+CGQMIN=%s",ctxid);

	/* Attach to GPRS network */
	err = at_send_command("AT+CGATT=1");
	
	if (use_ppp) {
		if (setupPPP(t,ctxid,user,pass) < 0) {
			goto error;
		}
	} else {
		if (setupNDIS(t,ctxid) < 0) {
			goto error;
		}
	}
	
	ALOGD("Data connection established!\n");
	return;
	
error:
	ALOGE("Unable to setup PDP\n");
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);

}

/* CHECK There are several error cases if PDP deactivation fails
 * 24.008: 8, 25, 36, 38, 39, 112
 */
static void requestDeactivateDefaultPDP(void *data, size_t datalen, RIL_Token t)
{
    char * cid;

    D("requestDeactivateDefaultPDP()");

    cid = ((char **)data)[0];
    if (killConn(cid) < 0)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
}

/**
 * RIL_REQUEST_LAST_PDP_FAIL_CAUSE
 *
 * Requests the failure cause code for the most recently failed PDP
 * context activate.
 *
 * See also: RIL_REQUEST_LAST_CALL_FAIL_CAUSE.
 *
 */

/* Last pdp fail cause */
static int s_lastPdpFailCause = PDP_FAIL_ERROR_UNSPECIFIED;
static void requestLastPDPFailCause(RIL_Token t)
{
    RIL_onRequestComplete(t, RIL_E_SUCCESS, &s_lastPdpFailCause, sizeof(int));
}

static void requestOrSendPDPContextList(RIL_Token *t)
{
    ATResponse *atResponse = NULL;
    ATLine *p_cur;
    RIL_Data_Call_Response_v6 *responses = NULL;

    int err;
    int fd;
    char *line, *out;
    int i,n = 0;

    err = at_send_command_multiline ("AT+CGACT?", "+CGACT:", &atResponse);
    if (err != AT_NOERROR) {
        if (t != NULL)
            RIL_onRequestComplete(*t, RIL_E_GENERIC_FAILURE, NULL, 0);
        else
            RIL_onUnsolicitedResponse(RIL_UNSOL_DATA_CALL_LIST_CHANGED, NULL, 0);

        at_response_free(atResponse);
        return;
    }

    for (p_cur = atResponse->p_intermediates; p_cur != NULL;
            p_cur = p_cur->p_next)
        n++;

    responses = (RIL_Data_Call_Response_v6*)
                    alloca(n * sizeof(RIL_Data_Call_Response_v6));

    for (i = 0; i < n; i++) {
        responses[i].status = -1;
        responses[i].suggestedRetryTime = -1;
        responses[i].cid = -1;
        responses[i].active = -1;
        responses[i].type = (char*)"";
        responses[i].ifname = using_rndis ? (char*)RNDIS_IFACE : (char*)PPP_IFACE;
        responses[i].addresses = (char*)"";
        responses[i].dnses = (char*)"";
        responses[i].gateways = (char*)"";
    }

    RIL_Data_Call_Response_v6 *response = responses;
    for (p_cur = atResponse->p_intermediates; p_cur != NULL;
            p_cur = p_cur->p_next) {

        char *line = p_cur->line;

        err = at_tok_start(&line);
        if (err < 0)
            goto error;

        err = at_tok_nextint(&line, &response->cid);
        if (err < 0)
            goto error;

        err = at_tok_nextint(&line, &response->active);
        if (err < 0)
            goto error;

        response++;
    }

    at_response_free(atResponse);

    err = at_send_command_multiline("AT+CGDCONT?", "+CGDCONT:", &atResponse);
    if (err != AT_NOERROR)
        goto error;

    for (p_cur = atResponse->p_intermediates; p_cur != NULL;
            p_cur = p_cur->p_next) {

        char *line = p_cur->line;
        int cid;

        err = at_tok_start(&line);
        if (err < 0)
            goto error;

        err = at_tok_nextint(&line, &cid);
        if (err < 0)
            goto error;

        for (i = 0; i < n; i++) {
            if (responses[i].cid == cid)
                break;
        }

        if (i >= n) {
            /* details for a context we didn't hear about in the last request */
            continue;
        }

        // Assume no error
        responses[i].status = 0;

        // type
        err = at_tok_nextstr(&line, &out);
        if (err < 0)
            goto error;

        responses[i].type = (char*)alloca(strlen(out) + 1);
        strcpy(responses[i].type, out);

        // APN ignored for v5
        err = at_tok_nextstr(&line, &out);
        if (err < 0)
            goto error;

        responses[i].ifname = using_rndis ? (char*)RNDIS_IFACE : (char*)PPP_IFACE;

        err = at_tok_nextstr(&line, &out);
        if (err < 0)
            goto error;

        responses[i].addresses = (char*)alloca(strlen(out) + 1);
        strcpy(responses[i].addresses, out);
    }


    at_response_free(atResponse);
    atResponse = NULL;

	/* Make sure interface is UP and running. If not, invalidate datacall to 
	   force Android to reconnect - This will happen when resuming from suspend, as pppd
	   has probably ended as a consequence of the USB bus being suspended and the terminal
	   disappearing. So, that is why we have this check here. Not only the modem has to
	   report an active connection, also linux has to export an active net interface !*/
	if (ifc_init() >= 0) {
		in_addr_t addr;
		in_addr_t mask;
		unsigned flags = 0;
		ifc_get_info( responses[i].ifname, &addr, &mask, &flags);
		if (!(flags & IFF_UP)) {
			for (i = 0; i < n; i++) {
				if (responses[i].active) {
					ALOGD("DataCall cid:%d forced to inactive, as associated interface [%s] is DOWN", responses[i].cid, responses[i].ifname );
					responses[i].active = 0;
				}
			}
		}
		ifc_close();
	}
	
    if (t != NULL)
        RIL_onRequestComplete(*t, RIL_E_SUCCESS, responses,
                n * sizeof(RIL_Data_Call_Response_v6));
    else
        RIL_onUnsolicitedResponse(RIL_UNSOL_DATA_CALL_LIST_CHANGED, responses,
                n * sizeof(RIL_Data_Call_Response_v6));

    return;

error:
    if (t != NULL)
        RIL_onRequestComplete(*t, RIL_E_GENERIC_FAILURE, NULL, 0);
    else
        RIL_onUnsolicitedResponse(RIL_UNSOL_DATA_CALL_LIST_CHANGED, NULL, 0);

    at_response_free(atResponse);
}

/**
 * RIL_REQUEST_PDP_CONTEXT_LIST
 *
 * Queries the status of PDP contexts, returning for each
 * its CID, whether or not it is active, and its PDP type,
 * APN, and PDP adddress.
 */
static void requestPDPContextList(RIL_Token t)
{
    requestOrSendPDPContextList(&t);
}

static void onDataCallListChanged(void *param)
{
    requestOrSendPDPContextList(NULL);
}

/*
 * Configure preferred message storage
 *  mem1 = SM, mem2 = SM
 */
static int setPreferredMessageStorage(void)
{
    ATResponse *atResponse = NULL;
    char *tok = NULL;
    int used1, total1;
    int err;
    int return_value;

    err = at_send_command_singleline("AT+CPMS=\"SM\",\"SM\"","+CPMS: ", &atResponse);
    if (err != AT_NOERROR) {
        ALOGE("%s() Unable to set preferred message storage", __func__);
        goto error;
    }

    /*
     * Depending on the host boot time the indication that message storage
     * on SIM is full (+CIEV: 10,1) may be sent before the RIL is started.
     * The RIL will explicitly check status of SIM messages storage using
     * +CPMS intermediate response and inform Android if storage is full.
     * +CPMS: <used1>,<total1>,<used2>,<total2>,<used3>,<total3>
     */
    tok = atResponse->p_intermediates->line;

    err = at_tok_start(&tok);
    if (err < 0)
        goto error;

    err = at_tok_nextint(&tok, &used1);
    if (err < 0)
        goto error;

    err = at_tok_nextint(&tok, &total1);
    if (err < 0)
        goto error;

    if (used1 >= total1)
        RIL_onUnsolicitedResponse(RIL_UNSOL_SIM_SMS_STORAGE_FULL,NULL, 0);

    return_value = 0;

    goto exit;

error:
    ALOGE("%s() Failed during AT+CPMS sending/handling!", __func__);
    return_value = 1;

exit:
    at_response_free(atResponse);
    return return_value;
}

static char s_outstanding_acknowledge = 0;

#define OUTSTANDING_SMS    0
#define OUTSTANDING_STATUS 1
#define OUTSTANDING_CB     2

#define MESSAGE_STORAGE_READY_TIMER 3

#define BSM_LENGTH 88

/* Check if ME is ready to set preferred message storage */
static void checkMessageStorageReady(void *p)
{
    int err;
    struct timespec trigger_time;
    (void) p;

    err = at_send_command_singleline("AT+CPMS?","+CPMS: ", NULL);
    if (err == AT_NOERROR) {
        if (setPreferredMessageStorage() == 0) {
            ALOGI("Message storage is ready");
            return;
        }
    }

    ALOGE("%s() Message storage is not ready"
            "A new attempt will be done in %d seconds",
            __func__, MESSAGE_STORAGE_READY_TIMER);

    trigger_time.tv_sec = MESSAGE_STORAGE_READY_TIMER;
    trigger_time.tv_nsec = 0;

    enqueueRILEvent(checkMessageStorageReady, NULL, &trigger_time);
}

struct held_pdu {
    char type;
    char *sms_pdu;
    struct held_pdu *next;
};

static pthread_mutex_t s_held_pdus_mutex = PTHREAD_MUTEX_INITIALIZER;
static struct held_pdu *s_held_pdus = NULL;

static struct held_pdu *dequeue_held_pdu(void)
{
    struct held_pdu *hpdu = NULL;

    if (s_held_pdus != NULL) {
        hpdu = s_held_pdus;
        s_held_pdus = hpdu->next;
        hpdu->next = NULL;
    }

    return hpdu;
}

static void enqueue_held_pdu(char type, const char *sms_pdu)
{
    struct held_pdu *hpdu = (struct held_pdu *) malloc(sizeof(*hpdu));
    if (hpdu == NULL) {
        ALOGE("%s() failed to allocate memory!", __func__);
        return;
    }

    memset(hpdu, 0, sizeof(*hpdu));
    hpdu->type = type;
    hpdu->sms_pdu = strdup(sms_pdu);
    if (NULL == hpdu->sms_pdu) {
        ALOGE("%s() failed to allocate memory!", __func__);
        return;
    }

    if (s_held_pdus == NULL)
       s_held_pdus = hpdu;
    else {
        struct held_pdu *p = s_held_pdus;
        while (p->next != NULL)
            p = p->next;

        p->next = hpdu;
    }
}

void isSimSmsStorageFull(void *p)
{
    ATResponse *atResponse = NULL;
    char *tok = NULL;
    char* storage_area = NULL;
    int used1, total1;
    int err;
    (void) p;

    err = at_send_command_singleline("AT+CPMS?", "+CPMS: ", &atResponse);
    if (err != AT_NOERROR)
        goto error;

    tok = atResponse->p_intermediates->line;

    err = at_tok_start(&tok);
    if (err < 0)
        goto error;

    err = at_tok_nextstr(&tok, &storage_area);
    if (err < 0)
        goto error;

    err = at_tok_nextint(&tok, &used1);
    if (err < 0)
        goto error;

    err = at_tok_nextint(&tok, &total1);
    if (err < 0)
        goto error;

    if (used1 >= total1)
        RIL_onUnsolicitedResponse(RIL_UNSOL_SIM_SMS_STORAGE_FULL, NULL, 0);

    goto exit;

error:
    ALOGE("%s() failed during AT+CPMS sending/handling!", __func__);
exit:
    at_response_free(atResponse);
    return;
}

/**
 * RIL_UNSOL_SIM_SMS_STORAGE_FULL
 *
 * SIM SMS storage area is full, cannot receive
 * more messages until memory freed
 */
static void onNewSmsIndication(void)
{
    enqueueRILEvent(isSimSmsStorageFull, NULL, NULL);
}

static void onNewSms(const char *sms_pdu)
{
    pthread_mutex_lock(&s_held_pdus_mutex);

    /* No RIL_UNSOL_RESPONSE_NEW_SMS or RIL_UNSOL_RESPONSE_NEW_SMS_STATUS_REPORT
     * messages should be sent until a RIL_REQUEST_SMS_ACKNOWLEDGE has been received for
     * previous new SMS.
     */
    if (s_outstanding_acknowledge) {
        ALOGI("Waiting for ack for previous sms, enqueueing PDU");
        enqueue_held_pdu(OUTSTANDING_SMS, sms_pdu);
    } else {
        s_outstanding_acknowledge = 1;
        RIL_onUnsolicitedResponse(RIL_UNSOL_RESPONSE_NEW_SMS,
                                  sms_pdu, strlen(sms_pdu));
    }

    pthread_mutex_unlock(&s_held_pdus_mutex);
}

static void onNewStatusReport(const char *sms_pdu)
{
    char *response = NULL;
    int err;

    /* Baseband will not prepend SMSC addr, but Android expects it. */
    err = asprintf(&response, "%s%s", "00", sms_pdu);
    if (err == -1) {
        D("%s() Error allocating memory!", __func__);
        return;
    }

    pthread_mutex_lock(&s_held_pdus_mutex);

    /* No RIL_UNSOL_RESPONSE_NEW_SMS or RIL_UNSOL_RESPONSE_NEW_SMS_STATUS_REPORT
     * messages should be sent until a RIL_REQUEST_SMS_ACKNOWLEDGE has been received for
     * previous new SMS.
     */
    if (s_outstanding_acknowledge) {
        ALOGE("%s() Waiting for previous ack, enqueueing PDU..", __func__);
        enqueue_held_pdu(OUTSTANDING_STATUS, response);
    } else {
        s_outstanding_acknowledge = 1;
        RIL_onUnsolicitedResponse(RIL_UNSOL_RESPONSE_NEW_SMS_STATUS_REPORT,
                                  response, strlen(response));
    }
    free(response);
    pthread_mutex_unlock(&s_held_pdus_mutex);
}

static void onNewBroadcastSms(const char *pdu)
{
    char *message = NULL;
    D("%s() Length : %d", __func__, strlen(pdu));

    if (strlen(pdu) != (2 * BSM_LENGTH)) {
        ALOGE("%s() Broadcast Message length error! Discarding!", __func__);
        goto error;
    }
    D("%s() PDU: %176s", __func__, pdu);

    message = (char*) alloca(BSM_LENGTH);
    if (!message) {
        ALOGE("%s() error allocating memory for message! Discarding!", __func__);
        goto error;
    }

    stringToBinary(pdu, 2*BSM_LENGTH, (unsigned char *)message);
    D("%s() Message: %88s", __func__, message);

    pthread_mutex_lock(&s_held_pdus_mutex);

    /* No RIL_UNSOL_RESPONSE_NEW_SMS or RIL_UNSOL_RESPONSE_NEW_SMS_STATUS_REPORT
     * or RIL_UNSOL_RESPONSE_NEW_CB
     * messages should be sent until a RIL_REQUEST_SMS_ACKNOWLEDGE has been received for
     * previous new SMS.
     */
    if (s_outstanding_acknowledge) {
        ALOGE("%s() Waiting for previous ack, enqueueing PDU..", __func__);
        enqueue_held_pdu(OUTSTANDING_CB, message);
    } else {
        s_outstanding_acknowledge = 1;
        RIL_onUnsolicitedResponse(RIL_UNSOL_RESPONSE_NEW_BROADCAST_SMS,
                              message, BSM_LENGTH);
    }

    pthread_mutex_unlock(&s_held_pdus_mutex);

error:
    return;
}

static void onNewSmsOnSIM(const char *s)
{
    char *line;
    char *mem;
    char *tok;
    int err = 0;
    int index = -1;

    tok = line = strdup(s);

    err = at_tok_start(&tok);
    if (err < 0)
        goto error;

    err = at_tok_nextstr(&tok, &mem);
    if (err < 0)
        goto error;

    if (strncmp(mem, "SM", 2) != 0)
        goto error;

    err = at_tok_nextint(&tok, &index);
    if (err < 0)
        goto error;

	/* Huawei modems use a 0-based message slot index, but the SIM record is 1-based. 
	   We will translate Huawei indices to Record indices, to make the Android RIL 
	   able to read SMS stored in SIM using SimIo with the same indices as the Sms 
	   delete command and NewSmsOnSim index */
	index += 1;
		
    RIL_onUnsolicitedResponse(RIL_UNSOL_RESPONSE_NEW_SMS_ON_SIM,
                              &index, sizeof(int *));

finally:
    free(line);
    return;

error:
    ALOGE("%s() Failed to parse +CMTI.", __func__);
    goto finally;
}


/**
 * RIL_REQUEST_SEND_SMS
 *
 * Sends an SMS message.
*/
static void requestSendSMS(void *data, size_t datalen, RIL_Token t)
{
    (void) datalen;
    int err, aterr = 0;
    const char *testSmsc;
    char smsc[30];
    const char *pdu;
    char *line;
    int tpLayerLength;
    char *cmd1, *cmd2;
    RIL_SMS_Response response;
    RIL_Errno ret = RIL_E_SUCCESS;
    ATResponse *atResponse = NULL;

    testSmsc = ((const char **) data)[0];
    pdu = ((const char **) data)[1];

    tpLayerLength = strlen(pdu) / 2;

    /* NULL for default SMSC. */
    if (testSmsc == NULL) {
        char* temp;
        int tosca;
        int plus = 0;
        int length = 0;
        int i,curChar = 0;

        err = at_send_command_singleline("AT+CSCA?", "+CSCA:", &atResponse);
        if (err != AT_NOERROR)
            goto error;

        line = atResponse->p_intermediates->line;

        err = at_tok_start(&line);
        if (err < 0) goto error;

        err = at_tok_nextstr(&line, &temp);
        if (err < 0) goto error;

        err = at_tok_nextint(&line, &tosca);
        if (err < 0) goto error;

        if(temp[0]=='+')
            plus = 1;

        length = strlen(temp) - plus;
        sprintf(smsc,"%.2x%.2x",(length + 1) / 2 + 1, tosca);

        for (i = 0; curChar < length - 1; i+=2 ) {
            smsc[5+i] = temp[plus+curChar++];
            smsc[4+i] = temp[plus+curChar++];
        }

        if ( length % 2) { // One extra number
            smsc[4+length] = temp[curChar];
            smsc[3+length]='F';
            smsc[5+length]='\0';
        } else {
            smsc[4+length] = '\0';
        }
        at_response_free(atResponse);
    }
    else
        strcpy(smsc,testSmsc);

    asprintf(&cmd1, "AT+CMGS=%d", tpLayerLength);
    asprintf(&cmd2, "%s%s", smsc, pdu);
    aterr = at_send_command_sms(cmd1, cmd2, "+CMGS:", &atResponse);
    free(cmd1);
    free(cmd2);

    if (aterr != AT_NOERROR)
        goto error;

    memset(&response, 0, sizeof(response));

    /* Set errorCode to -1 if unknown or not applicable
     * See 3GPP 27.005, 3.2.5 for GSM/UMTS
     */
    response.errorCode = -1;

    line = atResponse->p_intermediates->line;

    err = at_tok_start(&line);
    if (err < 0)
        goto error;

    err = at_tok_nextint(&line, &response.messageRef);
    if (err < 0)
        goto error;

    /* No support for ackPDU. Do we need it? */
    RIL_onRequestComplete(t, RIL_E_SUCCESS, &response, sizeof(response));

finally:
    at_response_free(atResponse);
    return;

error:
    switch (at_get_cms_error(aterr)) {
    case CMS_NO_NETWORK_SERVICE:
    case CMS_NETWORK_TIMEOUT:
        ret = RIL_E_SMS_SEND_FAIL_RETRY;
        break;
    default:
        ret = RIL_E_GENERIC_FAILURE;
        break;
    }
    RIL_onRequestComplete(t, ret, NULL, 0);
    goto finally;
}


/**
 * RIL_REQUEST_SEND_SMS_EXPECT_MORE
 *
 * Send an SMS message. Identical to RIL_REQUEST_SEND_SMS,
 * except that more messages are expected to be sent soon. If possible,
 * keep SMS relay protocol link open (eg TS 27.005 AT+CMMS command).
*/
static void requestSendSMSExpectMore(void *data, size_t datalen, RIL_Token t)
{
    /* Throw the command on the channel and ignore any errors, since we
       need to send the SMS anyway and subsequent SMSes will be sent anyway. */
    at_send_command("AT+CMMS=1");

    requestSendSMS(data, datalen, t);
}


/**
 * RIL_REQUEST_WRITE_SMS_TO_SIM
 *
 * Stores a SMS message to SIM memory.
*/
void requestWriteSmsToSim(void *data, size_t datalen, RIL_Token t)
{
    RIL_SMS_WriteArgs *args;
    char *cmd;
    char *pdu;
    char *line;
    int length;
    int index;
    int err;
    ATResponse *atResponse = NULL;

    (void) datalen;

    args = (RIL_SMS_WriteArgs *) data;

    length = strlen(args->pdu) / 2;

    asprintf(&cmd, "AT+CMGW=%d,%d", length, args->status);
    asprintf(&pdu, "%s%s", (args->smsc ? args->smsc : "00"), args->pdu);
    err = at_send_command_sms(cmd, pdu, "+CMGW:", &atResponse);
    free(cmd);
    free(pdu);

    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;

    err = at_tok_start(&line);
    if (err < 0)
        goto error;

    err = at_tok_nextint(&line, &index);
    if (err < 0)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, &index, sizeof(int *));

finally:
    at_response_free(atResponse);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    goto finally;
}

/**
 * RIL_REQUEST_DELETE_SMS_ON_SIM
 *
 * Deletes a SMS message from SIM memory.
 */
static void requestDeleteSmsOnSim(void *data, size_t datalen, RIL_Token t)
{
    int err;
	
	/* Huawei modems use a 0-based message slot index, but the SIM record is 1-based. 
	   We will translate Huawei indices to Record indices, to make the Android RIL 
	   able to read SMS stored in SIM using SimIo with the same indices as the Sms 
	   delete command and NewSmsOnSim index */
	int idx = ((int *) data)[0] - 1;
	if (idx < 0) {
		ALOGE("DeleteSmsOnSim: Invalid index! (%d)",idx);
		RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
		return;
	}
	
    err = at_send_command("AT+CMGD=%d", idx);
    if (err != AT_NOERROR)
        RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    else
        RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}

/**
 * RIL_REQUEST_GET_SMSC_ADDRESS
 */
static void requestGetSMSCAddress(RIL_Token t)
{
    ATResponse *atResponse = NULL;
    int err;
    char *line;
    char *response;

    err = at_send_command_singleline("AT+CSCA?", "+CSCA:", &atResponse);

    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;

    err = at_tok_start(&line);
    if (err < 0)
        goto error;

    err = at_tok_nextstr(&line, &response);
    if (err < 0)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, response, sizeof(char *));

finally:
    at_response_free(atResponse);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    goto finally;
}



/**
 * RIL_REQUEST_SET_SMSC_ADDRESS
 */
static void requestSetSMSCAddress(void *data, size_t datalen, RIL_Token t)
{
    (void) datalen;
    int err;
    const char *smsc = (const char *)data;

    err = at_send_command("AT+CSCA=\"%s\"", smsc);
    if (err != AT_NOERROR)
        RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    else
        RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}


/**
 * RIL_REQUEST_REPORT_SMS_MEMORY_STATUS
 */
static void requestSmsStorageFull(void *data, size_t datalen, RIL_Token t)
{
    int ack;

    ack = ((int *) data)[0];

    /* Android will call RIL_REQUEST_REPORT_SMS_MEMORY_STATUS in case of:
     * 0. memory is full
     * 1. memory was full and been cleaned up, inform modem memory is available now.
     */
    switch (ack) {
    case 0:
        /* Android will handle this, no need to inform modem. always return success. */
        ALOGI("SMS storage full");
        break;

    case 1:
        /* Since we are not using +CNMA command. It's fine to return without informing network */
        ALOGI("Failed to inform network for Message Cleanup. Need cmd : ESMSMEMAVAIL");
        break;

    default:
        ALOGE("%s() Invalid parameter", __func__);
        RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
        return;
    }

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}


/**
 * RIL_REQUEST_SMS_ACKNOWLEDGE
 *
 * Acknowledge successful or failed receipt of SMS previously indicated
 * via RIL_UNSOL_RESPONSE_NEW_SMS .
*/
static void requestSMSAcknowledge(void *data, size_t datalen, RIL_Token t)
{
    int ackSuccess;
    int err;
    struct held_pdu *hpdu;

    ackSuccess = ((int *)data)[0];

    if (ackSuccess == 1) {
        err = at_send_command("AT+CNMA=1");
    } else if (ackSuccess == 0)  {
        err = at_send_command("AT+CNMA=2");
    } else {
        ALOGE("unsupported arg to RIL_REQUEST_SMS_ACKNOWLEDGE\n");
        goto error;
    }

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);

    pthread_mutex_lock(&s_held_pdus_mutex);
    hpdu = dequeue_held_pdu();

    if (hpdu != NULL) {
        ALOGE("%s() Outstanding requests in queue, dequeueing and sending.",
         __func__);
        int unsolResponse = 0;

        if (hpdu->type == OUTSTANDING_SMS)
            unsolResponse = RIL_UNSOL_RESPONSE_NEW_SMS;
        else if (hpdu->type == OUTSTANDING_CB)
            unsolResponse = RIL_UNSOL_RESPONSE_NEW_BROADCAST_SMS;
        else
            unsolResponse = RIL_UNSOL_RESPONSE_NEW_SMS_STATUS_REPORT;

        RIL_onUnsolicitedResponse(unsolResponse, hpdu->sms_pdu,
                                  strlen(hpdu->sms_pdu));

        free(hpdu->sms_pdu);
        free(hpdu);
    } else
        s_outstanding_acknowledge = 0;

    pthread_mutex_unlock(&s_held_pdus_mutex);

    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);

}




#define BROADCAST_MAX_RANGES_SUPPORTED 10

/**
 * RIL_REQUEST_GSM_GET_BROADCAST_SMS_CONFIG
 */
static void requestGSMGetBroadcastSMSConfig(RIL_Token t)
{
    ATResponse *atResponse = NULL;
    int mode, err = 0;
    unsigned int i, count = 0;
    char *mids;
    char *range;
    char *trange;
    char *tok = NULL;

    RIL_GSM_BroadcastSmsConfigInfo *configInfo[BROADCAST_MAX_RANGES_SUPPORTED];

    err = at_send_command_singleline("AT+CSCB?", "+CSCB:", &atResponse);

    if (err != AT_NOERROR)
        goto error;

    tok = atResponse->p_intermediates->line;

    err = at_tok_start(&tok);
    if (err < 0)
        goto error;

    err = at_tok_nextint(&tok, &mode);
    if (err < 0)
        goto error;

    /**
     * Get the string that yields the service ids (mids). AT+CSCB <mids>
     * parameter may contain a mix of single service ids (,%d,) and service id
     * ranges (,%d-%d,).
     */
    err = at_tok_nextstr(&tok, &mids);
    if (err < 0)
        goto error;

    while (at_tok_nextstr(&mids, &range) == 0) {
        /**
         * Replace any '-' sign with ',' sign to allow for at_tok_nextint
         * for both fromServiceId and toServiceId below.
         */
        trange = range;
        while ((NULL != trange) && ('\0' != *trange)) {
            if ('-' == *trange)
                *trange = ',';
            trange++;
        }
        if (count < BROADCAST_MAX_RANGES_SUPPORTED) {
            configInfo[count] = (RIL_GSM_BroadcastSmsConfigInfo*) calloc(1,
                sizeof(RIL_GSM_BroadcastSmsConfigInfo));
            if (NULL == configInfo[count])
                goto error;

            /* No support for "Not accepted mids", selected is always 1 */
            configInfo[count]->selected = 1;

            /* Fetch fromServiceId value */
            err = at_tok_nextint(&range, &(configInfo[count]->fromServiceId));
            if (err < 0)
                goto error;
            /* Try to fetch toServiceId value if it exist */
            err = at_tok_nextint(&range, &(configInfo[count]->toServiceId));
            if (err < 0)
                configInfo[count]->toServiceId =
                    configInfo[count]->fromServiceId;

            count++;
        } else {
            ALOGW("%s() Max limit (%d) passed, can not send all ranges "
                 "reported by modem.", __func__,
                 BROADCAST_MAX_RANGES_SUPPORTED);
            break;
        }
    }

    RIL_onRequestComplete(t, RIL_E_SUCCESS, &configInfo,
                          sizeof(RIL_GSM_BroadcastSmsConfigInfo *) * count);

    goto exit;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);

exit:
    at_response_free(atResponse);
    for (i = 0; i < count; i++)
        free(configInfo[i]);
}



/**
 * RIL_REQUEST_GSM_SET_BROADCAST_SMS_CONFIG
 */
static void requestGSMSetBroadcastSMSConfig(void *data, size_t datalen,
                                     RIL_Token t)
{
    int err, count, i;
    char *tmp, *mids = NULL;
    RIL_GSM_BroadcastSmsConfigInfo **configInfoArray =
        (RIL_GSM_BroadcastSmsConfigInfo **) data;
    RIL_GSM_BroadcastSmsConfigInfo *configInfo = NULL;

    count = datalen / sizeof(RIL_GSM_BroadcastSmsConfigInfo *);
    ALOGI("Number of MID ranges in BROADCAST_SMS_CONFIG: %d", count);

    for (i = 0; i < count; i++) {
        configInfo = configInfoArray[i];
        /* No support for "Not accepted mids" in AT */
        if (configInfo->selected) {
            tmp = mids;
            asprintf(&mids, "%s%d-%d%s", (tmp ? tmp : ""),
                configInfo->fromServiceId, configInfo->toServiceId,
                (i == (count - 1) ? "" : ",")); /* Last one? Skip comma */
            free(tmp);
        }
    }

    if (mids == NULL) {
        RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
        return;
    }

    err = at_send_command("AT+CSCB=0,\"%s\"", mids);
    free(mids);

    if (err != AT_NOERROR) {
        RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
        return;
    }

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}


/**
 * RIL_REQUEST_GSM_SMS_BROADCAST_ACTIVATION
 */
static void requestGSMSMSBroadcastActivation(void *data, size_t datalen,
                                      RIL_Token t)
{
    ATResponse *atResponse = NULL;
    int mode, mt, bm, ds, bfr, skip;
    int activation;
    char *tok;
    int err;

    (void) datalen;

    /* AT+CNMI=[<mode>[,<mt>[,<bm>[,<ds>[,<bfr>]]]]] */
    err = at_send_command_singleline("AT+CNMI?", "+CNMI:", &atResponse);
    if (err != AT_NOERROR)
        goto error;

    tok = atResponse->p_intermediates->line;

    err = at_tok_start(&tok);
    if (err < 0)
        goto error;
    err = at_tok_nextint(&tok, &mode);
    if (err < 0)
        goto error;
    err = at_tok_nextint(&tok, &mt);
    if (err < 0)
        goto error;
    err = at_tok_nextint(&tok, &skip);
    if (err < 0)
        goto error;
    err = at_tok_nextint(&tok, &ds);
    if (err < 0)
        goto error;
    err = at_tok_nextint(&tok, &bfr);
    if (err < 0)
        goto error;

    /* 0 - Activate, 1 - Turn off */
    activation = *((const int *)data);
    if (activation == 0)
        bm = 2;
    else
        bm = 0;

    err = at_send_command("AT+CNMI=%d,%d,%d,%d,%d", mode, mt, bm, ds, bfr);

    if (err != AT_NOERROR)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);

finally:
    at_response_free(atResponse);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    goto finally;
}

/**
 * Fetch information about UICC card type (SIM/USIM)
 *
 * \return UICC_Type: type of UICC card.
 */
static UICC_Type getUICCType(void)
{
    ATResponse *atResponse = NULL;
    char* line;
    static UICC_Type UiccType = UICC_TYPE_UNKNOWN;
    int err,type;

    if (getRadioState() == RADIO_STATE_OFF ||
        getRadioState() == RADIO_STATE_UNAVAILABLE) {
        return UICC_TYPE_UNKNOWN;
    }

    /* Only if card type is not known yet ... */
    if (UiccType == UICC_TYPE_UNKNOWN) {

        /* Get the card type */
        err = at_send_command_singleline("AT^CARDMODE", "^CARDMODE:", &atResponse);
        if (err != AT_NOERROR)
            goto error;

        line = atResponse->p_intermediates->line;

        err = at_tok_start(&line);

        if (err < 0)
            goto error;

        err = at_tok_nextint(&line, &type);
        if (err < 0)
            goto error;


        UiccType =   (type == 0) ? UICC_TYPE_SIM :
                    ((type == 1) ? UICC_TYPE_USIM :
                    ((type == 2) ? UICC_TYPE_UIM : UICC_TYPE_UNKNOWN));

    }

    ALOGI("Detected card type :%d", UiccType);

error:
    at_response_free(atResponse);

    return UiccType;
}

/**
 * RIL_REQUEST_SIM_IO
 *
 * Request SIM I/O operation.
 * This is similar to the TS 27.007 "restricted SIM" operation
 * where it assumes all of the EF selection will be done by the
 * callee.
 *
 * Reverse Engineered from Kuawei generic RIL *
 */
 
typedef struct _tagSW1SW2{
	unsigned char mask[2];
	unsigned char sw[2];
	char *text; 
} SW1SW2;

static const SW1SW2 sim_answ[]=
{
	  /* Response to commands which are correctly executed */
      { { 0xff, 0xff }, { 0x90, 0x00 } , "Ok" },
      { { 0xff, 0x00 }, { 0x9f, 0x00 } , "%d response bytes available" },
	  /* Memory management */
	  { { 0xff, 0x00 }, { 0x92, 0x00 } , "Update successful but after using an internal retry of %d times" },
	  { { 0xff, 0xff }, { 0x92, 0x40 } , "memory problem" },
	  /* Referencing management */
	  { { 0xff, 0xff }, { 0x94, 0x00 } , "no EF selected" },
	  { { 0xff, 0xff }, { 0x94, 0x02 } , "out of range (invalid address)" },
	  { { 0xff, 0xff }, { 0x94, 0x04 } , "file ID or pattern not found" },
	  { { 0xff, 0xff }, { 0x94, 0x08 } , "file is inconsistent with the command" },
	  /* Security management */
	  { { 0xff, 0xff }, { 0x98, 0x02 } , "no CHV initialised" },
	  { { 0xff, 0xff }, { 0x98, 0x04 } , "access condition not fulfilled,authentication failed at least one attempt left" },
	  { { 0xff, 0xff }, { 0x98, 0x08 } , "in contradiction with CHV status" },
	  { { 0xff, 0xff }, { 0x98, 0x10 } , "in contradiction with invalidation stats" },
	  { { 0xff, 0xff }, { 0x98, 0x40 } , "unsuccessful CHV verification,no attempt left. CHV blocked" },
	  { { 0xff, 0xff }, { 0x98, 0x50 } , "increase cannot be performed,Max value reached" },
	  
      { { 0xff, 0x00 }, { 0x61, 0x00 } , "%d response bytes available" },

      { { 0xff, 0xff }, { 0x62, 0x81 } , "returned data may be corrupt" },
      { { 0xff, 0xff }, { 0x62, 0x82 } , "EOF reached prematurely" },
      { { 0xff, 0xff }, { 0x62, 0x83 } , "selected file invalid" },
      { { 0xff, 0xff }, { 0x62, 0x84 } , "FCI not formated" },
      { { 0xff, 0x00 }, { 0x62, 0x00 } , "nvmem unchanged" },

      { { 0xff, 0x00 }, { 0x63, 0x00 } , "nvmem changed" },
      { { 0xff, 0x00 }, { 0x63, 0x81 } , "file filled up by last write" },
      { { 0xff, 0xf0 }, { 0x63, 0xc0 } , "Counter=%1.1X" },

      { { 0xff, 0xff }, { 0x64, 0x00 } , "nvmem unchanged" },
      { { 0xff, 0x00 }, { 0x64, 0x00 } , "nvmem unchanged - RFU" },

      { { 0xff, 0xff }, { 0x65, 0x00 } , "nvmem changed" },
      { { 0xff, 0xff }, { 0x65, 0x81 } , "nvmem changed - memory failure" },
      { { 0xff, 0x00 }, { 0x65, 0x00 } , "nvmem changed - unknown?" },

      { { 0xff, 0x00 }, { 0x66, 0x00 } , "security related %d" },

      { { 0xff, 0xff }, { 0x67, 0x00 } , "wrong length" },
      { { 0xff, 0x00 }, { 0x67, 0x00 } , "wrong length - %d expected" },

      { { 0xff, 0xff }, { 0x68, 0x81 } , "wrong cla - logical channel not supported" },
      { { 0xff, 0xff }, { 0x68, 0x82 } , "wrong cla - secure messaging not supported" },
      { { 0xff, 0x00 }, { 0x68, 0x00 } , "cla not supported" },

      { { 0xff, 0xff }, { 0x69, 0x81 } , "command incompatible with file structure" },
      { { 0xff, 0xff }, { 0x69, 0x82 } , "security status not satisfied" },
      { { 0xff, 0xff }, { 0x69, 0x83 } , "authentication method blocked" },
      { { 0xff, 0xff }, { 0x69, 0x84 } , "referenced data invalid" },
      { { 0xff, 0xff }, { 0x69, 0x85 } , "conditions of use not satisfied" },
      { { 0xff, 0xff }, { 0x69, 0x86 } , "command not allowed - no current EF" },
      { { 0xff, 0xff }, { 0x69, 0x87 } , "expected SM data objects missing" },
      { { 0xff, 0xff }, { 0x69, 0x88 } , "SM data objects incorrect" },
      { { 0xff, 0x00 }, { 0x69, 0x00 } , "command not allowed" },

      { { 0xff, 0xff }, { 0x6a, 0x80 } , "P1-P2: incorrect parameters in data field" },
      { { 0xff, 0xff }, { 0x6a, 0x81 } , "P1-P2: function not supported" },
      { { 0xff, 0xff }, { 0x6a, 0x82 } , "P1-P2: file not found" },
      { { 0xff, 0xff }, { 0x6a, 0x83 } , "P1-P2: record not found" },
      { { 0xff, 0xff }, { 0x6a, 0x84 } , "P1-P2: not enough memory space in file" },
      { { 0xff, 0xff }, { 0x6a, 0x85 } , "P1-P2: Lc inconsistent with TLV" },
      { { 0xff, 0xff }, { 0x6a, 0x86 } , "P1-P2 incorrect" },
      { { 0xff, 0xff }, { 0x6a, 0x87 } , "P1-P2 inconsistent with Lc" },
      { { 0xff, 0xff }, { 0x6a, 0x88 } , "Referenced data not found" },
      { { 0xff, 0x00 }, { 0x6a, 0x00 } , "P1-P2 invalid" },

      { { 0xff, 0x00 }, { 0x6b, 0x00 } , "P1-P2 invalid" },

      { { 0xff, 0x00 }, { 0x6c, 0x00 } , "wrong length -  %d expected" },

      { { 0xff, 0x00 }, { 0x6d, 0x00 } , "INS code not supported or invalid" },
      
      { { 0xff, 0x00 }, { 0x6e, 0x00 } , "CLA %02X not supported" },

      { { 0xff, 0x00 }, { 0x6f, 0x00 } , "no precise diagnosis" },

      { { 0x00, 0x00 }, { 0x00, 0x00 } , "Unknown response" }
};
 
/* Interpret and print SIM_IO command result */
static void print_simansw(unsigned char sw1, unsigned char sw2)
{
	int j,i;
	
	ALOGD("sw1: 0x%02x, sw2: 0x%02x",sw1,sw2);	
	for(j = 0; j < sizeof(sim_answ)/sizeof(sim_answ[0]); j++) {
	
		if ((sw1 & sim_answ[j].mask[0]) == sim_answ[j].sw[0] &&
			(sw2 & sim_answ[j].mask[1]) == sim_answ[j].sw[1]) {
			ALOGD(sim_answ[j].text,sw2);
			return;
		}
	}		
	
	ALOGD("Unknown error");
}

static unsigned int hex2int(char dig)
{
	if (dig >= '0' && dig <= '9')
		return dig - '0';
	if (dig >= 'a' && dig <= 'f')
		return dig - 'a' + 10;
	if (dig >= 'A' && dig <= 'F')
		return dig - 'A' + 10;
	return 0;
}

static void hexStringToBytes(unsigned char* b, const char* str)
{
	int len = strlen(str);
	len >>= 1;
	while (len--) {
		*b++ = (hex2int(str[0]) << 4) | hex2int(str[1]);
		str += 2;
	};
}

static void requestSIM_IO(void *data, size_t datalen, RIL_Token t)
{
	ATResponse *atResponse = NULL;
	RIL_SIM_IO_Response sr;
	int err;
	RIL_SIM_IO_v6 *p_args;
	char *line;
	char *fmt;

	/* FIXME handle pin2 */
	memset(&sr, 0, sizeof(sr));

	p_args = (RIL_SIM_IO_v6 *)data;
	
	if (!p_args) 
		goto error;
	
	if (p_args->path != NULL && (strlen(p_args->path) & 3) != 0)
		goto error;
			
	if (p_args->fileid == 0x4F30) {
		if (!p_args->data) {
			fmt = "AT+CRSM=%d,%d,%d,%d,%d,,\"3F007F105F3A\"";
		} else {
			fmt = "AT+CRSM=%d,%d,%d,%d,%d,%s,\"3F007F105F3A\"";
		}
	} else {
		if (!p_args->data) {
			fmt = "AT+CRSM=%d,%d,%d,%d,%d";
		} else {
			fmt = "AT+CRSM=%d,%d,%d,%d,%d,%s";
		}
	}
		
	err = at_send_command_singleline(fmt, "+CRSM:", &atResponse, 
			 p_args->command,
			 p_args->fileid, p_args->p1,
			 p_args->p2, p_args->p3,
			 p_args->data);

    if (err != AT_NOERROR)
        goto error;

	line = atResponse->p_intermediates->line;

	err = at_tok_start(&line);
    if (err < 0) goto error;

	err = at_tok_nextint(&line, &(sr.sw1));
	if (err < 0) goto error;

	err = at_tok_nextint(&line, &(sr.sw2));
	if (err < 0) goto error;

	if (at_tok_hasmore(&line)) {
		err = at_tok_nextstr(&line, &(sr.simResponse));
		if (err < 0) goto error;
	}
	
	/* Interpret and print results as a debugging aid */
	print_simansw(sr.sw1,sr.sw2);

	/* If dealing with a USIM card ... */
	if (p_args->command == 0xC0 &&
		sr.simResponse[0] == '6' &&
		sr.simResponse[1] == '2' &&
		sr.simResponse[6] == '0' &&
		sr.simResponse[7] == '5' &&
		strlen(sr.simResponse) <= 30 &&
		getUICCType() != UICC_TYPE_SIM) {
		
		/* Convert it to a format Android understands */
		
		unsigned char buf[15];
		unsigned int val;
		
		// Convert to bytes...
		hexStringToBytes(buf,sr.simResponse);
		
		// Reformat response...
		sr.simResponse[0x00] = '0';
		sr.simResponse[0x01] = '0';
		sr.simResponse[0x02] = '0';
		sr.simResponse[0x03] = '0';
		
		val = buf[8] * (buf[7] + (buf[6]<<8U));
		sprintf(&sr.simResponse[0x04],"%04x",val & 0xFFFFU);
		
		sr.simResponse[0x08] = sr.simResponse[0x16];
		sr.simResponse[0x09] = sr.simResponse[0x17];
		sr.simResponse[0x0A] = sr.simResponse[0x18];
		sr.simResponse[0x0B] = sr.simResponse[0x19];
		sr.simResponse[0x0D] = '4';
		sr.simResponse[0x1A] = '0';
		sr.simResponse[0x1B] = '1';
		sr.simResponse[0x1C] = sr.simResponse[0x0E];
		sr.simResponse[0x1D] = sr.simResponse[0x0F];
		sr.simResponse[0x1E] = 0;
	}
	
	RIL_onRequestComplete(t, RIL_E_SUCCESS, &sr, sizeof(sr));
	at_response_free(atResponse);
	return;

error:
	RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
	at_response_free(atResponse);
}

/*
 * The following list contains values for the structure "RIL_AppStatus" to be
 * sent to Android on a given SIM state. It is indexed by the SIM_Status above.
 */
static const RIL_AppStatus app_status_array[] = {
    /*
     * RIL_AppType,  RIL_AppState,
     * RIL_PersoSubstate,
     * Aid pointer, App Label pointer, PIN1 replaced,
     * RIL_PinState (PIN1),
     * RIL_PinState (PIN2)
     */
    /* SIM_ABSENT = 0 */
    {
        RIL_APPTYPE_UNKNOWN, RIL_APPSTATE_UNKNOWN,
        RIL_PERSOSUBSTATE_UNKNOWN,
        NULL, NULL, 0,
        RIL_PINSTATE_UNKNOWN,
        RIL_PINSTATE_UNKNOWN
    },
    /* SIM_NOT_READY = 1 */
    {
        RIL_APPTYPE_SIM, RIL_APPSTATE_DETECTED,
        RIL_PERSOSUBSTATE_UNKNOWN,
        NULL, NULL, 0,
        RIL_PINSTATE_UNKNOWN,
        RIL_PINSTATE_UNKNOWN
    },
    /* SIM_READY = 2 */
    {
        RIL_APPTYPE_SIM, RIL_APPSTATE_READY,
        RIL_PERSOSUBSTATE_READY,
        NULL, NULL, 0,
        RIL_PINSTATE_UNKNOWN,
        RIL_PINSTATE_UNKNOWN
    },
    /* SIM_PIN = 3 */
    {
        RIL_APPTYPE_SIM, RIL_APPSTATE_PIN,
        RIL_PERSOSUBSTATE_UNKNOWN,
        NULL, NULL, 0,
        RIL_PINSTATE_ENABLED_NOT_VERIFIED,
        RIL_PINSTATE_UNKNOWN
    },
    /* SIM_PUK = 4 */
    {
        RIL_APPTYPE_SIM, RIL_APPSTATE_PUK,
        RIL_PERSOSUBSTATE_UNKNOWN,
        NULL, NULL, 0,
        RIL_PINSTATE_ENABLED_BLOCKED,
        RIL_PINSTATE_UNKNOWN
    },
    /* SIM_NETWORK_PERSO = 5 */
    {
        RIL_APPTYPE_SIM, RIL_APPSTATE_SUBSCRIPTION_PERSO,
        RIL_PERSOSUBSTATE_SIM_NETWORK,
        NULL, NULL, 0,
        RIL_PINSTATE_ENABLED_NOT_VERIFIED,
        RIL_PINSTATE_UNKNOWN
    },
    /* SIM_PIN2 = 6 */
    {
        RIL_APPTYPE_SIM, RIL_APPSTATE_READY,
        RIL_PERSOSUBSTATE_UNKNOWN,
        NULL, NULL, 0,
        RIL_PINSTATE_UNKNOWN,
        RIL_PINSTATE_ENABLED_NOT_VERIFIED
    },
    /* SIM_PUK2 = 7 */
    {
        RIL_APPTYPE_SIM, RIL_APPSTATE_READY,
        RIL_PERSOSUBSTATE_UNKNOWN,
        NULL, NULL, 0,
        RIL_PINSTATE_UNKNOWN,
        RIL_PINSTATE_ENABLED_BLOCKED
    },
    /* SIM_NETWORK_SUBSET_PERSO = 8 */
    {
        RIL_APPTYPE_SIM, RIL_APPSTATE_SUBSCRIPTION_PERSO,
        RIL_PERSOSUBSTATE_SIM_NETWORK_SUBSET,
        NULL, NULL, 0,
        RIL_PINSTATE_ENABLED_NOT_VERIFIED,
        RIL_PINSTATE_UNKNOWN
    },
    /* SIM_SERVICE_PROVIDER_PERSO = 9 */
    {
        RIL_APPTYPE_SIM, RIL_APPSTATE_SUBSCRIPTION_PERSO,
        RIL_PERSOSUBSTATE_SIM_SERVICE_PROVIDER,
        NULL, NULL, 0,
        RIL_PINSTATE_ENABLED_NOT_VERIFIED,
        RIL_PINSTATE_UNKNOWN
    },
    /* SIM_CORPORATE_PERSO = 10 */
    {
        RIL_APPTYPE_SIM, RIL_APPSTATE_SUBSCRIPTION_PERSO,
        RIL_PERSOSUBSTATE_SIM_CORPORATE,
        NULL, NULL, 0,
        RIL_PINSTATE_ENABLED_NOT_VERIFIED,
        RIL_PINSTATE_UNKNOWN
    },
    /* SIM_SIM_PERSO = 11 */
    {
        RIL_APPTYPE_SIM, RIL_APPSTATE_SUBSCRIPTION_PERSO,
        RIL_PERSOSUBSTATE_SIM_SIM,
        NULL, NULL, 0,
        RIL_PINSTATE_ENABLED_NOT_VERIFIED,
        RIL_PINSTATE_UNKNOWN
    },
    /* SIM_STERICSSON_LOCK = 12 */
    {
        RIL_APPTYPE_SIM, RIL_APPSTATE_SUBSCRIPTION_PERSO,
        RIL_PERSOSUBSTATE_UNKNOWN,    /* API (ril.h) does not have this lock */
        NULL, NULL, 0,
        RIL_PINSTATE_ENABLED_NOT_VERIFIED,
        RIL_PINSTATE_UNKNOWN
    },
    /* SIM_BLOCKED = 13 */
    {
        RIL_APPTYPE_SIM, RIL_APPSTATE_UNKNOWN,
        RIL_PERSOSUBSTATE_UNKNOWN,
        NULL, NULL, 0,
        RIL_PINSTATE_ENABLED_BLOCKED,
        RIL_PINSTATE_UNKNOWN
    },
    /* SIM_PERM_BLOCKED = 14 */
    {
        RIL_APPTYPE_SIM, RIL_APPSTATE_UNKNOWN,
        RIL_PERSOSUBSTATE_UNKNOWN,
        NULL, NULL, 0,
        RIL_PINSTATE_ENABLED_PERM_BLOCKED,
        RIL_PINSTATE_UNKNOWN
    },
    /* SIM_NETWORK_PERSO_PUK = 15 */
    {
        RIL_APPTYPE_SIM, RIL_APPSTATE_SUBSCRIPTION_PERSO,
        RIL_PERSOSUBSTATE_SIM_NETWORK_PUK,
        NULL, NULL, 0,
        RIL_PINSTATE_ENABLED_NOT_VERIFIED,
        RIL_PINSTATE_UNKNOWN
    },
    /* SIM_NETWORK_SUBSET_PERSO_PUK = 16 */
    {
        RIL_APPTYPE_SIM, RIL_APPSTATE_SUBSCRIPTION_PERSO,
        RIL_PERSOSUBSTATE_SIM_NETWORK_SUBSET_PUK,
        NULL, NULL, 0,
        RIL_PINSTATE_ENABLED_NOT_VERIFIED,
        RIL_PINSTATE_UNKNOWN
    },
    /* SIM_SERVICE_PROVIDER_PERSO_PUK = 17 */
    {
        RIL_APPTYPE_SIM, RIL_APPSTATE_SUBSCRIPTION_PERSO,
        RIL_PERSOSUBSTATE_SIM_SERVICE_PROVIDER_PUK,
        NULL, NULL, 0,
        RIL_PINSTATE_ENABLED_NOT_VERIFIED,
        RIL_PINSTATE_UNKNOWN
    },
    /* SIM_CORPORATE_PERSO_PUK = 18 */
    {
        RIL_APPTYPE_SIM, RIL_APPSTATE_SUBSCRIPTION_PERSO,
        RIL_PERSOSUBSTATE_SIM_CORPORATE_PUK,
        NULL, NULL, 0,
        RIL_PINSTATE_ENABLED_NOT_VERIFIED,
        RIL_PINSTATE_UNKNOWN
    },
    /* SIM_SIM_PERSO_PUK = 19 */
    {
        RIL_APPTYPE_SIM, RIL_APPSTATE_SUBSCRIPTION_PERSO,
        RIL_PERSOSUBSTATE_SIM_SIM_PUK,
        NULL, NULL, 0,
        RIL_PINSTATE_ENABLED_NOT_VERIFIED,
        RIL_PINSTATE_UNKNOWN
    },
    /* SIM_PUK2_PERM_BLOCKED = 20 */
    {
        RIL_APPTYPE_SIM, RIL_APPSTATE_UNKNOWN,
        RIL_PERSOSUBSTATE_UNKNOWN,
        NULL, NULL, 0,
        RIL_PINSTATE_UNKNOWN,
        RIL_PINSTATE_ENABLED_PERM_BLOCKED
    }
};


/**
 * Get the current card status.
 *
 * This must be freed using freeCardStatus.
 * @return: On success returns RIL_E_SUCCESS.
 */
static int getCardStatus(RIL_CardStatus_v6 **pp_card_status)
{
    RIL_CardState card_state;
    int num_apps;

    SIM_Status sim_status = getSIMStatus();
    if (sim_status == SIM_ABSENT) {
        card_state = RIL_CARDSTATE_ABSENT;
        num_apps = 0;
    } else {
        card_state = RIL_CARDSTATE_PRESENT;
        num_apps = 1;
    }

    /* Allocate and initialize base card status. */
    RIL_CardStatus_v6 *p_card_status = (RIL_CardStatus_v6 *) malloc(sizeof(RIL_CardStatus_v6));
    p_card_status->card_state = card_state;
    p_card_status->universal_pin_state = RIL_PINSTATE_UNKNOWN;
    p_card_status->gsm_umts_subscription_app_index = RIL_CARD_MAX_APPS;
    p_card_status->cdma_subscription_app_index = RIL_CARD_MAX_APPS;
    p_card_status->ims_subscription_app_index = RIL_CARD_MAX_APPS;
    p_card_status->num_applications = num_apps;

    /* Initialize application status. */
    int i;
    for (i = 0; i < RIL_CARD_MAX_APPS; i++)
        p_card_status->applications[i] = app_status_array[SIM_ABSENT];

    /* Pickup the appropriate application status
       that reflects sim_status for gsm. */
    if (num_apps != 0) {
        UICC_Type uicc_type = getUICCType();

        /* Only support one app, gsm/wcdma. */
        p_card_status->num_applications = 1;
        p_card_status->gsm_umts_subscription_app_index = 0;

        /* Get the correct app status. */
        p_card_status->applications[0] = app_status_array[sim_status];
        if (uicc_type == UICC_TYPE_SIM)
            ALOGI("[Card type discovery]: Legacy SIM");
        else { /* defaulting to USIM */
            ALOGI("[Card type discovery]: USIM");
            p_card_status->applications[0].app_type = RIL_APPTYPE_USIM;
        }
    }

    *pp_card_status = p_card_status;
    return RIL_E_SUCCESS;
}

/**
 * Free the card status returned by getCardStatus.
 */
static void freeCardStatus(RIL_CardStatus_v6 *p_card_status) {
    free(p_card_status);
}

/**
 * RIL_REQUEST_GET_SIM_STATUS
 *
 * Requests status of the SIM interface and the SIM card.
 *
 * Valid errors:
 *  Must never fail.
 */
static void requestGetSimStatus(RIL_Token t)
{
    RIL_CardStatus_v6* p_card_status = NULL;

    if (getCardStatus(&p_card_status) != RIL_E_SUCCESS)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, (char*)p_card_status, sizeof(*p_card_status));

finally:
    if (p_card_status != NULL)
        freeCardStatus(p_card_status);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    goto finally;
}



/**
 * RIL_REQUEST_CHANGE_SIM_PIN
 * RIL_REQUEST_CHANGE_SIM_PIN2
*/
static void requestChangePassword(void *data, size_t datalen, RIL_Token t,
                           const char *facility, int request)
{
    int err = 0;
    char *oldPassword = NULL;
    char *newPassword = NULL;
    int num_retries = -1;

    if (datalen != 3 * sizeof(char *) || strlen(facility) != 2)
        goto error;


    oldPassword = ((char **) data)[0];
    newPassword = ((char **) data)[1];

    err = at_send_command("AT+CPWD=\"%s\",\"%s\",\"%s\"", facility,
                oldPassword, newPassword);
    if (err != AT_NOERROR)
        goto error;

    num_retries = 3;
    RIL_onRequestComplete(t, RIL_E_SUCCESS, &num_retries, sizeof(int *));

    return;

error:
    if (at_get_cme_error(err) == CME_INCORRECT_PASSWORD) {
        num_retries = 3;
        RIL_onRequestComplete(t, RIL_E_PASSWORD_INCORRECT, &num_retries, sizeof(int *));
    } else {
        RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    }
}


/**
 * RIL_REQUEST_QUERY_FACILITY_LOCK
 *
 * Query the status of a facility lock state.
 */
static void requestQueryFacilityLock(void *data, size_t datalen, RIL_Token t)
{
    int err, response;
    ATResponse *atResponse = NULL;
    char *line = NULL;
    char *facility_string = NULL;
    char *facility_password = NULL;
    char *facility_class = NULL;
	const char *fmt;

    (void) datalen;

    if (datalen < 3 * sizeof(char **)) {
        ALOGE("%s() bad data length!", __func__);
        goto error;
    }

    facility_string = ((char **) data)[0];
    facility_password = ((char **) data)[1];
    facility_class = ((char **) data)[2];

	if (facility_password != NULL && facility_password[0] != 0) {
		if (facility_class != NULL && facility_class[0] != 0) {
			fmt = "AT+CLCK=\"%s\",2,%s,%s";
		} else {
			fmt = "AT+CLCK=\"%s\",2,%s";
		}
	} else {
		fmt = "AT+CLCK=\"%s\",2";
	}
    err = at_send_command_singleline(fmt, "+CLCK:", &atResponse,
            facility_string, facility_password, facility_class);
    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;

    err = at_tok_start(&line);

    if (err < 0)
        goto error;

    err = at_tok_nextint(&line, &response);

    if (err < 0)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, &response, sizeof(int));
    at_response_free(atResponse);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    at_response_free(atResponse);
}


/**
 * RIL_REQUEST_SET_FACILITY_LOCK
 *
 * Enable/disable one facility lock.
 */
static void requestSetFacilityLock(void *data, size_t datalen, RIL_Token t)
{
    int err;
    char *facility_string = NULL;
    int facility_mode = -1;
    char *facility_mode_str = NULL;
    char *facility_password = NULL;
    char *facility_class = NULL;
    int num_retries = -1;
    RIL_Errno errorril = RIL_E_GENERIC_FAILURE;
    (void) datalen;

    if (datalen < 4 * sizeof(char **)) {
        ALOGE("%s() bad data length!", __func__);
        goto exit;
    }

    facility_string = ((char **) data)[0];
    facility_mode_str = ((char **) data)[1];
    facility_password = ((char **) data)[2];
    facility_class = ((char **) data)[3];

    if (*facility_mode_str != '0' && *facility_mode_str != '1') {
        ALOGE("%s() bad facility mode!", __func__);
        goto exit;
    }

    facility_mode = atoi(facility_mode_str);

    /*
     * Skip adding facility_password to AT command parameters if it is NULL,
     * printing NULL with %s will give string "(null)".
     */
    err = at_send_command("AT+CLCK=\"%s\",%d,\"%s\",%s", facility_string,
            facility_mode, facility_password ? facility_password : "", facility_class);

    if (at_get_error_type(err) == AT_ERROR)
        goto exit;
    if (err != AT_NOERROR) {
        switch (at_get_cme_error(err)) {
        /* CME ERROR 11: "SIM PIN required" happens when PIN is wrong */
        case CME_SIM_PIN_REQUIRED:
            ALOGI("Wrong PIN");
            errorril = RIL_E_PASSWORD_INCORRECT;
            break;
        /*
         * CME ERROR 12: "SIM PUK required" happens when wrong PIN is used
         * 3 times in a row
         */
        case CME_SIM_PUK_REQUIRED:
            ALOGI("PIN locked, change PIN with PUK");
            num_retries = 0;/* PUK required */
            errorril = RIL_E_PASSWORD_INCORRECT;
            break;
        /* CME ERROR 16: "Incorrect password" happens when PIN is wrong */
        case CME_INCORRECT_PASSWORD:
            ALOGI("Incorrect password, Facility: %s", facility_string);
            errorril = RIL_E_PASSWORD_INCORRECT;
            break;
        /* CME ERROR 17: "SIM PIN2 required" happens when PIN2 is wrong */
        case CME_SIM_PIN2_REQUIRED:
            ALOGI("Wrong PIN2");
            errorril = RIL_E_PASSWORD_INCORRECT;
            break;
        /*
         * CME ERROR 18: "SIM PUK2 required" happens when wrong PIN2 is used
         * 3 times in a row
         */
        case CME_SIM_PUK2_REQUIRED:
            ALOGI("PIN2 locked, change PIN2 with PUK2");
            num_retries = 0;/* PUK2 required */
            errorril = RIL_E_SIM_PUK2;
            break;
        default: /* some other error */
            num_retries = -1;
            break;
        }
        goto finally;
    }

    errorril = RIL_E_SUCCESS;

finally:
    if (strncmp(facility_string, "SC", 2) == 0)
        num_retries = 1;
    else if  (strncmp(facility_string, "FD", 2) == 0)
        num_retries = 1;
exit:
    RIL_onRequestComplete(t, errorril, &num_retries,  sizeof(int *));
}



/**
 * Enter SIM PIN, might be PIN, PIN2, PUK, PUK2, etc.
 *
 * Data can hold pointers to one or two strings, depending on what we
 * want to enter. (PUK requires new PIN, etc.).
 *
 * FIXME: Do we need to return remaining tries left on error as well?
 *        Also applies to the rest of the requests that got the retries
 *        in later commits to ril.h.
 */
static void requestEnterSimPin(void *data, size_t datalen, RIL_Token t, int request)
{
    int err = 0;
    int cme_err;
    const char **strings = (const char **) data;
    int num_retries = -1;

    if (datalen == sizeof(char *)) {
        err = at_send_command("AT+CPIN=\"%s\"", strings[0]);
    } else if (datalen == 2 * sizeof(char *)) {
        if (!strings[1]){
            err = at_send_command("AT+CPIN=\"%s\"", strings[0]);
        } else {
            err = at_send_command("AT+CPIN=\"%s\",\"%s\"", strings[0], strings[1]);
        }
    } else if (datalen == 3 * sizeof(char *)) {
            err = at_send_command("AT+CPIN=\"%s\",\"%s\"", strings[0], strings[1]);
    } else {
        goto error;
    }

    cme_err = at_get_cme_error(err);

    if (cme_err != CME_ERROR_NON_CME && err != AT_NOERROR) {
        switch (cme_err) {
        case CME_SIM_PIN_REQUIRED:
        case CME_SIM_PUK_REQUIRED:
        case CME_INCORRECT_PASSWORD:
        case CME_SIM_PIN2_REQUIRED:
        case CME_SIM_PUK2_REQUIRED:
            num_retries = 1;
            RIL_onRequestComplete(t, RIL_E_PASSWORD_INCORRECT, &num_retries, sizeof(int *));
            break;
        default:
            goto error;
        }
    } else {
        /* Got OK, return success. */

        num_retries = 1;
        RIL_onRequestComplete(t, RIL_E_SUCCESS, &num_retries, sizeof(int *));
		
		/* Make sure we get notifications for network registeration
		   of both voice and data now */
		at_send_command("AT+CREG=2");
		at_send_command("AT+CGREG=2");

		/* Notify that SIM is ready */
		setRadioState(RADIO_STATE_SIM_READY);
    }
    return;
error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
}


/**
 * RIL_REQUEST_QUERY_NETWORK_SELECTION_MODE
 *
 * Query current network selectin mode.
 */
static void requestQueryNetworkSelectionMode(RIL_Token t)
{
    int err;
    ATResponse *atResponse = NULL;
    int response = 0;
    char *line;

    err = at_send_command_singleline("AT+COPS?", "+COPS:", &atResponse);

    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;

    err = at_tok_start(&line);

    if (err < 0)
        goto error;

    err = at_tok_nextint(&line, &response);

    if (err < 0)
        goto error;

    /*
     * Android accepts 0(automatic) and 1(manual).
     * Modem may return mode 4(Manual/automatic).
     * Convert it to 1(Manual) as android expects.
     */
    if (response == 4)
        response = 1;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, &response, sizeof(int));

finally:
    at_response_free(atResponse);
    return;

error:
    ALOGE("%s() Must never return error when radio is on", __func__);
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    goto finally;
}


struct operatorPollParams {
    RIL_Token t;
    int loopcount;
};


#define REPOLL_OPERATOR_SELECTED 30     /* 30 * 2 = 1M = ok? */
static const struct timespec TIMEVAL_OPERATOR_SELECT_POLL = { 2, 0 };

/*
 * s_registrationDeniedReason is used to keep track of registration deny
 * reason for which is called by pollOperatorSelected from
 * RIL_REQUEST_SET_NETWORK_SELECTION_AUTOMATIC, so that in case
 * of invalid SIM/ME, Android will not continuously poll for operator.
 *
 */
static Reg_Deny_DetailReason s_registrationDeniedReason = DEFAULT_VALUE;


/**
 * Poll +COPS? and return a success, or if the loop counter reaches
 * REPOLL_OPERATOR_SELECTED, return generic failure.
 */
static void pollOperatorSelected(void *params)
{
    int err = 0;
    int response = 0;
    char *line = NULL;
    ATResponse *atResponse = NULL;
    struct operatorPollParams *poll_params;
    RIL_Token t;

    assert(params != NULL);

    poll_params = (struct operatorPollParams *) params;
    t = poll_params->t;

    if (poll_params->loopcount >= REPOLL_OPERATOR_SELECTED)
        goto error;

    err = at_send_command_singleline("AT+COPS?", "+COPS:", &atResponse);
    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;

    err = at_tok_start(&line);
    if (err < 0)
        goto error;

    err = at_tok_nextint(&line, &response);
    if (err < 0)
        goto error;

    /* If we don't get more than the COPS: {0-4} we are not registered.
       Loop and try again. */
    if (!at_tok_hasmore(&line)) {
        switch (s_registrationDeniedReason) {
        case IMSI_UNKNOWN_IN_HLR: /* fall through */
        case ILLEGAL_ME:
            RIL_onRequestComplete(t, RIL_E_ILLEGAL_SIM_OR_ME, NULL, 0);
            free(poll_params);
            break;
        default:
            poll_params->loopcount++;
            enqueueRILEvent(pollOperatorSelected,
                            poll_params, &TIMEVAL_OPERATOR_SELECT_POLL);
        }
    } else {
        /* We got operator, throw a success! */
        RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
        free(poll_params);
    }

    at_response_free(atResponse);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    free(poll_params);
    at_response_free(atResponse);
    return;
}

/**
 * RIL_REQUEST_SET_NETWORK_SELECTION_AUTOMATIC
 *
 * Specify that the network should be selected automatically.
*/
static void requestSetNetworkSelectionAutomatic(RIL_Token t)
{
    int err = 0;
    ATResponse *atResponse = NULL;
    int mode = 0;
    int skip;
    char *line;
    char *netOperator = NULL;
    struct operatorPollParams *poll_params = NULL;

    poll_params = (struct operatorPollParams*)
                    malloc(sizeof(struct operatorPollParams));
    if (NULL == poll_params)
        goto error;

    /* First check if we are already scanning or in manual mode */
    err = at_send_command_singleline("AT+COPS=3,2;+COPS?", "+COPS:", &atResponse);
    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;

    err = at_tok_start(&line);
    if (err < 0)
        goto error;

    /* Read network selection mode */
    err = at_tok_nextint(&line, &mode);
    if (err < 0)
        goto error;

    /* If we're unregistered, we may just get
       a "+COPS: 0" response. */
    if (!at_tok_hasmore(&line)) {
        if (mode == 1) {
            D("%s() Changing manual to automatic network mode", __func__);
            goto do_auto;
        } else
            goto check_reg;
    }

    err = at_tok_nextint(&line, &skip);
    if (err < 0)
        goto error;

    /* A "+COPS: 0, n" response is also possible. */
    if (!at_tok_hasmore(&line)) {
        if (mode == 1) {
            D("%s() Changing manual to automatic network mode", __func__);
            goto do_auto;
        } else
            goto check_reg;
    }

    /* Read numeric operator */
    err = at_tok_nextstr(&line, &netOperator);
    if (err < 0)
        goto error;

    /* If operator is found then do a new scan,
       else let it continue the already pending scan */
    if (netOperator && strlen(netOperator) == 0) {
        if (mode == 1) {
            D("%s() Changing manual to automatic network mode", __func__);
            goto do_auto;
        } else
            goto check_reg;
    }

    /* Operator found */
    if (mode == 1) {
        D("%s() Changing manual to automatic network mode", __func__);
        goto do_auto;
    } else {
        D("%s() Already in automatic mode with known operator, trigger a new network scan",
        __func__);
        goto do_auto;
    }

    /* Check if module is scanning,
       if not then trigger a rescan */
check_reg:
    at_response_free(atResponse);
    atResponse = NULL;

    /* Check CS domain first */
    err = at_send_command_singleline("AT+CREG?", "+CREG:", &atResponse);
    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;

    err = at_tok_start(&line);
    if (err < 0)
        goto error;

    /* Read registration unsolicited mode */
    err = at_tok_nextint(&line, &mode);
    if (err < 0)
        goto error;

    /* Read registration status */
    err = at_tok_nextint(&line, &mode);
    if (err < 0)
        goto error;

    /* If scanning has stopped, then perform a new scan */
    if (mode == 0) {
        D("%s() Already in automatic mode, but not currently scanning on CS,"
         "trigger a new network scan", __func__);
        goto do_auto;
    }

    /* Now check PS domain */
    at_response_free(atResponse);
    atResponse = NULL;
    err = at_send_command_singleline("AT+CGREG?", "+CGREG:", &atResponse);
    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;

    err = at_tok_start(&line);
    if (err < 0)
        goto error;

    /* Read registration unsolicited mode */
    err = at_tok_nextint(&line, &mode);
    if (err < 0)
        goto error;

    /* Read registration status */
    err = at_tok_nextint(&line, &mode);
    if (err < 0)
        goto error;

    /* If scanning has stopped, then perform a new scan */
    if (mode == 0) {
        D("%s() Already in automatic mode, but not currently scanning on PS,"
         "trigger a new network scan", __func__);
        goto do_auto;
    } else {
        D("%s() Already in automatic mode and scanning", __func__);
        goto finish_scan;
    }

do_auto:
    at_response_free(atResponse);
    atResponse = NULL;

    /* This command does two things, one it sets automatic mode,
       two it starts a new network scan! */
    err = at_send_command("AT+COPS=0");
    if (err != AT_NOERROR)
        goto error;

finish_scan:

    at_response_free(atResponse);
    atResponse = NULL;

    poll_params->loopcount = 0;
    poll_params->t = t;

    enqueueRILEvent(pollOperatorSelected,
                    poll_params, &TIMEVAL_OPERATOR_SELECT_POLL);

    return;

error:
    free(poll_params);
    at_response_free(atResponse);
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    return;
}


/**
 * RIL_REQUEST_SET_NETWORK_SELECTION_MANUAL
 *
 * Manually select a specified network.
 *
 * The radio baseband/RIL implementation is expected to fall back to
 * automatic selection mode if the manually selected network should go
 * out of range in the future.
 */
void requestSetNetworkSelectionManual(void *data, size_t datalen,
                                      RIL_Token t)
{
    /*
     * AT+COPS=[<mode>[,<format>[,<oper>[,<AcT>]]]]
     *    <mode>   = 4 = Manual (<oper> field shall be present and AcT optionally) with fallback to automatic if manual fails.
     *    <format> = 2 = Numeric <oper>, the number has structure:
     *                   (country code digit 3)(country code digit 2)(country code digit 1)
     *                   (network code digit 2)(network code digit 1)
     */

    (void) datalen;
    int err = 0;
    const char *mccMnc = (const char *) data;

    /* Check inparameter. */
    if (mccMnc == NULL)
        goto error;

    /* Build and send command. */
    err = at_send_command("AT+COPS=1,2,\"%s\"", mccMnc);
    if (err != AT_NOERROR) {
        err = at_send_command("AT+COPS=0");
        if(err != AT_NOERROR)
            goto error;
    }

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
}



/**
 * RIL_REQUEST_QUERY_AVAILABLE_NETWORKS
 *
 * Scans for available networks.
*/
static void requestQueryAvailableNetworks(RIL_Token t)
{
    #define QUERY_NW_NUM_PARAMS 4

    /*
     * AT+COPS=?
     *   +COPS: [list of supported (<stat>,long alphanumeric <oper>
     *           ,short alphanumeric <oper>,numeric <oper>[,<AcT>])s]
     *          [,,(list of supported <mode>s),(list of supported <format>s)]
     *
     *   <stat>
     *     0 = unknown
     *     1 = available
     *     2 = current
     *     3 = forbidden
     */
    int err = 0;
    ATResponse *atResponse = NULL;
    const char *statusTable[] =
        { "unknown", "available", "current", "forbidden" };
    char **responseArray = NULL;
    char *p;
    int n = 0;
    int i = 0;

    err = at_send_command_multiline("AT+COPS=?", "+COPS:", &atResponse);
    if (err != AT_NOERROR || atResponse->p_intermediates == NULL) {
        ALOGE("Failed to get operator responses: err:%d, intermediates: %p",err,atResponse->p_intermediates);
        goto error;
    }

    p = atResponse->p_intermediates->line;
    ALOGD("Got line '%s'",p);

    /* count number of '('. */
    err = at_tok_charcounter(p, '(', &n);
    if (err < 0) goto error;

    /* We don't want to process the list of supported modes and the list of supported
       commands, so just skip them */
    if (n < 2) n = 0;
    else n -= 2;

    /* Allocate array of strings, blocks of 4 strings. */
    responseArray = (char **)alloca(n * QUERY_NW_NUM_PARAMS * sizeof(char *));

    /* Loop and collect response information into the response array. */
    for (i = 0; i < n; i++) {
        int status = 0;
        char *line = NULL;
        char *s = NULL;
        char *longAlphaNumeric = NULL;
        char *shortAlphaNumeric = NULL;
        char *numeric = NULL;
        char *remaining = NULL;

        s = line = getFirstElementValue(p, "(", ")", &remaining);
        p = remaining;

        if (line == NULL) {
            ALOGE("%s() Null pointer while parsing COPS response."
             "This should not happen.", __func__);
            break;
        }

        ALOGD("%d operator: '%s'",i,line);

        /* <stat> */
        err = at_tok_nextint(&line, &status);
        if (err < 0) {
error2:
            free(s);
            goto error;
        }

        /* long alphanumeric <oper> */
        err = at_tok_nextstr(&line, &longAlphaNumeric);
        if (err < 0)
            goto error2;

        /* short alphanumeric <oper> */
        err = at_tok_nextstr(&line, &shortAlphaNumeric);
        if (err < 0)
            goto error2;

        /* numeric <oper> */
        err = at_tok_nextstr(&line, &numeric);
        if (err < 0)
            goto error2;

        responseArray[i * QUERY_NW_NUM_PARAMS + 0] = (char*) alloca(strlen(longAlphaNumeric) + 1);
        strcpy(responseArray[i * QUERY_NW_NUM_PARAMS + 0], longAlphaNumeric);

        responseArray[i * QUERY_NW_NUM_PARAMS + 1] = (char*) alloca(strlen(shortAlphaNumeric) + 1);
        strcpy(responseArray[i * QUERY_NW_NUM_PARAMS + 1], shortAlphaNumeric);

        responseArray[i * QUERY_NW_NUM_PARAMS + 2] = (char*) alloca(strlen(numeric) + 1);
        strcpy(responseArray[i * QUERY_NW_NUM_PARAMS + 2], numeric);

        free(s);

        /*
         * Check if modem returned an empty string, and fill it with MNC/MMC
         * if that's the case.
         */
        if (responseArray[i * QUERY_NW_NUM_PARAMS + 0] && strlen(responseArray[i * QUERY_NW_NUM_PARAMS + 0]) == 0) {
            responseArray[i * QUERY_NW_NUM_PARAMS + 0] = (char*) alloca(strlen(responseArray[i * QUERY_NW_NUM_PARAMS + 2]) + 1);
            strcpy(responseArray[i * QUERY_NW_NUM_PARAMS + 0], responseArray[i * QUERY_NW_NUM_PARAMS + 2]);
        }

        if (responseArray[i * QUERY_NW_NUM_PARAMS + 1] && strlen(responseArray[i * QUERY_NW_NUM_PARAMS + 1]) == 0) {
            responseArray[i * QUERY_NW_NUM_PARAMS + 1] = (char*) alloca(strlen(responseArray[i * QUERY_NW_NUM_PARAMS + 2]) + 1);
            strcpy(responseArray[i * QUERY_NW_NUM_PARAMS + 1], responseArray[i * QUERY_NW_NUM_PARAMS + 2]);
        }

        /* Add status */
        responseArray[i * QUERY_NW_NUM_PARAMS + 3] = (char*) alloca(strlen(statusTable[status])+1);
        strcpy(responseArray[i * QUERY_NW_NUM_PARAMS + 3],statusTable[status]);
    }

    RIL_onRequestComplete(t, RIL_E_SUCCESS, responseArray,
                          i * QUERY_NW_NUM_PARAMS * sizeof(char *));

finally:
    at_response_free(atResponse);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    goto finally;
}

/**
 * RIL_REQUEST_SET_BAND_MODE
 *
 * Assign a specified band for RF configuration.
 *
 * "data" is int *
 * ((int *)data)[0] is == 0 for "unspecified" (selected by baseband automatically)
 * ((int *)data)[0] is == 1 for "EURO band"  (GSM-900 / DCS-1800 / WCDMA-IMT-2000)
 * ((int *)data)[0] is == 2 for "US band"    (GSM-850 / PCS-1900 / WCDMA-850 / WCDMA-PCS-1900)
 * ((int *)data)[0] is == 3 for "JPN band" (WCDMA-800 / WCDMA-IMT-2000)
 * ((int *)data)[0] is == 4 for "AUS band"   (GSM-900 / DCS-1800 / WCDMA-850 / WCDMA-IMT-2000)
 * ((int *)data)[0] is == 5 for "AUS band 2" (GSM-900 / DCS-1800 / WCDMA-850)
 *
 * "response" is NULL
 *
 * Valid errors:
 *  SUCCESS
 *  RADIO_NOT_AVAILABLE
 *  GENERIC_FAILURE
 */
static void requestSetBandMode(void *data, size_t datalen, RIL_Token t)
{
    int band,err;
    const char* strband;

    assert (datalen >= sizeof(int *));
    band = ((int *)data)[0];

	switch (band) {
		case 0:
		default:
			strband = "3FFFFFFF";
			break;
		case 1:
			strband = "2000000400380";
			break;
		case 2:
			strband = "4A80000";
			break;
	}
	
    /* Set allowed bands */
    err = at_send_command("AT^SYSCFG=16,3,%s,1,2", strband );
    if (err != AT_NOERROR)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, sizeof(int));
    return;

error:
    ALOGE("ERROR: requestSetBandMode() failed\n");
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);

}

/**
 * RIL_REQUEST_QUERY_AVAILABLE_BAND_MODE
 *
 * Query the list of band mode supported by RF.
 *
 * "data" is NULL
 *
 * "response" is int *
 * "response" points to an array of int's, the int[0] is the size of array, reset is one for
 * each available band mode.
 *
 *  0 for "unspecified" (selected by baseband automatically)
 *  1 for "EURO band" (GSM-900 / DCS-1800 / WCDMA-IMT-2000)
 *  2 for "US band" (GSM-850 / PCS-1900 / WCDMA-850 / WCDMA-PCS-1900)
 *  3 for "JPN band" (WCDMA-800 / WCDMA-IMT-2000)
 *  4 for "AUS band" (GSM-900 / DCS-1800 / WCDMA-850 / WCDMA-IMT-2000)
 *  5 for "AUS band 2" (GSM-900 / DCS-1800 / WCDMA-850)
 *
 * Valid errors:
 *  SUCCESS
 *  RADIO_NOT_AVAILABLE
 *  GENERIC_FAILURE
 *
 * See also: RIL_REQUEST_SET_BAND_MODE
 */
static void requestQueryAvailableBandMode(void *data, size_t datalen, RIL_Token t)
{
    int err = 0;
	char *line;
    ATResponse *atResponse = NULL;
    char *strband;
    int mode, acqorder, band;

    err = at_send_command_singleline("AT^SYSCFG?", "^SYSCFG:", &atResponse);
    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;

    err = at_tok_start(&line);
    if (err < 0)
        goto error;

    err = at_tok_nextint(&line, &mode);
	if (err < 0)
		goto error;

    err = at_tok_nextint(&line, &acqorder);
	if (err < 0)
		goto error;
		
	err = at_tok_nextstr(&line, &strband);
	if (err < 0)
		goto error;

	if (strcmp(strband, "2000000400380") == 0)
		band = 1;
	else if (strcmp(strband, "4A80000") == 0)
		band = 2;
	else 
		band = 0;
	
    RIL_onRequestComplete(t, RIL_E_SUCCESS, &band, sizeof(int));
	
finally:
    at_response_free(atResponse);
    return;

error:
    ALOGE("%s() Failed to get current band mode", __func__);
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    goto finally;
}


/**
 * RIL_REQUEST_SET_PREFERRED_NETWORK_TYPE
 *
 * Requests to set the preferred network type for searching and registering
 * (CS/PS domain, RAT, and operation mode).
 */
static void requestSetPreferredNetworkType(void *data, size_t datalen, RIL_Token t)
{
    int rat,err;
    const char* cmd;

    assert (datalen >= sizeof(int *));
    rat = ((int *)data)[0];

    switch (rat) {
        case PREF_NET_TYPE_GSM_ONLY:  		/* GSM only */
            cmd = "AT^SYSCFG=13,1,40000000,2,4"; /* GSM only */
            break;
		case PREF_NET_TYPE_WCDMA:			/* WCDMA only */
            cmd = "AT^SYSCFG=14,2,40000000,2,4"; /* WCDMA only */
            break;
        case PREF_NET_TYPE_GSM_WCDMA: 		/* GSM/WCDMA (WCDMA preferred) */
            cmd = "AT^SYSCFG=2,2,40000000,2,4";  /* Automatic, prefer WCDMA */
			break;
		case PREF_NET_TYPE_GSM_WCDMA_AUTO:	/* GSM/WCDMA (auto mode, according to PRL) */
        default:    /* Dual Mode - WCDMA preferred*/
            cmd = "AT^SYSCFG=2,0,40000000,2,4";  /* for 3G Preferred */
            break;
    }

    /* Set mode */
    err = at_send_command(cmd);
    if (err != AT_NOERROR)
        goto error;


    /* Trigger autoregister */
    err = at_send_command("AT+COPS=0");
    if (err != AT_NOERROR)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, sizeof(int));
    return;

error:
    ALOGE("ERROR: requestSetPreferredNetworkType() failed\n");
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
}


/**
 * RIL_REQUEST_GET_PREFERRED_NETWORK_TYPE
 *
 * Query the preferred network type (CS/PS domain, RAT, and operation mode)
 * for searching and registering.
 */
static void requestGetPreferredNetworkType(RIL_Token t)
{
    ATResponse *atResponse = NULL;
    int err;
    char *line;
    int ret1,ret2;
    int response = PREF_NET_TYPE_GSM_WCDMA_AUTO;

    err = at_send_command_singleline("AT^SYSCFG?", "^SYSCFG:", &atResponse);

    if (err != AT_NOERROR) {
        // assume radio is off
        goto error;
    }

    line = atResponse->p_intermediates->line;

    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &ret1);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &ret2);
    if (err < 0) goto error;

    /* Based on reported syscfg */
    if (ret1 == 13 && ret2 == 1) {
        response = PREF_NET_TYPE_GSM_ONLY;  /* GSM only */
    } else if (ret1 == 14 && ret2 == 2) {
        response = PREF_NET_TYPE_WCDMA; 	/* WCDMA only */
    } else if (ret1 == 2 && ret2 == 2) {
        response = PREF_NET_TYPE_GSM_WCDMA; /* GSM or WCDMA */
    } else {
        response = PREF_NET_TYPE_GSM_WCDMA_AUTO; /* Auto */
    }

    D("requestGetPreferredNetworkType() mode:%d\n",response);
    RIL_onRequestComplete(t, RIL_E_SUCCESS, &response, sizeof(int));
    at_response_free(atResponse);
    return;

error:
    ALOGE("ERROR: requestGetPreferredNetworkType() failed - modem does not support command\n");
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    at_response_free(atResponse);
}

/* +CGREG AcT values */
enum CREG_AcT {
    CGREG_ACT_GSM               = 0,
    CGREG_ACT_GSM_COMPACT       = 1, /* Not Supported */
    CGREG_ACT_UTRAN             = 2,
    CGREG_ACT_GSM_EGPRS         = 3,
    CGREG_ACT_UTRAN_HSDPA       = 4,
    CGREG_ACT_UTRAN_HSUPA       = 5,
    CGREG_ACT_UTRAN_HSUPA_HSDPA = 6
};

/* +CGREG stat values */
enum CREG_stat {
    CGREG_STAT_NOT_REG            = 0,
    CGREG_STAT_REG_HOME_NET       = 1,
    CGREG_STAT_NOT_REG_SEARCHING  = 2,
    CGREG_STAT_REG_DENIED         = 3,
    CGREG_STAT_UKNOWN             = 4,
    CGREG_STAT_ROAMING            = 5
};


/**
 * RIL_REQUEST_VOICE_REGISTRATION_STATE
 *
 * Request current registration state.
 */
static void requestRegistrationState(RIL_Token t)
{
    int err = 0;
    const char resp_size = 15;
    int response[15];
    char *responseStr[15] = {0};
    ATResponse *cgreg_resp = NULL;
    char *line;
    int commas = 0;
    int skip, cs_status = 0;
    int i;

    /* Setting default values in case values are not returned by AT command */
    for (i = 0; i < resp_size; i++)
        responseStr[i] = NULL;

    memset(response, 0, sizeof(response));

    err = at_send_command_singleline("AT+CREG?", "+CREG:", &cgreg_resp);
    if (err != AT_NOERROR)
        goto error;

    line = cgreg_resp->p_intermediates->line;

    err = at_tok_start(&line);
    if (err < 0)
        goto error;

    /*
     * The solicited version of the CREG response is
     * +CREG: n, stat, [lac, cid]
     * and the unsolicited version is
     * +CREG: stat, [lac, cid]
     * The <n> parameter is basically "is unsolicited creg on?"
     * which it should always be.
     *
     * Now we should normally get the solicited version here,
     * but the unsolicited version could have snuck in
     * so we have to handle both.
     *
     * Also since the LAC and CID are only reported when registered,
     * we can have 1, 2, 3, or 4 arguments here.
     *
     * finally, a +CGREG: answer may have a fifth value that corresponds
     * to the network type, as in;
     *
     *   +CGREG: n, stat [,lac, cid [,networkType]]
     */

    /* Count number of commas */
    err = at_tok_charcounter(line, ',', &commas);

    if (err < 0)
        goto error;

    switch (commas) {
    case 0:                    /* +CREG: <stat> */
        err = at_tok_nextint(&line, &response[0]);
        if (err < 0)
            goto error;

        response[1] = -1;
        response[2] = -1;
        break;

    case 1:                    /* +CREG: <n>, <stat> */
        err = at_tok_nextint(&line, &skip);
        if (err < 0)
            goto error;

        err = at_tok_nextint(&line, &response[0]);
        if (err < 0)
            goto error;

        response[1] = -1;
        response[2] = -1;
        if (err < 0)
            goto error;
        break;
    case 2:                    /* +CREG: <stat>, <lac>, <cid> */
        err = at_tok_nextint(&line, &response[0]);
        if (err < 0)
            goto error;

        err = at_tok_nexthexint(&line, &response[1]);
        if (err < 0)
            goto error;

        err = at_tok_nexthexint(&line, &response[2]);
        if (err < 0)
            goto error;
        break;
    case 3:                    /* +CREG: <n>, <stat>, <lac>, <cid> */
    case 4:                    /* +CREG: <n>, <stat>, <lac>, <cid>, <?> */
        err = at_tok_nextint(&line, &skip);
        if (err < 0)
            goto error;

        err = at_tok_nextint(&line, &response[0]);
        if (err < 0)
            goto error;

        err = at_tok_nexthexint(&line, &response[1]);
        if (err < 0)
            goto error;

        err = at_tok_nexthexint(&line, &response[2]);
        if (err < 0)
            goto error;
        break;
    default:
        goto error;
    }


    asprintf(&responseStr[0], "%d", response[0]);

    if (response[1] > 0)
        asprintf(&responseStr[1], "%04x", response[1]);
    if (response[2] > 0)
        asprintf(&responseStr[2], "%08x", response[2]);

    if (response[0] == CGREG_STAT_REG_HOME_NET ||
        response[0] == CGREG_STAT_ROAMING)
        responseStr[3] = (0);

    RIL_onRequestComplete(t, RIL_E_SUCCESS, responseStr,
                          resp_size * sizeof(char *));

finally:

    for (i = 0; i < resp_size; i++)
        free(responseStr[i]);

    at_response_free(cgreg_resp);
    return;

error:
    ALOGE("%s() Must never return an error when radio is on", __func__);
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    goto finally;
}

/**
 * RIL_REQUEST_DATA_REGISTRATION_STATE
 *
 * Request current GPRS registration state.
 */
static void requestGprsRegistrationState(RIL_Token t)
{
    int err = 0;
    const char resp_size = 6;
    int response[resp_size];
    char *responseStr[resp_size];
    ATResponse *atResponse = NULL;
    char *line, *p;
    int commas = 0;
    int skip, tmp;
    int count = 3;

    memset(responseStr, 0, sizeof(responseStr));
    memset(response, 0, sizeof(response));
    response[1] = -1;
    response[2] = -1;

    err = at_send_command_singleline("AT+CGREG?", "+CGREG: ", &atResponse);
    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;
    err = at_tok_start(&line);
    if (err < 0)
        goto error;
    /*
     * The solicited version of the +CGREG response is
     * +CGREG: n, stat, [lac, cid [,<AcT>]]
     * and the unsolicited version is
     * +CGREG: stat, [lac, cid [,<AcT>]]
     * The <n> parameter is basically "is unsolicited creg on?"
     * which it should always be.
     *
     * Now we should normally get the solicited version here,
     * but the unsolicited version could have snuck in
     * so we have to handle both.
     *
     * Also since the LAC, CID and AcT are only reported when registered,
     * we can have 1, 2, 3, 4 or 5 arguments here.
     */
    /* Count number of commas */
    p = line;
    err = at_tok_charcounter(line, ',', &commas);
    if (err < 0) {
        ALOGE("%s() at_tok_charcounter failed", __func__);
        goto error;
    }

    switch (commas) {
    case 0:                    /* +CGREG: <stat> */
        err = at_tok_nextint(&line, &response[0]);
        if (err < 0) goto error;
        break;

    case 1:                    /* +CGREG: <n>, <stat> */
        err = at_tok_nextint(&line, &skip);
        if (err < 0) goto error;
        err = at_tok_nextint(&line, &response[0]);
        if (err < 0) goto error;
        break;

    case 2:                    /* +CGREG: <stat>, <lac>, <cid> */
        err = at_tok_nextint(&line, &response[0]);
        if (err < 0) goto error;
        err = at_tok_nexthexint(&line, &response[1]);
        if (err < 0) goto error;
        err = at_tok_nexthexint(&line, &response[2]);
        if (err < 0) goto error;
        break;

    case 3:                    /* +CGREG: <n>, <stat>, <lac>, <cid> */
                               /* +CGREG: <stat>, <lac>, <cid>, <AcT> */
        err = at_tok_nextint(&line, &tmp);
        if (err < 0) goto error;

        /* We need to check if the second parameter is <lac> */
        if (*(line) == '"') {
            response[0] = tmp; /* <stat> */
            err = at_tok_nexthexint(&line, &response[1]); /* <lac> */
            if (err < 0) goto error;
            err = at_tok_nexthexint(&line, &response[2]); /* <cid> */
            if (err < 0) goto error;
            err = at_tok_nextint(&line, &response[3]); /* <AcT> */
            if (err < 0) goto error;
            count = 4;
        } else {
            err = at_tok_nextint(&line, &response[0]); /* <stat> */
            if (err < 0) goto error;
            err = at_tok_nexthexint(&line, &response[1]); /* <lac> */
            if (err < 0) goto error;
            err = at_tok_nexthexint(&line, &response[2]); /* <cid> */
            if (err < 0) goto error;
        }
        break;

    case 4:                    /* +CGREG: <n>, <stat>, <lac>, <cid>, <AcT> */
        err = at_tok_nextint(&line, &skip); /* <n> */
        if (err < 0) goto error;
        err = at_tok_nextint(&line, &response[0]); /* <stat> */
        if (err < 0) goto error;
        err = at_tok_nexthexint(&line, &response[1]); /* <lac> */
        if (err < 0) goto error;
        err = at_tok_nexthexint(&line, &response[2]); /* <cid> */
        if (err < 0) goto error;
        err = at_tok_nextint(&line, &response[3]); /* <AcT> */
        if (err < 0) goto error;
        count = 4;
        break;

    default:
        ALOGE("%s() Invalid input", __func__);
        goto error;
    }

    /* Converting to stringlist for Android */

    asprintf(&responseStr[0], "%d", response[0]); /* state */

    if (response[1] >= 0)
        asprintf(&responseStr[1], "%04x", response[1]); /* LAC */
    else
        responseStr[1] = NULL;

    if (response[2] >= 0)
        asprintf(&responseStr[2], "%08x", response[2]); /* CID */
    else
        responseStr[2] = NULL;

    if (response[0] == CGREG_STAT_REG_HOME_NET ||
        response[0] == CGREG_STAT_ROAMING)
        asprintf(&responseStr[3], "%d",response[3]);
    else
        responseStr[3] = NULL;

    responseStr[5] = (char*) "1";

    RIL_onRequestComplete(t, RIL_E_SUCCESS, responseStr, resp_size * sizeof(char *));

finally:

    if (responseStr[0])
        free(responseStr[0]);
    if (responseStr[1])
        free(responseStr[1]);
    if (responseStr[2])
        free(responseStr[2]);
    if (responseStr[3])
        free(responseStr[3]);

    at_response_free(atResponse);
    return;

error:
    ALOGE("%s Must never return an error when radio is on", __func__);
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    goto finally;
}


/**
 * RIL_REQUEST_OEM_HOOK_RAW
 *
 * This request reserved for OEM-specific uses. It passes raw byte arrays
 * back and forth.
*/
static void requestOEMHookRaw(void *data, size_t datalen, RIL_Token t)
{
    RIL_onRequestComplete(t, RIL_E_SUCCESS, data, datalen);
    return;
}


/**
 * RIL_REQUEST_OEM_HOOK_STRINGS
 *
 * This request reserved for OEM-specific uses. It passes strings
 * back and forth.
 */
static void requestOEMHookStrings(void * data, size_t datalen, RIL_Token t)
{
    int i;
    const char **cur;
    ATResponse *atResponse = NULL;
    ATLine *atline;
    int linecount;
    int err;

    D("%s() got OEM_HOOK_STRINGS: %8p %lu", __func__, data, (long) datalen);

    for (i = (datalen / sizeof(char *)), cur = (const char **) data;
         i > 0; cur++, i--) {
        D("%s(): String: %s", __func__, *cur);
    }

    /* Only take the first string in the array for now */
    cur = (const char **) data;
    err = at_send_command_raw(*cur, &atResponse);

    if ((err != AT_NOERROR && at_get_error_type(err) == AT_ERROR)
            || atResponse == NULL || atResponse->finalResponse == NULL)
        goto error;

    /* Count number of lines including prefix, intermediate and final response */
    linecount = 0;
    atline = atResponse->p_intermediates;
    while (atline != NULL && atline->line != NULL) {
        linecount++;
        atline = atline->p_next;
    }
    linecount++; /* for finalResponse */

    /* Create RIL response */
    if (linecount > 0) {
        cur = (const char **) alloca(linecount * sizeof (char *));
        if (cur != NULL) {
            linecount = 0;
            atline = atResponse->p_intermediates;
            while (atline != NULL && atline->line != NULL) {
                cur[linecount++] = atline->line;
                atline = atline->p_next;
            }
            cur[linecount++] = atResponse->finalResponse;
        } else
            goto error;
    }

    RIL_onRequestComplete(t, RIL_E_SUCCESS, cur, linecount * sizeof(char *));
    at_response_free(atResponse);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    at_response_free(atResponse);
}


static void unsolicitedNitzTime(const char * s)
{
    int err;
    char * response = NULL;
    char * dt = NULL;
	char * tm = NULL;
    char * tz = NULL; /* Timezone */
    static char sNITZtime[64] = {0};

    char * line = strdup(s);

    /* Higher layers expect a NITZ string in this format:
     *  08/10/28,19:08:37-20,1 (yy/mm/dd,hh:mm:ss(+/-)tz,dst)
     */
	if (strStartsWith(s,"^NWTIME:")) {
		/* Network time, as reported by Huawei modems:
		   ^NWTIME: 12/09/22,08:33:59+8,01 */
		
        /* We got the network time, now assemble the response and send to upper layers */
        at_tok_start(&line);

        err = at_tok_nextstr(&line, &dt);
        if (err < 0) goto error;

        err = at_tok_nextstr(&line, &tm);
        if (err < 0) goto error;

        err = at_tok_nextstr(&line, &tz);
        if (err < 0) goto error;

        asprintf(&response, "%s,%s,%s", dt ,tm ,tz);
        RIL_onUnsolicitedResponse(RIL_UNSOL_NITZ_TIME_RECEIVED, response, strlen(response));
        free(response);

        free(line);
        return;
		
	} else	 
    if (strStartsWith(s,"+CTZV:")){

        /* Get Time and Timezone data and store in static variable.
         * Wait until DST is received to send response to upper layers
         */
        at_tok_start(&line);

        err = at_tok_nextstr(&line, &tz);
        if (err < 0) goto error;

        err = at_tok_nextstr(&line, &response);
        if (err < 0) goto error;

        strcpy(sNITZtime,response);
        strcat(sNITZtime,tz);

        free(line);
        return;

    } else if (strStartsWith(s,"+CTZDST:")){

        /* We got DST, now assemble the response and send to upper layers */
        at_tok_start(&line);

        err = at_tok_nextstr(&line, &tz);
        if (err < 0) goto error;

        asprintf(&response, "%s,%s", sNITZtime, tz);
        RIL_onUnsolicitedResponse(RIL_UNSOL_NITZ_TIME_RECEIVED, response, strlen(response));
        free(response);

        free(line);
        return;

    }

error:
    ALOGE("Invalid NITZ line %s\n", s);
	free(line);
}

/* Unsolicited call start status */
static void unsolicitedCONN(const char * s)
{
    int err;
	int call_id;
	int type;
    char * line = strdup(s);

    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &call_id);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &type);
    if (err < 0) goto error;

	ALOGD("Call started: ID:%d, type:%d", call_id, type);

	/* Store last call index */
	lastCallIdx = call_id;
	
    free(line);
    return;

error:
    ALOGI("Error getting Call Start status");
    free(line);
    return;
}

/* Unsolicited call start status */
static void unsolicitedORIG(const char * s)
{
    int err;
	int call_id;
	int type;
    char * line = strdup(s);

    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &call_id);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &type);
    if (err < 0) goto error;

	ALOGD("Call originated: ID:%d, type:%d", call_id, type);

	/* Store last call index */
	lastCallIdx = call_id;
	
    free(line);
    return;

error:
    ALOGI("Error getting Call Originated status");
    free(line);
    return;
}

/* Unsolicited call conference status */
static void unsolicitedCONF(const char * s)
{
    int err;
	int call_id;
    char * line = strdup(s);

    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &call_id);
    if (err < 0) goto error;

	ALOGD("Call conference started: ID:%d", call_id);

	/* Store last call index */
	lastCallIdx = call_id;
	
    free(line);
    return;

error:
    ALOGI("Error getting Call Conference Start status");
    free(line);
    return;
}

/* Unsolicited call end status */
static void unsolicitedCEND(const char * s)
{
    int err;
	int call_id;
	int duration;
	int end_status;
	int cc_cause = CALL_FAIL_NORMAL;
    char * line = strdup(s);

    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &call_id);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &duration);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &end_status); 
    if (err < 0) goto error;
	
    if (at_tok_hasmore(&line)) { /* not present if error */
        err = at_tok_nextint(&line, &cc_cause);
		if (err < 0) goto error;
	}
	
	ALOGD("Call ended: ID:%d, duration:%ds status:%d, cccause:%d", call_id, duration, end_status, cc_cause);

	/* Store last call error */
	lastCallFailCause = cc_cause;
	
    free(line);
    return;

error:
    ALOGI("Error getting Call End status");
	lastCallFailCause = CALL_FAIL_NORMAL;
    free(line);
    return;
}


static void unsolicitedDCONN(const char *s)
{
}

static void unsolicitedDEND(const char *s)
{
}

static void unsolicitedRSSI(const char * s)
{
    int err;
    int rssi;
    RIL_SignalStrength_v6 signalStrength;
	
    char * line = strdup(s);

    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &rssi);
    if (err < 0) goto error;

    signalStrength.GW_SignalStrength.signalStrength = rssi;
    signalStrength.GW_SignalStrength.bitErrorRate = 99;
    signalStrength.CDMA_SignalStrength.dbm = 0;
    signalStrength.CDMA_SignalStrength.ecio = 0;
    signalStrength.EVDO_SignalStrength.dbm = 0;
    signalStrength.EVDO_SignalStrength.ecio = 0;
    signalStrength.EVDO_SignalStrength.signalNoiseRatio = 0;
    signalStrength.LTE_SignalStrength.signalStrength = 0;
    signalStrength.LTE_SignalStrength.rsrp = 0;
    signalStrength.LTE_SignalStrength.rsrq = 0;
    signalStrength.LTE_SignalStrength.rssnr = 0;
    signalStrength.LTE_SignalStrength.cqi = 0;

    ALOGI("Signal Strength %d", rssi);

    RIL_onUnsolicitedResponse(RIL_UNSOL_SIGNAL_STRENGTH, &signalStrength, sizeof(signalStrength));
    free(line);
    return;

error:
    /* The notification was for a battery event - do not send a msg to upper layers */
    ALOGI("Error getting Signal Strength");
    free(line);
    return;
}

/* Get SIM status: 1=valid, all the others are invalid */
static int getSimState(void)
{
    ATResponse *atResponse = NULL;
    int err;
    char *line;
    int status;
	int srv_status,srv_domain,roam_status,sys_mode,sim_state = -1;

    // Check status.
    err = at_send_command_singleline("AT^SYSINFO", "^SYSINFO:", &atResponse);
    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;
    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &srv_status);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &srv_domain);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &roam_status);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &sys_mode);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &sim_state);
    if (err < 0) goto error;
	

	ALOGE("Sim State: %d",sim_state);
    at_response_free(atResponse);
    return sim_state;

error:
	ALOGE("Failed to get sim state");

    at_response_free(atResponse);
    return -1;
}

static int requestNetworkRegistration()
{
	ATResponse *atResponse = NULL;
    int err;
    char *line;
	int srv_status,srv_domain,nr = 0;

    // Check status.
    err = at_send_command_singleline("AT^SYSINFO", "^SYSINFO:", &atResponse);
    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;
    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &srv_status);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &srv_domain);
    if (err < 0) goto error;

	nr = (srv_status == 2 && (srv_domain == 2 || srv_domain == 3));
	
	ALOGE("Network registration (PS) : %s", nr ? "Yes" : "No");
    at_response_free(atResponse);
    return nr;

error:
	ALOGE("Failed to get network registration");

    at_response_free(atResponse);
    return -1;
}

/* SIM status change */
static void unsolicitedSimStatus(const char * s)
{
    int err;
    int state;
    char * line = NULL;

    /*
    ^SIMST:0 Invalid USIM card state
	^SIMST:1 Valid USIM card state
	^SIMST:2 Invalid USIM card in case of CS
	^SIMST:3 Invalid USIM card in case of PS
	^SIMST:4 Invalid USIM card in case of CS or PS
	^SIMST:255 Card not present
    */

    line = strdup(s);

    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &state);
    if (err < 0) goto error;

	ALOGD("SIM state: %d",state);
	
	free(line);
    return;

error:
    ALOGI("Error getting state");
    free(line);
    return;
}

static void unsolicitedSrvStatus(const char * s)
{
    int err;
    int mode;
    char * line = NULL;

    /*
    ^SRVST:0 No service
	^SRVST:1 Restricted service
	^SRVST:2 Valid service
	^SRVST:3 Restricted Regional Service
	^SRVST:4 Power saving and deep sleep
    */

    line = strdup(s);

    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &mode);
    if (err < 0) goto error;

	free(line);
    return;

error:
    ALOGI("Error getting mode");
    free(line);
    return;
}


static void unsolicitedMode(const char * s)
{
    int err;
    int mode1;
    int mode2;
    char * line = NULL;

    /*
    ^MODE:3,2 indicates GPRS
    ^MODE:3,3 indicates EDGE
    ^MODE:5,4 indicates 3G
    ^MODE:5,5 indicates HSDPA
    */

    line = strdup(s);

    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &mode1);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &mode2);
    if (err < 0) goto error;

    free(line);
    return;

error:
    ALOGI("Error getting mode");
    free(line);
    return;
}


/**
 * RIL_REQUEST_SIGNAL_STRENGTH
 *
 * Requests current signal strength and bit error rate.
 *
 * Must succeed if radio is on.
 */
static void requestSignalStrength(RIL_Token t)
{
    RIL_SignalStrength_v6 signalStrength;
    ATResponse *atResponse = NULL;
    int err;
    char *line;
    int ber;
    int rssi;

    memset(&signalStrength, 0, sizeof(RIL_SignalStrength_v6));

    signalStrength.LTE_SignalStrength.signalStrength = 0x7FFFFFFF;
    signalStrength.LTE_SignalStrength.rsrp = 0x7FFFFFFF;
    signalStrength.LTE_SignalStrength.rsrq = 0x7FFFFFFF;
    signalStrength.LTE_SignalStrength.rssnr = 0x7FFFFFFF;
    signalStrength.LTE_SignalStrength.cqi = 0x7FFFFFFF;

    err = at_send_command_singleline("AT+CSQ", "+CSQ:", &atResponse);
    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;

    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line,&rssi);
    if (err < 0) goto error;

    signalStrength.GW_SignalStrength.signalStrength = rssi;

    err = at_tok_nextint(&line, &ber);
    if (err < 0)
        goto error;

    signalStrength.GW_SignalStrength.bitErrorRate = ber;

    signalStrength.CDMA_SignalStrength.dbm = 0;
    signalStrength.CDMA_SignalStrength.ecio = 0;
    signalStrength.EVDO_SignalStrength.dbm = 0;
    signalStrength.EVDO_SignalStrength.ecio = 0;
    signalStrength.EVDO_SignalStrength.signalNoiseRatio = 0;
    signalStrength.LTE_SignalStrength.signalStrength = 0;
    signalStrength.LTE_SignalStrength.rsrp = 0;
    signalStrength.LTE_SignalStrength.rsrq = 0;
    signalStrength.LTE_SignalStrength.rssnr = 0;
    signalStrength.LTE_SignalStrength.cqi = 0;

    ALOGI("SignalStrength %d BER: %d", rssi, ber);

    RIL_onRequestComplete(t, RIL_E_SUCCESS, &signalStrength,
                          sizeof(RIL_SignalStrength_v6));

    at_response_free(atResponse);
    atResponse = NULL;
    return;

error:
    ALOGE("%s() Must never return an error when radio is on", __func__);
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    at_response_free(atResponse);
    return;
}



/**
 * RIL_REQUEST_OPERATOR
 *
 * Request current operator ONS or EONS.
 */
static void requestOperator(RIL_Token t)
{
    int err;
    int i;
    int skip;
    ATLine *cursor;
    static const int num_resp_lines = 3;
    char *response[num_resp_lines];
    ATResponse *atResponse = NULL;

    memset(response, 0, sizeof(response));

    err = at_send_command_multiline
        ("AT+COPS=3,0;+COPS?;+COPS=3,1;+COPS?;+COPS=3,2;+COPS?", "+COPS:",
         &atResponse);

    if (err != AT_NOERROR)
        goto error;

    /* We expect 3 lines here:
     * +COPS: 0,0,"T - Mobile"
     * +COPS: 0,1,"TMO"
     * +COPS: 0,2,"310170"
     */
    for (i = 0, cursor = atResponse->p_intermediates;
         cursor != NULL && i < num_resp_lines;
         cursor = cursor->p_next, i++) {
        char *line = cursor->line;

        err = at_tok_start(&line);

        if (err < 0)
            goto error;

        err = at_tok_nextint(&line, &skip);

        if (err < 0)
            goto error;

        /* If we're unregistered, we may just get
           a "+COPS: 0" response. */
        if (!at_tok_hasmore(&line)) {
            response[i] = NULL;
            continue;
        }

        err = at_tok_nextint(&line, &skip);

        if (err < 0)
            goto error;

        /* A "+COPS: 0, n" response is also possible. */
        if (!at_tok_hasmore(&line)) {
            response[i] = NULL;
            continue;
        }

        err = at_tok_nextstr(&line, &(response[i]));

        if (err < 0)
            goto error;
    }

    if (i != num_resp_lines)
        goto error;

    /*
     * Check if modem returned an empty string, and fill it with MNC/MMC
     * if that's the case.
     */
    if (response[2] && response[0] && strlen(response[0]) == 0) {
        response[0] = (char*) alloca(strlen(response[2]) + 1);
        strcpy(response[0], response[2]);
    }

    if (response[2] && response[1] && strlen(response[1]) == 0) {
        response[1] = (char*) alloca(strlen(response[2]) + 1);
        strcpy(response[1], response[2]);
    }

    RIL_onRequestComplete(t, RIL_E_SUCCESS, response, sizeof(response));

finally:
    at_response_free(atResponse);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    goto finally;
}


/**
 * RIL_REQUEST_RADIO_POWER
 *
 * Toggle radio on and off (for "airplane" mode).
*/
static void requestRadioPower(void *data, size_t datalen, RIL_Token t)
{
    int onOff;
    int err;
    int restricted;

    if (datalen < sizeof(int *)) {
        ALOGE("%s() bad data length!", __func__);
        goto error;
    }

    onOff = ((int *)data)[0];

    if (onOff == 0 && getRadioState() != RADIO_STATE_OFF) {
        char value[PROPERTY_VALUE_MAX];

		err = at_send_command("AT+CFUN=0");			
		if (err != AT_NOERROR)
			goto error;

        if (property_get("sys.shutdown.requested", value, NULL)) {
            setRadioState(RADIO_STATE_UNAVAILABLE);
        } else
            setRadioState(RADIO_STATE_OFF);

    } else if (onOff > 0 && getRadioState() == RADIO_STATE_OFF) {

		err = at_send_command("AT+CFUN=1");			
        if (err != AT_NOERROR) {
		
            // Some stacks return an error when there is no SIM,
            // but they really turn the RF portion on
            // So, if we get an error, let's check to see if it
            // turned on anyway

            if (isRadioOn() != 1) {
                goto error;
            }
        }
        setRadioState(RADIO_STATE_SIM_NOT_READY);
    }

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);

    restricted = RIL_RESTRICTED_STATE_NONE;
    RIL_onUnsolicitedResponse(RIL_UNSOL_RESTRICTED_STATE_CHANGED,
                              &restricted, sizeof(int *));

    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
}


/**
 * RIL_REQUEST_GET_IMSI
*/
static void requestGetIMSI(RIL_Token t)
{
    ATResponse *atResponse = NULL;
    int err;

    err = at_send_command_numeric("AT+CIMI", &atResponse);

    if (err != AT_NOERROR)
        RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    else {
        RIL_onRequestComplete(t, RIL_E_SUCCESS,
                atResponse->p_intermediates->line,
                sizeof(char *));
    }
    at_response_free(atResponse);
}

/**
 * RIL_REQUEST_GET_IMEI
 *
 * Get the device IMEI, which should be 15 decimal digits.
 */
static void requestGetIMEI(RIL_Token t)
{
    ATResponse *atResponse = NULL;
    int err;

    /* IMEI */
    err = at_send_command_numeric("AT+CGSN", &atResponse);

    if (err != AT_NOERROR)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, atResponse->p_intermediates->line, sizeof(char *));

    at_response_free(atResponse);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    at_response_free(atResponse);
}
	 
/**
 * RIL_REQUEST_GET_IMEISV
 *
 * Get the device IMEISV, which should be two decimal digits.
*/
static void requestGetIMEISV(RIL_Token t)
{
    RIL_onRequestComplete(t, RIL_E_SUCCESS, (void*)"01", sizeof(char *));
}

/**
 * RIL_REQUEST_DEVICE_IDENTITY
 *
 * Request the device ESN / MEID / IMEI / IMEISV.
 *
 */
static void requestDeviceIdentity(RIL_Token t)
{
    ATResponse *atResponse = NULL;
    char* response[4];
    int err;

    /* IMEI */
    err = at_send_command_numeric("AT+CGSN", &atResponse);

    if (err != AT_NOERROR)
        goto error;

    response[0] = atResponse->p_intermediates->line;

    /* IMEISVN */
    response[1] = (char*) "01";

    /* CDMA not supported */
    response[2] = (char*) "";
    response[3] = (char*) "";

    RIL_onRequestComplete(t, RIL_E_SUCCESS, &response, sizeof(response));

    at_response_free(atResponse);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    at_response_free(atResponse);
}



/**
 * RIL_REQUEST_BASEBAND_VERSION
 *
 * Return string value indicating baseband version, eg
 * response from AT+CGMR.
*/
static void requestBasebandVersion(RIL_Token t)
{
    int err;
    ATResponse *atResponse = NULL;
    char *line;

    err = at_send_command_singleline("AT+CGMR", "\0", &atResponse);

    if (err != AT_NOERROR) {
        ALOGE("%s() Error reading Base Band Version", __func__);
        RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
        return;
    }

    line = atResponse->p_intermediates->line;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, line, sizeof(char *));

    at_response_free(atResponse);
}

/**
 * RIL_REQUEST_STK_SETUP_CALL
 *
 */
static void requestStkSetupCall(void * data, size_t datalen, RIL_Token t)
{
    int onOff;
    int err;

    if (datalen < sizeof(int *)) {
        ALOGE("%s() bad data length!", __func__);
        goto error;
    }

    onOff = ((int *)data)[0];
	err = at_send_command("AT^CSTC=%d", (onOff)?1:0 );			
	if (err != AT_NOERROR)
		goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
}

/**
 * RIL_REQUEST_STK_SEND_TERMINAL_RESPONSE
 *
 * Requests to send a terminal response to SIM for a received
 * proactive command.
 */
static void requestStkSendTerminalResponse(void * data, size_t datalen, RIL_Token t)
{
      int err;
    const char *ec = (const char *) data;
    (void)datalen;

    err = at_send_command("AT^CSTR=%d,\"%s\"",strlen(ec),ec);

    if (err != AT_NOERROR)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL,0 );
    return;
	
error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);

}


/**
 * RIL_REQUEST_STK_SEND_ENVELOPE_COMMAND
 *
 * Requests to send a SAT/USAT envelope command to SIM.
 * The SAT/USAT envelope command refers to 3GPP TS 11.14 and 3GPP TS 31.111.
 */
static void requestStkSendEnvelopeCommand(void * data, size_t datalen, RIL_Token t)
{
    int err;
    const char *ec = (const char *) data;
    (void)datalen;

    err = at_send_command("AT^CSEN=%d,\"%s\"",strlen(ec),ec);

    if (err != AT_NOERROR)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0 );
    return;
	
error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
}


/**
 * RIL_REQUEST_STK_GET_PROFILE
 *
 * Requests the profile of SIM tool kit.
 * The profile indicates the SAT/USAT features supported by ME.
 * The SAT/USAT features refer to 3GPP TS 11.14 and 3GPP TS 31.111.
 */
static void requestStkGetProfile(RIL_Token t)
{
    RIL_onRequestComplete(t, RIL_E_SUCCESS, (void*)"default", sizeof(char *));
}


static void initStackUnsolv()
{
	at_send_command("AT^CSMN");
}

/**
 * RIL_REQUEST_STK_SET_PROFILE
 *
 * Download the STK terminal profile as part of SIM initialization
 * procedure.
 */
static void requestStkSetProfile(void * data, size_t datalen, RIL_Token t)
{
    int err = 0;
    int length = 0;
    char *profile = NULL;

    profile = (char *)data;
    length = strlen(profile);

    // Accept everyrhing
    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}


/**
 * RIL_REQUEST_REPORT_STK_SERVICE_IS_RUNNING
 *
 * Turn on STK unsol commands.
 */
void requestReportStkServiceIsRunning(RIL_Token t)
{
    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}

/**
 * RIL_REQUEST_DIAL
 */
static void requestDial(void *data, size_t datalen, RIL_Token t)
{
    RIL_Dial *p_dial;
    const char *clir;
    int ret;

    p_dial = (RIL_Dial *)data;

    switch (p_dial->clir) {
        case 1: clir = "I"; break;  /*invocation*/
        case 2: clir = "i"; break;  /*suppression*/
        default:
        case 0: clir = ""; break;   /*subscription default*/
    }

    /* Originate call */
    ret = at_send_command("ATD%s%s;", p_dial->address, clir);

    /* Switch to audio from USB1 */
    at_send_command("AT^DDSETEX=2");
	
    /* success or failure is ignored by the upper layer here.
       it will call GET_CURRENT_CALLS and determine success that way */
    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}


static void requestHangup(void *data, size_t datalen, RIL_Token t)
{
    int *p_line;

    int ret;

    p_line = (int *)data;

	/* Just in case, stop the audio tunnel now. We need to stop if before the modem
       stops streaming audio. If another call is being held, it will be restarted a
       little bit later by the ccurrent call list callback */
	stopAudioTunnel(NULL);
	
    // 3GPP 22.030 6.5.5
    // "Releases a specific active call X"
    ret = at_send_command("AT+CHLD=1%d", p_line[0]);

    /* success or failure is ignored by the upper layer here.
       it will call GET_CURRENT_CALLS and determine success that way */
    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}

static void requestHangupWaitingOrBackground(RIL_Token t)
{
	/* Just in case, stop the audio tunnel now. We need to stop if before the modem
       stops streaming audio. If another call is being hold, it will be restarted a
       little bit later by the ccurrent call list callback */
	stopAudioTunnel(NULL);

    // 3GPP 22.030 6.5.5
    // "Releases all held calls or sets User Determined User Busy
    //  (UDUB) for a waiting call."
    at_send_command("AT+CHLD=0");

    /* success or failure is ignored by the upper layer here.
       it will call GET_CURRENT_CALLS and determine success that way */
    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}


static void requestHangupForegroundResumeBackground(RIL_Token t)
{
	/* Just in case, stop the audio tunnel now. We need to stop if before the modem
       stops streaming audio. If another call is being hold, it will be restarted a
       little bit later */
	stopAudioTunnel(NULL);

    // 3GPP 22.030 6.5.5
    // "Releases all active calls (if any exist) and accepts
    //  the other (held or waiting) call."
    at_send_command("AT+CHLD=1");

    /* success or failure is ignored by the upper layer here.
       it will call GET_CURRENT_CALLS and determine success that way */
    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}


static void requestSwitchWaitingOrHoldingAndActive(RIL_Token t)
{
	/* Just in case, stop the audio tunnel now. We need to stop if before the modem
       stops streaming audio. If another call is being hold, it will be restarted a
       little bit later */
	stopAudioTunnel(NULL);

    // 3GPP 22.030 6.5.5
    // "Places all active calls (if any exist) on hold and accepts
    //  the other (held or waiting) call."
    at_send_command("AT+CHLD=2");

    /* success or failure is ignored by the upper layer here.
       it will call GET_CURRENT_CALLS and determine success that way */
    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}


static void requestAnswer(RIL_Token t)
{
	/* Answer call */
	at_send_command("ATA");
	
	/* Switch to audio from USB1 */
	at_send_command("AT^DDSETEX=2");
	
    /* success or failure is ignored by the upper layer here.
       it will call GET_CURRENT_CALLS and determine success that way */
    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}


static void requestConference(RIL_Token t)
{
    // 3GPP 22.030 6.5.5
    // "Adds a held call to the conversation"
    at_send_command("AT+CHLD=3");

    /* success or failure is ignored by the upper layer here.
       it will call GET_CURRENT_CALLS and determine success that way */
    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}


static void requestUDUB(RIL_Token t)
{
    /* user determined user busy */
    /* sometimes used: ATH */
    at_send_command("ATH");

    /* success or failure is ignored by the upper layer here.
       it will call GET_CURRENT_CALLS and determine success that way */
    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);

}


static void requestSeparateConnection(void * data, size_t datalen, RIL_Token t)
{
    int party = ((int*)data)[0];

    // Make sure that party is in a valid range.
    // (Note: The Telephony middle layer imposes a range of 1 to 7.
    // It's sufficient for us to just make sure it's single digit.)
    if (party > 0 && party < 10) {
        at_send_command("AT+CHLD=2%d", party);
        RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
    } else {
        RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    }
}

static int wasMuted = 0;
static void requestSetMute(void *data, size_t datalen, RIL_Token t)
{
    int err;
    assert (datalen >= sizeof(int *));
	int mute = ((int*)data)[0];
	
	/* Store mute status */
	wasMuted = mute;
	
    /* mute - Don't consider it an error if it fails, as we are using a Software loopback for audio data */
    err = at_send_command("AT+CMUT=%d", mute);

	/* Update the audio tunnel mute status */
	muteAudioTunnel(wasMuted);
	
    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
    return;
}


static void requestGetMute(RIL_Token t)
{
    int err;
    ATResponse *atResponse = NULL;
    int response[1];
    char *line;

	/* If we fail, return a cached version of it */
    err = at_send_command_singleline("AT+CMUT?", "+CMUT:", &atResponse);
    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;

    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &response[0]);
    if (err < 0) goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, response, sizeof(char*));
    at_response_free(atResponse);

    return;

error:
    ALOGE("ERROR: requestGetMute failed - Returned cached result");
	
	response[0] = wasMuted;
    RIL_onRequestComplete(t, RIL_E_SUCCESS, response, sizeof(char*));
    at_response_free(atResponse);
}


static void requestDTMF(void * data, size_t datalen, RIL_Token t)
{
    int err = 0;
    char c = ((char *)data)[0];

    /* Start DTMF generation for 250ms */
    err = at_send_command("AT^DTMF=%d,%c", lastCallIdx, (int)c); /* 1=call idx */
	
	/* If failed, try the older method */
    if (err != AT_NOERROR) {
		err = at_send_command("AT+VTS=%c", (int)c);
	}
	
    if (err != AT_NOERROR) {
        RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    } else {
        RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
    }

}


static void requestDtmfStop(RIL_Token t)
{
    int err;

    /* Send a command to cancel the DTMF tone*/
    err = at_send_command("AT");
    if (err != AT_NOERROR)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
    return;

error:
    ALOGE("ERROR: requestDtmfStop failed");
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);

}

static void requestDtmfStart(void *data, size_t datalen, RIL_Token t)
{
    int err;
    char c;

    assert (datalen >= sizeof(char *));

    c = ((char *)data)[0];

    /* Start DTMF generation for 250ms */
    err = at_send_command("AT^DTMF=%d,%c", lastCallIdx, (int)c); /* 1=call idx */
	
	/* If failed, try the older method */
    if (err != AT_NOERROR) {
		err = at_send_command("AT+VTS=%c", (int)c);
	}
	
    if (err != AT_NOERROR)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
    return;

error:
    ALOGE("ERROR: requestDtmfStart failed");
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);

}

/* decodes USSDs. Sometimes, they arrive as GSM8, sometimes they arrive as HEX encoded GSM7... */
static char * decodeUSSD(const char* message)
{
	bytes_t utf8 = NULL;
	
	/* Could be in Hex or in GSM8 ... - Check it - Some modems return HEX 
	   encoded GSM7, others return GSM8... */
	int pos = 0;
	int ishex = 1;
	while (1) {
		unsigned char c = message[pos];
		if (!c) break;
		if (!((c >= '0' && c <= '9') || ( c >= 'A' && c <= 'F' ))) {
			ishex = 0;
			break;
		}
		pos++;
	};

	/* Not hex encoded. Just convert to UTF-8 from GSM8 */
	if (!ishex) {
		
		/* Convert GSM8 to UTF8 */
		int gsmbytes = strlen(message);		
		int utf8bytes = utf8_from_gsm8((cbytes_t)message,gsmbytes,NULL);
		utf8 = malloc( utf8bytes + 1);
		utf8_from_gsm8((cbytes_t)message,gsmbytes,utf8);
		utf8[utf8bytes]= '\0';

	} else {
	
		int gsmseptets;
		int utf8bytes;

		/* Convert HEX to GSM7 */
		int hexlen = strlen(message);
		int binlen = (hexlen + 1) >> 1;
		bytes_t bytes = malloc(binlen + 1);
		gsm_hex_to_bytes((cbytes_t)message,hexlen,bytes);
		bytes[binlen] = 0;

		/* And then, GSM7 to UTF8 */
		gsmseptets = (binlen * 8) / 7; // Round DOWN
		utf8bytes = utf8_from_gsm7(bytes, 0, gsmseptets, NULL);
		utf8 = malloc(utf8bytes + 1);
		utf8_from_gsm7(bytes, 0, gsmseptets, utf8);
		utf8[utf8bytes] = 0;
		free(bytes);
	}
	
	return utf8;
}

static void unsolicitedUSSD(const char *s)
{
    char *line, *linestart;
    int typeCode, count, err;
	char *message;
    char *responseStr[2] = {0,0};

    D("unsolicitedUSSD %s\n",s);

    linestart = line = strdup(s);
	
    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &typeCode);
    if (err < 0) goto error;

	if (typeCode < 0 || typeCode > 5) goto error;
	
	if (at_tok_hasmore(&line)) {
		
		err = at_tok_nextstr(&line, &message);
        if(err < 0) goto error;

		/* Decode message */
		responseStr[1] = (char*) decodeUSSD(message);
		
        count = 2;
		
    } else {
	
        responseStr[1] = NULL;
        count = 1;
    }
    
    asprintf(&responseStr[0], "%d", typeCode);

    RIL_onUnsolicitedResponse (RIL_UNSOL_ON_USSD, responseStr, count*sizeof(char*));
	
	free(linestart);
    if (responseStr[0])
        free(responseStr[0]);
	if (responseStr[1])
		free(responseStr[1]);

    return;

error:
    ALOGE("unexpectedUSSD error\n");
    free(linestart);
}


static void requestCancelUSSD(RIL_Token t)
{
    int err = 0;
    ATResponse *atResponse = NULL;

    err = at_send_command_numeric("AT+CUSD=2", &atResponse);

    if (err != AT_NOERROR) {
        RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    } else {
        RIL_onRequestComplete(t, RIL_E_SUCCESS,
                atResponse->p_intermediates->line, sizeof(char *));
    }

    at_response_free(atResponse);
    return;
}


static void requestQueryCallWaiting(void *data, size_t datalen, RIL_Token t)
{
    ATResponse *atResponse = NULL;
    int err;
    char *line;
    int aclass;
    int response[2];

    aclass = ((int *)data)[0];

    err = at_send_command_singleline("AT+CCWA=1,2,%d", "+CCWA:", &atResponse, aclass);
    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;

    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &(response[0]));
    if (err < 0) goto error;

    if (at_tok_hasmore(&line)) {
        err = at_tok_nextint(&line, &(response[1]));
        if (err < 0) goto error;
    }

    RIL_onRequestComplete(t, RIL_E_SUCCESS, response, sizeof(response));
    at_response_free(atResponse);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    at_response_free(atResponse);
}

static void requestSetCallWaiting(void *data, size_t datalen, RIL_Token t)
{
    ATResponse *atResponse = NULL;
    int err;
    int enabled, aclass;

    if((datalen<2)||(data==NULL)) goto error;

    enabled = ((int *)data)[0];
    aclass = ((int *)data)[1];

    err = at_send_command("AT+CCWA=0,%d,%d",enabled,aclass);
    if (err != AT_NOERROR)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
    at_response_free(atResponse);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    at_response_free(atResponse);
}


static void requestQueryCallForwardStatus(RIL_Token t)
{
    int err = 0;
    int i = 0;
    int n = 0;
    int tmp = 0;
    ATResponse *atResponse = NULL;
    ATLine *p_cur;
    RIL_CallForwardInfo **responses = NULL;

    err = at_send_command_multiline("AT+CCFC=0,2", "+CCFC:", &atResponse);

    if (err != AT_NOERROR)
        goto error;

    for (p_cur = atResponse->p_intermediates; p_cur != NULL; p_cur = p_cur->p_next)
        n++;

    responses = (RIL_CallForwardInfo **) alloca(n * sizeof(RIL_CallForwardInfo *));

    for (i = 0; i < n; i++) {
        responses[i] = (RIL_CallForwardInfo*)alloca(sizeof(RIL_CallForwardInfo));
        responses[i]->status = 0;
        responses[i]->reason = 0;
        responses[i]->serviceClass = 0;
        responses[i]->toa = 0;
        responses[i]->number = (char*)"";
        responses[i]->timeSeconds = 0;
    }

    for (i = 0,p_cur = atResponse->p_intermediates; p_cur != NULL; p_cur = p_cur->p_next, i++) {
        char *line = p_cur->line;

        err = at_tok_start(&line);
        if (err < 0) goto error;

        err = at_tok_nextint(&line, &(responses[i]->status));
        if (err < 0) goto error;

        err = at_tok_nextint(&line, &(responses[i]->serviceClass));
        if (err < 0) goto error;

        if(!at_tok_hasmore(&line)) continue;

        err = at_tok_nextstr(&line, &(responses[i]->number));
        if (err < 0) goto error;

        if(!at_tok_hasmore(&line)) continue;

        err = at_tok_nextint(&line, &(responses[i]->toa));
        if (err < 0) goto error;

        if(!at_tok_hasmore(&line)) continue;
        at_tok_nextint(&line,&tmp);

        if(!at_tok_hasmore(&line)) continue;
        at_tok_nextint(&line,&tmp);

        if(!at_tok_hasmore(&line)) continue;
        err = at_tok_nextint(&line, &(responses[i]->timeSeconds));
        if (err < 0) goto error;

    }

    RIL_onRequestComplete(t, RIL_E_SUCCESS, responses, sizeof(RIL_CallForwardInfo **));
    at_response_free(atResponse);

    return;

error:
    at_response_free(atResponse);
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
}


static void requestSetCallForward(void *data, RIL_Token t)
{
    int err = 0;
    RIL_CallForwardInfo *info = NULL;

    info = ((RIL_CallForwardInfo *) data);

    if(data == NULL)
        goto error;

    err = at_send_command("AT+CCFC = %d, %d, \"%s\"",
            info->reason, info->status,
            info->number);
    if (err != AT_NOERROR)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
}


static void requestGetCLIR(void *data, size_t datalen, RIL_Token t)
{
    /* Even though data and datalen must be NULL and 0 respectively this
     * condition is not verified
     */
    ATResponse *atResponse = NULL;
    int response[2];
    char *line = NULL;
    int err = 0;

    err = at_send_command_singleline("AT+CLIR?", "+CLIR:", &atResponse);
    if (err != AT_NOERROR)
        goto error;


    line = atResponse->p_intermediates->line;
    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &(response[0]));
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &(response[1]));
    if (err < 0) goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, response, sizeof(response));
    at_response_free(atResponse);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    at_response_free(atResponse);
}


static void requestSetCLIR(void *data, size_t datalen, RIL_Token t)
{
    int err = 0;
    err = at_send_command("AT+CLIR=%d", ((int *)data)[0]);
    if (err < 0)
        RIL_onRequestComplete(t, RIL_E_PASSWORD_INCORRECT, NULL, 0);
    else
        RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}


static void requestSendUSSD(void *data, size_t datalen, RIL_Token t)
{
    int err = 0;
    cbytes_t ussdRequest = (cbytes_t)(data);

	/*
     * ussdRequest should be checked for invalid characters that can be used to
     * send invalid AT commands. However Android have complete check of
     * ussd strings before they are sent to the RIL.
     *
     * AT+CUSD=[<n>[,<str>[,<dcs>]]]
     * <n> = 0,1,2 Dissable, Enable, Cancel
     * <str> = CUSD string in UTF8
     * <dcs> = Cell Brodcast Data Coding Scheme:
     *   0000 German
     *   0001 English
     *   0010 Italian
     *   0011 French
     *   0100 Spanish
     *   0101 Dutch
     *   0110 Swedish
     *   0111 Danish
     *   1000 Portuguese
     *   1001 Finnish
     *   1010 Norwegian
     *   1011 Greek
     *   1100 Turkish
     *   1101..1110 Reserved for European languages
     *   1111 Language unspecified
     *
     * According to Android ril.h , CUSD messages are always sent as utf8,
     * but the dcs field does not have an entry for this.
     * The nearest "most correct" would be 15 = unspecified,
     * not adding the dcs would result in the default "0" meaning German,
     * and some networks are not happy with this.
	 *
	 * Huawei modems, depending on firmware version, seem to accept GSM8 or
	 *  a HEX encoded GSM7 request. Try GSM8; if it fails, try GSM7 encoded in HEX
     */
	
	/* Convert to GSM8 from UTF8 */
	int utf8len = strlen((const char*)ussdRequest);
	int gsmbytes = utf8_to_gsm8(ussdRequest, utf8len, NULL);
	bytes_t gsm8 = malloc(gsmbytes + 1);
	utf8_to_gsm8(ussdRequest, utf8len, gsm8);
	gsm8[gsmbytes] = 0;
	
	/* Try to send it as GSM8 */	
    err = at_send_command("AT+CUSD=1,%s,15", gsm8);
	
	/* Some modems like the string to be between quotes ... Retry if error */
	if (err != AT_NOERROR) {
		err = at_send_command("AT+CUSD=1,\"%s\",15", gsm8);
	}
	free(gsm8);
	
	/* If modem did not accept the USSD, retry it but using HEX encoded GSM7 */
    if (err != AT_NOERROR) {
		char* hex = NULL;
	
		/* Convert to GSM7 */
		int gsmseptets = utf8_to_gsm7(ussdRequest, utf8len, NULL, 0); 
		int gsmbytes = ((gsmseptets * 7) + 7) / 8; // Round UP, to be able to transmit incomplete septets
		bytes_t gsm7 = malloc(gsmbytes + 1);
		utf8_to_gsm7(ussdRequest, utf8len, gsm7, 0);
		gsm7[gsmbytes] = 0;
			
		/* And then encode to HEX */
		hex = malloc(2*gsmbytes+1);
		gsm_hex_from_bytes(hex, gsm7, gsmbytes);
		hex[2*gsmbytes]='\0';
		
		/* Retry to send the command */
		ALOGD("utf8len: %d, gsmseptets: %d, gsmbytes:%d", utf8len, gsmseptets, gsmbytes);
		err = at_send_command("AT+CUSD=1,\"%s\",15", hex);
		free(gsm7);
		free(hex);
		
		if (err != AT_NOERROR)
			goto error;
	}

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
}


static void requestChangeBarringPassword(void *data, size_t datalen, RIL_Token t)
{
    int err = 0;
    char *string = NULL;
    char *old_password = NULL;
    char *new_password = NULL;

    assert (datalen >=  (3 * sizeof(char **)));

    string     = ((char **)data)[0];
    old_password = ((char **)data)[1];
    new_password = ((char **)data)[2];

    err = at_send_command("AT+CPWD=\"%s\",\"%s\",\"%s\"", string, old_password, new_password);
    if (err != AT_NOERROR)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
}


static void requestQueryCLIP(RIL_Token t)
{
    ATResponse *atResponse = NULL;
    int err;
    char *line;
    int response;

    err = at_send_command_singleline("AT+CLIP?","+CLIP:",&atResponse);
    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;
    err = at_tok_start(&line);
    if (err < 0) goto error;

    /* The first number is discarded */
    err = at_tok_nextint(&line, &response);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &response);
    if (err < 0) goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, &response, sizeof(int));
    at_response_free(atResponse);
    return;

error:
    at_response_free(atResponse);
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
}

static void requestResetRadio(RIL_Token t)
{
    int err = 0;

    /* Reset MS */
    err = at_send_command("AT+CFUN=6");

    /* Go online */
    err = at_send_command("AT^RFSWITCH=1");  /* ignore error. If unsupported, the next line will enable RF */
    err = at_send_command("AT+CFUN=1");
    if (err != AT_NOERROR)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    return;
}

static void requestSetSuppSVCNotification(void *data, size_t datalen, RIL_Token t)
{
    int err = 0;
    int enabled = 0;
    enabled = ((int *)data)[0];

    err = at_send_command("AT+CSSN=%d,%d", enabled, enabled);
    if (err != AT_NOERROR)
        RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    else
        RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}


static void requestExplicitCallTransfer(RIL_Token t)
{
    int err = 0;
    err = at_send_command("AT+CHLD=4");
    if (err != AT_NOERROR)
        RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    else
        RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}


static void requestSetLocationUpdates(void *data, size_t datalen, RIL_Token t)
{
    int err = 0;
    int updates = 0;

    updates = ((int *)data)[0] == 1? 2 : 1;

    err = at_send_command("AT+CREG=%d", updates);
    if (err != AT_NOERROR)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
}

static void requestLastFailCause(RIL_Token t)
{
    ATResponse *atResponse = NULL;
    int err = 0;
    int response = 0;
    char *tmp = NULL;
    char *line = NULL;

    err = at_send_command_singleline("AT+CEER", "+CEER:", &atResponse);
    if(err != AT_NOERROR)
        goto error;

	/*
	CALL_FAIL_NORMAL = 16,
    CALL_FAIL_BUSY = 17,
    CALL_FAIL_CONGESTION = 34,
    CALL_FAIL_ACM_LIMIT_EXCEEDED = 68,
    CALL_FAIL_CALL_BARRED = 240,
    CALL_FAIL_FDN_BLOCKED = 241,
    CALL_FAIL_ERROR_UNSPECIFIED = 0xffff
	*/
		
    line = atResponse->p_intermediates->line;
    err = at_tok_start(&line);
    if(err < 0) goto error;

    err = at_tok_nextstr(&line, &tmp);
    if(err < 0) goto error;

    if (at_tok_hasmore(&line)) {
		err = at_tok_nextint(&line, &response);
		if(err < 0) goto error;
	}

	ALOGD("Textual last call fail cause: %s [%d]", tmp, response);
	
error:

	/* Send the last call fail cause */
    RIL_onRequestComplete(t, RIL_E_SUCCESS, &lastCallFailCause, sizeof(int));
    at_response_free(atResponse);
    return;
}

/*
 * "data" is int *
 * ((int *)data)[0] is == 0 from RUIM/SIM (default)
 * ((int *)data)[0] is == 1 from NV
 */
static void requestCdmaSubscription(void * data, size_t datalen, RIL_Token t)
{

    /*
    * "data" is int *
    * ((int *)data)[0] is == 0 from RUIM/SIM (default)
    * ((int *)data)[0] is == 1 from NV
    */

    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}


/*
 * RIL_REQUEST_GET_NEIGHBORING_CELL_IDS
 */
static void requestNeighboringCellIds(void * data, size_t datalen, RIL_Token t)
{
    int err;
    ATResponse *atResponse = NULL;
    char *line, *p;
    int commas;
    int skip;
    int lac = 0,cid = 0;
    int count = 3;
    int ber;
    int rssi;

    RIL_NeighboringCell **pp_cellIds;
    RIL_NeighboringCell *p_cellIds;

    pp_cellIds = (RIL_NeighboringCell **)alloca(sizeof(RIL_NeighboringCell *));
    p_cellIds = (RIL_NeighboringCell *)alloca(sizeof(RIL_NeighboringCell));
    pp_cellIds[0]=p_cellIds;

    err = at_send_command_singleline("AT+CREG?", "+CREG:", &atResponse);
    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;

    err = at_tok_start(&line);
    if (err < 0) goto error;

    /* Ok you have to be careful here
     * The solicited version of the CREG response is
     * +CREG: n, stat, [lac, cid]
     * and the unsolicited version is
     * +CREG: stat, [lac, cid]
     * The <n> parameter is basically "is unsolicited creg on?"
     * which it should always be
     *
     * Now we should normally get the solicited version here,
     * but the unsolicited version could have snuck in
     * so we have to handle both
     *
     * Also since the LAC and CID are only reported when registered,
     * we can have 1, 2, 3, or 4 arguments here
     *
     * finally, a +CGREG: answer may have a fifth value that corresponds
     * to the network type, as in;
     *
     *   +CGREG: n, stat [,lac, cid [,networkType]]
     */

    /* count number of commas */
    commas = 0;
    for (p = line ; *p != '\0' ;p++) {
        if (*p == ',') commas++;
    }
    switch (commas) {
        case 0: /* +CREG: <stat> */
        case 1: /* +CREG: <n>, <stat> */
            goto error;

        case 2: /* +CREG: <stat>, <lac>, <cid> */
            err = at_tok_nextint(&line, &skip);
            if (err < 0) goto error;
            err = at_tok_nexthexint(&line, &lac);
            if (err < 0) goto error;
            err = at_tok_nexthexint(&line, &cid);
            if (err < 0) goto error;
            break;
        case 3: /* +CREG: <n>, <stat>, <lac>, <cid> */
            err = at_tok_nextint(&line, &skip);
            if (err < 0) goto error;
            err = at_tok_nextint(&line, &skip);
            if (err < 0) goto error;
            err = at_tok_nexthexint(&line, &lac);
            if (err < 0) goto error;
            err = at_tok_nexthexint(&line, &cid);
            if (err < 0) goto error;
            break;
        case 4: /* +CGREG: <n>, <stat>, <lac>, <cid>, <networkType> */
            err = at_tok_nextint(&line, &skip);
            if (err < 0) goto error;
            err = at_tok_nextint(&line, &skip);
            if (err < 0) goto error;
            err = at_tok_nexthexint(&line, &lac);
            if (err < 0) goto error;
            err = at_tok_nexthexint(&line, &cid);
            if (err < 0) goto error;
            err = at_tok_nexthexint(&line, &skip);
            if (err < 0) goto error;
            count = 4;
            break;
        default:
            goto error;
    }


    at_response_free(atResponse);

    err = at_send_command_singleline("AT+CSQ", "+CSQ:", &atResponse);
    if (err != AT_NOERROR)
        goto error;

    line = atResponse->p_intermediates->line;

    err = at_tok_start(&line);
    if (err < 0) goto error;

    err = at_tok_nextint(&line,&rssi);
    if (err < 0) goto error;

    err = at_tok_nextint(&line, &ber);
    if (err < 0)
        goto error;

    /* Dump the current cell information */
    p_cellIds[0].cid = (char*) alloca(16);
    sprintf(p_cellIds[0].cid, "%04x%04x", lac & 0xFFFF, cid & 0xFFFF );
    p_cellIds[0].rssi = rssi;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, pp_cellIds, sizeof(pp_cellIds));
    at_response_free(atResponse);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    at_response_free(atResponse);
}

static pthread_t s_tid_queueRunner;


/** Do post- SIM ready initialization. */
static void onSIMReady(void *p)
{
    int err = 0;
    (void) p;

    /* Check if ME is ready to set preferred message storage */
    checkMessageStorageReady(NULL);

    /* Subscribe to network registration events.
     *  n = 2 - Enable network registration and location information
     *          unsolicited result code +CREG: <stat>[,<lac>,<ci>]
     */
    err = at_send_command("AT+CREG=2");
    if (err != AT_NOERROR) {
        /* Some handsets -- in tethered mode -- don't support CREG=2. */
        at_send_command("AT+CREG=1");
    }
	
	/* Enable proactive network registration notifications */
	at_send_command("AT+CGREG=2");
	
	/* Extended Cellular result codes enabled */
    at_send_command("AT+CRC=1");

	/* Calling line identification enabled */
	at_send_command("AT+CLIP=1");
	
	/* Call waiting enabled */
    at_send_command("AT+CCWA=1");
	
    /* Alternating voice/data off */
	at_send_command("AT+CMOD=0");
	
	/*  +CSSU unsolicited supp service notifications */
	at_send_command("AT+CSSN=0,1");

	/*  Connected line identification On*/
	at_send_command("AT+COLP=1");

	/*
	 * Ensure that the modem is using GSM character set and not IRA,
	 * otherwise weirdness with umlauts and other non-ASCII characters
	 * can result
	 */ 
    at_send_command("AT+CSCS=\"GSM\"");

	/*  USSD unsolicited */
	at_send_command("AT+CUSD=1");

    /* Configure Short Message (SMS) Format
     *  mode = 0 - PDU mode.
     */
    at_send_command("AT+CMGF=0");

	/* Disable boot messages */
	at_send_command("AT^BOOT=0,0");
	
	/* And flow messages */
	at_send_command("AT^DSFLOWRPT=0");
	
    /* Select SMS type */
    at_send_command_singleline("AT+CSMS=0", "+CSMS:", NULL);

    /* Always send SMS messages directly to the TE */
    at_send_command("AT+CNMI=2,1,2,2,0");
	
    /* Enable unsolicited RSSI reporting */
    at_send_command("AT^CURC=1");
	
}

static void requestNotSupported(RIL_Token t, int request)
{
    D("Request %d is unsupported", request);
    RIL_onRequestComplete(t, RIL_E_REQUEST_NOT_SUPPORTED, NULL, 0);
    return;
}


/*** Callback methods from the RIL library to us ***/

static void processRequest (int request, void *data, size_t datalen, RIL_Token t)
{
    D("%s() %s", __func__, requestToString(request));

    /*
     * These commands won't accept RADIO_NOT_AVAILABLE, so we just return
     * GENERIC_FAILURE if we're not in SIM_STATE_READY.
     */
    RIL_RadioState radio_state = getRadioState();

    if (radio_state != RADIO_STATE_SIM_READY &&
            (request == RIL_REQUEST_WRITE_SMS_TO_SIM ||
             request == RIL_REQUEST_DELETE_SMS_ON_SIM)) {
        RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
        return;
    }

    /* Ignore all requests except RIL_REQUEST_GET_SIM_STATUS
     * when RADIO_STATE_UNAVAILABLE.
     */
    if (radio_state == RADIO_STATE_UNAVAILABLE
            && request != RIL_REQUEST_GET_SIM_STATUS) {
        RIL_onRequestComplete(t, RIL_E_RADIO_NOT_AVAILABLE, NULL, 0);
        return;
    }

    /* Ignore all non-power requests when RADIO_STATE_OFF
     * (except RIL_REQUEST_GET_SIM_STATUS and a few more).
     */
    if ((radio_state == RADIO_STATE_OFF || radio_state == RADIO_STATE_SIM_NOT_READY) &&
           !(request == RIL_REQUEST_RADIO_POWER ||
             request == RIL_REQUEST_GET_SIM_STATUS ||
             request == RIL_REQUEST_STK_GET_PROFILE ||
             request == RIL_REQUEST_STK_SET_PROFILE ||
             request == RIL_REQUEST_REPORT_STK_SERVICE_IS_RUNNING ||
             request == RIL_REQUEST_GET_IMEISV ||
             request == RIL_REQUEST_GET_IMEI ||
             request == RIL_REQUEST_BASEBAND_VERSION ||
             request == RIL_REQUEST_SCREEN_STATE)) {
        RIL_onRequestComplete(t, RIL_E_RADIO_NOT_AVAILABLE, NULL, 0);
        return;
    }

    /* Don't allow radio operations when sim is absent or locked! */
    if (radio_state == RADIO_STATE_SIM_LOCKED_OR_ABSENT &&
           !(request == RIL_REQUEST_ENTER_SIM_PIN ||
             request == RIL_REQUEST_ENTER_SIM_PUK ||
             request == RIL_REQUEST_ENTER_SIM_PIN2 ||
             request == RIL_REQUEST_ENTER_SIM_PUK2 ||
             request == RIL_REQUEST_ENTER_NETWORK_DEPERSONALIZATION ||
             request == RIL_REQUEST_GET_SIM_STATUS ||
             request == RIL_REQUEST_RADIO_POWER ||
             request == RIL_REQUEST_GET_IMEISV ||
             request == RIL_REQUEST_GET_IMEI ||
             request == RIL_REQUEST_BASEBAND_VERSION ||
             request == RIL_REQUEST_GET_CURRENT_CALLS)) {
        RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
        return;
    }


    switch (request) {
        case RIL_REQUEST_GET_CURRENT_CALLS:
            if (radio_state == RADIO_STATE_SIM_LOCKED_OR_ABSENT)
                RIL_onRequestComplete(t, RIL_E_RADIO_NOT_AVAILABLE, NULL, 0);
            else
                requestGetCurrentCalls(data, datalen, t);
            break;

        case RIL_REQUEST_SCREEN_STATE:
            requestScreenState(data, datalen, t);
            /* Trigger a rehash of network values, just to be sure. */
            if (((int *)data)[0] == 1)
                RIL_onUnsolicitedResponse(
                                   RIL_UNSOL_RESPONSE_VOICE_NETWORK_STATE_CHANGED,
                                   NULL, 0);
            break;

        /* Data Call Requests */
        case RIL_REQUEST_SETUP_DATA_CALL:
            requestSetupDefaultPDP(data, datalen, t);
            break;
        case RIL_REQUEST_DEACTIVATE_DATA_CALL:
            requestDeactivateDefaultPDP(data, datalen, t);
            break;
        case RIL_REQUEST_LAST_DATA_CALL_FAIL_CAUSE:
            requestLastPDPFailCause(t);
            break;
        case RIL_REQUEST_DATA_CALL_LIST:
            requestPDPContextList(t);
            break;

        /* SMS Requests */
        case RIL_REQUEST_SEND_SMS:
            requestSendSMS(data, datalen, t);
            break;
        case RIL_REQUEST_SEND_SMS_EXPECT_MORE:
            requestSendSMSExpectMore(data, datalen, t);
            break;
        case RIL_REQUEST_WRITE_SMS_TO_SIM:
            requestWriteSmsToSim(data, datalen, t);
            break;
        case RIL_REQUEST_DELETE_SMS_ON_SIM:
            requestDeleteSmsOnSim(data, datalen, t);
            break;
        case RIL_REQUEST_GET_SMSC_ADDRESS:
            requestGetSMSCAddress(t);
            break;
        case RIL_REQUEST_SET_SMSC_ADDRESS:
            requestSetSMSCAddress(data, datalen, t);
            break;
        case RIL_REQUEST_REPORT_SMS_MEMORY_STATUS:
            requestSmsStorageFull(data, datalen, t);
            break;
        case RIL_REQUEST_SMS_ACKNOWLEDGE:
            requestSMSAcknowledge(data, datalen, t);
            break;
        case RIL_REQUEST_GSM_GET_BROADCAST_SMS_CONFIG:
            requestGSMGetBroadcastSMSConfig(t);
            break;
        case RIL_REQUEST_GSM_SET_BROADCAST_SMS_CONFIG:
            requestGSMSetBroadcastSMSConfig(data, datalen, t);
            break;
        case RIL_REQUEST_GSM_SMS_BROADCAST_ACTIVATION:
            requestGSMSMSBroadcastActivation(data, datalen, t);
            break;

        /* SIM Handling Requests */
        case RIL_REQUEST_SIM_IO:
            requestSIM_IO(data, datalen, t);
            break;
        case RIL_REQUEST_GET_SIM_STATUS:
            requestGetSimStatus(t);
            break;
        case RIL_REQUEST_ENTER_SIM_PIN:
        case RIL_REQUEST_ENTER_SIM_PUK:
        case RIL_REQUEST_ENTER_SIM_PIN2:
        case RIL_REQUEST_ENTER_SIM_PUK2:
            requestEnterSimPin(data, datalen, t, request);
            break;
        case RIL_REQUEST_CHANGE_SIM_PIN:
            requestChangePassword(data, datalen, t, "SC", request);
            break;
        case RIL_REQUEST_CHANGE_SIM_PIN2:
            requestChangePassword(data, datalen, t, "P2", request);
            break;
        case RIL_REQUEST_QUERY_FACILITY_LOCK:
            requestQueryFacilityLock(data, datalen, t);
            break;
        case RIL_REQUEST_SET_FACILITY_LOCK:
            requestSetFacilityLock(data, datalen, t);
            break;

        /* Network Requests */
        case RIL_REQUEST_ENTER_NETWORK_DEPERSONALIZATION:
            requestEnterSimPin(data, datalen, t, request);
            break;
        case RIL_REQUEST_QUERY_NETWORK_SELECTION_MODE:
            requestQueryNetworkSelectionMode(t);
            break;
        case RIL_REQUEST_SET_NETWORK_SELECTION_AUTOMATIC:
            requestSetNetworkSelectionAutomatic(t);
            break;
        case RIL_REQUEST_SET_NETWORK_SELECTION_MANUAL:
            requestSetNetworkSelectionManual(data, datalen, t);
            break;
        case RIL_REQUEST_QUERY_AVAILABLE_NETWORKS:
            requestQueryAvailableNetworks(t);
            break;
        case RIL_REQUEST_SET_PREFERRED_NETWORK_TYPE:
            requestSetPreferredNetworkType(data, datalen, t);
            break;
        case RIL_REQUEST_GET_PREFERRED_NETWORK_TYPE:
            requestGetPreferredNetworkType(t);
            break;
        case RIL_REQUEST_VOICE_REGISTRATION_STATE:
            requestRegistrationState(t);
            break;
        case RIL_REQUEST_DATA_REGISTRATION_STATE:
            requestGprsRegistrationState(t);
            break;

        /* OEM */
        case RIL_REQUEST_OEM_HOOK_RAW:
            // echo back data
            requestOEMHookRaw(data, datalen, t);
            break;
        case RIL_REQUEST_OEM_HOOK_STRINGS:
            requestOEMHookStrings(data, datalen, t);
            break;

        /* Misc */
        case RIL_REQUEST_SIGNAL_STRENGTH:
            requestSignalStrength(t);
            break;
        case RIL_REQUEST_OPERATOR:
            requestOperator(t);
            break;
        case RIL_REQUEST_RADIO_POWER:
            requestRadioPower(data, datalen, t);
            break;
        case RIL_REQUEST_GET_IMSI:
            requestGetIMSI(t);
            break;
        case RIL_REQUEST_GET_IMEI:
            requestGetIMEI(t);
            break;
        case RIL_REQUEST_GET_IMEISV:
            requestGetIMEISV(t);
            break;
        case RIL_REQUEST_DEVICE_IDENTITY:
            requestDeviceIdentity(t);
            break;
        case RIL_REQUEST_BASEBAND_VERSION:
            requestBasebandVersion(t);
            break;

		/* Band control */
		case RIL_REQUEST_SET_BAND_MODE:
			requestSetBandMode(data, datalen, t);
			break;
		case RIL_REQUEST_QUERY_AVAILABLE_BAND_MODE:
			requestQueryAvailableBandMode(data, datalen, t);
			break;

        /* SIM Application Toolkit */
        case RIL_REQUEST_STK_SEND_TERMINAL_RESPONSE:
            requestStkSendTerminalResponse(data, datalen, t);
            break;
        case RIL_REQUEST_STK_SEND_ENVELOPE_COMMAND:
            requestStkSendEnvelopeCommand(data, datalen, t);
            break;
        case RIL_REQUEST_STK_GET_PROFILE:
            requestStkGetProfile(t);
            break;
        case RIL_REQUEST_STK_SET_PROFILE:
            requestStkSetProfile(data, datalen, t);
            break;
        case RIL_REQUEST_REPORT_STK_SERVICE_IS_RUNNING:
            requestReportStkServiceIsRunning(t);
            break;

        /* Misc ops */
        case RIL_REQUEST_DIAL:
            requestDial(data, datalen, t);
            break;
        case RIL_REQUEST_HANGUP:
            requestHangup(data, datalen, t);
            break;
        case RIL_REQUEST_HANGUP_WAITING_OR_BACKGROUND:
            requestHangupWaitingOrBackground(t);
            break;
        case RIL_REQUEST_HANGUP_FOREGROUND_RESUME_BACKGROUND:
            requestHangupForegroundResumeBackground(t);
            break;
        case RIL_REQUEST_SWITCH_WAITING_OR_HOLDING_AND_ACTIVE:
            requestSwitchWaitingOrHoldingAndActive(t);
            break;
        case RIL_REQUEST_ANSWER:
            requestAnswer(t);
            break;
        case RIL_REQUEST_CONFERENCE:
            requestConference(t);
            break;
        case RIL_REQUEST_UDUB:
            requestUDUB(t);
            break;
        case RIL_REQUEST_SEPARATE_CONNECTION:
            requestSeparateConnection(data, datalen, t);
            break;
        case RIL_REQUEST_SET_MUTE:
            requestSetMute(data, datalen, t);
            break;
        case RIL_REQUEST_GET_MUTE:
            requestGetMute(t);
            break;
        case RIL_REQUEST_DTMF:
            requestDTMF(data, datalen, t);
            break;
        case RIL_REQUEST_DTMF_STOP:
            requestDtmfStop(t);
            break;
        case RIL_REQUEST_DTMF_START:
            requestDtmfStart(data, datalen, t);
            break;
        case RIL_REQUEST_CANCEL_USSD:
            requestCancelUSSD(t);
            break;
        case RIL_REQUEST_QUERY_CALL_WAITING:
            requestQueryCallWaiting(data, datalen, t);
            break;
        case RIL_REQUEST_SET_CALL_WAITING:
            requestSetCallWaiting(data, datalen, t);
            break;
        case RIL_REQUEST_QUERY_CALL_FORWARD_STATUS:
            requestQueryCallForwardStatus(t);
            break;
        case RIL_REQUEST_SET_CALL_FORWARD:
            requestSetCallForward(data, t);
            break;
        case RIL_REQUEST_GET_CLIR:
            requestGetCLIR(data, datalen, t);
            break;
        case RIL_REQUEST_SET_CLIR:
            requestSetCLIR(data, datalen, t);
            break;
        case RIL_REQUEST_SEND_USSD:
            requestSendUSSD(data, datalen, t);
            break;
        case RIL_REQUEST_CHANGE_BARRING_PASSWORD:
            requestChangeBarringPassword(data, datalen, t);
            break;
        case RIL_REQUEST_QUERY_CLIP:
            requestQueryCLIP(t);
            break;
        case RIL_REQUEST_RESET_RADIO:
            requestResetRadio(t);
            break;
        case RIL_REQUEST_SET_SUPP_SVC_NOTIFICATION:
            requestSetSuppSVCNotification(data, datalen, t);
            break;
        case RIL_REQUEST_EXPLICIT_CALL_TRANSFER:
            requestExplicitCallTransfer(t);
            break;
        case RIL_REQUEST_SET_LOCATION_UPDATES:
            requestSetLocationUpdates(data, datalen, t);
            break;
        case RIL_REQUEST_LAST_CALL_FAIL_CAUSE:
            requestLastFailCause(t);
            break;
        case 503: //GET_SIM_TYPES
            RIL_onRequestComplete(t, RIL_E_SUCCESS, 0, sizeof(int));
            break;
        case 504: //GET_PB_ENTRIES_LENGTH
            {
                int response[6];
                response[0]=1;
                response[1]=0;
                response[2]=0;
                response[3]=0;
                response[4]=0;
                response[5]=0;
                RIL_onRequestComplete(t, RIL_E_SUCCESS, response, sizeof(response));
            }
            break;

        case 525: //GET_SUBSCRIBER_NUMBER
            break;

        case RIL_REQUEST_CDMA_SET_SUBSCRIPTION_SOURCE:
            requestCdmaSubscription(data, datalen, t);
            break;

        case RIL_REQUEST_GET_NEIGHBORING_CELL_IDS:
            requestNeighboringCellIds(data, datalen, t);
            break;

        default:
            ALOGW("FIXME: Unsupported request logged: %s",
                 requestToString(request));
            requestNotSupported(t, request);
            break;
    }
}

/**
 * Call from RIL to us to make a RIL_REQUEST
 *
 * Must be completed with a call to RIL_onRequestComplete()
 *
 * RIL_onRequestComplete() may be called from any thread, before or after
 * this function returns.
 *
 * Will always be called from the same thread, so returning here implies
 * that the radio is ready to process another command (whether or not
 * the previous command has completed).
 */

/**
 * Call from RIL to us to make a RIL_REQUEST.
 *
 * Must be completed with a call to RIL_onRequestComplete().
 */
static void onRequest(int request, void *data, size_t datalen, RIL_Token t)
{
    RILRequest *r;
    RequestQueue *q = &s_requestQueue;
    int err;

    r = (RILRequest *) malloc(sizeof(RILRequest));
    memset(r, 0, sizeof(RILRequest));

    /* Formulate a RILRequest and put it in the queue. */
    r->request = request;
    r->data = dupRequestData(request, data, datalen);
    r->datalen = datalen;
    r->token = t;

    if ((err = pthread_mutex_lock(&q->queueMutex)) != 0)
        ALOGE("%s() failed to take queue mutex: %s!", __func__, strerror(err));

    /* Queue empty, just throw r on top. */
    if (q->requestList == NULL) {
        q->requestList = r;
    } else {
        RILRequest *l = q->requestList;
        while (l->next != NULL)
            l = l->next;

        l->next = r;
    }

    if ((err = pthread_cond_broadcast(&q->cond)) != 0)
        ALOGE("%s() failed to broadcast queue update: %s!",
            __func__, strerror(err));

    if ((err = pthread_mutex_unlock(&q->queueMutex)) != 0)
        ALOGE("%s() failed to release queue mutex: %s!",
            __func__, strerror(err));
}

/**
 * Call from RIL to us to find out whether a specific request code
 * is supported by this implementation.
 *
 * Return 1 for "supported" and 0 for "unsupported".
 *
 * Currently just stubbed with the default value of one. This is currently
 * not used by android, and therefore not implemented here. We return
 * RIL_E_REQUEST_NOT_SUPPORTED when we encounter unsupported requests.
 */
static int onSupports(int requestCode)
{
    (void) requestCode;
    ALOGI("onSupports() called!");

    return 1;
}

/**
 * onCancel() is currently stubbed, because android doesn't use it and
 * our implementation will depend on how a cancellation is handled in
 * the upper layers.
 */
static void onCancel(RIL_Token t)
{
    (void) t;
    ALOGI("onCancel() called!");
}

static const char * getVersion(void)
{
    return RIL_VERSION_STRING;
}

static void sendTime(void *p)
{
    time_t t;
    struct tm tm;
    char str[20];
    char tz[6];
    int num[4];
    int tzi;
    int i;
    (void) p;

    tzset();
    t = time(NULL);

    if (!(localtime_r(&t, &tm)))
        return;
    if (!(strftime(tz, 12, "%z", &tm)))
        return;

    for (i = 0; i < 4; i++)
        num[i] = tz[i+1] - '0';

    /* convert timezone hours to timezone quarters of hours */
    tzi = (num[0] * 10 + num[1]) * 4 + (num[2] * 10 + num[3]) / 15;
    strftime(str, 20, "%y/%m/%d,%T", &tm);
    at_send_command("AT+CCLK=\"%s%c%02d\"", str, tz[0], tzi);
}

/**
 * Initialize everything
 */
static char initializeCommon(void)
{
    int err = 0;
    unsigned int i;
    static const char* const initcmd[] = {

        /*  echo off */
        "ATE0",

        /*  Send verbose results */
        "ATQ0V1",

        /*  No auto-answer */
        "ATS0=0",
	
        /*  Extended errors without textual decriptions */
        "AT+CMEE=1",

        /* Enable RF */
        "AT^RFSWITCH=1"
    };

    D("%s()", __func__);

    setRadioState(RADIO_STATE_OFF);

    if (at_handshake() < 0) {
        LOG_FATAL("Handshake failed!");
        return 1;
    }
	
    for (i=0; i < sizeof(initcmd) / sizeof(initcmd[0]); i++) {
        err = at_send_command(initcmd[i]);
        if (err != AT_NOERROR) {
            ALOGE("Failed sending command '%s'",initcmd[i]);
        }
    }

	/* TE-MS handshake function */
	//"AT^HS=0,0",

	
    /* Assume radio is off on error. */
    if (isRadioOn() > 0)
        setRadioState(RADIO_STATE_SIM_NOT_READY);

    return 0;
}

/**
 * Called by atchannel when an unsolicited line appears
 * This is called on atchannel's reader thread. AT commands may
 * not be issued here
 */
static void onUnsolicited (const char *s, const char *sms_pdu)
{
    /* Ignore unsolicited responses until we're initialized.
       This is OK because the RIL library will poll for initial state. */
    if (getRadioState() == RADIO_STATE_UNAVAILABLE)
        return;

	/* Timezone */
    if (strStartsWith(s, "%CTZV:")
            || strStartsWith(s,"+CTZV:")
            || strStartsWith(s,"+CTZDST:")
            || strStartsWith(s,"+HTCCTZV:")
			|| strStartsWith(s,"^NWTIME:")
			) {
        unsolicitedNitzTime(s);
	
	/* Call status/start/end indicator */
    } else if (strStartsWith(s,"+CRING:")
            || strStartsWith(s,"RING")
			|| strStartsWith(s,"+CLIP")
            || strStartsWith(s,"NO CARRIER")
            || strStartsWith(s,"+CCWA")
			|| strStartsWith(s,"^ORIG:")
			|| strStartsWith(s,"^CONF:")
			|| strStartsWith(s,"^CONN:")
			|| strStartsWith(s,"^CEND:")
          ) {

		/* Start/stop audio tunnel if needed when a successful connection is done */
		if (   strStartsWith(s,"^ORIG:")) {
			enqueueRILEvent(startAudioTunnel, NULL, NULL);
			unsolicitedORIG(s);
		} else 
		if (   strStartsWith(s,"^CONF:")) {
			enqueueRILEvent(startAudioTunnel, NULL, NULL);
			unsolicitedCONF(s);
		} else 
		if (   strStartsWith(s,"^CONN:")) {
			enqueueRILEvent(startAudioTunnel, NULL, NULL);
			unsolicitedCONN(s);
		} else 
		if (strStartsWith(s,"^CEND:")) {
			/* This is the only fast way to detect when call was hang by the callee */
			enqueueRILEvent(stopAudioTunnel, NULL, NULL);
			unsolicitedCEND(s);
		}
		
		RIL_onUnsolicitedResponse (
                RIL_UNSOL_RESPONSE_CALL_STATE_CHANGED,
                NULL, 0);
        enqueueRILEvent(onDataCallListChanged, NULL, NULL);

	/* Data call status/start/end */
    } else if (strStartsWith(s,"^DCONN:") 
			|| strStartsWith(s,"^DEND:")) {

		RIL_onUnsolicitedResponse (
                RIL_UNSOL_RESPONSE_CALL_STATE_CHANGED,
                NULL, 0);
        enqueueRILEvent(onDataCallListChanged, NULL, NULL);
		
    } else if (strStartsWith(s,"^COMM:")) {
			   
		
    } else if (strStartsWith(s,"^RSSI:") ||
               strStartsWith(s,"%RSSI:")) {
        unsolicitedRSSI(s);
    } else if (strStartsWith(s,"^MODE:")) {
        unsolicitedMode(s);
    } else if (strStartsWith(s,"^SRVST:")) {
        unsolicitedSrvStatus(s);
    } else if (strStartsWith(s,"^SIMST:")) {
        unsolicitedSimStatus(s);
    } else if (strStartsWith(s,"+CREG:")
            || strStartsWith(s,"+CGREG:")) {
        RIL_onUnsolicitedResponse (
                RIL_UNSOL_RESPONSE_VOICE_NETWORK_STATE_CHANGED,
                NULL, 0);
        enqueueRILEvent(onDataCallListChanged, NULL, NULL);
    } else if (strStartsWith(s, "+CMT:")) {
        onNewSms(sms_pdu);
    } else if (strStartsWith(s, "+CBM:")) {
        onNewBroadcastSms(sms_pdu);
    } else if (strStartsWith(s, "+CMTI:")
			|| strStartsWith(s, "+CDSI:")) {
        onNewSmsOnSIM(s);
    } else if (strStartsWith(s, "+CDS:")) {
        onNewStatusReport(sms_pdu);
    } else if (strStartsWith(s, "+CGEV:")) {
        /* Really, we can ignore NW CLASS and ME CLASS events here,
         * but right now we don't since extranous
         * RIL_UNSOL_DATA_CALL_LIST_CHANGED calls are tolerated
         */
        /* can't issue AT commands here -- call on main thread */
        enqueueRILEvent(onDataCallListChanged, NULL, NULL);
    } else if (strStartsWith(s, "+CUSD:")) {
        unsolicitedUSSD(s);
    } else if (strStartsWith(s, "+CIEV: 7") ||
               strStartsWith(s, "Received SMS:") ||
			   strStartsWith(s, "^SMMEMFULL") ) {
        onNewSmsIndication();
    }
}

static void signalCloseQueues(void)
{
    unsigned int i;
    for (i = 0; i < (sizeof(s_requestQueues) / sizeof(RequestQueue *)); i++) {
        int err;
        RequestQueue *q = s_requestQueues[i];
        if ((err = pthread_mutex_lock(&q->queueMutex)) != 0)
            ALOGE("%s() failed to take queue mutex: %s",
                __func__, strerror(err));

        q->closed = 1;
        if ((err = pthread_cond_signal(&q->cond)) != 0)
            ALOGE("%s() failed to broadcast queue update: %s",
                __func__, strerror(err));

        if ((err = pthread_mutex_unlock(&q->queueMutex)) != 0)
            ALOGE("%s() failed to take queue mutex: %s", __func__,
                 strerror(err));
    }
}


/* Called on command or reader thread */
static void onATReaderClosed()
{
    ALOGI("AT channel closed\n");

    setRadioState (RADIO_STATE_UNAVAILABLE);
    signalCloseQueues();
    at_close();
}

/* Called on command thread */
static void onATTimeout()
{
    ALOGI("AT channel timeout; restarting..");
    /* Last resort, throw escape on the line, close the channel
       and hope for the best. */
    at_send_escape();

    setRadioState(RADIO_STATE_UNAVAILABLE);
    signalCloseQueues();

    /* TODO We may cause a radio reset here. */
}

static void usage(char *s)
{
#ifdef RIL_SHLIB
    fprintf(stderr, "generic-ril requires: -p <tcp port> or -d /dev/tty_device\n");
#else
    fprintf(stderr, "usage: %s [-p <tcp port>] [-d /dev/tty_device] [-v /dev/tty_device]\n", s);
    exit(-1);
#endif
}

struct queueArgs {
    int port;
    char * loophost;
    const char *device_path;
};

static int safe_read(int fd, char *buf, int count)
{
    int n;
    int i = 0;

    while (i < count) {
        n = read(fd, buf + i, count - i);
        if (n > 0)
            i += n;
        else if (!(n < 0 && errno == EINTR))
            return -1;
    }

    return count;
}

static void *queueRunner(void *param)
{
    int fd = -1;
    int ret = 0;
    int n;
    fd_set input;
    struct timespec timeout;
    int max_fd = -1;
    char start[MAX_BUF];
    struct queueArgs *queueArgs = (struct queueArgs *) param;
    struct RequestQueue *q = NULL;

    ALOGI("%s() starting!", __func__);

    for (;;) {
        fd = -1;
        max_fd = -1;
        while (fd < 0) {
            if (queueArgs->port > 0) {
                if (queueArgs->loophost)
                    fd = socket_network_client(queueArgs->loophost, queueArgs->port, SOCK_STREAM);
                else
                    fd = socket_loopback_client(queueArgs->port, SOCK_STREAM);
            } else if (queueArgs->device_path != NULL) {

                fd = open (queueArgs->device_path, O_RDWR);
                if ( fd >= 0 && !memcmp( queueArgs->device_path, "/dev/ttyUSB", 11 ) ) {

                    /* disable echo on serial ports */
                    struct termios  ios;
                    tcgetattr( fd, &ios );
                    ios.c_lflag = 0;  /* disable ECHO, ICANON, etc... */
                    tcsetattr( fd, TCSANOW, &ios );
                }
            }

            if (fd < 0) {
                ALOGE("%s() Failed to open AT channel %s (%s), retrying in %d.",
                    __func__, queueArgs->device_path,
                    strerror(errno), TIMEOUT_SEARCH_FOR_TTY);
                sleep(TIMEOUT_SEARCH_FOR_TTY);
                /* Never returns. */
            }
        }

        ret = at_open(fd, onUnsolicited);

        if (ret < 0) {
            ALOGE("%s() AT error %d on at_open", __func__, ret);
            at_close();
            continue;
        }

        at_set_on_reader_closed(onATReaderClosed);
        at_set_on_timeout(onATTimeout);

        q = &s_requestQueue;

        if (initializeCommon()) {
            ALOGE("%s() Failed to initialize common", __func__);
            at_close();
            continue;
        }

        q->closed = 0;
        at_make_default_channel();

        ALOGE("%s() Looping the requestQueue!", __func__);
        for (;;) {
            RILRequest *r;
            RILEvent *e;
            struct timespec ts;
            int err;

            memset(&ts, 0, sizeof(ts));

        if ((err = pthread_mutex_lock(&q->queueMutex)) != 0)
            ALOGE("%s() failed to take queue mutex: %s!",
                __func__, strerror(err));

            if (q->closed != 0) {
                ALOGW("%s() AT Channel error, attempting to recover..", __func__);
                if ((err = pthread_mutex_unlock(&q->queueMutex)) != 0)
                    ALOGE("Failed to release queue mutex: %s!", strerror(err));
                break;
            }

            while (q->closed == 0 && q->requestList == NULL &&
                q->eventList == NULL) {
                if ((err = pthread_cond_wait(&q->cond, &q->queueMutex)) != 0)
                    ALOGE("%s() failed broadcast queue cond: %s!",
                        __func__, strerror(err));
            }

            /* eventList is prioritized, smallest abstime first. */
            if (q->closed == 0 && q->requestList == NULL && q->eventList) {
                int err = 0;
                err = pthread_cond_timedwait(&q->cond, &q->queueMutex, &q->eventList->abstime);
                if (err && err != ETIMEDOUT)
                    ALOGE("%s() timedwait returned unexpected error: %s",
                __func__, strerror(err));
            }

            if (q->closed != 0) {
                if ((err = pthread_mutex_unlock(&q->queueMutex)) != 0)
                    ALOGE("%s(): Failed to release queue mutex: %s!",
                        __func__, strerror(err));
                continue; /* Catch the closed bit at the top of the loop. */
            }

            e = NULL;
            r = NULL;

            clock_gettime(CLOCK_MONOTONIC, &ts);

            if (q->eventList != NULL &&
                timespec_cmp(q->eventList->abstime, ts, < )) {
                e = q->eventList;
                q->eventList = e->next;
            }

            if (q->requestList != NULL) {
                r = q->requestList;
                q->requestList = r->next;
            }

            if ((err = pthread_mutex_unlock(&q->queueMutex)) != 0)
                ALOGE("%s(): Failed to release queue mutex: %s!",
                    __func__, strerror(err));

            if (e) {
                e->eventCallback(e->param);
                free(e);
            }

            if (r) {
                processRequest(r->request, r->data, r->datalen, r->token);
                freeRequestData(r->request, r->data, r->datalen);
                free(r);
            }
        }

        at_close();
        ALOGE("%s() Re-opening after close", __func__);
    }
    return NULL;
}

#ifdef RIL_SHLIB
const RIL_RadioFunctions *RIL_Init(const struct RIL_Env *env, int argc,
                                   char **argv)
{
    int opt;
    int port = -1;
    char *loophost = NULL;
    const char *ctrl_path = NULL;
    struct queueArgs *queueArgs;
    pthread_attr_t attr;

#if 0
	/* Logcat test */
	char local_ip[64];
	char dns1[64];
	char dns2[64];
	char gw[64];
	struct timeval from;
	int rr;
	gettimeofday(&from,NULL);
	rr = get_pppd_info(&from,local_ip, dns1, dns2, gw);
	ALOGD("result: %d",rr);
#endif

#if 0
	char * m = decodeUSSD("D9775D0E6286E7749050FEBECFD3EE33685A9ECFD36F37283D07BDCD201868297449CBED70DA9D769F414679B90C2287E96190FDCDAEB7CBA0F41C849BCD60B41C68297401");
	ALOGD("Decoded msg: '%s'",m);
	free(m);
#endif	

	memset(&sAudioChannel,0,sizeof(sAudioChannel));
	
	s_rilenv = env;
	
    /* By default, use USB1 as audio channel */
    strcpy(sAudioDevice,"/dev/ttyUSB1");

    D("%s() entering...", __func__);

    while (-1 != (opt = getopt(argc, argv, "z:n:i:p:d:v:"))) {
        switch (opt) {
            case 'z':
                loophost = optarg;
                D("%s() Using loopback host %s..", __func__, loophost);
                break;

            case 'p':
                port = atoi(optarg);
                if (port == 0) {
                    usage(argv[0]);
                    return NULL;
                }
                D("%s() Opening loopback port %d", __func__, port);
                break;

            case 'd':
                ctrl_path = optarg;
                D("%s() Opening tty device %s", __func__, ctrl_path);
                break;

            case 'v':
                strcpy(sAudioDevice,optarg);
                D("%s() Opening voice tty device %s", __func__, sAudioDevice);
                break;

			case 'n':
                rndis_iface_dev = optarg;
                D("%s() Using network interface %s as primary data channel.", __func__, rndis_iface_dev);
                break; 

			case 'i':
                ppp_iface_dev = optarg;
                D("%s() Using ppp interface %s as primary data channel.", __func__, ppp_iface_dev);
                break; 
				
            default:
                usage(argv[0]);
                return NULL;
        }
    }

    if (port < 0 && ctrl_path == NULL) {
        usage(argv[0]);
        return NULL;
    }

    queueArgs = (struct queueArgs*) malloc(sizeof(struct queueArgs));
    memset(queueArgs, 0, sizeof(struct queueArgs));

    queueArgs->device_path = ctrl_path;
    queueArgs->port = port;
    queueArgs->loophost = loophost;

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    pthread_create(&s_tid_queueRunner, &attr, queueRunner, queueArgs);

    return &s_callbacks;
}

#else /* RIL_SHLIB */
int main (int argc, char **argv)
{
    int opt;
    int port = -1;
    char *loophost = NULL;
    const char *ctrl_path = NULL;
    struct queueArgs *queueArgs;

	memset(&sAudioChannel,0,sizeof(sAudioChannel));
	
    /* By default, use USB1 as audio channel */
    strcpy(sAudioDevice,"/dev/ttyUSB1");

    D("%s() entering...", __func__);

    while (-1 != (opt = getopt(argc, argv, "z:n:i:p:d:v:"))) {
        switch (opt) {
            case 'z':
                loophost = optarg;
                D("%s() Using loopback host %s..", __func__, loophost);
                break;

            case 'p':
                port = atoi(optarg);
                if (port == 0) {
                    usage(argv[0]);
                    return 0;
                }
                D("%s() Opening loopback port %d", __func__, port);
                break;

            case 'd':
                ctrl_path = optarg;
                D("%s() Opening tty device %s", __func__, ctrl_path);
                break;

            case 'v':
                strcpy(sAudioDevice,optarg);
                D("%s() Opening voice tty device %s", __func__, sAudioDevice);
                break;

			case 'n':
                rndis_iface_dev = optarg;
                D("%s() Using network interface %s as primary data channel.", __func__, rndis_iface_dev);
                break; 

			case 'i':
                ppp_iface_dev = optarg;
                D("%s() Using ppp interface %s as primary data channel.", __func__, ppp_iface_dev);
                break; 
				
            default:
                usage(argv[0]);
                return 0;
        }
    }

    if (port < 0 && ctrl_path == NULL) {
        usage(argv[0]);
        return 0;
    }

    queueArgs = (struct queueArgs*) malloc(sizeof(struct queueArgs));
    memset(queueArgs, 0, sizeof(struct queueArgs));

    queueArgs->device_path = ctrl_path;
    queueArgs->port = port;
    queueArgs->loophost = loophost;

    RIL_register(&s_callbacks);

    queueRunner(queueArgs);

    return 0;
}
#endif
