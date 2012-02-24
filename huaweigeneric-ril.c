/* //device/system/htcgeneric-ril/htcgeneric-ril.c
 **
 ** Copyright 2006, The Android Open Source Project
 ** Copyright 2012 Eduardo Jos[e Tagle <ejtagle@tutopia.com>
 ** Copyright 2012 Artem Makhutov <artem@makhutov.org>
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

/*
AT^CURC=0
ATQ0V1
 
^MODE: 
^RSSI: 
^SRVST:  
+CCCM:  
^CONN: 
^CEND:   
+CDSI: 
^SMMEMFULL: 
+CSSU:  
^DCONN 
^DEND 
^SIMST      ¡
+CRING: 
AT^SYSCFG=2,3,%s,1,2 
^RSSI 
^SMMEMFULL
*/

/*
AT^SYSCFG=2,1,3FFFFFFF,1,2 for GPRS/EDGE Preferred
AT^SYSCFG=2,2,3FFFFFFF,1,2 for 3G Preferred
AT^SYSCFG=13,1,3FFFFFFF,1,2 for GPRS/EDGE Only
AT^SYSCFG=14,2,3FFFFFFF,1,2 for 3G Only

The third parameter, 0x3FFFFFFF tells the card to use all bands. A value of 0x400380 here means GSM900/1800/WCDMA2100 only and a value of 0x200000 here means GSM1900 only.

I don't know what the final "1,2" parameter is for. But for some it has to be "2,4" instead for some reason.

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

I have a linux machine, Huawei E1550 modem and a task. The task is to call to some number and beep there with DTMF. Hours of googling and reading documentations gave this:
1. Sending DTMF tones with AT commands is possible if modem is in voice mode (AT+FCLASS=? : 8). Command is AT+VTS=<DTMF> ,<DTMF> is 0-9,*,#,A-D.
2. This device (E1550) for AT+FCLASS=? returns 0-1, so 8 is not possible.
3. Also somewhere was said that ATDT<DTMF> is possible. This neither works.
4. Mobile partner under windows (with some fixes in both *.xml files in \config\ folder) _can_ send DTMF, so it's possible somehow.

So the question is: can i send DTMF tones via E1550 using AT commands running linux and how?

Sorry for my english and messy post.
Thanks in advance!
chip89
01-17-2011, 12:38
Thanks for help... 
If anyone is interested, sending DTMF signals in modem-mode (+FCLASS=0) is possible. But it is device-dependent feature.
For Huawei E1550 it is AT^DTMF=<n> command.

Good luck.

----
That need to be tested but voice activation should work with:
AT^U2DIAG=5

then the modem become 0x12d1:0x140c a flash drive.

then I plug in on my Linux Ubuntu computer, use minicom and type AT^U2DIAG=0 to deactivate the damn flash drive.

Maybe it's better to first type AT^U2DIAG=0 and then AT^U2DIAG=5 ...

Regards, Phil.
*/
 
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
#include "gsm.h"
#include "requestdatahandler.h"
#include <getopt.h>
#include <sys/socket.h>
#include <cutils/sockets.h>
#include <termios.h>
#include <cutils/properties.h>
#include <stdbool.h>

#define LOG_NDEBUG 0
#define LOG_TAG "RIL"
#include <utils/Log.h>

#define RIL_VERSION_STRING "MBM Huawei-ril 1.0.0.0"
#define MAX_AT_RESPONSE 0x1000

#define timespec_cmp(a, b, op)         \
        ((a).tv_sec == (b).tv_sec    \
        ? (a).tv_nsec op (b).tv_nsec \
        : (a).tv_sec op (b).tv_sec)

#define TIMEOUT_SEARCH_FOR_TTY 5 /* Poll every Xs for the port*/
#define TIMEOUT_EMRDY 10 /* Module should respond at least within 10s */
#define MAX_BUF 1024

/* pathname returned from RIL_REQUEST_SETUP_DATA_CALL / RIL_REQUEST_SETUP_DEFAULT_PDP */
#define PPP_TTY_PATH "ppp0"


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
static int killConn(char * cid);
static int wait_for_property(const char *name, const char *desired_value, int maxwait);
static void checkMessageStorageReady(void *p);
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

static RIL_RadioState sState = RADIO_STATE_UNAVAILABLE;
static pthread_mutex_t s_state_mutex = PTHREAD_MUTEX_INITIALIZER;
static const struct timespec TIMEVAL_SIMPOLL = {1,0};

const struct RIL_Env *s_rilenv;
#define RIL_onRequestComplete(t, e, response, responselen) s_rilenv->OnRequestComplete(t,e, response, responselen)
#define RIL_onUnsolicitedResponse(a,b,c) s_rilenv->OnUnsolicitedResponse(a,b,c)

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
    .queueMutex = PTHREAD_MUTEX_INITIALIZER,
    .cond = PTHREAD_COND_INITIALIZER,
    .requestList = NULL,
    .eventList = NULL,
    .enabled = 1,
    .closed = 1
};

static RequestQueue *s_requestQueues[] = {
    &s_requestQueue
};

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

    RILEvent *e = malloc(sizeof(RILEvent));
    memset(e, 0, sizeof(RILEvent));

    e->eventCallback = callback;
    e->param = param;

    if (relativeTime == NULL) {
        relativeTime = alloca(sizeof(struct timespec));
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
        LOGE("%s() failed to take queue mutex: %s!", __func__, strerror(err));

    if (q->eventList == NULL)
        q->eventList = e;
    else {
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
        LOGE("%s() failed to take broadcast queue update: %s!",
            __func__, strerror(err));

    if ((err = pthread_mutex_unlock(&q->queueMutex)) != 0)
        LOGE("%s() failed to release queue mutex: %s!",
            __func__, strerror(err));

}

/** returns 1 if on, 0 if off, and -1 on error */
static int isRadioOn()
{
	ATResponse *atResponse = NULL;
	int err;
	char *line;
	char ret;

	err = at_send_command_singleline("AT+CFUN?", "+CFUN:", &atResponse);

	if (err != AT_NOERROR) {
		// assume radio is off
		goto error;
	}

	line = atResponse->p_intermediates->line;

	err = at_tok_start(&line);
	if (err < 0) goto error;

	err = at_tok_nextbool(&line, &ret);
	if (err < 0) goto error;

	at_response_free(atResponse);

	return (int)ret;

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
        LOGE("%s() failed to take state mutex: %s!", __func__, strerror(err));

    oldState = sState;

    LOGI("%s() oldState=%s newState=%s", __func__, radioStateToString(oldState),
         radioStateToString(newState));

    sState = newState;

    if ((err = pthread_mutex_unlock(&s_state_mutex)) != 0)
        LOGE("%s() failed to release state mutex: %s!", __func__, strerror(err));

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
 
 
/** Returns one of SIM_*. Returns SIM_NOT_READY on error. */
static SIM_Status getSIMStatus(void)
{
    ATResponse *atresponse = NULL;
    int err;
    SIM_Status ret = SIM_ABSENT;
    char *cpinLine;
    char *cpinResult;

    if (getRadioState() == RADIO_STATE_OFF ||
        getRadioState() == RADIO_STATE_UNAVAILABLE) {
        return SIM_NOT_READY;
    }

    err = at_send_command_singleline("AT+CPIN?", "+CPIN:", &atresponse);

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

    cpinLine = atresponse->p_intermediates->line;
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
    at_response_free(atresponse);
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
        LOGI("SIM_NOT_READY, poll for sim state.");
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
			LOGE("Unsupported call state: %d\n",state);
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
	LOGE("invalid CLCC line\n");
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

	LOGI("Calls=%d,Valid=%d\n",countCalls,countValidCalls);

	RIL_onRequestComplete(t, RIL_E_SUCCESS, pp_calls,
			countValidCalls * sizeof (RIL_Call *));

	at_response_free(atResponse);

	if (needRepoll)	{
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

	if(screenState == 1)
	{
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
	LOGE("ERROR: requestScreenState failed");
	RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
}


/* wait for a propertyvalue change */
static int wait_for_property(const char *name, const char *desired_value, int maxwait)
{
    char value[PROPERTY_VALUE_MAX] = {'\0'};
    int maxnaps = maxwait / 1;

    if (maxnaps < 1) {
        maxnaps = 1;
    }

    while (maxnaps-- > 0) {
        usleep(1000000);
        if (property_get(name, value, NULL)) {
            if (desired_value == NULL || 
                    strcmp(value, desired_value) == 0) {
                return 0;
            }
        }
    }
    return -1; /* failure */
}

static int killConn(char * cid)
{
	int err;
	int fd;
	int i=0;

	LOGD("killConn");

    property_set("ctl.stop", "pppd_gprs");
    if (wait_for_property("init.svc.pppd_gprs", "stopped", 10) < 0) {
        goto error;
    }

    LOGD("killall pppd finished");

	err = at_send_command("AT+CGACT=0,%s", cid);
	if (err != AT_NOERROR) 
		goto error;
		
	at_send_command("ATH");
	return 0;

error:
	return -1;
}


//static char userPassStatic[512] = "preload";

static void requestSetupDefaultPDP(void *data, size_t datalen, RIL_Token t)
{
	const char *apn;
	char *user = NULL;
	char *pass = NULL;
	char userpass[512];
	int err;
	int fd, pppstatus,i;
	FILE *pppconfig;
	size_t cur = 0;
	ssize_t written, rlen;
	char status[32] = {0};
	char *buffer;
	long buffSize, len;
	int retry = 10;
	
	int n = 1;
	RIL_Data_Call_Response_v6 responses;
	char ppp_dnses[(PROPERTY_VALUE_MAX * 2) + 3] = {'\0'};
	char ppp_local_ip[PROPERTY_VALUE_MAX] = {'\0'};
	char ppp_dns1[PROPERTY_VALUE_MAX] = {'\0'};
	char ppp_dns2[PROPERTY_VALUE_MAX] = {'\0'};
	char ppp_gw[PROPERTY_VALUE_MAX] = {'\0'};
	
	apn = ((const char **)data)[2];
	user = ((char **)data)[3];
	if(user != NULL)
	{
		if (strlen(user)<2)
			user = "dummy";
	} else
		user = "dummy";

	pass = ((char **)data)[4];
	if(pass != NULL)
	{
		if (strlen(pass)<2)
			pass = "dummy";
	} else
		pass = "dummy";

	LOGD("requesting data connection to APN '%s'\n", apn);

	//Make sure there is no existing connection or pppd instance running
	if(killConn("1") < 0) {
		LOGE("killConn Error!\n");
		goto error;
	}
	
	/* Define the PDP context */
	err = at_send_command("AT+CGDCONT=1,\"IP\",\"%s\",,0,0", apn);
	
	/* Set required QoS params to default */
	err = at_send_command("AT+CGQREQ=1"); 
	
	/* Set minimum QoS params to default */
	err = at_send_command("AT+CGQMIN=1");
	
	/* packet-domain event reporting */
	err = at_send_command("AT+CGEREP=1,0"); 
	
	/* Hangup anything that's happening there now */
	err = at_send_command("AT+CGACT=0,1");
	
	/* Start data on PDP context 1 */
	err = at_send_command("ATD*99***1#");	
	if (err != AT_NOERROR) {
		goto error;
	}
	sleep(2); //Wait for the modem to finish

	//set up the pap/chap secrets file
	sprintf(userpass, "%s * %s", user, pass);
	
	/* start the gprs pppd */
	// system("/system/bin/pppd /dev/ttyUSB0 115200 nocrtscts usepeerdns debug ipcp-accept-local ipcp-accept-remote defaultroute");
	property_set("ctl.start", "pppd_gprs");
	if (wait_for_property("init.svc.pppd_gprs", "running", 10) < 0) {
        	goto error;
	}

	sleep(2); // Allow time for ip-up to complete

	if (wait_for_property("net.ppp0.local-ip", NULL, 10) < 0) {
		LOGE("Timeout waiting net.ppp0.local-ip - giving up!\n");
		goto error;
	}

	property_get("net.ppp0.local-ip", ppp_local_ip, NULL);
	property_get("net.ppp0.dns1", ppp_dns1, NULL);
	property_get("net.ppp0.dns2", ppp_dns2, NULL);
	property_get("net.ppp0.gw", ppp_gw, NULL);
	sprintf(ppp_dnses, "%s %s", ppp_dns1, ppp_dns2);

	LOGI("Got net.ppp0.local-ip: %s\n", ppp_local_ip);

	responses.status = 0;
	responses.suggestedRetryTime = -1;
	responses.cid = 1;
	responses.active = 2;
	responses.type = "PPP";
	responses.ifname = PPP_TTY_PATH;
	responses.addresses = ppp_local_ip;
	responses.dnses = ppp_dnses;
	responses.gateways = ppp_gw;
	
	RIL_onRequestComplete(t, RIL_E_SUCCESS, &responses, sizeof(RIL_Data_Call_Response_v6));
	return;

error:
	LOGE("Unable to setup PDP\n");
	RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);

}

/* CHECK There are several error cases if PDP deactivation fails
 * 24.008: 8, 25, 36, 38, 39, 112
 */ 
static void requestDeactivateDefaultPDP(void *data, size_t datalen, RIL_Token t)
{
	char * cid;

	LOGD("requestDeactivateDefaultPDP()");

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
    ATResponse *atresponse = NULL;
	ATLine *p_cur;
   	RIL_Data_Call_Response_v6 *responses;

    int err;
	int fd;
    char *line, *out;
	int i,n = 0;

	err = at_send_command_multiline ("AT+CGACT?", "+CGACT:", &atresponse);
	if (err != AT_NOERROR) 
		goto error;

	for (p_cur = atresponse->p_intermediates; p_cur != NULL;
			p_cur = p_cur->p_next)
		n++;

	responses = alloca(n * sizeof(RIL_Data_Call_Response_v6));

	for (i = 0; i < n; i++) {
        responses[i].status = -1;
        responses[i].suggestedRetryTime = -1;
        responses[i].cid = -1;
        responses[i].active = -1;
        responses[i].type = "";
        responses[i].ifname = PPP_TTY_PATH;
        responses[i].addresses = "";
        responses[i].dnses = "";
        responses[i].gateways = "";
	}

	RIL_Data_Call_Response_v6 *response = responses;
	for (p_cur = atresponse->p_intermediates; p_cur != NULL;
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

	at_response_free(atresponse);

    err = at_send_command_multiline("AT+CGDCONT?", "+CGDCONT:", &atresponse);
    if (err != AT_NOERROR)
        goto error;

	for (p_cur = atresponse->p_intermediates; p_cur != NULL;
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
			
		responses[i].type = alloca(strlen(out) + 1);
		strcpy(responses[i].type, out);

		// APN ignored for v5
		err = at_tok_nextstr(&line, &out);
		if (err < 0)
			goto error;

		responses[i].ifname = PPP_TTY_PATH;

		err = at_tok_nextstr(&line, &out);
		if (err < 0)
			goto error;
      
		responses[i].addresses = alloca(strlen(out) + 1);
		strcpy(responses[i].addresses, out);
	}

	
    at_response_free(atresponse);
    atresponse = NULL;

    if (t != NULL)
        RIL_onRequestComplete(*t, RIL_E_SUCCESS, responses,
                n * sizeof(RIL_Data_Call_Response_v6));
    else
        RIL_onUnsolicitedResponse(RIL_UNSOL_DATA_CALL_LIST_CHANGED, &response,
                sizeof(RIL_Data_Call_Response_v6));

    return;

error:
    if (t != NULL)
        RIL_onRequestComplete(*t, RIL_E_GENERIC_FAILURE, NULL, 0);
    else
        RIL_onUnsolicitedResponse(RIL_UNSOL_DATA_CALL_LIST_CHANGED, NULL, 0);

    at_response_free(atresponse); 
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
    ATResponse *atresponse = NULL;
    char *tok = NULL;
    int used1, total1;
    int err;
    int return_value;

    err = at_send_command_singleline("AT+CPMS=\"SM\",\"SM\"","+CPMS: ", &atresponse);
    if (err != AT_NOERROR) {
        LOGE("%s() Unable to set preferred message storage", __func__);
        goto error;
    }

    /*
     * Depending on the host boot time the indication that message storage
     * on SIM is full (+CIEV: 10,1) may be sent before the RIL is started.
     * The RIL will explicitly check status of SIM messages storage using
     * +CPMS intermediate response and inform Android if storage is full.
     * +CPMS: <used1>,<total1>,<used2>,<total2>,<used3>,<total3>
     */
    tok = atresponse->p_intermediates->line;

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
    LOGE("%s() Failed during AT+CPMS sending/handling!", __func__);
    return_value = 1;

exit:
    at_response_free(atresponse);
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
            LOGI("Message storage is ready");
            return;
        }
    }

    LOGE("%s() Message storage is not ready"
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
    struct held_pdu *hpdu = malloc(sizeof(*hpdu));
    if (hpdu == NULL) {
        LOGE("%s() failed to allocate memory!", __func__);
        return;
    }

    memset(hpdu, 0, sizeof(*hpdu));
    hpdu->type = type;
    hpdu->sms_pdu = strdup(sms_pdu);
    if (NULL == hpdu->sms_pdu) {
        LOGE("%s() failed to allocate memory!", __func__);
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
    ATResponse *atresponse = NULL;
    char *tok = NULL;
    char* storage_area = NULL;
    int used1, total1;
    int err;
    (void) p;

    err = at_send_command_singleline("AT+CPMS?", "+CPMS: ", &atresponse);
    if (err != AT_NOERROR)
        goto error;

    tok = atresponse->p_intermediates->line;

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
    LOGE("%s() failed during AT+CPMS sending/handling!", __func__);
exit:
    at_response_free(atresponse);
    return;
}

/**
 * RIL_UNSOL_SIM_SMS_STORAGE_FULL
 *
 * SIM SMS storage area is full, cannot receive
 * more messages until memory freed
 */
void onNewSmsIndication(void)
{
    enqueueRILEvent(isSimSmsStorageFull, NULL, NULL);
} 

void onNewSms(const char *sms_pdu)
{
    pthread_mutex_lock(&s_held_pdus_mutex);

    /* No RIL_UNSOL_RESPONSE_NEW_SMS or RIL_UNSOL_RESPONSE_NEW_SMS_STATUS_REPORT
     * messages should be sent until a RIL_REQUEST_SMS_ACKNOWLEDGE has been received for
     * previous new SMS.
     */
    if (s_outstanding_acknowledge) {
        LOGI("Waiting for ack for previous sms, enqueueing PDU");
        enqueue_held_pdu(OUTSTANDING_SMS, sms_pdu);
    } else {
        s_outstanding_acknowledge = 1;
        RIL_onUnsolicitedResponse(RIL_UNSOL_RESPONSE_NEW_SMS,
                                  sms_pdu, strlen(sms_pdu));
    }

    pthread_mutex_unlock(&s_held_pdus_mutex);
}

void onNewStatusReport(const char *sms_pdu)
{
    char *response = NULL;
    int err;

    /* Baseband will not prepend SMSC addr, but Android expects it. */
    err = asprintf(&response, "%s%s", "00", sms_pdu);
    if (err == -1) {
        LOGD("%s() Error allocating memory!", __func__);
        return;
    }

    pthread_mutex_lock(&s_held_pdus_mutex);

    /* No RIL_UNSOL_RESPONSE_NEW_SMS or RIL_UNSOL_RESPONSE_NEW_SMS_STATUS_REPORT
     * messages should be sent until a RIL_REQUEST_SMS_ACKNOWLEDGE has been received for
     * previous new SMS.
     */
    if (s_outstanding_acknowledge) {
        LOGE("%s() Waiting for previous ack, enqueueing PDU..", __func__);
        enqueue_held_pdu(OUTSTANDING_STATUS, response);
    } else {
        s_outstanding_acknowledge = 1;
        RIL_onUnsolicitedResponse(RIL_UNSOL_RESPONSE_NEW_SMS_STATUS_REPORT,
                                  response, strlen(response));
    }
    free(response);
    pthread_mutex_unlock(&s_held_pdus_mutex);
}

void onNewBroadcastSms(const char *pdu)
{
    char *message = NULL;
    LOGD("%s() Length : %d", __func__, strlen(pdu));

    if (strlen(pdu) != (2 * BSM_LENGTH)) {
        LOGE("%s() Broadcast Message length error! Discarding!", __func__);
        goto error;
    }
    LOGD("%s() PDU: %176s", __func__, pdu);

    message = alloca(BSM_LENGTH);
    if (!message) {
        LOGE("%s() error allocating memory for message! Discarding!", __func__);
        goto error;
    }

    stringToBinary(pdu, 2*BSM_LENGTH, (unsigned char *)message);
    LOGD("%s() Message: %88s", __func__, message);

    pthread_mutex_lock(&s_held_pdus_mutex);

    /* No RIL_UNSOL_RESPONSE_NEW_SMS or RIL_UNSOL_RESPONSE_NEW_SMS_STATUS_REPORT
     * or RIL_UNSOL_RESPONSE_NEW_CB
     * messages should be sent until a RIL_REQUEST_SMS_ACKNOWLEDGE has been received for
     * previous new SMS.
     */
    if (s_outstanding_acknowledge) {
        LOGE("%s() Waiting for previous ack, enqueueing PDU..", __func__);
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

void onNewSmsOnSIM(const char *s)
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

    RIL_onUnsolicitedResponse(RIL_UNSOL_RESPONSE_NEW_SMS_ON_SIM,
                              &index, sizeof(int *));

finally:
    free(line);
    return;

error:
    LOGE("%s() Failed to parse +CMTI.", __func__);
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
    int err, aterr;
    const char *testSmsc;
	char smsc[30];
    const char *pdu;
    char *line;
    int tpLayerLength;
    char *cmd1, *cmd2;
    RIL_SMS_Response response;
    RIL_Errno ret = RIL_E_SUCCESS;
    ATResponse *atresponse = NULL;

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
	
		err = at_send_command_singleline("AT+CSCA?", "+CSCA:", &atresponse);
		if (err != AT_NOERROR) 
			goto error;

		line = atresponse->p_intermediates->line;

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

		if ( length % 2) {//One extra number
			smsc[4+length] = temp[curChar];
			smsc[3+length]='F';
			smsc[5+length]='\0';
		} else {
			smsc[4+length] = '\0';
		}
		at_response_free(atresponse);
	}
	else
		strcpy(smsc,testSmsc);

    asprintf(&cmd1, "AT+CMGS=%d", tpLayerLength);
    asprintf(&cmd2, "%s%s", smsc, pdu);
    aterr = at_send_command_sms(cmd1, cmd2, "+CMGS:", &atresponse);
    free(cmd1);
    free(cmd2);

    if (aterr != AT_NOERROR)
        goto error;

    memset(&response, 0, sizeof(response));
	
	/* Set errorCode to -1 if unknown or not applicable
     * See 3GPP 27.005, 3.2.5 for GSM/UMTS
     */
    response.errorCode = -1;

    line = atresponse->p_intermediates->line;

    err = at_tok_start(&line);
    if (err < 0)
        goto error;

    err = at_tok_nextint(&line, &response.messageRef);
    if (err < 0)
        goto error;

    /* No support for ackPDU. Do we need it? */
    RIL_onRequestComplete(t, RIL_E_SUCCESS, &response, sizeof(response));

finally:
    at_response_free(atresponse);
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
    ATResponse *atresponse = NULL;

    (void) datalen;

    args = (RIL_SMS_WriteArgs *) data;

    length = strlen(args->pdu) / 2;
	
    asprintf(&cmd, "AT+CMGW=%d,%d", length, args->status);
    asprintf(&pdu, "%s%s", (args->smsc ? args->smsc : "00"), args->pdu);
    err = at_send_command_sms(cmd, pdu, "+CMGW:", &atresponse);
    free(cmd);
    free(pdu);

    if (err != AT_NOERROR)
        goto error;

    line = atresponse->p_intermediates->line;

    err = at_tok_start(&line);
    if (err < 0)
        goto error;

    err = at_tok_nextint(&line, &index);
    if (err < 0)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, &index, sizeof(int *));

finally:
    at_response_free(atresponse);
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

    err = at_send_command("AT+CMGD=%d", ((int *) data)[0]);
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
    ATResponse *atresponse = NULL;
    int err;
    char *line;
    char *response;

    err = at_send_command_singleline("AT+CSCA?", "+CSCA:", &atresponse);

    if (err != AT_NOERROR)
        goto error;

    line = atresponse->p_intermediates->line;

    err = at_tok_start(&line);
    if (err < 0)
        goto error;

    err = at_tok_nextstr(&line, &response);
    if (err < 0)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, response, sizeof(char *));

finally:
    at_response_free(atresponse);
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
        LOGI("SMS storage full");
        break;

    case 1:
        /* Since we are not using +CNMA command. It's fine to return without informing network */
        LOGI("Failed to inform network for Message Cleanup. Need cmd : ESMSMEMAVAIL");
        break;

    default:
        LOGE("%s() Invalid parameter", __func__);
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
		LOGE("unsupported arg to RIL_REQUEST_SMS_ACKNOWLEDGE\n");
		goto error;
	}

	RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
	
    pthread_mutex_lock(&s_held_pdus_mutex);
    hpdu = dequeue_held_pdu();

    if (hpdu != NULL) {
        LOGE("%s() Outstanding requests in queue, dequeueing and sending.",
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
    ATResponse *atresponse = NULL;
    int mode, err = 0;
    unsigned int i, count = 0;
    char *mids;
    char *range;
    char *trange;
    char *tok = NULL;

    RIL_GSM_BroadcastSmsConfigInfo *configInfo[BROADCAST_MAX_RANGES_SUPPORTED];

    err = at_send_command_singleline("AT+CSCB?", "+CSCB:", &atresponse);

    if (err != AT_NOERROR)
        goto error;

    tok = atresponse->p_intermediates->line;

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
            configInfo[count] = calloc(1,
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
            LOGW("%s() Max limit (%d) passed, can not send all ranges "
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
    at_response_free(atresponse);
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
    LOGI("Number of MID ranges in BROADCAST_SMS_CONFIG: %d", count);

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
    ATResponse *atresponse = NULL;
    int mode, mt, bm, ds, bfr, skip;
    int activation;
    char *tok;
    int err;

    (void) datalen;

    /* AT+CNMI=[<mode>[,<mt>[,<bm>[,<ds>[,<bfr>]]]]] */
    err = at_send_command_singleline("AT+CNMI?", "+CNMI:", &atresponse);
    if (err != AT_NOERROR)
        goto error;

    tok = atresponse->p_intermediates->line;

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
    at_response_free(atresponse);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    goto finally;
}
 

/**
 * RIL_REQUEST_SIM_IO
 *	
 * Request SIM I/O operation.
 * This is similar to the TS 27.007 "restricted SIM" operation
 * where it assumes all of the EF selection will be done by the
 * callee.
 */ 
static void requestSIM_IO(void *data, size_t datalen, RIL_Token t)
{
	ATResponse *atresponse = NULL;
	RIL_SIM_IO_Response sr;
	int err;
	char *cmd = NULL;
	RIL_SIM_IO_v6 *p_args;
	char *line;

	memset(&sr, 0, sizeof(sr));

	p_args = (RIL_SIM_IO_v6 *)data;

	/* FIXME handle pin2 */

	if (p_args->data == NULL) {
		asprintf(&cmd, "AT+CRSM=%d,%d,%d,%d,%d",
				p_args->command, p_args->fileid,
				p_args->p1, p_args->p2, p_args->p3);
	} else {
		asprintf(&cmd, "AT+CRSM=%d,%d,%d,%d,%d,%s",
				p_args->command, p_args->fileid,
				p_args->p1, p_args->p2, p_args->p3, p_args->data);
	}

	err = at_send_command_singleline(cmd, "+CRSM:", &atresponse);
	if (err != AT_NOERROR) 
		goto error;

	line = atresponse->p_intermediates->line;

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

	RIL_onRequestComplete(t, RIL_E_SUCCESS, &sr, sizeof(sr));
	at_response_free(atresponse);
	free(cmd);
	return;

error:
	RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
	at_response_free(atresponse);
	free(cmd);
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
    RIL_CardStatus_v6 *p_card_status = malloc(sizeof(RIL_CardStatus_v6));
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

        /* Only support one app, gsm/wcdma. */
        p_card_status->num_applications = 1;
        p_card_status->gsm_umts_subscription_app_index = 0;

        /* Get the correct app status. */
        p_card_status->applications[0] = app_status_array[sim_status];

		LOGI("[Card type discovery]: USIM");
		p_card_status->applications[0].app_type = RIL_APPTYPE_USIM;
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
                           char *facility, int request)
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
    } else
        RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
}


/**
 * RIL_REQUEST_QUERY_FACILITY_LOCK
 *
 * Query the status of a facility lock state.
 */
static void requestQueryFacilityLock(void *data, size_t datalen, RIL_Token t)
{
    int err, response;
    ATResponse *atresponse = NULL;
    char *line = NULL;
    char *facility_string = NULL;
    char *facility_password = NULL;
    char *facility_class = NULL;

    (void) datalen;

    if (datalen < 3 * sizeof(char **)) {
        LOGE("%s() bad data length!", __func__);
        goto error;
    }

    facility_string = ((char **) data)[0];
    facility_password = ((char **) data)[1];
    facility_class = ((char **) data)[2];

    err = at_send_command_singleline("AT+CLCK=\"%s\",2,%s,%s", "+CLCK:", &atresponse,
            facility_string, facility_password, facility_class);
    if (err != AT_NOERROR)
        goto error;

    line = atresponse->p_intermediates->line;

    err = at_tok_start(&line);

    if (err < 0)
        goto error;

    err = at_tok_nextint(&line, &response);

    if (err < 0)
        goto error;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, &response, sizeof(int));
    at_response_free(atresponse);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    at_response_free(atresponse);
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
        LOGE("%s() bad data length!", __func__);
        goto exit;
    }

    facility_string = ((char **) data)[0];
    facility_mode_str = ((char **) data)[1];
    facility_password = ((char **) data)[2];
    facility_class = ((char **) data)[3];

    if (*facility_mode_str != '0' && *facility_mode_str != '1') {
        LOGE("%s() bad facility mode!", __func__);
        goto exit;
    }

    facility_mode = atoi(facility_mode_str);

    /*
     * Skip adding facility_password to AT command parameters if it is NULL,
     * printing NULL with %s will give string "(null)".
     */
    err = at_send_command("AT+CLCK=\"%s\",%d,\"%s\",%s", facility_string,
            facility_mode, facility_password, facility_class);

    if (at_get_error_type(err) == AT_ERROR)
        goto exit;
    if (err != AT_NOERROR) {
        switch (at_get_cme_error(err)) {
        /* CME ERROR 11: "SIM PIN required" happens when PIN is wrong */
        case CME_SIM_PIN_REQUIRED:
            LOGI("Wrong PIN");
            errorril = RIL_E_PASSWORD_INCORRECT;
            break;
        /*
         * CME ERROR 12: "SIM PUK required" happens when wrong PIN is used
         * 3 times in a row
         */
        case CME_SIM_PUK_REQUIRED:
            LOGI("PIN locked, change PIN with PUK");
            num_retries = 0;/* PUK required */
            errorril = RIL_E_PASSWORD_INCORRECT;
            break;
        /* CME ERROR 16: "Incorrect password" happens when PIN is wrong */
        case CME_INCORRECT_PASSWORD:
            LOGI("Incorrect password, Facility: %s", facility_string);
            errorril = RIL_E_PASSWORD_INCORRECT;
            break;
        /* CME ERROR 17: "SIM PIN2 required" happens when PIN2 is wrong */
        case CME_SIM_PIN2_REQUIRED:
            LOGI("Wrong PIN2");
            errorril = RIL_E_PASSWORD_INCORRECT;
            break;
        /*
         * CME ERROR 18: "SIM PUK2 required" happens when wrong PIN2 is used
         * 3 times in a row
         */
        case CME_SIM_PUK2_REQUIRED:
            LOGI("PIN2 locked, change PIN2 with PUK2");
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
        if(!strings[1]){
            err = at_send_command("AT+CPIN=\"%s\"", strings[0]);
        } else {
            err = at_send_command("AT+CPIN=\"%s\",\"%s\"", strings[0], strings[1]);
        }
    } else if (datalen == 3 * sizeof(char *)) {
            err = at_send_command("AT+CPIN=\"%s\",\"%s\"", strings[0], strings[1]);
    } else
        goto error;

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
        /*
         * Got OK, return success and wait for *EPEV to trigger poll
         * of SIM state.
         */

        num_retries = 1;
        RIL_onRequestComplete(t, RIL_E_SUCCESS, &num_retries, sizeof(int *));
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
    ATResponse *atresponse = NULL;
    int response = 0;
    char *line;

    err = at_send_command_singleline("AT+COPS?", "+COPS:", &atresponse);

    if (err != AT_NOERROR)
        goto error;

    line = atresponse->p_intermediates->line;

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
    at_response_free(atresponse);
    return;

error:
    LOGE("%s() Must never return error when radio is on", __func__);
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
    ATResponse *atresponse = NULL;
    struct operatorPollParams *poll_params;
    RIL_Token t;

    assert(params != NULL);

    poll_params = (struct operatorPollParams *) params;
    t = poll_params->t;

    if (poll_params->loopcount >= REPOLL_OPERATOR_SELECTED)
        goto error;

    err = at_send_command_singleline("AT+COPS?", "+COPS:", &atresponse);
    if (err != AT_NOERROR)
        goto error;

    line = atresponse->p_intermediates->line;

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

    at_response_free(atresponse);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    free(poll_params);
    at_response_free(atresponse);
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
    ATResponse *atresponse = NULL;
    int mode = 0;
    int skip;
    char *line;
    char *operator = NULL;
    struct operatorPollParams *poll_params = NULL;

    poll_params = malloc(sizeof(struct operatorPollParams));
    if (NULL == poll_params)
        goto error;

    /* First check if we are already scanning or in manual mode */
    err = at_send_command_singleline("AT+COPS=3,2;+COPS?", "+COPS:", &atresponse);
    if (err != AT_NOERROR)
        goto error;

    line = atresponse->p_intermediates->line;

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
            LOGD("%s() Changing manual to automatic network mode", __func__);
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
            LOGD("%s() Changing manual to automatic network mode", __func__);
            goto do_auto;
        } else
            goto check_reg;
    }

    /* Read numeric operator */
    err = at_tok_nextstr(&line, &operator);
    if (err < 0)
        goto error;

    /* If operator is found then do a new scan,
       else let it continue the already pending scan */
    if (operator && strlen(operator) == 0) {
        if (mode == 1) {
            LOGD("%s() Changing manual to automatic network mode", __func__);
            goto do_auto;
        } else
            goto check_reg;
    }

    /* Operator found */
    if (mode == 1) {
        LOGD("%s() Changing manual to automatic network mode", __func__);
        goto do_auto;
    } else {
        LOGD("%s() Already in automatic mode with known operator, trigger a new network scan",
	    __func__);
        goto do_auto;
    }

    /* Check if module is scanning,
       if not then trigger a rescan */
check_reg:
    at_response_free(atresponse);
    atresponse = NULL;

    /* Check CS domain first */
    err = at_send_command_singleline("AT+CREG?", "+CREG:", &atresponse);
    if (err != AT_NOERROR)
        goto error;

    line = atresponse->p_intermediates->line;

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
        LOGD("%s() Already in automatic mode, but not currently scanning on CS,"
	     "trigger a new network scan", __func__);
        goto do_auto;
    }

    /* Now check PS domain */
    at_response_free(atresponse);
    atresponse = NULL;
    err = at_send_command_singleline("AT+CGREG?", "+CGREG:", &atresponse);
    if (err != AT_NOERROR)
        goto error;

    line = atresponse->p_intermediates->line;

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
        LOGD("%s() Already in automatic mode, but not currently scanning on PS,"
	     "trigger a new network scan", __func__);
        goto do_auto;
    }
    else
    {
        LOGD("%s() Already in automatic mode and scanning", __func__);
        goto finish_scan;
    }

do_auto:
    at_response_free(atresponse);
    atresponse = NULL;

    /* This command does two things, one it sets automatic mode,
       two it starts a new network scan! */
    err = at_send_command("AT+COPS=0");
    if (err != AT_NOERROR)
        goto error;

finish_scan:

    at_response_free(atresponse);
    atresponse = NULL;

    poll_params->loopcount = 0;
    poll_params->t = t;

    enqueueRILEvent(pollOperatorSelected,
                    poll_params, &TIMEVAL_OPERATOR_SELECT_POLL);

    return;

error:
    free(poll_params);
    at_response_free(atresponse);
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
    ATResponse *atresponse = NULL;
    const char *statusTable[] =
        { "unknown", "available", "current", "forbidden" };
    char **responseArray = NULL;
    char *p;
    int n = 0;
    int i = 0;

    err = at_send_command_multiline("AT+COPS=?", "+COPS:", &atresponse);
    if (err != AT_NOERROR)
        goto error;

    p = atresponse->p_intermediates->line;

    /* count number of '('. */
    err = at_tok_charcounter(p, '(', &n);
    if (err < 0) goto error;

    /* Allocate array of strings, blocks of 4 strings. */
    responseArray = alloca(n * QUERY_NW_NUM_PARAMS * sizeof(char *));

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
            LOGE("%s() Null pointer while parsing COPS response."
	         "This should not happen.", __func__);
            break;
        }
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

        responseArray[i * QUERY_NW_NUM_PARAMS + 0] = alloca(strlen(longAlphaNumeric) + 1);
        strcpy(responseArray[i * QUERY_NW_NUM_PARAMS + 0], longAlphaNumeric);

        responseArray[i * QUERY_NW_NUM_PARAMS + 1] = alloca(strlen(shortAlphaNumeric) + 1);
        strcpy(responseArray[i * QUERY_NW_NUM_PARAMS + 1], shortAlphaNumeric);

        responseArray[i * QUERY_NW_NUM_PARAMS + 2] = alloca(strlen(numeric) + 1);
        strcpy(responseArray[i * QUERY_NW_NUM_PARAMS + 2], numeric);

        free(s);

        /*
         * Check if modem returned an empty string, and fill it with MNC/MMC
         * if that's the case.
         */
        if (responseArray[i * QUERY_NW_NUM_PARAMS + 0] && strlen(responseArray[i * QUERY_NW_NUM_PARAMS + 0]) == 0) {
            responseArray[i * QUERY_NW_NUM_PARAMS + 0] = alloca(strlen(responseArray[i * QUERY_NW_NUM_PARAMS + 2]) + 1);
            strcpy(responseArray[i * QUERY_NW_NUM_PARAMS + 0], responseArray[i * QUERY_NW_NUM_PARAMS + 2]);
        }

        if (responseArray[i * QUERY_NW_NUM_PARAMS + 1] && strlen(responseArray[i * QUERY_NW_NUM_PARAMS + 1]) == 0) {
            responseArray[i * QUERY_NW_NUM_PARAMS + 1] = alloca(strlen(responseArray[i * QUERY_NW_NUM_PARAMS + 2]) + 1);
            strcpy(responseArray[i * QUERY_NW_NUM_PARAMS + 1], responseArray[i * QUERY_NW_NUM_PARAMS + 2]);
        }

		/* Add status */
        responseArray[i * QUERY_NW_NUM_PARAMS + 3] = alloca(strlen(statusTable[status])+1);
		strcpy(responseArray[i * QUERY_NW_NUM_PARAMS + 3],statusTable[status]);
    }

    RIL_onRequestComplete(t, RIL_E_SUCCESS, responseArray,
                          i * QUERY_NW_NUM_PARAMS * sizeof(char *));

finally:
    at_response_free(atresponse);
    return;

error:
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

	/* Huawei specific 	
		AT^SYSCFG=2,1,3FFFFFFF,2,4 for GPRS/EDGE Preferred
		AT^SYSCFG=2,2,3FFFFFFF,2,4 for 3G Preferred
		AT^SYSCFG=13,1,3FFFFFFF,2,4 for GPRS/EDGE Only
		AT^SYSCFG=14,2,3FFFFFFF,2,4 for 3G Only

		The third parameter, 0x3FFFFFFF tells the card to use all bands.
		A value of 0x400380 here means GSM900/1800/WCDMA2100 only and a 
		value of 0x200000 here means GSM1900 only.

	*/
	
	switch (rat) {
		case PREF_NET_TYPE_GSM_ONLY:  /* GSM only */
			cmd = "AT^SYSCFG=13,1,3FFFFFFF,2,4"; /* for GPRS/EDGE Only */
			break; 
		case PREF_NET_TYPE_GSM_WCDMA: /* WCDMA only */
			cmd = "AT^SYSCFG=14,2,3FFFFFFF,2,4"; /* for 3G Only */
			break;  
		case PREF_NET_TYPE_GSM_WCDMA_AUTO:
		default: 	/* Dual Mode - WCDMA preferred*/
			cmd = "AT^SYSCFG=2,2,3FFFFFFF,2,4";	 /* for 3G Preferred */
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
	LOGE("ERROR: requestSetPreferredNetworkType() failed\n");
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
	/*
	AT^SYSCFG=2,1,3FFFFFFF,1,2 for GPRS/EDGE Preferred
	AT^SYSCFG=2,2,3FFFFFFF,1,2 for 3G Preferred
	AT^SYSCFG=13,1,3FFFFFFF,1,2 for GPRS/EDGE Only
	AT^SYSCFG=14,2,3FFFFFFF,1,2 for 3G Only
	*/

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
	if (ret1 == 13) {
		response = PREF_NET_TYPE_GSM_ONLY;  /* GSM only */
	} else if (ret1 == 14) {
		response = PREF_NET_TYPE_GSM_WCDMA; /* WCDMA only */
	} else {
		response = PREF_NET_TYPE_GSM_WCDMA_AUTO; /* for 3G Preferred */
	}
	
	LOGD("requestGetPreferredNetworkType() mode:%d\n",response);
	RIL_onRequestComplete(t, RIL_E_SUCCESS, &response, sizeof(int));
	at_response_free(atResponse);	
	return;
	
error:
	LOGE("ERROR: requestGetPreferredNetworkType() failed - modem does not support command\n");
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
    LOGE("%s() Must never return an error when radio is on", __func__);
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
    ATResponse *atresponse = NULL;
    char *line, *p;
    int commas = 0;
    int skip, tmp;
    int count = 3;

    memset(responseStr, 0, sizeof(responseStr));
    memset(response, 0, sizeof(response));
    response[1] = -1;
    response[2] = -1;

    err = at_send_command_singleline("AT+CGREG?", "+CGREG: ", &atresponse);
    if (err != AT_NOERROR)
        goto error;

    line = atresponse->p_intermediates->line;
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
        LOGE("%s() at_tok_charcounter failed", __func__);
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
        LOGE("%s() Invalid input", __func__);
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

    responseStr[5] = "1";

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

    at_response_free(atresponse);
    return;

error:
    LOGE("%s Must never return an error when radio is on", __func__);
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
    ATResponse *atresponse = NULL;
    ATLine *atline;
    int linecount;
    int err;

    LOGD("%s() got OEM_HOOK_STRINGS: %8p %lu", __func__, data, (long) datalen);

    for (i = (datalen / sizeof(char *)), cur = (const char **) data;
         i > 0; cur++, i--) {
        LOGD("%s(): String: %s", __func__, *cur);
    }

    /* Only take the first string in the array for now */
    cur = (const char **) data;
    err = at_send_command_raw(*cur, &atresponse);

    if ((err != AT_NOERROR && at_get_error_type(err) == AT_ERROR)
            || atresponse == NULL || atresponse->finalResponse == NULL)
        goto error;

    /* Count number of lines including prefix, intermediate and final response */
    linecount = 0;
    atline = atresponse->p_intermediates;
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
            atline = atresponse->p_intermediates;
            while (atline != NULL && atline->line != NULL) {
                cur[linecount++] = atline->line;
                atline = atline->p_next;
            }
            cur[linecount++] = atresponse->finalResponse;
        } else
            goto error;
    }

    RIL_onRequestComplete(t, RIL_E_SUCCESS, cur, linecount * sizeof(char *));
    at_response_free(atresponse);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    at_response_free(atresponse); 
}


static void unsolicitedNitzTime(const char * s)
{
	int err;
	char * response = NULL;
	char * line = NULL;
	char * p = NULL;
	char * tz = NULL; /* Timezone */
	static char sNITZtime[64] = {0};
	
	line = strdup(s);

	/* Higher layers expect a NITZ string in this format:
	 *  08/10/28,19:08:37-20,1 (yy/mm/dd,hh:mm:ss(+/-)tz,dst)
	 */

	if(strStartsWith(s,"+CTZV:")){

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

	}
	else if(strStartsWith(s,"+CTZDST:")){

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
	LOGE("Invalid NITZ line %s\n", s);
}

static void unsolicitedRSSI(const char * s)
{
	int err;
	int rssi;
	RIL_SignalStrength_v6 signalStrength;
	char * line = NULL;

	line = strdup(s);

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

	LOGI("Signal Strength %d", rssi);

	RIL_onUnsolicitedResponse(RIL_UNSOL_SIGNAL_STRENGTH, &signalStrength, sizeof(signalStrength));
	free(line);
	return;

error:
	/* The notification was for a battery event - do not send a msg to upper layers */
	LOGI("Error getting Signal Strength");
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
	LOGI("Error getting mode");
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
    ATResponse *atresponse = NULL;
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

    err = at_send_command_singleline("AT+CSQ", "+CSQ:", &atresponse);
    if (err != AT_NOERROR)
        goto error;
    
    line = atresponse->p_intermediates->line;

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

	LOGI("SignalStrength %d BER: %d", rssi, ber);
	
	RIL_onRequestComplete(t, RIL_E_SUCCESS, &signalStrength,
						  sizeof(RIL_SignalStrength_v6));
						  
    at_response_free(atresponse);
    atresponse = NULL;
    return;

error:
	LOGE("%s() Must never return an error when radio is on", __func__);
	RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    at_response_free(atresponse);
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
    ATResponse *atresponse = NULL;

    memset(response, 0, sizeof(response));

    err = at_send_command_multiline
        ("AT+COPS=3,0;+COPS?;+COPS=3,1;+COPS?;+COPS=3,2;+COPS?", "+COPS:",
         &atresponse);

    if (err != AT_NOERROR)
        goto error;

    /* We expect 3 lines here:
     * +COPS: 0,0,"T - Mobile"
     * +COPS: 0,1,"TMO"
     * +COPS: 0,2,"310170"
     */
    for (i = 0, cursor = atresponse->p_intermediates;
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
        response[0] = alloca(strlen(response[2]) + 1);
        strcpy(response[0], response[2]);
    }

    if (response[2] && response[1] && strlen(response[1]) == 0) {
        response[1] = alloca(strlen(response[2]) + 1);
        strcpy(response[1], response[2]);
    }

    RIL_onRequestComplete(t, RIL_E_SUCCESS, response, sizeof(response));

finally:
    at_response_free(atresponse);
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
        LOGE("%s() bad data length!", __func__);
        goto error;
    } 
	
	onOff = ((int *)data)[0];

	if (onOff == 0 && getRadioState() != RADIO_STATE_OFF) {
		char value[PROPERTY_VALUE_MAX];
		
		/* Set low power consumption mode */
		err = at_send_command("AT+CFUN=7");
        if (err != AT_NOERROR)
            goto error;
			
		/* Just in case, switch RF OFF */
		err = at_send_command("AT^RFSWITCH=0");
        if (err != AT_NOERROR)
            goto error;

        if (property_get("sys.shutdown.requested", value, NULL)) {
            setRadioState(RADIO_STATE_UNAVAILABLE);
        } else
            setRadioState(RADIO_STATE_OFF);  
					
	} else if (onOff > 0 && getRadioState() == RADIO_STATE_OFF) {

		/* Just in case, switch RF ON */
		err = at_send_command("AT^RFSWITCH=1");
        if (err != AT_NOERROR)
            goto error;

		/* Set operation mode as online */
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
	} else {
        LOGE("%s() Erroneous input", __func__);
        goto error;
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
    ATResponse *atresponse = NULL;
    int err;

    err = at_send_command_numeric("AT+CIMI", &atresponse);

    if (err != AT_NOERROR)
        RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    else {
        RIL_onRequestComplete(t, RIL_E_SUCCESS,
                              atresponse->p_intermediates->line,
                              sizeof(char *));
    } 
	at_response_free(atresponse);
}



/**
 * RIL_REQUEST_GET_IMEISV
 *
 * Get the device IMEISV, which should be two decimal digits.
*/ 
static void requestGetIMEISV(RIL_Token t)
{
	RIL_onRequestComplete(t, RIL_E_SUCCESS,"01", sizeof(char *));
}



/* RIL_REQUEST_DEVICE_IDENTITY
 *
 * Request the device ESN / MEID / IMEI / IMEISV.
 *
 */
static void requestDeviceIdentity(RIL_Token t)
{
    ATResponse *atresponse = NULL;
    char* response[4];
    int err;

    /* IMEI */
    err = at_send_command_numeric("AT+CGSN", &atresponse);

    if (err != AT_NOERROR)
        goto error;

    response[0] = atresponse->p_intermediates->line;
	
	/* IMEISVN */
	response[1] = "01";
		
    /* CDMA not supported */
    response[2] = "";
    response[3] = "";

    RIL_onRequestComplete(t, RIL_E_SUCCESS, &response, sizeof(response));

    at_response_free(atresponse);
    return;

error:
    RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
    at_response_free(atresponse);
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
    ATResponse *atresponse = NULL;
    char *line;

    err = at_send_command_singleline("AT+CGMR", "\0", &atresponse);

    if (err != AT_NOERROR) {
        LOGE("%s() Error reading Base Band Version", __func__);
        RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
        return;
    }

    line = atresponse->p_intermediates->line;

    RIL_onRequestComplete(t, RIL_E_SUCCESS, line, sizeof(char *));

    at_response_free(atresponse);
}
 

/**
 * RIL_REQUEST_STK_SEND_TERMINAL_RESPONSE
 *
 * Requests to send a terminal response to SIM for a received
 * proactive command.
 */ 
static void requestStkSendTerminalResponse(void * data, size_t datalen, RIL_Token t) 
{
    (void)datalen;
    const char *stkResponse = (const char *) data;
    RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0); 
}


/**
 * RIL_REQUEST_STK_SEND_ENVELOPE_COMMAND
 *
 * Requests to send a SAT/USAT envelope command to SIM.
 * The SAT/USAT envelope command refers to 3GPP TS 11.14 and 3GPP TS 31.111.
 */ 
static void requestStkSendEnvelopeCommand(void * data, size_t datalen, RIL_Token t) 
{
    const char *ec = (const char *) data;
    (void)datalen;
    RIL_onRequestComplete(t, RIL_E_SUCCESS, "ok", sizeof(char *));
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
	RIL_onRequestComplete(t, RIL_E_SUCCESS, "default", sizeof(char *));
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

	/* success or failure is ignored by the upper layer here.
	   it will call GET_CURRENT_CALLS and determine success that way */
	RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}


static void requestHangup(void *data, size_t datalen, RIL_Token t)
{
	int *p_line;

	int ret;

	p_line = (int *)data;

	// 3GPP 22.030 6.5.5
	// "Releases a specific active call X"
	ret = at_send_command("AT+CHLD=1%d", p_line[0]);

	/* success or failure is ignored by the upper layer here.
	   it will call GET_CURRENT_CALLS and determine success that way */
	RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}

static void requestHangupWaitingOrBackground(RIL_Token t)
{
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
	at_send_command("ATA");

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
	if (party > 0 && party < 10){
		at_send_command("AT+CHLD=2%d", party);
		RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
	}
	else{
		RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
	}
}


static void requestSetMute(void *data, size_t datalen, RIL_Token t)
{
	int err;
	assert (datalen >= sizeof(int *));

	/* mute */
	err = at_send_command("AT+CMUT=%d", ((int*)data)[0]);
	if (err != AT_NOERROR) 
		goto error;

	RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
	return;

error:
	LOGE("ERROR: requestSetMute failed");
	RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);

}


static void requestGetMute(RIL_Token t)
{
	int err;
	ATResponse *atResponse = NULL;
	int response[1];
	char *line;

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
	LOGE("ERROR: requestGetMute failed");
	RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
	at_response_free(atResponse);
}


static void requestDTMF(void * data, size_t datalen, RIL_Token t)
{
	int err = 0;
	char c = ((char *)data)[0];

	err = at_send_command("AT+VTS=%c", (int)c);
	if(err != AT_NOERROR){
		RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
	}
	else{
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
	
	/* Unmute */
	err = at_send_command("AT+CMUT=0");
	if (err != AT_NOERROR) 
		goto error;

	RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
	return;

error:
	LOGE("ERROR: requestDtmfStop failed");
	RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);

}


static void requestDtmfStart(void *data, size_t datalen, RIL_Token t)
{
	int err;
	char c;

	assert (datalen >= sizeof(char *));

	c = ((char *)data)[0];
	
	/* Mute audio */
	at_send_command("AT+CMUT=1");
	
	/* Start DTMF generation (huawei specific command) */
	err = at_send_command("AT^DTMF=%c", (int)c);
	if (err != AT_NOERROR) 
		goto error;

	RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
	return;

error:
	LOGE("ERROR: requestDtmfStart failed");
	RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);

}

static void unsolicitedUSSD(const char *s)
{
	char *line, *linestart;
	int typeCode, count, err, len;
	char *message;
	char *outputmessage;
	char *responseStr[2] = {0,0};

	LOGD("unsolicitedUSSD %s\n",s);

	linestart=line=strdup(s);
	err = at_tok_start(&line);
	if(err < 0) goto error;

	err = at_tok_nextint(&line, &typeCode);
	if(err < 0) goto error;

	if(at_tok_hasmore(&line)) {
		err = at_tok_nextstr(&line, &message);
		if(err < 0) goto error;
		
		outputmessage = malloc(strlen(message)*2+1);
		gsm_hex_to_bytes((cbytes_t)message,strlen(message),(bytes_t)outputmessage);
		
		responseStr[1] = malloc(strlen(outputmessage)*2+1);
		
		len = utf8_from_gsm8((cbytes_t)outputmessage,strlen(outputmessage),(bytes_t)responseStr[1]);
		
		responseStr[1][strlen(message)/2]='\0';
		free(outputmessage);
		
		count = 2;
	} else {
		responseStr[1]=NULL;
		count = 1;
	}
	free(linestart);
	asprintf(&responseStr[0], "%d", typeCode);
	
	RIL_onUnsolicitedResponse (RIL_UNSOL_ON_USSD, responseStr, count*sizeof(char*));
	if (responseStr[0]) 
		free(responseStr[0]);
	if (responseStr[1])
		free(responseStr[1]);

	return;

error:
	LOGE("unexpectedUSSD error\n");
	free(linestart);
}


static void requestCancelUSSD(RIL_Token t)
{
	int err = 0;
	ATResponse *atResponse = NULL;

	err = at_send_command_numeric("AT+CUSD=2", &atResponse);

	if (err != AT_NOERROR){
		RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
	}
	else{
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
	int class;
	int response[2];

	class = ((int *)data)[0];
	
	err = at_send_command_singleline("AT+CCWA=1,2,%d", "+CCWA:", &atResponse, class);
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
	int enabled, class;

	if((datalen<2)||(data==NULL)) goto error;

	enabled = ((int *)data)[0];
	class = ((int *)data)[1];

	err = at_send_command("AT+CCWA=0,%d,%d",enabled,class);
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

	responses = alloca(n * sizeof(RIL_CallForwardInfo *));

	for(i = 0; i < n; i++) {
		responses[i] = alloca(sizeof(RIL_CallForwardInfo));
		responses[i]->status = 0;
		responses[i]->reason = 0;
		responses[i]->serviceClass = 0;
		responses[i]->toa = 0;
		responses[i]->number = "";
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
	int len;
	cbytes_t ussdRequest;
	bytes_t temp;
	char *newUSSDRequest;
	ussdRequest = (cbytes_t)(data);
	
	temp = malloc(strlen((char *)ussdRequest)*sizeof(char)+1);
	len = utf8_to_gsm8(ussdRequest,strlen((char *)ussdRequest),temp);
	newUSSDRequest = malloc(2*len*sizeof(char)+1);
	gsm_hex_from_bytes(newUSSDRequest,temp, len);
	newUSSDRequest[2*len]='\0';	
	free(temp);
	
	err = at_send_command("AT+CUSD=1,\"%s\",15", newUSSDRequest);
	free(newUSSDRequest);
	if (err != AT_NOERROR) 
		goto error;
		
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

	string	   = ((char **)data)[0];
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
	if(err < 0) goto error;

	/* The first number is discarded */
	err = at_tok_nextint(&line, &response);
	if(err < 0) goto error;

	err = at_tok_nextint(&line, &response);
	if(err < 0) goto error;

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
	if(err != AT_NOERROR)
		goto error;
		
	/* Go online */
	err = at_send_command("AT+CFUN=1");
	if(err != AT_NOERROR)
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
	if(err != AT_NOERROR)
		RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
	else
		RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}


static void requestExplicitCallTransfer(RIL_Token t)
{
	int err = 0;
	err = at_send_command("AT+CHLD=4");
	if(err != AT_NOERROR)
		RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
	else
		RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}


static void requestSetLocationUpdates(void *data, size_t datalen, RIL_Token t)
{
	int err = 0;
	int updates = 0;
	ATResponse *atResponse = NULL;

	updates = ((int *)data)[0] == 1? 2 : 1;

	err = at_send_command_singleline("AT+CREG=%d","+CLIP:",&atResponse, updates);
	if (err != AT_NOERROR)
        goto error;

	at_response_free(atResponse);

	//Always return success for CDMA (for now)
	RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
	return;

error:
	at_response_free(atResponse);
	RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);
}

static void requestLastFailCause(RIL_Token t)
{
	ATResponse *atresponse = NULL;
	int err = 0;
	int response = 0;
	char *tmp = NULL;
	char *line = NULL;

	err = at_send_command_singleline("AT+CEER", "+CEER:", &atresponse);
	if(err != AT_NOERROR) 
		goto error;

	line = atresponse->p_intermediates->line;
	err = at_tok_start(&line);
	if(err < 0) goto error;

	err = at_tok_nextstr(&line, &tmp);
	if(err < 0) goto error;

	err = at_tok_nextint(&line, &response);
	if(err < 0) goto error;

	RIL_onRequestComplete(t, RIL_E_SUCCESS, &response, sizeof(int));
	at_response_free(atresponse);
	return;

error:
	at_response_free(atresponse);
	RIL_onRequestComplete(t, RIL_E_GENERIC_FAILURE, NULL, 0);

}

/*
 * "data" is int *
 * ((int *)data)[0] is == 0 from RUIM/SIM (default)
 * ((int *)data)[0] is == 1 from NV
 */
static void requestCdmaSubscription(void * data, size_t datalen, RIL_Token t) 
{	
	RIL_onRequestComplete(t, RIL_E_SUCCESS, NULL, 0);
}


static pthread_t s_tid_queueRunner;


/** Do post- SIM ready initialization. */
static void onSIMReady(void *p)
{
    int err = 0;
    (void) p;

    /* Check if ME is ready to set preferred message storage */
    checkMessageStorageReady(NULL);

    /* Select message service */
    at_send_command("AT+CSMS=0");

   /* Configure new messages indication
    *  mode = 2 - Buffer unsolicited result code in TA when TA-TE link is
    *             reserved(e.g. in on.line data mode) and flush them to the
    *             TE after reservation. Otherwise forward them directly to
    *             the TE.
    *  mt   = 2 - SMS-DELIVERs (except class 2 messages and messages in the
    *             message waiting indication group (store message)) are
    *             routed directly to TE using unsolicited result code:
    *             +CMT: [<alpha>],<length><CR><LF><pdu> (PDU mode)
    *             Class 2 messages are handled as if <mt> = 1
    *  bm   = 2 - New CBMs are routed directly to the TE using unsolicited
    *             result code:
    *             +CBM: <length><CR><LF><pdu> (PDU mode)
    *  ds   = 1 - SMS-STATUS-REPORTs are routed to the TE using unsolicited
    *             result code: +CDS: <length><CR><LF><pdu> (PDU mode)
    *  bfr  = 0 - TA buffer of unsolicited result codes defined within this
    *             command is flushed to the TE when <mode> 1...3 is entered
    *             (OK response is given before flushing the codes).
    */
    at_send_command("AT+CNMI=2,2,2,1,0");

	/* Select cell broadcast messages: Do not accept messages */
	at_send_command("AT+CSCB=1");
	
	/* Enable automatic timezone update */
	at_send_command("AT+CTZU=1");
	
	/* Enable timezone change reporting */
	at_send_command("AT+CTZR=1");
	
    /* Subscribe to network registration events.
     *  n = 2 - Enable network registration and location information
     *          unsolicited result code +CREG: <stat>[,<lac>,<ci>]
     */
    err = at_send_command("AT+CREG=2");
    if (err != AT_NOERROR) {
        /* Some handsets -- in tethered mode -- don't support CREG=2. */
        at_send_command("AT+CREG=1");
    }

    /* Configure Short Message (SMS) Format
     *  mode = 0 - PDU mode.
     */
    at_send_command("AT+CMGF=0");
	
	/* Enable unsolicited RSSI reporting */
	at_send_command("AT^CURC=1");
	
	/* Enable GPRS reporting */
	at_send_command("AT+CGEREP=1,0");
	
	
	/* Select SMS type */
	at_send_command_singleline("AT+CSMS=1", "+CSMS:", NULL);
}



static void requestNotSupported(RIL_Token t, int request)
{
	LOGD("Request %d is unsupported", request);
	RIL_onRequestComplete(t, RIL_E_REQUEST_NOT_SUPPORTED, NULL, 0);
	return;
}


/*** Callback methods from the RIL library to us ***/

static void processRequest (int request, void *data, size_t datalen, RIL_Token t)
{
    LOGD("%s() %s", __func__, requestToString(request));

    /*
     * These commands won't accept RADIO_NOT_AVAILABLE, so we just return
     * GENERIC_FAILURE if we're not in SIM_STATE_READY.
     */
    RIL_RadioState radio_state = getRadioState();

    if (radio_state != RADIO_STATE_SIM_READY
        && (request == RIL_REQUEST_WRITE_SMS_TO_SIM ||
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
    if ((radio_state == RADIO_STATE_OFF || radio_state == RADIO_STATE_SIM_NOT_READY)
        && !(request == RIL_REQUEST_RADIO_POWER ||
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
    if (radio_state == RADIO_STATE_SIM_LOCKED_OR_ABSENT
        && !(request == RIL_REQUEST_ENTER_SIM_PIN ||
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
			requestGetIMEISV(t);
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

#if 0
		case RIL_REQUEST_CDMA_SET_SUBSCRIPTION:
			requestCdmaSubscription(data, datalen, t);
			break;
#endif
		default:
            LOGW("FIXME: Unsupported request logged: %s",
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

    r = malloc(sizeof(RILRequest));
    memset(r, 0, sizeof(RILRequest));

    /* Formulate a RILRequest and put it in the queue. */
    r->request = request;
    r->data = dupRequestData(request, data, datalen);
    r->datalen = datalen;
    r->token = t;

    if ((err = pthread_mutex_lock(&q->queueMutex)) != 0)
        LOGE("%s() failed to take queue mutex: %s!", __func__, strerror(err));

    /* Queue empty, just throw r on top. */
    if (q->requestList == NULL)
        q->requestList = r;
    else {
        RILRequest *l = q->requestList;
        while (l->next != NULL)
            l = l->next;

        l->next = r;
    }

    if ((err = pthread_cond_broadcast(&q->cond)) != 0)
        LOGE("%s() failed to broadcast queue update: %s!",
            __func__, strerror(err));

    if ((err = pthread_mutex_unlock(&q->queueMutex)) != 0)
        LOGE("%s() failed to release queue mutex: %s!",
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
    LOGI("onSupports() called!");

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
    LOGI("onCancel() called!");
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


static char initializeCommon(void)
{
    int err = 0;
	unsigned int i;
	static const char* const initcmd[] = {

		/* make sure the radio is off */
		"AT+CFUN=7",

		/*  atchannel is tolerant of echo but it must */
		/*  reset and have verbose result codes */
		"ATZV1",

		/*  echo off */
		"ATE0", 

		/*  No auto-answer */
		"ATS0=0",
		
		/*  No auto-answer */
		"AT%AUTOANSWER=0"

		/*  send results */
		"ATQ0",

		/*  check for busy, don't check for dialone */
		"ATX3",

		/*  set DCD depending on service */
		"AT&C1",

		/*  set DTR according to service */
		"AT&D1",

		/*  Extended errors without textual decriptions */
		"AT+CMEE=1",

		/*  Alternating voice/data off */
		"AT+CMOD=0",

		/*  Not muted */
		"AT+CMUT=0",

		/*  detailed rings, unknown */
		"AT+CRC=1;+CR=1",

		/*  caller id = yes */
		"AT+CLIP=1",

		/*  don't hide outgoing callerID */
		"AT+CLIR=0",

		/*  bring up the device (online mode), also resets the stack. Don't do this! Handled elsewhere */
		//"AT+CFUN=1",

		/*  Call Waiting notifications */
		"AT+CCWA=1",

		/*  No connected line identification */
		"AT+COLP=0",

		/*  USSD unsolicited */
		"AT+CUSD=1",

		/*  SMS PDU mode */
		"AT+CMGF=0",

		//"AT+GTKC=2",

		/*  +CSSU unsolicited supp service notifications */
		"AT+CSSN=0,1",

		/*  HEX character set */
		//"AT+CSCS=\"HEX\"",
		"AT+CSCS=\"IRA\"",

		/*  Modem mode */
		"AT+FCLASS=0",

		"AT+CNMI=1,2,2,2,0",
		//"AT+CPPP=1",

		"AT"
	};

    if (at_handshake() < 0) {
        LOG_FATAL("Handshake failed!");
        return 1;
    }

		
	for (i=0; i < sizeof(initcmd) / sizeof(initcmd[0]); i++) {
		err = at_send_command(initcmd[i]);
		if (err != AT_NOERROR) {
			LOGE("Failed sending command '%s'",initcmd[i]);
			return 1;
		}
	}
	
    /* Send the current time of the OS to the module */
    sendTime(NULL); 
	
    return 0;
}

/**
 * Initialize everything that can be configured while we're still in
 * AT+CFUN=0.
 */
static char initializeChannel(void)
{
    int err = 0;
	unsigned int i;
	static const char* const initcmd[] = {

		/* Configure Packet Domain Network Registration Status events
		 *    2 = Enable network registration and location information
		 *        unsolicited result code
		 */

		/*  Network registration events */
		"AT+CREG=2"

		/*  GPRS registration events */
		"AT+CGREG=2",

		"AT+CGEQREQ=1,4,0,0,0,0,2,0,\"0E0\",\"0E0\",3,0,0",
		
		/* Enable unsolicited reports */
		"AT^CURC=1",

		/* Enable GPRS reporting */
		"AT+CGEREP=1,0",
		
	    /* Set phone functionality - 4 = Disable the phone's transmit and receive RF circuits. */
		"AT+CFUN=4",
		
		/* for 3G Preferred */
		"AT^SYSCFG=2,2,3FFFFFFF,1,2"
	};

    LOGD("%s()", __func__);

    setRadioState(RADIO_STATE_OFF);
		
	for (i=0; i < sizeof(initcmd) / sizeof(initcmd[0]); i++) {
		err = at_send_command(initcmd[i]);
		if (err != AT_NOERROR) {
			LOGE("Failed sending command '%s'",initcmd[i]);
			return 1;
		}
	}
	
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

	if (strStartsWith(s, "%CTZV:")
			|| strStartsWith(s,"+CTZV:")
			|| strStartsWith(s,"+CTZDST:")
			|| strStartsWith(s,"+HTCCTZV:")) {
		unsolicitedNitzTime(s);
	} else if (strStartsWith(s,"+CRING:")
			|| strStartsWith(s,"RING")
			|| strStartsWith(s,"NO CARRIER")
			|| strStartsWith(s,"+CCWA")
		  ) {
		RIL_onUnsolicitedResponse (
				RIL_UNSOL_RESPONSE_CALL_STATE_CHANGED,
				NULL, 0);
		enqueueRILEvent(onDataCallListChanged, NULL, NULL);
	} else if (strStartsWith(s,"^RSSI:") ||
			   strStartsWith(s,"%RSSI:")
				) {
		unsolicitedRSSI(s);
	} else if (strStartsWith(s,"^MODE:")) {
		unsolicitedMode(s);
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
    } else if (strStartsWith(s, "+CMTI:")) {
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
			   strStartsWith(s, "Received SMS:")) { 
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
            LOGE("%s() failed to take queue mutex: %s",
                __func__, strerror(err));

        q->closed = 1;
        if ((err = pthread_cond_signal(&q->cond)) != 0)
            LOGE("%s() failed to broadcast queue update: %s",
                __func__, strerror(err));

        if ((err = pthread_mutex_unlock(&q->queueMutex)) != 0)
            LOGE("%s() failed to take queue mutex: %s", __func__,
                 strerror(err));
    }
}


/* Called on command or reader thread */
static void onATReaderClosed()
{
	LOGI("AT channel closed\n");

	setRadioState (RADIO_STATE_UNAVAILABLE);
	signalCloseQueues();
	at_close();
}

/* Called on command thread */
static void onATTimeout()
{
    LOGI("AT channel timeout; restarting..");
    /* Last resort, throw escape on the line, close the channel
       and hope for the best. */
    at_send_escape();

    setRadioState(RADIO_STATE_UNAVAILABLE);
    signalCloseQueues();

    /* TODO We may cause a radio reset here. */
}

static void usage(char *s)
{
    fprintf(stderr, "usage: %s [-p <tcp port>] [-d /dev/tty_device]\n", s);
    exit(-1);
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

    LOGI("%s() starting!", __func__);

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
                /* Program is not controlling terminal -> O_NOCTTY */
                /* Dont care about DCD -> O_NDELAY */
                fd = open(queueArgs->device_path, O_RDWR | O_NOCTTY); /* | O_NDELAY); */
                if (fd >= 0 && !memcmp(queueArgs->device_path, "/dev/ttyA", 9)) {
                    struct termios ios;
                    /* Clear the struct and then call cfmakeraw*/
                    tcflush(fd, TCIOFLUSH);
                    tcgetattr(fd, &ios);
                    memset(&ios, 0, sizeof(struct termios));
                    cfmakeraw(&ios);
                    /* OK now we are in a known state, set what we want*/
                    ios.c_cflag |= CRTSCTS;
                    /* ios.c_cc[VMIN]  = 0; */
                    /* ios.c_cc[VTIME] = 1; */
                    ios.c_cflag |= CS8;
                    tcsetattr(fd, TCSANOW, &ios);
                    tcflush(fd, TCIOFLUSH);
                    tcsetattr(fd, TCSANOW, &ios);
                    tcflush(fd, TCIOFLUSH);
                    tcflush(fd, TCIOFLUSH);
                    cfsetospeed(&ios, B115200);
                    cfsetispeed(&ios, B115200);
                    tcsetattr(fd, TCSANOW, &ios);

                }
            }

            if (fd < 0) {
                LOGE("%s() Failed to open AT channel %s (%s), retrying in %d.",
		    __func__, queueArgs->device_path,
		    strerror(errno), TIMEOUT_SEARCH_FOR_TTY);
                sleep(TIMEOUT_SEARCH_FOR_TTY);
                /* Never returns. */
            }
        }

        /* Reset the blocking mode*/
        fcntl(fd, F_SETFL, 0);
        FD_ZERO(&input);
        FD_SET(fd, &input);
        if (fd >= max_fd)
            max_fd = fd + 1;

        timeout.tv_sec = TIMEOUT_EMRDY;
        timeout.tv_nsec = 0;

        LOGD("%s() waiting for emrdy...", __func__);
        n = select(max_fd, &input, NULL, NULL,(struct timeval*) &timeout);

        if (n < 0) {
            LOGE("%s() Select error", __func__);
            return NULL;
        } else if (n == 0)
            LOGE("%s() timeout, go ahead anyway(might work)...", __func__);
        else {
            memset(start, 0, MAX_BUF);
            safe_read(fd, start, MAX_BUF-1);

            if (start == NULL) {
                LOGD("%s() Eiii empty string", __func__);
                tcflush(fd, TCIOFLUSH);
                FD_CLR(fd, &input);
                close(fd);
                continue;
            }

            if (strstr(start, "EMRDY") == NULL) {
                LOGD("%s() Eiii this was not EMRDY: %s", __func__, start);
                tcflush(fd, TCIOFLUSH);
                FD_CLR(fd, &input);
                close(fd);
                continue;
            }

            LOGD("%s() Got EMRDY", __func__);
        }

        ret = at_open(fd, onUnsolicited);

        if (ret < 0) {
            LOGE("%s() AT error %d on at_open", __func__, ret);
            at_close();
            continue;
        }

        at_set_on_reader_closed(onATReaderClosed);
        at_set_on_timeout(onATTimeout);

        q = &s_requestQueue;

		if(initializeCommon()) {
            LOGE("%s() Failed to initialize channel!", __func__);
            at_close();
            continue;
        }
		
		q->closed = 0;
		if (initializeChannel()) {
			LOGE("%s() Failed to initialize channel!", __func__);
			at_close();
			continue;
		}

		at_make_default_channel();

        LOGE("%s() Looping the requestQueue!", __func__);
        for (;;) {
            RILRequest *r;
            RILEvent *e;
            struct timespec ts;
            int err;

            memset(&ts, 0, sizeof(ts));

        if ((err = pthread_mutex_lock(&q->queueMutex)) != 0)
            LOGE("%s() failed to take queue mutex: %s!",
                __func__, strerror(err));

            if (q->closed != 0) {
                LOGW("%s() AT Channel error, attempting to recover..", __func__);
                if ((err = pthread_mutex_unlock(&q->queueMutex)) != 0)
                    LOGE("Failed to release queue mutex: %s!", strerror(err));
                break;
            }

            while (q->closed == 0 && q->requestList == NULL &&
                q->eventList == NULL) {
                if ((err = pthread_cond_wait(&q->cond, &q->queueMutex)) != 0)
                    LOGE("%s() failed broadcast queue cond: %s!",
                        __func__, strerror(err));
            }

            /* eventList is prioritized, smallest abstime first. */
            if (q->closed == 0 && q->requestList == NULL && q->eventList) {
                int err = 0;
                err = pthread_cond_timedwait(&q->cond, &q->queueMutex, &q->eventList->abstime);
                if (err && err != ETIMEDOUT)
                    LOGE("%s() timedwait returned unexpected error: %s",
		        __func__, strerror(err));
            }

            if (q->closed != 0) {
                if ((err = pthread_mutex_unlock(&q->queueMutex)) != 0)
                    LOGE("%s(): Failed to release queue mutex: %s!",
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
                LOGE("%s(): Failed to release queue mutex: %s!",
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
        LOGE("%s() Re-opening after close", __func__);
    }
    return NULL;
}

const RIL_RadioFunctions *RIL_Init(const struct RIL_Env *env, int argc,
                                   char **argv)
{
    int opt;
    int port = -1;
    char *loophost = NULL;
    const char *device_path = NULL;
    struct queueArgs *queueArgs;
    pthread_attr_t attr;

    s_rilenv = env;

    LOGD("%s() entering...", __func__);

    while (-1 != (opt = getopt(argc, argv, "z:p:d:"))) {
        switch (opt) {
            case 'z':
                loophost = optarg;
                LOGD("%s() Using loopback host %s..", __func__, loophost);
                break;

            case 'p':
                port = atoi(optarg);
                if (port == 0) {
                    usage(argv[0]);
                    return NULL;
                }
                LOGD("%s() Opening loopback port %d", __func__, port);
                break;

            case 'd':
                device_path = optarg;
                LOGD("%s() Opening tty device %s", __func__, device_path);
                break;

            default:
                usage(argv[0]);
                return NULL;
        }
    }

    if (port < 0 && device_path == NULL) {
        usage(argv[0]);
        return NULL;
    }

    queueArgs = malloc(sizeof(struct queueArgs));
    memset(queueArgs, 0, sizeof(struct queueArgs));

    queueArgs->device_path = device_path;
    queueArgs->port = port;
    queueArgs->loophost = loophost;

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    pthread_create(&s_tid_queueRunner, &attr, queueRunner, queueArgs);

    return &s_callbacks;
}
