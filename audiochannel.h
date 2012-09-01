/*
**
** Copyright (C) 2012 Eduardo José Tagle <ejtagle@tutopia.com>
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
**
*/
#ifndef _AUDIOCHANNEL_H
#define _AUDIOCHANNEL_H 1

#include "audioqueue.h"
#include <pthread.h>

struct GsmAudioTunnel {

    // 3G voice modem
    int fd;                         // Voice data serial port handler

    // Common properties
    unsigned int frame_size;        // Frame size
    unsigned int sampling_rate;     // Sampling rate
    unsigned int bits_per_sample;   // Bits per sample. valid values = 16/8
	pthread_t modem_t;				// 3G modem playback/record thread
	int ismuted;					// If audio record is muted
	
    // Playback
    void* play_strm;                // Playback stream
    volatile int play_thread_exited;// If play thread has exited
    void* play_buf;                 // Pointer to the playback buffer
	struct AudioQueue play_q;		// Audio playback queue
	
    // Record
    void* rec_strm;                 // Record stream
    volatile int rec_thread_exited; // If record thread has exited
    void* rec_buf;                  // Pointer to the recording buffer
	struct AudioQueue rec_q;		// Audio record queue
	
};

#define GSM_AUDIO_CHANNEL_STATIC_INIT { -1, 0,0,0,0,0,0, 0,0,0,{0} ,0,0,0,{0} }

#ifdef __cplusplus
extern "C" {
#endif

int gsm_audio_tunnel_start(struct GsmAudioTunnel *stream,
    const char* gsmvoicechannel,
    unsigned int sampling_rate,
    unsigned int frame_size,
    unsigned int bits_per_sample);

int gsm_audio_tunnel_stop(struct GsmAudioTunnel *stream);

int gsm_audio_tunnel_running(struct GsmAudioTunnel *ctx);

int gsm_audio_tunnel_mute(struct GsmAudioTunnel *stream, int muteit);

#ifdef __cplusplus
}
#endif

#endif
