/**
 * Copyright (C) 2010 Eduardo José Tagle <ejtagle@tutopia.com>
 *
 * Since deeply inspired from portaudio dev port:
 * Copyright (C) 2009-2010 r3gis (http://www.r3gis.fr)
 * Copyright (C) 2008-2009 Teluu Inc. (http://www.teluu.com)
 * Copyright (C) 2003-2008 Benny Prijono <benny@prijono.org>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/epoll.h>
#include <semaphore.h>
#include <signal.h>
#include <linux/socket.h>
#include <sys/socket.h> 
#include <errno.h>
#include <stddef.h>
#include <sys/time.h>
#include <sys/select.h>

#include "audiochannel.h"

#define LOG_NDEBUG 0
#define LOG_TAG "RILAudioCh"
#include <utils/Log.h>

#include <system/audio.h>
#include <media/AudioRecord.h>
#include <media/AudioSystem.h>
#include <media/AudioTrack.h>

// ---- Android sound streaming ----

/* Audio oversampling factor. To use higher sampling rates reduces latency quite a bit ... */
#define AUDIO_OVERSAMPLING 1

// #define CHECK_MEM_OVERRUN 1
#define AUDIOCHANNEL_DEBUG 0
#if AUDIOCHANNEL_DEBUG
#  define  D(...)   ALOGD(__VA_ARGS__)
#else
#  define  D(...)   ((void)0)
#endif 

static inline int labs(int x)
{
	return (x < 0) ? -x : x;
}

/* modemAudioIOThread:
    Output/inputs an audio frame (160 samples) to the 3G audio port of the cell modem
	We need to write to be able to read from modem
*/
static void* modemAudioIOThread(void* data)
{
	int            n;
    int            max_fd;
    fd_set         input;
    struct timeval timeout;

	struct GsmAudioTunnel* ctx = (struct GsmAudioTunnel*)data;
    int res = 0;
	int bps = (ctx->bits_per_sample/8);
	int frame_bytes = ctx->frame_size * bps;

	ALOGD("modemAudioIOThread begin");
						
	// Discard all pending data*/
	ALOGD("Discarding old data....");
	tcflush(ctx->fd, TCIOFLUSH); 
	ALOGD("Discarding old data... Done");
						
	// Get audio from the queue and push it into the modem
	while (AudioQueue_isrunning(&ctx->rec_q) &&
		   AudioQueue_isrunning(&ctx->play_q)) {

		// Write audio to the 3G modem audio port in 320 bytes chunks... This is
		//  required by huawei modems...
		D("[T]Before AudioQueue_get");
		res = AudioQueue_get(&ctx->rec_q,ctx->play_buf,ctx->frame_size, ctx->timeout);
		D("[T]After AudioQueue_get: 0x%04x, 0x%04x, 0x%04x, 0x%04x, 0x%04x",((short*)ctx->play_buf)[0],((short*)ctx->play_buf)[1],((short*)ctx->play_buf)[2],((short*)ctx->play_buf)[3],((short*)ctx->play_buf)[4] );
	
#ifdef CHECK_MEM_OVERRUN
		if (((int*)ctx->rec_buf)[-1                                        ] != 0x1A3B5C7D) {
			ALOGE("recbuf: Corruption at start: 0x%08x",((int*)ctx->rec_buf)[-1]);
		}
		if (((int*)ctx->rec_buf)[(ctx->frame_size * AUDIO_OVERSAMPLING * (ctx->bits_per_sample/8))>>2] != 0xD7C5B3A1) {
			ALOGE("recbuf: Corruption at end: 0x%08x",((int*)ctx->rec_buf)[(ctx->frame_size * (ctx->bits_per_sample/8))>>2]);
		}
		if (((int*)ctx->play_buf)[-1                                        ] != 0x1A3B5C7D) {
			ALOGE("playbuf: Corruption at start: 0x%08x",((int*)ctx->play_buf)[-1]);
		}
		if (((int*)ctx->play_buf)[(ctx->frame_size * AUDIO_OVERSAMPLING *  (ctx->bits_per_sample/8))>>2] != 0xD7C5B3A1) {
			ALOGE("playbuf: Corruption at end: 0x%08x",((int*)ctx->play_buf)[(ctx->frame_size * (ctx->bits_per_sample/8))>>2]);
		}
#endif

		if (!AudioQueue_isrunning(&ctx->rec_q) || 
			!AudioQueue_isrunning(&ctx->play_q))
			break;

		/* Fill missing samples with silence, if needed */
		if ((int)ctx->frame_size > res) {
			memset((char*)ctx->play_buf + res * bps, 0, (ctx->frame_size - res) * bps);
		}

#ifdef CHECK_MEM_OVERRUN
		if (((int*)ctx->rec_buf)[-1                                        ] != 0x1A3B5C7D) {
			ALOGE("recbuf: Corruption at start: 0x%08x",((int*)ctx->rec_buf)[-1]);
		}
		if (((int*)ctx->rec_buf)[(ctx->frame_size * AUDIO_OVERSAMPLING *  (ctx->bits_per_sample/8))>>2] != 0xD7C5B3A1) {
			ALOGE("recbuf: Corruption at end: 0x%08x",((int*)ctx->rec_buf)[(ctx->frame_size * (ctx->bits_per_sample/8))>>2]);
		}
		if (((int*)ctx->play_buf)[-1                                        ] != 0x1A3B5C7D) {
			ALOGE("playbuf: Corruption at start: 0x%08x",((int*)ctx->play_buf)[-1]);
		}
		if (((int*)ctx->play_buf)[(ctx->frame_size * AUDIO_OVERSAMPLING *  (ctx->bits_per_sample/8))>>2] != 0xD7C5B3A1) {
			ALOGE("playbuf: Corruption at end: 0x%08x",((int*)ctx->play_buf)[(ctx->frame_size * (ctx->bits_per_sample/8))>>2]);
		}
#endif

		// Write audio chunk
		D("[T]Before write");
		res = write(ctx->fd, ctx->play_buf, frame_bytes);
		D("[T]After write: res: %d",res);
				  
		if (!AudioQueue_isrunning(&ctx->rec_q) || 
			!AudioQueue_isrunning(&ctx->play_q) ||
			 res < 0)
			break;

		// Read data from the modem
		D("[T]Before Select");

		do {
			// Initialize the input set
			FD_ZERO(&input);
			FD_SET(ctx->fd, &input);
			max_fd = ctx->fd + 1;

			// Initialize the timeout structure: 40ms is enough for this waiting
			timeout.tv_sec  = 0;
			timeout.tv_usec = 40000;

			// Do the select
			n = select(max_fd, &input, NULL, NULL, &timeout);

		} while (n == 0 &&
			AudioQueue_isrunning(&ctx->rec_q) &&
			AudioQueue_isrunning(&ctx->play_q));
		
		D("[T]After Select");		
		
		/* See if there was an error */
		if (!AudioQueue_isrunning(&ctx->rec_q) || 
			!AudioQueue_isrunning(&ctx->play_q) ||
			 n < 0)
			break;

		/* If something to read, read it */
		if (FD_ISSET(ctx->fd, &input)) {
			D("[T]Before read");
			res = read(ctx->fd, ctx->rec_buf, frame_bytes);
			D("[T]After read: res: %d",res);

			if (!AudioQueue_isrunning(&ctx->play_q) || 
				!AudioQueue_isrunning(&ctx->rec_q) || 
				 res < 0)
				break;

#ifdef CHECK_MEM_OVERRUN
			if (((int*)ctx->rec_buf)[-1                                        ] != 0x1A3B5C7D) {
				ALOGE("recbuf: Corruption at start: 0x%08x",((int*)ctx->rec_buf)[-1]);
			}
			if (((int*)ctx->rec_buf)[(ctx->frame_size * AUDIO_OVERSAMPLING *  (ctx->bits_per_sample/8))>>2] != 0xD7C5B3A1) {
				ALOGE("recbuf: Corruption at end: 0x%08x",((int*)ctx->rec_buf)[(ctx->frame_size * (ctx->bits_per_sample/8))>>2]);
			}
			if (((int*)ctx->play_buf)[-1                                        ] != 0x1A3B5C7D) {
				ALOGE("playbuf: Corruption at start: 0x%08x",((int*)ctx->play_buf)[-1]);
			}
			if (((int*)ctx->play_buf)[(ctx->frame_size * AUDIO_OVERSAMPLING *  (ctx->bits_per_sample/8))>>2] != 0xD7C5B3A1) {
				ALOGE("playbuf: Corruption at end: 0x%08x",((int*)ctx->play_buf)[(ctx->frame_size * (ctx->bits_per_sample/8))>>2]);
			} 
#endif

			// If muted, silence audio
			if (ctx->ismuted) {
				memset( ctx->rec_buf, 0, frame_bytes);
			}

#ifdef CHECK_MEM_OVERRUN
			if (((int*)ctx->rec_buf)[-1                                        ] != 0x1A3B5C7D) {
				ALOGE("recbuf: Corruption at start: 0x%08x",((int*)ctx->rec_buf)[-1]);
			}
			if (((int*)ctx->rec_buf)[(ctx->frame_size * AUDIO_OVERSAMPLING *  (ctx->bits_per_sample/8))>>2] != 0xD7C5B3A1) {
				ALOGE("recbuf: Corruption at end: 0x%08x",((int*)ctx->rec_buf)[(ctx->frame_size * (ctx->bits_per_sample/8))>>2]);
			}
			if (((int*)ctx->play_buf)[-1                                        ] != 0x1A3B5C7D) {
				ALOGE("playbuf: Corruption at start: 0x%08x",((int*)ctx->play_buf)[-1]);
			}
			if (((int*)ctx->play_buf)[(ctx->frame_size * AUDIO_OVERSAMPLING *  (ctx->bits_per_sample/8))>>2] != 0xD7C5B3A1) {
				ALOGE("playbuf: Corruption at end: 0x%08x",((int*)ctx->play_buf)[(ctx->frame_size * (ctx->bits_per_sample/8))>>2]);
			}
#endif

			// EM770W firmware corrupts received audio... Try to workaround the damage...

			// 1st Pass: If received less data than requested, this means voice data corruption. 
			// More frequent than you could think. We must compensate it, or we end with garbled 
			// voice...
			if (res < frame_bytes) {
				int p;
				signed char* b;
				int tf;

				
				// Try to reconstruct data . Determine variance of low and high nibbles.
				b = (signed char*)ctx->rec_buf;
				tf = frame_bytes - res;
				for (p = 0; p < 317 && tf!=0; p+=2) {
					if (labs(b[p+2] - b[p]) < labs(b[p+1] - b[p+3]) ) {
						/* Probably, this is the point ... Insert an space */
						memmove(b+p+1,b+p,320-p-1);
						tf--;
						p+=2;
					}
				}
			}
			
			/* 2nd pass: Detect endianness inversions and correct them */
			{
				signed short* d = (signed short*)ctx->rec_buf;
				signed short ss, sp = 0, s = d[2]; // Handle first sample by reflection
				int todo = 160;
				
				while (todo--) {
					sp = s; 	/* keep previous sample */
					s  = *d++; 	/* Calculate the other possible samples */
					ss = (((unsigned short)s) << 8U) | ((((unsigned short)s) >> 8U) & 0xFFU);
					
					/* Choose the one that creates less volume difference */
					if (labs(sp - ss  ) < labs(sp - s ) ) {
						/* Inverted is closer to original. Keep inverted */
						s = ss;
						d[-1] = s;
					} 
				}
			}
			
			/* 3rd pass: Remove clicks - we use a 3 sample window to predict and correct 1-sample clicks...*/
			{
				signed short* d = (signed short*)ctx->rec_buf;
				signed short spp = 0, sp = *d++, s = *d++;
				signed short p;
				int todo = 158;
				
				while (todo--) {
					/* Store previous and get new sample */
					spp = sp;
					sp = s; 	
					s  = *d++; 	
					
					/* Estimate medium */
					p = (s + spp) / 2;
					
					/* If predicted is very different from real, assume noise and replace it */
					if ( labs( sp - p ) > labs(p >> 2) ) {
						sp = p;
						d[-2] = sp;
					}
				}
			}

			/* 4th pass: Remove 6 Sample clicks... The modem also sometimes creates them. Detect and remove them if possible */
			{
				signed short* d = (signed short*)ctx->rec_buf;
				signed short sp = 0, s = *d++;
				signed short p;
				int todo = 154;
				
				while (todo--) {
					/* Store previous and get new sample */
					sp = s; 	
					s  = *d++; 	
					
					/* If a 4 times jump in value is detected, and 6 samples later we are on track, assume it is a modem generated 
					   click and remove it - We prefer to remove in excess here*/
					if (labs(s) > labs(sp  )*4 &&
						labs(s) > labs(d[6])*4 ) {
						
						/* Detected an undesired click, remove it! */
						int step = ((d[6] - sp) << (16 - 3));
						int x = sp << 16;
						x+= step;
						s = d[-1] = x >> 16;
						x+= step;
						d[ 0] = x >> 16;
						x+= step;
						d[ 1] = x >> 16; 
						x+= step;
						d[ 2] = x >> 16; 
						x+= step;
						d[ 3] = x >> 16; 
						x+= step;
						d[ 4] = x >> 16;
						x+= step;
						d[ 5] = x >> 16;
					}
				}
			}
			
#if LOG_MODEM_AUDIO
			/* Log audio into SD */
			write(ctx->logfd, ctx->rec_buf, frame_bytes);
#endif

			// Write it to the audio queue
			D("[T]Before AudioQueue_add: %04x %04x %04x %04x %04x",((short*)ctx->rec_buf)[0] & 0xFFFF,((short*)ctx->rec_buf)[1] & 0xFFFF,((short*)ctx->rec_buf)[2] & 0xFFFF,((short*)ctx->rec_buf)[3] & 0xFFFF,((short*)ctx->rec_buf)[4] & 0xFFFF );
			AudioQueue_add(&ctx->play_q, ctx->rec_buf, ctx->frame_size);
			D("[T]After AudioQueue_add");

#ifdef CHECK_MEM_OVERRUN
			if (((int*)ctx->rec_buf)[-1                                        ] != 0x1A3B5C7D) {
				ALOGE("recbuf: Corruption at start: 0x%08x",((int*)ctx->rec_buf)[-1]);
			}
			if (((int*)ctx->rec_buf)[(ctx->frame_size * AUDIO_OVERSAMPLING *  (ctx->bits_per_sample/8))>>2] != 0xD7C5B3A1) {
				ALOGE("recbuf: Corruption at end: 0x%08x",((int*)ctx->rec_buf)[(ctx->frame_size * (ctx->bits_per_sample/8))>>2]);
			}
			if (((int*)ctx->play_buf)[-1                                        ] != 0x1A3B5C7D) {
				ALOGE("playbuf: Corruption at start: 0x%08x",((int*)ctx->play_buf)[-1]);
			}
			if (((int*)ctx->play_buf)[(ctx->frame_size * AUDIO_OVERSAMPLING *  (ctx->bits_per_sample/8))>>2] != 0xD7C5B3A1) {
				ALOGE("playbuf: Corruption at end: 0x%08x",((int*)ctx->play_buf)[(ctx->frame_size * (ctx->bits_per_sample/8))>>2]);
			}
#endif
		}
		
	};
	
	ALOGD("modemAudioIOThread ended");
    return NULL;
}

/* Called with audio sampled from mic */
static void AndroidRecorderCallback(int event, void* userData, void* info)
{
    struct GsmAudioTunnel *ctx = (struct GsmAudioTunnel*) userData;
    android::AudioRecord::Buffer* uinfo = (android::AudioRecord::Buffer*) info;
	int bps,frames;

    if(!ctx || !uinfo || event != android::AudioRecord::EVENT_MORE_DATA)
		return;

	if (!AudioQueue_isrunning(&ctx->rec_q))
        goto on_break;

	/* Bytes per sample */
	bps = ctx->bits_per_sample/8;
	
	/* Calculate total frames */
	frames = uinfo->size / bps;
	
#if AUDIO_OVERSAMPLING > 1
	/* Subsample */
	{
		short *src = (short *)uinfo->raw + frames - 1;
		short *dst = (short *)uinfo->raw + frames - 1;
		int p;
		short v = *src++,vold = 0;
		for (p = 0; p < ctx->frame_size-1; p++) {
			int x,d,s;

			// Current and target values
			vold = v;
			v = *src++;
			
			// Calculate delta and start value
			d = (v - vold) * (65536 / AUDIO_OVERSAMPLING);
			s =  vold << 16;
			
			// Interpolate values (linear interpolation)
			for (x = 0; x < AUDIO_OVERSAMPLING; x++) {
				*dst++ = s >> 16;
				s += d;
			}
		}
		
		// Last sample can't be interpolated...
		for (x = 0; x < AUDIO_OVERSAMPLING; x++) {
			*dst++ = v;
		}
	}
#endif
		
	// Post data into the recording queue. Queue should self adapt and adjust sampling rate
	D("[A]Before AudioQueue_add");
	uinfo->size = AudioQueue_add(&ctx->rec_q, uinfo->raw, frames) * bps;
	D("[A]After AudioQueue_add");
    return;

on_break:
	if (!ctx->rec_thread_exited) {
		ALOGD("Record thread stopped");
		ctx->rec_thread_exited = 1;
	}
    return;
}

/* Called to get audio samples to playback */
static void AndroidPlayerCallback( int event, void* userData, void* info)
{
    struct GsmAudioTunnel *ctx = (struct GsmAudioTunnel*) userData;
    android::AudioTrack::Buffer* uinfo = (android::AudioTrack::Buffer*) info;
	int bps,frames;
	
    if (!ctx || !uinfo || event != android::AudioTrack::EVENT_MORE_DATA)
        return;
	
    if (!AudioQueue_isrunning(&ctx->play_q))
        goto on_break;

	// Read data from the Playback audioqueue
	D("[A]Before AudioQueue_get");
	bps = ctx->bits_per_sample/8;
	uinfo->size = (frames = AudioQueue_get(&ctx->play_q, uinfo->raw, uinfo->size / bps, ctx->timeout)) * bps;
	D("[A]After AudioQueue_get");
	
#if AUDIO_OVERSAMPLING > 1
	/* Oversample to send them to the Android playback channel*/
	if (frames) {
		short *src = uinfo->raw;
		short *dst = uinfo->raw;
		int p;
		short v = *src++,vold = 0;
		for (p = 0; p < frames-1; p++) {
			int x,d,s;

			// Current and target values
			vold = v;
			v = *src++;
			
			// Calculate delta and start value
			d = (v - vold) * (65536 / AUDIO_OVERSAMPLING);
			s =  vold << 16;
			
			// Interpolate values (linear interpolation)
			for (x = 0; x < AUDIO_OVERSAMPLING; x++) {
				*dst++ = s >> 16;
				s += d;
			}
		}
		
		// Last sample can't be interpolated...
		for (x = 0; x < AUDIO_OVERSAMPLING; x++) {
			*dst++ = v;
		}
	}
#endif
	
    return;

on_break:
	if (!ctx->play_thread_exited) {
		ALOGD("Play thread stopped");
		ctx->play_thread_exited = 1;
	}
	
	/* Silence output if we are not running */
	memset(uinfo->raw, 0, uinfo->size);
    return;
}

 //AT^DDSETEX=2

int gsm_audio_tunnel_start(struct GsmAudioTunnel *ctx,const char* gsmvoicechannel,unsigned int sampling_rate,unsigned int frame_size,unsigned int bits_per_sample)
{
	pthread_attr_t modem_attr;
    struct termios newtio;
	int create_result = 0;
	size_t playBuffSize = 0;
	size_t playNotifyBuffSize = 0;
	size_t recBuffSize = 0;
	size_t recNotifyBuffSize = 0;
	int play_qsize;
	int rec_qsize;

	audio_format_t format = (bits_per_sample > 8) 
        ? AUDIO_FORMAT_PCM_16_BIT
        : AUDIO_FORMAT_PCM_8_BIT;

    /* If already running, dont do it again */
    if (AudioQueue_isrunning(&ctx->rec_q) ||
		AudioQueue_isrunning(&ctx->play_q))
        return 0;

    memset(ctx,0,sizeof(struct GsmAudioTunnel));
	ctx->fd = -1;

    ctx->sampling_rate = sampling_rate;
    ctx->frame_size = frame_size;
    ctx->bits_per_sample = bits_per_sample;
	ctx->timeout = 1000 * frame_size / sampling_rate;  

	ALOGD("Opening GSM voice channel '%s', sampling_rate:%u hz, frame_size:%u, bits_per_sample:%u  ...",
        gsmvoicechannel,sampling_rate,frame_size,bits_per_sample);

#if LOG_MODEM_AUDIO
	ctx->logfd = open("/sdcard/modemlog", O_RDWR);
#endif
		
    // Compute buffer sizes for record and playback
#if 0
    playBuffSize = 0;
    android::AudioSystem::getInputBufferSize(
                    ctx->sampling_rate * AUDIO_OVERSAMPLING, // Samples per second
                    format,
                    AUDIO_CHANNEL_IN_MONO,
                    &playBuffSize);
	recBuffSize = playBuffSize;
#else
	android::AudioRecord::getMinFrameCount((int*)&recBuffSize,
	                    ctx->sampling_rate * AUDIO_OVERSAMPLING, // Samples per second
						format,
						1);
						
    //android::AudioSystem::getInputBufferSize(
    //                ctx->sampling_rate, // Samples per second
    //                format,
    //                AUDIO_CHANNEL_IN_MONO,
    //                &recBuffSize);
	//recBuffSize	<<= 1; // Convert to bytes
	//recBuffSize <<= 1; // Convert to bytes
	//while (recBuffSize < frame_size)  recBuffSize <<= 1;
	//while (playBuffSize < frame_size) playBuffSize <<= 1;
						
	android::AudioTrack::getMinFrameCount((int*)&playBuffSize,
						AUDIO_STREAM_VOICE_CALL,
	                    ctx->sampling_rate * AUDIO_OVERSAMPLING); // Samples per second
	
	// Do not accept less than the frame size ... Makes no point going lower than that
	if (playBuffSize < frame_size)
		playBuffSize = frame_size;
	if (recBuffSize < frame_size)
		recBuffSize = frame_size;
	
	// Compute minimum playback queue size as a power of 2
	play_qsize = 0;
	while ((1 << play_qsize) < playBuffSize) play_qsize++;
	play_qsize += 2; // And quad it
	
	// Compute minimum record queue size as a power of 2
	rec_qsize = 0;
	while ((1 << rec_qsize) < recBuffSize) rec_qsize++;
	rec_qsize += 2; // And quad it
	
#endif

    // We use 2 * size of input/output buffer for ping pong use of record/playback buffers.
    playNotifyBuffSize = playBuffSize;
	playBuffSize <<= 1;
	recNotifyBuffSize = recBuffSize;
    recBuffSize <<= 1;
	ALOGD("play samples: %d [q:%d], record samples: %d [q:%d]",playNotifyBuffSize,play_qsize,recNotifyBuffSize,rec_qsize);
		
	// Init the audioqueues
	if (AudioQueue_init(&ctx->play_q,play_qsize,bits_per_sample>>3) < 0) {
		ALOGE("Could not init Playback AudioQueue");
		goto error;
	}
	if (AudioQueue_init(&ctx->rec_q,rec_qsize,bits_per_sample>>3) < 0) {
		ALOGE("Could not init Record AudioQueue");
		goto error;
	}
	
	ALOGD("Opening voice channel....");
	
/*	if (echocancel_init(&ctx->echo, recNotifyBuffSize ) < 0) {
		ALOGE("Could not init Record AudioQueue");
		goto error;
	}
*/	
    // Open the device(com port) in blocking mode 
    ctx->fd = open(gsmvoicechannel, O_RDWR | O_NOCTTY);
    if (ctx->fd < 0) {
		ALOGE("Could not open '%s'",gsmvoicechannel);
		goto error;
    }
	 	
    // Configure it to get data as raw as possible
    tcgetattr(ctx->fd, &newtio );
    newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR | IGNBRK | IGNCR | IXOFF;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VMIN]=1;
    newtio.c_cc[VTIME]=1;
    tcsetattr(ctx->fd,TCSANOW, &newtio);

	ALOGD("Creating streams....");
#ifdef CHECK_MEM_OVERRUN
    ctx->rec_buf = malloc(8 + ctx->frame_size * (ctx->bits_per_sample/8) * AUDIO_OVERSAMPLING);
    if (!ctx->rec_buf) {
		ALOGE("Failed to allocate buffer for playback");
		goto error;
    }

    ctx->play_buf = malloc(8 + ctx->frame_size * (ctx->bits_per_sample/8) * AUDIO_OVERSAMPLING);
    if (!ctx->play_buf) {
		ALOGE("Failed to allocate buffer for record");
		goto error;
    }

	ctx->rec_buf = (int*)ctx->rec_buf + 1;
	((int*)ctx->rec_buf)[-1                                        ] = 0x1A3B5C7D;
	((int*)ctx->rec_buf)[(ctx->frame_size * AUDIO_OVERSAMPLING * (ctx->bits_per_sample/8))>>2] = 0xD7C5B3A1;
	ctx->play_buf = (int*)ctx->play_buf + 1;
	((int*)ctx->play_buf)[-1                                        ] = 0x1A3B5C7D;
	((int*)ctx->play_buf)[(ctx->frame_size * AUDIO_OVERSAMPLING * (ctx->bits_per_sample/8))>>2] = 0xD7C5B3A1;

#else
    ctx->rec_buf = malloc(ctx->frame_size * (ctx->bits_per_sample/8) * AUDIO_OVERSAMPLING);
    if (!ctx->rec_buf) {
		ALOGE("Failed to allocate buffer for playback");
		goto error;
    }

    ctx->play_buf = malloc(ctx->frame_size * (ctx->bits_per_sample/8) * AUDIO_OVERSAMPLING);
    if (!ctx->play_buf) {
		ALOGE("Failed to allocate buffer for record");
		goto error;
    }
#endif

    // Create audio record channel
    ctx->rec_strm = new android::AudioRecord();
    if(!ctx->rec_strm) {
		ALOGE("fail to create audio record");
		goto error;
    }
	
    // Create audio playback channel
    ctx->play_strm = new android::AudioTrack();
    if(!ctx->play_strm) {
		ALOGE("Failed to create AudioTrack");
		goto error;
    }

    // Unmute microphone
    // android::AudioSystem::muteMicrophone(false);
    create_result = ((android::AudioRecord*)ctx->rec_strm)->set(
                    AUDIO_SOURCE_MIC,
                    ctx->sampling_rate * AUDIO_OVERSAMPLING,
                    format,
                    AUDIO_CHANNEL_IN_MONO,
                    recBuffSize,
					android::AudioRecord::RECORD_AGC_ENABLE, 	//flags
                    &AndroidRecorderCallback,
                    (void *) ctx,
                    recNotifyBuffSize, // Notification frames
                    false,
                    0);

    if(create_result != android::NO_ERROR){
		ALOGE("fail to check audio record : error code %d", create_result);
		goto error;
    }

    if(((android::AudioRecord*)ctx->rec_strm)->initCheck() != android::NO_ERROR) {
		ALOGE("fail to check audio record : buffer size is : %d, error code : %d", recBuffSize, ((android::AudioRecord*)ctx->rec_strm)->initCheck() );
		goto error;
    }

    // android::AudioSystem::setMasterMute(false);
    create_result = ((android::AudioTrack*)ctx->play_strm)->set(
                    AUDIO_STREAM_VOICE_CALL,
                    ctx->sampling_rate * AUDIO_OVERSAMPLING, //this is sample rate in Hz (16000 Hz for example)
                    format,
                    AUDIO_CHANNEL_OUT_MONO, //For now this is mono (we expect 1)
                    playBuffSize,
					AUDIO_OUTPUT_FLAG_NONE, //flags
                    &AndroidPlayerCallback,
                    (void *) ctx,
                    playNotifyBuffSize,
                    0,
                    false,
                    0);

    if(create_result != android::NO_ERROR){
		ALOGE("fail to check audio record : error code %d", create_result);
		goto error;
    }

    if(((android::AudioTrack*)ctx->play_strm)->initCheck() != android::NO_ERROR) {
		ALOGE("fail to check audio playback : buffer size is : %d, error code : %d", playBuffSize, ((android::AudioTrack*)ctx->play_strm)->initCheck() );
		goto error;
    }

    /* Save the current audio routing setting, then switch it to earpiece. */
    // android::AudioSystem::getMode(&ctx->saved_audio_mode);
    // android::AudioSystem::getRouting(ctx->saved_audio_mode, &ctx->saved_audio_routing);
    // android::AudioSystem::setRouting(ctx->saved_audio_mode,
    //                      android::AudioSystem::ROUTE_EARPIECE,
    //                      android::AudioSystem::ROUTE_ALL);

	ALOGD("Starting streaming...");

    if (ctx->play_strm) {
        ((android::AudioTrack*)ctx->play_strm)->start();
    }

    if (ctx->rec_strm) {
        ((android::AudioRecord*)ctx->rec_strm)->start();
    }

	// Create the playback thread
	pthread_attr_init(&modem_attr);	
	if (pthread_create(&ctx->modem_t,&modem_attr,modemAudioIOThread,ctx) < 0) {
		ALOGE("Failed to start modemAudioIO Thread");
error:
		AudioQueue_end(&ctx->rec_q);
		AudioQueue_end(&ctx->play_q);
        if (ctx->play_strm) delete ((android::AudioTrack*)ctx->play_strm);
        if (ctx->rec_strm) delete ((android::AudioRecord*)ctx->rec_strm);
#ifdef CHECK_MEM_OVERRUN
        if (ctx->play_buf) free(((int*)ctx->play_buf)-1);
        if (ctx->rec_buf) free(((int*)ctx->rec_buf)-1);
#else
        if (ctx->play_buf) free(ctx->play_buf);
        if (ctx->rec_buf) free(ctx->rec_buf);
#endif
        if (ctx->fd) close(ctx->fd);
#if LOG_MODEM_AUDIO
		if (ctx->logfd) close(ctx->logfd);
#endif

        return -1;
	}

	ALOGD("Done");

    // OK, done
    return 0;
}

/* API: mute audio record channel */
int gsm_audio_tunnel_mute(struct GsmAudioTunnel *ctx, int muteit)
{
	ctx->ismuted = muteit;
	return 0;
}

/* API: query if tunnel is running */
int gsm_audio_tunnel_running(struct GsmAudioTunnel *ctx)
{
	if (AudioQueue_isrunning(&ctx->rec_q) || 
		AudioQueue_isrunning(&ctx->play_q))
        return 1;
	return 0;
}

/* API: destroy ctx. */
int gsm_audio_tunnel_stop(struct GsmAudioTunnel *ctx)
{
    int i = 0;

    /* If not running, dont do it again */
    if (!AudioQueue_isrunning(&ctx->rec_q) || 
		!AudioQueue_isrunning(&ctx->play_q))
        return 0;

	ALOGD("Signal all audio threads to stop");
	AudioQueue_end(&ctx->play_q);
	AudioQueue_end(&ctx->rec_q);

	// Wait until android audio threads are "idling"
    for (i=0; 
		(!ctx->rec_thread_exited || !ctx->play_thread_exited) && i<100; 
		++i){
        usleep(100000);
	}
	// After all sleep for 0.2 seconds since android device can be slow
    usleep(200000);
	ALOGD("Android audio threads are idle");

	if (ctx->rec_strm) { ((android::AudioRecord*)ctx->rec_strm)->stop(); }
    if (ctx->play_strm) { ((android::AudioTrack*)ctx->play_strm)->stop(); }
	usleep(200000);
	ALOGD("Stopped android audio streaming");
	
	pthread_join(ctx->modem_t,NULL);
	ALOGD("End modemIO thread");

    // Restore the audio routing setting
    //      android::AudioSystem::setRouting(ctx->saved_audio_mode,
    //                      ctx->saved_audio_routing,
    //                      android::AudioSystem::ROUTE_ALL);


	ALOGD("Closing streaming");

	if (ctx->play_strm) delete ((android::AudioTrack*)ctx->play_strm);
	if (ctx->rec_strm) delete ((android::AudioRecord*)ctx->rec_strm);
#ifdef CHECK_MEM_OVERRUN
	if (ctx->play_buf) free(((int*)ctx->play_buf)-1);
	if (ctx->rec_buf) free(((int*)ctx->rec_buf)-1);
#else
	if (ctx->play_buf) free(ctx->play_buf);
	if (ctx->rec_buf) free(ctx->rec_buf);
#endif
	if (ctx->fd) close(ctx->fd);
	
#if LOG_MODEM_AUDIO
	if (ctx->logfd) close(ctx->logfd);
#endif
	
    memset(ctx,0,sizeof(struct GsmAudioTunnel));

	ALOGD("Done");
    return 0;
}

