/*
*  This AGC algorithm was taken from isdn2h323 (http://www.telos.de). It was
*  converted from C++ to C, and modified to add control over the recording level.
*  Converted to fixed point by Phil Frisbie, Jr. 4/12/2003
*/

#include "agc.h"

#define LOG_NDEBUG 0
#define LOG_TAG "RILAudioAGC"
#include <utils/Log.h>

void agc_init(struct agc_ctx* ctx, short level /* 16384-32767*/ )
{
	ALOGD("[%p] Initializing AGC: level: %d",ctx, level);
    ctx->sample_max = 1;
    ctx->counter = 0;
    ctx->igain = 65536;
    if(level < 16384) {
        level = 16384;
    }
    ctx->ipeak = (int)(level * 65536);
    ctx->silence_counter = 0;
}

void agc_process_16bit(struct agc_ctx *ctx, short *buffer, int len) 
{
    int i;
    for(i=0; i<len; i++) {
	
        long gain_new;
        int sample;

        /* get the abs of buffer[i] */
        sample = buffer[i];
        sample = (sample < 0 ? -(sample) : sample);

        if(sample > (int)ctx->sample_max)
        {
            /* update the max */
            ctx->sample_max = (unsigned int)sample;
        }
        ctx->counter ++;

        /* Will we get an overflow with the current gain factor? */
        if (((sample * ctx->igain) >> 16) > ctx->ipeak)
        {
            /* Yes: Calculate new gain. */
            ctx->igain = ((ctx->ipeak / ctx->sample_max) * 62259) >> 16;
            ctx->silence_counter = 0;
            buffer[i] = (short) ((buffer[i] * ctx->igain) >> 16);
            continue;
        }

        /* Calculate new gain factor 10x per second */
        if (ctx->counter >= 8000/10) 
        {
            if (ctx->sample_max > 800)        /* speaking? */
            {
                gain_new = ((ctx->ipeak / ctx->sample_max) * 62259) >> 16;
                
                if (ctx->silence_counter > 40)  /* pause -> speaking */
                    ctx->igain += (gain_new - ctx->igain) >> 2;
                else
                    ctx->igain += (gain_new - ctx->igain) / 20;

                ctx->silence_counter = 0;
            }
            else   /* silence */
            {
                ctx->silence_counter++;
                /* silence > 2 seconds: reduce gain */
                if ((ctx->igain > 65536) && (ctx->silence_counter >= 20))
                    ctx->igain = (ctx->igain * 62259) >> 16;
            }
            ctx->counter = 0;
            ctx->sample_max = 1;
        }
        buffer[i] = (short) ((buffer[i] * ctx->igain) >> 16);
    }
	
	//ALOGD("[%p] Gain: %d",ctx, ctx->igain);
}

void agc_process_8bit(struct agc_ctx *ctx, unsigned char *buffer, int len)
{
    int i;
    for(i=0; i<len; i++) {
	
        long gain_new;
        int sample;

        /* get the abs of buffer[i] */
        sample = (buffer[i] - 128) << 8;
        sample = (sample < 0 ? -(sample) : sample);

        if(sample > (int)ctx->sample_max)
        {
            /* update the max */
            ctx->sample_max = (unsigned int)sample;
        }
        ctx->counter ++;

        /* Will we get an overflow with the current gain factor? */
        if (((sample * ctx->igain) >> 16) > ctx->ipeak)
        {
            /* Yes: Calculate new gain. */
            ctx->igain = ((ctx->ipeak / ctx->sample_max) * 62259) >> 16;
            ctx->silence_counter = 0;
            buffer[i] = (unsigned char) (( ((buffer[i] - 128) << 8) * ctx->igain) >> 24) + 128;
            continue;
        }

        /* Calculate new gain factor 10x per second */
        if (ctx->counter >= 8000/10) 
        {
            if (ctx->sample_max > 800)        /* speaking? */
            {
                gain_new = ((ctx->ipeak / ctx->sample_max) * 62259) >> 16;
                
                if (ctx->silence_counter > 40)  /* pause -> speaking */
                    ctx->igain += (gain_new - ctx->igain) >> 2;
                else
                    ctx->igain += (gain_new - ctx->igain) / 20;

                ctx->silence_counter = 0;
            }
            else   /* silence */
            {
                ctx->silence_counter++;
                /* silence > 2 seconds: reduce gain */
                if ((ctx->igain > 65536) && (ctx->silence_counter >= 20))
                    ctx->igain = (ctx->igain * 62259) >> 16;
            }
            ctx->counter = 0;
            ctx->sample_max = 1;
        }
		buffer[i] = (unsigned char) (( ((buffer[i] - 128) << 8) * ctx->igain) >> 24) + 128;
    }
	
	//ALOGD("[%p] Gain: %d",ctx, ctx->igain);
}
