#include "audioqueue.h"
#include <string.h>
#include <stdio.h>

#define LOG_NDEBUG 0
#define LOG_TAG "RILAudioQueue"
#include <utils/Log.h>

#define f_mul(a,b) 	(((long long)(a))*(b) >> 28)
#define f_div(a,b) 	((((long long)(a)) << 28) / (b))
#define f_fract(a) 	((a) & ((1<<28)-1))
#define f_intp(a)  	((a) >> 28)
#define f_fromint(a)((a) << 28)
#define ASRC_F_ONE 	(1<<28)

#define TOBEREADY_COUNT 10 /* count of read/writes to start smooth filtering of sample time. Before it, we just use averaging */

#define  AUDIOQUEUE_DEBUG 0
#if AUDIOQUEUE_DEBUG
#  define  D(...)   ALOGD(__VA_ARGS__)
#else
#  define  D(...)   ((void)0)
#endif 


/* Return count in buffer.  */
#define CIRC_CNT(wr_pos,rd_pos,size) (((wr_pos) - (rd_pos)) & ((size)-1))

/* Return space available, 0..size-1.  We always leave one free char
   as a completely full buffer has wr_pos == rd_pos, which is the same as
   empty.  */
#define CIRC_SPACE(wr_pos,rd_pos,size) CIRC_CNT((rd_pos),((wr_pos)+1),(size))

/* Return count up to the end of the buffer.  Carefully avoid
   accessing wr_pos and rd_pos more than once, so they can change
   underneath us without returning inconsistent results.  */
#define CIRC_CNT_TO_END(wr_pos,rd_pos,size) \
	({int end = (size) - (rd_pos); \
	  int n = ((wr_pos) + end) & ((size)-1); \
	  n < end ? n : end;})

/* Return space available up to the end of the buffer.  */
#define CIRC_SPACE_TO_END(wr_pos,rd_pos,size) \
	({int end = (size) - 1 - (wr_pos); \
	  int n = (end + (rd_pos)) & ((size)-1); \
	  n <= end ? n : end+1;}) 


// Init the audio queue
int AudioQueue_init(struct AudioQueue* ctx,unsigned int p2maxsamples, unsigned int sample_sz)
{
	unsigned int maxsamples = 1U << p2maxsamples;
	memset(ctx,0,sizeof(struct AudioQueue));
	
	ctx->size = maxsamples;
	ctx->sample_sz = sample_sz;
	
	/* Linear resampler */
	ctx->ratio = ASRC_F_ONE;
	
	ctx->data = malloc(maxsamples * sample_sz);

	if (!ctx->data) {
		return -1;
	}
	
	ctx->running = 1;
	
	return 0;
}

int AudioQueue_isrunning(struct AudioQueue* ctx)
{
	return ctx->running;
}

int AudioQueue_add(struct AudioQueue* ctx, void* data,unsigned int samples)
{
	unsigned int samples_todo = samples;
	
	// If exited, avoid adding
	if (!ctx->running)
		return -1;
		
	D("add[%p]: begin: Store %d samples",ctx,samples);
	
	// Not filled, add to the queue.
	while (ctx->running && samples_todo) {
	
		// Calculate remaining space until end of buffer. We always leave a byte free 
		//  so we can differenciate between empty and full buffers
		unsigned int rem = CIRC_SPACE_TO_END(ctx->wr_pos,ctx->rd_pos,ctx->size);
				
		D("add[%p]: samples_todo: %u, rem: %u, rd: %u, wr: %u, sz: %u",ctx, samples_todo, rem, ctx->rd_pos, ctx->wr_pos, ctx->size);
		
		if (rem == 0) {
			/* not enough data... Ignore the part we can't store */			
			unsigned int ratio = ctx->ratio;
		
			// Adjust ratio to avoid this the next time 
			ratio += ratio/200;
		
			// Limit to sensible values
			if (ratio > (ASRC_F_ONE + (ASRC_F_ONE / 20))) {
				ratio = (ASRC_F_ONE + (ASRC_F_ONE / 20));
			}
			ctx->ratio = ratio;
		
			D("add[%p]: Not enough space on queue... Partially dropping packet, new ratio: %u",ctx,ratio);
			break; 
			
		} else {

			// Do not store more than needed
			if (rem > samples_todo) {
				rem = samples_todo;
			}
			
			// Store data in queue
			memcpy( (char*)ctx->data + ctx->wr_pos * ctx->sample_sz, data, rem * ctx->sample_sz);
			data = (char*) data + rem * ctx->sample_sz;
			ctx->wr_pos = (ctx->wr_pos + rem) & (ctx->size-1);
			samples_todo -= rem;
		}
	};
	
	D("add[%p]: end: Stored %d samples, size %d, rd:%d, wr:%d",ctx,samples - samples_todo,ctx->size, ctx->rd_pos, ctx->wr_pos);
	
	// Return the count of stored samples
	return samples - samples_todo;
}

int AudioQueue_get(struct AudioQueue* ctx, void* data,unsigned int samples)
{
	unsigned int samples_todo = samples;

	// If exited, avoid adding
	if (!ctx->running)
		return -1;
		
	D("get[%p]: begin: Read of %d samples",ctx,samples);
		
	// While samples to be read
	while (ctx->running && samples_todo) {
	
		// Read the first free position
		unsigned int rd_endni = ctx->wr_pos;
		unsigned int step = ctx->step, mask = ctx->size-1;
		unsigned int rd_pos = ctx->rd_pos;
		
		D("get[%p]: [1] samples_todo: %u, rd: %u, wr: %u, sz: %u",ctx, samples_todo, ctx->rd_pos, ctx->wr_pos, ctx->size);
		
		// linear interpolation resampling until all requested data is providedno more
		if (ctx->sample_sz == 2 ) {
			
			short const *psrc = ctx->data;
			short *pdst = (short*)data;
			const short *pdstendni = (short*)data + samples_todo;
	
			/* We use the current one and next one samples */
			while (pdst != pdstendni) {
				unsigned int ipart = f_intp(step);

				// If not enough data, break now
				if (((rd_pos + ipart + 1) & mask) == rd_endni ||
					((rd_pos + ipart    ) & mask) == rd_endni) 
					break;
				
				rd_pos  = (rd_pos + ipart) & mask;
				step    = f_fract(step);

				*pdst++ = psrc[rd_pos] + f_mul(psrc[(rd_pos+1) & mask] - psrc[rd_pos], step);

				step   += ctx->ratio;
			}
			samples_todo -= pdst - (short*)data;
			data = pdst;
			
		} else {
			
			unsigned char const *psrc = ctx->data;
			unsigned char *pdst = (unsigned char*)data;
			const unsigned char *pdstendni = (unsigned char*)data + samples_todo;
	
			/* We use the current one and next one samples */
			while (pdst != pdstendni) {
				unsigned int ipart = f_intp(step);

				// If not enough data, break now
				if (((rd_pos + ipart + 1) & mask) == rd_endni ||
					((rd_pos + ipart    ) & mask) == rd_endni) 
					break;
				
				rd_pos  = (rd_pos + ipart) & mask;
				step    = f_fract(step);

				*pdst++ = psrc[rd_pos] + f_mul(psrc[(rd_pos+1) & mask] - psrc[rd_pos], step);

				step   += ctx->ratio;
			}
			samples_todo -= pdst - (unsigned char*)data;
			data = pdst;
	
		}
		
		// Update buffer pointers and linear resampler step
		ctx->step = step;
		ctx->rd_pos = rd_pos;
		
		D("get[%p]: [2] samples_todo: %u, rd: %u, wr: %u, sz: %u",ctx, samples_todo, ctx->rd_pos, ctx->wr_pos, ctx->size);			
		
		if (samples_todo) {
		
			// No more samples to provide. Adjust ratio so this never happens again 
			unsigned int ratio = ctx->ratio;
			ratio -= ratio / 200;
			
			// Limit to sensible values
			if (ratio < (ASRC_F_ONE - (ASRC_F_ONE / 20))) {
				ratio = (ASRC_F_ONE - (ASRC_F_ONE / 20));
			}
			ctx->ratio = ratio;
			
			D("get[%p]: Not enough data on queue... Partially returning silence, new ratio: %u",ctx,ratio);		

			// Complete with silence
			memset(data,0,samples_todo * ctx->sample_sz);

			// Return what we have...
			break;
		}
	};

	D("get[%p]: end: got %d samples, total: %d, rd:%d, wr:%d",ctx, samples - samples_todo, ctx->size,ctx->rd_pos, ctx->wr_pos);
	
	// Return the count of read samples
	return samples - samples_todo;
}

int AudioQueue_end(struct AudioQueue* ctx)
{
	
	// If exited, avoid reexecuting deinitialization
	if (!ctx->running)
		return 0;
		
	// Signal end
	ctx->running = 0;

	// Some delay to let add and get end...
	sleep(1);
	
	free(ctx->data);
	memset(ctx,0,sizeof(struct AudioQueue));
	return 0;
}
