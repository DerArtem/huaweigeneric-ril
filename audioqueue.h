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

/* A resampling Audio queue */

#ifndef __AUDIOQUEUE_H
#define __AUDIOQUEUE_H 1

#include <pthread.h>
#include <sys/time.h>

struct AudioQueue {
	volatile int running;	// != 0 if running
	unsigned int size;		// Queue size in samples
	unsigned int sample_sz;	// Sample size in bytes
	unsigned int wr_pos;	// Write position in samples
	unsigned int rd_pos;	// Read position in samples
	unsigned int ratio;		// Resampling ratio including speedup/speeddown due to fullness
	unsigned int step;		// Linear interpolation step

	void* data;				// Queue Data buffer
};

#ifdef __cplusplus
extern "C" {
#endif

int AudioQueue_init(struct AudioQueue* ctx,unsigned int p2maxsamples, unsigned int sample_sz);
int AudioQueue_isrunning(struct AudioQueue* ctx);
int AudioQueue_add(struct AudioQueue* ctx, void* data,unsigned int samples);
int AudioQueue_get(struct AudioQueue* ctx, void* data,unsigned int samples);
int AudioQueue_end(struct AudioQueue* ctx);

#ifdef __cplusplus
}
#endif

#endif