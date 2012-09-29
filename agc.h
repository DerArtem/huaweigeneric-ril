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

#ifndef _AGC_H
#define _AGC_H

struct agc_ctx {
	unsigned int sample_max;
	int counter;
	int igain;
	int ipeak;
	int silence_counter;
};

#ifdef __cplusplus
extern "C" {
#endif

void agc_init(struct agc_ctx* ctx, short level /* 16384-32767*/ );
void agc_process_16bit(struct agc_ctx *ctx, short *buffer, int samples);
void agc_process_8bit(struct agc_ctx *ctx, unsigned char *buffer, int samples);


#ifdef __cplusplus
}
#endif

#endif