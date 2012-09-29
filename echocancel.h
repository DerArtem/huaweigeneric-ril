/*
 * Copyrightm (C) 2010 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __ECHO_CANCEL_H
#define __ECHO_CANCEL_H
 
#include <stdint.h>

struct echocancel_ctx {
    int Shift;
    int Scale;
    int SampleCount;
    int WindowSize;
    int TailLength;
    int RecordLength;
    int RecordOffset;

    uint16_t *Xs;
    uint32_t *XSums;
    uint32_t *X2Sums;
    uint16_t *XRecords;

    uint32_t YSum;
    uint32_t Y2Sum;
    uint32_t *YRecords;
    uint32_t *Y2Records;

    uint32_t *XYSums;
    uint32_t *XYRecords;

    int32_t LastX;
    int32_t LastY;

    float Weight;
};

#ifdef __cplusplus
extern "C" {
#endif

// The sampleCount must be power of 2.
int echocancel_init(struct echocancel_ctx* ctx,int sampleCount, int tailLength);
void echocancel_end(struct echocancel_ctx* ctx);
void echocancel_run(struct echocancel_ctx* ctx,int16_t *playbacked, int16_t *recorded);

#ifdef __cplusplus
}
#endif

#endif
