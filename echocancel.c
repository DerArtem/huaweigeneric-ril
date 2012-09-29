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

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define LOG_TAG "EchoCancel"
#include <utils/Log.h>

#include "echocancel.h"

// It is very difficult to do echo cancellation at this level due to the lack of
// the timing information of the samples being played and recorded. Therefore,
// for the first release only echo suppression is implemented.

// The algorithm is derived from the "previous works" summarized in
//   A new class of doubletalk detectors based on cross-correlation,
//   J Benesty, DR Morgan, JH Cho, IEEE Trans. on Speech and Audio Processing.
// The method proposed in that paper is not used because of its high complexity.

// It is well known that cross-correlation can be computed using convolution,
// but unfortunately not every mobile processor has a (fast enough) FPU. Thus
// we use integer arithmetic as much as possible and do lots of bookkeeping.
// Again, parameters and thresholds are chosen by experiments.

int echocancel_init(struct echocancel_ctx* ctx,int sampleCount, int tailLength)
{
	int shift;
	
    tailLength += sampleCount * 4;

    shift = 0;
    while ((sampleCount >> shift) > 1 && (tailLength >> shift) > 256) {
        ++shift;
    }

    ctx->Shift = shift + 4;
    ctx->Scale = 1 << shift;
    ctx->SampleCount = sampleCount;
    ctx->WindowSize = sampleCount >> shift;
    ctx->TailLength = tailLength >> shift;
    ctx->RecordLength = tailLength * 2 / sampleCount;
    ctx->RecordOffset = 0;

    ctx->Xs = malloc(sizeof(*ctx->Xs)*(ctx->TailLength + ctx->WindowSize));
    memset(ctx->Xs, 0, sizeof(*ctx->Xs) * (ctx->TailLength + ctx->WindowSize));
    ctx->XSums = malloc(sizeof(*ctx->XSums)*(ctx->TailLength));
    memset(ctx->XSums, 0, sizeof(*ctx->XSums) * ctx->TailLength);
    ctx->X2Sums = malloc(sizeof(*ctx->X2Sums)* ctx->TailLength);
    memset(ctx->X2Sums, 0, sizeof(*ctx->X2Sums) * ctx->TailLength);
    ctx->XRecords = malloc(sizeof(*ctx->XRecords) * (ctx->RecordLength * ctx->WindowSize));
    memset(ctx->XRecords, 0, sizeof(*ctx->XRecords) * ctx->RecordLength * ctx->WindowSize);

    ctx->YSum = 0;
    ctx->Y2Sum = 0;
    ctx->YRecords = malloc( sizeof(*ctx->YRecords) * ctx->RecordLength);
    memset(ctx->YRecords, 0, sizeof(*ctx->YRecords) * ctx->RecordLength);
    ctx->Y2Records = malloc( sizeof (*ctx->Y2Records) * ctx->RecordLength);
    memset(ctx->Y2Records, 0, sizeof(*ctx->Y2Records) * ctx->RecordLength);

    ctx->XYSums = malloc( sizeof( *ctx->XYSums) * ctx->TailLength);
    memset(ctx->XYSums, 0, sizeof( *ctx->XYSums) * ctx->TailLength);
    ctx->XYRecords = malloc( sizeof( *ctx->XYRecords) * ctx->RecordLength * ctx->TailLength);
    memset(ctx->XYRecords, 0, sizeof(*ctx->XYRecords) * ctx->RecordLength * ctx->TailLength);

    ctx->LastX = 0;
    ctx->LastY = 0;
    ctx->Weight = 1.0f / (ctx->RecordLength * ctx->WindowSize);
	
	return 0;
}

void echocancel_end(struct echocancel_ctx* ctx)
{
    free(ctx->Xs);
    free(ctx->XSums);
    free(ctx->X2Sums);
    free(ctx->XRecords);
    free(ctx->YRecords);
    free(ctx->Y2Records);
    free(ctx->XYSums);
    free(ctx->XYRecords);
	memset(ctx,0,sizeof(*ctx));
}

void echocancel_run(struct echocancel_ctx* ctx,int16_t *playbacked, int16_t *recorded)
{
	uint16_t *xRecords;
	uint16_t ys[ctx->WindowSize];
    uint32_t ySum = 0;
    uint32_t y2Sum = 0;
	uint32_t *xyRecords;
    int latency = 0;
    float corr2 = 0.0f;
    float varX = 0.0f;
    float varY;
	int i,j;
	
    // Update Xs.
    for (i = ctx->TailLength - 1; i >= 0; --i) {
        ctx->Xs[i + ctx->WindowSize] = ctx->Xs[i];
    }
    for (i = ctx->WindowSize - 1, j = 0; i >= 0; --i, j += ctx->Scale) {
        uint32_t sum = 0;
		int k;
        for (k = 0; k < ctx->Scale; ++k) {
            int32_t x = playbacked[j + k] << 15;
            ctx->LastX += x;
            sum += ((ctx->LastX >= 0) ? ctx->LastX : -ctx->LastX) >> 15;
            ctx->LastX -= (ctx->LastX >> 10) + x;
        }
        ctx->Xs[i] = sum >> ctx->Shift;
    }

    // Update XSums, X2Sums, and XRecords.
    for (i = ctx->TailLength - ctx->WindowSize - 1; i >= 0; --i) {
        ctx->XSums[i + ctx->WindowSize] = ctx->XSums[i];
        ctx->X2Sums[i + ctx->WindowSize] = ctx->X2Sums[i];
    }
    xRecords = &ctx->XRecords[ctx->RecordOffset * ctx->WindowSize];
    for (i = ctx->WindowSize - 1; i >= 0; --i) {
        uint16_t x = ctx->Xs[i];
        ctx->XSums[i] = ctx->XSums[i + 1] + x - xRecords[i];
        ctx->X2Sums[i] = ctx->X2Sums[i + 1] + x * x - xRecords[i] * xRecords[i];
        xRecords[i] = x;
    }

    // Compute Ys.
    for (i = ctx->WindowSize - 1, j = 0; i >= 0; --i, j += ctx->Scale) {
        uint32_t sum = 0;
		int k;
        for (k = 0; k < ctx->Scale; ++k) {
            int32_t y = recorded[j + k] << 15;
            ctx->LastY += y;
            sum += ((ctx->LastY >= 0) ? ctx->LastY : -ctx->LastY) >> 15;
            ctx->LastY -= (ctx->LastY >> 10) + y;
        }
        ys[i] = sum >> ctx->Shift;
    }

    // Update YSum, Y2Sum, YRecords, and Y2Records.
    ySum = 0;
    y2Sum = 0;
    for (i = ctx->WindowSize - 1; i >= 0; --i) {
        ySum += ys[i];
        y2Sum += ys[i] * ys[i];
    }
    ctx->YSum += ySum - ctx->YRecords[ctx->RecordOffset];
    ctx->Y2Sum += y2Sum - ctx->Y2Records[ctx->RecordOffset];
    ctx->YRecords[ctx->RecordOffset] = ySum;
    ctx->Y2Records[ctx->RecordOffset] = y2Sum;

    // Update XYSums and XYRecords.
    xyRecords = &ctx->XYRecords[ctx->RecordOffset * ctx->TailLength];
    for (i = ctx->TailLength - 1; i >= 0; --i) {
        uint32_t xySum = 0;
        for (j = ctx->WindowSize - 1; j >= 0; --j) {
            xySum += ctx->Xs[i + j] * ys[j];
        }
        ctx->XYSums[i] += xySum - xyRecords[i];
        xyRecords[i] = xySum;
    }

    // Compute correlations.
    latency = 0;
    corr2 = 0.0f;
    varX = 0.0f;
    varY = ctx->Y2Sum - ctx->Weight * ctx->YSum * ctx->YSum;
    for (i = ctx->TailLength - 1; i >= 0; --i) {
        float cov = ctx->XYSums[i] - ctx->Weight * ctx->XSums[i] * ctx->YSum;
        if (cov > 0.0f) {
            float varXi = ctx->X2Sums[i] - ctx->Weight * ctx->XSums[i] * ctx->XSums[i];
            float corr2i = cov * cov / (varXi * varY + 1);
            if (corr2i > corr2) {
                varX = varXi;
                corr2 = corr2i;
                latency = i;
            }
        }
    }
    //ALOGI("corr^2 %.5f, var %8.0f %8.0f, latency %d", corr2, varX, varY,
    //        latency * mScale);

    // Do echo suppression.
    if (corr2 > 0.1f && varX > 10000.0f) {
        int factor = (corr2 > 1.0f) ? 0 : (1.0f - sqrtf(corr2)) * 4096;
        for (i = 0; i < ctx->SampleCount; ++i) {
            recorded[i] = recorded[i] * factor >> 16;
        }
    }

    // Increase RecordOffset.
    ++ctx->RecordOffset;
    if (ctx->RecordOffset == ctx->RecordLength) {
        ctx->RecordOffset = 0;
    }
}
