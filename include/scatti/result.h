/********************************************************************
* Permission is hereby granted, free of charge, to any person obtaining
* a copy of this software and associated documentation files (the
* "Software"), to deal in the Software without restriction, including
* without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to
* permit persons to whom the Software is furnished to do so, subject
* to the following conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
* Description: scatti - pure C99 jerk-limited real-time trajectory
*              generation. Port of Ruckig with embed-friendly
*              memory and math hooks.
* Author:      杨阳 (Yang Yang) <mika-net@outlook.com>
* Origin:      Based on Ruckig Community Edition by Lars Berscheid
*              (https://github.com/pantor/ruckig)
* License:     MIT (SPDX-License-Identifier: MIT)
* Copyright (c) 2026 杨阳 (Yang Yang)
********************************************************************/

#ifndef SCATTI_RESULT_H
#define SCATTI_RESULT_H

typedef enum {
    SCattiWorking = 0,
    SCattiFinished = 1,
    SCattiError = -1,
    SCattiErrorInvalidInput = -100,
    SCattiErrorTrajectoryDuration = -101,
    SCattiErrorPositionalLimits = -102,
    SCattiErrorZeroLimits = -104,
    SCattiErrorExecutionTimeCalculation = -110,
    SCattiErrorSynchronizationCalculation = -111
} SCattiResult;

typedef enum {
    SCattiPosition = 0,
    SCattiVelocity = 1
} SCattiControlInterface;

typedef enum {
    SCattiSyncTime = 0,
    SCattiSyncTimeIfNecessary = 1,
    SCattiSyncPhase = 2,
    SCattiSyncNone = 3
} SCattiSynchronization;

typedef enum {
    SCattiContinuous = 0,
    SCattiDiscrete = 1
} SCattiDurationDiscretization;

#endif /* SCATTI_RESULT_H */
