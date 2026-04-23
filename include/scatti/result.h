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
