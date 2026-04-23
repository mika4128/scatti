#ifndef SCATTI_RESULT_H
#define SCATTI_RESULT_H

typedef enum {
    CRuckigWorking = 0,
    CRuckigFinished = 1,
    CRuckigError = -1,
    CRuckigErrorInvalidInput = -100,
    CRuckigErrorTrajectoryDuration = -101,
    CRuckigErrorPositionalLimits = -102,
    CRuckigErrorZeroLimits = -104,
    CRuckigErrorExecutionTimeCalculation = -110,
    CRuckigErrorSynchronizationCalculation = -111
} CRuckigResult;

typedef enum {
    CRuckigPosition = 0,
    CRuckigVelocity = 1
} CRuckigControlInterface;

typedef enum {
    CRuckigSyncTime = 0,
    CRuckigSyncTimeIfNecessary = 1,
    CRuckigSyncPhase = 2,
    CRuckigSyncNone = 3
} CRuckigSynchronization;

typedef enum {
    CRuckigContinuous = 0,
    CRuckigDiscrete = 1
} CRuckigDurationDiscretization;

#endif /* SCATTI_RESULT_H */
