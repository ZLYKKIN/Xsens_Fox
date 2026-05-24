
#ifndef FUSION_AHRS_H
#define FUSION_AHRS_H

#include "FusionConvention.h"
#include "FusionMath.h"
#include <stdbool.h>

typedef struct {
    FusionConvention convention;
    float            sampleRateHz;
    float            magDipModelDeg;
    float            magDeclinationDeg;

    float            magNormReferenceLocal;

    float            magDipGateRelax;
    float            magAngGateRelax;

    float            magNormGateRelax;
} FusionAhrsSettings;

typedef struct {

    FusionQuaternion q;
    FusionVector     b_g;
    FusionVector     b_a;
    FusionVector     m0;
    FusionVector     v_lp;

    float magNormBias;
    float skinPhiScalar;

    float P[17 * 17];

    FusionVector a_lp;
    bool         a_lp_ready;

    FusionVector a_lin_body;
    FusionVector a_lin_world;
    bool         a_lin_ready;

    float fAccBoost;
    float dAccHighTime;

    float sBg;

    float        tauM0;
    FusionVector m0_avg;
    bool         m0_avg_ready;

    float        magClearStreakSec;

    float        magClosedStreakSec;
    float        magRedefBoostTimer;
    bool         magWasClosed;

    float stillnessTime;
    int   zruSampleCount;
    int   zruFrameCounter;

    FusionQuaternion qSkin;
    bool             qSkinReady;

    float sigmaAcc;
    float sigmaGyr;
    float sigmaMag;

    float dAcc;
    float magResidualNorm;
    bool  magGateOpen;
    bool  accUsedThisFrame;
    bool  zruActiveThisFrame;

    FusionAhrsSettings settings;
} FusionAhrs;

typedef struct {
    float accelerationError;
    bool  accelerometerIgnored;
    float accelerationRecoveryTrigger;
    float magneticError;
    bool  magnetometerIgnored;
    float magneticRecoveryTrigger;
} FusionAhrsInternalStates;

typedef struct {
    bool startup;
    bool angularRateRecovery;
    bool accelerationRecovery;
    bool magneticRecovery;
} FusionAhrsFlags;

extern const FusionAhrsSettings fusionAhrsDefaultSettings;

void FusionAhrsInitialise(FusionAhrs *ahrs);
void FusionAhrsRestart(FusionAhrs *ahrs);
void FusionAhrsSetSettings(FusionAhrs *ahrs, const FusionAhrsSettings *settings);

void FusionAhrsSetNoise(FusionAhrs *ahrs,
                        float sigmaAccMs2,
                        float sigmaGyrDegS,
                        float sigmaMagNorm);

void FusionAhrsSetSampleRate(FusionAhrs *ahrs, float sampleRateHz);

void FusionAhrsUpdate(FusionAhrs *ahrs,
                      FusionVector gyroscope,
                      FusionVector accelerometer,
                      FusionVector magnetometer,
                      float dt);
void FusionAhrsUpdateNoMagnetometer(FusionAhrs *ahrs,
                                    FusionVector gyroscope,
                                    FusionVector accelerometer,
                                    float dt);

FusionQuaternion FusionAhrsGetQuaternion(const FusionAhrs *ahrs);
void             FusionAhrsSetQuaternion(FusionAhrs *ahrs, FusionQuaternion q);
FusionVector     FusionAhrsGetGravity(const FusionAhrs *ahrs);
FusionVector     FusionAhrsGetLinearAcceleration(const FusionAhrs *ahrs);
FusionVector     FusionAhrsGetEarthAcceleration(const FusionAhrs *ahrs);
FusionAhrsInternalStates FusionAhrsGetInternalStates(const FusionAhrs *ahrs);
FusionAhrsFlags          FusionAhrsGetFlags(const FusionAhrs *ahrs);
void                     FusionAhrsSetHeading(FusionAhrs *ahrs, float headingDeg);

#endif
