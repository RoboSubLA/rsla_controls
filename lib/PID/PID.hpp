#pragma once

#include <math.h>

namespace RSLA
{
    enum class ErrorMode : unsigned char
    {
        LINEAR,
        ANGULAR,
    };

    enum class ProportionalMode : unsigned char
    {
        PROPORTIONAL_ON_ERROR,
        PROPORTIONAL_ON_MEASUREMENT,
    };

    enum class DerivativeMode : unsigned char
    {
        DERIVATIVE_ON_ERROR,
        DERIVATIVE_ON_MEASUREMENT,
    };

    class PID
    {
    public:
        float kP;
        float kI;
        float kD;
        float setpoint;

        float antiwindup;
        bool enableAntiwindup = false;

        float bias;

        float constraint;
        bool enableConstraint = false;

        float output;

        ErrorMode errorMode = ErrorMode::LINEAR;
        ProportionalMode proportionalMode = ProportionalMode::PROPORTIONAL_ON_ERROR;
        DerivativeMode derivativeMode = DerivativeMode::DERIVATIVE_ON_ERROR;

        PID()                                               : kP(1), kI(0), kD(0), setpoint(0), antiwindup(0) {};
        PID(float p, float i, float d)                      : kP(p), kI(i), kD(d), setpoint(0), antiwindup(0) {};
        PID(float p, float i, float d, float s)             : kP(p), kI(i), kD(d), setpoint(s), antiwindup(0) {};
        PID(float p, float i, float d, float a, float s)    : kP(p), kI(i), kD(d), setpoint(s), antiwindup(a), enableAntiwindup(true) {};

        float update(float pv, float dt);

        void zeroIntegrator() { integrator = 0; };
        void reset() { setpoint = output = error = integrator = derivative = derivLast = 0; };
    private:
        float error;
        float integrator;
        float derivative;
        float derivLast;
    };
}