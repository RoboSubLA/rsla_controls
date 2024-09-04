#include "PID.hpp"

// PID update function
float RSLA::PID::update(float pv, float dt)
{
    // Calculate this timestep's error, based on setpoint and input
    error = setpoint - pv;

    // Convert 0-360 to -180-180 for angular errors
    if(errorMode == ErrorMode::ANGULAR)
    {
        error = fmodf(error + 360 + 180, 360) - 180;
    }

    // Integrate and apply antiwindup clamp
    integrator += error * dt;
    if(enableAntiwindup)
    {
        if(integrator > antiwindup) // Upper bound
        {
            integrator = antiwindup;
        }
        else if(integrator < -antiwindup) // Lower bound
        {
            integrator = -antiwindup;
        }
    }

    // Calculate this timestep's derivative
    if(derivativeMode == DerivativeMode::DERIVATIVE_ON_ERROR)
    {
        derivative = (error - derivLast) / dt;
        derivLast = error;
    }
    else if(derivativeMode == DerivativeMode::DERIVATIVE_ON_MEASUREMENT)
    {
        derivative = (pv - derivLast) / dt;
        derivLast = pv;
    }

    // Sum the total controller output
    if(proportionalMode == ProportionalMode::PROPORTIONAL_ON_ERROR)
    {
        output = (kP * error) + (kI * integrator) + (kD * derivative) + bias;
    }
    else if(proportionalMode == ProportionalMode::PROPORTIONAL_ON_MEASUREMENT)
    {
        output = (kP * pv) + (kI * integrator) + (kD * derivative) + bias;
    }

    if(enableConstraint)
    {
        if(output > constraint)
        {
            output = constraint;
        }
        else if(output < -constraint)
        {
            output = -constraint;
        }
    }

    return output;
}

