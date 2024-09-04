#pragma once

#include <Arduino.h>
#include "LibAlg.hpp"

namespace RSLA
{
    class ThrusterSolver
    {
    public:
        LA::FloatMatrix allocationMatrix;
        LA::FloatMatrix controlVector;
        LA::FloatMatrix outputVector;

        bool validConfiguration = true;

        ThrusterSolver(LA::FloatMatrix thrusters);

        bool solve(float fx, float fy, float fz, float tx, float ty, float tz);
        bool solve(float f[6]);

        unsigned int numThrusters = 8;
        float getForce(unsigned int index);

        unsigned short getPWM(unsigned int index, float voltage);
    };
}