#include "ThrusterSolver.hpp"

// Thruster solver cosntructor
RSLA::ThrusterSolver::ThrusterSolver(LA::FloatMatrix thrusters) :   allocationMatrix(thrusters.rows, thrusters.cols),
                                                                    controlVector(6, 1),
                                                                    outputVector(thrusters.cols, 1)
{
    // Convert thruster configuration matrix to allocation pseudoinverse
    allocationMatrix = ~thrusters * ((thrusters * ~thrusters).inverse(0.001f, validConfiguration));

    // Set for safekeeping
    numThrusters = thrusters.cols;
}

// Thruster solver function
bool RSLA::ThrusterSolver::solve(float fx, float fy, float fz, float tx, float ty,  float tz)
{
    // Set up control vector
    controlVector(0, 0) = fx;
    controlVector(1, 0) = fy;
    controlVector(2, 0) = fz;
    controlVector(3, 0) = tx;
    controlVector(4, 0) = ty;
    controlVector(5, 0) = tz;

    // Run thruster solver
    outputVector = allocationMatrix * controlVector;

    // Return configuration validity for... reasons
    return validConfiguration;
}

// Thruster solver function
bool RSLA::ThrusterSolver::solve(float c[6])
{
    // Set up control vector
    controlVector = c;

    // Run thruster solver
    outputVector = allocationMatrix * controlVector;

    // Return configuration validity for... reasons
    return validConfiguration;
}

// Force output function
float RSLA::ThrusterSolver::getForce(unsigned int index)
{
    // Check out of range
    if(index < 0 || index >= numThrusters)
    {
        return NAN;
    }

    // Return respective output vector value
    return outputVector(index, 0);
}

// PWM output function
unsigned short RSLA::ThrusterSolver::getPWM(unsigned int index, float voltage)
{
    // Check out of range
    if(index < 0 || index >= numThrusters)
    {
        return 0;
    }

    // Get output force
    float force = getForce(index);

    // Clamp output force to absolute limits
    if (force > 51.4f)
    {
        return 1900;
    }
    else if (force < -39.9f)
    {
        return 1100;
    }

    // Convert force to PWM output with a polyfit
    int16_t pwm = 0;

    if (force > 0.401f) // Upper deadband bound
    {
        pwm = (int16_t)(24.83f * powf(force, 0.6895f) + 17.72f);
    }
    else if (force < -0.356f) // Lower deadband bound
    {
        pwm = -(int16_t)(28.91f * powf(-force, 0.6951f) + 16.95f);
    }

    // Offset PWM to band center and return
    return 1500 + pwm;
}