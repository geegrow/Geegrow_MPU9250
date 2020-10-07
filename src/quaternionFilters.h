#ifndef QUATERNIONFILTERS_H
#define QUATERNIONFILTERS_H

#include <Arduino.h>

#define PI  3.14159265

class Madgwick
{
    public:
        Madgwick() {}
        static void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

        static float yaw, pitch, roll;
};

// void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
//                               float gz, float mx, float my, float mz);
// void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
//                             float gz, float mx, float my, float mz);
// const float * getQ();

#endif /* QUATERNIONFILTERS_H */
