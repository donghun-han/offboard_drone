#ifndef SUBUTIL_H
#define SUBUTIL_H

#define Kpx     1.0
#define Kpz     1.0

#define eps 	0.000001
#define PI      3.141592

#include <cmath>

float R2D(float rad_data){
    float deg_data = rad_data * (180.0/PI);

    return deg_data;
}

float D2R(float deg_data){
    float rad_data = deg_data * (PI/180.0);

    return rad_data;
}

float satmax(float data, float max)
{
    float res = 0.0;

    if(fabs(data) > max)
        res = (data + eps)/fabs(data + eps)*max;
    else
        res = data;

    return res;
}

// -PI ~ PI
float wrap(float rad_data)
{
    float res = 0.0;
    rad_data = fmod(rad_data + D2R(180.0), D2R(360.0));

    if (rad_data < 0.0){
        rad_data = rad_data + D2R(360.0);

    }
    res = rad_data - D2R(180.0);

    return res;
}

float getNED_angle_err(float angle_cmd, float angle_cur){
    float res = wrap(angle_cmd - angle_cur);
    return res;
}

#endif