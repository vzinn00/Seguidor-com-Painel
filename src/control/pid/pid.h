#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID
{
private:
    float kp;
    float ki;
    float kd;

    float erroAnterior;
    float integral;

    float saidaMin;
    float saidaMax;

public:
    PID(float kp, float ki, float kd,
        float saidaMin, float saidaMax);

    float calcular(float erro, float dt);

    void reset();
};

#endif