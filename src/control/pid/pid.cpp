#include "pid.h"

PID::PID(float kp, float ki, float kd,
         float saidaMin, float saidaMax)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;

    this->saidaMin = saidaMin;
    this->saidaMax = saidaMax;

    erroAnterior = 0;
    integral = 0;
}

float PID::calcular(float erro, float dt)
{
    if (dt <= 0)
        return 0;

    // Ação proporcional
    float proporcional = kp * erro;

    // Ação integral
    integral += erro * dt;

    float integrativo = ki * integral;

    // Ação derivativa
    float derivativo = kd * (erro - erroAnterior) / dt;

    // Saída do controlador
    float saida = proporcional + integrativo + derivativo;

    erroAnterior = erro;

    // Saturação
    return constrain(saida, saidaMin, saidaMax);
}

void PID::reset()
{
    erroAnterior = 0;
    integral = 0;
}