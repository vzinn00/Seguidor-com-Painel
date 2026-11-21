#include "pid.h"

PID::PID(float kp, float ki, float kd,
         float saidaMin, float saidaMax)
    : kp(kp),
      ki(ki),
      kd(kd),
      erroAnterior(0.0f),
      integral(0.0f),
      saidaMin(saidaMin),
      saidaMax(saidaMax)
{
}

float PID::calcular(float erro, float dt)
{
    if (!isfinite(erro) || !isfinite(dt) || dt <= 0.0f)
        return 0.0f;

    const float proporcional = kp * erro;
    const float derivativo = kd * (erro - erroAnterior) / dt;

    // Integra apenas quando a saturação não está sendo agravada.
    const float integralCandidata = integral + erro * dt;
    const float saidaCandidata =
        proporcional + ki * integralCandidata + derivativo;

    const bool saturandoAcima = saidaCandidata > saidaMax && erro > 0.0f;
    const bool saturandoAbaixo = saidaCandidata < saidaMin && erro < 0.0f;

    if (!saturandoAcima && !saturandoAbaixo)
        integral = integralCandidata;

    const float saida =
        proporcional + ki * integral + derivativo;

    erroAnterior = erro;

    return constrain(saida, saidaMin, saidaMax);
}

void PID::reset()
{
    erroAnterior = 0.0f;
    integral = 0.0f;
}
