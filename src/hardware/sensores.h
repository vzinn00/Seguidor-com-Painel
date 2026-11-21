#pragma once

#include <Arduino.h>
#include <QTRSensors.h>

#define NUM_SENSORS 8
#define SensorDireita 2
#define SensorEsquerda 36

#define esqC1 23
#define esqC2 21
#define dirC1 19
#define dirC2 14
#define ENCODER_PULSOS_POR_VOLTA 72.0f
#define INTERVALO_VELOCIDADE_MS 20

#define DIAMETRO_RODA_MM 65.0f // Ajustar para o diâmetro real da roda

class sensores
{
public:
    void begin();
    void calibrate(unsigned long TEMPO_CALIBRACAO = 5000);

    void update();
    bool direitaDetected() const;
    bool esquerdaDetected() const;
    int getLeituraDireita() const;
    int getLeituraEsquerda() const;

    int getMinDireita() const;
    int getMaxDireita() const;

    int getMinEsquerda() const;
    int getMaxEsquerda() const;

    bool linhaDetectada() const;
    
    int calculaErro() const;

    const uint16_t* getValues() const;
    void atualizarVelocidade();
    float getVelocidadeEsq() const;
    float getVelocidadeDir() const;

    void zerarDistancia();
    float getDistanciaEsqMm() const;
    float getDistanciaDirMm() const;

private:
    QTRSensors qtr;
    uint16_t sensorValues[NUM_SENSORS] = {};
    uint16_t sensoresLaterais[2] = {};
    bool detectDireita = false;
    bool detectEsquerda = false;
    float velDir = 0.0f;
    float velEsq = 0.0f;
    int erro = 0;
    int thersholdDirMin = 4095, thersholdDirMax = 0, thersholdEsqMin = 4095, thersholdEsqMax = 0;
};