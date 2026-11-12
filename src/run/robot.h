#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <atomic>

#include "hardware/sensores.h"
#include "control/fuzzy/funcreg.h"
#include "control/fuzzy/defuzzy.h"
#include "control/pid/pid.h"

#define PWM_MAX 255

// RPM medido pelo encoder quando o motor roda com PWM 255 (sem carga ou na pista).
// Usado para converter o RPM dos encoders para a escala 0-255 do PWM. AJUSTE para o seu motor.
#define RPM_MAXIMO_PWM255 500.0f

class Robot
{
public:
    void begin();
    void update();
    void setSeguidorAtivo(bool ativo);

private:

    static constexpr uint8_t LED_INTERNO = 22;
    
    // =================================================
    // MODULOS
    // =================================================

    sensores sensoresRobot;
    // Servidor HTTP proprio (nao bloqueante), atendido por uma tarefa FreeRTOS
    // no nucleo 0, independente do laco de controle.
    WiFiServer servidorWeb{80};

    // Escrito pela tarefa do painel, lido pelo laco de controle.
    std::atomic<bool> seguimentoAtivo{true};
    std::atomic<bool> estadoAlterado{false};
    fuzzy controladorFuzzy;
    defuzzy controladorDefuzzy;

    PID controladorPID{PID_KP, PID_KD};


    // =================================================
    // TIPO DE CONTROLE
    // =================================================

    enum TipoControle
    {
        CONTROLE_FUZZY,
        CONTROLE_PID
    };

    static constexpr TipoControle CONTROLE_ATUAL =
        CONTROLE_PID;


    // =================================================
    // RECUPERACAO DE LINHA
    // =================================================

    int ultimoPwmEsq = 0;
    int ultimoPwmDir = 0;

    bool linhaPerdida = false;
    unsigned long inicioLinhaPerdida = 0;

    static constexpr unsigned long TEMPO_RECUPERACAO_MS = 500;


    // =================================================
    // CONTROLE
    // =================================================

    static constexpr float PWM_BASE = 90.0f;//150.0

    static constexpr float PID_KP = 0.058f;//0.06
    static constexpr float PID_KD = 0.1f;//0.3

    //Funcionou com 150 0.058 0.1
    //Funcionou com 180 0.078 0.2

    // =================================================
    // DEBUG
    // =================================================

    static constexpr unsigned long INTERVALO_DEBUG_MS = 100;

    unsigned long ultimoDebug = 0;

    const unsigned long TEMPO_CALIBRACAO = 5000;

    // ================================================= 
    // MOTOR ESQUERDO
    // =================================================

    static constexpr uint8_t AIN1 = 18;
    static constexpr uint8_t AIN2 = 5;
    static constexpr uint8_t PWMA = 13;


    // =================================================
    // MOTOR DIREITO
    // =================================================

    static constexpr uint8_t BIN1 = 16;
    static constexpr uint8_t BIN2 = 17;
    static constexpr uint8_t PWMB = 4;


    // =================================================
    // PWM
    // =================================================

    static constexpr uint32_t PWM_FREQ = 1000;
    static constexpr uint8_t PWM_RES = 8;

    static constexpr uint8_t PWM_CHANNEL_A = 0;
    static constexpr uint8_t PWM_CHANNEL_B = 1;


    // =================================================
    // FUNCOES
    // =================================================

    void configurarMotores();

    void motorEsquerdo(int pwm);
    void motorDireito(int pwm);

    void giroCalib();
    void pararMotores();
    void configurarPainelWeb();
    String gerarEstadoJson() const;
    void sincronizarEstado();

    // =================================================
    // SERVIDOR DO PAINEL (tarefa propria)
    // =================================================

    struct ConexaoWeb
    {
        WiFiClient cliente;
        bool ativa = false;
        bool primeiraLinhaLida = false;
        uint8_t tamLinha = 0;
        uint32_t ultimos4 = 0;
        uint32_t inicio = 0;
        char linha[96] = {0};
    };

    static constexpr uint8_t MAX_CONEXOES_WEB = 5;
    static constexpr uint32_t TIMEOUT_CONEXAO_WEB_MS = 2000;

    ConexaoWeb conexoesWeb[MAX_CONEXOES_WEB];

    static void tarefaPainel(void* arg);
    void atenderPainel();
    void processarRequisicao(ConexaoWeb& con);
    static void responderHttp(WiFiClient& cli, int codigo, const char* status,
                              const char* tipo, const char* corpo, size_t tam);


    // =================================================
    // PARADA
    // =================================================

    int contadorDeteccoesDireita = 0;

    bool direitaAnterior = false;
    bool contandoParada = false;
    bool roboParado = false;

    ;

    unsigned long inicioContagemParada = 0;
    static constexpr unsigned long TEMPO_ATE_PARADA_MS = 600;
    
    unsigned long inicioPista = 0;
    bool pistaIniciada = false;

    int deteccoesDireita = 0;
    static constexpr int DETECCOES_NECESSARIAS = 3;
    static constexpr unsigned long TEMPO_PISTA_MS = 16000;
};