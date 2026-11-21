#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <atomic>

#include "hardware/sensores.h"
#include "control/fuzzy/funcreg.h"
#include "control/fuzzy/defuzzy.h"
#include "control/pid/pid.h"

// Configurações compartilhadas pelo controle e pelo painel.
static constexpr int PWM_MAX = 255;
static constexpr float RPM_MAXIMO_PWM255 = 2250.0f;

class Robot
{
public:
    void begin();
    void update();
    void setSeguidorAtivo(bool ativo);

private:

    // Escolha do critério de parada
    enum class ModoParada
    {
        TEMPO,
        DISTANCIA
    };

    // ALTERE AQUI O MODO DE PARADA
    static constexpr ModoParada MODO_PARADA = ModoParada::DISTANCIA;
    // Parada por tempo
    static constexpr unsigned long TEMPO_ATE_PARADA_MS = 21000;
    // Parada por distância
    static constexpr float DISTANCIA_ATE_PARADA_MM = 1500.0f;

    static constexpr uint8_t LED_INTERNO = 22;
    
    // =================================================
    // CONFIGURACAO DO CONTROLE
    // =================================================

    static constexpr float RPM_BASE = 800.0f;
    static constexpr float RPM_CORRECAO_MAX = 500.0f;
    static constexpr float PWM_BASE = 90.0f;
    static constexpr float PWM_CORRECAO_MAX = 100.0f;

    static constexpr float PID_KP_TRAJETORIA = 0.51f;
    static constexpr float PID_KD_TRAJETORIA = 0.018f;

    static constexpr float PID_KP_VELOCIDADE = 0.08f;
    static constexpr float PID_KI_VELOCIDADE = 0.5f;


    // Configure o atraso desejado antes de começar a contar as marcas laterais.
    static constexpr unsigned long TEMPO_PISTA_MS = 0;
    static constexpr int DETECCOES_NECESSARIAS = 3;

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

    // PD responsável pelo seguimento da linha
    PID controladorTrajetoria{
        PID_KP_TRAJETORIA,
        0.0f,
        PID_KD_TRAJETORIA,
        -RPM_CORRECAO_MAX,
        RPM_CORRECAO_MAX
    };

    // PI responsável pela velocidade dos motores
    PID controladorVelEsq{
        PID_KP_VELOCIDADE,
        PID_KI_VELOCIDADE,
        0.0f,
        -PWM_CORRECAO_MAX,
        PWM_CORRECAO_MAX
    };

    PID controladorVelDir{
        PID_KP_VELOCIDADE,
        PID_KI_VELOCIDADE,
        0.0f,
        -PWM_CORRECAO_MAX,
        PWM_CORRECAO_MAX
    };


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

    unsigned long ultimoControle = 0;

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


    // Detecção do sensor lateral
    bool direitaAnterior = false;
    bool contandoParada = false;
    bool roboParado = false;

    unsigned long inicioContagemParada = 0;

    // Medição de distância
    float distanciaTotalMm = 0.0f;
    float distanciaInicialParada = 0.0f;
    float ultimaDistanciaEsqMm = 0.0f;
    float ultimaDistanciaDirMm = 0.0f;

    void atualizarDistanciaPercorrida();

    // Controle da pista
    unsigned long inicioPista = 0;
    bool pistaIniciada = false;

    int deteccoesDireita = 0;
};