#include "robot.h"
#include "../web_panel.h"


// =================================================
// BEGIN
// =================================================

void Robot::begin()
{
    Serial.begin(115200);
    pinMode(LED_INTERNO, OUTPUT);
    digitalWrite(LED_INTERNO, seguimentoAtivo ? HIGH : LOW);
    // SENSORES
    sensoresRobot.begin();

    // MOTORES
    configurarMotores();

    pararMotores();


    // =================================================
    // CALIBRACAO
    // =================================================

    Serial.println("Iniciando calibracao...");

    giroCalib();

    sensoresRobot.calibrate(TEMPO_CALIBRACAO);

    pararMotores();


    // =================================================
    // CONTROLE
    // =================================================

    controladorPID.reset();

    // Painel local: o celular conecta diretamente ao Wi-Fi criado pelo ESP32.
    configurarPainelWeb();

    Serial.println("Robot pronto!");
    inicioPista = millis();
    pistaIniciada = true;
}


void Robot::setSeguidorAtivo(bool ativo)
{
    // Chamado pela tarefa do painel (outro nucleo). Aqui so se publica o pedido;
    // motores, PID e LED sao atuados pelo laco de controle em sincronizarEstado().
    // Ordem importa: primeiro o valor, depois o aviso de mudanca.
    seguimentoAtivo = ativo;
    estadoAlterado = true;
}


// Executa no laco de controle (update), nunca na tarefa web.
void Robot::sincronizarEstado()
{
    const bool ativo = seguimentoAtivo;

    digitalWrite(LED_INTERNO, ativo ? HIGH : LOW);

    controladorPID.reset();
    linhaPerdida = false;
    ultimoPwmEsq = 0;
    ultimoPwmDir = 0;
    contandoParada = false;
    direitaAnterior = sensoresRobot.direitaDetected();

    if (!ativo)
    {
        pararMotores();
        Serial.println("Seguidor desativado pelo painel; calibracao preservada.");
    }
    else
    {
        // Nova sessao sem chamar calibrate().
        roboParado = false;
        pistaIniciada = true;
        inicioPista = millis();
        deteccoesDireita = 0;
        Serial.println("Seguidor ativado pelo painel; usando calibracao atual.");
    }
}

// =================================================
// UPDATE
// =================================================

void Robot::update()
{
    // O painel roda em tarefa propria; aqui so se aplica o comando pendente.
    // Fica ANTES da leitura dos sensores para o desligamento ser o mais rapido possivel.
    if (estadoAlterado.exchange(false))
    {
        sincronizarEstado();
    }

    // =================================================
    // LEITURA DOS SENSORES
    // =================================================

    sensoresRobot.update();

    // O modo desligado para apenas o movimento. Sensores, encoders e painel continuam ativos.
    if (!seguimentoAtivo)
    {
        pararMotores();
        return;
    }

    if (roboParado)
    {
        pararMotores();
        return;
    }


    // =================================================
    // SENSOR LATERAL DIREITO
    // =================================================

    bool direitaAtual = sensoresRobot.direitaDetected();

    if (!contandoParada &&
        !roboParado &&
        pistaIniciada &&
        (millis() - inicioPista >= TEMPO_PISTA_MS) &&
        direitaAtual &&
        !direitaAnterior)
    {
        deteccoesDireita++;

        Serial.print("DETECCAO DIREITA #");
        Serial.println(deteccoesDireita);

        // Só começa a contagem para parada na 3ª detecção
        if (deteccoesDireita >= DETECCOES_NECESSARIAS)
        {
            inicioContagemParada = millis();
            contandoParada = true;

            Serial.println("3 DETECCOES! INICIANDO CONTAGEM PARA PARADA");
        }
    }

    direitaAnterior = direitaAtual;

    // =================================================
    // CONTAGEM
    // =================================================

    if (contandoParada)
    {
        if (millis() - inicioContagemParada >=
            TEMPO_ATE_PARADA_MS)
        {
            pararMotores();

            contandoParada = false;
            roboParado = true;
            pistaIniciada = false;

            Serial.println("ROBO PARADO!");

            return;
        }
    }


    // =================================================
    // LINHA PERDIDA
    // =================================================

    if (!sensoresRobot.linhaDetectada())
    {
        if (!linhaPerdida)
        {
            linhaPerdida = true;

            inicioLinhaPerdida =
                millis();

            Serial.println(
                "LINHA PERDIDA! Mantendo ultimo PWM..."
            );
        }


        // Mantem o ultimo comando
        if (millis() - inicioLinhaPerdida <
            TEMPO_RECUPERACAO_MS)
        {
            motorEsquerdo(ultimoPwmEsq);
            motorDireito(ultimoPwmDir);

            return;
        }


        // Linha nao recuperada
        pararMotores();

        controladorPID.reset();

        Serial.println(
            "LINHA NAO RECUPERADA! MOTORES PARADOS."
        );

        return;
    }


    // Linha encontrada novamente
    linhaPerdida = false;


    // =================================================
    // CALCULO DO ERRO
    // =================================================

    int erroAtual =
        sensoresRobot.calculaErro();


    // =================================================
    // CONTROLE
    // =================================================

    float correcao = 0.0f;

    int pwmEsq = 0;
    int pwmDir = 0;


    // =================================================
    // FUZZY
    // =================================================

    if (CONTROLE_ATUAL == CONTROLE_FUZZY)
    {
        // O fuzzy precisa da variacao do erro.
        static int erroAnteriorFuzzy = 0;

        int varErro =
            erroAtual - erroAnteriorFuzzy;

        erroAnteriorFuzzy =
            erroAtual;


        valoresFuzzy regras =
            controladorFuzzy.funcoes(
                erroAtual,
                varErro
            );


        controladorDefuzzy.calcularPWM(
            regras,
            PWM_BASE
        );


        pwmEsq =
            controladorDefuzzy.pwmEsq;

        pwmDir =
            controladorDefuzzy.pwmDir;

        correcao =
            controladorDefuzzy.getCorrecao();
    }


    // =================================================
    // PD
    // =================================================

    else if (CONTROLE_ATUAL == CONTROLE_PID)
    {
        correcao =
            controladorPID.calcular(
                erroAtual
            );


        // Correcao positiva:
        //
        // Esquerdo aumenta
        // Direito diminui
        //
        // Correcao negativa:
        //
        // Esquerdo diminui
        // Direito aumenta

        pwmEsq =
            PWM_BASE + correcao;

        pwmDir =
            PWM_BASE - correcao;


        pwmEsq =
            constrain(
                pwmEsq,
                0,
                PWM_MAX
            );


        pwmDir =
            constrain(
                pwmDir,
                0,
                PWM_MAX
            );
    }


    // =================================================
    // ATUACAO
    // =================================================

    motorEsquerdo(pwmEsq);
    motorDireito(pwmDir);


    // Guarda o ultimo PWM
    // para recuperacao da linha

    ultimoPwmEsq = pwmEsq;
    ultimoPwmDir = pwmDir;


    // =================================================
    // DEBUG
    // =================================================

    if (millis() - ultimoDebug >=
        INTERVALO_DEBUG_MS)
    {
        ultimoDebug = millis();


        const uint16_t* valores =
            sensoresRobot.getValues();


        Serial.print("SENSORES: ");


        for (int i = 0; i < NUM_SENSORS; i++)
        {
            Serial.print(valores[i]);


            if (i < NUM_SENSORS - 1)
            {
                Serial.print(" | ");
            }
        }


        Serial.print(" || Erro: ");
        Serial.print(erroAtual);


        Serial.print(" | Correcao: ");
        Serial.print(correcao);


        Serial.print(" | PWM Esq: ");
        Serial.print(pwmEsq);


        Serial.print(" | PWM Dir: ");
        Serial.print(pwmDir);


        // =================================================
        // DEBUG SENSORES LATERAIS
        // =================================================

        Serial.print(" || LATERAIS | ");

        Serial.print("D: ");
        Serial.print(sensoresRobot.getLeituraDireita());

        Serial.print(" [");
        Serial.print(sensoresRobot.getMinDireita());
        Serial.print("-");
        Serial.print(sensoresRobot.getMaxDireita());
        Serial.print("] ");

        Serial.print(
            sensoresRobot.direitaDetected()
                ? "DETECTOU"
                : "NAO"
        );

        Serial.print(" | E: ");
        Serial.print(sensoresRobot.getLeituraEsquerda());

        Serial.print(" [");
        Serial.print(sensoresRobot.getMinEsquerda());
        Serial.print("-");
        Serial.print(sensoresRobot.getMaxEsquerda());
        Serial.print("] ");

        Serial.println(
            sensoresRobot.esquerdaDetected()
                ? "DETECTOU"
                : "NAO"
        );
    }
}


// =================================================
// CONFIGURACAO DOS MOTORES
// =================================================

void Robot::configurarMotores()
{
    // Motor esquerdo
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);


    // Motor direito
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);


    // PWM
    ledcSetup(
        PWM_CHANNEL_A,
        PWM_FREQ,
        PWM_RES
    );


    ledcSetup(
        PWM_CHANNEL_B,
        PWM_FREQ,
        PWM_RES
    );


    ledcAttachPin(
        PWMA,
        PWM_CHANNEL_A
    );


    ledcAttachPin(
        PWMB,
        PWM_CHANNEL_B
    );


    // Sentido para frente
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);


    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);


    // Comeca parado
    ledcWrite(
        PWM_CHANNEL_A,
        0
    );


    ledcWrite(
        PWM_CHANNEL_B,
        0
    );
}


// =================================================
// MOTOR ESQUERDO
// =================================================

void Robot::motorEsquerdo(int pwm)
{
    pwm =
        constrain(
            pwm,
            0,
            PWM_MAX
        );


    digitalWrite(
        AIN1,
        HIGH
    );


    digitalWrite(
        AIN2,
        LOW
    );


    ledcWrite(
        PWM_CHANNEL_A,
        pwm
    );
}


// =================================================
// MOTOR DIREITO
// =================================================

void Robot::motorDireito(int pwm)
{
    pwm =
        constrain(
            pwm,
            0,
            PWM_MAX
        );


    digitalWrite(
        BIN1,
        HIGH
    );


    digitalWrite(
        BIN2,
        LOW
    );


    ledcWrite(
        PWM_CHANNEL_B,
        pwm
    );
}


// =================================================
// GIRO DE CALIBRACAO
// =================================================

void Robot::giroCalib()
{
    digitalWrite(
        AIN1,
        LOW
    );

    digitalWrite(
        AIN2,
        LOW
    );

    ledcWrite(
        PWM_CHANNEL_A,
        180
    );


    digitalWrite(
        BIN1,
        LOW
    );

    digitalWrite(
        BIN2,
        LOW
    );

    ledcWrite(
        PWM_CHANNEL_B,
        180
    );
}


// =================================================
// PARAR MOTORES
// =================================================

void Robot::pararMotores()
{
    ultimoPwmEsq = 0;
    ultimoPwmDir = 0;
    digitalWrite(
        AIN1,
        HIGH
    );

    digitalWrite(
        AIN2,
        HIGH
    );

    ledcWrite(
        PWM_CHANNEL_A,
        0
    );


    digitalWrite(
        BIN1,
        HIGH
    );

    digitalWrite(
        BIN2,
        HIGH
    );

    ledcWrite(
        PWM_CHANNEL_B,
        0
    );
}

// =================================================
// PAINEL WEB LOCAL
// =================================================

void Robot::configurarPainelWeb()
{
    WiFi.mode(WIFI_AP);

    // Sem economia de energia do radio: evita atrasos de ate centenas de ms.
    WiFi.setSleep(false);

    // canal 1, SSID visivel, no maximo 2 clientes (menos disputa de ar).
    // Se o ambiente tiver muitas redes 2.4 GHz, teste os canais 6 e 11.
    const bool apOk = WiFi.softAP("CorredorPolones", "CorredorPolones56", 1, 0, 2);

    if (apOk)
    {
        Serial.println();
        Serial.println("Painel web iniciado.");
        Serial.print("Wi-Fi: CorredorPolones | Endereco: http://");
        Serial.println(WiFi.softAPIP());
    }
    else
    {
        Serial.println("ERRO: nao foi possivel iniciar o Wi-Fi do painel.");
    }

    servidorWeb.begin();

    // Tarefa propria no nucleo 0 (o laco de controle roda no nucleo 1).
    xTaskCreatePinnedToCore(
        tarefaPainel,
        "painel_web",
        6144,
        this,
        1,
        nullptr,
        0
    );
}


void Robot::tarefaPainel(void* arg)
{
    Robot* self = static_cast<Robot*>(arg);

    for (;;)
    {
        self->atenderPainel();
        vTaskDelay(1);   // ~1 ms
    }
}


// Servidor nao bloqueante: varias conexoes ao mesmo tempo. Uma conexao
// ociosa (o Chrome abre sockets "reserva") nao trava mais as demais.
void Robot::atenderPainel()
{
    const uint32_t agora = millis();

    // 1) Aceita conexoes novas
    WiFiClient novo = servidorWeb.available();

    while (novo)
    {
        int livre = -1;
        int maisAntiga = 0;

        for (int i = 0; i < MAX_CONEXOES_WEB; ++i)
        {
            if (!conexoesWeb[i].ativa)
            {
                livre = i;
                break;
            }

            if ((agora - conexoesWeb[i].inicio) >
                (agora - conexoesWeb[maisAntiga].inicio))
            {
                maisAntiga = i;
            }
        }

        if (livre < 0)
        {
            conexoesWeb[maisAntiga].cliente.stop();
            livre = maisAntiga;
        }

        ConexaoWeb& c = conexoesWeb[livre];
        c.cliente = novo;
        c.ativa = true;
        c.primeiraLinhaLida = false;
        c.tamLinha = 0;
        c.ultimos4 = 0;
        c.inicio = agora;
        c.linha[0] = 0;

        novo = servidorWeb.available();
    }

    // 2) Le o que chegou em cada conexao, sem esperar
    for (int i = 0; i < MAX_CONEXOES_WEB; ++i)
    {
        ConexaoWeb& c = conexoesWeb[i];

        if (!c.ativa)
        {
            continue;
        }

        bool completo = false;
        int disponiveis = c.cliente.available();

        while (disponiveis-- > 0 && !completo)
        {
            const int ch = c.cliente.read();

            if (ch < 0)
            {
                break;
            }

            if (!c.primeiraLinhaLida)
            {
                if (ch == '\n')
                {
                    c.primeiraLinhaLida = true;
                }
                else if (c.tamLinha < sizeof(c.linha) - 1)
                {
                    c.linha[c.tamLinha++] = (char)ch;
                }
            }

            // Fim dos cabecalhos HTTP: \r\n\r\n
            c.ultimos4 = (c.ultimos4 << 8) | (uint8_t)ch;

            if (c.ultimos4 == 0x0D0A0D0Au)
            {
                completo = true;
            }
        }

        if (completo)
        {
            c.linha[c.tamLinha] = 0;
            processarRequisicao(c);
            c.cliente.stop();
            c.ativa = false;
        }
        else if (!c.cliente.connected() && c.cliente.available() == 0)
        {
            c.cliente.stop();
            c.ativa = false;
        }
        else if ((agora - c.inicio) > TIMEOUT_CONEXAO_WEB_MS)
        {
            c.cliente.stop();
            c.ativa = false;
        }
    }
}


void Robot::responderHttp(WiFiClient& cli, int codigo, const char* status,
                          const char* tipo, const char* corpo, size_t tam)
{
    char cab[192];

    const int n = snprintf(
        cab, sizeof(cab),
        "HTTP/1.1 %d %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %u\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n",
        codigo, status, tipo, (unsigned)tam
    );

    if (tam > 0 && tam <= 1024)
    {
        // Resposta pequena: cabecalho + corpo em um unico pacote.
        String resp;
        resp.reserve(n + tam);
        resp += cab;
        resp += corpo;
        cli.write((const uint8_t*)resp.c_str(), resp.length());
    }
    else
    {
        cli.write((const uint8_t*)cab, n);

        if (tam > 0)
        {
            cli.write((const uint8_t*)corpo, tam);
        }
    }
}


void Robot::processarRequisicao(ConexaoWeb& con)
{
    // Primeira linha: "GET /api/data HTTP/1.1"
    const char* p = strchr(con.linha, ' ');

    if (!p)
    {
        responderHttp(con.cliente, 400, "Bad Request", "text/plain", "", 0);
        return;
    }

    ++p;

    char caminho[32];
    size_t n = 0;

    while (*p && *p != ' ' && *p != '?' && n < sizeof(caminho) - 1)
    {
        caminho[n++] = *p++;
    }

    caminho[n] = 0;

    if (strcmp(caminho, "/api/data") == 0)
    {
        const String json = gerarEstadoJson();
        responderHttp(con.cliente, 200, "OK", "application/json",
                      json.c_str(), json.length());
    }
    else if (strcmp(caminho, "/api/follow") == 0)
    {
        // enabled=0 desliga; qualquer outro valor liga (igual ao codigo original)
        const char* q = strstr(con.linha, "enabled=");
        const bool novoEstado = !(q && q[8] == '0');

        if (novoEstado != seguimentoAtivo)
        {
            setSeguidorAtivo(novoEstado);
        }

        // O estado devolvido ja e o novo; os motores sao atuados no proximo ciclo.
        const String json = gerarEstadoJson();
        responderHttp(con.cliente, 200, "OK", "application/json",
                      json.c_str(), json.length());
    }
    else if (strcmp(caminho, "/") == 0)
    {
        responderHttp(con.cliente, 200, "OK", "text/html; charset=utf-8",
                      PAINEL_HTML, sizeof(PAINEL_HTML) - 1);
    }
    else if (strcmp(caminho, "/favicon.ico") == 0)
    {
        responderHttp(con.cliente, 204, "No Content", "text/plain", "", 0);
    }
    else
    {
        responderHttp(con.cliente, 404, "Not Found", "application/json",
                      "{\"error\":\"not_found\"}", 21);
    }
}

// Converte o RPM do encoder para a escala 0-255 do PWM.
static int rpmParaPwm(float rpm)
{
    const float v = fabsf(rpm) / RPM_MAXIMO_PWM255 * PWM_MAX;
    return constrain((int)(v + 0.5f), 0, PWM_MAX);
}


String Robot::gerarEstadoJson() const
{
    String json;
    json.reserve(480);

    const uint16_t* valores = sensoresRobot.getValues();

    json = "{\"sensors\":[";
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        if (i > 0) json += ',';
        json += String(valores[i]);
    }

    json += "],\"error\":";
    json += String(sensoresRobot.calculaErro());
    json += ",\"line\":";
    json += sensoresRobot.linhaDetectada() ? "true" : "false";
    json += ",\"follow\":";
    json += seguimentoAtivo ? "true" : "false";
    json += ",\"pwmLeft\":";
    json += String(seguimentoAtivo ? ultimoPwmEsq : 0);
    json += ",\"pwmRight\":";
    json += String(seguimentoAtivo ? ultimoPwmDir : 0);
    json += ",\"rpmLeft\":";
    json += String(sensoresRobot.getVelocidadeEsq(), 1);
    json += ",\"rpmRight\":";
    json += String(sensoresRobot.getVelocidadeDir(), 1);
    json += ",\"encPwmLeft\":";
    json += String(rpmParaPwm(sensoresRobot.getVelocidadeEsq()));
    json += ",\"encPwmRight\":";
    json += String(rpmParaPwm(sensoresRobot.getVelocidadeDir()));
    json += ",\"leftRaw\":";
    json += String(sensoresRobot.getLeituraEsquerda());
    json += ",\"rightRaw\":";
    json += String(sensoresRobot.getLeituraDireita());
    json += ",\"leftMin\":";
    json += String(sensoresRobot.getMinEsquerda());
    json += ",\"leftMax\":";
    json += String(sensoresRobot.getMaxEsquerda());
    json += ",\"rightMin\":";
    json += String(sensoresRobot.getMinDireita());
    json += ",\"rightMax\":";
    json += String(sensoresRobot.getMaxDireita());
    json += ",\"leftDetected\":";
    json += sensoresRobot.esquerdaDetected() ? "true" : "false";
    json += ",\"rightDetected\":";
    json += sensoresRobot.direitaDetected() ? "true" : "false";
    json += "}";

    return json;
}
