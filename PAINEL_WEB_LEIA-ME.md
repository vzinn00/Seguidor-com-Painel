# Seguidor com painel web local

## O que foi adicionado

- O ESP32 cria uma rede Wi-Fi própria: `CorredorPolones`.
- Senha: `CorredorPolones56`.
- Abra no celular: `http://192.168.4.1`.
- Painel atualiza os dados automaticamente a cada 200 ms.
- Mostra os oito sensores QTR, erro, sensores laterais, detecção lateral e RPM dos encoders.
- O botão **Desativar** para os motores sem desligar o ESP32 nem chamar a rotina de calibração.
- Ao ativar novamente, o controle PID é reiniciado para evitar aproveitar o erro anterior.

## Como gravar

1. Abra a pasta do projeto no VS Code com PlatformIO.
2. Compile e grave no ESP32 como de costume.
3. Aguarde a calibração inicial terminar.
4. No celular, conecte-se à rede `Seguidor-ESP32` com a senha `seguidor32`.
5. Acesse `http://192.168.4.1` no navegador.

A rede criada pelo ESP32 é local e não fornece internet. Se o celular avisar que a rede não tem internet, permaneça conectado a ela.

## Observações importantes

- A calibração atual continua em RAM, como no projeto original. Desativar/ativar o seguidor não a apaga; reiniciar ou desligar a alimentação ainda executa novamente a calibração inicial.
- O servidor usa `WiFi.h` e `WebServer.h`, incluídos no framework Arduino-ESP32; não foi adicionada dependência externa ao `platformio.ini`.
- A compilação física e o teste com o seu robô ainda precisam ser feitos no seu ambiente/ESP32. Teste primeiro com as rodas suspensas, pois o botão atua diretamente no controle dos motores.

## Correção de latência do liga/desliga

- O servidor HTTP agora roda em tarefa própria (núcleo 0), sem depender do `delay(20)` do laço de controle.
- O comando só publica um pedido (`std::atomic`); o laço de controle aplica no início do próximo ciclo.
- O painel mostra "leitura X ms" e "último comando Y ms" no rodapé, para medir a latência real.
- Wi-Fi sem economia de energia, canal 1, máximo de 2 clientes. Se houver muitas redes 2,4 GHz no local, teste os canais 6 e 11.
- Desative os dados móveis do celular enquanto usa o painel.
