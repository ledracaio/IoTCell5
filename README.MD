# 🏭 Célula 5 – Contagem e Rastreamento de Itens na Linha  
## Trabalho 02 – Conectividade MQTT  
### Curso de Sistemas de Informação – UNIDAVI  
**Aluno:** Caio Augusto Ledra  
**Professor:** Esp. Ademar Perfoll Junior  
**Disciplina:** Internet das Coisas (IoT)  

---

## 📘 Descrição do Projeto

Este projeto implementa a **Célula 5** do laboratório de IoT, responsável pela **contagem e rastreamento de itens** em uma linha de produção.  
Na segunda etapa do trabalho, o sistema foi integrado à **Internet das Coisas (IoT)** por meio do **protocolo MQTT**, publicando telemetria e status em um **broker MQTT**, com suporte a **LWT (Last Will & Testament)** e **reconexão automática**.  

O sistema baseia-se em um **Arduino Mega + ESP8266 WiFi Shield**, sensor **HC-SR04**, **LCD 16x2**, **LEDs**, **buzzer** e **botões** (reset e pausa).  
A telemetria é publicada periodicamente e o dispositivo responde a comandos MQTT de consulta de status.

---

## 🧠 Objetivos do Trabalho

- Conectar o dispositivo à rede Wi-Fi e ao broker MQTT.  
- Publicar o **estado do dispositivo** (`online/offline`).  
- Enviar **telemetria periódica** e **eventos pontuais**.  
- Assinar comandos MQTT (ex.: `get_status`).  
- Implementar **reconexão automática** em caso de falha.  
- Garantir **compatibilidade com o padrão de tópicos definido** pela disciplina.

---

## 🌐 Estrutura de Tópicos MQTT

Formato padronizado:

`iot/riodosul/si/BSN22025T26F8/cell/5/device/c5-caio-ledra/`

 ```bash
 | Tipo | Tópico | Direção | QoS | Retained | Descrição |
 |------|---------|----------|------|-----------|------------|
 | Estado | `.../state` | Publicação | 0 | ✅ | Indica se o dispositivo está online/offline |
 | Telemetria | `.../telemetry` | Publicação | 1 | ❌ | Envia dados de sensores e status |
 | Evento | `.../event` | Publicação | 1 | ❌ | Publica eventos como “falha_crítica” |
 | Comando | `.../cmd` | Assinatura | 1 | ❌ | Recebe comandos em JSON |
 | Config | `.../config` | Pub/Sub | 1 | ✅ | Envia e recebe thresholds de operação |
 ```

---

## 📦 Estrutura do Payload JSON

### Telemetria (`telemetry`)
 ```json
 {
   "ts": 1739812345,
   "cellId": 5,
   "devId": "c5-caio-ledra",
   "metrics": {
     "count": 58,
     "velocity": 12
   },
   "status": "normal",
   "units": {
     "count": "itens",
     "velocity": "itens/min"
   },
   "thresholds": {
     "normal": "<=10s",
     "atencao": "<=20s",
     "critico": ">20s"
   }
 }
 ```

### Evento (`event`)
 ```json
 {
   "ts": 1739812401,
   "type": "falha_critica",
   "info": "20s sem item detectado"
 }
 ```

### Comando (`cmd`)
 ```json
 {
   "action": "get_status"
 }
 ```

---

## ⚙️ Funcionalidades Implementadas

✅ Conexão Wi-Fi automática (SSID e senha em `secrets.h`)  
✅ Conexão MQTT com LWT (offline retained)  
✅ Reconexão automática Wi-Fi + MQTT  
✅ Publicação periódica (3s) e on-change (mudança de status)  
✅ Leitura do sensor ultrassônico HC-SR04  
✅ Cálculo de velocidade (itens/minuto)  
✅ Detecção de falhas normal e crítica  
✅ Eventos publicados em MQTT (`event`)  
✅ Assinatura de comandos (`get_status`)  
✅ Exibição de dados no LCD 16x2  

---

## 📋 Tabela de Componentes

| Componente | Quant. | Valor | Função |
|-------------|--------|--------|--------|
| Arduino Mega 2560 | 1 | — | Microcontrolador principal |
| Módulo WiFi ESP8266 | 1 | — | Conectividade MQTT |
| Sensor ultrassônico HC-SR04 | 1 | — | Medição de distância |
| Display LCD 16x2 | 1 | — | Exibição de contagem e status |
| LED Verde | 1 | — | Indicação de item detectado |
| LED Vermelho | 1 | — | Indicação de falha |
| Buzzer | 1 | — | Alerta sonoro |
| Botão Reset | 1 | — | Reinicia contagem |
| Botão Pausa | 1 | — | Pausa alarmes temporariamente |
| Resistores | 4 | 220Ω | Limitação de corrente |
| Protoboard | 2 | — | Conexão do circuito |
| Jumpers | ~20 | — | Conexão do circuito |

---

## 🔌 Mapa de Pinos (Arduino Mega)

| Componente | Pino | Função |
|-------------|------|--------|
| HC-SR04 TRIG | D5 | Saída de trigger |
| HC-SR04 ECHO | D6 | Entrada de eco |
| LCD RS | 48 | Controle RS |
| LCD E | 46 | Enable |
| LCD D4–D7 | 45, 43, 51, 53 | Dados |
| LED Verde | 8 | Sinal de item detectado |
| LED Vermelho | 9 | Alerta de falha |
| Buzzer | 10 | Alerta sonoro |
| Botão Reset | 11 | Reset contador |
| Botão Pausa | 12 | Pausar alerta |

---

## 🧰 Bibliotecas Utilizadas

- `ESP8266WiFi.h`
- `PubSubClient.h`
- `ArduinoJson.h`
- `EEPROM.h`
- `LiquidCrystal.h`

---


## 🧪 Testes Realizados

| Teste | Resultado Esperado | Status |
|--------|-------------------|--------|
| Publicação state=online ao conectar | OK | ✅ |
| Publicação state=offline via LWT | OK | ✅ |
| Telemetria periódica (3s) | OK | ✅ |
| Mudança de status entre normal/atenção/crítico | OK | ✅ |
| Resposta a get_status | OK | ✅ |
| Reconexão Wi-Fi + MQTT automática | OK | ✅ |
| Eventos de falha publicados | OK | ✅ |

---

## 💬 Exemplos de Comandos

Enviar comando MQTT com `mosquitto_pub`:

 ```bash
 mosquitto_pub -h broker.hivemq.com \
 -t "iot/riodosul/si/BSN22025T26F8/cell/5/device/c5-caio-ledra/cmd" \
 -m '{"action":"get_status"}'
 ```

## 🧷 Referências

- **ARDUINO.** Arduino Documentation. Disponível em: <https:docs.arduino.cc/>  
- **ELEGOO.** HC-SR04 Ultrasonic Sensor Datasheet. 2024.  
- **HITACHI.** LCD Controller/Driver HD44780 Datasheet. 2023.  
- **MQTT.** MQTT Essentials - HiveMQ. Disponível em: <https:www.hivemq.com/mqtt/>  
- **UNIDAVI.** Normas para Elaboração de Trabalhos Acadêmicos. 2023.  

## Desenvolvido por @ledracaio
