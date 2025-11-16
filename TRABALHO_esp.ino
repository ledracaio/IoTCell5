/* 
  ESP8266 final - HC-SR04 + MQTT + LEDs + Buttons + Buzzer + EEPROM + Serial->Mega visible
  TRIG = D5 (GPIO14)
  ECHO = D6 (GPIO12)
  Serial baud = 115200
*/

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

// ---------- CONFIG ----------
const char ssid[] = "Unidavi Aberta";
const char password[] = "";

const char broker[] = "a8bb7dc1940348a7939da47ef2447f98.s1.eu.hivemq.cloud";
const int brokerPort = 8883;
const char mqttUser[] = "celula5";
const char mqttPass[] = "CelulaInd5";

int cellId = 5;
const char devId[] = "c5-caio-ledra";

String baseTopic;
String topicState, topicTelemetry, topicEvent, topicCmd, topicConfig;

// ---------- PINS ----------
const uint8_t PIN_TRIG = D5; // GPIO14
const uint8_t PIN_ECHO = D6; // GPIO12

// you said "any empty" — these are safe choices (avoid boot pins issues)
const uint8_t LED_VERDE    = D1; // GPIO5
const uint8_t LED_VERMELHO = D7; // GPIO13

const uint8_t BUZZER       = D0; // GPIO16 (no PWM on some cores — we toggle)
const uint8_t BTN_RESET    = D3; // GPIO0 (active LOW) — don't hold during boot
const uint8_t BTN_PAUSE    = D4; // GPIO2 (active LOW) — don't hold during boot

// ---------- LOGIC PARAMS ----------
long contador = 0;
unsigned long ultimaDeteccao = 0;
unsigned long tempoUltimaPeca = 0;
unsigned long tempoAtual = 0;

int velocidade = 0; // it/min
bool pausarFalha = false;

unsigned long lastBtnReset = 0;
unsigned long lastBtnPause = 0;
const unsigned long debounceBtn = 300;

const int LIMITE_ENTRADA = 10; // cm
const int LIMITE_SAIDA = 15;   // cm
int debounce_ms = 120;
const int LEITURAS_MEDIA = 5;

int ultimoEstado = 0;

unsigned long intervaloFalha = 10000UL;
unsigned long ultimaPublicTelemetry = 0;
const unsigned long TELEMETRY_INTERVAL = 3000UL;

// ---------- NETWORK ----------
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

// Serial buffer
String serialBuf = "";

// EEPROM addresses
const int EEPROM_SIZE = 512;
const int ADDR_COUNTER = 0;

// ---------- HELPERS: HC-SR04 ----------
long medirDistanciaRaw() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long dur = pulseIn(PIN_ECHO, HIGH, 30000); // 30ms timeout
  if (dur == 0) return -1;
  long dist = dur * 0.0343 / 2.0;
  return dist;
}

long medirDistanciaMedia() {
  long soma = 0;
  int cont = 0;
  for (int i = 0; i < LEITURAS_MEDIA; ++i) {
    long d = medirDistanciaRaw();
    if (d > 0) { soma += d; cont++; }
    delay(8);
  }
  if (cont == 0) return -1;
  return soma / cont;
}

// ---------- MQTT helpers ----------
void publishStringTopic(const String &topic, const String &payload, bool retained=false) {
  mqttClient.publish(topic.c_str(), payload.c_str(), retained);
}

void publishTelemetryJSON(const String &status) {
  StaticJsonDocument<256> doc;
  doc["ts"] = (unsigned long)(millis()/1000UL);
  doc["cellId"] = cellId;
  doc["devId"] = devId;

  JsonObject metrics = doc.createNestedObject("metrics");
  metrics["count"] = contador;
  metrics["velocity"] = velocidade;
  metrics["det"] = (millis() - ultimaDeteccao <= 1000) ? 1 : 0;

  doc["status"] = status;

  JsonObject units = doc.createNestedObject("units");
  units["count"] = "itens";
  units["velocity"] = "it/min";
  units["det"] = "0/1";

  JsonObject thresholds = doc.createNestedObject("thresholds");
  thresholds["debounce_ms"] = debounce_ms;
  thresholds["falha_ms"] = intervaloFalha;
  thresholds["critico_ms"] = 2 * intervaloFalha;

  String out; serializeJson(doc, out);
  publishStringTopic(topicTelemetry, out, false);

  // debug -> Serial (visible via Mega bridge)
  Serial.println(out);
}

void publishEvent(const char* tipo, const char* info = "") {
  StaticJsonDocument<128> doc;
  doc["ts"] = (unsigned long)(millis()/1000UL);
  doc["type"] = tipo;
  if (info && strlen(info)>0) doc["info"] = info;
  String out; serializeJson(doc, out);
  publishStringTopic(topicEvent, out, false);
  Serial.println(out);
}

void publishConfigRetained() {
  StaticJsonDocument<128> doc;
  doc["debounce_ms"] = debounce_ms;
  String out; serializeJson(doc, out);
  publishStringTopic(topicConfig, out, true);
}

// ---------- MQTT callback ----------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i=0;i<length;i++) msg += (char)payload[i];
  Serial.print("[MQTT RX] "); Serial.print(topic); Serial.print(" -> "); Serial.println(msg);

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, msg);
  if (!err) {
    if (doc.containsKey("action")) {
      String action = doc["action"].as<String>();
      if (action == "get_status") {
        publishTelemetryJSON("normal");
      } else if (action == "set_thresholds") {
        if (doc.containsKey("data") && doc["data"].containsKey("debounce_ms")) {
          int novo = doc["data"]["debounce_ms"];
          if (novo>0 && novo<60000) {
            debounce_ms = novo;
            publishConfigRetained();
            publishTelemetryJSON("normal");
          }
        }
      }
    }
  }
}

// ---------- WiFi / MQTT ----------
void connectWiFi() {
  Serial.print("WiFi connecting...");
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(300);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi failed");
  }
}

void connectMQTT() {
  mqttClient.setServer(broker, brokerPort);
  mqttClient.setCallback(mqttCallback);
  wifiClient.setInsecure();

  // LWT: PubSubClient doesn't support setWill() easily; many brokers accept last message - we emulate by publishing "online" retained on connect.
  while (!mqttClient.connected()) {
    Serial.print("MQTT connecting...");
    String clientId = String(devId) + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPass)) {
      Serial.println("connected");
      mqttClient.publish(topicState.c_str(), "online", true);
      mqttClient.subscribe(topicCmd.c_str());
      publishConfigRetained();
    } else {
      Serial.print("failed rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 2s");
      delay(2000);
    }
  }
}

// ---------- Button helpers ----------
bool buttonPressed(uint8_t pin) {
  return digitalRead(pin) == LOW; // active LOW, using INPUT_PULLUP
}

// ---------- EEPROM ----------
void saveCounterToEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(ADDR_COUNTER, contador);
  EEPROM.commit();
  EEPROM.end();
  Serial.print("[EEPROM] saved contador="); Serial.println(contador);
}

void loadCounterFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(ADDR_COUNTER, contador);
  if (contador < 0 || contador > 10000000) contador = 0;
  EEPROM.end();
  Serial.print("[EEPROM] loaded contador="); Serial.println(contador);
}

// ---------- Setup ----------
void setupPins() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  pinMode(LED_VERDE, OUTPUT);
  pinMode(LED_VERMELHO, OUTPUT);

  pinMode(BUZZER, OUTPUT);

  pinMode(BTN_RESET, INPUT_PULLUP);
  pinMode(BTN_PAUSE, INPUT_PULLUP);

  digitalWrite(LED_VERDE, LOW);
  digitalWrite(LED_VERMELHO, LOW);
  digitalWrite(BUZZER, LOW);
}

void setup() {
  Serial.begin(115200); // IMPORTANT: set same baud on Mega bridge
  delay(50);

  baseTopic = String("iot/riodosul/si/BSN22025T26F8/cell/") + cellId + "/device/" + devId + "/";
  topicState = baseTopic + "state";
  topicTelemetry = baseTopic + "telemetry";
  topicEvent = baseTopic + "event";
  topicCmd = baseTopic + "cmd";
  topicConfig = baseTopic + "config";

  setupPins();
  loadCounterFromEEPROM();

  connectWiFi();
  connectMQTT();

  publishTelemetryJSON("normal");
}

// ---------- Main loop ----------
void loop() {
  tempoAtual = millis();

  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqttClient.connected()) connectMQTT();
  mqttClient.loop();

  // read sensor
  long distancia = medirDistanciaMedia();

  // detection
  if (distancia > 0 && distancia < LIMITE_ENTRADA && ultimoEstado == 0 && (tempoAtual - tempoUltimaPeca) > (unsigned long)debounce_ms) {
    contador++;
    ultimaDeteccao = tempoAtual;
    if (tempoUltimaPeca > 0) {
      float intervalo_min = (tempoAtual - tempoUltimaPeca) / 60000.0;
      if (intervalo_min > 0) velocidade = int(1.0 / intervalo_min);
    }
    tempoUltimaPeca = tempoAtual;
    ultimoEstado = 1;

    // feedback green
    digitalWrite(LED_VERDE, HIGH);
    delay(120);
    digitalWrite(LED_VERDE, LOW);

    saveCounterToEEPROM();
    publishEvent("item_detectado", String(contador).c_str());
    publishTelemetryJSON(pausarFalha ? "pausado" : "normal");
  }

  if (distancia >= LIMITE_SAIDA) ultimoEstado = 0;

  // failure detection & local outputs
  String statusNow = "normal";
  if (pausarFalha) {
    statusNow = "pausado";
    digitalWrite(LED_VERMELHO, LOW);
    digitalWrite(BUZZER, LOW);
  } else {
    if (tempoAtual - ultimaDeteccao > 2 * intervaloFalha) {
      statusNow = "critico";
      digitalWrite(LED_VERMELHO, HIGH);
      digitalWrite(BUZZER, HIGH);
      publishEvent("falha_critica", "20s_sem_item");
    } else if (tempoAtual - ultimaDeteccao > intervaloFalha) {
      statusNow = "atencao";
      digitalWrite(LED_VERMELHO, HIGH);
      digitalWrite(BUZZER, LOW);
      publishEvent("falha", "10s_sem_item");
    } else {
      statusNow = "normal";
      digitalWrite(LED_VERMELHO, LOW);
      digitalWrite(BUZZER, LOW);
    }
  }

  // periodic telemetry
  if (tempoAtual - ultimaPublicTelemetry >= TELEMETRY_INTERVAL) {
    publishTelemetryJSON(statusNow);
    ultimaPublicTelemetry = tempoAtual;
  }

  // buttons (active LOW)
  if (buttonPressed(BTN_RESET) && tempoAtual - lastBtnReset > debounceBtn) {
    lastBtnReset = tempoAtual;
    contador = 0;
    tempoUltimaPeca = 0;
    velocidade = 0;
    saveCounterToEEPROM();
    publishEvent("reset", "");
    publishTelemetryJSON("normal");
    Serial.println("{\"ack\":\"reset\"}");
  }

  if (buttonPressed(BTN_PAUSE) && tempoAtual - lastBtnPause > debounceBtn) {
    lastBtnPause = tempoAtual;
    pausarFalha = !pausarFalha;
    publishEvent(pausarFalha ? "falha_pausada" : "falha_normal", "");
    publishTelemetryJSON(pausarFalha ? "pausado" : "normal");
    Serial.print("{\"ack\":\"pause\",\"state\":\""); Serial.print(pausarFalha?"pausado":"normal"); Serial.println("\"}");
  }

  // handle serial commands (from Mega or USB)
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      serialBuf.trim();
      if (serialBuf.length() > 0) {
        StaticJsonDocument<128> doc;
        DeserializationError e = deserializeJson(doc, serialBuf);
        if (!e) {
          if (doc.containsKey("cmd")) {
            String cmd = doc["cmd"].as<String>();
            if (cmd == "reset") {
              contador = 0;
              saveCounterToEEPROM();
              publishEvent("reset", "");
              publishTelemetryJSON("normal");
            } else if (cmd == "pause") {
              pausarFalha = !pausarFalha;
              publishEvent(pausarFalha ? "falha_pausada" : "falha_normal", "");
              publishTelemetryJSON(pausarFalha ? "pausado" : "normal");
            } else if (cmd == "set_threshold") {
              if (doc.containsKey("debounce_ms")) {
                int novo = doc["debounce_ms"];
                if (novo>0 && novo<60000) {
                  debounce_ms = novo;
                  publishConfigRetained();
                  publishTelemetryJSON("normal");
                }
              }
            }
          }
        } else {
          Serial.print("[FROM_USB] "); Serial.println(serialBuf);
        }
      }
      serialBuf = "";
    } else {
      serialBuf += c;
    }
  }

  delay(20);
}
