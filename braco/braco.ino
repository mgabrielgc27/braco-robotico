#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <ESP32Servo.h>

const int N = 6;
const int CANAL = 1;

const int pinosServo[N] = { 18, 23, 22, 21, 19, 13 };

const int angMin[N] = { 0, 0, 0, 0, 0, 110 };
const int angMax[N] = { 180, 165, 180, 180, 180, 170 };
const int angInicial[N] = { 90, 90, 90, 90, 90, 170 };

const unsigned long tmoSinal = 2000;

Servo servos[N];

typedef struct {
  uint8_t ang[N];
} Pacote;

uint8_t macBroadcast[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

QueueHandle_t filaPacotes;
SemaphoreHandle_t mutexEstado;

int alvo[N];      // Onde o controle quer que o braço vá
int angAtual[N];  // Onde o braço fisicamente está agora
bool conectado = false;

void aoReceber(const esp_now_recv_info_t *info, const uint8_t *dados, int tam) {
  if (tam == sizeof(Pacote)) {
    Pacote p;
    memcpy(&p, dados, sizeof(Pacote));
    // Coloca na fila
    xQueueSendFromISR(filaPacotes, &p, NULL);
  }
}

// Tarefa 1: Gerencia a conexão, lê a fila e responde com ACKs (Core 0)
void TaskComunicacao(void *pvParameters) {
  Pacote p;
  
  for (;;) {
    // Fica travada aqui aguardando um pacote. Se passar tmoSinal (2000ms) sem chegar nada, retorna falso.
    if (xQueueReceive(filaPacotes, &p, pdMS_TO_TICKS(tmoSinal)) == pdTRUE) {
      
      xSemaphoreTake(mutexEstado, portMAX_DELAY);
      if (!conectado) Serial.println("Controle Conectado!");
      conectado = true;
      for (int i = 0; i < N; i++) {
        alvo[i] = p.ang[i];
      }
      xSemaphoreGive(mutexEstado);

      uint8_t ok = 1;
      esp_now_send(macBroadcast, &ok, 1);
      
    } else {
      // Timeout! Passaram-se 2 segundos sem pacotes.
      xSemaphoreTake(mutexEstado, portMAX_DELAY);
      if (conectado) Serial.println("Sinal perdido! Retornando à posição segura.");
      conectado = false;
      // Define a posição segura para onde o braço deve ir ao perder conexão
      for (int i = 0; i < N; i++) {
        alvo[i] = angInicial[i];
      }
      xSemaphoreGive(mutexEstado);
    }
  }
}

// Tarefa 2: Movimentação Constante e Suave dos Servos (Core 1)
void TaskMotores(void *pvParameters) {
  for (;;) {
    int alvosLocais[N];

    xSemaphoreTake(mutexEstado, portMAX_DELAY);
    for (int i = 0; i < N; i++) alvosLocais[i] = alvo[i];
    xSemaphoreGive(mutexEstado);

    // Lógica de velocidade constante (1 grau por ciclo)
    for (int i = 0; i < N; i++) {
      if (angAtual[i] < alvosLocais[i]) {
        angAtual[i]++;
        servos[i].write(angAtual[i]);
      } 
      else if (angAtual[i] > alvosLocais[i]) {
        angAtual[i]--;
        servos[i].write(angAtual[i]);
      }
    }

    // O tempo do delay dita a VELOCIDADE MÁXIMA do braço.
    // 15ms = movimento macio. Diminuir para mais rápido, aumentar para mais lento.
    vTaskDelay(pdMS_TO_TICKS(15)); 
  }
}

void setup() {
  Serial.begin(115200);
  
  filaPacotes = xQueueCreate(10, sizeof(Pacote));
  mutexEstado = xSemaphoreCreateMutex();

  for (int i = 0; i < N; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(pinosServo[i], 500, 2500);
    angAtual[i] = angInicial[i];
    alvo[i] = angInicial[i];
    servos[i].write(angInicial[i]);
    delay(1000); // Liga um por vez para evitar pico de corrente
  }

  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(CANAL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ESP-NOW");
    while (true) vTaskDelay(portMAX_DELAY);
  }

  esp_now_register_recv_cb(aoReceber);

  esp_now_peer_info_t par = {};
  memcpy(par.peer_addr, macBroadcast, 6);
  par.channel = CANAL;
  par.encrypt = false;
  esp_now_add_peer(&par);

  xTaskCreatePinnedToCore(TaskComunicacao, "Comm", 4000, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskMotores, "Motores", 4000, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelete(NULL);
}