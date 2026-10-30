#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int N = 6;
const int CANAL = 1;

const int pinosPot[N] = { 39, 36, 34, 35, 32, 33 };
const char *nomes[N] = { "Base", "Ombro", "Cotovelo", "Pulso", "Giro", "Garra" };
const int angMin[N] = { 0, 0, 0, 0, 0, 110 };
const int angMax[N] = { 180, 165, 180, 180, 180, 170 };
const int angInicial[N] = { 90, 0, 90, 90, 90, 110 };

const int tolerancia = 3;
const float fatorFiltro = 0.25;
const unsigned long tmoConexao = 2000;

LiquidCrystal_I2C lcd(0x27, 16, 2);

// O pacote agora contém APENAS os ângulos (6 bytes)
typedef struct {
  uint8_t ang[N];
} Pacote;

Pacote pct;
uint8_t macBroadcast[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

SemaphoreHandle_t mutexEstado;

float leituraFiltrada[N];
int angPot[N];
bool calibrado[N] = { false, false, false, false, false, false };
bool sistemaCalibrado = false;
bool conectado = false;

int faltaCalibrar = N;
int idFaltaCalibrar = 0;

volatile unsigned long ultimoAck = 0;

// --- FUNÇÕES BÁSICAS ---
int lerAngulo(int i) {
  long soma = 0;
  for (int k = 0; k < 8; k++) soma += analogRead(pinosPot[i]);
  int leitura = soma / 8;
  int ang = map(leitura, 0, 4095, angMin[i], angMax[i]);
  return constrain(ang, angMin[i], angMax[i]);
}

void tela(const char *l1, const char *l2) {
  char b1[17], b2[17];
  snprintf(b1, sizeof(b1), "%-16s", l1);
  snprintf(b2, sizeof(b2), "%-16s", l2);
  lcd.setCursor(0, 0);
  lcd.print(b1);
  lcd.setCursor(0, 1);
  lcd.print(b2);
}

void aoReceber(const esp_now_recv_info_t *info, const uint8_t *dados, int tam) {
  if (info == nullptr || dados == nullptr) return;
  if (tam == 1 && dados[0] == 1) {
    ultimoAck = millis();
  }
}

// --- TAREFAS DO FREERTOS ---

// Tarefa 1: Lê analógicos, aplica filtro e checa a calibração (Core 1)
void TaskSensores(void *pvParameters) {
  for (;;) {
    // Lê os pinos FORA do Mutex para não prender o processador durante as leituras
    int leiturasTemp[N];
    for (int i = 0; i < N; i++) {
      leiturasTemp[i] = lerAngulo(i);
    }

    // Entra na área crítica para atualizar variáveis globais
    xSemaphoreTake(mutexEstado, portMAX_DELAY);

    for (int i = 0; i < N; i++) {
      leituraFiltrada[i] = leituraFiltrada[i] * (1.0 - fatorFiltro) + leiturasTemp[i] * fatorFiltro;
      angPot[i] = (int)(leituraFiltrada[i] + 0.5);
    }

    if (conectado && !sistemaCalibrado) {
      int falta = 0;
      int primeiro = -1;
      for (int i = 0; i < N; i++) {
        if (!calibrado[i] && abs(angPot[i] - angInicial[i]) <= tolerancia) {
          calibrado[i] = true;
        }
        if (!calibrado[i]) {
          falta++;
          if (primeiro < 0) primeiro = i;
        }
      }
      faltaCalibrar = falta;
      idFaltaCalibrar = primeiro;
      if (falta == 0) sistemaCalibrado = true;

    } else if (!conectado) {
      // Se cair a conexão, reseta a calibração
      sistemaCalibrado = false;
      faltaCalibrar = N;
      idFaltaCalibrar = 0;
      for (int i = 0; i < N; i++) calibrado[i] = false;
    }

    xSemaphoreGive(mutexEstado);

    vTaskDelay(pdMS_TO_TICKS(15));  // Roda a cada 15ms
  }
}

// Tarefa 2: Gerencia Timeout e Envia pacotes ESP-NOW (Core 0)
void TaskEnvio(void *pvParameters) {
  for (;;) {
    unsigned long agora = millis();
    bool conn = (ultimoAck != 0) && ((agora - ultimoAck) < tmoConexao);

    xSemaphoreTake(mutexEstado, portMAX_DELAY);
    conectado = conn;
    bool sysCalib = sistemaCalibrado;

    // Monta o pacote
    for (int i = 0; i < N; i++) {
      if (sysCalib) {
        // Se calibrado, envia a posição real do potenciômetro
        pct.ang[i] = constrain(angPot[i], angMin[i], angMax[i]);
      } else {
        // Se NÃO calibrado, envia a posição inicial para "travar" o braço no lugar seguro
        pct.ang[i] = angInicial[i];
      }
    }
    xSemaphoreGive(mutexEstado);

    esp_now_send(macBroadcast, (uint8_t *)&pct, sizeof(pct));

    vTaskDelay(pdMS_TO_TICKS(20));  // Roda a cada 20ms
  }
}

// Tarefa 3: Atualiza o LCD (Core 1)
void TaskLCD(void *pvParameters) {
  int estadoLcd = -1;
  int angulosParaExibir[N];

  for (;;) {
    // Copia as variáveis rapidamente para não travar outras tarefas
    xSemaphoreTake(mutexEstado, portMAX_DELAY);
    bool conn = conectado;
    bool sysCalib = sistemaCalibrado;
    int falta = faltaCalibrar;
    int idFalta = idFaltaCalibrar;
    int potAtual = (idFalta >= 0) ? angPot[idFalta] : 0;
    int alvoAtual = (idFalta >= 0) ? angInicial[idFalta] : 0;
    if (conn && sysCalib) {
      for(int i = 0; i < N; i++) {
        angulosParaExibir[i] = angPot[i];
      }
    }
    xSemaphoreGive(mutexEstado);

    if (!conn) {
      if (estadoLcd != 0) {
        tela("Aguardando", "conexao...");
        estadoLcd = 0;
      }
    } else if (!sysCalib) {
      char l1[17], l2[17];
      snprintf(l1, sizeof(l1), "Calibrando %d/%d", N - falta, N);
      snprintf(l2, sizeof(l2), "%s %3d>%3d", nomes[idFalta], potAtual, alvoAtual);
      tela(l1, l2);
      estadoLcd = 1;
    } else {
      char l1[17], l2[17];
      snprintf(l1, sizeof(l1), "B:%-3d O:%-3d C:%-3d", angulosParaExibir[0], angulosParaExibir[1], angulosParaExibir[2]);
      snprintf(l2, sizeof(l2), "P:%-3d G:%-3d A:%-3d", angulosParaExibir[3], angulosParaExibir[4], angulosParaExibir[5]);
      tela(l1, l2);
      estadoLcd = 2;
    }

    vTaskDelay(pdMS_TO_TICKS(150));  // Atualiza tela a cada 150ms
  }
}

void setup() {
  Serial.begin(115200);

  // Inicializa I2C e Display
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  tela("Controle braco", "Iniciando...");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  for (int i = 0; i < N; i++) {
    leituraFiltrada[i] = lerAngulo(i);
  }

  // Inicializa Wi-Fi e ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(CANAL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    tela("Erro ESP-NOW", "Reinicie");
    while (true) vTaskDelay(portMAX_DELAY);  // Trava o código aqui
  }

  esp_now_register_recv_cb(aoReceber);

  esp_now_peer_info_t par = {};
  memcpy(par.peer_addr, macBroadcast, 6);
  par.channel = CANAL;
  par.encrypt = false;
  esp_now_add_peer(&par);

  // Cria o Mutex para proteger as variáveis
  mutexEstado = xSemaphoreCreateMutex();

  // Inicia as Tarefas
  xTaskCreatePinnedToCore(TaskEnvio, "Envio", 4000, NULL, 2, NULL, 0);        // Core 0 (WiFi)
  xTaskCreatePinnedToCore(TaskSensores, "Sensores", 4000, NULL, 1, NULL, 1);  // Core 1
  xTaskCreatePinnedToCore(TaskLCD, "LCD", 4000, NULL, 1, NULL, 1);            // Core 1
}

void loop() {
  vTaskDelete(NULL);
}
