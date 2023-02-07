/*

Nome ALUNO A- André Vieira 2181194
Nome ALUNO B- Daniel Ribeiro 2181358
IPLEIRIA - Instituto Politécnico de Leiria
ESTG - Escola Superior de Tecnologia e Gestão
LEAU- Licenciatura em Engenharia Automovel
SEEV - Sistemas Elétricos e Eletrónicos de Veículos

TP1: Pretende-se  neste  trabalho  prático  a  implementação  de controlador  que  permita  a correta gestão de temperaturas de um motor de combustão interna e a gestão das ventoinhas consoante a temperatura.

LINK: (link do código do vídeo do youtube).

*/
#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#define TFT_CS 15
#define TFT_DC 2

#define MODO_AUTOMATICO 0
#define MODO_MANUAL 1

#define ADC_RESOLUTION 8
#define VREF_PLUS  3.3
#define VREF_MINUS  0.0

#define ADC_SENSOR_IN 32
#define ADC_SENSOR_OUT 33

#define MULTIPLICADOR_ADC 0.47588
#define MULTIPLICADOR_PWM 3.875
#define MULTIPLICADOR_PERCENTAGEM 0.3922

#define PINO_LED_VERDE 13
#define PINO_LED_AMARELO 12
#define PINO_LED_VERMELHO 14

#define PINO_VENTOINHA_1 27
#define PINO_VENTOINHA_2 26

#define PINO_INTERRUPCAO 22

struct struct_temperaturas {
	float temperatura_in;
	float temperatura_out;
};

SemaphoreHandle_t xBinarySemaphore;
SemaphoreHandle_t xMutex_UART;
SemaphoreHandle_t xMutex_SPI;

QueueHandle_t xQueue_modo_arrefecimento = NULL;
QueueHandle_t xQueue_temperaturas = NULL;
QueueHandle_t xQueue_potencia_ventoinhas = NULL;

const char *pcTextFor_Processamento = "Processamento task is running ";
const char *pcTextFor_ADC = "ADC task is running ";
const char *pcTextFor_PWM = "PWM task is running ";
const char *pcTextFor_LDC = "LCD task is running ";
const char *pcTextFor_LED = "LED task is running ";
const char *pcTextFor_INTERRUPT = "Interrupt task is running ";

static void IRAM_ATTR vInterruptHandler_muda_estado_arrefecimento(void);

void vTask_processamento(void *pvParameters);
void vTask_ADC(void *pvParameters);
void vTask_PWM(void *pvParameters);
void vTask_LCD(void *pvParameters);
void vTask_LED(void *pvParameters);
void vTask_gestao_interrupcao(void *pvParameters);

void setup() {
	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

	Serial.begin(115200);

	pinMode(PINO_INTERRUPCAO, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(PINO_INTERRUPCAO),
			&vInterruptHandler_muda_estado_arrefecimento, FALLING);

	vSemaphoreCreateBinary(xBinarySemaphore);

	bool aux_verificacao = true;

	xMutex_UART = xSemaphoreCreateMutex();
	if (xMutex_UART == NULL) {
		aux_verificacao = false;
	}

	xMutex_SPI = xSemaphoreCreateMutex();
	if (xMutex_SPI == NULL) {
		aux_verificacao = false;
	}

	xQueue_modo_arrefecimento = xQueueCreate(1, sizeof(char));
	if (xQueue_modo_arrefecimento == NULL) {
		aux_verificacao = false;
	}

	xQueue_temperaturas = xQueueCreate(1, sizeof(struct struct_temperaturas));
	if (xQueue_temperaturas == NULL) {
		aux_verificacao = false;
	}

	xQueue_potencia_ventoinhas = xQueueCreate(1, sizeof(char));
	if (xQueue_potencia_ventoinhas == NULL) {
		aux_verificacao = false;
	}

	if (aux_verificacao == true) {

		xTaskCreatePinnedToCore(vTask_processamento, "PROCESSAMENTO", 1024,
				(void*) pcTextFor_Processamento, 5, NULL, 1);

		xTaskCreatePinnedToCore(vTask_ADC, "ADC", 1024, (void*) pcTextFor_ADC,
				4, NULL, 1);

		xTaskCreatePinnedToCore(vTask_PWM, "PWM", 1024, (void*) pcTextFor_PWM,
				4, NULL, 1);

		xTaskCreatePinnedToCore(vTask_LCD, "LCD", 4096, (void*) pcTextFor_LDC,
				2, NULL, 1);

		xTaskCreatePinnedToCore(vTask_LED, "LED", 1024, (void*) pcTextFor_LED,
				3, NULL, 1);

		xTaskCreatePinnedToCore(vTask_gestao_interrupcao, "INTERRUPCAO", 1024,
				(void*) pcTextFor_INTERRUPT, 6, NULL, 1);
	}

	interrupts();

}

void loop() {
	vTaskDelete(NULL);
}

void vTask_LCD(void *pvParameters) {
	char *pcTaskName;
	TickType_t xLastWakeTime;
	pcTaskName = (char*) pvParameters;
	xLastWakeTime = xTaskGetTickCount();

	Adafruit_ILI9341 tft = Adafruit_ILI9341(15, 2, 23, 18, 4, 19);	//lcd-TFT

	tft.begin();
	tft.fillScreen(ILI9341_BLACK);
	tft.setRotation(2);

	tft.setTextSize(2);

	float temperatura_in_anterior = 0;
	float temperatura_out_anterior = 0;
	uint8_t modo_arrefecimento_ant = NULL;
	uint8_t valor_potencia_ventoinhas_ant = NULL;

	struct_temperaturas temperaturas_atuais = { 0, 0 };
	uint8_t modo_arrefecimento = NULL;
	uint8_t valor_potencia_ventoinhas = NULL;
	xSemaphoreTake(xMutex_SPI, portMAX_DELAY);
	{
		tft.fillRoundRect(5, 20, 235, 75, 10, ILI9341_ORANGE);
		tft.setTextColor(ILI9341_BLACK);
		tft.setCursor(10, 27);
		tft.print("Potencia:     %");

		tft.fillRoundRect(5, 100, 235, 75, 10, ILI9341_ORANGE);
		tft.setTextColor(ILI9341_BLACK);
		tft.setCursor(10, 107);
		tft.print("temp. in:");
		tft.setCursor(10, 127);
		tft.print("temp. out:");

		tft.fillRoundRect(5, 180, 235, 75, 10, ILI9341_ORANGE);
		tft.setTextColor(ILI9341_BLACK);
		tft.setCursor(10, 187);
		tft.print("Modo:");
	}
	xSemaphoreGive(xMutex_SPI);
	for (;;) {
		xSemaphoreTake(xMutex_UART, portMAX_DELAY);
		{
			Serial.println(pcTaskName);
		}
		xSemaphoreGive(xMutex_UART);

		xSemaphoreTake(xMutex_SPI, portMAX_DELAY);
		{
			xQueuePeek(xQueue_potencia_ventoinhas, &valor_potencia_ventoinhas,
					0);
			xQueuePeek(xQueue_modo_arrefecimento, &modo_arrefecimento, 0);
			xQueuePeek(xQueue_temperaturas, &temperaturas_atuais, 0);

			if (temperatura_in_anterior != temperaturas_atuais.temperatura_in) {
				tft.setTextColor(ILI9341_BLACK, ILI9341_ORANGE);
				tft.setCursor(125, 107);
				tft.print(temperaturas_atuais.temperatura_in);
				temperatura_in_anterior = temperaturas_atuais.temperatura_in;

			}
			if (temperatura_out_anterior
					!= temperaturas_atuais.temperatura_out) {
				tft.setTextColor(ILI9341_BLACK, ILI9341_ORANGE);
				tft.setCursor(130, 127);
				tft.print(temperaturas_atuais.temperatura_out);
				temperatura_out_anterior = temperaturas_atuais.temperatura_out;
			}
			if (modo_arrefecimento_ant != modo_arrefecimento) {
				tft.setTextColor(ILI9341_BLACK, ILI9341_ORANGE);

				if (modo_arrefecimento == MODO_AUTOMATICO) {
					tft.setCursor(100, 187);
					tft.print("           ");
					tft.setCursor(100, 187);
					tft.print("Automatico");
				} else if (modo_arrefecimento == MODO_MANUAL) {
					tft.setCursor(100, 187);
					tft.print("           ");
					tft.setCursor(100, 187);
					tft.print("Manual");
				}
				modo_arrefecimento_ant = modo_arrefecimento;
			}
			if (valor_potencia_ventoinhas_ant != valor_potencia_ventoinhas) {
				tft.setTextColor(ILI9341_BLACK, ILI9341_ORANGE);
				tft.setCursor(130, 27);
				tft.print(
						valor_potencia_ventoinhas * MULTIPLICADOR_PERCENTAGEM);
				tft.setCursor(170, 27);
				valor_potencia_ventoinhas_ant = valor_potencia_ventoinhas;
			}
		}
		xSemaphoreGive(xMutex_SPI);

		vTaskDelayUntil(&xLastWakeTime, (400 / portTICK_PERIOD_MS));
	}
}

void vTask_processamento(void *pvParameters) {
	char *pcTaskName;
	TickType_t xLastWakeTime;
	pcTaskName = (char*) pvParameters;
	xLastWakeTime = xTaskGetTickCount();

	struct_temperaturas temperaturas_atuais = { 0, 0 };
	uint8_t potencia_ventoinhas = NULL;
	uint8_t modo_arrefecimento = NULL;

	for (;;) {
		xSemaphoreTake(xMutex_UART, portMAX_DELAY);
		{
			Serial.println(pcTaskName);
		}
		xSemaphoreGive(xMutex_UART);

		xQueuePeek(xQueue_modo_arrefecimento, &modo_arrefecimento, 0);

		if (modo_arrefecimento == MODO_MANUAL) {
			potencia_ventoinhas = 255;
		}

		if (modo_arrefecimento == MODO_AUTOMATICO) {
			xQueuePeek(xQueue_temperaturas, &temperaturas_atuais, 0);

			potencia_ventoinhas = temperaturas_atuais.temperatura_in
					* MULTIPLICADOR_PWM;
		}

		xQueueOverwrite(xQueue_potencia_ventoinhas, &potencia_ventoinhas);
		vTaskDelayUntil(&xLastWakeTime, (100 / portTICK_PERIOD_MS));
	}
}

void vTask_gestao_interrupcao(void *pvParameters) {
	char *pcTaskName;
	TickType_t xLastWakeTime;
	pcTaskName = (char*) pvParameters;
	xLastWakeTime = xTaskGetTickCount();

	//xSemaphoreTake(xBinarySemaphore, 0);
	uint8_t tipo_arrefecimento = NULL;

	for (;;) {

		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
		{
			xSemaphoreTake(xMutex_UART, portMAX_DELAY);
			{
				Serial.println(pcTaskName);
			}
			xSemaphoreGive(xMutex_UART);

			xQueuePeek(xQueue_modo_arrefecimento, &tipo_arrefecimento, 0);

			if (tipo_arrefecimento == MODO_AUTOMATICO) {
				tipo_arrefecimento = MODO_MANUAL;

			} else if (tipo_arrefecimento == MODO_MANUAL) {
				tipo_arrefecimento = MODO_AUTOMATICO;
			}
			xQueueOverwrite(xQueue_modo_arrefecimento, &tipo_arrefecimento);

			vTaskDelayUntil(&xLastWakeTime, (250 / portTICK_PERIOD_MS));
		}
	}
}

void vTask_ADC(void *pvParameters) {
	char *pcTaskName;
	TickType_t xLastWakeTime;
	pcTaskName = (char*) pvParameters;
	xLastWakeTime = xTaskGetTickCount();

	analogReadResolution(ADC_RESOLUTION);

	uint8_t adc_temperatura_in = 0;
	uint8_t adc_temperatura_out = 0;

	struct_temperaturas temperaturas = { 0, 0 };

	for (;;) {

		adc_temperatura_in = analogRead(ADC_SENSOR_IN);
		adc_temperatura_out = analogRead(ADC_SENSOR_OUT);

		temperaturas.temperatura_in = adc_temperatura_in * MULTIPLICADOR_ADC;
		temperaturas.temperatura_out = adc_temperatura_out * MULTIPLICADOR_ADC;

		xQueueOverwrite(xQueue_temperaturas, &temperaturas);

		xSemaphoreTake(xMutex_UART, portMAX_DELAY);
		{
			Serial.println(pcTaskName);
			Serial.print("valor de temperatura in: ");
			Serial.println(temperaturas.temperatura_in);
			Serial.print("valor de temperatura out: ");
			Serial.println(temperaturas.temperatura_out);
		}
		xSemaphoreGive(xMutex_UART);

		vTaskDelayUntil(&xLastWakeTime, (200 / portTICK_PERIOD_MS));
	}
}

void vTask_PWM(void *pvParameters) {
	char *pcTaskName;
	TickType_t xLastWakeTime;
	pcTaskName = (char*) pvParameters;
	xLastWakeTime = xTaskGetTickCount();

	int freq = 5000;
	int resolution = 8;
	int ledChannel_ventoinha_1 = 8;
	int ledChannel_ventoinha_2 = 9;

	ledcSetup(ledChannel_ventoinha_1, freq, resolution);
	ledcAttachPin(PINO_VENTOINHA_1, ledChannel_ventoinha_1);

	ledcSetup(ledChannel_ventoinha_2, freq, resolution);
	ledcAttachPin(PINO_VENTOINHA_2, ledChannel_ventoinha_2);

	uint8_t valor_potencia_ventoinhas = NULL;

	for (;;) {

		xQueuePeek(xQueue_potencia_ventoinhas, &valor_potencia_ventoinhas, 0);

		ledcWrite(ledChannel_ventoinha_1, valor_potencia_ventoinhas);
		ledcWrite(ledChannel_ventoinha_2, valor_potencia_ventoinhas);

		xSemaphoreTake(xMutex_UART, portMAX_DELAY);
		{
			Serial.println(pcTaskName);
			Serial.print("valor potencia de ventoinhas: ");
			Serial.println(valor_potencia_ventoinhas);

		}
		xSemaphoreGive(xMutex_UART);

		vTaskDelayUntil(&xLastWakeTime, (200 / portTICK_PERIOD_MS));
	}
}

void vTask_LED(void *pvParameters) {
	char *pcTaskName;
	TickType_t xLastWakeTime;
	pcTaskName = (char*) pvParameters;
	xLastWakeTime = xTaskGetTickCount();

	int freq = 5000;
	int resolution = 8;
	int ledChannel_verde = 5;
	int ledChannel_amarelo = 6;
	int ledChannel_vermelho = 7;

	ledcSetup(ledChannel_verde, freq, resolution);
	ledcAttachPin(PINO_LED_VERDE, ledChannel_verde);

	ledcSetup(ledChannel_amarelo, freq, resolution);
	ledcAttachPin(PINO_LED_AMARELO, ledChannel_amarelo);

	ledcSetup(ledChannel_vermelho, freq, resolution);
	ledcAttachPin(PINO_LED_VERMELHO, ledChannel_vermelho);

	struct_temperaturas temperaturas_atuais = { 0, 0 };

	for (;;) {
		xSemaphoreTake(xMutex_UART, portMAX_DELAY);
		{
			Serial.println(pcTaskName);
		}
		xSemaphoreGive(xMutex_UART);

		xQueuePeek(xQueue_temperaturas, &temperaturas_atuais, 0);

		if (temperaturas_atuais.temperatura_in <= 80) {
			ledcWrite(ledChannel_verde, 0);
			ledcWrite(ledChannel_amarelo, 255);
			ledcWrite(ledChannel_vermelho, 0);
		}

		if (temperaturas_atuais.temperatura_in > 80
				&& temperaturas_atuais.temperatura_in <= 100) {
			ledcWrite(ledChannel_verde, 255);
			ledcWrite(ledChannel_amarelo, 0);
			ledcWrite(ledChannel_vermelho, 0);
		}

		if (temperaturas_atuais.temperatura_in > 100) {
			ledcWrite(ledChannel_verde, 0);
			ledcWrite(ledChannel_amarelo, 0);
			ledcWrite(ledChannel_vermelho, 255);
		}

		vTaskDelayUntil(&xLastWakeTime, (300 / portTICK_PERIOD_MS));
	}
}

static void IRAM_ATTR vInterruptHandler_muda_estado_arrefecimento(void) {
	static portBASE_TYPE xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR(xBinarySemaphore,
			(signed portBASE_TYPE*)&xHigherPriorityTaskWoken);

	Serial.println("Estou na interrupt");

	if (xHigherPriorityTaskWoken == pdTRUE) {
		portYIELD_FROM_ISR();

	}
}
