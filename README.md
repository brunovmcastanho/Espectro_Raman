ESP32 TCD1304 SPI Master – TCC2

Firmware desenvolvido como parte do Trabalho de Conclusão de Curso 2 (TCC2) de
Bruno Vinicius Machado Castanho
Universidade Tecnológica Federal do Paraná (UTFPR) – Câmpus Toledo

O projeto implementa um sistema de aquisição para um espectrômetro de baixo custo baseado no sensor linear TCD1304, utilizando um STM32F401RE para geração dos sinais de controle do CCD e um ESP32 como mestre da aquisição, interface gráfica (TFT) e servidor de rede (Wi-Fi em modo Access Point).

Visão Geral

O firmware do ESP32 é responsável por:

Configurar o Wi-Fi em modo Access Point (AP) e atuar como servidor TCP.

Enviar comandos de aquisição para o STM32F401RE via SPI (HSPI).

Receber os dados de espectro do STM32 e processá-los.

Exibir o espectro em um display TFT ST7796S.

Enviar os dados processados ao PC via TCP, para visualização/armazenamento.

Implementar interface de usuário com encoder rotativo (navegação, clique e long press).

Implementar dark spectrum (subtração de fundo).

Permitir ajustes de:

Período de SH (SH_PERIOD)

Período de ICG (ICG_PERIOD)

Limites de plotagem (X min/max, Y min/max)

Unidade do eixo X: pixels, nm ou cm⁻¹ (Raman shift)

Hardware Utilizado

ESP32

Interface SPI com STM32 (HSPI)

Controle do display TFT ST7796S

Gerenciamento do encoder rotativo

Modo Wi-Fi Access Point + servidor TCP

STM32F401RE

Geração dos sinais de controle do TCD1304 (ICG, SH, clock)

Leitura do sinal analógico do CCD (ADC + DMA)

Envio dos dados de pixels via SPI para o ESP32

Sensor CCD Linear TCD1304

Display TFT

Controlador: ST7796S

Resolução: 480x320 (orientação paisagem)

Encoder Rotativo com botão

Rotação: navegação em menus / alteração de valores

Clique curto: seleção/enter

Long press: função de salvar dark spectrum

Ligações Principais (ESP32)

Ajuste conforme seu hardware real; estes são os pinos usados no código atual.

Display TFT ST7796S

PIN_TFT_CS → GPIO 15

PIN_TFT_DC → GPIO 2

PIN_TFT_RST → -1 (reset interno)

SPI com STM32 (HSPI)

PIN_STM_SCLK → GPIO 14

PIN_STM_MISO → GPIO 12

PIN_STM_MOSI → GPIO 13

PIN_STM_SS → GPIO 5

Sinal DATA READY

DATA_READY_PIN → GPIO 4 (interrupção RISING)

Encoder rotativo

ENCODER_CLK_PIN → GPIO 26

ENCODER_DT_PIN → GPIO 27

ENCODER_SW_PIN → GPIO 25

Recursos Principais do Firmware
1. Modo Access Point e Servidor TCP

O ESP32 cria uma rede Wi-Fi própria:

const char* ap_ssid     = "ESP32-Spectrometer-AP";
const char* ap_password = "password123";
const int   server_port = 9999;


O PC conecta-se a:

IP: geralmente 192.168.4.1

Porta: 9999

A cada aquisição o ESP32 envia:

Marcador de início: AA BB CC DD

Vetor pixel_data (uint16_t, espectro corrigido)

2. Interface Gráfica (TFT)

Menus para navegação intuitiva

Exibição do espectro com grade e eixos

Unidade do eixo X configurável:

Pixel

nm

cm⁻¹ (Raman shift)

Status da rede exibido na tela

3. Máquina de Estados
enum ScreenState {
  STATE_MAIN_MENU,
  STATE_SETTINGS,
  STATE_SETTINGS_ACQ,
  STATE_SETTINGS_AXIS,
  STATE_ACQUISITION_CONT,
  STATE_ACQUISITION_SINGLE,
  STATE_ACQUISITION_DARK
};


Cada estado define o comportamento do sistema, menus e modos de aquisição.

Calibração Espectral

A conversão pixel → comprimento de onda é feita com dois pontos conhecidos:

const float CAL_LAMBDA_1 = 532.0;
const int   CAL_PIXEL_1  = 387;
const float CAL_LAMBDA_2 = 632.8;
const int   CAL_PIXEL_2  = 2416;


Coeficientes:

cal_slope_a     = (CAL_LAMBDA_2 - CAL_LAMBDA_1) / (CAL_PIXEL_2 - CAL_PIXEL_1);
cal_intercept_b = CAL_LAMBDA_1 - cal_slope_a * CAL_PIXEL_1;


Conversão para deslocamento Raman:

𝜈
~
=
(
1
𝜆
exc
−
1
𝜆
)
×
10
7
ν
~
=(
λ
exc
	​

1
	​

−
λ
1
	​

)×10
7
Protocolo SPI com STM32

O ESP32 envia:

Header ER

SH_PERIOD

ICG_PERIOD

Modo

NUM_INTEGRATIONS

E recebe um buffer completo contendo os pixels brutos do CCD.

Como Compilar e Gravar
Bibliotecas necessárias

Adafruit_GFX

Adafruit_ST7796S_kbv

AiEsp32RotaryEncoder

WiFi (nativa)

SPI (nativa)

Passos

Instalar suporte ao ESP32 na IDE Arduino ou PlatformIO.

Ajustar pinos caso necessário.

Compilar.

Enviar ao módulo ESP32.

Como Utilizar

Ligue o sistema.

Conecte o PC ao Wi-Fi gerado pelo ESP32.

Abra um cliente TCP e conecte ao IP mostrado no display.

Navegue pelos menus usando o encoder:

Aquisição contínua → streaming de dados

Aquisição única → captura sob demanda

Dark Spectrum → long press para salvar

Organização do Código

Principais funções:

drawMainMenuScreen() – interface do menu principal

drawGraph_GFX() – renderização do espectro

pixelToWavelength() – conversão pixel → nm

wavelengthToWavenumber() – conversão nm → cm⁻¹

buildCommand() – montagem do pacote SPI

processData_Raw() – conversão do buffer SPI para pixels

applyDarkSpectrumCorrection() – subtração do dark spectrum

sendDataViaWiFi() – envio via TCP

Licença
Copyright (c) 2025  
Autor: Bruno Vinicius Machado Castanho

Todos os direitos reservados.

Contato

Autor: Bruno Vinicius Machado Castanho

Instituição: UTFPR – Universidade Tecnológica Federal do Paraná
