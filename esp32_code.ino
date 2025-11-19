/*
 * Autor: Bruno Vinicius Machado Castanho
 * Instituição: Universidade Tecnológica Federal do Paraná (UTFPR) - Campus Toledo
 * Projeto: TCC2 - ESP32 TCD1304 SPI Master
 *
 * Descrição: Firmware do ESP32 para aquisição de dados do sensor linear TCD1304
 * via STM32, exibição em display TFT e transmissão dos espectros via Wi-Fi
 * operando como ponto de acesso (Access Point) e servidor TCP.
 */

#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_ST7796S_kbv.h"
#include <AiEsp32RotaryEncoder.h>
#include <WiFi.h> // Biblioteca Wi-Fi para modo Access Point e servidor TCP

// -----------------------------------------------------------------------------
// Configurações da rede Wi-Fi em modo Access Point
// -----------------------------------------------------------------------------
const char* ap_ssid = "ESP32-Spectrometer-AP";
const char* ap_password = "password123"; // Senha deve ter no mínimo 8 caracteres
const int server_port = 9999;

// Objetos relacionados ao servidor Wi-Fi
WiFiServer server(server_port);
WiFiClient client; // Armazena o cliente (PC) conectado

// -----------------------------------------------------------------------------
// Máquina de estados da interface de usuário
// -----------------------------------------------------------------------------
enum ScreenState {
  STATE_MAIN_MENU,
  STATE_SETTINGS, 
  STATE_SETTINGS_ACQ,    
  STATE_SETTINGS_AXIS,   
  STATE_ACQUISITION_CONT,  
  STATE_ACQUISITION_SINGLE,
  STATE_ACQUISITION_DARK 
};
ScreenState currentState = STATE_MAIN_MENU;
bool redrawScreen = true; 

// -----------------------------------------------------------------------------
// Protocolo de comunicação com o CCD (pacotes e resolução do ADC)
// -----------------------------------------------------------------------------
const int PACKET_SIZE = 7388;
const int PIXEL_COUNT = 3694;
#define ADC_MAX_VALUE 4095

// -----------------------------------------------------------------------------
// Parâmetros globais de aquisição
// -----------------------------------------------------------------------------
uint32_t SH_PERIOD = 5000000;      
uint32_t ICG_PERIOD = 5000000;
const uint16_t NUM_INTEGRATIONS = 1;

// -----------------------------------------------------------------------------
// Pinos de conexão de hardware
// -----------------------------------------------------------------------------
const int PIN_TFT_CS    = 15;
const int PIN_TFT_DC    = 2;
const int PIN_TFT_RST   = -1;

const int PIN_STM_SCLK = 14; 
const int PIN_STM_MISO = 12; 
const int PIN_STM_MOSI = 13; 
const int PIN_STM_SS    = 5;   

const int DATA_READY_PIN = 4; 

#define ENCODER_CLK_PIN 26
#define ENCODER_DT_PIN  27
#define ENCODER_SW_PIN  25
#define ROTARY_ENCODER_STEPS 4  

// -----------------------------------------------------------------------------
// Configurações da interface SPI com o STM32
// -----------------------------------------------------------------------------
const uint32_t STM32_SPI_CLOCK = 16000000;  
SPISettings stm32_spiSettings(STM32_SPI_CLOCK, MSBFIRST, SPI_MODE0);

// -----------------------------------------------------------------------------
// Configurações de desenho do gráfico no display TFT
// -----------------------------------------------------------------------------
#define NUM_DIVISOES_X 8  
#define NUM_DIVISOES_Y 4

#define COR_FUNDO       ST7796S_BLACK
#define COR_EIXOS       ST7796S_WHITE
#define COR_GRID        ST7796S_DARKGREY
#define COR_GRAFICO     ST7796S_GREEN
#define COR_TEXTO       ST7796S_WHITE
#define COR_DESTAQUE_BG ST7796S_GREEN
#define COR_DESTAQUE_FG ST7796S_BLACK

#define MARGEM_Y      20
#define MARGEM_X      30 

// -----------------------------------------------------------------------------
// Parâmetros de calibração espectral (2 pontos) e unidade de eixo X
// -----------------------------------------------------------------------------
const float CAL_LAMBDA_1 = 532.0;
const int   CAL_PIXEL_1  = 387;
const float CAL_LAMBDA_2 = 632.8;
const int   CAL_PIXEL_2  = 2416;
const float EXC_LAMBDA = 532.0;

float cal_slope_a = 0.0;
float cal_intercept_b = 0.0;

enum XAxisUnit { UNIT_PIXEL, UNIT_NM, UNIT_CM_1 };
XAxisUnit currentXUnit = UNIT_PIXEL;

// -----------------------------------------------------------------------------
// Limites de plotagem do gráfico (em pixels e contagens ADC)
// -----------------------------------------------------------------------------
int plot_pixel_min = 0;
int plot_pixel_max = PIXEL_COUNT - 1;
int plot_adc_min = 0;
int plot_adc_max = ADC_MAX_VALUE;

// -----------------------------------------------------------------------------
// Buffers globais de comunicação e dados de espectro
// -----------------------------------------------------------------------------
uint8_t tx_buffer[PACKET_SIZE];
uint8_t rx_buffer[PACKET_SIZE];

uint16_t pixel_data[PIXEL_COUNT];        // Dados processados (após correção de dark)
uint16_t raw_pixel_data[PIXEL_COUNT];    // Dados brutos do CCD
uint16_t dark_spectrum_data[PIXEL_COUNT]; // Espectro de referência "dark"

bool darkSpectrumSaved = false;           

volatile bool dataReady = false;
volatile bool waitingForSingleScan = false; 

// -----------------------------------------------------------------------------
// Objetos de hardware (SPI, display TFT e encoder rotativo)
// -----------------------------------------------------------------------------
SPIClass * spi_stm32 = new SPIClass(HSPI);
Adafruit_ST7796S_kbv tft = Adafruit_ST7796S_kbv(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ENCODER_CLK_PIN, ENCODER_DT_PIN, ENCODER_SW_PIN, -1, ROTARY_ENCODER_STEPS);

// -----------------------------------------------------------------------------
// Variáveis de estado da interface (menus, seleção e edição)
// -----------------------------------------------------------------------------
int mainMenuSelection = 0; 
const int MAIN_MENU_MAX = 3; 

int settingsSelection = 0; 
const int SETTINGS_MAX = 2;

int settingsAcqSelection = 0; 
const int SETTINGS_ACQ_MAX = 2;

int settingsAxisSelection = 0;
const int SETTINGS_AXIS_MAX = 5;

bool settingsEditMode = false; 
int16_t lastEncoderValue = 0;

const long LONG_PRESS_DURATION = 3000; // 3 segundos para detecção de long press

// -----------------------------------------------------------------------------
// Marcador de início de pacote para transmissão via Wi-Fi
// -----------------------------------------------------------------------------
const uint8_t PACKET_START_MARKER[4] = {0xAA, 0xBB, 0xCC, 0xDD};

// ============================================================================
// FUNÇÕES DE DESENHO DA INTERFACE (GFX)
// ============================================================================

/**
 * @brief Desenha um item de menu com destaque opcional
 */
void drawMenuItem(int y, const char* label, bool highlighted) {
  int w = 200;
  int h = 40;
  int x = (tft.width() - w) / 2;

  tft.setTextSize(2);
  if (highlighted) {
    tft.fillRect(x, y - h / 2, w, h, COR_DESTAQUE_BG);
    tft.setTextColor(COR_DESTAQUE_FG);
    tft.setCursor(x + 10, y - 8);
    tft.print(label);
  } else {
    tft.drawRect(x, y - h / 2, w, h, COR_EIXOS);
    tft.setTextColor(COR_EIXOS);
    tft.setCursor(x + 10, y - 8);
    tft.print(label);
  }
}

/**
 * @brief Desenha a tela do menu principal, incluindo status da rede AP
 */
void drawMainMenuScreen() {
  tft.fillScreen(COR_FUNDO);
  tft.setTextColor(COR_TEXTO);
  tft.setTextSize(3);
  tft.setCursor((tft.width() - 250) / 2, 40);
  tft.print("ESPECTROMETRO");

  drawMenuItem(120, "Aquis. Continua", (mainMenuSelection == 0));
  drawMenuItem(180, "Aquis. Unica",    (mainMenuSelection == 1));
  drawMenuItem(240, "Dark Spectrum",   (mainMenuSelection == 2));
  drawMenuItem(300, "Configuracoes",   (mainMenuSelection == 3));

  // Informações de rede e status do cliente na parte inferior da tela
  tft.setTextSize(1);
  tft.setCursor(5, tft.height() - 20);
  tft.setTextColor(ST7796S_CYAN);
  tft.print("REDE (AP): ");
  tft.print(ap_ssid);

  tft.setCursor(5, tft.height() - 10);
  tft.print("IP: ");
  tft.print(WiFi.softAPIP());

  tft.setCursor(150, tft.height() - 10);
  if (client && client.connected()) {
    tft.setTextColor(ST7796S_GREEN);
    tft.print("CLIENTE: CONECTADO");
  } else {
    tft.setTextColor(ST7796S_RED);
    tft.print("CLIENTE: --");
  }
}

/**
 * @brief Desenha item de configuração com valor numérico
 */
void drawSettingsItem(int y, const char* label, uint32_t value, bool selected, bool editing) {
  int w = tft.width() - 40;
  int h = 50;
  int x = 20;

  if (selected || editing) {
    tft.drawRect(x, y - h / 2, w, h, COR_DESTAQUE_BG);
  } else {
    tft.drawRect(x, y - h / 2, w, h, COR_GRID);
  }

  tft.setTextColor(COR_TEXTO);
  tft.setTextSize(2);
  tft.setCursor(x + 10, y - 15);
  tft.print(label);

  if (editing) {
    tft.fillRect(x + 200, y - 15, 150, 20, COR_DESTAQUE_BG);
    tft.setTextColor(COR_DESTAQUE_FG);
  } else {
    tft.setTextColor(COR_TEXTO);
  }

  tft.setCursor(x + 200, y - 15);
  tft.print(value);
}

/**
 * @brief Desenha item de configuração com valor textual/float
 */
void drawSettingsStringItem(int y, const char* label, const char* value, bool selected, bool editing) {
  int w = tft.width() - 40;
  int h = 50;
  int x = 20;

  if (selected || editing) {
    tft.drawRect(x, y - h / 2, w, h, COR_DESTAQUE_BG);
  } else {
    tft.drawRect(x, y - h / 2, w, h, COR_GRID);
  }

  tft.setTextColor(COR_TEXTO);
  tft.setTextSize(2);
  tft.setCursor(x + 10, y - 15);
  tft.print(label);

  if (editing) {
    tft.fillRect(x + 200, y - 15, 150, 20, COR_DESTAQUE_BG);
    tft.setTextColor(COR_DESTAQUE_FG);
  } else {
    tft.setTextColor(COR_TEXTO);
  }

  tft.setCursor(x + 200, y - 15);
  tft.print(value);
}

/**
 * @brief Desenha a tela principal de configurações
 */
void drawSettingsScreen() {
  tft.fillScreen(COR_FUNDO);
  tft.setTextColor(COR_TEXTO);
  tft.setTextSize(3);
  tft.setCursor(20, 40);
  tft.print("Configuracoes");

  drawMenuItem(120, "Config. Aquisicao", (settingsSelection == 0));
  drawMenuItem(180, "Config. Eixos",     (settingsSelection == 1));
  drawMenuItem(240, "Voltar",            (settingsSelection == 2));
}

/**
 * @brief Desenha o sub-menu de configurações de aquisição
 */
void drawSettingsAcqScreen() {
  tft.fillScreen(COR_FUNDO);
  tft.setTextColor(COR_TEXTO);
  tft.setTextSize(3);
  tft.setCursor(20, 20);
  tft.print("Config. Aquisicao");

  bool selected, editing;

  // Item 0: SH Period
  selected = (settingsAcqSelection == 0);
  editing = (selected && settingsEditMode);
  drawSettingsItem(100, "SH Period:", SH_PERIOD, selected, editing);

  // Item 1: ICG Period
  selected = (settingsAcqSelection == 1);
  editing = (selected && settingsEditMode);
  drawSettingsItem(160, "ICG Period:", ICG_PERIOD, selected, editing);

  // Item 2: Voltar
  drawMenuItem(260, "Voltar", (settingsAcqSelection == 2));
}

/**
 * @brief Desenha o sub-menu de configurações dos eixos de plotagem
 */
void drawSettingsAxisScreen() {
  tft.fillScreen(COR_FUNDO);
  tft.setTextColor(COR_TEXTO);
  tft.setTextSize(3);
  tft.setCursor(20, 20);
  tft.print("Config. Eixos");

  char val_buf[20];
  bool selected, editing;

  // Item 0: Unidade do eixo X
  selected = (settingsAxisSelection == 0);
  editing = (selected && settingsEditMode);
  const char* unit_label;
  if (currentXUnit == UNIT_NM) unit_label = "nm";
  else if (currentXUnit == UNIT_CM_1) unit_label = "cm^-1";
  else unit_label = "Pixel";
  drawSettingsStringItem(80, "Eixo X:", unit_label, selected, editing);

  // Item 1: X Min
  selected = (settingsAxisSelection == 1);
  editing = (selected && settingsEditMode);
  dtostrf(getXUnitValue(plot_pixel_min), 6, (currentXUnit == UNIT_PIXEL ? 0 : 1), val_buf);
  drawSettingsStringItem(130, "X Min:", val_buf, selected, editing);

  // Item 2: X Max
  selected = (settingsAxisSelection == 2);
  editing = (selected && settingsEditMode);
  dtostrf(getXUnitValue(plot_pixel_max), 6, (currentXUnit == UNIT_PIXEL ? 0 : 1), val_buf);
  drawSettingsStringItem(180, "X Max:", val_buf, selected, editing);

  // Item 3: Y Min
  selected = (settingsAxisSelection == 3);
  editing = (selected && settingsEditMode);
  snprintf(val_buf, 20, "%d", plot_adc_min);
  drawSettingsStringItem(230, "Y Min:", val_buf, selected, editing);

  // Item 4: Y Max
  selected = (settingsAxisSelection == 4);
  editing = (selected && settingsEditMode);
  snprintf(val_buf, 20, "%d", plot_adc_max);
  drawSettingsStringItem(280, "Y Max:", val_buf, selected, editing);

  // Item 5: Voltar
  drawMenuItem(350, "Voltar", (settingsAxisSelection == 5));
}

/**
 * @brief Desenha o gráfico principal (eixos, grade e dados)
 */
void drawGraph_GFX(ScreenState mode) {
  tft.fillScreen(COR_FUNDO);

  int w = tft.width(), h = tft.height();
  int plot_x0 = MARGEM_X, plot_y0 = MARGEM_Y;
  int plot_x1 = w - MARGEM_X, plot_y1 = h - MARGEM_Y;
  int plot_w = plot_x1 - plot_x0, plot_h = plot_y1 - plot_y0;

  char label_buf[10];

  tft.setTextColor(COR_TEXTO);
  tft.setTextSize(1);

  // Grade e rótulos do eixo X
  for (int i = 0; i <= NUM_DIVISOES_X; i++) {
    int x = plot_x0 + (i * plot_w) / NUM_DIVISOES_X;
    tft.drawFastVLine(x, plot_y0, plot_h, COR_GRID);

    int pixel_val = map(i, 0, NUM_DIVISOES_X, plot_pixel_min, plot_pixel_max);
    float display_val = getXUnitValue(pixel_val);
    dtostrf(display_val, 5, (currentXUnit == UNIT_PIXEL ? 0 : 1), label_buf);

    tft.setCursor(x - 10, plot_y1 + 5); 
    tft.print(label_buf);
  }

  // Grade e rótulos do eixo Y
  for (int i = 0; i <= NUM_DIVISOES_Y; i++) {
    int y = plot_y0 + (i * plot_h) / NUM_DIVISOES_Y;
    tft.drawFastHLine(plot_x0, y, plot_w, COR_GRID);

    int adc_label = map(i, 0, NUM_DIVISOES_Y, plot_adc_max, plot_adc_min); 
    tft.setCursor(5, y - 4); 
    tft.print(adc_label);
  }

  // Moldura dos eixos
  tft.drawRect(plot_x0, plot_y0, plot_w, plot_h, COR_EIXOS);

  // Desenho do espectro (pixel_data)
  int y_adc_prev = constrain(pixel_data[plot_pixel_min], plot_adc_min, plot_adc_max);
  int prev_y = map(y_adc_prev, plot_adc_min, plot_adc_max, plot_y1, plot_y0);
  int prev_x = plot_x0;

  for (int x_plot = 1; x_plot < plot_w; x_plot++) {
    int i = map(x_plot, 0, plot_w - 1, plot_pixel_min, plot_pixel_max);
    int y_adc = constrain(pixel_data[i], plot_adc_min, plot_adc_max);
    int y = map(y_adc, plot_adc_min, plot_adc_max, plot_y1, plot_y0);
    int x = plot_x0 + x_plot;

    tft.drawLine(prev_x, prev_y, x, y, COR_GRAFICO);
    prev_x = x;
    prev_y = y;
  }

  // Mensagens de ajuda contextual de acordo com o modo de aquisição
  tft.setTextColor(COR_TEXTO);
  tft.setTextSize(1);
  int cursorX = tft.width() - 130; 

  if (mode == STATE_ACQUISITION_CONT) {
    tft.setCursor(cursorX, 10);
    tft.print("Clique para Sair");
  } else if (mode == STATE_ACQUISITION_SINGLE) {
    tft.setCursor(cursorX, 10);
    tft.print("Clique: Novo Scan");
    tft.setCursor(cursorX, 20); 
    tft.print("Gire: Sair");
  } else if (mode == STATE_ACQUISITION_DARK) {
    tft.setCursor(cursorX, 10);
    tft.print("Hold (3s): Salvar");
    tft.setCursor(cursorX, 20); 
    tft.print("Clique/Gire: Sair");
  }
}

// ============================================================================
// FUNÇÕES DE CONVERSÃO E LÓGICA
// ============================================================================

/**
 * @brief Converte posição de pixel em comprimento de onda (nm)
 */
float pixelToWavelength(int pixel) {
  return (cal_slope_a * (float)pixel) + cal_intercept_b;
}

/**
 * @brief Converte comprimento de onda (nm) em deslocamento Raman (cm^-1)
 */
float wavelengthToWavenumber(float lambda_nm) {
  if (lambda_nm <= EXC_LAMBDA) {
    return 0.0;
  }
  return (1.0 / EXC_LAMBDA - 1.0 / lambda_nm) * 10000000.0;
}

/**
 * @brief Retorna o valor do eixo X na unidade atualmente selecionada
 */
float getXUnitValue(int pixel) {
  switch (currentXUnit) {
    case UNIT_NM:
      return pixelToWavelength(pixel);
    case UNIT_CM_1:
      return wavelengthToWavenumber(pixelToWavelength(pixel));
    case UNIT_PIXEL:
    default:
      return (float)pixel;
  }
}

/**
 * @brief Altera o estado da máquina de estados da UI e faz configurações de entrada
 */
void changeState(ScreenState newState) {
  currentState = newState;
  redrawScreen = true;

  if (currentState == STATE_ACQUISITION_CONT) {
    Serial.println("Entrando no modo de Aquisicao (Continua)...");
    buildCommand();
    requestFirstScan(); 

  } else if (currentState == STATE_ACQUISITION_SINGLE) {
    Serial.println("Entrando no modo de Aquisicao (Unica)...");
    buildCommand();
    requestFirstScan(); 
    waitingForSingleScan = true; 

    tft.fillScreen(COR_FUNDO);
    tft.setTextColor(COR_TEXTO);
    tft.setTextSize(2);
    tft.setCursor(20, 150);
    tft.print("Aguardando 1a aquisicao...");

  } else if (currentState == STATE_ACQUISITION_DARK) {
    Serial.println("Entrando no modo Dark Spectrum...");
    buildCommand();
    requestFirstScan(); 
    waitingForSingleScan = true;

    tft.fillScreen(COR_FUNDO);
    tft.setTextColor(COR_TEXTO);
    tft.setTextSize(2);
    tft.setCursor(20, 150);
    tft.print("Aguardando aquisicao (Dark)...");

  } else if (currentState == STATE_SETTINGS) {
    settingsEditMode = false; 
  } else if (currentState == STATE_SETTINGS_ACQ) {
    settingsEditMode = false;
  } else if (currentState == STATE_SETTINGS_AXIS) {
    settingsEditMode = false;
  }
}

/**
 * @brief Trata entradas do encoder e botão no menu principal
 */
void handleMainMenuInput(int rotation, bool clicked) {
  if (rotation != 0) {
    mainMenuSelection += rotation;
    if (mainMenuSelection > MAIN_MENU_MAX) mainMenuSelection = 0;
    if (mainMenuSelection < 0) mainMenuSelection = MAIN_MENU_MAX;
    redrawScreen = true;
  }

  if (clicked) {
    if (mainMenuSelection == 0) {
      changeState(STATE_ACQUISITION_CONT);
    } else if (mainMenuSelection == 1) {
      changeState(STATE_ACQUISITION_SINGLE);
    } else if (mainMenuSelection == 2) {
      changeState(STATE_ACQUISITION_DARK);
    } else if (mainMenuSelection == 3) {
      changeState(STATE_SETTINGS);
    }
  }
}

/**
 * @brief Trata entradas do encoder e botão no menu de configurações principal
 */
void handleSettingsInput(int rotation, bool clicked) {
  if (rotation != 0) {
    settingsSelection += rotation;
    if (settingsSelection > SETTINGS_MAX) settingsSelection = 0;
    if (settingsSelection < 0) settingsSelection = SETTINGS_MAX;
    redrawScreen = true;
  }

  if (clicked) {
    if (settingsSelection == 0) {
      changeState(STATE_SETTINGS_ACQ);
    } else if (settingsSelection == 1) {
      changeState(STATE_SETTINGS_AXIS);
    } else if (settingsSelection == 2) {
      changeState(STATE_MAIN_MENU);
    }
    redrawScreen = true;
  }
}

/**
 * @brief Trata entradas no sub-menu de configurações de aquisição
 */
void handleSettingsAcqInput(int rotation, bool clicked) {
  if (settingsEditMode) {
    // Modo edição: altera valores numéricos
    if (rotation != 0) { 
      int step_time = (rotation > 0) ? 100 : -100;
      
      switch (settingsAcqSelection) {
        case 0: // SH Period
          SH_PERIOD += step_time;
          if (SH_PERIOD < 40) SH_PERIOD = 40;
          break;
        case 1: // ICG Period
          ICG_PERIOD += step_time * 10;
          if (ICG_PERIOD < 32000) ICG_PERIOD = 32000;
          break;
      }
      redrawScreen = true;
    }

    if (clicked) {
      settingsEditMode = false; 
      redrawScreen = true;
    }

  } else {
    // Modo navegação entre itens
    if (rotation != 0) {
      settingsAcqSelection += rotation;
      if (settingsAcqSelection > SETTINGS_ACQ_MAX) settingsAcqSelection = 0;
      if (settingsAcqSelection < 0) settingsAcqSelection = SETTINGS_ACQ_MAX;
      redrawScreen = true;
    }

    if (clicked) {
      switch (settingsAcqSelection) {
        case 0: // SH
        case 1: // ICG
          settingsEditMode = true; 
          break;
        case 2: // Voltar
          changeState(STATE_SETTINGS);
          break;
      }
      redrawScreen = true;
    }
  }
}

/**
 * @brief Trata entradas no sub-menu de configurações de eixos
 */
void handleSettingsAxisInput(int rotation, bool clicked) {
  if (settingsEditMode) {
    // Modo edição: ajusta limites de eixo em pixels e ADC
    if (rotation != 0) {
      int step_px = (rotation > 0) ? 10 : -10; 
      int step_adc = (rotation > 0) ? 50 : -50; 

      switch (settingsAxisSelection) {
        case 1: // X Min
          plot_pixel_min += step_px;
          plot_pixel_min = constrain(plot_pixel_min, 0, plot_pixel_max - 10); 
          break;
        case 2: // X Max
          plot_pixel_max += step_px;
          plot_pixel_max = constrain(plot_pixel_max, plot_pixel_min + 10, PIXEL_COUNT - 1);
          break;
        case 3: // Y Min
          plot_adc_min += step_adc;
          plot_adc_min = constrain(plot_adc_min, 0, plot_adc_max - 100); 
          break;
        case 4: // Y Max
          plot_adc_max += step_adc;
          plot_adc_max = constrain(plot_adc_max, plot_adc_min + 100, ADC_MAX_VALUE);
          break;
      }
      redrawScreen = true;
    }

    if (clicked) {
      settingsEditMode = false;
      redrawScreen = true;
    }

  } else {
    // Modo navegação entre itens de eixos
    if (rotation != 0) {
      settingsAxisSelection += rotation;
      if (settingsAxisSelection > SETTINGS_AXIS_MAX) settingsAxisSelection = 0;
      if (settingsAxisSelection < 0) settingsAxisSelection = SETTINGS_AXIS_MAX;
      redrawScreen = true;
    }

    if (clicked) {
      switch (settingsAxisSelection) {
        case 1: // X Min
        case 2: // X Max
        case 3: // Y Min
        case 4: // Y Max
          settingsEditMode = true;
          break;

        case 0: // Eixo X: alterna unidade de exibição
          if (currentXUnit == UNIT_PIXEL) currentXUnit = UNIT_NM;
          else if (currentXUnit == UNIT_NM) currentXUnit = UNIT_CM_1;
          else currentXUnit = UNIT_PIXEL;
          break;

        case 5: // Voltar
          changeState(STATE_SETTINGS); 
          break;
      }
      redrawScreen = true;
    }
  }
}

/**
 * @brief Trata entrada durante a aquisição contínua (usar clique para sair)
 */
void handleAcquisitionInput(bool clicked) {
  if (clicked) {
    changeState(STATE_MAIN_MENU);
  }
}

/**
 * @brief Seleciona a tela a ser desenhada de acordo com o estado atual
 */
void drawCurrentScreen() {
  switch (currentState) {
    case STATE_MAIN_MENU:
      drawMainMenuScreen();
      break;

    case STATE_SETTINGS:
      drawSettingsScreen();
      break;

    case STATE_SETTINGS_ACQ:
      drawSettingsAcqScreen();
      break;

    case STATE_SETTINGS_AXIS:
      drawSettingsAxisScreen();
      break;
      
    case STATE_ACQUISITION_CONT:
      // Primeiro quadro é desenhado aqui
      processData_Raw();
      applyDarkSpectrumCorrection();
      drawGraph_GFX(STATE_ACQUISITION_CONT); 
      break;
      
    case STATE_ACQUISITION_SINGLE:
    case STATE_ACQUISITION_DARK:
      // Esses modos são atualizados diretamente no loop()
      break;
  }
}

// ============================================================================
// FUNÇÕES PRINCIPAIS (CCD, PROCESSAMENTO E REDE)
// ============================================================================

/**
 * @brief Rotina de interrupção: chamada quando o STM32 sinaliza dado pronto
 */
void IRAM_ATTR onDataReady() {
  dataReady = true;
}

/**
 * @brief Rotina de interrupção do encoder rotativo
 */
void IRAM_ATTR readEncoderISR() {
  rotaryEncoder.readEncoder_ISR();
}

/**
 * @brief Monta o pacote de comando com parâmetros de aquisição para o STM32
 */
void buildCommand() {
  memset(tx_buffer, 0, PACKET_SIZE);

  tx_buffer[0] = 0x45; tx_buffer[1] = 0x52; // "ER"
  tx_buffer[2] = (SH_PERIOD >> 24) & 0xFF;
  tx_buffer[3] = (SH_PERIOD >> 16) & 0xFF;
  tx_buffer[4] = (SH_PERIOD >> 8)  & 0xFF;
  tx_buffer[5] = (SH_PERIOD >> 0)  & 0xFF;
  tx_buffer[6] = (ICG_PERIOD >> 24) & 0xFF;
  tx_buffer[7] = (ICG_PERIOD >> 16) & 0xFF;
  tx_buffer[8] = (ICG_PERIOD >> 8)  & 0xFF;
  tx_buffer[9] = (ICG_PERIOD >> 0)  & 0xFF;
  tx_buffer[10] = 0x01; // Campo de modo (reservado para uso futuro)
  tx_buffer[11] = (NUM_INTEGRATIONS >> 8) & 0xFF;
  tx_buffer[12] = (NUM_INTEGRATIONS >> 0) & 0xFF;
}

/**
 * @brief Troca pacote SPI com STM32: recebe dados e envia próximo comando
 */
void readDataAndRequestNext() {
  spi_stm32->beginTransaction(stm32_spiSettings);
  digitalWrite(PIN_STM_SS, LOW);
  spi_stm32->transferBytes(tx_buffer, rx_buffer, PACKET_SIZE);
  digitalWrite(PIN_STM_SS, HIGH);
  spi_stm32->endTransaction();
}

/**
 * @brief Envia o primeiro comando de aquisição sem aguardar dados (kick-start)
 */
void requestFirstScan() {
  Serial.println("Enviando primeiro comando (kick-start)...");
  spi_stm32->beginTransaction(stm32_spiSettings);
  digitalWrite(PIN_STM_SS, LOW);
  spi_stm32->writeBytes(tx_buffer, PACKET_SIZE);  
  digitalWrite(PIN_STM_SS, HIGH);
  spi_stm32->endTransaction();
}

/**
 * @brief Envia o buffer de espectro (pixel_data) para o cliente via Wi-Fi (TCP)
 */
void sendDataViaWiFi() {
  // Envia dados apenas se o cliente estiver conectado
  if (client && client.connected()) {
    Serial.println("Enviando dados via Wi-Fi (AP)...");

    // 1. Envia marcador de início
    client.write(PACKET_START_MARKER, 4);

    // 2. Envia os dados do espectro (corrigidos por dark)
    client.write((uint8_t*)pixel_data, sizeof(pixel_data));

    Serial.println("Envio Wi-Fi (AP) completo.");
  } else {
    Serial.println("Nenhum cliente conectado, envio ignorado.");
  }
}

/**
 * @brief Salva o espectro bruto atual como espectro de referência "dark"
 */
void saveDarkSpectrum() {
  memcpy(dark_spectrum_data, raw_pixel_data, sizeof(dark_spectrum_data));
  darkSpectrumSaved = true;
  Serial.println("Dark Spectrum Salvo!");

  // Feedback visual
  tft.fillScreen(COR_FUNDO);
  tft.setTextColor(ST7796S_YELLOW);
  tft.setTextSize(3);
  tft.setCursor(50, 150);
  tft.print("Dark Spectrum Salvo!");
  delay(1500); // Exibe mensagem temporariamente
}

/**
 * @brief Aplica correção de dark: subtrai espectro de referência do espectro bruto
 */
void applyDarkSpectrumCorrection() {
  if (darkSpectrumSaved) {
    for (int i = 0; i < PIXEL_COUNT; i++) {
      // Evita underflow (resultado negativo)
      if (raw_pixel_data[i] > dark_spectrum_data[i]) {
        pixel_data[i] = raw_pixel_data[i] - dark_spectrum_data[i];
      } else {
        pixel_data[i] = 0;
      }
    }
  } else {
    // Caso não haja referência dark, usa apenas os dados brutos
    memcpy(pixel_data, raw_pixel_data, sizeof(pixel_data));
  }
}

/**
 * @brief Converte dados recebidos via SPI (rx_buffer) para raw_pixel_data (uint16_t)
 */
void processData_Raw() {
  for (int i = 0; i < PIXEL_COUNT; i++) {
    int index = i * 2;
    uint8_t lsb = rx_buffer[index];
    uint8_t msb = rx_buffer[index + 1];

    // Conversão do par de bytes em contagem ADC com inversão
    raw_pixel_data[i] = ADC_MAX_VALUE - ((msb << 8) | lsb);
  }
}

// ============================================================================
// SETUP & LOOP
// ============================================================================

/**
 * @brief Configuração inicial do ESP32: display, Wi-Fi, SPI, encoder e calibração
 */
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 TCD1304 SPI Master - Modo AP/Servidor TCP");

  // 1. Inicializa o display TFT para apresentar mensagens de status
  Serial.println("Inicializando display TFT (VSPI)...");
  tft.begin();
  tft.setRotation(1); // Modo paisagem (480x320)
  Serial.println("Display OK.");

  // 2. Inicia o modo Access Point do Wi-Fi e o servidor TCP
  tft.fillScreen(COR_FUNDO);
  tft.setTextColor(COR_TEXTO);
  tft.setTextSize(2);
  tft.setCursor(20, 100);
  tft.print("Iniciando Rede Wi-Fi (AP)...");
  tft.setCursor(20, 130);
  tft.print(ap_ssid);

  Serial.print("Iniciando AP: ");
  Serial.println(ap_ssid);
  WiFi.softAP(ap_ssid, ap_password);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP: ");
  Serial.println(myIP);

  tft.fillScreen(COR_FUNDO);
  tft.setCursor(20, 100);
  tft.print("Rede Criada!");
  tft.setCursor(20, 130);
  tft.print("IP: ");
  tft.print(myIP);

  // Inicia o servidor TCP
  server.begin();
  Serial.print("Servidor TCP iniciado na porta: ");
  Serial.println(server_port);
  tft.setCursor(20, 160);
  tft.print("Servidor TCP na porta 9999");

  delay(2000);

  // 3. Calcula coeficientes de calibração espectral
  cal_slope_a = (CAL_LAMBDA_2 - CAL_LAMBDA_1) / (float)(CAL_PIXEL_2 - CAL_PIXEL_1);
  cal_intercept_b = CAL_LAMBDA_1 - cal_slope_a * (float)CAL_PIXEL_1;
  Serial.print("Calibracao: slope (a) = "); Serial.println(cal_slope_a, 6);
  Serial.print("Calibracao: intercept (b) = "); Serial.println(cal_intercept_b, 6);

  // 4. Inicializa o encoder rotativo com aceleração
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setAcceleration(250);
  lastEncoderValue = rotaryEncoder.readEncoder();

  // 5. Configura pino de seleção de chip (SS) do STM32
  pinMode(PIN_STM_SS, OUTPUT);
  digitalWrite(PIN_STM_SS, HIGH);

  // 6. Inicializa barramento HSPI usado para comunicação com o STM32
  Serial.println("Iniciando barramento HSPI (STM32)...");
  spi_stm32->begin(PIN_STM_SCLK, PIN_STM_MISO, PIN_STM_MOSI, -1);

  // 7. Configura pino de interrupção de dados prontos vindos do STM32
  pinMode(DATA_READY_PIN, INPUT);  
  attachInterrupt(digitalPinToInterrupt(DATA_READY_PIN), onDataReady, RISING);

  // 8. Prepara primeiro comando a ser enviado ao STM32
  buildCommand();

  Serial.println("Setup completo. Entrando no Menu Principal.");
  redrawScreen = true;
}

/**
 * @brief Loop principal: gerencia cliente TCP, UI, aquisição e desenho de telas
 */
void loop() {
  // --------------------------------------------------------------------------
  // Gerenciamento de conexão do cliente TCP (PC) ao servidor no ESP32
  // --------------------------------------------------------------------------
  if (server.hasClient()) {
    // Se já existe um cliente conectado, encerra antes de aceitar o novo
    if (client && client.connected()) {
      client.stop();
      Serial.println("Cliente antigo desconectado.");
    }

    // Aceita novo cliente
    client = server.available();
    if (client && client.connected()) {
      Serial.println("Novo cliente conectado!");
      redrawScreen = true; // Atualiza status na tela
    }
  }

  // Verifica se o cliente existente se desconectou
  if (client && !client.connected()) {
    Serial.println("Cliente desconectado.");
    client.stop(); // Garante fechamento da conexão
    redrawScreen = true; // Atualiza status na UI
  }

  // --------------------------------------------------------------------------
  // Leitura de entrada do encoder e detecção de clique / long press
  // --------------------------------------------------------------------------
  int rotation = 0;
  bool clicked = false;
  bool longPress = false; 

  static unsigned long lastTimePressed = 0;      // Controle anti-rebote do clique
  static unsigned long pressStartTime = 0;       // Instante de início do pressionamento
  static bool isPressing = false;                // Indica se está pressionando
  static bool longPressEventTriggered = false;   // Flag para disparo único de long press

  if (rotaryEncoder.encoderChanged()) {
    int16_t currentValue = rotaryEncoder.readEncoder();
    rotation = (currentValue > lastEncoderValue) ? 1 : -1;
    lastEncoderValue = currentValue;
  }

  // Lógica manual de detecção de clique e long press
  if (rotaryEncoder.isEncoderButtonDown()) {
    if (!isPressing) {
      // Início do pressionamento
      isPressing = true;
      pressStartTime = millis();
      longPressEventTriggered = false;
      Serial.println("Button Down");
    }

    // Verifica se tempo de long press foi atingido
    if (isPressing && !longPressEventTriggered && (millis() - pressStartTime > LONG_PRESS_DURATION)) {
      longPress = true;
      longPressEventTriggered = true;
      Serial.println("Long Press Event!");
    }
  } else {
    // Botão foi solto
    if (isPressing) {
      isPressing = false;
      unsigned long pressDuration = millis() - pressStartTime;

      // Clique curto (não foi long press e menor que 1 s)
      if (!longPressEventTriggered && pressDuration < 1000) { 
        if (millis() - lastTimePressed > 500) { // Anti-rebote
          clicked = true;
          lastTimePressed = millis();
          Serial.println("Button Clicked!");
        }
      }
      Serial.println("Button Up");
    }
  }

  // --------------------------------------------------------------------------
  // Lógica de navegação da UI com base no estado atual
  // --------------------------------------------------------------------------
  switch (currentState) {
    case STATE_MAIN_MENU:
      handleMainMenuInput(rotation, clicked);
      break;

    case STATE_SETTINGS:
      handleSettingsInput(rotation, clicked); 
      break;

    case STATE_SETTINGS_ACQ:
      handleSettingsAcqInput(rotation, clicked);
      break;

    case STATE_SETTINGS_AXIS:
      handleSettingsAxisInput(rotation, clicked);
      break;

    case STATE_ACQUISITION_CONT:
      handleAcquisitionInput(clicked);
      break;

    case STATE_ACQUISITION_SINGLE:
      if (rotation != 0) { 
        Serial.println("Saindo do modo unico (rotacao)...");
        changeState(STATE_MAIN_MENU);
      }
      // Clique é tratado mais abaixo, na lógica do modo único
      break;

    case STATE_ACQUISITION_DARK:
      if (clicked || rotation != 0) {
        changeState(STATE_MAIN_MENU);
      }
      if (longPress) {
        saveDarkSpectrum();
        changeState(STATE_MAIN_MENU); 
      }
      break;
  }

  // --------------------------------------------------------------------------
  // Lógica de aquisição de dados e atualização de gráfico
  // --------------------------------------------------------------------------
  if (currentState == STATE_ACQUISITION_CONT) {
    if (dataReady) {
      dataReady = false;  
      readDataAndRequestNext(); 
      
      processData_Raw(); 
      applyDarkSpectrumCorrection(); 
      sendDataViaWiFi();          // Envia espectro ao cliente
      drawGraph_GFX(STATE_ACQUISITION_CONT); 
    }
  }
  else if (currentState == STATE_ACQUISITION_SINGLE) {
    if (dataReady && waitingForSingleScan) {
      dataReady = false;
      waitingForSingleScan = false; 
      readDataAndRequestNext(); 
      
      processData_Raw(); 
      applyDarkSpectrumCorrection(); 
      sendDataViaWiFi();          // Envia espectro ao cliente
      drawGraph_GFX(STATE_ACQUISITION_SINGLE); 
    }
    else if (clicked && !waitingForSingleScan) {
      Serial.println("Modo Unico: Pedindo novo scan...");
      requestFirstScan(); 
      waitingForSingleScan = true; 

      tft.fillScreen(COR_FUNDO);
      tft.setTextColor(COR_TEXTO);
      tft.setTextSize(2);
      tft.setCursor(20, 150);
      tft.print("Aguardando nova aquisicao...");
    }
    if (dataReady && !waitingForSingleScan) {
      dataReady = false; 
    }
  }
  else if (currentState == STATE_ACQUISITION_DARK) {
    if (dataReady && waitingForSingleScan) {
      dataReady = false;
      waitingForSingleScan = false; 
      readDataAndRequestNext(); 
      
      processData_Raw(); 
      memcpy(pixel_data, raw_pixel_data, sizeof(pixel_data));
      drawGraph_GFX(STATE_ACQUISITION_DARK);
    }
    if (dataReady && !waitingForSingleScan) {
      dataReady = false; 
    }
  }

  // --------------------------------------------------------------------------
  // Redesenho das telas de menu/configuração quando necessário
  // --------------------------------------------------------------------------
  if (redrawScreen && 
     (currentState != STATE_ACQUISITION_CONT && 
      currentState != STATE_ACQUISITION_SINGLE &&
      currentState != STATE_ACQUISITION_DARK)) {
    drawCurrentScreen();
    redrawScreen = false;
  }

  delay(10); 
}
