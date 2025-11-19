# ESP32 TCD1304 SPI Master -- TCC2

Firmware desenvolvido como parte do Trabalho de Conclusão de Curso 2
(TCC2) de\
**Bruno Vinicius Machado Castanho**\
**Universidade Tecnológica Federal do Paraná (UTFPR) -- Câmpus Toledo**

O projeto implementa um sistema de aquisição para um espectrômetro de
baixo custo baseado no sensor linear **TCD1304**, utilizando um
**STM32F401RE** para geração dos sinais de controle do CCD e um
**ESP32** como mestre da aquisição, interface gráfica (TFT) e servidor
de rede (Wi-Fi em modo Access Point).

------------------------------------------------------------------------

## Visão Geral

O firmware do ESP32 é responsável por:

-   Configurar o **Wi-Fi em modo Access Point (AP)** e atuar como
    **servidor TCP**.
-   Enviar comandos de aquisição para o **STM32F401RE** via **SPI
    (HSPI)**.
-   Receber os dados de espectro do STM32 e processá-los.
-   Exibir o espectro em um **display TFT ST7796S**.
-   Enviar os dados processados ao PC via **TCP**, para
    visualização/armazenamento.
-   Implementar interface de usuário com **encoder rotativo**
    (navegação, clique e long press).
-   Implementar **dark spectrum** (subtração de fundo).
-   Permitir ajustes de:
    -   Período de SH (`SH_PERIOD`)
    -   Período de ICG (`ICG_PERIOD`)
    -   Limites de plotagem (X min/max, Y min/max)
    -   Unidade do eixo X: **pixels**, **nm** ou **cm⁻¹ (Raman shift)**

------------------------------------------------------------------------

## Hardware Utilizado

-   **ESP32**

    -   Interface SPI com STM32 (HSPI)
    -   Controle do display TFT ST7796S
    -   Gerenciamento do encoder rotativo
    -   Modo Wi-Fi Access Point + servidor TCP

-   **STM32F401RE**

    -   Geração dos sinais de controle do TCD1304 (ICG, SH, clock)
    -   Leitura do sinal analógico do CCD (ADC + DMA)
    -   Envio dos dados de pixels via SPI para o ESP32

-   **Sensor CCD Linear TCD1304**

-   **Display TFT**

    -   Controlador: **ST7796S**
    -   Resolução: 480x320 (orientação paisagem)

-   **Encoder Rotativo**

    -   Navegação entre menus
    -   Clique curto para seleção
    -   Long press para salvar dark spectrum

------------------------------------------------------------------------

## Ligações Principais (ESP32)

### Display TFT

-   `PIN_TFT_CS` → GPIO 15\
-   `PIN_TFT_DC` → GPIO 2\
-   `PIN_TFT_RST` → -1

### SPI com STM32

-   `PIN_STM_SCLK` → GPIO 14\
-   `PIN_STM_MISO` → GPIO 12\
-   `PIN_STM_MOSI` → GPIO 13\
-   `PIN_STM_SS` → GPIO 5

### Encoder

-   `CLK` → GPIO 26\
-   `DT` → GPIO 27\
-   `SW` → GPIO 25

------------------------------------------------------------------------

## Máquina de Estados

Estados principais:

    STATE_MAIN_MENU
    STATE_SETTINGS
    STATE_SETTINGS_ACQ
    STATE_SETTINGS_AXIS
    STATE_ACQUISITION_CONT
    STATE_ACQUISITION_SINGLE
    STATE_ACQUISITION_DARK

------------------------------------------------------------------------

## Calibração Espectral

Conversão pixel → comprimento de onda:

    λ = a * pixel + b

Com dois pontos de calibração:

-   532 nm em pixel 387\
-   632.8 nm em pixel 2416

Conversão para deslocamento Raman:

    ṽ = (1/λ_exc − 1/λ) × 10^7

------------------------------------------------------------------------

## Licença

    Copyright (c) 2025  
    Autor: Bruno Vinicius Machado Castanho
    Todos os direitos reservados.
