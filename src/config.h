#pragma once

// --- Pantalla (pines oficiales Elecrow CrowPanel 2.1") ---
#define TFT_WIDTH   480
#define TFT_HEIGHT  480
#define TFT_BL_PIN  6      // Backlight (LEDC)

// Pines bus RGB paralelo
#define TFT_DE    40
#define TFT_VSYNC  7
#define TFT_HSYNC 15
#define TFT_PCLK  41
#define TFT_R0    46
#define TFT_R1     3
#define TFT_R2     8
#define TFT_R3    18
#define TFT_R4    17
#define TFT_G0    14
#define TFT_G1    13
#define TFT_G2    12
#define TFT_G3    11
#define TFT_G4    10
#define TFT_G5     9
#define TFT_B0     5
#define TFT_B1    45
#define TFT_B2    48
#define TFT_B3    47
#define TFT_B4    21

// Pines SPI inicialización ST7701
#define TFT_SPI_CS   16
#define TFT_SPI_SCK   2
#define TFT_SPI_SDA   1
#define TFT_RST      -1

// --- I2C (touch + PCF8574) ---
#define I2C_SDA  38
#define I2C_SCL  39

// PCF8574 @ 0x20 — control LCD power y reset
#define PCF8574_ADDR     0x21
#define PCF_LCD_POWER    3    // bit 3: alimentación LCD (1=ON)
#define PCF_LCD_RESET    4    // bit 4: reset LCD (0=reset activo, 1=normal)

// --- Encoder ---
#define ENC_A  42
#define ENC_B   4
#define ENC_SW  0
