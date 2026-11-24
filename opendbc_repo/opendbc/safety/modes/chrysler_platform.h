#pragma once

// Platform declarations shared by the stock and optional AOL owners.

// Chrysler Pacifica/Jeep addresses
#define CHRYSLER_EPS_2            0x220U  // EPS driver input torque
#define CHRYSLER_ESP_1            0x140U  // Brake pedal and vehicle speed
#define CHRYSLER_ESP_8            0x11CU  // Brake pedal and vehicle speed
#define CHRYSLER_ECM_5            0x22FU  // Throttle position sensor
#define CHRYSLER_DAS_3            0x1F4U  // ACC engagement states from DASM
#define CHRYSLER_DAS_6            0x2A6U  // LKAS HUD and auto headlight control from DASM
#define CHRYSLER_LKAS_COMMAND     0x292U  // LKAS controls from DASM
#define CHRYSLER_CRUISE_BUTTONS   0x23BU  // Cruise control buttons

// RAM DT addresses
#define CHRYSLER_RAM_DT_EPS_2            0x31U
#define CHRYSLER_RAM_DT_ESP_1            0x83U
#define CHRYSLER_RAM_DT_ESP_8            0x79U
#define CHRYSLER_RAM_DT_ECM_5            0x9DU
#define CHRYSLER_RAM_DT_DAS_3            0x99U
#define CHRYSLER_RAM_DT_DAS_6            0xFAU
#define CHRYSLER_RAM_DT_LKAS_COMMAND     0xA6U
#define CHRYSLER_RAM_DT_CRUISE_BUTTONS   0xB1U

// RAM HD addresses
#define CHRYSLER_RAM_HD_EPS_2            0x220U
#define CHRYSLER_RAM_HD_ESP_1            0x140U
#define CHRYSLER_RAM_HD_ESP_8            0x11CU
#define CHRYSLER_RAM_HD_ECM_5            0x22FU
#define CHRYSLER_RAM_HD_DAS_3            0x1F4U
#define CHRYSLER_RAM_HD_DAS_6            0x275U
#define CHRYSLER_RAM_HD_LKAS_COMMAND     0x276U
#define CHRYSLER_RAM_HD_CRUISE_BUTTONS   0x23AU

typedef enum {
  CHRYSLER_RAM_DT,
  CHRYSLER_RAM_HD,
  CHRYSLER_PACIFICA,  // plus Jeep
} ChryslerPlatform;
static ChryslerPlatform chrysler_platform;

#define CHRYSLER_ADDR(name) ((uint32_t)((chrysler_platform == CHRYSLER_RAM_DT) ? CHRYSLER_RAM_DT_##name : \
                                        ((chrysler_platform == CHRYSLER_RAM_HD) ? CHRYSLER_RAM_HD_##name : CHRYSLER_##name)))
