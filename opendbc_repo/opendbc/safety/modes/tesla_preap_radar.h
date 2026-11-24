#pragma once
#include "opendbc/safety/can_tx.h"

static uint8_t tesla_preap_crc(const CANPacket_t *msg) {
  unsigned int crc = 255U;
  for (int i = 0; i < 7; i++) {
    crc ^= msg->data[i];
    for (int bit = 0; bit < 8; bit++) {
      crc = ((crc & 128U) != 0U) ? ((crc << 1U) ^ 29U) : (crc << 1U);
      crc &= 255U;
    }
  }
  return (uint8_t)(crc ^ 255U);
}

static bool tesla_preap_radar_enabled = false;
static bool tesla_preap_radar_nosecone = false;
static const CANPacket_t *tesla_preap_radar_ticket = NULL;

static bool tesla_preap_radar_tx(const CANPacket_t *msg) {
  const CANPacket_t *expected = tesla_preap_radar_ticket;
  tesla_preap_radar_ticket = NULL;
  bool accepted = tesla_preap_radar_enabled && (expected != NULL) &&
    (msg->bus == 1U) && (msg->addr == expected->addr) &&
    (GET_LEN(msg) == GET_LEN(expected)) && !msg->fd && !msg->extended && !msg->returned && !msg->rejected;
  if (expected != NULL) {
    for (unsigned int i = 0U; i < GET_LEN(expected); i++) {
      accepted = accepted && (msg->data[i] == expected->data[i]);
    }
  }
  return accepted;
}

static uint32_t tesla_preap_radar_word(const CANPacket_t *msg, unsigned int offset) {
  return (uint32_t)msg->data[offset] | ((uint32_t)msg->data[offset + 1U] << 8U) |
    ((uint32_t)msg->data[offset + 2U] << 16U) | ((uint32_t)msg->data[offset + 3U] << 24U);
}

static void tesla_preap_radar_put(CANPacket_t *msg, unsigned int offset, uint32_t value) {
  for (unsigned int i = 0U; i < 4U; i++) { msg->data[offset + i] = (uint8_t)(value >> (8U * i)); }
}

static void tesla_preap_radar_sum(CANPacket_t *msg) {
  unsigned int sum = (msg->addr & 255U) + (msg->addr >> 8U);
  for (unsigned int i = 0U; i < (GET_LEN(msg) - 1U); i++) { sum += msg->data[i]; }
  msg->data[GET_LEN(msg) - 1U] = (uint8_t)(sum & 255U);
}

static void tesla_preap_radar_emit(CANPacket_t *packet) {
  if (tesla_preap_radar_enabled && !relay_malfunction && (tesla_preap_radar_ticket == NULL)) {
    can_set_checksum(packet);
    tesla_preap_radar_ticket = packet;
    can_send(packet, 1U, false);
    tesla_preap_radar_ticket = NULL;
  }
}

static void tesla_preap_radar_copy(const CANPacket_t *source, uint32_t destination) {
  CANPacket_t packet = {0};
  packet.addr = destination;
  packet.bus = 1U;
  packet.data_len_code = source->data_len_code;
  for (unsigned int i = 0U; i < GET_LEN(source); i++) { packet.data[i] = source->data[i]; }
  tesla_preap_radar_emit(&packet);
}

static void tesla_preap_radar_rx(const CANPacket_t *msg) {
  if (tesla_preap_radar_enabled && (msg->bus == 0U) && !msg->fd && !msg->extended && !msg->returned && !msg->rejected) {
    if (msg_matches(msg, 0x45U, 0U, 8U)) { tesla_preap_radar_copy(msg, 0x219U); }
    else if (msg_matches(msg, 0x108U, 0U, 8U)) { tesla_preap_radar_copy(msg, 0x109U); }
    else if (msg_matches(msg, 0x145U, 0U, 8U)) { tesla_preap_radar_copy(msg, 0x149U); }
    else if (msg_matches(msg, 0x20AU, 0U, 8U)) { tesla_preap_radar_copy(msg, 0x159U); }
    else if (msg_matches(msg, 0x308U, 0U, 8U)) { tesla_preap_radar_copy(msg, 0x209U); }
    else if (msg_matches(msg, 0x30AU, 0U, 8U)) { tesla_preap_radar_copy(msg, 0x2D9U); }
    else if (msg_matches(msg, 0x405U, 0U, 8U)) { tesla_preap_radar_copy(msg, 0x2B9U); }
    else if (msg_matches(msg, 0x398U, 0U, 8U)) {
      CANPacket_t packet = {0};
      packet.addr = 0x2A9U;
      packet.bus = 1U;
      packet.data_len_code = 8U;
      const uint32_t low = (tesla_preap_radar_word(msg, 0U) & 0xFFFFF33FU) | 0x100U | 0x440U;
      const uint32_t high = (tesla_preap_radar_word(msg, 4U) & 0xCFFF0F0FU) | 0x10000000U |
        (tesla_preap_radar_nosecone ? 16U : 0U);
      tesla_preap_radar_put(&packet, 0U, low);
      tesla_preap_radar_put(&packet, 4U, high);
      tesla_preap_radar_sum(&packet);
      tesla_preap_radar_emit(&packet);
    } else if (msg_matches(msg, 0xEU, 0U, 8U) && (tesla_preap_crc(msg) == msg->data[7])) {
      CANPacket_t packet = *msg;
      packet.bus = 1U;
      packet.addr = 0x199U;
      uint32_t low = tesla_preap_radar_word(msg, 0U);
      uint32_t high = tesla_preap_radar_word(msg, 4U);
      if (((low >> 16U) & 0xFF3FU) == 0xFF3FU) {
        low = (low & 0x00C0FFFFU) | 0x00200000U;
        high = (high & 0x00FFFFF0U) | 4U;
        tesla_preap_radar_put(&packet, 0U, low);
        tesla_preap_radar_put(&packet, 4U, high);
        packet.data[7] = tesla_preap_crc(&packet);
      }
      tesla_preap_radar_emit(&packet);
    } else if (msg_matches(msg, 0x115U, 0U, 6U)) {
      tesla_preap_radar_copy(msg, 0x129U);
      CANPacket_t packet = {0};
      packet.addr = 0x1A9U;
      packet.bus = 1U;
      packet.data_len_code = 5U;
      const unsigned int counter = msg->data[4] >> 4U;
      tesla_preap_radar_put(&packet, 0U, 0x000C0000U | ((uint32_t)counter << 28U));
      packet.data[4] = (uint8_t)((0x38U + 12U + (counter << 4U)) & 255U);
      tesla_preap_radar_emit(&packet);
    } else if (msg_matches(msg, 0x118U, 0U, 6U)) {
      tesla_preap_radar_copy(msg, 0x119U);
      const unsigned int raw_speed = (tesla_preap_radar_word(msg, 0U) >> 16U) & 4095U;
      uint32_t speed = 8191U;
      if (raw_speed != 4095U) {
        const int mph_x100 = ((int)raw_speed * 5) - 2500;
        const int kph_x100 = (mph_x100 * 1609) / 1000;
        const int scaled_speed = kph_x100 / 4;
        speed = (kph_x100 < 0) ? 0U : ((uint32_t)scaled_speed & 8191U);
      }
      CANPacket_t packet = {0};
      packet.addr = 0x169U;
      packet.bus = 1U;
      packet.data_len_code = 8U;
      const uint32_t low = speed | (speed << 13U) | (speed << 26U);
      const uint32_t high = ((speed >> 6U) | (speed << 7U) | ((uint32_t)(msg->data[4] & 15U) << 20U)) & 0x00FFFFFFU;
      tesla_preap_radar_put(&packet, 0U, low);
      tesla_preap_radar_put(&packet, 4U, high);
      unsigned int checksum = 0x76U;
      for (unsigned int i = 0U; i < 7U; i++) { checksum += packet.data[i]; }
      packet.data[7] = (uint8_t)(checksum & 255U);
      tesla_preap_radar_emit(&packet);
    } else {
    }
  }
}
