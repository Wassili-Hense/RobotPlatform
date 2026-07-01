
#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <mbedtls/version.h>
#include <mbedtls/sha256.h>
#include "pen_proto.h"

static constexpr uint8_t PEN_CHANNEL = 1U;
static constexpr uint32_t PEN_CAPS = 0x00000001U;
static constexpr char PEN_BIND_SECRET[] = "PEN-DEMO-BIND-SECRET";
static constexpr uint8_t PEN_BROADCAST_MAC[6] = { 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU };

static inline uint32_t pen_make_id_from_efuse(void) {
    return (uint32_t)(ESP.getEfuseMac() & 0xFFFFFFFFULL);
}

static inline bool pen_get_sta_mac(uint8_t outMac[6]) {
    if (outMac == nullptr) return false;
    return esp_wifi_get_mac(WIFI_IF_STA, outMac) == ESP_OK;
}

static inline void pen_sha256_start(mbedtls_sha256_context* ctx) {
    mbedtls_sha256_init(ctx);
#if defined(MBEDTLS_VERSION_NUMBER) && (MBEDTLS_VERSION_NUMBER >= 0x03000000)
    (void)mbedtls_sha256_starts(ctx, 0);
#else
    (void)mbedtls_sha256_starts_ret(ctx, 0);
#endif
}

static inline void pen_sha256_update(mbedtls_sha256_context* ctx, const void* data, size_t len) {
#if defined(MBEDTLS_VERSION_NUMBER) && (MBEDTLS_VERSION_NUMBER >= 0x03000000)
    (void)mbedtls_sha256_update(ctx, reinterpret_cast<const unsigned char*>(data), len);
#else
    (void)mbedtls_sha256_update_ret(ctx, reinterpret_cast<const unsigned char*>(data), len);
#endif
}

static inline void pen_sha256_finish(mbedtls_sha256_context* ctx, uint8_t out[32]) {
#if defined(MBEDTLS_VERSION_NUMBER) && (MBEDTLS_VERSION_NUMBER >= 0x03000000)
    (void)mbedtls_sha256_finish(ctx, out);
#else
    (void)mbedtls_sha256_finish_ret(ctx, out);
#endif
    mbedtls_sha256_free(ctx);
}

static inline void pen_sha256_label(const char* label, const uint8_t sessionKey[32], uint8_t out[32]) {
    mbedtls_sha256_context ctx;
    pen_sha256_start(&ctx);
    pen_sha256_update(&ctx, sessionKey, 32U);
    pen_sha256_update(&ctx, label, strlen(label));
    pen_sha256_finish(&ctx, out);
}

static inline void pen_derive_session(uint32_t rcId,
                                      uint32_t deviceId,
                                      uint32_t sessionId,
                                      const uint8_t rcMac[6],
                                      const uint8_t devMac[6],
                                      const uint8_t rcNonce[16],
                                      const uint8_t devNonce[16],
                                      uint8_t sessionKey[32],
                                      uint8_t pmk[16],
                                      uint8_t lmk[16],
                                      uint8_t rcProof[16],
                                      uint8_t devProof[16]) {
    mbedtls_sha256_context ctx;
    uint8_t tmp[32];
    pen_sha256_start(&ctx);
    pen_sha256_update(&ctx, PEN_BIND_SECRET, strlen(PEN_BIND_SECRET));
    pen_sha256_update(&ctx, &rcId, sizeof(rcId));
    pen_sha256_update(&ctx, &deviceId, sizeof(deviceId));
    pen_sha256_update(&ctx, &sessionId, sizeof(sessionId));
    pen_sha256_update(&ctx, rcMac, 6U);
    pen_sha256_update(&ctx, devMac, 6U);
    pen_sha256_update(&ctx, rcNonce, 16U);
    pen_sha256_update(&ctx, devNonce, 16U);
    pen_sha256_finish(&ctx, sessionKey);

    pen_sha256_label("PMK", sessionKey, tmp); memcpy(pmk, tmp, 16U);
    pen_sha256_label("LMK", sessionKey, tmp); memcpy(lmk, tmp, 16U);
    pen_sha256_label("RC", sessionKey, tmp); memcpy(rcProof, tmp, 16U);
    pen_sha256_label("DEV", sessionKey, tmp); memcpy(devProof, tmp, 16U);
}

static inline bool pen_add_peer(const uint8_t mac[6], bool encrypt, const uint8_t lmk[16]) {
    if (mac == nullptr) return false;
    if (esp_now_is_peer_exist(mac)) (void)esp_now_del_peer(mac);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6U);
    peer.channel = PEN_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = encrypt;
    if (encrypt && (lmk != nullptr)) memcpy(peer.lmk, lmk, 16U);
    return esp_now_add_peer(&peer) == ESP_OK;
}

template <typename PayloadT>
static inline bool pen_send_frame(const uint8_t mac[6], uint8_t msgType, uint32_t sessionId, uint16_t seq, const PayloadT& payload) {
    struct __attribute__((packed)) frame_t { pen_hdr_t hdr; PayloadT payload; pen_crc_t crc; } frame = {};
    frame.hdr.magic = PEN_MAGIC;
    frame.hdr.msgType = msgType;
    frame.hdr.sessionId = sessionId;
    frame.hdr.seq = seq;
    frame.payload = payload;
    if (!pen_finalize_frame(&frame, sizeof(frame))) return false;
    return esp_now_send(mac, reinterpret_cast<const uint8_t*>(&frame), sizeof(frame)) == ESP_OK;
}

static inline bool pen_send_empty_frame(const uint8_t mac[6], uint8_t msgType, uint32_t sessionId, uint16_t seq) {
    struct __attribute__((packed)) frame_t { pen_hdr_t hdr; pen_crc_t crc; } frame = {};
    frame.hdr.magic = PEN_MAGIC;
    frame.hdr.msgType = msgType;
    frame.hdr.sessionId = sessionId;
    frame.hdr.seq = seq;
    if (!pen_finalize_frame(&frame, sizeof(frame))) return false;
    return esp_now_send(mac, reinterpret_cast<const uint8_t*>(&frame), sizeof(frame)) == ESP_OK;
}
