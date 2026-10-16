/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef HMAC_SHA256_H
#define HMAC_SHA256_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * SHA-256 and HMAC-SHA256 (FIPS 180-4, RFC 2104), pure C, no dependencies.
 *
 * Why not mbedTLS, which this image already links for WiFi/TLS: the remote-command
 * authentication (cmd_proto.c) has to be host-tested against the RFC 4231 vectors and
 * against the Python tool that signs the commands, and a host test cannot link a
 * Zephyr-configured mbedTLS. ~100 lines of well-known arithmetic, verified against the
 * published vectors, is the smaller risk.
 */

#define HMAC_SHA256_LEN 32u

/* Plain SHA-256. Public for one use: the key FINGERPRINT (first 4 bytes of SHA-256(key)) that the
 * shell and the Python tool both print, so an operator can see that the key a node stored is the
 * key the server holds without ever displaying the key. */
void ancla_sha256(const uint8_t *msg, size_t len, uint8_t out[32]);

void ancla_hmac_sha256(const uint8_t *key, size_t key_len, const uint8_t *msg, size_t msg_len,
		 uint8_t out[HMAC_SHA256_LEN]);

/* Constant-time equality for two MACs: the comparison time must not reveal how many leading
 * bytes of a forged signature were right. */
bool ancla_hmac_sha256_equal(const uint8_t a[HMAC_SHA256_LEN], const uint8_t b[HMAC_SHA256_LEN]);

#endif /* HMAC_SHA256_H */
