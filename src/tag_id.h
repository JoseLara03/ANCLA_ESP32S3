/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * FNV-1a 32-bit hash, used to derive a stable per-tag platform identity
 * (Tid) from a tag's EUI. See CLAUDE.md's "stable tag identity" entry for
 * why this exists and why it must not be replaced by anything that reads
 * only part of the EUI.
 */

#ifndef TAG_ID_H
#define TAG_ID_H

#include <stddef.h>
#include <stdint.h>

/* FNV-1a 32-bit over `len` bytes at `data`. Deterministic: the same bytes
 * always produce the same result, on any run, on any board. Not a
 * cryptographic hash -- it is chosen for even bit distribution across an
 * 8-byte EUI at negligible cost, not for collision resistance against an
 * adversary. */
uint32_t tag_id_from_eui(const uint8_t *data, size_t len);

#endif /* TAG_ID_H */
