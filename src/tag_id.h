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

/* FNV-1a 32-bit over `len` bytes at `data`, with bit 31 cleared.
 * Deterministic: the same bytes always produce the same result, on any run,
 * on any board. Not a cryptographic hash -- it is chosen for even bit
 * distribution across an 8-byte EUI at negligible cost, not for collision
 * resistance against an adversary.
 *
 * THE RESULT IS ALWAYS <= INT32_MAX, and that is a wire requirement, not a
 * detail. The downstream platform stores Tid in a SIGNED 32-bit column: a
 * value above INT32_MAX arrives negative and the record is dropped without
 * an error anywhere. Diagnosed on the bench 2026-08-24 -- a tag hashing to
 * 2728562623 (0xA2A28FBF) never appeared on the platform while two tags at
 * 693116308 and 2082962887 did, with the gateway logging and publishing all
 * three correctly (pos_json_fix() emits %u and was verified to be blameless).
 *
 * Masking rather than remapping is deliberate: `& 0x7FFFFFFF` is the identity
 * for every hash that already fits, so tags already visible on the platform
 * KEEP their existing Tid and only the broken ones change. Anything that
 * remapped the whole range -- a modulo, a fold -- would renumber every tag
 * and orphan every existing platform record.
 *
 * Cost: the id space halves to 2^31. For a deployment of a few hundred tags
 * the birthday collision probability stays under 1e-4, which is the same
 * accepted-collision-bound argument that justified hashing the EUI at all. */
uint32_t tag_id_from_eui(const uint8_t *data, size_t len);

#endif /* TAG_ID_H */
