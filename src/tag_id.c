/*
 * Copyright (c) 2026 Innovaforce
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tag_id.h"

uint32_t tag_id_from_eui(const uint8_t *data, size_t len)
{
    uint32_t hash = 2166136261u;   /* FNV-1a 32-bit offset basis */

    for (size_t i = 0; i < len; i++) {
        hash ^= data[i];
        hash *= 16777619u;         /* FNV-1a 32-bit prime */
    }

    /* Clear bit 31 so the result always fits a POSITIVE signed 32-bit
     * integer. The downstream platform stores Tid in a signed 32-bit column;
     * a value above INT32_MAX arrives there negative and the record is
     * silently dropped. See tag_id.h for why this is masked and not
     * remapped. */
    return hash & 0x7FFFFFFFu;
}
