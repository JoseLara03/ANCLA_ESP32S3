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
    return hash;
}
