/********************************************************************
* Permission is hereby granted, free of charge, to any person obtaining
* a copy of this software and associated documentation files (the
* "Software"), to deal in the Software without restriction, including
* without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to
* permit persons to whom the Software is furnished to do so, subject
* to the following conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
* Description: scatti - pure C99 jerk-limited real-time trajectory
*              generation. Port of Ruckig with embed-friendly
*              memory and math hooks.
* Author:      杨阳 (Yang Yang) <mika-net@outlook.com>
* Origin:      Based on Ruckig Community Edition by Lars Berscheid
*              (https://github.com/pantor/ruckig)
* License:     MIT (SPDX-License-Identifier: MIT)
* Copyright (c) 2026 杨阳 (Yang Yang)
********************************************************************/

#ifndef SCATTI_BLOCK_H
#define SCATTI_BLOCK_H

#include <stdbool.h>
#include <stddef.h>
#include <scatti/profile.h>
#include <scatti/scatti_config.h>

typedef struct {
    double left, right;
    SCattiProfile profile;
    bool valid;
} SCattiInterval;

typedef struct {
    SCattiProfile p_min;
    double t_min;
    SCattiInterval a;
    SCattiInterval b;
} SCattiBlock;

void scatti_block_init(SCattiBlock *block);
void scatti_block_set_min_profile(SCattiBlock *block, const SCattiProfile *profile);

/* Calculate block from valid profiles. Returns true if successful. */
bool scatti_block_calculate(SCattiBlock *block, SCattiProfile *valid_profiles,
                     size_t valid_profile_counter, size_t max_profiles);

/* Inlined for hot-path performance (called in tight synchronization loop) */
SCATTI_FORCE_INLINE bool scatti_block_is_blocked(const SCattiBlock *block, double t) {
    return (t < block->t_min)
        || (block->a.valid && block->a.left < t && t < block->a.right)
        || (block->b.valid && block->b.left < t && t < block->b.right);
}

const SCattiProfile* scatti_block_get_profile(const SCattiBlock *block, double t);

#endif /* SCATTI_BLOCK_H */
