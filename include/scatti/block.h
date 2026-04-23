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
