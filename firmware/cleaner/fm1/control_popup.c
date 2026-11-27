/* SPDX-License-Identifier: GPL-3.0-only */
/* Control popup: see control_popup.h. */
#include "control_popup.h"

#define FM1CP__ON 0x80000000u
#define FM1CP__GEN(w) ((w) >> 19 & 0xFFFu)

static FM1CP_FN uint32_t fm1cp__clamp(uint32_t v, uint32_t max) { return v > max ? max : v; }

FM1CP_FN void fm1cp_init(fm1cp *c, uint32_t hold_ms, uint32_t min_ms)
{
    c->posted = 0;
    c->seen = 0;
    c->taken_at = 0;
    c->stamp = 0;
    c->hold_ms = (uint16_t)fm1cp__clamp(hold_ms, 0xFFFFu);
    c->min_ms = (uint16_t)fm1cp__clamp(min_ms, 0xFFFFu);
    c->active = c->dirty = 0;
    c->shown.id = 0;
    c->shown.value = c->shown.part = 0;
}

FM1CP_FN void fm1cp_post(fm1cp *c, uint32_t id, uint32_t value, uint32_t part)
{
    const uint32_t gen = (FM1CP__GEN(c->posted) + 1u) & 0xFFFu;
    c->posted = FM1CP__ON | gen << 19 | fm1cp__clamp(part, FM1CP_PART_MAX) << 17 | fm1cp__clamp(id, FM1CP_ID_MAX) << 7 |
                fm1cp__clamp(value, FM1CP_VALUE_MAX);
}

/* The word as one load; a newer generation than the last taken one. */
static FM1CP_FN int fm1cp__fresh(const fm1cp *c, uint32_t *w)
{
    *w = c->posted;
    return (*w & FM1CP__ON) && FM1CP__GEN(*w) != c->seen;
}

static FM1CP_FN void fm1cp__unpack(uint32_t w, fm1cp_notice *n)
{
    n->part = (uint8_t)(w >> 17 & 3u);
    n->id = (uint16_t)(w >> 7 & 0x3FFu);
    n->value = (uint8_t)(w & 0x7Fu);
}

FM1CP_FN int fm1cp_take(fm1cp *c, uint32_t now_ms, fm1cp_notice *out)
{
    uint32_t w;
    if (!fm1cp__fresh(c, &w)) return 0;
    if (c->taken_at && now_ms - c->taken_at < c->min_ms) return 0;   /* (kept waiting: the newest is taken later) */
    c->seen = FM1CP__GEN(w);
    c->taken_at = now_ms ? now_ms : 1u;
    fm1cp__unpack(w, out);
    return 1;
}

#ifndef FM1CP_NO_STEP
FM1CP_FN int fm1cp_step(fm1cp *c, uint32_t now_ms, int blocked)
{
    fm1cp_notice n;
    uint32_t w;
    if (blocked) {                       /* (a menu, dialog or naming owns the display: drop, do not queue) */
        if (fm1cp__fresh(c, &w)) c->seen = FM1CP__GEN(w);
        if (!c->active) return FM1CP_IDLE;
        c->active = c->dirty = 0;
        return FM1CP_RESTORE;
    }
    if (fm1cp_take(c, now_ms, &n)) {
        if (!c->active || n.id != c->shown.id || n.value != c->shown.value || n.part != c->shown.part)
            c->dirty = 1;
        c->shown = n;
        c->stamp = now_ms;
        c->active = 1;
    }
    if (!c->active) return FM1CP_IDLE;
    if (now_ms - c->stamp >= c->hold_ms) {
        c->active = c->dirty = 0;
        return FM1CP_RESTORE;
    }
    if (c->dirty) {
        c->dirty = 0;
        return FM1CP_DRAW;
    }
    return FM1CP_SHOWN;
}

FM1CP_FN const fm1cp_notice *fm1cp_shown(const fm1cp *c) { return &c->shown; }
#endif
