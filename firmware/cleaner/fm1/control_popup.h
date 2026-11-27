/* SPDX-License-Identifier: GPL-3.0-only */
/* Control popup: show the MIDI control you just moved.
 *
 * The MIDI interrupt posts (control, value, part) with one 32-bit store; the
 * main loop takes the newest one, at most once per min_ms, and the host shows
 * it. Hosts that draw their own popup also use fm1cp_step for its lifetime:
 * hold_ms after the last change, cancelled while a menu, dialog or naming
 * screen owns the display, with one full-page restore when it ends.
 *
 * No allocation, no locks, no drawing, no globals. One producer (the MIDI
 * interrupt), one consumer (the main loop). */
#ifndef FM1CP_H
#define FM1CP_H

#include <stdint.h>

#ifndef FM1CP_FN
#define FM1CP_FN                         /* a host's section / inline attribute for these functions */
#endif

#define FM1CP_ID_MAX 1023u               /* a control id: a CC 0..127 or a host parameter index */
#define FM1CP_VALUE_MAX 127u
#define FM1CP_PART_MAX 3u                /* the host's part / track / channel group, 0..3 */

typedef struct {
    uint16_t id;
    uint8_t value;
    uint8_t part;
} fm1cp_notice;

typedef struct {
    volatile uint32_t posted;            /* interrupt side: 1 | gen 12 | part 2 | id 10 | value 7, 0 = none */
    uint32_t seen;                       /* main side: the generation last taken */
    uint32_t taken_at;                   /* fm1cp_take's last delivery (ms) */
    uint32_t stamp;                      /* fm1cp_step: the shown notice's time (ms) */
    uint16_t hold_ms, min_ms;
    uint8_t active, dirty;
    fm1cp_notice shown;
} fm1cp;

enum {
    FM1CP_IDLE,                          /* nothing shown: draw the page as usual */
    FM1CP_DRAW,                          /* draw the popup now (new or changed) */
    FM1CP_SHOWN,                         /* the popup owns the page and is current: draw nothing under it */
    FM1CP_RESTORE                        /* the popup ended or yielded: redraw the whole page once */
};

FM1CP_FN void fm1cp_init(fm1cp *c, uint32_t hold_ms, uint32_t min_ms);
/* Interrupt side. Values past their range are clamped. The newest post wins. */
FM1CP_FN void fm1cp_post(fm1cp *c, uint32_t id, uint32_t value, uint32_t part);
/* Main side: 1 and *out when a post newer than the last taken one is waiting
 * and min_ms has passed since the last delivery; it stays waiting otherwise. */
FM1CP_FN int fm1cp_take(fm1cp *c, uint32_t now_ms, fm1cp_notice *out);
/* Main side, once a frame, for hosts that draw their own popup: takes a new
 * notice (respecting min_ms), times it out, and yields while blocked (menu,
 * dialog, naming). Returns FM1CP_*; *shown is the notice to draw. */
#ifndef FM1CP_NO_STEP                    /* (a host with its own message strip needs only take) */
FM1CP_FN int fm1cp_step(fm1cp *c, uint32_t now_ms, int blocked);
FM1CP_FN const fm1cp_notice *fm1cp_shown(const fm1cp *c);
#endif

#endif
