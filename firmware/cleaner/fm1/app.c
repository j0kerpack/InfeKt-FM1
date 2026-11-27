/* SPDX-License-Identifier: GPL-3.0-only */
/* Copyright (C) 2026 J0KERPACK */
/* InfeKt-FM1 on fm1-base: the synth engine (build/cleaner_ble/engine, fm1.toml o2) played from the keys, MIDI over
 * USB, the TRS jack and Bluetooth, the patch and four parameters a page on the knobs.
 *
 * The engine renders in the audio interrupt (app_render), one 128-frame block per call; everything that reaches it
 * (the keys, the knobs, a patch change, MIDI in) is taken there, at a block's start, so only that interrupt touches
 * the engine. The main loop reads the panel and draws.
 *
 *   keys       play (with velocity); they go out over MIDI too (GLO > MIDI > OUT: the channel, USB and Bluetooth)
 *   OCT- OCT+  the octave (each on its release; OCT- pressed less than 0.5 s with nothing else: held 0.5 s, it is
 *              SHIFT, and HOME lights); both together, held 0.3 s: the quick PANIC (quick_panic), everything stops;
 *              on the sequencers' pages a tap shows the block of 16 steps left / right instead (seq_blk)
 *   SHIFT+HOME undo: a knob's turn or a patch change back, newest first (the sequencers' pages: the pattern's, un_*)
 *   SHIFT+C#5/D#5 (the top C# and D#, the only keys lit with SHIFT) the key mode: MONO / POLY
 *   SHIFT+A#4/F#5 (the USB build, two parts) the part the panel edits: A / B (lit with SHIFT: the
 *              other part's key); GLO > MULTI: the mode (SINGLE, LAYER, SPLIT, CHANNELS), the split, B's channel, the
 *              voices; GLO > AUDIO: MAX VOICES, the unison caps, STEAL
 *   PRESET     the patch (on HOME only: locked in the menus; INIT, A, B, C .. H, the empty slots left out), the
 *              list of patches up for 2 s; a patch's edits stay
 *              in RAM when another is chosen (not over a power-off)
 *   HOME       the HOME screen (at power-on): the patch, the load, the settings, the connections, MIDI in; in a list
 *              or a group: one level up; on HOME with two parts (the USB build, not SINGLE): a tap switches the part
 *              edited (part B: the screen green)
 *   SELECT     on HOME: the quick-access list, a block a step (K1..K4 change its four parameters), then EDIT (REC
 *              opens the list's slots: REC on one opens the blocks to choose from, REC keeps one, HOME goes back);
 *              in a menu: its blocks, then EDIT and BACK; in a list: the cursor
 *   GLO        the global settings (GLO's section below): TRANSPOSE, MIDI, BLUETOOTH, AUDIO; saved by themselves
 *   FX SEL ENV LFO EDIT ARP  the menus (ui.c U_MENU): a menu's blocks, SELECT a step, K1..K4 the
 *              lit one; EDIT (REC) the list of its groups, a group its rows the same way; BACK (REC) or HOME up a
 *              level. SEL again: its other page (filters, oscillators); EDIT: the matrix's six assigns (a
 *              sequencer's page: its EDIT list, se_*). A list-valued parameter shows its values over the rows while
 *              it turns (2 s)
 *   ALGO       the tempo (the one running: kept over a patch change while a sequencer runs, tp_run; both parts'.
 *              Two parts, 1.18: one global tempo, GLO's, no patch sets it)
 *   SHIFT+PLAY PLAY mode (K1..K4, SELECT, PRESET, ALGO: seven assigned parameters; PLAY red), again its assignment
 *              editor, again off (play_*); a menu's button, GLO or HOME leaves it
 *   SEQ        the arp sequencer (16, 32 or 64 steps of 1/16, a block of 16 on the white keys, 32 patterns, sq_*,
 *              the development notes): its page (PLAY starts it); on the page again: the drum sequencer's page (11
 *              synthesized instruments on the black keys, 16 to 64 steps each, 16 patterns, dr_*
 *              13); again: back (both keep running); in the two-part builds then the pattern launcher
 *              (ln_*, 1.18C10 / 1.18C11), then the drums' page again; SHIFT+SEQ both off
 *   SAVE       the user banks C .. H: a slot (SELECT, K1 or OCT- / OCT+ a bank), REC its name (SELECT the
 *              letter, K1 or OCT- / OCT+ the cursor, K2 the case), REC writes the patch there (a filled slot: REC
 *              again) and keeps the octave, the patch and the settings; HOME back (sv_*). Lit while the patch has
 *              edits not saved
 *   REC        where it has no other meaning (HOME, the menus' blocks, PLAY mode): HOLD on and off (lit while on);
 *              the sequencers' pages but their CLEAR: RECORD (dr_arm, sq_arm); held a second (the Bluetooth build): Bluetooth off / on. In the
 *              lists, GLO, SAVE and the name editor, on EDIT / BACK, the quick-access slots, CLEAR and the kit's RESET
 *              it keeps its meaning there (looking for a Bluetooth MIDI device: GLO > BLUETOOTH > SEARCH)
 *   PLAY       the sequencers' transport, from any view: the arp and drum sequencers whose PATTERN is not NONE start
 *              together on the shared clock, again both stop (lit while one runs); MIDI START / STOP with CLOCK
 *              EXTERNAL the same
 *   MASTER     the volume
 *   HOME       held: the block's time by part on the status line; held 3 s, no key down: the chip's benchmark
 *
 * Part of the unity build (fm1-base device/unity.c), after the platform: gfx.c and announce.c are in scope. */
#define SY_MAX_VOICES 16                 /* (as the engine's units are built: fm1.toml o2_cflags; checked at start against sy_synth_size) */
#define SY_DELAY_FRAMES 32768
#define SY_CORES 2
#ifndef SY_UNI_CAP
#define SY_UNI_CAP                       /* (the unison caps and UNISON FIRST, both builds: fm1.toml o2_cflags) */
#endif
#include "synth.h"
/* No factory banks in the image (no
 * third-party data). The index space stays (A, B: 0 .. 255, never filled, hidden from PRESET, the lists and OCT's bank
 * steps), the player's banks (the old C, D, E: an update keeps their patches; six since 1.18C30, the bank store) are
 * shown as A .. F (patch_tag, the bank upload's strip; the page maps its bank numbers 0-5 the same) */
#define CLEANER_NO_FACTORY 1
#define BANK_N 0u                        /* (bank A: none) */
#define BANKB_N 128u                     /* (bank B's slots: the index space only) */
#include "init_patch.h"            /* (below A001: the init patch, a plain saw) */
/* the patches by index (set.patch): eight banks of 128 slots, A (the firmware's: its first BANK_N filled, the rest
 * empty for now), B (the firmware's), C .. H (the player's six, in the store: ub_* below; C, D, E alone until
 * 1.18C29), then the init patch (640 until 1.18C29, OLD_PATCH_INIT: the saved indices that hold it are moved,
 * settings_t pt_ok PT_V2, the launcher's sets LS_V2). A patch's name on the screen is its bank's letter and slot,
 * "C012" (patch_tag; the cleaner, CLEANER_NO_FACTORY: the user banks as A .. F) */
#define PA_N 128u                        /* bank A's slots */
#define PB_0 PA_N                        /* bank B's first */
#define PU_0 (PB_0 + BANKB_N)            /* the user banks' first (C001) */
#define UB_BANKS 6u                      /* C .. H (1.18C30; C, D, E before: UB_BANKS_OLD) */
#define UB_BANKS_OLD 3u
#define UB_SLOTS 128u
#define UB_N (UB_BANKS * UB_SLOTS)
#define PATCH_INIT (PU_0 + UB_N)         /* the init patch (below A001 in PRESET's order) */
#define OLD_PATCH_INIT (PU_0 + UB_BANKS_OLD * UB_SLOTS)   /* (its index until 1.18C29: 640) */
#define PATCH_N (PATCH_INIT + 1u)
#define UB_L0 'A'                        /* (the user banks' first letter on the screen: CLEANER_NO_FACTORY) */
/* the patch's extension (0.89): FX > SHIMMER's values, a patch's own beside its 256 bytes (the format
 * has no byte to spare; the engine never sees them as patch bytes, a dump never carries them): the panel's shadow
 * holds them past the patch, bytes 256 .. 256 + SHX_N - 1, so the edit memory, undo, SAVE and the edited mark take
 * them as any byte. The engine gets part A's alone (shx_sync: the reverb is A's). A factory patch, the init patch, a
 * single from SysEx and every slot saved before 0.89: the defaults, shimmer off. In the Bluetooth two-part build
 * (1.04, HYB_SHIM_LEND: its ring 16-bit, HALF fills it) only while Bluetooth's memory is lent (SHX_LIVE): with
 * Bluetooth on the extension is edited, kept and saved as ever but its knobs say "SHIMMER: BT OFF" and the engine
 * has none (HYB_SHIMMER from the engine's synth.h) */
#ifdef HYB_SHIMMER
#define SHX_N HYB_SHIM_N                 /* AMOUNT, LAYERS, GRAIN, TONE; MODULATION (1.18C45: HIGH's place, kept for v2
                                          * before), WIDTH (1.18C47, EDIT > SHIMMER: DIRECT's place, always 0 before);
                                          * TAIL, SWELL (0.91, EDIT > SHIMMER); GRAIN on EDIT > SHIMMER since 1.18C45.
                                          * The values the engine takes, before the targets in an extension (x) */
#define SHX_GL SHX_N                     /* (1.18C47) W GLIDE 0..15 (the width's glide 0.5 x 2^(v / 5) s, the engine's
                                          * RVS_WT0): the panel's byte 256 + SHX_GL (past the values), in an extension
                                          * x its last byte, SHX_GX (after the targets: an extension frame of before
                                          * reads its default); the engine gets it in TAIL's byte, bits 1-4 as
                                          * v ^ SHX_GDEF (shx_tl) */
#define SHX_GDEF 5u                      /* (1 s; the engine's RVS_WGD) */
#define SHX_TAIL 6u                      /* TAIL: 0 LIMITED (the shimmer's tail coupled to the reverb's), 1 FREE (as
                                          * 0.89 / 0.90) */
#define SHX_SWELL 7u                     /* SWELL 0..127: 3 s x (v / 127)^2 (0 at once, as 0.89 / 0.90) */
#define SHX_DEFS 0u, 0u, 100u, 64u, 0u, 0u, 0u, 57u   /* as shipped: AMOUNT 0 (off), OCT, GRAIN 122 ms, TONE 3.9 kHz
                                          * (MODULATION 0, DIRECT 0), TAIL LIMITED, SWELL 57 (0.60 s). A record saved before
                                          * 0.91 reads TAIL LIMITED, SWELL 0 (shx_rec_make) */
static const uint8_t SHX_DEF[SHX_N] = {SHX_DEFS};
_Static_assert(sizeof (uint8_t[]){SHX_DEFS} == SHX_N, "a default a value of the extension");
#define SH_LEN (256u + SHX_N + 1u)       /* shadow: the patch, its extension (the values, W GLIDE) */
#ifndef HYB_SHIM_LEND
#define SHX_LIVE 1                       /* (the shimmer there: always; the Bluetooth two-part build's, below) */
#endif
/* the shimmer as a target of the matrix's nine destinations, LFO 1's and 2's assigns and LFO 3's destination (0.89):
 * the patch's own byte there is off (the matrix's and the assigns' 0; LFO 3's SHX_L3OFF), the target in the
 * extension. On the panel (shadow) such a byte holds SHX_VD + t - 1 (t 1 SHIM AMT, 2 SHIM MOD (1.18C45; SHIM TONE
 * before, the same code); 3, 4 kept): above every patch value (0..127), so a patch's own code (72+ included) never reads as one; the engine, a
 * save and the store get the patch's byte off (shx_eb) and the target in the extension */
#define SHX_NR 12u
static const uint8_t SHX_RB[SHX_NR] = {128u + 65u, 128u + 68u, 128u + 70u, 128u + 73u, 128u + 75u, 128u + 77u,
                                       128u + 104u, 128u + 107u, 128u + 110u, 128u + 79u, 128u + 81u, 128u + 11u};
#define SHX_VD 200u
#define SHX_NT 2u                        /* the targets: AMOUNT, MODULATION (1.18C45; TONE before) */
#define SHX_L3OFF 3u                     /* (LFO 3's byte on the shimmer: "DEST 3", nothing in the engine; ASSUMPTION:
                                          * the reference's "none" for LFO 3 is not known) */
static uint8_t shx_tl(const uint8_t *v, uint32_t g)   /* the engine's TAIL byte: TAIL, W GLIDE g (1.18C47) */
{
    return (uint8_t)((v[SHX_TAIL] & 1u) | ((g & 15u) ^ SHX_GDEF) << 1);
}
static uint32_t shx_eb(uint32_t b, uint32_t v)   /* panel byte b's value v as the engine and the store take it */
{
    return v >= SHX_VD ? (b == 128u + 11u ? SHX_L3OFF : 0u) : v;
}
static int shx_ri(uint32_t b)            /* byte b's place in SHX_RB, -1 none */
{
    uint32_t k;
    for (k = 0; k < SHX_NR; k++)
        if (SHX_RB[k] == b) return (int)k;
    return -1;
}
#else
#define SH_LEN 256u
#endif
_Static_assert(BANK_N <= PA_N && BANKB_N == 128u, "banks A and B: 128 slots each");
/* the first releases' index (settings_t patch8): A 0..39 (bank A's 40 then), B 40..167, the init patch 168 */
#define OLD_A_N 40u
#define OLD_INIT (OLD_A_N + 128u)
static uint32_t patch_of_old(uint32_t o) { return o < OLD_A_N ? o : o < OLD_INIT ? PB_0 + o - OLD_A_N : PATCH_INIT; }
static uint32_t old_of_patch(uint32_t p)   /* (a patch the first releases do not have: their init patch) */
{
    return p < OLD_A_N && p < BANK_N ? p : p >= PB_0 && p < PU_0 ? OLD_A_N + p - PB_0 : OLD_INIT;
}
static const uint8_t *ub_bytes(uint32_t u);
static const uint8_t *patch_bytes(uint32_t i)   /* (a user patch: read from the store into ub_cache; an empty slot:
                                                 * the init patch). Main loop only */
{
    if (i >= PU_0 && i < PATCH_INIT) return ub_bytes(i - PU_0);
    return INIT_PATCH;
}
#include "bench.h"

/* code of the panel added late (PLAY mode) goes after all the other code in the flash: the reverb's code in RAM
 * (.ram_text) calls a helper of its own that the compiler left in the flash (rt_read48) with a jump that reaches 4 MB,
 * and the app's code before it in the flash moves it out of reach (2 KB were to spare before PLAY mode, under 1 KB
 * after its changes to input and draw, 2026-10-30). The linker script
 * places the sections named *_const after all the code (in XIP's .text: executable) */
#define UI_LATE __attribute__((section(".hyb_ui_const"), noinline))
/* (1.18C33) cold UI code at -Oz (P-a; the image had ~100 B left): the frame, the knob's
 * drawing, HOME's tiles, GLO's device list, the settings' load. Never the audio path (app_render, the timer ISR,
 * dr_*, sq_*) */
#if defined(__clang__)
#define UI_SMALL __attribute__((minsize))
#else
#define UI_SMALL
#endif

/* the flash writes (the settings): the platform stops the audio while it erases, so the blocks after it find the
 * FIFO empty and count as xruns. Those are not dropouts of the sound's load: the governor and the HOME figures leave
 * out the xruns seen within STORE_QUIET_MS after a write ended (store_t, read in app_render) */
#define STORE_QUIET_MS 1000u
static volatile uint32_t store_t;        /* plat_ms() at a write's start and its end (| 1), 0 none yet */
/* the deferred saves (the patterns, the drum kit, GLO's settings: store_wait) wait for complete idle: a flash erase
 * stops the audio for a moment, a click while anything sounds. st_busy_t: plat_ms() when something last went on (the
 * transport, a voice, the drums, the output above STORE_QUIET_PK, a key, a button, a knob); STORE_IDLE_MS of nothing
 * before such a save. Explicit saves (SAVE, the sequencers' own commands) do not wait */
#define STORE_IDLE_MS 2000u
#define STORE_QUIET_PK 2653u             /* the output's peak over a block, 24 bits: -70 dBFS */
static volatile uint32_t st_busy_t PLAT_POOL;
static uint32_t store_n[2] PLAT_POOL;     /* writes asked of the store: done, skipped as unchanged (a host test's) */
static int store_same(uint32_t obj, uint32_t off, const void *src, uint32_t n, int whole)   /* 1: the store holds
                                          * these bytes there already (whole: and the object is n long), so a write
                                          * would change nothing: none (0.65, the fewest erases) */
{
    uint8_t b[64];
    uint32_t k, i, j;
    const int sz = plat_store_size(obj);
    if (sz < 0 || (whole ? (uint32_t)sz != n : (uint32_t)sz < off + n)) return 0;
    for (k = 0; k < n; k += i) {
        i = n - k < sizeof b ? n - k : (uint32_t)sizeof b;
        if (plat_store_read(obj, off + k, b, i) != (int)i) return 0;
        for (j = 0; j < i; j++)
            if (b[j] != ((const uint8_t *)src)[k + j]) return 0;
    }
    store_n[1]++;
    return 1;
}
static int store_save(uint32_t obj, const void *src, uint32_t len)   /* plat_store_save, its time kept */
{
    int r;
    if (store_same(obj, 0u, src, len, 1)) return 0;
    store_n[0]++;
    store_t = plat_ms() | 1u;
    r = plat_store_save(obj, src, len);
    store_t = plat_ms() | 1u;
    return r;
}
static int store_write(uint32_t obj, uint32_t off, const void *src, uint32_t n)   /* plat_store_write, its time kept */
{
    int r;
    if (store_same(obj, off, src, n, 0)) return 0;
    store_n[0]++;
    store_t = plat_ms() | 1u;
    r = plat_store_write(obj, off, src, n);
    store_t = plat_ms() | 1u;
    return r;
}
static int store_stage(uint32_t obj, uint32_t off, const void *src, uint32_t n)   /* plat_store_stage (it may write
                                          * the object staged before), its time kept; a part the store holds as it is
                                          * not staged (an object none of whose parts changed is not written) */
{
    int r;
    if (store_same(obj, off, src, n, 0)) return 0;
    store_n[0]++;
    store_t = plat_ms() | 1u;
    r = plat_store_stage(obj, off, src, n);
    store_t = plat_ms() | 1u;
    return r;
}
static int store_flush(void)             /* plat_store_flush, its time kept */
{
    int r;
    store_t = plat_ms() | 1u;
    r = plat_store_flush();
    store_t = plat_ms() | 1u;
    return r;
}
static int store_recent(void)            /* a flash write ended less than STORE_QUIET_MS ago (or runs) */
{
    uint32_t t = store_t;
    return t && (int32_t)(plat_ms() - t) < (int32_t)STORE_QUIET_MS;   /* (signed: t is now | 1, 1.18C46) */
}

/* ------------------------------------------------------- the user banks --- */
/* C .. H: 128 slots each, a slot a patch's 256 bytes. A bank is UB_OBJS logical sectors of the bank store
 * (bankstore.c, 1.18C30: 3840 bytes each; until 1.18C29 the platform's store objects OBJ_UB0.., which the store reads
 * as its first 27 sectors) of UB_PER slots (the ninth holds 8), slot s of a bank at byte (s % UB_PER) * 256 of its
 * sector s / UB_PER. A slot is filled when its name's first byte (240) is not 0: one never written reads as zeros,
 * and a saved name is padded with spaces. Read and written a slot at a time (bk_read, bk_stage: the store stages a
 * page at a time, so the app needs no 3840-byte buffer; the RAM is short). Main loop only */
#define UB_PER 15u
#define UB_OBJS 9u
_Static_assert(OBJ_SEQ == OBJ_UB0 + UB_BANKS_OLD * UB_OBJS && UB_OBJS * UB_PER >= UB_SLOTS, "the user banks' objects "
               "(OBJ_UB0 .. OBJ_SEQ - 1: 1.18C29's banks, the store's first sectors)");
#define BK_NLOG (UB_BANKS * UB_OBJS)
#define BK_NOLD (UB_BANKS_OLD * UB_OBJS)
#define BK_OBJ0 OBJ_UB0
#define BK_ATTR UI_LATE
#define BK_STORE_OK() plat_store_ok()    /* (a flash part not known: RAM only, every save fails, as the platform's) */
#include "bankstore.c"
_Static_assert(BK_NLOG + 1u <= BK_NPH, "the six banks and a spare in 0xA5000..0xDBFFF");
static int ub_same(uint32_t l, uint32_t off, const void *src, uint32_t n)   /* 1: the store holds these bytes there
                                          * already (a write would change nothing: none, as store_same) */
{
    uint8_t b[64];
    uint32_t k, i, j;
    for (k = 0; k < n; k += i) {
        i = n - k < sizeof b ? n - k : (uint32_t)sizeof b;
        if (bk_read(l, off + k, b, i) != 0) return 0;
        for (j = 0; j < i; j++)
            if (b[j] != ((const uint8_t *)src)[k + j]) return 0;
    }
    store_n[1]++;
    return 1;
}
static int ub_put(uint32_t l, uint32_t off, const void *src, uint32_t n)   /* bk_stage (it may write the sector
                                          * staged before), its time kept; bytes the store holds as they are not
                                          * staged (a sector none of whose bytes changed is not written) */
{
    int r;
    if (ub_same(l, off, src, n)) return 0;
    store_n[0]++;
    store_t = plat_ms() | 1u;
    r = bk_stage(l, off, src, n);
    store_t = plat_ms() | 1u;
    return r;
}
static int ub_flush(void)                /* bk_flush, its time kept */
{
    int r;
    store_t = plat_ms() | 1u;
    r = bk_flush();
    store_t = plat_ms() | 1u;
    return r;
}
static uint8_t ub_full[UB_N / 8u] PLAT_POOL;   /* bit u: slot u filled (ub_scan at power-on, kept by ub_write) */
static uint8_t ub_cache[256] PLAT_POOL;  /* one slot's bytes as stored: the patch up, if it is a user patch (em_set
                                          * tells its edits apart against them) */
static uint32_t ub_cached;               /* that slot + 1, 0 none */
#define UB_NM 8u
static struct { uint16_t u1; char nm[10]; } ub_nm[UB_NM] PLAT_POOL;   /* names read lately (the lists), slot + 1 */
static uint32_t ub_nm_next;

static uint32_t ub_obj(uint32_t u) { return u / UB_SLOTS * UB_OBJS + u % UB_SLOTS / UB_PER; }   /* (its sector) */
static uint32_t ub_off(uint32_t u) { return u % UB_SLOTS % UB_PER * 256u; }
static int ub_filled(uint32_t u) { return u < UB_N && (ub_full[u >> 3] >> (u & 7u) & 1u); }
static void ub_scan(void)                /* power-on: the store's sectors found (bk_init), which slots are filled (a
                                          * byte read a slot) */
{
    uint32_t u, k;
    bk_init();
    ub_cached = 0;
    for (k = 0; k < UB_NM; k++)
        ub_nm[k].u1 = 0;
    for (u = 0; u < UB_N; u++) {
        uint8_t c = 0;
        if (!(u & 7u)) ub_full[u >> 3] = 0;
        if (bk_ph[ub_obj(u)] && bk_read(ub_obj(u), ub_off(u) + 240u, &c, 1u) == 0 && c)
            ub_full[u >> 3] |= (uint8_t)(1u << (u & 7u));
    }
}
static const uint8_t *ub_bytes(uint32_t u)   /* slot u as stored (ub_cache), or the init patch (empty, unreadable) */
{
    if (ub_cached == u + 1u) return ub_cache;
    ub_cached = 0;
    if (!ub_filled(u) || bk_read(ub_obj(u), ub_off(u), ub_cache, 256u) != 0)
        return INIT_PATCH;
    ub_cached = u + 1u;
    return ub_cache;
}
static void ub_name(uint32_t u, char *nm)   /* slot u's name, 10 bytes (the cache, else read and kept) */
{
    uint32_t k;
    const char *q = 0;
    if (ub_cached == u + 1u) q = (const char *)ub_cache + 240;
    for (k = 0; !q && k < UB_NM; k++)
        if (ub_nm[k].u1 == u + 1u) q = ub_nm[k].nm;
    if (!q) {
        k = ub_nm_next++ % UB_NM;
        if (bk_read(ub_obj(u), ub_off(u) + 240u, ub_nm[k].nm, 10u) == 0) ub_nm[k].u1 = (uint16_t)(u + 1u);
        else { ub_nm[k].u1 = 0; ub_nm[k].nm[0] = 0; }
        q = ub_nm[k].nm;
    }
    for (k = 0; k < 10u; k++)
        nm[k] = q[k];
}
#ifdef HYB_SHIMMER
/* a user slot's extension in the store: in its bank's ninth sector past its 8 slots (a sector holds 3840 bytes, the
 * ninth 2048 of patches), SHX_REC bytes a slot: SHX_TAG, the values' count, the values. Zeros (none written, or past
 * the end of an object of the platform's that every save before 0.89 left shorter) are no extension: the defaults, shimmer off. The defaults are saved as
 * zeros (nothing written where none was: a save without the shimmer writes as 0.88 did). Main loop only.
 * 0.91: TAIL and SWELL in bits the record had free (the ninth object is full: 128 x 14 bytes past its patches):
 *   byte 1   bits 1, 2 set (the count 6 to 0.89 / 0.90, which read all six values), bit 0 TAIL (1 FREE), bits 3-7
 *            SWELL's bits 0-4
 *   byte 6, 7 (HIGH, DIRECT: 0..127; 1.18C45 byte 6 MODULATION, 1.18C47 byte 7 WIDTH) bit 7: SWELL's bits 5, 6
 * so a record of 0.89 / 0.90 (byte 1 = 6, the high bits 0) reads TAIL LIMITED, SWELL 0 (at once, as it sounded).
 * 1.18C47: W GLIDE ^ SHX_GDEF in bit 7 of bytes 2-5 (AMOUNT, LAYERS, GRAIN, TONE: 0..127, bit 7 always 0 before),
 * bits 0-3: a record of before reads W GLIDE's default */
#define SHX_REC 14u                      /* tag, the values' count, SHX_NV values, the targets SHX_NR x 4 bits */
#define SHX_NV 6u                        /* the values in their own bytes (AMOUNT .. DIRECT); TAIL, SWELL packed */
#define SHX_TAG 0x53u                    /* ("S") */
#define SHX_OFF (UB_SLOTS % UB_PER * 256u)
_Static_assert(SHX_OFF + UB_SLOTS * SHX_REC <= BK_PAY && 2u + SHX_NV + SHX_NR / 2u <= SHX_REC
               && SHX_N == SHX_NV + 2u && SHX_SWELL == SHX_N - 1u, "the slots' extensions in the ninth object");
#define SHX_GX (SHX_N + SHX_NR)          /* (1.18C47) W GLIDE's place in an extension x */
#define SHX_XN (SHX_GX + 1u)             /* an extension x: the values, the targets, W GLIDE */
static uint8_t shx_c[SHX_XN] PLAT_POOL;   /* a slot's values, targets and W GLIDE as stored (shx_cu: the slot + 1) */
static uint16_t shx_cu PLAT_POOL;
static const uint8_t SHX_DEF0[SHX_XN] = {SHX_DEFS, [SHX_GX] = SHX_GDEF};   /* (the defaults, no target) */
static uint32_t shx_obj(uint32_t u) { return u / UB_SLOTS * UB_OBJS + UB_OBJS - 1u; }   /* (its bank's ninth sector) */
static uint32_t shx_off(uint32_t u) { return SHX_OFF + u % UB_SLOTS * SHX_REC; }
static void shx_rec_read(uint32_t u, uint8_t *r)   /* slot u's record (zeros: none) */
{
    uint32_t k;
    if (bk_read(shx_obj(u), shx_off(u), r, SHX_REC) != 0)
        for (k = 0; k < SHX_REC; k++) r[k] = 0;
}
static void shx_rec_make(const uint8_t *sh, uint8_t *r)   /* the panel's patch sh (SH_LEN) as a record (the
                                          * defaults and no target: zeros) */
{
    uint32_t k, d = 1;
    for (k = 0; k < SHX_REC; k++) r[k] = 0;
    for (k = 0; k < SHX_N; k++) d &= sh[256u + k] == SHX_DEF[k];
    d &= sh[256u + SHX_GL] == SHX_GDEF;
    for (k = 0; k < SHX_NR; k++) {
        const uint32_t v = sh[SHX_RB[k]], t = v >= SHX_VD ? v - SHX_VD + 1u : 0u;
        r[2u + SHX_NV + k / 2u] |= (uint8_t)((t & 15u) << (k & 1u) * 4u);
        d &= !t;
    }
    if (d) { for (k = 0; k < SHX_REC; k++) r[k] = 0; return; }
    {
        const uint32_t sw = sh[256u + SHX_SWELL] & 127u;
        r[0] = SHX_TAG;
        r[1] = (uint8_t)(6u | (sh[256u + SHX_TAIL] & 1u) | (sw & 31u) << 3);
        for (k = 0; k < SHX_NV; k++) r[2u + k] = (uint8_t)(sh[256u + k] & 127u);
        r[2u + 4u] |= (uint8_t)((sw >> 5 & 1u) << 7);
        r[2u + 5u] |= (uint8_t)((sw >> 6 & 1u) << 7);
        for (k = 0; k < 4u; k++) r[2u + k] |= (uint8_t)((((sh[256u + SHX_GL] & 15u) ^ SHX_GDEF) >> k & 1u) << 7);
    }
}
static const uint8_t *patch_ext(uint32_t i)   /* patch i's extension as stored: its values, then its targets (0 none):
                                          * a filled user slot's, else the defaults */
{
    uint32_t k, n;
    uint8_t r[SHX_REC];
    if (i < PU_0 || i >= PATCH_INIT || !ub_filled(i - PU_0)) return SHX_DEF0;
    if (shx_cu != i - PU_0 + 1u) {
        shx_rec_read(i - PU_0, r);
        n = r[0] == SHX_TAG ? r[1] : 0u;
        for (k = 0; k < SHX_N; k++) shx_c[k] = SHX_DEF[k];
        if (n) {                         /* (TAIL, SWELL: 0.91's packed bits; a record of 0.89 / 0.90: 0, 0) */
            for (k = 0; k < SHX_NV && k < n; k++) shx_c[k] = (uint8_t)(r[2u + k] & 127u);
            shx_c[SHX_TAIL] = (uint8_t)(n & 1u);
            shx_c[SHX_SWELL] = (uint8_t)(n >> 3 | (r[2u + 4u] >> 7) << 5 | (r[2u + 5u] >> 7) << 6);
        }
        shx_c[SHX_GX] = n ? (uint8_t)((r[2] >> 7 | (r[3] >> 7) << 1 | (r[4] >> 7) << 2 | (r[5] >> 7) << 3) ^ SHX_GDEF)
                          : (uint8_t)SHX_GDEF;
        for (k = 0; k < SHX_NR; k++) shx_c[SHX_N + k] = n ? (uint8_t)(r[2u + SHX_NV + k / 2u] >> (k & 1u) * 4u & 15u) : 0u;
        shx_cu = (uint16_t)(i - PU_0 + 1u);
    }
    return shx_c;
}
static void shx_put(const uint8_t *x, uint8_t *sh)   /* an extension x (patch_ext's) onto the panel's patch sh */
{
    uint32_t k;
    for (k = 0; k < SHX_N; k++) sh[256u + k] = x[k];
    sh[256u + SHX_GL] = (uint8_t)(x[SHX_GX] & 15u);
    for (k = 0; k < SHX_NR; k++)
        if (x[SHX_N + k] >= 1u && x[SHX_N + k] <= SHX_NT) sh[SHX_RB[k]] = (uint8_t)(SHX_VD + x[SHX_N + k] - 1u);
}
static uint32_t shx_stored(uint32_t p, uint32_t b)   /* patch p's byte b as stored, as the panel shows it (its
                                          * extension's past 256; a destination on the shimmer: SHX_VD + t - 1) */
{
    int k;
    if (b >= 256u) return patch_ext(p)[b == 256u + SHX_GL ? SHX_GX : b - 256u];
    if ((k = shx_ri(b)) >= 0) {
        const uint32_t t = patch_ext(p)[SHX_N + (uint32_t)k];
        if (t >= 1u && t <= SHX_NT) return SHX_VD + t - 1u;
    }
    return patch_bytes(p)[b];
}
static int shx_none(uint32_t u, uint8_t *r)   /* slot u has a record (r: zeros, to write over it) */
{
    uint32_t k, n = 0;
    shx_rec_read(u, r);
    for (k = 0; k < SHX_REC; k++) { n |= r[k]; r[k] = 0; }
    return n != 0u;
}
static int shx_differs(uint32_t u, const uint8_t *sh, uint8_t *r)   /* the record of sh (into r) differs from slot
                                          * u's in the store */
{
    uint8_t o[SHX_REC];
    uint32_t k;
    shx_rec_make(sh, r);
    shx_rec_read(u, o);
    for (k = 0; k < SHX_REC; k++)
        if (o[k] != r[k]) return 1;
    return 0;
}
#endif
static int ub_write(uint32_t u, const uint8_t *b)   /* a patch into slot u (its 256 bytes, b in RAM; and its
                                          * extension after them, HYB_SHIMMER): 0 = done */
{
    uint32_t k;
#ifdef HYB_SHIMMER
    uint8_t x[SHX_REC];
    int r;
    const int xd = shx_differs(u, b, x);
    for (k = 0; k < 256u; k++)           /* (the patch as stored: a destination on the shimmer off, ub_cache then holds
                                          * it as the store does) */
        ub_cache[k] = (uint8_t)shx_eb(k, b[k]);
    ub_cached = 0;
    b = ub_cache;
    if (xd) {                            /* (the extension changed: both parts staged, the objects written once each,
                                          * one erase when the slot is in the ninth object) */
        r = ub_put(ub_obj(u), ub_off(u), b, 256u);
        if (!r) r = ub_put(shx_obj(u), shx_off(u), x, SHX_REC);
        if (ub_flush() != 0) r = -1;
    } else {
        r = ub_put(ub_obj(u), ub_off(u), b, 256u);
        if (ub_flush() != 0) r = -1;
    }
    shx_cu = 0;
#else
    int r = ub_put(ub_obj(u), ub_off(u), b, 256u);
    if (ub_flush() != 0) r = -1;
#endif
    for (k = 0; k < UB_NM; k++)
        if (ub_nm[k].u1 == u + 1u) ub_nm[k].u1 = 0;
    ub_cached = 0;
    if (r) return r;
    ub_full[u >> 3] |= (uint8_t)(1u << (u & 7u));
    for (k = 0; k < 256u; k++)
        ub_cache[k] = b[k];
    ub_cached = u + 1u;
    return 0;
}


static int ub_stage(uint32_t u)          /* ub_cache (a patch from SysEx) into slot u, staged: its sector is written
                                          * when the next one is staged, a slot before it, or at ub_flush (one erase
                                          * for its 15 slots in order). 0 = done */
{
    uint32_t k;
    int r = ub_put(ub_obj(u), ub_off(u), ub_cache, 256u);
    for (k = 0; k < UB_NM; k++)
        if (ub_nm[k].u1 == u + 1u) ub_nm[k].u1 = 0;
    ub_cached = 0;
#ifdef HYB_SHIMMER
    shx_cu = 0;                          /* (its extension goes at the upload's end: bu_ext_off) */
#endif
    if (r) return r;
    ub_full[u >> 3] |= (uint8_t)(1u << (u & 7u));
    ub_cached = u + 1u;
    return 0;
}

static int patch_filled(uint32_t p)      /* a patch to load: A's filled slots, B, a filled user slot, the init patch */
{
    return p < PU_0 ? 0 : p < PATCH_INIT ? ub_filled(p - PU_0) : p == PATCH_INIT;
}
/* ------------------------------------------------------------- settings --- */
#define SET_MAGIC 0x48594231u            /* "HYB1" */
/* a step of the sequencer (sq_*): ev bit 7 on, bits 0-3 the event (SQ_POLY, shown FINGER POLY
 * since 1.02, SQ_NOTE, SQ_FING + n for finger n + 1, SQ_CHORD 1.02); off the NOTE event's offset in semitones (-24 ..
 * +24; 1.18C31 FING n's too: its finger's note transposed), a CHORD step's chord (0 .. 7, the pattern's ch; -1 none yet); la bits 0-4 the length (SQ_LEN: 0 1/64,
 * 1 1/32, 2 1/16, 3 .. 17 2 .. 16 sixteenths (up to 0.70 at most to the end of its 16-step block), 0.71: 18 .. 29
 * 20 .. 64 sixteenths in fours (a beat), tied over the blocks and the pattern's loop, at most the pattern's LENGTH),
 * bits 5-7 the align in 1/8 of a step, early by it with ev bit 4 (SQ_ALNEG, 0.78: -7/8 .. -1/8, played in the step
 * before; never with align 0; every pattern before: 0, late); vel 0 .. 127 (0: silent). A pattern: up to 64 steps (SQ_NMAX, four blocks of
 * 16, the white keys show one), the time division (0.25x .. 4x), its LENGTH (lc: 16 << lc steps played; the steps past
 * it kept), a mark and (pad, 0.71) EDIT > PATCH ARP (SQ_PARP); the current one alone in RAM (sqp, pool). Stored in
 * two parts, each read and written by itself (only what changed is written): the head (steps 1-16, the mark, the
 * time division, the length: SQ_REC bytes, the first releases' 16-step record with the length in its padding, 0 = 16,
 * so their patterns load as they were) in OBJ_SEQ, and the rest (steps 17-64 and a mark of their own: SQ_XREC bytes;
 * never written: fresh steps, off) in OBJ_SEQX + n / SQ_XPER (app.h) */
typedef struct { uint8_t ev; int8_t off; uint8_t la; uint8_t vel; } sq_step_t;
#define SQ_N 16u                         /* the steps a block (the white keys) */
#define SQ_NB 4u                         /* the blocks at most */
#define SQ_NMAX (SQ_N * SQ_NB)
#define SQ_NLC 3u                        /* LENGTH: 16, 32, 64 */
#define SQ_STEPS(lc) (SQ_N << (lc))      /* the steps played at LENGTH lc */
#define SQ_ON 0x80u
#define SQ_ALNEG 0x10u                   /* ev bit 4: ALIGN early */
#define SQ_AL(t) ((t)->ev & SQ_ALNEG ? -(int32_t)((t)->la >> 5) : (int32_t)((t)->la >> 5))   /* a step's ALIGN (1/8
                                          * steps, -7 .. 7) */
enum { SQ_POLY, SQ_NOTE, SQ_FING, SQ_CHORD = SQ_FING + 8, SQ_NEV };   /* (the stored codes; SQ_CHORD 1.02. The
                                          * EVENT list's order (1.02): CHORD, NOTE, FING 1-8, FING POLY (SQ_POLY):
                                          * SQ_EVU, its own inverse, maps a code to its place and back) */
#define SQ_OFS(ev) ((ev) == SQ_NOTE || (uint32_t)(ev) - SQ_FING < 8u)   /* (1.18C31) an event OFFSET applies to:
                                          * NOTE, FING 1-8 (not FING POLY, CHORD) */
static const uint8_t SQ_EVL[] = {SQ_CHORD, 1, 2, 3, 4, 5, 6, 7, 8, 9, SQ_POLY};
#define SQ_EVU(x) ((uint32_t)SQ_EVL[(x)])
#define SQ_CHN 8u                        /* (1.02) a pattern's chords (CHORD steps: off its index 0 .. 7; -1 none yet:
                                          * the base note alone) */
#define SQ_CHV 6u                        /* a chord's notes at most: offsets from the base, + 64 (16 .. 112: -48 .. +48),
                                          * lowest first, 0 past the last; a chord of none: free */
#define SQ_LBAR 17u                      /* the length of 16 steps (a bar at 1x) */
#define SQ_LTOP 29u                      /* the longest: 64 steps */
#define SQ_LEN(i) ((i) < 2u ? 2u << (i) : (i) <= SQ_LBAR ? ((i) - 1u) * 8u : ((i) - 13u) * 32u)   /* length i in ticks
                                          * (1/8 step) */
#define SQ_LMAXP(lc) (SQ_LBAR + ((SQ_STEPS(lc) - SQ_N) >> 2))   /* the longest a step can be set to at LENGTH lc: the
                                          * pattern's length (17, 21, 29) */
#define SQ_PARP 1u                       /* sq_pat_t pad bit 0: EDIT > PATCH ARP ON (the patch's arpeggiator not held
                                          * off while it runs: its notes feed it) */
#define SQ_NLAT 2u                       /* pad bit 1 (0.75): EDIT > KEY LATCH OFF (0, every pattern before it: ON;
                                          * sq_latch) */
#define SQ_NRQ 4u                        /* pad bit 2 (1.14): EDIT > REC QUANT OFF (0, every pattern before it: ON, the
                                          * keys recorded onto the nearest step; OFF: onto the step they fall in, their
                                          * eighth as its ALIGN, sq_rec_hit); the other bits 0 */
#define SQ_PADM (SQ_PARP | SQ_NLAT | SQ_NRQ)   /* (the pad's bits; GLO > SEQ's defaults, set.sq_def, the same) */
#define SQ_PC_SH 3u                      /* (1.18C40) set.sq_def bits 3-4: GLO > SEQ > PRECOUNT, the bars counted in before
                                          * RECORD's start (0 OFF, as shipped and in every older object; 1, 2; pc_*).
                                          * Never a pattern's pad bit (sq_defaults, sq_clr_to: SQ_PADM) */
#define SQ_PCM (3u << SQ_PC_SH)
#define SQ_PCB (set.sq_def >> SQ_PC_SH & 3u)
#define SQ_DOUT 0x20u                    /* (1.19.3) set.sq_def bit 5: GLO > SEQ > DRUM OUT, the drums' local hits out over
                                          * MIDI on DRUM CHANNEL (dr_out; 0 OFF, as shipped and in every older object) */
#define SQ_DEF0 (SQ_PARP | 2u << SQ_PC_SH)   /* (1.18C48, the user's choice) GLO > SEQ as shipped: PATCH ARP ON, PRECOUNT 2
                                          * BARS (KEY LATCH, REC QUANT ON as before). Until 1.18C47 0: PATCH ARP OFF,
                                          * PRECOUNT OFF. A settings object keeps its own (0 as shipped or chosen: it
                                          * cannot tell); one older than the setting (SET_LEN_V4K) takes these */
#define SQ_FON(pad, b) ((((pad) ^ (SQ_NLAT | SQ_NRQ)) & (b)) != 0u)   /* (the setting b of a pad byte ON: PATCH ARP
                                          * set, KEY LATCH and REC QUANT clear) */
#define SQ_DIV1 2u                       /* the time division: 0 .. 4 = 0.25x, 0.5x, 1x, 2x, 4x */
#define SQ_NDIV 5u
#define SQ_PATS 32u
#define SQ_MARK 0x53u                    /* a pattern written ('S'); else (never written, zeros): the default */
#define SQ_XMARK 0x58u                   /* its steps 17-64 written ('X'); else fresh steps */
typedef struct { sq_step_t st[SQ_NMAX]; uint8_t mark, div, lc, pad; uint8_t ch[SQ_CHN][SQ_CHV]; } sq_pat_t;   /* (mark
                                          * .. pad: the head's last 4 bytes as stored; ch, 1.02: the chords, after
                                          * the 32 heads in OBJ_SEQ, SQ_CHO) */
#define SQ_REC (SQ_N * 4u + 4u)          /* the head's bytes in OBJ_SEQ (steps 1-16, mark, div, lc, pad) */
#define SQ_XREC ((SQ_NMAX - SQ_N) * 4u + 4u)   /* the rest's in OBJ_SEQX + (steps 17-64, the mark, 3 bytes 0) */
#define SQ_XPER 16u                      /* patterns a rest object */
#define SQ_CHO (SQ_PATS * SQ_REC)        /* (1.02) the chords' place in OBJ_SEQ: pattern n's at SQ_CHO + n * SQ_CHB (an
                                          * object written before is shorter: none) */
#define SQ_CHB (SQ_CHN * SQ_CHV)
#define SQ_CH8(q) ((uint8_t *)(q)->ch)   /* (the chords as one array of SQ_CHB) */
_Static_assert(sizeof(sq_pat_t) == SQ_NMAX * 4u + 4u + SQ_CHB, "the pattern: its steps, the head's last 4 bytes, the chords");
_Static_assert(sizeof SQ_EVL == SQ_NEV, "the EVENT list: every event once");
_Static_assert(SQ_CHO + SQ_PATS * SQ_CHB <= PLAT_STORE_MAX, "the chords in OBJ_SEQ");
_Static_assert(SQ_PATS * SQ_REC <= PLAT_STORE_MAX && SQ_XPER * SQ_XREC <= PLAT_STORE_MAX
               && SQ_PATS <= SQ_XPER * (OBJ_DRUMX - OBJ_SEQX), "the patterns' objects");
#ifdef HYB_MULTI
/* two arp sequencers (the USB build): SEQ A plays part A, SEQ B part B, each its current pattern
 * (sqpa, pool) and its runtime (sqr, below); sqp and sq are the one the panel shows (sq_mi), the audio interrupt
 * plays each in turn (sq_ii: SQI, SQPI). They share the 32 patterns: one in use by the other cannot be chosen */
#define SQ_NI 2u
static sq_pat_t sqpa[SQ_NI] PLAT_POOL;   /* the current patterns (set.sq_pat, set.sqb_pat) */
static uint8_t sq_mi PLAT_POOL;                  /* main loop: the arp sequencer shown and edited (0 A, 1 B) */
static uint8_t sq_ii PLAT_POOL;                  /* audio interrupt: the one being played (sq_block) */
#define sqp sqpa[sq_mi]
#define SQPI sqpa[sq_ii]
#else
static sq_pat_t sqp PLAT_POOL;           /* the current pattern (set.sq_pat) */
#define SQPI sqp
#endif
#define SQ_V1 1u
/* the drum sequencer (dr_*): 11 instruments (DR_NI, on the black keys) of up to 64 steps. A
 * pattern: per block of 16 steps and instrument a bit a step on (on), a byte a step (st: bits 0-1 the accent 0..2,
 * bits 2-4 the length 0..7 (DR_LMUL), bits 5-7 the align in 1/8 of a step), a bit a step the align's sign (neg, 0.80:
 * set, -7/8 .. -1/8, the hit early, in the step before as the arp sequencer's SQ_ALNEG; never with align 0), the time
 * division, its LENGTH (lc, as the arp sequencer's) and a mark; the current one alone in RAM (drp, pool). Stored as
 * the arp sequencer's in two parts: the head (block 1: on, st; the mark; the time division | lc << 4: DR_REC bytes,
 * the first releases' record, their time division < 16 so lc 0) in OBJ_DRUM, the rest (blocks 2-4 and a mark:
 * DR_XREC; never written: steps off) in OBJ_DRUMX + n / DR_XPER (app.h). The signs (0.80) apart from the records,
 * whose places never change: the head's (DR_NEGH bytes) after the kit in OBJ_DRUM, the rest's (DR_NEGX) after the
 * six records of the pattern's own OBJ_DRUMX object (room for DR_XNK; the others' in the last object's free slots:
 * dr_xneg). An object written before 0.80 is shorter: its signs read 0 (late or 0, as before), and while a pattern has
 * none set they are not written there (dr_negf: no extra flash write) */
#define DR_NI 11u
#define DR_PATS 16u
#define DR_MARK 0x44u                    /* a pattern written ('D'); else (never written, zeros): the default */
#define DR_XMARK 0x58u                   /* its blocks 2-4 written ('X') */
#define DR_NACC 3u
#define DR_NLEN 8u
#define DR_LEN1 4u                       /* the length as shipped (x 1.0) */
#define DR_SB(acc, len, aln) ((uint8_t)((acc) | (len) << 2 | (aln) << 5))
typedef struct {
    uint16_t on[SQ_NB][DR_NI]; uint8_t st[SQ_NB][DR_NI][SQ_N]; uint16_t neg[SQ_NB][DR_NI]; uint8_t mark, div, lc, pad;
} dr_pat_t;
#define DR_ON(i, s) (drp.on[(s) >> 4][i] >> ((s) & 15u) & 1u)   /* instrument i's step s on */
#define DR_ST(i, s) drp.st[(s) >> 4][i][(s) & 15u]                /* its step byte */
#define DR_NEG(i, s) (drp.neg[(s) >> 4][i] >> ((s) & 15u) & 1u)  /* its align's sign (1: early) */
#define DR_AL(i, s) (DR_NEG(i, s) ? -(int32_t)(DR_ST(i, s) >> 5) : (int32_t)(DR_ST(i, s) >> 5))   /* its ALIGN (1/8
                                          * steps, -7 .. 7) */
#define DR_REC (DR_NI * 2u + DR_NI * SQ_N + 2u)                  /* the head's bytes in OBJ_DRUM */
#define DR_XREC ((SQ_NB - 1u) * (DR_NI * 2u + DR_NI * SQ_N) + 2u)   /* the rest's */
#define DR_XPER 6u
_Static_assert(DR_REC == 200u, "the first releases' record");
_Static_assert(DR_XPER * DR_XREC <= PLAT_STORE_MAX && DR_PATS <= DR_XPER * (OBJ_LSET - OBJ_DRUMX),
               "the drum patterns' objects");
_Static_assert(DR_PATS * DR_REC <= PLAT_STORE_MAX, "the drum patterns' heads: one store object");
static dr_pat_t drp PLAT_POOL;           /* the current drum pattern (set.dr_pat) */
static dr_pat_t un_shd PLAT_POOL;        /* the undo's copy of drp as last seen (un_*); after 0.99 also the drums'
                                          * spare (dr_sp): the pattern chosen while they run, loaded ahead into it and
                                          * swapped with drp by the audio interrupt on the line (dr_clk) */
static uint8_t dr_sp PLAT_POOL, dr_spn PLAT_POOL;   /* (after 0.99) the spare holds a pattern (1; 0: the undo's
                                          * copy), its number */
#define DR_V1 1u
#define DR_LEV1 60u                      /* the drums' level as shipped (0 .. DR_LMAX) */
#define DR_LCLEAN 100u                   /* LEVEL up to this: the level alone (as before); above it the drums' bus is
                                          * driven into a saturator (DRUM DRIVE, dr_drive), the level that of 100
                                          * (1.18C42: the kick alone, DR_KDRIVE) */
#define DR_LMAX 127u
/* the drum kit (GLO > DRUM KIT, kit_*): each instrument's sound, one kit for all patterns,
 * taken at each hit (dr_hit). As shipped every instrument sounds as before the kit. Stored after the patterns in
 * OBJ_DRUM (DR_KIT_OFF; an object written before it has none: the kit as shipped); in RAM only here (pool) */
#define DR_KMARK 0x4Bu                   /* the kit written ('K') */
#define DR_KV1 1u
#define DR_KN 24u                        /* TUNE, DECAY, TONE: 0 .. DR_KN, DR_K0 as shipped (the middle) */
#define DR_K0 12u
#define DR_KLEV 100u                     /* LEVEL as shipped (0 .. DR_KLMAX; the factor (lev / 100)^2) */
#define DR_KLMAX 127u
#define DR_KATT 25u                      /* ATTACK / CLICK: 0 (as shipped) .. DR_KATT */
#define DR_KPAN 50                       /* PAN: -DR_KPAN .. DR_KPAN, 0 (centred) as shipped */
typedef struct {
    uint8_t lev;                         /* LEVEL 0 .. DR_KLMAX */
    uint8_t tune;                        /* TUNE: -12 .. +12 semitones (the noise's: its filters), DR_SEMI's index */
    uint8_t dec;                         /* DECAY: x 2^((dec - 12) / 6), 0.25 .. 4 (DR_SEMI's index, squared) */
    uint8_t att;                         /* ATTACK: a fade-in of 2 ms a step; BD and the toms: CLICK, 4 % a step */
    uint8_t tone;                        /* TONE: -12 .. +12 (the instrument's brightness, dr_hit, dr_render) */
    uint8_t m909 : 1;                    /* MODEL: 0 the 808 voice (as before), 1 the 909 voice (BD SD LT MT HT) */
    uint8_t snd : 7;                     /* SEND 0 .. 127 (0.88): into part A's delay / reverb (dr_level, dr_render);
                                          * in MODEL's byte, whose bits 1-7 a kit saved before holds 0: SEND 0 */
    int8_t pan;                          /* PAN */
} dr_kp_t;
_Static_assert(sizeof(dr_kp_t) == 7u, "an instrument: the first kit's seven bytes");
typedef struct { uint8_t mark, ver; dr_kp_t k[DR_NI]; uint8_t drv; } dr_kit_t;   /* (drv: the drums' bus, in the
                                          * first kit's padding (its 0: WARM, OFF): bits 0-3 GLO > DRUM KIT > BUS DRIVE's
                                          * curve (DV_*), bits 4-5 COMP (DC_*)) */
#define DR_KIT_OFF (DR_PATS * DR_REC)
_Static_assert(DR_KIT_OFF + sizeof(dr_kit_t) <= PLAT_STORE_MAX, "the kit: after the patterns, in OBJ_DRUM");
#define DR_NEGH (DR_NI * 2u)             /* a pattern's signs of block 1 (neg[0]) in OBJ_DRUM, at DR_NEG_OFF + n * it */
#define DR_NEGX ((SQ_NB - 1u) * DR_NI * 2u)   /* of blocks 2-4 (neg[1..3]) in an OBJ_DRUMX object (dr_xneg) */
#define DR_NEG_OFF (DR_KIT_OFF + (uint32_t)sizeof(dr_kit_t))
#define DR_XNK ((PLAT_STORE_MAX - DR_XPER * DR_XREC) / DR_NEGX)   /* the patterns of an OBJ_DRUMX object whose signs it
                                          * holds after its records (n % DR_XPER below it; the others' in the last
                                          * object's free record slots) */
_Static_assert(DR_NEG_OFF + DR_PATS * DR_NEGH <= PLAT_STORE_MAX, "the signs of block 1: after the kit");
_Static_assert(DR_XNK >= DR_PATS % DR_XPER && (DR_PATS % DR_XPER) * DR_XREC + DR_PATS / DR_XPER * (DR_XPER - DR_XNK)
               * DR_NEGX <= DR_XPER * DR_XREC && DR_PATS / DR_XPER == OBJ_LSET - OBJ_DRUMX - 1u,
               "the signs of blocks 2-4: in the OBJ_DRUMX objects' room");
static dr_kit_t drk PLAT_POOL;           /* the kit */
static const dr_kp_t DR_KP0 = {DR_KLEV, DR_K0, DR_K0, 0, DR_K0, 0, 0, 0};   /* an instrument as shipped (MODEL: kit_p0) */
enum { KB_SOUND, KB_VOICE, KB_BUS, KB_N };   /* the kit page's blocks (SELECT) */
static struct { uint32_t dirty_t, ask_t, tt_t, lcd; uint8_t blk, top, ask, tt, dseq; } kv PLAT_POOL;   /* the kit's
                                          * page (V_KIT): the kit changed then (0: saved); RESET asked then (ask); a
                                          * BUS knob turned then (tt: 1 + its column; up in the strip for SQ_TT_MS);
                                          * the block K1..K4 act on (KB_*), the one on the page's first row (the
                                          * second shows top + 1); the strip as shown; opened from the drums' EDIT
                                          * list (dseq: HOME back there, 0.74) */
#define QA_SLOTS 8u                      /* the quick-access list's slots */
#define QA_EMPTY 0xFFu                   /* a slot with no module */
#ifdef HYB_SHIMMER
#define QA_NSEC 25u                      /* the blocks a slot may hold (ui.c U_NBLK; 0.84: FX's LEVEL, 23; 0.89 SHIMMER, 24) */
#else
#define QA_NSEC 24u                      /* the blocks a slot may hold (ui.c U_NBLK; 0.84: FX's LEVEL, 23) */
#endif
#define QA_V2 2u                         /* settings_t qa_ok: qa holds blocks (U_ROW); 1: the first list's twelve sections */
typedef struct {
    uint32_t magic;
    int8_t oct;                          /* -2 .. +2 */
    uint8_t patch8;                      /* the patch as the first releases number it (OLD_INIT; patch below) */
    uint8_t lat;                         /* 0 LOW, 1 MED (the reference's 2-block note latency), 2 HIGH (MED, 14 voices) */
    uint8_t ble_off;                     /* Bluetooth switched off (REC held) */
    /* (added after the first release: an object saved before has only the fields above, SET_LEN_V1 bytes) */
    uint8_t dev_ok;                      /* a Bluetooth MIDI device to reconnect to: the last one connected */
    uint8_t dev_type;                    /* its address type (plat_ble_device_info) */
    uint8_t dev_addr[6];                 /* its address */
    char dev_name[21];                   /* its name, for the screen */
    /* (added in the third release: an object saved before is SET_LEN_V1 or SET_LEN_V2 bytes; then the defaults) */
    uint8_t qa_ok;                       /* QA_V2 (1: the earlier list): qa holds the player's quick-access list */
    uint8_t qa[QA_SLOTS];                /* the quick-access slots (SELECT on HOME): a block (U_ROW), QA_EMPTY none */
    /* (added in the fourth release, GLO: an object saved before is SET_LEN_V1, _V2 or _V3 bytes; then the defaults) */
    uint8_t glo_ok;                      /* GLO_V1: the fields below hold the player's GLO settings */
    int8_t tr;                           /* TRANSPOSE: the panel's keys (and what they send), semitones -24 .. +24 */
    uint8_t midi_ch;                     /* the MIDI channel listened to: 0 any (OMNI), 1 .. 16 */
    uint8_t thru;                        /* MIDI thru: what comes in goes out to the other ports */
    uint8_t clk;                         /* CLOCK: CLK_INT, CLK_EXT (the clock and transport taken from MIDI in), CLK_MASTER
                                          * (the fourth release to the tenth: CLOCK IN, 0 off, 1 on: CLK_INT, CLK_EXT) */
    uint8_t ble_auto;                    /* back to the saved Bluetooth MIDI device by itself (auto_reconnect) */
    /* (added in the fifth release: an object saved before is SET_LEN_V4 bytes or less; then 0) */
    uint8_t key_dark;                    /* GLO's KEY LIGHTS: 0 ON, 1 OFF (the keys' lights off), 2 NIGHT (1.18C43: every
                                          * light a level down, app.c lt_frame; an older object holds 0 or 1) */
    uint8_t rv_q;                        /* GLO > AUDIO > REVERB (0.81, in the fifth release's padding: every build and
                                          * every older object, which holds 0 there, share it): RVQ_DEF the build's own
                                          * (until 1.18C30 the single build FULL, the two-part builds HALF; since then
                                          * AUTO: RVQ_DEF_Q), RVQ_FULL, RVQ_HALF, RVQ_AUTO (after 1.18C30) */
    uint8_t sq_def;                      /* (1.14, the fifth release's padding: every older object holds 0 there) GLO >
                                          * SEQ: the arp patterns' PATCH ARP, KEY LATCH, REC QUANT as a pattern is
                                          * cleared or never saved (the pattern's pad bits, SQ_PADM: 0 as shipped) */
    uint8_t tempo;                       /* (the fifth release's padding; the two-part builds, 1.18) the global tempo:
                                          * its byte (BPM - 63) + 1, 0 none saved yet (an older object holds 0 there:
                                          * then part A's patch's at power-on). Saved with GLO's settings (glo_save,
                                          * SAVE's mx_save_set); the single build keeps it as loaded */
    /* (the fifth release's object ended here: SET_LEN_V4K, its padding included) */
    /* (added in the sixth release, PLAY mode: an object saved before is SET_LEN_V4K bytes or less; then the defaults) */
    uint16_t pm_ok;                      /* PM_V1: pm holds the player's assignments */
    uint16_t pm[7];                      /* PLAY mode's controls K1..K4, SELECT, PRESET, ALGO (PC_N): a patch byte (the
                                          * parameter of the play mode's list on it, app.c pl_*), PM_DEF the default,
                                          * PM_SMART the smart choice */
    /* (added in the seventh release, the user banks: an object saved before is SET_LEN_V5 bytes or less; then patch8) */
    uint16_t patch;                      /* the patch (PATCH_N's index: A, B, C .. H, the init patch) */
    uint16_t pt_ok;                      /* PT_V2: patch holds it (PT_V1: in 1.18C29's index) */
    /* (added in the eighth release, the step sequencer: an object saved before is SET_LEN_V6 bytes or less; then the
     * first pattern) */
    uint16_t sq_ok;                      /* SQ_V1: sq_pat holds the pattern chosen */
    uint8_t sq_pat;                      /* the sequencer's pattern (0 .. SQ_PATS - 1; the patterns: OBJ_SEQ) */
    uint8_t sq_pad;                      /* (the eighth release's padding, now) SQ_DISARM / DR_DISARM: a sequencer's PATTERN
                                          * NONE (0: both have their pattern, as shipped and as before: PLAY starts both) */
    /* (added in the ninth release, the drum sequencer: an object saved before is SET_LEN_V7 bytes or less; then the
     * first pattern and the level as shipped) */
    uint16_t dr_ok;                      /* DR_V1: dr_pat and dr_lev hold the player's */
    uint8_t dr_pat;                      /* the drum sequencer's pattern (0 .. DR_PATS - 1; the patterns: OBJ_DRUM) */
    uint8_t dr_lev;                      /* the drums' level, 0 .. DR_LMAX (above DR_LCLEAN: the drive; 0.56 and before:
                                          * 0 .. 100) */
    /* (added in the tenth release, the synth's volume: an object saved before is SET_LEN_V8 bytes or less; then full
     * volume, CC 7 on it) */
    uint16_t vo_ok;                      /* VO_V1: vol and vol_cc hold the player's */
    uint8_t vol;                         /* the synth's volume (GLO > AUDIO > SYNTH VOL), 0 .. 127; CC 7 sets it too */
    uint8_t vol_cc;                      /* CC 7: 1 the synth's volume (vol), 0 the engine's part volume (as shipped
                                          * by the reference's map: a patch byte) */
    /* (added in the eleventh release, MIDI in and out, the clock's modes, the arp sequencer's base note: an object
     * saved before is SET_LEN_V9 bytes or less; then the defaults, the clock's mode kept from its CLOCK IN) */
    uint8_t md_ok;                       /* MD_V1: the fields below hold the player's */
    uint8_t out_ch;                      /* OUT CHANNEL, 0 .. 15 (channel 1 .. 16): what the app sends */
    uint8_t out_to;                      /* OUT PORTS: OP_ALL (USB and Bluetooth), OP_USB, OP_BT, OP_OFF */
    uint8_t in_off;                      /* IN PORTS turned off, a bit a PORT_* (1 << PORT_USB, PORT_BLE, PORT_TRS) */
    uint8_t clk_src;                     /* CLOCK SOURCE (CLK_EXT): CS_ANY, CS_USB, CS_TRS, CS_BT */
    uint8_t sq_base;                     /* the arp sequencer's BASE NOTE (nothing held): 0 OFF, else the note */
    uint8_t eng;                         /* GLO > AUDIO > ENGINE: EN_ALL, EN_DRUMS, EN_MIDI (in the eleventh release's
                                          * padding: its objects hold 0 there, EN_ALL; no new release needed) */
    uint8_t md_pad;                      /* MD_AUTO (0.65, both builds): GLO > AUTO SAVE on (the eleventh release's
                                          * padding: older objects hold 0, OFF); the Bluetooth build: AUDIO's unison
                                          * settings in the other bits (MD_UNI_*: 0 OFF, OFF, OFF, NOTES) */
#ifdef HYB_MULTI
    /* (the USB build's two parts: added after the eleventh release's fields; an object saved before, or
     * by the Bluetooth build (which loads its first SET_LEN_V10 bytes and saves those), is SET_LEN_V10 bytes or less:
     * then the defaults) */
    uint8_t mx_ok;                       /* MX_V1: the fields below hold the player's */
    uint8_t mx_mode;                     /* GLO > MULTI > MODE: SY_M_SINGLE, _LAYER, _SPLIT, _CHANNELS */
    uint8_t mx_split;                    /* SPLIT: part B below this key, A from it up */
    uint8_t mx_chb;                      /* part B's MIDI channel: 0 none, 1 .. 16 */
    uint8_t mx_nva;                      /* VOICES: part A's of the 16 (1 .. 15), B the rest */
    uint8_t mx_send;                     /* B SEND into A's delay / reverb: 0 .. 127, MX_SEND_PATCH B's patch's */
    uint16_t mx_patch;                   /* part B's patch (PATCH_N's index), kept by SAVE as set.patch is */
    uint8_t mx_dyn;                      /* MULTI > ALLOC: 0 FIXED (each part its share), 1 DYNAMIC (one pool) */
    uint8_t mx_maxv;                     /* AUDIO > MAX VOICES: both parts' voices at most, 4 .. the latency's 16 */
    uint8_t mx_uni, mx_unim;             /* AUDIO > UNISON MAX (0 OFF, 1 .. 3; 1.18C48: UNI MAX SINGLE, 4 with mx_unx's
                                          * bit 3) and MONO UNI MAX (0 OFF, 1 .. 5; 6 with mx_unx's bit 4) */
    uint8_t mx_uauto;                    /* AUDIO > UNISON AUTO: the poly cap steps down as a chord grows */
    uint8_t mx_steal;                    /* AUDIO > STEAL: 0 NOTES, 1 UNISON FIRST (a held note's extra unison voice) */
    /* (SEQ B, the second arp sequencer: added in the USB build's second release; an object saved before is
     * SET_LEN_V11 bytes or less: then the defaults, PATTERN NONE) */
    uint8_t sqb_ok;                      /* SQB_V1: the fields below hold the player's */
    uint8_t sqb_pat;                     /* SEQ B's pattern (0 .. SQ_PATS - 1; never SEQ A's while both have one) */
    uint8_t sqb_pad;                     /* SQ_DISARM: SEQ B's PATTERN NONE (sqb_pat kept) */
    uint8_t sqb_base;                    /* SEQ B's BASE NOTE: 0 OFF, else the note */
    uint8_t mx_vcc[2];                   /* MULTI > VOL A CC, VOL B CC (0.60, in SEQ B's record's padding: an older
                                          * object holds 0 there): 0 as shipped (PV_CC0), 1 .. 120 controller 0 .. 119,
                                          * PV_CC_OFF none */
    /* (1.18C10, the pattern launcher, ln_*: added after SEQ B's record; an object saved before is SET_LEN_V12 bytes
     * or less: the table empty. Written by an explicit SAVE alone, ln_save) */
    uint8_t ln_ok;                       /* LN_V1 / LN_V2: ln holds the player's table */
    uint8_t ln_pad[3];
    uint8_t ln[16][4];                   /* LN_ROWS rows: SYNTH 1's, SYNTH 2's arp pattern, the drum pattern (0 none,
                                          * 1 + n pattern n), the row's mode (LN_REP ..) */
    /* (1.18C22, GLO > AUDIO > OSC: added after the launcher's table; an object saved before is SET_LEN_V13 bytes or
     * less: ORIGINAL) */
    uint8_t osc_ok;                      /* OSC_V1: osc_cl holds the player's */
    uint8_t osc_cl;                      /* OSC: 0 ORIGINAL (as measured, the aliasing too), 1 CLEAN (less aliasing) */
    uint8_t flt_sm;                      /* (after 1.18C34, in OSC's padding: 1.18C22 .. C34 saved it 0, ORIGINAL, and load
                                          * it as 0) FILT KNOBS: 0 ORIGINAL (a new cutoff / resonance value steps), 1
                                          * SMOOTH (it glides: synth.c flt_glide) */
    uint8_t ck_fl;                       /* (1.19.5, in OSC's padding: 1.18C22 .. 1.19.4 saved 0 there and clear it at
                                          * load) CK_FL_UNLOCK: GLO > MIDI > CLK LOCK OFF (0: ON, an older object's) */
    /* (1.18C26, GLO > VELOCITY: added after OSC; an object saved before is SET_LEN_V14 bytes or less: the defaults,
     * velocity on, NORMAL, SENSITIVITY 0, DRUM ACC RAW, the top row with the row's ends) */
    uint8_t ve_ok;                       /* VE_V1: the fields below hold the player's */
    uint8_t ve_fl;                       /* VE_OFF: VELOCITY OFF (every key plays ve_fix); VE_DRUM: DRUM ACC VELOCITY */
    uint8_t ve_fix;                      /* FIXED: the velocity with VELOCITY OFF, 1 .. 127 */
    uint8_t ve_crv;                      /* CURVE: VC_SOFT2 .. VC_HARD2 */
    int8_t ve_sens;                      /* SENSITIVITY: -5 .. +5 (touch_curve's other way round: + louder) */
    uint8_t ve_lo[7], ve_hi[7];          /* CALIBRATE: the top row's keys' own ends (the probe's peak for 1 and for 127,
                                          * touch_top_ends); ve_hi 0: none, the row's */
    int8_t ck_ofs;                       /* (1.19.4, in VELOCITY's padding: 1.18C26 .. 1.19.3 saved 0 there and clear it
                                          * at load) GLO > MIDI > CLK OFFSET, -50 .. +50 ms (ck_shift) */
    /* (1.18C31, GLO > MIDI > DRUM CHANNEL: added after VELOCITY; an object saved before is SET_LEN_V15 bytes or less:
     * channel 10. An older firmware loads its first SET_LEN_V15 bytes, plat_store_load's max, and saves those) */
    uint8_t dc_ok;                       /* DC_V1: dr_ch holds the player's */
    uint8_t dr_ch;                       /* DRUM CHANNEL: 0 OFF, 1 .. 16 (the drums' MIDI notes in: dr_mi_in) */
    uint8_t mx_unx;                      /* (1.18C48, in DRUM CHANNEL's padding: 1.18C31 .. C47 saved 0 there and clear it at
                                          * load) GLO > AUDIO's unison caps past the older fields' ranges: bits 0-2 UNI MAX
                                          * MULTI + 1 (0: as shipped, MX_UNIL_DEF), bit 3 UNI MAX SINGLE 4 (mx_uni holds 3,
                                          * which an older firmware reads), bit 4 MONO UNI MAX 6 (mx_unim holds 5) */
    uint8_t mo;                          /* (1.19.4, in DRUM CHANNEL's padding: 1.18C31 .. 1.19.3 saved 0 there and clear
                                          * it at load) the parts' MIDI out and the new IN values: bits 0-4 B OUT (MO_B:
                                          * 1 OFF, 2 .. 17 channel 1 .. 16; 0: an older object's, then B CHANNEL's
                                          * channel, OFF: OUT CHANNEL's, as 1.19.3 sent), MO_AOFF A OUT OFF (out_ch
                                          * kept), MO_AINOFF A IN OFF (midi_ch 0, OMNI to an older firmware), MO_BOMNI
                                          * B IN OMNI (mx_chb 0, OFF to an older firmware) */
    /* (1.19.6, the MIDI CC map: after DRUM CHANNEL, where 1.19.2 .. 1.19.5 kept PLAY mode's controls' CCs (pc_ok
     * PC_V1, then pm_cc[7] in cm[0..6]: converted at load, ve_check). 1.19.5 loads the first SET_LEN_V17 bytes,
     * plat_store_load's max: cm_ok is not its PC_V1, so every control OFF there) */
    uint8_t cm_ok;                       /* CM_V1: cm holds the map */
    uint8_t cm[128];                     /* incoming controller c: 0 its own line (the fixed map), 1 + the line it was
                                          * given to (its patch byte, < CM_NT), 0xFF none (its line moved away) */
    uint8_t cm_show;                     /* (GLO > MIDI > CC SHOW, in the map's padding: 1.19.6 .. saved 0 there, OFF)
                                          * a fixed map controller's move says its parameter and value */
    uint8_t cm_pad[2];
    /* (1.19.7: VOL A / B after the map; an object saved before is SET_LEN_V18 bytes or less: as shipped. 1.19.6
     * loads the first SET_LEN_V18 bytes (its object's size) and keeps everything else; saving, it writes 344 B, and
     * 1.19.7 then has both at PV_UNITY) */
    uint8_t pv_ok;                       /* PV_V1: pv_st holds the player's */
    uint8_t pv_st[2];                    /* VOL A, VOL B (pv_vol) as saved, + 1 */
    uint8_t pv_pad;
#endif
} settings_t;
#define SET_LEN_V1 8                     /* the first release's settings: magic, oct, patch, lat, ble_off */
#define SET_LEN_V2 40                    /* the second release's: the device too (its size, padding included) */
#define SET_LEN_V3 48                    /* the third release's: the quick-access list too (padding included) */
#define SET_LEN_V4 52                    /* the fourth release's: GLO too */
#define SET_LEN_V4K 56                   /* the fifth release's: the key lights too (padding included) */
#define SET_LEN_V5 72                    /* the sixth release's: PLAY mode's assignments too */
#define SET_LEN_V6 76                    /* the seventh release's: the patch over all banks too */
#define SET_LEN_V7 80                    /* the eighth release's: the sequencer's pattern number too */
#define SET_LEN_V8 84                    /* the ninth release's: the drum sequencer's pattern and level too */
#define SET_LEN_V9 88                    /* the tenth release's: the synth's volume too */
#define SET_LEN_V10 96                   /* the eleventh release's: MIDI in and out, the clock, the base note too */
#define MD_V1 1u
#define MD_AUTO 1u                       /* settings_t md_pad: AUTO SAVE on (store_wait) */
#if FM1B_BLE && !defined(HYB_MULTI)     /* (the Bluetooth build: GLO > AUDIO's unison settings, md_pad's bits 1-7; 0
                                          * as an older object holds: each OFF, STEAL NOTES, nothing changed) */
#define MD_UNI_SH 1u                     /* UNISON MAX: 0 OFF, 1 .. 3 (2 bits) */
#define MD_UNIM_SH 3u                    /* MONO UNI MAX: 0 OFF, 1 .. 5 (3 bits) */
#define MD_UAUTO 0x40u                   /* UNISON AUTO on */
#define MD_STEAL 0x80u                   /* STEAL: UNISON FIRST */
#define SET_UNI (set.md_pad >> MD_UNI_SH & 3u)
#define SET_UNIM (set.md_pad >> MD_UNIM_SH & 7u)
#define SET_UAUTO (set.md_pad >> 6 & 1u)
#define SET_STEAL (set.md_pad >> 7 & 1u)
#endif
#ifdef HYB_MULTI
#define SET_LEN_V11 112                  /* the USB build's: the two parts too (padding included) */
#define SET_LEN_V12 116                  /* its second release's: SEQ B too */
#define SET_LEN_V13 184                  /* 1.18C10's: the pattern launcher's table too */
#define SET_LEN_V14 188                  /* 1.18C22's: GLO > AUDIO > OSC too */
#define SET_LEN_V15 208                  /* 1.18C26's: GLO > VELOCITY too */
#define SET_LEN_V16 212                  /* 1.18C31's: GLO > MIDI > DRUM CHANNEL too */
#define SET_LEN_V17 220                  /* 1.19.2's: PLAY mode's controls' MIDI CCs too */
#define SET_LEN_V18 344                  /* 1.19.6's: the MIDI CC map instead */
#define SET_LEN_V19 348                  /* 1.19.7's: VOL A / B too */
#define PV_V1 1u
#define CM_V1 2u
#define CM_NT 254u                       /* the map's lines: 0 .. 119 the controllers' bytes, 128 .. 253 the parameters
                                          * on the patch's later bytes (no number of their own) */
#define PC_V1 1u
#define MO_B 31u                         /* settings_t mo: B OUT (1.19.4) */
#define MO_AOFF 32u                      /* A OUT OFF */
#define MO_AINOFF 64u                    /* A IN OFF */
#define MO_BOMNI 128u                    /* B IN OMNI */
#define CK_OFS_MAX 50                    /* CLK OFFSET's range, ms */
#define CK_FL_UNLOCK 1u                  /* settings_t ck_fl: CLK LOCK OFF (1.19.5) */
#define DC_V1 1u
#define DR_CH_DEF 10u                    /* DRUM CHANNEL as shipped (GM's drum channel) */
#define OSC_V1 1u
#define VE_V1 1u
#define LN_V1 1u                         /* (the table before 1.18C18: modes REP, 1x .. 8x, NO = 0 .. 5, an empty row's default REP) */
#define LN_V2 2u                         /* (1.18C18: modes NO, REP, 1x .. 8x = 0 .. 5: a new row is NO) */
#define MX_V1 1u
#define MX_UNX_L 7u                      /* (1.18C48) settings_t mx_unx: UNI MAX MULTI + 1 (0 as shipped) */
#define MX_UNX_S4 8u                     /* UNI MAX SINGLE 4 (mx_uni 3) */
#define MX_UNX_M6 16u                    /* MONO UNI MAX 6 (mx_unim 5) */
#define MX_UNIL_DEF 2u                   /* UNI MAX MULTI as shipped (1.18C48, the user's choice) */
#define SET_UNIS (set.mx_uni + (set.mx_unx >> 3 & 1u))   /* UNI MAX SINGLE: 0 OFF, 1 .. 4 */
#define SET_UNIL ((set.mx_unx & MX_UNX_L) ? (set.mx_unx & MX_UNX_L) - 1u : MX_UNIL_DEF)   /* UNI MAX MULTI: 0 OFF, 1 .. 4 */
#define SET_UNI (set.mx_mode == SY_M_SINGLE ? SET_UNIS : SET_UNIL)   /* (AUDIO's unison settings: the Bluetooth build's
                                          * in md_pad; 1.18C48: the poly cap in force, SINGLE's or the multi modes') */
#define SET_UNIM (set.mx_unim + (set.mx_unx >> 4 & 1u))   /* MONO UNI MAX: 0 OFF, 1 .. 6 */
#ifdef HYB_MULTI
#define PV_CC0 103u                      /* VOL A CC as shipped (VOL B CC the next): controllers no page names (each
                                          * would set a patch byte no page shows; MIDI's "undefined" range) */
#define PV_CC_OFF 121u                   /* settings_t mx_vcc: none */
#define PV_UNITY 100u                    /* a part's volume as shipped: x 1 */
static uint8_t pv_vol[2] = {PV_UNITY, PV_UNITY};   /* the parts' volumes (SHIFT + ALGO, VOL A / B CC), 0 .. 127, the
                                          * gain (v / 100)^2 after the part's distortion (synth.h rt_g); 1.19.7:
                                          * saved with GLO's settings (set.pv_st) */
#endif
#define SQB_V1 1u
#define MX_SEND_PATCH 128u
#endif
enum { CLK_INT, CLK_EXT, CLK_MASTER, CLK_N };   /* settings_t clk (CLK_INT, CLK_EXT: the old CLOCK IN's 0, 1) */
enum { OP_ALL, OP_USB, OP_BT, OP_OFF, OP_N };   /* settings_t out_to (the TRS jack has no MIDI out) */
enum { CS_ANY, CS_USB, CS_TRS, CS_BT, CS_N };   /* settings_t clk_src */
enum { EN_ALL, EN_DRUMS, EN_MIDI, EN_N };   /* settings_t eng: SYNTH+DRUMS, DRUMS ONLY, MIDI ONLY (app_render eng_*) */
#define SQ_BASE1 60u                     /* the base note as shipped (C4) */
#define SQ_BASE_LO 24u                   /* its range: C1 .. C7 (0: OFF) */
#define SQ_BASE_HI 96u
#define VO_V1 1u
#define SQ_DISARM 1u                     /* settings_t sq_pad: the arp sequencer's PATTERN NONE (sq_pat kept) */
#define DR_DISARM 2u                     /* the drum sequencer's PATTERN NONE (dr_pat kept) */
#define PT_V1 1u                         /* settings_t pt_ok: patch (and mx_patch) in 1.18C29's index (three user banks:
                                          * the init patch OLD_PATCH_INIT) */
#define PT_V2 2u                         /* (1.18C30) in this one (six user banks, the init patch PATCH_INIT) */
#define PM_V1 1u
#define PM_DEF 0x100u                    /* settings_t pm: the control's default */
#define PM_SMART 0x101u                  /* settings_t pm: the smart choice (play_smart) */
#define GLO_V1 1u
enum { RVQ_DEF, RVQ_FULL, RVQ_HALF, RVQ_AUTO, RVQ_N };   /* settings_t rv_q (RVQ_AUTO: after 1.18C30; an older
                                          * firmware takes a value past its RVQ_N as RVQ_DEF, settings_load) */
#ifndef RVQ_DEF_Q
#ifdef RVQ_HALF_DEF                      /* (a measurement's or a host test's build may set it: its own default HALF
                                          * (1) or FULL (0), as before AUTO) */
#define RVQ_DEF_Q (RVQ_HALF_DEF ? RVQ_HALF : RVQ_FULL)
#elif defined(HYB_MULTI) && FM1B_BLE
#define RVQ_DEF_Q RVQ_HALF               /* (1.18C42, the user's choice) REVERB as shipped in the Bluetooth two-part build:
                                          * HALF (HALF32: the float ring since 1.18C41, cheaper than FULL). A setting
                                          * saved as "as shipped" (RVQ_DEF, never turned) takes it; a FULL, HALF or
                                          * AUTO chosen stays */
#else
#define RVQ_DEF_Q RVQ_AUTO               /* REVERB as shipped (after 1.18C30): AUTO, FULL while the load allows it
                                          * (rvq_tick); the Bluetooth two-part build: HALF since 1.18C42 (above). Until
                                          * 1.18C30: HALF in the Bluetooth two-part build (CPU; its FULL is the 16-bit
                                          * ring, or the lent float ring), FULL in the single build and the USB
                                          * two-part build. A FULL or HALF chosen stays */
#endif
#endif
#ifdef APP_HOST_CMD
static uint8_t host_old_def;             /* (1.18C42, the host simulator's "glold": REVERB and LATENCY as shipped before
                                          * 1.18C42, AUTO and LOW, for the scripts written against them) */
static uint8_t host_def47;               /* (1.18C48, the host's "gl47": GLO's and the kit's defaults as shipped until
                                          * 1.18C47 (one UNISON MAX 2, MONO UNI MAX 5, PATCH ARP and PRECOUNT OFF, the kit
                                          * all 808 / SYN), for the scripts written against them */
#define DEF47 host_def47
#define RVQ_DEF_NOW (host_old_def ? RVQ_AUTO : RVQ_DEF_Q)
#else
#ifdef HYB_TEST_DEF47
#define DEF47 1                          /* (1.18C48, a test build: the defaults until 1.18C47, the emulator's null test) */
#else
#define DEF47 0
#endif
#define RVQ_DEF_NOW RVQ_DEF_Q
#endif
#define RVQ_Q(q) ((q) == RVQ_DEF || (q) >= RVQ_N ? RVQ_DEF_NOW : (q))   /* the mode in force (RVQ_FULL, _HALF, _AUTO) */
void hyb_rv_set_full(int full);          /* (the engine, sy_cln_rv.c: REVERB to the reverb, taken between its blocks) */
#ifdef HYB_SHIMMER
void hyb_rv_shim(const uint8_t *v);      /* (the engine: FX > SHIMMER's values, SHX_N; AMOUNT > 0 runs the reverb HALF) */
#endif
#ifdef HYB_RV_LEND
void hyb_rv_lend(float *ring);            /* (the engine: REVERB FULL on this float ring of 128 KB, Bluetooth's memory) */
#endif
#define TR_MAX 24                        /* TRANSPOSE's range, semitones each way */
_Static_assert(__builtin_offsetof(settings_t, dev_ok) == SET_LEN_V1, "the first release's settings come first");
_Static_assert(__builtin_offsetof(settings_t, qa_ok) <= SET_LEN_V2 && __builtin_offsetof(settings_t, glo_ok) > SET_LEN_V2,
               "the second release's settings come first");
#define KL_ON 0u                         /* set.key_dark: KEY LIGHTS ON, OFF (1, as before 1.18C43), NIGHT */
#define KL_OFF 1u
#define KL_NIGHT 2u
#define KEYS_DARK (set.key_dark == KL_OFF)
_Static_assert(__builtin_offsetof(settings_t, glo_ok) <= SET_LEN_V3 && __builtin_offsetof(settings_t, key_dark) >= SET_LEN_V3,
               "the third release's settings come first");
_Static_assert(__builtin_offsetof(settings_t, key_dark) >= SET_LEN_V4, "the fourth release's settings come first");
_Static_assert(__builtin_offsetof(settings_t, pm_ok) >= SET_LEN_V4K, "the fifth release's settings come first");
_Static_assert(__builtin_offsetof(settings_t, patch) >= SET_LEN_V5, "the sixth release's settings come first");
_Static_assert(__builtin_offsetof(settings_t, sq_ok) >= SET_LEN_V6, "the seventh release's settings come first");
_Static_assert(__builtin_offsetof(settings_t, dr_ok) >= SET_LEN_V7, "the eighth release's settings come first");
_Static_assert(__builtin_offsetof(settings_t, vo_ok) >= SET_LEN_V8, "the ninth release's settings come first");
_Static_assert(__builtin_offsetof(settings_t, md_ok) >= SET_LEN_V9, "the tenth release's settings come first");
#ifdef HYB_MULTI
_Static_assert(__builtin_offsetof(settings_t, mx_ok) == SET_LEN_V10 && __builtin_offsetof(settings_t, sqb_pad) == SET_LEN_V11
               && __builtin_offsetof(settings_t, ln_ok) == SET_LEN_V12 && __builtin_offsetof(settings_t, osc_ok) == SET_LEN_V13
               && __builtin_offsetof(settings_t, ve_ok) == SET_LEN_V14 && __builtin_offsetof(settings_t, dc_ok) == SET_LEN_V15
               && __builtin_offsetof(settings_t, cm_ok) == SET_LEN_V16 && __builtin_offsetof(settings_t, pv_ok) == SET_LEN_V18
               && sizeof(settings_t) == SET_LEN_V19,
               "the two parts' settings after the eleventh release's, SEQ B's after them, the launcher's, OSC's, VELOCITY's, "
               "DRUM CHANNEL's, the MIDI CC map last");
_Static_assert(__builtin_offsetof(settings_t, flt_sm) == SET_LEN_V13 + 2u, "FILT KNOBS in OSC's padding (its byte 186, "
               "0 in every object 1.18C22 .. C34 saved)");
#else
_Static_assert(sizeof(settings_t) == SET_LEN_V10, "the eleventh release's settings: SET_LEN_V10 bytes");
#endif
static settings_t set;
#ifdef HYB_MULTI
static UI_LATE uint32_t pv_cc_of(uint32_t p)   /* part p's volume controller (VOL A / B CC), 128 none */
{
    const uint32_t b = set.mx_vcc[p];
    return !b ? PV_CC0 + p : b >= PV_CC_OFF ? 128u : b - 1u;
}
static UI_LATE uint32_t pv_saved(uint32_t p)   /* (1.19.7) part p's volume as saved */
{
    const uint32_t v = set.pv_st[p];
    return v && v <= 128u ? v - 1u : PV_UNITY;
}
static UI_LATE void pv_keep(void)        /* (1.19.7) the volumes now into set, to be saved */
{
    set.pv_st[0] = (uint8_t)(pv_vol[0] + 1u);
    set.pv_st[1] = (uint8_t)(pv_vol[1] + 1u);
}
static UI_LATE int pv_cc(uint32_t c, uint32_t v)   /* audio interrupt: controller c in, value v; 1 = a part's volume
                                          * (taken: not to the engine, where it would set a patch byte) */
{
    uint32_t p, t = 0;
    for (p = 0; p < 2u; p++)
        if (pv_cc_of(p) == c) { pv_vol[p] = (uint8_t)v; t = 1; }
    return (int)t;
}
/* (1.19.6) the MIDI CC map (GLO > MIDI > MIDI CC, GLO's settings): each line of the fixed map (a controller's patch
 * byte, named by the panel's parameter on it) can take another incoming number, by hand or learnt (cm_rec, cm_run);
 * one number, one line: a number given to a line is taken from the line that had it, which goes back to its own
 * number if that is free, else has none ("--"). A line's number moved: the CC acts through edit_set on the part edited
 * (panel, engine, undo, edit memory: the display follows, SAVE stores it), v x max / 127; its old number goes nowhere.
 * Lines kept: the engine's fixed map as before. Never mapped (pm_cc_bad): 1, 64, 120, 123, the VOL A / B CCs, 7 while
 * GLO > MIDI > CC 7 takes it */
static volatile uint16_t cm_q[8];        /* audio interrupt -> main loop: (line << 7 | value) */
static volatile uint8_t cm_h, cm_t;
static volatile uint8_t cm_arm;          /* learn armed (cm_run: a line waiting) */
#define FM1CP_FN UI_LATE
#define FM1CP_NO_STEP                    /* (app_say's strip shows it and times it out) */
#include "control_popup.c"               /* CC SHOW: fm1-waveloop-kits' control popup (GPL-3.0-only) */
static fm1cp cc_pop = {.min_ms = 40u};   /* audio interrupt -> main loop: the newest fixed map controller moved
                                          * (cp_run), at most one message each 40 ms */
static volatile uint8_t cm_lrn;          /* the controller learn took, | 0x80 */
static uint8_t cm_ed, cm_v;              /* the list: 1 a line's number changed (cm_v, 128 none; learn armed), 2 RESET
                                          * ALL asked */
static UI_LATE int pm_cc_bad(uint32_t c)
{
    return c == 1u || c == 64u || c == 120u || c == 123u || c == pv_cc_of(0) || c == pv_cc_of(1) || (c == 7u && set.vol_cc);
}
static UI_LATE uint32_t cm_tgt(const uint8_t *m, uint32_t c)   /* incoming c's line, 255 none */
{
    const uint32_t v = m[c];
    return !v ? c : v == 0xFFu ? 255u : v - 1u;
}
static UI_LATE uint32_t cm_num(const uint8_t *m, uint32_t t)   /* line t's number, 128 none */
{
    uint32_t c = 0;
    while (c < 128u && cm_tgt(m, c) != t) c++;
    return c;
}
static UI_LATE uint32_t cm_put(uint8_t *m, uint32_t n, uint32_t t)   /* line t to number n (128: none); the line that
                                          * lost n and has no number now, 255 none */
{
    uint32_t o, u = 255u;
    if ((t >= 120u && t < 128u) || t >= CM_NT || (n < 128u && pm_cc_bad(n))) return 255u;
    o = cm_num(m, t);
    if (o < 128u) m[o] = 0xFFu;
    if (n < 128u) { u = cm_tgt(m, n); m[n] = (uint8_t)(n == t ? 0u : t + 1u); }
    if (o < 128u && o != n && o != t && cm_num(m, o) >= 128u) m[o] = 0u;   /* (its old number: back to its own line) */
    if (u < 128u && m[u] == 0xFFu) m[u] = 0u;   /* (the line that lost it: its own number, if free) */
    return u < CM_NT && cm_num(m, u) >= 128u ? u : 255u;
}
static UI_LATE int cm_in(uint32_t c, uint32_t v)   /* audio interrupt: controller c, value v; 1 = taken */
{
    uint32_t t, h, f;
    if (pm_cc_bad(c)) return 0;
    if (cm_arm) { cm_lrn = (uint8_t)(c | 0x80u); return 1; }   /* (learning: the number, nothing set) */
    if (!(t = set.cm[c])) return 0;      /* (its own line: the engine's fixed map) */
    if (t != 0xFFu) {                    /* (queued; full: the newest replaced) */
        h = cm_h;
        f = ((h + 1u) & 7u) == cm_t;
        if (f) h = (h - 1u) & 7u;
        cm_q[h] = (uint16_t)((t - 1u) << 7 | v);
        if (!f) cm_h = (uint8_t)((h + 1u) & 7u);
    }
    return 1;
}
static UI_LATE uint32_t cm_mig(uint32_t code, uint32_t k);   /* (a 1.19.2 .. 1.19.5 PLAY mode control's line) */
#endif
/* the arp sequencer shown's settings (two in the USB build: sq_mi's) */
#ifdef HYB_MULTI
#define SQ_PATV (*(sq_mi ? &set.sqb_pat : &set.sq_pat))
#define SQ_PADV (*(sq_mi ? &set.sqb_pad : &set.sq_pad))
#define SQ_BASEV (*(sq_mi ? &set.sqb_base : &set.sq_base))
#else
#define SQ_PATV set.sq_pat
#define SQ_PADV set.sq_pad
#define SQ_BASEV set.sq_base
#endif
#define SQ_PATSH (SQ_PADV & SQ_DISARM ? -1 : (int32_t)SQ_PATV)   /* the arp sequencer shown's pattern, -1 NONE */
#define DR_PATSH (set.sq_pad & DR_DISARM ? -1 : (int32_t)set.dr_pat)   /* the drum sequencer's */

/* MIDI out (GLO > MIDI > OUT): what the app sends itself (the keys' notes, HOLD's, the arp
 * sequencer's, PANIC's) goes on OUT CHANNEL to OUT PORTS; the clock (CLOCK MASTER, sq_block) and thru go to OUT PORTS
 * too. Main loop and audio interrupt (the platform's out queues take both) */
static const uint8_t OP_TO[OP_N] = {TO_USB | TO_BLE, TO_USB, TO_BLE, 0};
static const uint8_t CS_PORT[CS_N] = {0, PORT_USB, PORT_TRS, PORT_BLE};   /* CLOCK SOURCE's port (CS_ANY: any) */
#define midi_to() ((uint32_t)OP_TO[set.out_to & 3u])
/* the settings as they work in this build: in the USB build (no Bluetooth) the saved Bluetooth
 * choices stay saved for the Bluetooth build (the same settings object) but work as their USB equivalent: OUT PORTS
 * USB+BT as USB, BT as OFF; CLOCK SOURCE BT as ANY; the BT port in left out of IN PORTS. The Bluetooth build: as saved */
#if FM1B_BLE
#define out_to_eff() set.out_to
#define clk_src_eff() set.clk_src
#define in_off_eff() set.in_off
#define IN_PORTS 3u
#ifdef HYB_MULTI
#define HYB_EDITION ""                    /* (GLO > ABOUT: the Bluetooth build with two parts, the
                                          * public release's only edition: no tag; " MULTI" until 1.18C49, cut off
                                          * at the line's end) */
#else
#define HYB_EDITION ""
#endif
#else
#define out_to_eff() (set.out_to == OP_ALL ? OP_USB : set.out_to == OP_BT ? OP_OFF : set.out_to)
#define clk_src_eff() (set.clk_src == CS_BT ? CS_ANY : set.clk_src)
#define in_off_eff() (set.in_off & ~(1u << PORT_BLE))
#define IN_PORTS 2u
#define HYB_EDITION " USB"                /* (GLO > ABOUT: the USB build told apart) */
#endif
#define HYB_RVTAG ""                     /* (GLO > ABOUT: the own reverb is the release's; 1.18C6 .. 1.18C49 showed "R") */
#ifdef HYB_MULTI
static UI_LATE void midi_send_p(uint32_t p, uint32_t st, uint32_t d1, uint32_t d2)   /* (1.19.4) a channel message
                                          * (st: its kind, 0x80 .. 0xE0) of part p (0 A, 1 B) on its OUT (GLO > MIDI >
                                          * A CHANNEL > OUT: out_ch, B's: mo) to OUT PORTS; OFF: nothing */
{
    const uint32_t to = midi_to(), c = p ? (set.mo & MO_B) - 2u : set.mo & MO_AOFF ? 16u : set.out_ch;
    st = (st & 0xF0u) | (c & 15u);
    if (to && c < 16u) plat_midi_send(st >> 4 | st << 8 | d1 << 16 | d2 << 24, to);
}
#define midi_send_ch(st, d1, d2) midi_send_p(0u, st, d1, d2)   /* (part A's) */
#else
static UI_LATE void midi_send_ch(uint32_t st, uint32_t d1, uint32_t d2)   /* a channel message (st: its kind, 0x80 ..
                                          * 0xE0) on OUT CHANNEL to OUT PORTS */
{
    const uint32_t to = midi_to();
    st = (st & 0xF0u) | (set.out_ch & 15u);
    if (to) plat_midi_send(st >> 4 | st << 8 | d1 << 16 | d2 << 24, to);
}
#endif

static void qa_defaults(settings_t *s)   /* the quick-access list as shipped: FILTER 1, AMP ENV, LFO 1, LFO 2 */
{
    static const uint8_t D[QA_SLOTS] = {7, 16, 18, 19, QA_EMPTY, QA_EMPTY, QA_EMPTY, QA_EMPTY};
    uint32_t k;
    s->qa_ok = QA_V2;
    for (k = 0; k < QA_SLOTS; k++)
        s->qa[k] = D[k];
}

static void glo_defaults(settings_t *s)  /* GLO as shipped: no transpose, any channel, thru off, the clock received
                                          * (EXTERNAL), the device reconnected */
{
    s->glo_ok = GLO_V1;
    s->tr = 0;
    s->midi_ch = 0;
    s->thru = 0;
    s->clk = CLK_EXT;
    s->ble_auto = 1;
}
#ifdef HYB_MULTI
#define HYB_VELO 1                       /* GLO > VELOCITY (1.18C26; the two-part builds: its settings after OSC's) */
#endif
#ifdef HYB_VELO
enum { VC_SOFT2, VC_SOFT, VC_NORMAL, VC_HARD, VC_HARD2, VC_N };   /* CURVE: SOFT louder for a light touch, HARD
                                          * the other way, NORMAL the keys as measured */
#define VE_OFF 1u                        /* settings_t ve_fl: VELOCITY OFF */
#define VE_DRUM 2u                       /* settings_t ve_fl: DRUM ACC VELOCITY (0: RAW, the keys' velocity as measured,
                                          * as before) */
#define VE_FIX_DEF 100u                  /* FIXED as shipped */
#define VE_TOP0 20u                      /* the top row's first key (C#5), VE_NTOP of them: no converter, the LED-line
                                          * probe's peak (touch_top_peak) */
#define VE_NTOP 7u
#define VE_SENS_MAX 5
static UI_LATE void ve_check(settings_t *s, int n)   /* GLO > VELOCITY: an older object's, or one out of range, as
                                          * shipped */
{
    uint32_t k;
    if (n < SET_LEN_V15 || s->ve_ok != VE_V1 || s->ve_fl > 3u || s->ve_fix < 1u || s->ve_fix > 127u
        || s->ve_crv >= VC_N || s->ve_sens < -VE_SENS_MAX || s->ve_sens > VE_SENS_MAX) {
        s->ve_ok = VE_V1;
        s->ve_fl = 0;
        s->ve_fix = VE_FIX_DEF;
        s->ve_crv = VC_NORMAL;
        s->ve_sens = 0;
        for (k = 0; k < VE_NTOP; k++) s->ve_lo[k] = s->ve_hi[k] = 0;
    }
    for (k = 0; k < VE_NTOP; k++)
        if (s->ve_hi[k] <= s->ve_lo[k]) s->ve_lo[k] = s->ve_hi[k] = 0;   /* (no ends of its own) */
    if (n < SET_LEN_V15 || s->ck_ofs < -CK_OFS_MAX || s->ck_ofs > CK_OFS_MAX) s->ck_ofs = 0;   /* (1.19.4: CLK OFFSET;
                                          * an older object's 0) */
    if (n < SET_LEN_V16 || s->dc_ok != DC_V1 || s->dr_ch > 16u) {   /* (1.18C31: DRUM CHANNEL; an older object's, 10) */
        s->dc_ok = DC_V1;
        s->dr_ch = DR_CH_DEF;
    }
    if (n < SET_LEN_V16 || !(s->mo & MO_B) || (s->mo & MO_B) > 17u)   /* (1.19.4: an older object: B OUT where SEQ B
                                          * sent, B CHANNEL's channel, OFF: OUT CHANNEL's; A OUT OUT CHANNEL's; the IN
                                          * values as stored) */
        s->mo = (uint8_t)(s->mx_chb ? s->mx_chb + 1u : s->out_ch + 2u);
    if (s->mx_chb) s->mo &= (uint8_t)~MO_BOMNI;   /* (the flags only with the old field at its stand-in, 0) */
    if (s->midi_ch) s->mo &= (uint8_t)~MO_AINOFF;
    if (n < SET_LEN_V19 || s->pv_ok != PV_V1) {   /* (1.19.7: VOL A / B, an older object's: as shipped) */
        s->pv_ok = PV_V1;
        s->pv_st[0] = s->pv_st[1] = 0;
        s->pv_pad = 0;
    }
    if (n < SET_LEN_V18 || s->cm_ok != CM_V1) {   /* (1.19.6: the MIDI CC map. 1.19.2 .. 1.19.5's object: each PLAY
                                          * mode control's CC to the line of its parameter then; an older one's: none) */
        uint8_t o[7];
        for (k = 0; k < 7u; k++) o[k] = n >= SET_LEN_V17 && s->cm_ok == PC_V1 ? s->cm[k] : 0u;
        s->cm_ok = CM_V1;
        s->cm_show = 0;
        for (k = 0; k < 128u; k++) s->cm[k] = 0;
        for (k = 0; k < 7u; k++)
            if (o[k] && o[k] <= 128u) cm_put(s->cm, o[k] - 1u, cm_mig(s->pm[k], k));
    }
}
#endif
static UI_LATE void md_defaults(settings_t *s)   /* MIDI in and out as shipped: channel 1 to USB and Bluetooth, every
                                          * port in, the clock from any port; the base note C4 (the clock's mode is
                                          * glo_defaults') */
{
    s->md_ok = MD_V1;
    s->out_ch = 0;
    s->out_to = OP_ALL;
    s->in_off = 0;
    s->clk_src = CS_ANY;
    s->sq_base = SQ_BASE1;
    s->eng = EN_ALL;
    s->md_pad = 0;
}
#ifdef HYB_MULTI
static UI_LATE void mx_defaults(settings_t *s)   /* the two parts as shipped: SINGLE, split at C4, B on channel 2, 8
                                          * voices each, B's send its patch's, B the init patch */
{
    s->mx_ok = MX_V1;
    s->mx_mode = SY_M_SINGLE;
    s->mx_split = 60u;
    s->mx_chb = 2u;
    s->mx_nva = SY_MAX_VOICES / 2;
    s->mx_send = MX_SEND_PATCH;
    s->mx_patch = (uint16_t)PATCH_INIT;
    s->mx_dyn = 1u;
    s->mx_maxv = 14u;
    s->mx_uni = (uint8_t)(DEF47 ? 2u : 3u);   /* (1.18C48, the user's choice: UNI MAX SINGLE 4, UNI MAX MULTI 2 (mx_unx
                                          * 0), MONO UNI MAX 6; until 1.18C47 one UNISON MAX for both, 2, and MONO UNI MAX 5) */
    s->mx_unim = 5u;
    s->mx_unx = (uint8_t)(DEF47 ? 0u : MX_UNX_S4 | MX_UNX_M6);
    s->mx_uauto = 0;
    s->mx_steal = 1u;
    s->mx_vcc[0] = s->mx_vcc[1] = 0;
}
static UI_LATE void sqb_defaults(settings_t *s)   /* SEQ B as shipped: PATTERN NONE (pattern 2 kept), the base
                                          * note C4 */
{
    s->sqb_ok = SQB_V1;
    s->sqb_pat = 1u;
    s->sqb_pad = SQ_DISARM;
    s->sqb_base = SQ_BASE1;
}
static UI_LATE void sqb_check(settings_t *s, int n)   /* settings_load: SEQ B's as loaded (n bytes), else as shipped;
                                          * one pattern for both: B's NONE */
{
    if (n < SET_LEN_V12 || s->sqb_ok != SQB_V1 || s->sqb_pat >= SQ_PATS || (s->sqb_pad & ~SQ_DISARM)
        || (s->sqb_base && (s->sqb_base < SQ_BASE_LO || s->sqb_base > SQ_BASE_HI)))
        sqb_defaults(s);
    if (!(s->sq_pad & SQ_DISARM) && !(s->sqb_pad & SQ_DISARM) && s->sqb_pat == s->sq_pat)
        s->sqb_pad = SQ_DISARM;
}
/* (1.18C10) the pattern launcher (a view of SEQ's round, ln_*): LN_ROWS rows of SYNTH 1's (SEQ A), SYNTH
 * 2's (SEQ B) and the drums' pattern and a mode; the main loop starts a row's patterns through the sequencers' own
 * change at the bar's line (sq_qwant: the audio interrupt switches on the line) */
#define LN_ROWS 16u
enum { LN_NO, LN_REP, LN_X1, LN_X2, LN_X4, LN_X8, LN_NMODE };   /* (1.18C18: NO first: a row is NO until a pattern is set (then REP)) */
static UI_LATE int ln_rows_bad(const uint8_t *t)   /* a table of LN_ROWS rows (4 bytes a row: SYNTH 1, SYNTH 2, the
                                          * drums, the mode) not valid */
{
    uint32_t r, bad = 0;
    for (r = 0; r < LN_ROWS && !bad; r++, t += 4)
        bad = t[0] > SQ_PATS || t[1] > SQ_PATS || t[2] > DR_PATS || t[3] >= LN_NMODE || (t[0] && t[0] == t[1]);
    return (int)bad;
}
static UI_LATE void ln_check(settings_t *s, int n)   /* settings_load: the table is not restored (1.18C27: the launcher
                                          * starts empty at every power-on; its rows are kept by its sets, ls_*). The
                                          * fields stay where they were (SET_LEN_V13 .., the layout; an older object's
                                          * table is ignored), written as zeros by the saves from here on */
{
    uint32_t r;
    (void)n;
    for (r = 0; r < sizeof s->ln; r++) ((uint8_t *)s->ln)[r] = 0;
    s->ln_ok = s->ln_pad[0] = s->ln_pad[1] = s->ln_pad[2] = 0;
}
static struct {
    uint8_t sel;                         /* the row selected (drawn at the table's top) */
    uint8_t play;                        /* 1 + the row playing (the launcher's run), 0 none */
    uint8_t next;                        /* 1 + the row asked for at the line sw (PLAY's queue, a mode's next row) */
    uint8_t iss;                         /* the parts whose pattern next's line was asked for (bits 0..2) */
    uint8_t keys, kused;                 /* the part keys down (the lowest three keys, on the launcher), turned with
                                          * a knob or EDIT (their release no MUTE) */
    uint8_t ret;                         /* a sequencer's page opened from the launcher (EDIT): HOME back */
    uint8_t qd;                          /* next queued by PLAY (not a mode's next row) */
    uint8_t lcd;                         /* the strip's state as shown (ln_idle: drawn again on a change) */
    uint8_t emp;                         /* the launcher holds the shared clock (its run, also with no part running: an empty row
                                          * counts its bars silent; cleared by every stop, ln_idle when no row plays) */
    uint8_t prow;                        /* 1 + the active row (last played or previewed), the only one whose edits are heard */
    uint8_t fix;                         /* the row just switched to: its cells checked against the parts (1.18C18, ln_fix) */
    uint8_t pst;                         /* a cell's preview started the transport from a stop (HOME stops it) */
    uint8_t khold;                       /* (1.18C21) the part keys held LN_HOLD_MS or longer: their release no MUTE */
    uint8_t epd, esel;                   /* (1.18C21) the drums chosen as the part edited (a part key held: LN_EP); a part chosen
                                          * so (the launcher's SEQ: that part's sequencer) */
    uint8_t rh;                          /* (1.18C30) the row held for a recording (ln_rec): see ln_idle */
    uint8_t sc[3], sc0;                  /* (1.18C21) the parts' patterns last seen (1 + ln_cur, 0 NONE; sc0: seen once; 1.18C48:
                                          * bit 7 the part running), ln_sync */
    uint32_t kt[3];                      /* (1.18C21) each part key's down time (ln_keys) */
    uint32_t selt;                       /* (1.18C27) SELECT's last detent then (the sets' page: LN_EDGE_MS) */
    uint32_t r0, len, sw;                /* the row's start, its length (the longest pattern), next's line: the shared
                                          * clock's ticks (CK_SH: 512 a bar, its 16 steps at 1x) */
} ln PLAT_POOL;
#ifdef APP_HOST_CMD
static uint8_t ln_nocyc = 1;             /* (host tests: SEQ's round without the launcher, as before 1.18C11, unless "lncyc 1") */
#define LN_CYC (!ln_nocyc)
static uint8_t ls_noedge;                /* (host tests: SELECT up past row 1 stops there, as before 1.18C27: "lsedge 0") */
#define LS_EDGE (!ls_noedge)
static uint8_t sqb_nocyc = 1;            /* (host tests: SEQ's round in SINGLE without SEQ B, as before 1.19.3, unless
                                          * "sqbcyc 1") */
#define SQB_CYC (!sqb_nocyc)
#else
#define LN_CYC 1
#define LS_EDGE 1
#define SQB_CYC 1
#endif
#define LN_EP (ln.epd ? 2u : (uint32_t)ed_part)   /* (1.18C21) the part edited as the launcher shows it: SYNTH 1, SYNTH 2, the drums */
#define SQ_BMO (set.mx_mode == SY_M_SINGLE)   /* (1.19.3) SEQ B as a MIDI-only sequencer: SINGLE, where part B is not
                                          * rendered. It runs, records and launches as in the other modes, its notes go
                                          * to MIDI out alone (sq_out: B CHANNEL, OFF: OUT CHANNEL), its page and the
                                          * launcher's SYNTH 2 column in grey with "MIDI ONLY" / "MIDI" (1.18C11 ..
                                          * 1.19.2: LN_BOFF, SYNTH 2's column, knob and key off there) */
#define LN_SILENT(q) ((q) == 2u ? set.eng == EN_MIDI : set.eng != EN_ALL || ((q) && SQ_BMO))   /* (1.19.3, the owner's
                                          * rule) launcher row q has no internal sound (ENGINE; SEQ B in SINGLE): its
                                          * MUTE mutes its MIDI out instead (sq_out, dr_out); with a sound MUTE is the
                                          * audio's alone, MIDI goes on (as 1.19.2) */
#define LN_MMUTE(q) ((ln_mute >> (q) & 1u) && LN_SILENT(q))
static volatile uint8_t ln_mute PLAT_POOL;   /* the parts muted (bits: SYNTH 1, SYNTH 2, DRUMS; pv_glide, dr_level) */
static UI_LATE void ln_open(void);
static uint8_t ln_vq;                    /* the part whose volume the strip shows (1 + part, 0 none: its colour) */
static const void *lcd_th;               /* the colours the strip was drawn in last */
static UI_LATE uint32_t ln_lights(uint32_t *lb, uint32_t *mid);
static UI_LATE int ln_back(void);
/* (1.18C27) the launcher's sets (ls_*): LS_SLOTS records of LS_REC bytes in OBJ_LSET (app.h),
 * slot k at k * LS_REC. A record is the launcher's rows (their patterns as numbers, their modes), the parts' volumes
 * (VOL A, VOL B, the drums' LEVEL), the tempo, part A's and part B's patch (as references: the patch's number over all
 * banks and its name then, not its bytes: 16 records of the patches' bytes would not fit the one object there is room
 * for) and the drum kit as it was (dr_kit_t, the kit's own record: every instrument's LEVEL, TUNE, DECAY, ATTACK, TONE,
 * MODEL, SEND, PAN, the bus's DRIVE and COMP). A slot never written reads as zeros or lies past the object's end:
 * empty. Written by an explicit SAVE (or DELETE) on the sets' page, or an import from the web page (remote.c 07..09),
 * never by itself. The format is the web page's export too: LS_M0 LS_M1 LS_V1 first (a later version may grow the
 * record into rsv, or the slots into another object: the slot count and the record's size are here alone) */
#define LS_SLOTS 16u
#define LS_REC 192u
#define LS_MAXB 3584u                    /* the object's bytes at most (app.h: bytes 0xF00.. of its sectors never written) */
#define LS_M0 0x4Cu                      /* ("LS") */
#define LS_M1 0x53u
#define LS_V1 1u                         /* (1.18C27 .. C29: pt in 1.18C29's index, the init patch OLD_PATCH_INIT) */
#define LS_V2 2u                         /* (1.18C30: in this one; a record of LS_V1 read as one of LS_V2, ls_v1) */
typedef struct {
    uint8_t m0, m1, ver, fl;             /* LS_M0, LS_M1, LS_V2 (LS_V1); fl 0 (none yet) */
    uint8_t ln[LN_ROWS][4];              /* the rows as set.ln */
    uint8_t vol[2], dlev, tempo;         /* VOL A, VOL B (0 .. 127), the drums' LEVEL (0 .. DR_LMAX), the tempo's byte + 1 */
    uint8_t pt[2][2];                    /* part A's, part B's patch: its number over all banks (PATCH_N's), low byte first */
    char nm[2][10];                      /* their names then (bytes 240 .. 249) */
    dr_kit_t kit;                        /* the drum kit (kit_*: its record in OBJ_DRUM) */
    uint8_t rsv[16];                     /* 0 */
} ls_rec_t;
_Static_assert(sizeof(ls_rec_t) == LS_REC && sizeof(dr_kit_t) == 80u && __builtin_offsetof(ls_rec_t, kit) == 96u,
               "a set's record: 192 bytes, the kit at 96");
_Static_assert(LS_SLOTS * LS_REC <= LS_MAXB && LS_MAXB <= PLAT_STORE_MAX, "the sets: one store object, short of 0xF00");
enum { LS_LOAD, LS_SAVE, LS_CLEAR, LS_DEL, LS_NACT };   /* the sets' page's actions (K4) */
static const char *const LS_ACT[LS_NACT] = {"LOAD", "SAVE", "CLEAR", "DELETE"};
static struct {
    uint8_t on;                          /* the sets' page up: 1 entered above row 1, 2 below row 16 (SELECT the other way: back) */
    uint8_t rsel;                        /* the row selected when it came up (back there) */
    uint8_t slot, act;                   /* the slot (K1), the action (K4) */
    uint8_t ask;                         /* REC asked to confirm: 1 + the action (LS_ASK_MS) */
    uint8_t pend;                        /* a write waiting for the transport's stop: 1 + the slot (its bytes in ls_buf) */
    uint8_t scan;                        /* full read from the store */
    uint16_t full;                       /* the slots filled (bit k) */
    uint16_t pb;                         /* part B's patch to load once the engine took part A's (1 + its number) */
    uint8_t chg;                         /* LOAD: the parts whose patch is not the one saved (bits; the message) */
    uint8_t gen;                         /* moved by every write (the page's rows drawn again: ui.c ls_draw) */
    uint32_t ask_t;
} lset PLAT_POOL;
static ls_rec_t ls_buf PLAT_POOL;        /* a record to write (RAM: the flash's writes take it there), or read */
static UI_LATE uint32_t ls_full(void);
static UI_LATE int ls_read(uint32_t k, ls_rec_t *r);
static UI_LATE void ls_idle(void);
static void patch_tag(char *s, uint32_t p);
#endif

static UI_LATE void pm_defaults(settings_t *s)   /* PLAY mode's controls as shipped: each its default */
{
    uint32_t k;
    s->pm_ok = PM_V1;
    for (k = 0; k < 7u; k++)
        s->pm[k] = PM_DEF;
}

static UI_LATE void sq_step0(sq_step_t *t)   /* a step as shipped, off: NOTE, offset 0, 1/16 long, align 0, velocity
                                          * 100 */
{
    t->ev = SQ_NOTE;
    t->off = 0;
    t->la = 2u;
    t->vel = 100;
}
#ifdef HYB_MULTI
#define UN_NS SQ_NI                      /* the arp sequencers (the undo's copies) */
#define UN_SQI sq_mi                     /* the one shown */
#else
#define UN_NS 1u
#define UN_SQI 0u
#endif
static UI_LATE void un_load(uint32_t q);         /* (the sequencers' undo, un_*: a pattern loaded) */
static UI_LATE void jr_apply(uint8_t *p, uint32_t k);   /* (its edits waiting, the journal: onto a pattern loaded) */
static UI_LATE int pk_load(uint32_t k, uint8_t *q);    /* (1.18C33: a marked pattern's copy; the holder's kept) */
static volatile uint8_t sq_cut[2] PLAT_POOL;   /* main loop: an arp sequencer's pattern replaced (sq_load, COPY TO,
                                          * CLEAR): its notes sounding off at the next block (sq_block). Per arp
                                          * sequencer */
static volatile uint8_t sq_pvm PLAT_POOL;   /* main loop (0.94): the sequencers muted while a step held is heard as it
                                          * is turned (sq_pv_mute, dr_pv_mute): bit 0 / 1 arp sequencer A / B (its
                                          * steps not played, its notes cut at the mute's start: sq_cut), bit 2 the
                                          * drums (their steps not struck); the preview's own note or hit alone */
static UI_LATE void sq_defaults(sq_pat_t *q, uint32_t k0)   /* a pattern as shipped from step k0 on: steps 1, 5, 9, 13
                                          * on, each as sq_step0, the rest (17-64) off; 1x, LENGTH 16 (k0 0) */
{
    uint32_t k;
    for (k = k0; k < SQ_NMAX; k++) {
        sq_step0(&q->st[k]);
        if (k < SQ_N && !(k % 4u)) q->st[k].ev |= SQ_ON;
    }
    if (k0) return;
    q->mark = SQ_MARK;
    q->div = SQ_DIV1;
    q->lc = 0;
    q->pad = (uint8_t)(set.sq_def & SQ_PADM);   /* (1.14: GLO > SEQ's PATCH ARP, KEY LATCH, REC QUANT: a pattern never
                                          * saved, CLEAR; as shipped 0, the settings before 1.14 too) */
    for (k = 0; k < SQ_CHB; k++) SQ_CH8(q)[k] = 0;   /* (no chords) */
}
static UI_LATE int sq_valid(const sq_pat_t *q, uint32_t k0, uint32_t k1)   /* steps k0 .. k1 - 1 can be */
{
    uint32_t k, ok = 1;
    for (k = k0; ok && k < k1; k++) {
        const sq_step_t *t = &q->st[k];
        ok = (t->ev & 0x6Fu) < SQ_NEV && t->off >= -24 && t->off <= 24 && (t->la & 31u) <= SQ_LTOP && t->vel <= 127u;
    }
    return (int)ok;
}
static UI_LATE void sq_load_to(sq_pat_t *q, uint32_t n);
static UI_LATE void sq_load(uint32_t n)  /* pattern n from the store into sqp (sq_load_to). Read in place: the audio
                                          * interrupt may mix the two for a tick */
{
    sq_load_to(&sqp, n);
    sq_cut[UN_SQI] = 1u;                 /* (the notes of the pattern before off: a long one would sound on) */
    un_load(1u + UN_SQI);
}
static UI_LATE void sq_load_to(sq_pat_t *q, uint32_t n)   /* pattern n from the store into q, its head and its rest
                                          * each (never written, unreadable: the default; the rest: fresh steps, off) */
{
    const uint32_t xo = OBJ_SEQX + n / SQ_XPER, xf = (n % SQ_XPER) * SQ_XREC;
    uint8_t xm[4], tr[4];
    if (pk_load(DR_PATS + n, (uint8_t *)q)) return;   /* (1.18C33: a marked pattern's own copy; the one q held kept) */
    if (plat_store_read(OBJ_SEQ, n * SQ_REC + SQ_N * 4u, tr, 4u) != 4 || tr[0] != SQ_MARK || tr[1] >= SQ_NDIV
        || tr[2] >= SQ_NLC || plat_store_read(OBJ_SEQ, n * SQ_REC, q->st, SQ_N * 4u) != (int)(SQ_N * 4u)
        || !sq_valid(q, 0, SQ_N))
        sq_defaults(q, 0);               /* (a first release's record: pad 0, LENGTH 16) */
    else {                               /* (the settings checked before the audio interrupt sees them) */
        uint32_t k;
        if (plat_store_read(OBJ_SEQ, SQ_CHO + n * SQ_CHB, q->ch, SQ_CHB) != (int)SQ_CHB)
            for (k = 0; k < SQ_CHB; k++) SQ_CH8(q)[k] = 0;   /* (1.02: the chords; written before 1.02, the object
                                          * shorter: none. Any byte plays safely: sq_fire keeps its notes in range) */
        q->mark = tr[0];
        q->div = tr[1];
        q->lc = tr[2];
        q->pad = (uint8_t)(tr[3] & SQ_PADM);   /* (PATCH ARP, 0 in every pattern written before 0.71: OFF; KEY
                                          * LATCH, 0 before 0.75: ON; REC QUANT, 0 before 1.14: ON. A pattern saved
                                          * keeps its own: GLO > SEQ's defaults never change it) */
    }
    if (plat_store_read(xo, xf, &q->st[SQ_N], SQ_XREC - 4u) != (int)(SQ_XREC - 4u)
        || plat_store_read(xo, xf + SQ_XREC - 4u, xm, 4u) != 4 || xm[0] != SQ_XMARK || !sq_valid(q, SQ_N, SQ_NMAX))
        sq_defaults(q, SQ_N);
    jr_apply((uint8_t *)q, DR_PATS + n); /* (its edits not saved yet: the journal, after 1.00) */
}
typedef int (*st_part_f)(uint32_t obj, uint32_t off, const void *src, uint32_t n);   /* a part of a pattern's
                                          * record: staged (store_stage), or compared (st_differs) */
static UI_LATE int st_differs(uint32_t obj, uint32_t off, const void *src, uint32_t n)   /* 1: the store does not hold
                                          * these bytes there (a test of store_same's, not counted as a write skipped) */
{
    const uint32_t k = store_n[1];
    const int r = !store_same(obj, off, src, n, 0);
    store_n[1] = k;
    return r;
}
static UI_LATE int dr_negf(st_part_f f, uint32_t o, uint32_t off, const void *g, uint32_t n)   /* a pattern's
                                          * signs (n bytes; 1.02 also the arp patterns' chords) through f; none set
                                          * where the object holds none (written before 0.80, shorter: they read 0)
                                          * need nothing */
{
    const int sz = plat_store_size(o);
    uint32_t i, z = 0;
    for (i = 0; i < n; i++) z |= ((const uint8_t *)g)[i];
    return !z && (sz < 0 || (uint32_t)sz < off + n) ? 0 : f(o, off, g, n);
}
static UI_LATE int sq_parts(const sq_pat_t *q, uint32_t n, uint32_t m, st_part_f f)   /* pattern n's parts m (1 the
                                          * head, 2 the rest) from q, each through f: f's results or'ed */
{
    static const uint8_t XM[4] = {SQ_XMARK, 0, 0, 0};
    const uint32_t xo = OBJ_SEQX + n / SQ_XPER, xf = (n % SQ_XPER) * SQ_XREC;
    int r = 0;
    if (m & 1u)
        r |= f(OBJ_SEQ, n * SQ_REC, q->st, SQ_N * 4u) | f(OBJ_SEQ, n * SQ_REC + SQ_N * 4u, &q->mark, 4u)
             | dr_negf(f, OBJ_SEQ, SQ_CHO + n * SQ_CHB, q->ch, SQ_CHB);   /* (the chords with the head, 1.02) */
    if (m & 2u)
        r |= f(xo, xf, &q->st[SQ_N], SQ_XREC - 4u) | f(xo, xf + SQ_XREC - 4u, XM, 4u);
    return r;
}
static UI_LATE int sq_store(const sq_pat_t *q, uint32_t n, uint32_t m)   /* pattern n's parts m (1 the head, 2 the
                                          * rest) from q (sqp; COPY TO: into another) into the store, staged (one erase
                                          * an object): 0 = done */
{
    return sq_parts(q, n, m, store_stage) | store_flush();
}

enum { DR_BD, DR_SD, DR_LT, DR_MT, DR_HT, DR_RS, DR_CP, DR_CH, DR_OH, DR_CY, DR_CB };   /* the instruments, in the
                                          * black keys' order */
static UI_LATE void dr_defaults(dr_pat_t *q, int beat, uint32_t b0)   /* a pattern as shipped from block b0 on: each
                                          * step's accent 0, length DR_LEN1, align 0 (its sign too); beat (block 1): BD on 1 5 9 13,
                                          * SD on 5 13, CH on every other step (else all off); 1x, LENGTH 16 (b0 0) */
{
    uint32_t b, i, k;
    for (b = b0; b < SQ_NB; b++)
        for (i = 0; i < DR_NI; i++) {
            q->on[b][i] = q->neg[b][i] = 0;
            for (k = 0; k < SQ_N; k++) q->st[b][i][k] = DR_SB(0u, DR_LEN1, 0u);
        }
    if (b0) return;
    if (beat) {
        q->on[0][DR_BD] = 0x1111u;
        q->on[0][DR_SD] = 0x1010u;
        q->on[0][DR_CH] = 0x5555u;
    }
    q->mark = DR_MARK;
    q->div = SQ_DIV1;
    q->lc = 0;
    q->pad = 0;
}
static UI_LATE void dr_xneg(uint32_t n, uint32_t *o, uint32_t *f)   /* drum pattern n's signs of blocks 2-4: their
                                          * object, their offset (DR_NEGX) */
{
    const uint32_t k = n % DR_XPER;
    *o = OBJ_DRUMX + (k < DR_XNK ? n / DR_XPER : DR_PATS / DR_XPER);
    *f = k < DR_XNK ? DR_XPER * DR_XREC + k * DR_NEGX
                    : (DR_PATS % DR_XPER) * DR_XREC + (n / DR_XPER * (DR_XPER - DR_XNK) + k - DR_XNK) * DR_NEGX;
}
static UI_LATE void dr_load_to(dr_pat_t *q, uint32_t n);
static UI_LATE void dr_load(uint32_t n)  /* drum pattern n from the store into drp (dr_load_to). In place, as sq_load */
{
    dr_load_to(&drp, n);
    un_load(0u);
}
static UI_LATE void dr_load_to(dr_pat_t *q, uint32_t n)   /* drum pattern n from the store into q, its head and its
                                          * rest each (never written, unreadable: the default beat; the rest: steps
                                          * off; their signs not there, an object written before 0.80: 0) */
{
    const uint32_t o = n * DR_REC, xo = OBJ_DRUMX + n / DR_XPER, xf = (n % DR_XPER) * DR_XREC;
    const uint32_t no = DR_NI * 2u, ns = DR_NI * SQ_N;
    uint32_t go, gf, i;
    uint8_t t[2];
    if (pk_load(n, (uint8_t *)q)) return;   /* (1.18C33: a marked pattern's own copy; the one q held kept) */
    if (plat_store_read(OBJ_DRUM, o, q->on[0], no) != (int)no || plat_store_read(OBJ_DRUM, o + no, q->st[0], ns) != (int)ns
        || plat_store_read(OBJ_DRUM, o + no + ns, t, 2u) != 2 || t[0] != DR_MARK || (t[1] & 15u) >= SQ_NDIV
        || (t[1] >> 4) >= SQ_NLC)
        dr_defaults(q, 1, 0);
    else {                               /* (a first release's record: the time division < 16, LENGTH 16) */
        q->mark = DR_MARK;
        q->div = (uint8_t)(t[1] & 15u);
        q->lc = (uint8_t)(t[1] >> 4);
        q->pad = 0;                      /* (a pattern on the stack too: un_note, jr_store) */
    }
    if (plat_store_read(xo, xf, q->on[1], (SQ_NB - 1u) * no) != (int)((SQ_NB - 1u) * no)
        || plat_store_read(xo, xf + (SQ_NB - 1u) * no, q->st[1], (SQ_NB - 1u) * ns) != (int)((SQ_NB - 1u) * ns)
        || plat_store_read(xo, xf + DR_XREC - 2u, t, 2u) != 2 || t[0] != DR_XMARK)
        dr_defaults(q, 0, 1);
    dr_xneg(n, &go, &gf);
    if (plat_store_read(OBJ_DRUM, DR_NEG_OFF + n * DR_NEGH, q->neg[0], DR_NEGH) != (int)DR_NEGH)
        for (i = 0; i < DR_NI; i++) q->neg[0][i] = 0;
    if (plat_store_read(go, gf, q->neg[1], DR_NEGX) != (int)DR_NEGX)   /* (neg[1..3] as one) */
        for (go = 1; go < SQ_NB; go++)
            for (i = 0; i < DR_NI; i++) q->neg[go][i] = 0;
    jr_apply((uint8_t *)q, n);           /* (its edits not saved yet: the journal, after 1.00) */
}

static UI_LATE int dr_parts(const dr_pat_t *q, uint32_t n, uint32_t m, st_part_f f)   /* drum pattern n's parts m
                                          * (1 the head, 2 the rest) from q, each through f (as sq_parts) */
{
    const uint32_t o = n * DR_REC, xo = OBJ_DRUMX + n / DR_XPER, xf = (n % DR_XPER) * DR_XREC;
    const uint32_t no = DR_NI * 2u, ns = DR_NI * SQ_N;
    const uint8_t t[2] = {DR_MARK, (uint8_t)(q->div | q->lc << 4)}, x[2] = {DR_XMARK, 0};
    uint32_t go, gf;
    int r = 0;
    dr_xneg(n, &go, &gf);
    if (m & 1u)
        r |= f(OBJ_DRUM, o, q->on[0], no) | f(OBJ_DRUM, o + no, q->st[0], ns) | f(OBJ_DRUM, o + no + ns, t, 2u)
             | dr_negf(f, OBJ_DRUM, DR_NEG_OFF + n * DR_NEGH, q->neg[0], DR_NEGH);
    if (m & 2u)
        r |= f(xo, xf, q->on[1], (SQ_NB - 1u) * no) | f(xo, xf + (SQ_NB - 1u) * no, q->st[1], (SQ_NB - 1u) * ns)
             | f(xo, xf + DR_XREC - 2u, x, 2u) | dr_negf(f, go, gf, q->neg[1], DR_NEGX);
    return r;
}
static UI_LATE int dr_store(const dr_pat_t *q, uint32_t n, uint32_t m)   /* drum pattern n's parts m (1 the head, 2
                                          * the rest) from q (drp; COPY TO: into another), staged: 0 = done */
{
    return dr_parts(q, n, m, store_stage) | store_flush();
}

#ifdef HYB_RV_LEND
/* Bluetooth off at power-on (the Bluetooth two-part build): it is not started until a restart,
 * and the memory it would use (the top 40 KB of its RAM, the tasks' 24 KB of stacks: never touched unstarted) is lent to
 * REVERB FULL, whose ring is then float and 128 KB (the 64 KB below the delay / reverb memory, SH.sp, and that) as the
 * USB build's. BLUETOOTH ON asks for a restart then. The layout is checked here and by the build (lend_check.py) */
static uint8_t bt_lent;
int hybrid_ble_off(void) { return set.ble_off || bt_lent; }
#ifdef HYB_SHIM_LEND
#define SHX_LIVE bt_lent                 /* (1.04) the shimmer on Bluetooth's lent memory alone (the engine's state and
                                          * buffer there): with Bluetooth on its knobs are locked (ui.c P_BTOFF), the
                                          * patches' extensions kept and saved as they are, GLO's REVERB line its own */
#endif
#else
int hybrid_ble_off(void) { return set.ble_off; }
#endif

/* ---------------------------------------------------------------- engine --- */
static sy_synth S PLAT_POOL;
#ifdef HYB_MULTI
/* two parts (the USB build): S is part A; part B, the parts' shared memory (the voice pool, the delay /
 * reverb, the tables) and their chorus buffers here, M the setup (GLO > MULTI, taken by the engine at a block's start) */
#if FM1B_BLE                             /* (the Bluetooth build with two parts: its RAM is full, part
                                          * B in the pool too, as its block buffers: SY_MX_ATTR) */
#define MX_MEM PLAT_POOL
#else
#define MX_MEM
#endif
static sy_synth SB MX_MEM;               /* (the USB build: in RAM, what Bluetooth leaves; the pool holds the rest) */
#if defined(HYB_RV_LEND) && defined(PLAT_HOST)
#ifndef HYB_RV_LEND_PRE
#define HYB_RV_LEND_PRE 0u
#endif
static struct { float pre[HYB_RV_LEND_PRE / 4u + 1u]; float lo[16384]; sy_shared_t sh; } bt_hl;   /* (the host simulator:
                                          * the lent memory as the firmware's, below SH and its delay / reverb memory) */
#define SH bt_hl.sh
#elif defined(HYB_RV_LEND)
static sy_shared_t SH __attribute__((section(".pool.lend")));   /* (first in the pool after the radio's stacks, its delay /
                                          * reverb memory first: build_variant.py --lend, bt_lend_init) */
#else
static sy_shared_t SH PLAT_POOL;
#endif
#ifdef HYB_RV_LEND
extern uint32_t _pool_start[], _bss_end[];   /* (the link's: main_fm1.c's types) */
uint32_t shim_stacks_lo(void);
#define BT_LEND_STACKS 24576u            /* (the radio's three tasks' stacks, 8 KB each, first in the pool) */
static void bt_lend_init(void)
{
#ifdef PLAT_HOST
    bt_lent = 0;                         /* (the host simulator: its static block, bt_hl; its reboot keeps the statics) */
    hyb_rv_lend(0);
    if (!set.ble_off) return;
    bt_lent = 1;
    hyb_rv_lend(bt_hl.lo);
#else
    char *sp = (char *)(void *)SH.sp.rvx_ring, *ps = (char *)(void *)_pool_start;
    if (!set.ble_off || (char *)(void *)&SH != sp || shim_stacks_lo() != (uint32_t)(uintptr_t)ps
        || sp != ps + BT_LEND_STACKS || (char *)(void *)_bss_end > ps)
        return;                          /* (Bluetooth on, or not this layout: REVERB FULL in 16 bits as before) */
    bt_lent = 1;
    hyb_rv_lend((float *)(void *)(sp - 65536));
#endif
}
#endif
static float CH_A[2][2048] PLAT_POOL, CH_B[2][2048] PLAT_POOL;
static float SAT_B[512 + 1] MX_MEM;      /* part B's saturation curve (the engine's SAT_N + 1) */
static sy_multi_t M MX_MEM;
static uint8_t ed_part;                  /* the part the panel edits, 0 A, 1 B (GLO > MULTI > EDIT, SHIFT + A#4 / F#5):
                                          * shadow, set.patch and sx_edit are its; the other's: shadow_o, patch_o */
static uint8_t shadow_o[SH_LEN] MX_MEM;            /* the other part's patch bytes (as shadow), its patch, its SysEx state */
static uint16_t patch_o;
static uint8_t sx_o;
static UI_LATE void mx_apply(void);
#define GOV_MAX M.max_voices             /* the governor's limit: both parts' (the engine splits it by VOICES) */
#else
#define GOV_MAX S.max_voices
#endif
#if FM1B_BLE && !defined(HYB_MULTI)
static UI_LATE void uni_apply(void);
#endif
#ifdef HYB_MULTI
static UI_LATE void osc_apply(void)      /* GLO > AUDIO > OSC and FILT KNOBS to both parts (taken at their next
                                          * block: synth.c) */
{
    S.osc_clean = SB.osc_clean = set.osc_cl ? 1.f : 0.f;
    S.flt_sm = SB.flt_sm = set.flt_sm ? 1u : 0u;
}
#endif
#ifdef HYB_VELO
/* GLO > VELOCITY (1.18C26): the keys' velocity as they play notes (the parts, MIDI out, the sequencers' RECORD), and the
 * drums' accents with DRUM ACC VELOCITY. VELOCITY OFF: every key at FIXED. On: the key's velocity as sensed (lib/touch.c:
 * the analog rows' landing, the top row's LED-line probe through its ends, SENSITIVITY bending both), then CURVE.
 * The top row's default curve: VE_TCURVE */
#define VE_TCURVE 180u                   /* ASSUMPTION (1.18C26): the top row's exponent x 100 between the probe's ends
                                          * 0 / 43 (lib/touch.c measured 2.40 on the peak over the whole press). The
                                          * user found the seven top keys softer than the others on the device; the
                                          * velocity is taken when the scan sees the key down, a probe reading or two
                                          * into the press, so its peak is lower than the measured one. At 1.80 a light
                                          * press (27 .. 36) plays 56 .. 92, a bare contact (20) 33; at 2.40 42 .. 83
                                          * and 21. CALIBRATE gives each key its own ends instead */
static UI_LATE void ve_apply(void)       /* VELOCITY's settings to the keys' sensing (touch.c) */
{
    uint32_t k;
    touch_curve(-(int)set.ve_sens);      /* (touch_curve: + makes 127 harder to reach; SENSITIVITY + the other way) */
    touch_top_curve(VE_TCURVE);
    for (k = 0; k < VE_NTOP; k++) touch_top_ends(VE_TOP0 + k, set.ve_lo[k], set.ve_hi[k]);
}
static UI_LATE uint32_t ve_curve(uint32_t v)   /* a velocity as sensed through VELOCITY's settings: FIXED, CURVE */
{
    static const float X[VC_N] = {0.5f, 0.7f, 1.f, 1.4f, 2.f};   /* (CURVE's exponents) */
    if (set.ve_fl & VE_OFF) return set.ve_fix;
    if (v <= 1u) return 1u;
    if (v > 127u) v = 127u;
    if (set.ve_crv != VC_NORMAL && set.ve_crv < VC_N)
        v = (uint32_t)(1.f + 126.f * fm_powf((float)(v - 1u) / 126.f, X[set.ve_crv]) + .5f);
    return v;
}
static UI_LATE uint32_t key_vel(uint32_t k) { return ve_curve(touch_vel(k)); }   /* key k's velocity as it plays */
static UI_LATE uint32_t key_vel_drum(uint32_t k)   /* key k's for a drum accent: DRUM ACC RAW (as shipped) the key's
                                          * as sensed, VELOCITY through VELOCITY's settings */
{
    return set.ve_fl & VE_DRUM ? key_vel(k) : touch_vel(k);
}
#else
#define key_vel(k) touch_vel(k)
#define key_vel_drum(k) touch_vel(k)
#endif

/* what the main loop asks of the engine, taken by the audio interrupt at a block's start: MIDI messages (status,
 * data 1, data 2) and parameter changes (EV_PARAM, the patch byte, its value) */
#define EV_PARAM 0x01u
#ifdef HYB_MULTI
#define EV_PARAM_B 0x02u                 /* (two parts: a byte of part B's patch; ev_put makes it of EV_PARAM) */
#define EV_PARAM_A 0x03u                 /* (a byte of part A's patch, whichever part is edited) */
#define EV_KEY_A 0x04u                   /* (a key's note to part A alone, d2 the velocity, 0 off: key_out) */
#define EV_KEY_B 0x05u                   /* (the same to part B) */
#define PATCH_REQ_B 0x2000               /* patch_req: pbuf into part B */
#endif
#define TP_BYTE 144u                     /* the tempo's patch byte (B 16: bpm - 63) */
/* the tempo running (tp_set): the engine's byte + 1 (0: none yet). A patch changed while a sequencer runs keeps it
 * (the patch's own byte not changed: no edit, no undo step); changed while both are stopped, the new patch's. The USB
 * build's two parts share it. ALGO turns it from there (param_turn), HOME shows it. Two parts, 1.18: the global tempo,
 * no patch load sets it (either part, any mode); every turn (either part, the remote editor) sets it and GLO's
 * settings keep it (set.tempo, saved by SAVE: glo_save, mx_save_set); both panel copies' B 16 hold it (shadow_send,
 * edit_set) */
static volatile uint8_t tp_run PLAT_POOL;
#define EVQ 64u
static uint32_t evq[EVQ];
static volatile uint32_t ev_w, ev_r;
static volatile int32_t patch_req = -1;  /* a patch to load */
static uint8_t pbuf[256] PLAT_POOL;      /* the patch the engine loads next (the bank's with its edits) */
/* the user wave bank (waves.c, 0.90): a slot written or cleared moves uw_seq; the audio interrupt then builds the
 * tables holding it again (uw_render, one a block) until uw_seen catches up */
static volatile uint8_t uw_seq PLAT_POOL;
static uint8_t uw_seen PLAT_POOL;
static void uw_render(void);
static void uw_scan(void);
static const uint8_t *uw_get(int id);
#define PATCH_REQ_BUF 0x1000             /* patch_req: load pbuf */
#ifdef HYB_SHIMMER
/* FX > SHIMMER to the engine (0.89): part A's extension (the reverb is A's), copied by the main loop (shx_sync), taken
 * by the audio interrupt at a block's start when shx_seq moved (with a patch change: the same block as the patch) */
static uint8_t shx_eng[SHX_N + SHX_NR] PLAT_POOL;   /* (the values, the targets: synth.h shx_rt) */
static volatile uint8_t shx_seq PLAT_POOL;
static void shx_sync(void);
#define SHX_SYNC() shx_sync()
#define SHX_EB(b, v) shx_eb((b), (v))
#else
#define SHX_SYNC() do { } while (0)
#define SHX_EB(b, v) (v)
#endif
static volatile int32_t lat_req = -1;
static volatile uint8_t re_cc PLAT_POOL;  /* (the audio interrupt: a controller went to the engine; remote.c tells it) */
static volatile uint8_t seq_req PLAT_POOL;   /* MIDI START (1) / STOP (2) with CLOCK EXTERNAL, for input() (the pool:
                                          * the Bluetooth build's RAM is full) */    /* the latency setting to take */
/* the latency modes (GLO), all with the reference's two blocks (SY_OPT_LATENCY; the effects on the second core a block
 * later, SY_OPT_FXPIPE); they differ in the platform's FIFO (plat_audio_fifo) and the voices it gives room for:
 * LOW renders in the audio interrupt (12 voices); MED a block ahead (2.9 ms more, 14 voices); HIGH two blocks ahead
 * (5.8 ms more, 16 voices): renders may then run past their time, several in a row, as long as the average stays
 * under it. The codes as saved: 1 LOW (the first releases' MED), 2 MED (their HIGH: the same block ahead), 3 HIGH; the
 * first releases' 0 (LOW, notes at the next block: dropped) is taken as LOW */
#define LAT_LOW 1u
#define LAT_MED 2u
#define LAT_HIGH 3u
#define VOICES_STD 12
static int voice_cap(void);

static uint16_t ev_drop;                 /* (after 1.08) the messages ev_put dropped since power-on (GLO > ABOUT > DIAGNOSIS) */
static void ev_put(uint32_t st, uint32_t d1, uint32_t d2)   /* main loop */
{
    uint32_t w = ev_w;
    if (w - ev_r >= EVQ) {
        if (ev_drop < 0xFFFFu) ev_drop++;
        return;                          /* (full: dropped) */
    }
#ifdef HYB_MULTI
    if (st == EV_PARAM && ed_part) st = EV_PARAM_B;   /* (the part edited) */
#endif
    evq[w % EVQ] = st | d1 << 8 | d2 << 16;
    RING_PUBLISH();
    ev_w = w + 1u;
}

/* ------------------------------------------------------ the step sequencer --- */
/* 16, 32 or 64 steps (its LENGTH) of 1/16 at the tempo the engine's LFO clock uses (S.bpm_now: the MIDI clock's
 * when it runs, else the tempo running, B 16 + 63: tp_run), independent of the engine's arpeggiator. The main loop turns it on and
 * off (sq.run) and hands it the keys' input notes (sq.keys); the audio interrupt (sq_block, at each block's start)
 * advances it in 1/8 of a step (a tick), carrying the fraction, and plays the steps through the engine's MIDI path
 * (sy_midi) and out over MIDI as the keys do (midi_send_ch). While it runs the input notes (the keys held off its
 * page, HOLD's, MIDI notes in) do not sound themselves (none: the BASE NOTE, set.sq_base); the patch's arpeggiator (B 1) is held at 0 in the engine (the
 * patch keeps its byte: no edit, SAVE stores it, the stop gives it back). Timing: block-accurate (128 frames), a
 * tick's events at the first block start at or after its time, no drift */
#define SQ_NT 24u                        /* the notes it can have sounding */
typedef struct {
    volatile uint8_t run;                /* main loop: on */
    volatile uint8_t step;               /* the step playing (the audio interrupt's; the screen and the lights) */
    volatile uint8_t hold;               /* main loop: running: its part's engine arpeggiator held off (B 1 at 0 in the
                                          * engine; sq_hold) */
    uint8_t on;                          /* the audio interrupt's: running */
    uint8_t join;                        /* on, waiting for the shared clock's next tick of its grid (sq_block) */
    uint8_t ldn;                         /* KEY LATCH: a MIDI note struck in since the input was last looked at
                                          * (sq_lat_on; sq_latch takes it as a grip changed, 1.08) */
    uint16_t tick;                       /* ticks (1/8 step) since the start */
    volatile uint32_t keys[4];           /* the main loop's input notes (the keys, HOLD's), a bit a note */
    uint32_t midi[4];                    /* MIDI notes held in (the audio interrupt's) */
    uint32_t lat[4];                     /* KEY LATCH (the audio interrupt's): the input notes gripped last (the notes
                                          * held at the last key struck and those struck since); the input empty: the
                                          * notes latched (0.75, sq_latch; 1.08 the grip) */
    volatile uint8_t lclr;               /* main loop: the latch to be cleared (BASE NOTE turned, KEY LATCH turned) */
    struct { uint8_t note, live; uint16_t off; } nt[SQ_NT];   /* the notes sounding and their end (a tick) */
} sq_rt_t;
#ifdef HYB_MULTI
static sq_rt_t sqr[SQ_NI] PLAT_POOL;            /* SEQ A, SEQ B (sqpa's) */
#define sq sqr[sq_mi]                    /* (the main loop's: the one shown) */
#define SQI sqr[sq_ii]                   /* (the audio interrupt's: the one played) */
#define SQ_BASEI (sq_ii ? set.sqb_base : set.sq_base)
static UI_LATE int sq_runs(void) { return sqr[0].run || sqr[1].run; }
#define SQ_RUNS sq_runs()               /* (either runs) */
#else
static sq_rt_t sq;
#define SQI sq
#define SQ_BASEI set.sq_base
#define SQ_RUNS sq.run
#endif
/* GLO > AUDIO > ENGINE (set.eng): SYNTH+DRUMS renders both; DRUMS ONLY leaves the synth out
 * (sy_render_i24 not called: no voices, effects or control blocks), the drums sound; MIDI ONLY renders nothing (the
 * output zeros, the drums silent). The block clock and all it drives (sq_block: both sequencers, the shared clock,
 * CLOCK MASTER), the keys' and the sequencer's MIDI out, MIDI in and thru go on. A part going off fades out over
 * ENG_FADE blocks (the synth's notes released at its start), then stops; the synth coming on starts from silence
 * (eng_wake). While the synth is off the engine gets nothing to queue (eng_midi: notes, CCs, bend, program changes
 * dropped; a patch byte set at once, so the patch stays the one edited) and the MIDI clock's tempo (S.bpm_now: the
 * sequencers' with CLOCK EXTERNAL) is read here on the engine's own fields as its event handling reads it (eng_clock,
 * app_render) */
#define ENG_FADE 16u                     /* the fade's blocks (~46 ms) */
static struct {
    uint8_t now;                         /* the mode rendered (EN_*) */
    uint8_t to;                          /* the mode faded to */
    uint8_t fade;                        /* the fade's blocks left (0: none) */
    uint8_t mute;                        /* the synth's next blocks not heard (eng_wake) */
    uint8_t syn;                         /* the engine takes MIDI (the synth on, not fading out) */
} eng;
static UI_LATE void eng_clock(uint32_t st)   /* audio interrupt, the synth off: a MIDI clock or transport message on
                                          * the engine's clock fields, as synth.c apply_event takes it (the tempo's
                                          * part; the arpeggiator's glide-in starts again with the synth, eng_wake) */
{
    if (st == 0xFAu || st == 0xFBu) {
        S.xclk_on = 1;
        if (st == 0xFAu) S.xclk_ticks = 0;
        S.xclk_frames = -1e9f;           /* (the start to the first tick is no spacing) */
    } else if (st == 0xFCu)
        S.xclk_on = 0;
    else if (st == 0xF8u) {
        if (S.xclk_frames > 0.f && S.xclk_frames < S.fs)
            S.xclk_per = S.xclk_per > 0.f ? 0.75f * S.xclk_per + 0.25f * S.xclk_frames : S.xclk_frames;
        S.xclk_frames = 0.f;
        S.xclk_on = 1;
        S.xclk_ticks++;
    }
}
static UI_LATE void eng_panic(void);
static UI_LATE void eng_midi(uint32_t st, uint32_t d1, uint32_t d2)   /* audio interrupt: a message to the engine
                                          * (sy_midi), the synth off: the clock's to eng_clock, a patch byte
                                          * (SY_EV_PARAM) set at once, the rest dropped */
{
    if (st == SY_EV_PARAM && d2 > 127u) d2 = 127u;   /* (as sy_set_param_queued) */
    if ((st & 0xF0u) == 0xB0u && d1 == 120u) eng_panic();   /* (1.18C49: all sound off, the tails too) */
    if (eng.syn) sy_midi(&S, (uint8_t)st, (uint8_t)d1, (uint8_t)d2);
    else if (st == SY_EV_PARAM) sy_set_param(&S, (int)d1, (int)d2);
    else if (st >= 0xF8u) eng_clock(st);
}
#ifdef HYB_MULTI
enum { EM_A, EM_B, EM_KEYS, EM_IN };     /* eng_to: part A alone, part B alone, the panel's keys (by the setup), MIDI in
                                          * (by the setup and part B's channel) */
static UI_LATE void eng_to(uint32_t to, uint32_t st, uint32_t d1, uint32_t d2)   /* audio interrupt: eng_midi's, two parts */
{
    sy_synth *p = to == EM_B ? &SB : &S;
    if (st == SY_EV_PARAM && d2 > 127u) d2 = 127u;
    if ((st & 0xF0u) == 0xB0u && d1 == 120u) eng_panic();   /* (1.18C49: all sound off, the tails too: the panel's
                                          * PANICs and MIDI in's CC 120; CC 123, all notes off, releases the notes alone) */
    if (eng.syn) {
        if (to == EM_KEYS) sy_multi_play(&M, (uint8_t)st, (uint8_t)d1, (uint8_t)d2);
        else if (to == EM_IN) sy_multi_midi(&M, (uint8_t)st, (uint8_t)d1, (uint8_t)d2);
        else sy_midi(p, (uint8_t)st, (uint8_t)d1, (uint8_t)d2);
    } else if (st == SY_EV_PARAM) sy_set_param(p, (int)d1, (int)d2);
    else if (st >= 0xF8u) eng_clock(st);
}
#define eng_midi(st, d1, d2) eng_to(EM_KEYS, st, d1, d2)   /* (the rest: the panel's) */
#endif
#ifdef HYB_MULTI
#define SQ_IIX sq_ii                     /* (the audio interrupt's sequencer, its index) */
#else
#define SQ_IIX 0u
#endif
static struct {                          /* (1.08) each arp sequencer's grip (the audio interrupt's; pool: .data + .bss
                                          * and input() as they were) */
    uint32_t hd[4];                      /* the input notes held when last looked at (sq_latch: a key struck) */
    uint32_t fp;                         /* the notes (nt[] bit i) of a FING POLY step feeding the patch's
                                          * arpeggiator: they follow the grip while the step sounds (sq_follow) */
    uint8_t fv;                          /* that step's velocity */
} sq_gr[2] PLAT_POOL;
#define SQG sq_gr[SQ_IIX]
static volatile uint16_t sq_mute[2] PLAT_POOL;   /* main loop: RECORD on the arp sequencer's page (sq_rec_hit): a step
                                          * recorded ahead of its tick, not played again there: the step counted from
                                          * the clock's start (13 bits) + 1; 0 none. Per arp sequencer */
static volatile uint8_t sq_metro PLAT_POOL;   /* main loop: RECORD on the arp sequencer's page: 1 + its index, the
                                          * metronome on its grid (sq_block); 0 none */
/* (1.18C40) GLO > SEQ > PRECOUNT: PLAY from a stop with RECORD armed on the page shown (the arp
 * sequencer's or the drums') counts in 1 or 2 bars of 4 beats (a beat: 4 steps of that sequencer's time division, as
 * RECORD's metronome) before step 1. The shared clock starts that many ticks before 0 (ck.mt negative, the main
 * loop's pc_tk taken at its start); the sequencers are on from PLAY but play no tick before 0 (they join at 0, the
 * drums' origin 0: step 1 there, as a start without a count), the metronome clicks the count's beats (sq_block), a
 * key is heard and not recorded (rec_abs: REC_PRE), unless it is heard within half a step (REC QUANT OFF: an eighth)
 * of step 1: then its rounding puts it on step 1, as the keys around any step. Nothing else changes: OFF (pc_tk 0)
 * starts the clock at 0, as before */
static volatile uint16_t pc_tk PLAT_POOL;   /* main loop: the count's ticks, for the clock's next start (0: none) */
static volatile uint8_t pc_cut PLAT_POOL;   /* main loop: RECORD off during the count: its end now (step 1 at once) */
/* (0.96, 13) a pattern chosen while its sequencer runs waits for the bar's line: the bar playing
 * (4 beats: 16 steps at 1x, 4 << div steps; a pattern shorter than that, ASSUMED: its end) plays out, then the new
 * pattern starts at its step 1 on the line. No second buffer (the Bluetooth two-part build's pool has no room): the
 * main loop's request (sq_qr: SQ_QR_SWAP, SQ_QR_NONE: stop at the line, SQ_QR_JOIN: a sequencer joining from NONE,
 * silent until its first line; the target in sq_qt: 0 none, 1 + n pattern n, SQ_QNONE; 0 and 1 the arp sequencers
 * A, B, 2 the drums). The audio interrupt (sq_qs, its own): in the bar's last step, its own notes played, SQ_QS_LOAD
 * (nothing more played before the line, the division and LENGTH kept from then: sq_qd, sq_ql); the main loop loads
 * the pattern in place then (sq_qm 1). At the line its notes off (ASSUMED: a note tied over the line ends at it),
 * the origin moved there (sq_org: the tick its step 1 is, the arp sequencers' in their own ticks, the drums' (dr_org)
 * in the clock's) and, loaded, the new pattern's step 1 at once (SQ_QS_DONE: exact). Not loaded yet (the main loop
 * busy): SQ_QS_WAIT, silent, and step 1 played as soon as it is (sq_qcatch: late, never dropped; a later step the
 * wait took is). NONE: SQ_QS_HALT, silent until the main loop stops it. The step before the line plays no early step of
 * the old pattern's next; the new step 1's early ALIGN sounds on the line (ASSUMED). The drums after 0.99: loaded
 * ahead into a spare and swapped in by the audio interrupt on the line, never late (dr_qwant, dr_qidle, dr_clk) */
#define SQ_QNONE 0xFFu
#define SQ_QPAT(q, cur) (sq_qt[q] ? (sq_qt[q] == SQ_QNONE ? -1 : (int32_t)sq_qt[q] - 1) : (cur))   /* the pattern
                                          * shown on the knob: the one waiting, else cur */
enum { SQ_QR_OFF, SQ_QR_SWAP, SQ_QR_NONE, SQ_QR_JOIN };
enum { SQ_QS_OFF, SQ_QS_LOAD, SQ_QS_WAIT, SQ_QS_DONE, SQ_QS_HALT };
static uint8_t sq_qt[3] PLAT_POOL, sq_qo[3] PLAT_POOL;   /* the target; the pattern before (a change taken back) */
static volatile uint8_t sq_qr[3] PLAT_POOL, sq_qm[3] PLAT_POOL;   /* main loop: the request; the target loaded */
static volatile uint8_t sq_qs[3] PLAT_POOL, sq_qd[3] PLAT_POOL, sq_ql[3] PLAT_POOL;   /* audio interrupt: its state;
                                          * the division and LENGTH kept from SQ_QS_LOAD to the line */
static uint16_t sq_org[2] PLAT_POOL;     /* each arp sequencer's origin */
static uint32_t dr_org PLAT_POOL, dr_qm PLAT_POOL;   /* the drums': in the clock's ticks; the last clock tick taken */
#define SQ_QP8(div, lc) (8u * ((4u << (div)) < SQ_STEPS(lc) ? (4u << (div)) : SQ_STEPS(lc)))   /* a bar's ticks, at most
                                          * the pattern's */
#ifdef APP_HOST_CMD
static int sq_log;                       /* (the host simulator: each note of the arp sequencers printed, hostcmd.c) */
static uint32_t sq_qt_now;               /* (the host: the tick from the origin being played, sq_lastt) */
static uint32_t sq_mt_now, sq_lastm[2], sq_line_mt[2];  /* (1.14, the host: the clock's tick being played; each arp sequencer's last
                                          * note on's, recqt) */
static float host_rq_h;                  /* (1.14, the host: RECORD's last key, where it was heard: the clock's ticks) */
static int sq_qbusy;                     /* (the host: sq_qidle held off, hostcmd.c "qbusy") */
static uint32_t sq_fulls, sq_ncol;      /* (the host, 0.97: a sequencer's page drawn whole; its parts drawn) */
static uint32_t host_pl;                 /* (the host, 1.18C35: patches loaded, auditioned; hostcmd.c "plload") */
static uint8_t sq_lastn[2];              /* (the host simulator: each arp sequencer's last note on, hostcmd.c) */
static uint16_t sq_lastt[2];             /* (the host simulator: its tick in the loop) */
static uint8_t sq_lastv[2];              /* (the host simulator, 1.02: its velocity; the last note-off's tick) */
static uint16_t sq_lasto[2];
static uint16_t dr_hpv, dr_hpn;          /* (the host simulator: the last instrument heard (dr.prev) and its step byte
                                          * << 8, how many) */
static uint16_t sq_hon[2], dr_hon;       /* (the host simulator, 0.94: each arp sequencer's note-ons, the drums' step
                                          * hits; hostcmd.c "sqnis") */
static uint32_t dr_hm;                   /* (the host, 1.18C40: the clock's tick of the drums' last step hit, "lastmis") */
static uint8_t sq_hpl[2][SQ_NMAX][8];   /* (the host simulator, 1.05: the notes each step played the last time it
                                          * played, in their order, 0 past the last; hostcmd.c "sqpl") */
static uint32_t sq_hplt[2][SQ_NMAX];     /* (its tick from the origin + 1) */
#endif
/* (1.18C33) PRESET's favorites (fv_*): up to FV_N references to user slots in the
 * player's order (favorite 1 first), global (not a part's), kept in OBJ_LSET past the launcher's sets (fv_load,
 * fv_put). The favorites editor (fv.ed) auditions each patch whole on part A's engine (its own effects; part B and
 * the drums muted, SEQ A's notes held back from the engine: fv_aud), the panel's buffers never touched */
#define FV_N 16u
static struct {
    uint16_t p[FV_N];                    /* favorite 1 .. n: the patch's number (PATCH_N's index; a user slot) */
    uint8_t n;                           /* the favorites */
    uint8_t sec;                         /* the patch list's cursor in the favorites' section: 1 + its line (0 EDIT
                                          * FAVORITES, 1 .. n the favorites top down: favorite n first); 0 not there */
    uint8_t ask;                         /* the patch list: "REMOVE FAV ?" asked for favorite ask (1 ..; 0 none) */
    uint8_t ed;                          /* the favorites editor up */
    uint8_t wr;                          /* a write waiting: bit 0 the order moved (FV_MOVE_MS after the last move), bit 1
                                          * a change, bit 2 the beat ran ("SAVE PENDING" said: "SAVED" at the stop) */
    uint8_t eb;                          /* the editor: the bank shown (its index: patch / PA_N) */
    uint8_t ei;                          /* the editor: its cursor (0 .. n - 1 the favorites, top down; then the bank's
                                          * filled slots) */
    uint32_t t;                          /* the question's time, the last move's */
} fv PLAT_POOL;
#define PL_RZ (fv.n + 1u)               /* the patch list's line RANDOMIZE (with RZ1) */
#define PL_INIT (fv.n + 1u + RZ1)       /* its line INIT (the bank's patches after it) */
static volatile uint8_t fv_aud PLAT_POOL;   /* the editor's audition on: part A plays it, the rest muted (pv_glide,
                                          * dr's gain, sq_out, key_out) */
static UI_LATE void sq_out(uint32_t p, uint32_t st, uint32_t note, uint32_t vel)   /* audio interrupt: a note to arp
                                          * sequencer p's part and out (st | SQ_MO: out alone, 1.02) */
{
    if (fv_aud && !p) st |= 0x100u;      /* (1.18C33: the favorites editor's audition on part A: SEQ A's notes out
                                          * alone, it runs on unheard) */
#ifdef HYB_MULTI
    if (!(st & 0x100u) && !(p && SQ_BMO)) eng_to(p ? EM_B : EM_A, st, note, vel);   /* (SEQ A plays part A, on OUT
                                          * CHANNEL; SEQ B part B, on part B's channel, none set: OUT CHANNEL. 1.19.3:
                                          * SINGLE, SEQ B MIDI out alone: part B has no voices there) */
    st &= 0xFFu;
    if (st == 0x90u && LN_MMUTE(p)) return;   /* (1.19.3: its row muted, no sound of its own: no note-on out; the
                                          * offs go on, its notes sounding were cut at the mute, ln_keys) */
    midi_send_p(p, st, note, vel);       /* (1.19.4: SEQ A on A's OUT, SEQ B on B's; OFF: none) */
#else
    (void)p;
    if (!(st & 0x100u)) eng_midi(st, note, vel);
    st &= 0xFFu;
    midi_send_ch(st, note, vel);
#endif
}
/* the preview's notes (0.85, sq_pv_play): the main loop's request, sq_pvq: the note (bits 0-6), on (bit 7), the
 * velocity (8-14), the part (15), a count (24-31: a new request; one overwritten before it was seen is dropped, its
 * note never started). The audio interrupt keeps what sounds (sq_pva): a new request cuts it at the block's start
 * (sq_pv_cut_a: before the sequencers' steps, so a start right after it is not cut by it) and starts its note after
 * the panel's events (sq_pv_on_a: after a hold of the arpeggiator sent with it). Nothing can stay on: whatever a
 * request asks, the note sounding is ended first */
static volatile uint32_t sq_pvq PLAT_POOL;
static uint8_t sq_pva[3u + 2u * (SQ_CHV - 1u)] PLAT_POOL;   /* the count seen; the note sounding (| 0x80, 0 none); its
                                          * part (bit 0), its note-on pending (bit 1), a note ended in this block
                                          * (bit 2); (1.02) a CHORD step's other notes + 1 (0 none): sounding (the
                                          * audio interrupt's, SQ_PVY), asked (the main loop's, written before its
                                          * request, SQ_PVX) */
#define SQ_PVY (sq_pva + 3)
#define SQ_PVX (sq_pva + 3 + SQ_CHV - 1)
static UI_LATE void sq_pv_cut_a(void)    /* audio interrupt, a block's start. A note ended here: the next one, and an
                                          * arp sequencer starting, wait for the next block (the engine takes a
                                          * block's note-ons before its note-offs: one key's off and on in one block
                                          * would leave the key let go under a voice still gated) */
{
    const uint32_t q = sq_pvq, e = sq_pva[1] ? 4u : 0u;
    sq_pva[2] &= 3u;
    if ((uint8_t)(q >> 24) == sq_pva[0]) return;
    sq_pva[0] = (uint8_t)(q >> 24);
    if (e) {
        uint32_t i;
        sq_out(sq_pva[2] & 1u, 0x80u, sq_pva[1] & 0x7Fu, 0u);
        for (i = 0; i < SQ_CHV - 1u; i++)   /* (a chord's other notes, 1.02) */
            if (SQ_PVY[i]) { sq_out(sq_pva[2] & 1u, 0x80u, SQ_PVY[i] - 1u, 0u); SQ_PVY[i] = 0; }
    }
    sq_pva[1] = 0;
    sq_pva[2] = (uint8_t)((q >> 15 & 1u) | (q >> 6 & 2u) | e);
}
static UI_LATE void sq_pv_on_a(void)     /* audio interrupt, after the panel's events */
{
    const uint32_t q = sq_pvq;
    if ((sq_pva[2] & 6u) != 2u || (uint8_t)(q >> 24) != sq_pva[0]) return;   /* (none; one ended in this block; a
                                          * newer one: the next block's) */
    uint32_t i;
    sq_pva[2] &= 1u;
    sq_pva[1] = (uint8_t)(q | 0x80u);
    sq_out(sq_pva[2], 0x90u, q & 0x7Fu, q >> 8 & 0x7Fu);
    for (i = 0; i < SQ_CHV - 1u; i++)    /* (a CHORD step's other notes, 1.02) */
        if ((SQ_PVY[i] = SQ_PVX[i]) != 0) sq_out(sq_pva[2], 0x90u, SQ_PVX[i] - 1u, q >> 8 & 0x7Fu);
}
static UI_LATE void sq_emit(uint32_t st, uint32_t note, uint32_t vel)   /* audio interrupt: a note to the engine and out */
{
#ifdef APP_HOST_CMD
    if (st == 0x90u) sq_lastm[SQ_IIX] = sq_mt_now;
    if (st == 0x90u) { sq_hon[SQ_IIX]++; sq_lastn[SQ_IIX] = (uint8_t)note; sq_lastv[SQ_IIX] = (uint8_t)vel; sq_lastt[SQ_IIX] = (uint16_t)(sq_qt_now & (SQ_STEPS(SQPI.lc) * 8u - 1u)); }
    if (st == 0x90u) {
        const uint32_t hs = sq_qt_now >> 3 & (SQ_STEPS(SQPI.lc) - 1u);
        uint32_t hi = 0;
        if (sq_hplt[SQ_IIX][hs] != sq_qt_now + 1u)
            for (sq_hplt[SQ_IIX][hs] = sq_qt_now + 1u; hi < 8u; hi++) sq_hpl[SQ_IIX][hs][hi] = 0;
        hi = 0;
        while (hi < 7u && sq_hpl[SQ_IIX][hs][hi]) hi++;
        sq_hpl[SQ_IIX][hs][hi] = (uint8_t)(note + 1u);
    }
    else sq_lasto[SQ_IIX] = (uint16_t)(SQI.tick & (SQ_STEPS(SQPI.lc) * 8u - 1u));
    if (sq_log) printf("  sqemit %s %u vel %u tick %u step %u ms %u\n", st == 0x90u ? "on " : "off", (unsigned)note,
                       (unsigned)vel, (unsigned)SQI.tick, (unsigned)(SQI.tick >> 3 & (SQ_STEPS(SQPI.lc) - 1u)),
                       (unsigned)plat_ms());
#endif
    sq_out(SQ_IIX, st, note, vel);
}
static UI_LATE void sq_all_off(void)     /* audio interrupt: its notes off */
{
    uint32_t i;
    for (i = 0; i < SQ_NT; i++)
        if (SQI.nt[i].live) { SQI.nt[i].live = 0; sq_emit(0x80u, SQI.nt[i].note, 0u); }
}
/* (1.02) a note's state (nt[].live): SQ_LV 1 sounding to its end (off), SQ_LVL latched (the patch's arpeggiator
 * holds it past its end: sq_fire), SQ_LVE ending in this tick (sq_tick, sq_fire; sq_endp sends its off unless a step
 * of the same tick took it on again). The arpeggiator fed (sq_af: the part's patch arpeggiator on, not held) takes a
 * block's note-ons before its note-offs (sy_part_head.inc): a key's off and on in one block lost the key (1.01: a
 * NOTE step after a step on the same note left the arpeggiator silent); its keys are held, so a note on again is no
 * off and on there, only over MIDI out (an off and an on, as before) */
#define SQ_LV 1u
#define SQ_LVL 2u
#define SQ_LVE 3u
#define SQ_MO 0x100u                     /* sq_emit's st: over MIDI out alone, not to the engine */
static UI_LATE uint32_t sq_af(void)      /* audio interrupt: SQI's notes feed its part's patch arpeggiator (B 1 on in
                                          * the engine: PATCH ARP ON, not held off) */
{
#ifdef HYB_MULTI
    return (sq_ii ? SB.patch[129] : S.patch[129]) != 0;
#else
    return S.patch[129] != 0;
#endif
}
static UI_LATE void sq_endp(void)        /* audio interrupt: the notes ending in this tick (SQ_LVE) off */
{
    uint32_t i;
    for (i = 0; i < SQ_NT; i++)
        if (SQI.nt[i].live == SQ_LVE) { SQI.nt[i].live = 0; sq_emit(0x80u, SQI.nt[i].note, 0u); }
}
static UI_LATE void sq_play(uint32_t note, uint32_t vel, uint32_t len, uint32_t lv)   /* audio interrupt: a note on
                                          * for len ticks (lv: SQ_LV, SQ_LVL latched; | 4 the arpeggiator fed) */
{
    uint32_t i, f = SQ_NT;
    for (i = 0; i < SQ_NT; i++) {
        if (SQI.nt[i].live && SQI.nt[i].note == note) { sq_emit(0x80u | (lv & 4u ? SQ_MO : 0u), note, 0u); f = i; break; }   /* (again:
                                          * restarted; the arpeggiator fed: its key stays held, 1.02) */
        if (!SQI.nt[i].live && f == SQ_NT) f = i;
    }
    if (f == SQ_NT) {                    /* (all in use: the one ending first gives way) */
        uint32_t best = 0xFFFFu;
        for (i = 0; i < SQ_NT; i++)
            if ((uint16_t)(SQI.nt[i].off - SQI.tick) < best) { best = (uint16_t)(SQI.nt[i].off - SQI.tick); f = i; }
        sq_emit(0x80u, SQI.nt[f].note, 0u);
    }
    SQI.nt[f].note = (uint8_t)note;
    SQI.nt[f].live = (uint8_t)(lv & 3u);
    SQI.nt[f].off = (uint16_t)(SQI.tick + len);
    sq_emit(0x90u, note, vel);
}
/* KEY LATCH (0.75, EDIT > KEY LATCH, the pattern's): the input notes gripped last gather in
 * SQI.lat: a key or a MIDI note struck makes it the notes held at that moment (1.08; up to 1.07 the notes held since
 * the input was last empty: a chord changed with a finger kept down latched both chords); letting keys go changes
 * nothing. The last one let go, they stay as the input (sq_tick): one note the live base note, a chord as HOLD keeps
 * it, until the next key replaces them. Internal input alone (they sounded not themselves: no note-off owed). Cleared
 * at the start (PLAY), by PANIC (sq_panic), the main loop's request (SQI.lclr) and while RECORD is armed on it (the
 * keys recorded are no input) */
static UI_LATE void sq_lat_clr(void)     /* audio interrupt: SQI's latch gone */
{
    SQI.lat[0] = SQI.lat[1] = SQI.lat[2] = SQI.lat[3] = 0;
    SQG.hd[0] = SQG.hd[1] = SQG.hd[2] = SQG.hd[3] = 0;
    SQI.ldn = SQI.lclr = 0;
}
static UI_LATE uint32_t sq_latch(void)   /* audio interrupt, a block's start, SQI on: its latch after the input now;
                                          * 1: a note struck since the last look (the grip changed, sq_follow) */
{
    uint32_t w, c[4], s = 0;
    if (SQI.lclr || sq_metro == 1u + SQ_IIX) { sq_lat_clr(); return 0; }
    for (w = 0; w < 4u; w++) {
        c[w] = SQI.keys[w] | SQI.midi[w];
        s |= c[w] & ~SQG.hd[w];
        SQG.hd[w] = c[w];
    }
    if (s)                               /* (a key struck: the grip the notes held now) */
        for (w = 0; w < 4u; w++) SQI.lat[w] = c[w];
    s |= SQI.ldn;                        /* (a MIDI note struck in, sq_lat_on: in the latch already) */
    SQI.ldn = 0;
    return s != 0;
}
static UI_LATE void sq_fire(const sq_step_t *p);
static UI_LATE const sq_step_t *sq_cel(const sq_step_t *p, uint32_t s, sq_step_t *t);   /* (1.05: CHORD ENTRY heard
                                          * live, after ui.c) */
static UI_LATE const uint8_t *sq_cech(void);
static UI_LATE void sq_steps(uint32_t t, uint32_t g, int line)   /* audio interrupt: tick t's steps (t from the
                                          * origin): the step's own (ALIGN 0 .. 7/8 late), the next one's (ALIGN early:
                                          * 8 - its eighths into this step; over the loop's end its first step's; a
                                          * change waiting, g the bar's ticks: not over the line, 0.96). line: the
                                          * first tick after a change, step 1's early one there too */
{
    const uint32_t m = SQ_STEPS(SQPI.lc) - 1u, st = t >> 3 & m;   /* (LENGTH: 16, 32 or 64 steps) */
    uint32_t i;
    sq_step_t ce;                        /* (1.05: the step in CHORD ENTRY as its chord is chosen now, sq_cel) */
#ifdef APP_HOST_CMD
    sq_qt_now = t;
#endif
    SQI.step = (uint8_t)st;
    for (i = 0; i < 2u; i++) {
        const sq_step_t *p = sq_cel(&SQPI.st[(st + i) & m], (st + i) & m, &ce);
        const int32_t a = SQ_AL(p);
        if ((p->ev & SQ_ON) && p->vel && (((a < 0) == (int)i && (uint32_t)(a & 7) == (t & 7u)) || (line && !i && a < 0))
            && (((t >> 3) + i) & 0x1FFFu) + 1u != sq_mute[SQ_IIX]   /* (RECORD: played live already, this pass) */
            && !(sq_pvm >> SQ_IIX & 1u)  /* (a step held heard as it is turned: its preview alone, 0.94) */
            && !(i && g && !((t + 8u) & ~7u & (g - 1u))))   /* (the next step past the line, g its ticks: the
                                          * pattern after it, 0.96) */
            sq_fire(p);
    }
}
static UI_LATE void sq_tick(void)        /* audio interrupt: tick SQI.tick's events: the notes ending, then a step's
                                          * (sq_steps); 0.96: a pattern waiting, its line (sq_q*) */
{
    const uint32_t ix = SQ_IIX, t = SQI.tick, rq = sq_qr[ix];
    uint32_t i, s = sq_qs[ix], line = 0, r = (uint16_t)(t - sq_org[ix]);
    const uint32_t p8 = s == SQ_QS_LOAD ? SQ_QP8(sq_qd[ix], sq_ql[ix]) : SQ_QP8(SQPI.div, SQPI.lc), af = sq_af();
    for (i = 0; i < SQ_NT; i++)          /* (the arpeggiator fed: a note ending now off after this tick's steps,
                                          * sq_endp, unless one takes it on again; latched, 1.02: off once the
                                          * arpeggiator, PATCH ARP or KEY LATCH is off) */
        if ((SQI.nt[i].live == SQ_LV && SQI.nt[i].off == (uint16_t)t)
            || (SQI.nt[i].live == SQ_LVL && (!af || (SQPI.pad & (SQ_PARP | SQ_NLAT)) != SQ_PARP))) {
            if (af && SQI.nt[i].live == SQ_LV) SQI.nt[i].live = SQ_LVE;
            else { SQI.nt[i].live = 0; sq_emit(0x80u, SQI.nt[i].note, 0u); }
        }
    if (rq && (s == SQ_QS_OFF || s == SQ_QS_LOAD) && !(r & (p8 - 1u))) {   /* (the line) */
        sq_org[ix] = (uint16_t)t;
#ifdef APP_HOST_CMD
        sq_line_mt[ix] = sq_mt_now;      /* (the host: this line in the clock's ticks, "lntr") */
#endif
        r = 0;
        line = 1;
        if (rq != SQ_QR_JOIN) sq_all_off();
        s = rq == SQ_QR_NONE ? SQ_QS_HALT : rq == SQ_QR_JOIN || sq_qm[ix] ? SQ_QS_DONE : SQ_QS_WAIT;
        sq_qs[ix] = (uint8_t)s;
    }
    if (s == SQ_QS_LOAD || s == SQ_QS_WAIT || s == SQ_QS_HALT || (rq == SQ_QR_JOIN && s == SQ_QS_OFF)) return;   /* (silent:
                                          * loaded or being loaded, NONE from the line, the joining one before it) */
    sq_steps(r, rq == SQ_QR_SWAP || rq == SQ_QR_NONE ? p8 : 0u, (int)line);
    if (rq == SQ_QR_SWAP && s == SQ_QS_OFF && !(((r | 7u) + 1u) & (p8 - 1u))) {   /* (the bar's last step: its own note
                                          * played, the main loop may load) */
        const sq_step_t *p = &SQPI.st[(r >> 3) & (SQ_STEPS(SQPI.lc) - 1u)];
        const int32_t a = SQ_AL(p);
        if (!(p->ev & SQ_ON) || a <= (int32_t)(r & 7u)) {
            sq_qd[ix] = SQPI.div;
            sq_ql[ix] = SQPI.lc;
            sq_qs[ix] = SQ_QS_LOAD;
        }
    }
}
static UI_LATE void sq_qcatch(void)      /* audio interrupt, a block's start: the request taken back (its state again
                                          * 0); loaded after the line (SQ_QS_WAIT): step 1's ticks from the line played
                                          * now (a later step the wait took: not played) */
{
    uint32_t u = sq_org[SQ_IIX];
    if (!sq_qr[SQ_IIX]) { sq_qs[SQ_IIX] = SQ_QS_OFF; return; }
    if (sq_qs[SQ_IIX] != SQ_QS_WAIT || !sq_qm[SQ_IIX] || !SQI.on) return;
    sq_qs[SQ_IIX] = SQ_QS_DONE;
    for (;;) {                           /* (step 1's ticks alone: a later step missed stays so, no burst) */
        if ((uint16_t)(u - sq_org[SQ_IIX]) < 8u) sq_steps((uint16_t)(u - sq_org[SQ_IIX]), 0u, u == sq_org[SQ_IIX]);
        if ((uint16_t)u == SQI.tick) break;
        u++;
    }
}
static UI_LATE uint32_t sq_chord(const sq_pat_t *q, const sq_step_t *t, int8_t *o)   /* (1.02) CHORD step t's
                                          * notes as offsets from the base (o, lowest first): their count; none
                                          * recorded yet: the base note alone */
{
    const uint8_t *c = (uint8_t)t->off < SQ_CHN ? q->ch[(uint8_t)t->off] : (uint8_t)t->off == SQ_CHN ? sq_cech()
                       : (const uint8_t *)0;   /* (SQ_CHN: CHORD ENTRY's notes chosen, sq_cel, 1.05) */
    uint32_t n = 0;
    while (c && n < SQ_CHV && c[n]) { o[n] = (int8_t)(c[n] - 64u); n++; }
    if (!n) o[n++] = 0;
    return n;
}
static UI_LATE uint32_t sq_input(uint8_t *in)   /* audio interrupt: SQI's input notes now, lowest first (16 at
                                          * most): the keys and MIDI notes held; none held, KEY LATCH ON: those
                                          * latched. Their count (0: none) */
{
    uint32_t i, w, n = 0, b, m[4], any = 0;
    for (w = 0; w < 4u; w++) any |= m[w] = SQI.keys[w] | SQI.midi[w];
    if (!any && !(SQPI.pad & SQ_NLAT))   /* (none held, KEY LATCH ON: the notes held last, latched; none: the BASE NOTE) */
        for (w = 0; w < 4u; w++) m[w] = SQI.lat[w];
    for (w = 0; w < 4u; w++) {
        b = m[w];
        for (i = 0; b && n < 16u; i++, b >>= 1)
            if (b & 1u) in[n++] = (uint8_t)(w * 32u + i);
    }
    return n;
}
/* (1.08) FING POLY into the patch's arpeggiator follows the grip: while such a step's chord sounds (its LENGTH; KEY
 * LATCH ON: latched until the next step with notes) a key struck (or a MIDI note in) makes the chord the input notes
 * now, at once (the block's start, before its ticks), as an arpeggiator played live: the notes no longer held are let
 * go (sq_endp, after the new ones: the engine takes a block's note-ons first, so the arpeggiator never runs empty and
 * goes on in its rhythm), the new ones held to the step's end (or latched). Keys let go change nothing (a chord let
 * go unevenly stays whole). Up to 1.07 the step kept the keys held at its start for its LENGTH: a chord changed
 * during a long step came in at the next bar, and with KEY LATCH ON a chord changed with a finger kept down latched
 * both chords together. SQG.fp: the step's notes (nt[] bits), ended by any other step with notes */
static UI_LATE void sq_play(uint32_t note, uint32_t vel, uint32_t len, uint32_t lv);
static UI_LATE void sq_follow(void)      /* audio interrupt, a block's start, SQI on: a note struck */
{
    uint8_t in[16];
    uint32_t i, k, n, lv = 0, off = 0, f = 0;
    for (i = 0; i < SQ_NT; i++)          /* (the step's notes still held: their state and end) */
        if ((SQG.fp >> i & 1u) && (SQI.nt[i].live == SQ_LV || SQI.nt[i].live == SQ_LVL)) {
            f |= 1u << i;
            lv = SQI.nt[i].live;
            off = SQI.nt[i].off;
        }
    SQG.fp = f;
    if (!f || !sq_af() || !(n = sq_input(in))) return;   /* (none, the arpeggiator not fed, no input: kept) */
    for (i = 0; i < SQ_NT; i++)          /* (those not in the grip: let go after the new ones' note-ons) */
        if (f >> i & 1u) {
            for (k = 0; k < n && in[k] != SQI.nt[i].note; k++) {}
            if (k == n) { SQI.nt[i].live = SQ_LVE; SQG.fp &= ~(1u << i); }
        }
    for (k = 0; k < n; k++) {            /* (the new ones, to the step's end) */
        for (i = 0; i < SQ_NT && !((SQG.fp >> i & 1u) && SQI.nt[i].note == in[k]); i++) {}
        if (i < SQ_NT) continue;
        sq_play(in[k], SQG.fv, (uint16_t)(off - SQI.tick), lv | 4u);
        for (i = 0; i < SQ_NT && !(SQI.nt[i].live == lv && SQI.nt[i].note == in[k]); i++) {}
        if (i < SQ_NT) SQG.fp |= 1u << i;
    }
    sq_endp();
}
static UI_LATE void sq_fire(const sq_step_t *p)   /* audio interrupt: step p played now */
{
    uint32_t i, w, n, ev = p->ev & 15u, lv;
    uint8_t in[16];
    int8_t co[SQ_CHV];
    n = sq_input(in);                    /* the input notes, lowest first (16 at most) */
    if (!n) {                            /* (none held, none latched: the BASE NOTE, so a sequencer started from its page
                                          * sounds; OFF: silent. An input note replaces it at the next step) */
        if (!SQ_BASEI) return;
        in[n++] = SQ_BASEI;
    }
    w = SQ_LEN(p->la & 31u);             /* (the length in ticks; over its block and the loop's end tied, at most the
                                          * pattern's LENGTH: there the step plays again) */
    if (w > SQ_STEPS(SQPI.lc) * 8u) w = SQ_STEPS(SQPI.lc) * 8u;
    if (ev == SQ_CHORD) {                /* (1.02: the chord on the base, as NOTE's offset; out of range: left out) */
        const uint32_t k = sq_chord(&SQPI, p, co), bs = in[0];
        for (i = n = 0; i < k; i++)
            if (bs + (uint32_t)(int32_t)co[i] < 128u) in[n++] = (uint8_t)(bs + (uint32_t)(int32_t)co[i]);
    } else if (ev != SQ_POLY) {          /* (NOTE: the base + OFFSET; FING n: a finger not held, skipped; 1.18C31: its
                                          * note + OFFSET; out of range, either: skipped) */
        const uint32_t x = ev == SQ_NOTE ? in[0] + (uint32_t)(int32_t)p->off
                           : ev - SQ_FING < n ? in[ev - SQ_FING] + (uint32_t)(int32_t)p->off : 128u;
        in[0] = (uint8_t)x;
        n = x < 128u;
    }
    if (!n) return;                      /* (none: a latched chord stays) */
    lv = sq_af() ? 4u : 0u;              /* (1.02: the patch's arpeggiator fed, PATCH ARP ON) */
    for (i = 0; i < SQ_NT; i++)          /* (a chord latched: ended by this step's notes, those it plays again kept) */
        if (SQI.nt[i].live == SQ_LVL) SQI.nt[i].live = SQ_LVE;
    lv |= lv && (ev == SQ_POLY || ev == SQ_CHORD) && (SQPI.pad & (SQ_PARP | SQ_NLAT)) == SQ_PARP ? SQ_LVL : SQ_LV;   /* (KEY
                                          * LATCH ON: a chord fed to the arpeggiator held past its LENGTH, as the keys
                                          * latched, until the next step with notes) */
    SQG.fp = 0;                          /* (1.08: a FING POLY step's chord no longer follows the grip) */
    for (i = 0; i < n; i++) sq_play(in[i], p->vel, w, lv);
    if (ev == SQ_POLY && (lv & 4u)) {    /* (1.08: FING POLY into the arpeggiator: its chord follows the grip,
                                          * sq_follow) */
        SQG.fv = p->vel;
        for (i = 0; i < SQ_NT; i++)
            for (w = 0; w < n; w++)
                if (SQI.nt[i].live == (lv & 3u) && SQI.nt[i].note == in[w]) SQG.fp |= 1u << i;
    }
}
/* ------------------------------------------------------ the drum machine --- */
/* The drum sequencer's runtime: ten synthesized voices (CH and OH share one: CH chokes OH),
 * each computed from a few states, no tables, no samples; mixed dry and centred into the output after the engine
 * (dr_render), at MASTER and the drums' level. Nothing runs while none sounds. The voices' sounds are this project's
 * own: sines with an exponential pitch sweep (BD, the toms), a tone and high-passed noise (SD), two sines pinged (RS),
 * noise bursts through a band-pass (CP), six square waves at inharmonic ratios and noise, high-passed (the hats, the
 * cymbal), two squares low-passed (CB) */
#define DR_NV 10u                        /* the voices: BD SD LT MT HT RS CP HH (CH, OH) CY CB */
#define DR_HH 7u
/* (1.18C42) DR_HALF: the voices at half the output's rate (22.05 kHz), their mix brought to the full
 * rate by an upsampler (DR_UP) before COMP; at the full rate only the kick while it is driven (DR_KDRIVE) and the
 * sampled hat. Each filter's coefficient is the full rate's two steps in one (c (2 - c) for a one-pole), so the
 * voices' bands stay as they were below ~10 kHz; above it the half rate has nothing of its own (the upsampler's
 * images aside). 0: every voice at the full rate, as 1.18C41 */
#ifndef DR_HALF
#define DR_HALF 1
#endif
#ifndef DR_UP
#define DR_UP 1                          /* the upsampler: 1 linear (the cheapest; its images of 5-11 kHz at 11-17 kHz,
                                          * 7-18 dB down: some air left above 11 kHz), 2 four points (cubic: the images
                                          * 8-28 dB down), 3 a half-band of 15 taps (flat to 8 kHz, the images 33-40 dB
                                          * down: nothing above ~11 kHz) */
#endif
#ifndef DR_HATF
#define DR_HATF 0                        /* 1: the hats and CY at the full rate (their cost twice, their top as 1.18C41's) */
#endif
#ifndef DR_KDRIVE
#define DR_KDRIVE 1                      /* (1.18C42) LEVEL above 100 drives the kick alone (its voice, before the mix and
                                          * COMP); 0: the whole bus after COMP, as 1.18C41 */
#endif
#if DR_HALF
#define DR_CHUNK 32u                     /* the frames rendered at a time (the buffers: drb) */
#define DR_HK (DR_CHUNK / 2u)            /* a chunk's frames at the half rate */
#define DR_MF 0x80u                      /* dr_voice_t m: the voice at the full rate (the kick driven, the sampled hat) */
#if DR_UP == 1
#define DR_UPH 1u                        /* the upsampler's history (half-rate frames) */
#elif DR_UP == 2
#define DR_UPH 3u
#else
#define DR_UPH 7u
#define DR_HB1 0.63356919f               /* (the half-band's taps 1, 3, 5, 7 frames off its centre: remez, pass 8 kHz,
                                          * stop 14.05 kHz, their sum 1/2) */
#define DR_HB2 -0.17732844f
#define DR_HB3 0.07310097f
#define DR_HB4 -0.02934172f
#endif
#else
#undef DR_KDRIVE
#define DR_KDRIVE 0                      /* (DR_HALF 0: the drums as 1.18C41, the bus driven) */
#define DR_CHUNK 16u                     /* the frames rendered at a time (three buffers on the stack) */
#endif
typedef struct {
    float a, ad;                         /* the amplitude and its factor a frame */
    float b, bd;                         /* the second envelope (pitch sweep, noise, click or attack) and its factor */
    float ph, ph2;                       /* the phases (0 .. 1); CP: ph counts the frames */
    float z1, z2;                        /* the filters' states (BD, the toms: the click's level, its factor a frame) */
    float e;                             /* ATTACK: the part of the fade-in still missing (1 .. 0), 0 none */
    uint8_t m;                           /* the kit at the hit: bit 0 the 909 voice, bits 1-5 ATTACK / CLICK */
    uint8_t tune, tone;                  /* the kit's TUNE and TONE at the hit (DR_SEMI's index) */
    int8_t pan;                          /* the kit's PAN at the hit */
} dr_voice_t;
typedef struct {                         /* an instrument: its voice's figures (flash) */
    float f0, f1;                        /* the frequencies, Hz (the tone, a second tone or the filter) */
    float sw, tb;                        /* the pitch sweep's depth (x f0 at the hit), the second envelope's time, s */
    float ta, g;                         /* the decay (-60 dB, s) at length x 1, the level */
} dr_ins_t;
static const dr_ins_t DR_INS[DR_NI] = {   /* (0.62: SD, CP +4 dB, the hats +4.5 dB, RS, CY, CB +2 dB against BD: the
                                          * user found them quiet beside the kick) */
    /*      f0       f1     sw     tb      ta     g */
    /* BD */ {48.f,    0.f,   5.f,   0.028f, 0.50f, 1.00f},
    /* SD */ {180.f,   1900.f, 0.f,  0.20f,  0.10f, 1.27f},   /* f1: the noise's high-pass; tb: the noise's decay */
    /* LT */ {88.f,    0.f,   0.7f,  0.10f,  0.45f, 0.85f},
    /* MT */ {128.f,   0.f,   0.6f,  0.09f,  0.38f, 0.80f},
    /* HT */ {180.f,   0.f,   0.55f, 0.08f,  0.32f, 0.75f},
    /* RS */ {1650.f,  460.f, 0.f,   0.004f, 0.045f, 0.88f},  /* tb: the click */
    /* CP */ {1150.f,  0.f,   0.f,   0.30f,  0.012f, 1.50f},  /* f0: the band-pass; ta: a burst; tb: the tail */
    /* CH */ {7200.f,  0.f,   0.f,   0.f,    0.055f, 8.4f},   /* f0: the high-pass (the level +4.5 dB since 0.62) */
    /* OH */ {7200.f,  0.f,   0.f,   0.f,    0.42f, 7.7f},
    /* CY */ {3600.f,  0.f,   0.f,   0.f,    1.30f, 3.8f},
    /* CB */ {540.f,   800.f, 0.f,   0.035f, 0.30f, 0.69f},   /* tb: the attack's fast part */
};
static const dr_ins_t DR_909[DR_HT + 1] = {   /* the 909 voices (the kit's MODEL): BD its own voice (dr_bd909: a
                                          * soft-clipped triangle, a slow deep pitch envelope, a pulse and noise at
                                          * the attack, shorter); SD two tones and brighter,
                                          * louder noise (dr_render); the toms a little higher, shorter, a noisy
                                          * attack (the click) */
    /*      f0       f1     sw     tb      ta     g */
    /* BD */ {50.f,    0.f,   4.5f,  0.14f,  0.40f, 0.90f},   /* sw: x f0 at the hit, linear in the envelope */
    /* SD */ {195.f,   2400.f, 0.f,  0.17f,  0.075f, 1.27f},
    /* LT */ {100.f,   0.f,   0.9f,  0.06f,  0.38f, 0.85f},
    /* MT */ {144.f,   0.f,   0.8f,  0.055f, 0.32f, 0.80f},
    /* HT */ {202.f,   0.f,   0.75f, 0.05f,  0.27f, 0.75f},
};
static const uint8_t DR_VOICE[DR_NI] = {0, 1, 2, 3, 4, 5, 6, DR_HH, DR_HH, 8, 9};
#define DR_SWEPT(i) ((i) == DR_BD || ((i) >= DR_LT && (i) <= DR_HT))   /* BD, the toms (instrument = voice) */
static const float DR_SEMI[DR_KN + 1] = {   /* 2^((i - 12) / 12): the kit's TUNE, TONE and DECAY (squared) */
    0.5f, 0.52973154f, 0.56123102f, 0.59460356f, 0.62996052f, 0.66741993f, 0.70710678f, 0.74915354f, 0.79370053f,
    0.84089642f, 0.89089872f, 0.94387431f, 1.f, 1.05946309f, 1.12246205f, 1.18920712f, 1.25992105f, 1.33483985f,
    1.41421356f, 1.49830708f, 1.58740105f, 1.68179283f, 1.78179744f, 1.88774863f, 2.f};
static const float DR_LMUL[DR_NLEN] = {0.15f, 0.25f, 0.4f, 0.6f, 1.f, 1.4f, 2.f, 3.f};   /* the lengths' factors */
static const float DR_ACCG[DR_NACC] = {0.6f, 0.8f, 1.f};   /* the accents' levels */
static const float DR_MHZ[6] = {205.3f, 304.4f, 369.6f, 522.7f, 540.f, 800.f};   /* the metal's squares, Hz */
#define DR_CP_GAP 0.0095f                /* CP: the bursts' spacing, s (three, then the tail) */
static struct {
    volatile uint8_t run;                /* main loop: on */
    volatile uint8_t step;               /* the step sounding (the audio interrupt's, latency included; the screen) */
    volatile uint16_t prev;              /* main loop: an instrument to hear now (+ 1, the accent << 4, its LENGTH ^
                                          * DR_LEN1 << 6: 0 as shipped; dr_pv), 0 none */
    uint8_t on;                          /* the audio interrupt's: running */
    volatile uint16_t hit;               /* the instruments struck at the step sounding (the screen) */
    uint16_t act;                        /* the voices sounding, a bit a voice */
    uint16_t ck;                         /* the voices with a click sounding (BD, the toms: the noise is needed) */
#if DR_HALF
    uint16_t fm;                         /* the voices at the full rate (DR_MF; dr_hit) */
#endif
    volatile float gain;                 /* main loop: the level (set.dr_lev) as a factor */
    uint32_t nz;                         /* the noise's state */
    uint32_t mph[6];                     /* the metal's phases */
    dr_voice_t v[DR_NV];
} dr;
/* the kit's SEND (0.88): the instruments sending, after their level and pan, before the bus's COMP
 * and DRIVE, summed into one stereo bus that part A's delay / reverb takes as it takes part B's send (sy_fx.c
 * sy_fx_render; synth.h dr_w, dr_b, dr_on), the next block (the synth's effects run before the drums). The dry path
 * as before. Nothing of it runs while every SEND is 0 */
static struct {
    float b[2u * SY_BLOCK];              /* the bus: L, then R, in the space's units (S.dr_b) */
    float ig[DR_NI];                     /* each instrument's factor into it (dr_level; 0: SEND 0) */
    uint8_t vi[DR_NV];                   /* the instrument each voice sounds (at its hit) */
} ds PLAT_POOL;
#if DR_HALF
static struct {                          /* (1.18C42) dr_render's state between blocks: the upsampler's histories (the
                                          * mix, its side, the send's two sides: DR_UPH frames of the half rate each) */
    float h[DR_UPH], hs[DR_UPH], hb[2][DR_UPH];
    uint8_t hsl, hbl, upz;               /* the side's, the send's history still to be played out; the histories 0 */
} drb PLAT_POOL;
#endif
/* the clock both sequencers share: ticks at 4x (1/32 of a step at 1x) from the tempo, the fraction carried; a
 * sequencer at time division d ticks on each 2^(4 - d)th of them, so the two stay on one grid whatever their division.
 * The drums' ticks are taken DR_LAT blocks after the clock's (the engine plays a note that much later: SY_OPT_LATENCY)
 * from a short history (ck.q) */
#define CK_SH(div) (4u - (div))           /* a time division's ticks: the clock's >> CK_SH */
#define CK_Q 3u
static struct {
    uint8_t on;                          /* running (either sequencer on) */
    uint8_t blk;                         /* the blocks counted (ck.q's place) */
    uint8_t mrun;                        /* CLOCK MASTER: a START sent, no STOP since */
    uint8_t pre;                         /* (1.18C40) started with a count (pc_tk): CLOCK MASTER's START waits for 0 */
    float frac;
    float mfrac;                         /* CLOCK MASTER: the MIDI clock's fraction of a tick (24 a beat), carried */
    uint32_t mt;                         /* the clock's ticks since it started */
#ifdef HYB_MULTI
    int8_t ofs;                          /* (1.19.4) CLK OFFSET as taken (ms; 0: INTERNAL, MASTER) */
    float adj;                           /* the shift still to make, s (+ earlier): at most a quarter of each block */
#endif
    struct { uint32_t m0; uint32_t n; } q[CK_Q];   /* each of the last blocks' ticks: the first, how many */
} ck;
#ifdef HYB_MULTI
/* (1.19.5) CLK LOCK (GLO > MIDI, ON as shipped; set.ck_fl): with CLOCK EXTERNAL the shared clock (ck) is steered onto
 * the master's place, not only its tempo. The master's place: its ticks (24 a beat) counted from the first after
 * START / CONTINUE (both a start from 0 here: the sequencers start from their first step on either; Song Position
 * Pointer is not taken), taken modulo a beat. Each tick, at the block it came in (the blocks' frames the time base;
 * Bluetooth's ticks come spaced as sent already, radio.c's ble_transport by their BLE MIDI timestamps), gives the
 * error: the clock's place at that block's start less the master's, in s, plus CLK OFFSET (wanted: the clock ahead by
 * -CLK OFFSET), wrapped to +- half a beat. The error's estimate (xk.e) takes each tick's with weight XK_A (the jitter of
 * the ticks' arrival averaged) and follows the corrections made between ticks exactly; each block the clock runs
 * faster or slower by XK_K x the estimate plus an integral (XK_KI; the tempo estimate's own bias, at most XK_IMAX),
 * at most 25 % (CLK OFFSET's limit): no tick is skipped or doubled, the place never goes back, a large error (a tempo
 * jump) is pulled in at the limit; nothing ever jumps (no resync but START's, as before). A tick lost on the way (a
 * gap of 2 .. 5 spacings, round within 0.3, between normal ones: two before, two after) is counted in; while that is
 * open no error is taken. Off (or no START seen since the clock came in, or the clock gone): 1.19.4's free run at the
 * tempo with CLK OFFSET's one-off shift (ck.adj) */
#define XK_K 4.f                         /* the loop's gain, 1/s: a 10 ms error 4 % faster / slower, ~1/e in 0.25 s */
#define XK_KI 1.f                        /* the integral's gain, 1/s^2 */
#define XK_IMAX 0.02f                    /* the integral at most (a tempo off by 2 %) */
#define XK_A 0.125f                      /* a tick's error's weight in the estimate */
#define XK_PN 8u                         /* the ticks after a long gap that decide whether ticks were lost */
static struct {
    uint8_t st;                          /* 0 no START known, 1 a START / CONTINUE in (its first tick to come), 2 counting */
    uint8_t lk;                          /* the loop's estimate valid (the clock running, a tick taken) */
    uint8_t pend;                        /* ticks maybe lost in a long gap (0 none): decided after XK_PN more ticks */
    uint8_t pn;                          /* those ticks taken */
    uint8_t cnt;                         /* the master's ticks since its first after START, modulo a beat (24) */
    uint32_t t, tl;                      /* the blocks' frames, at this block's start; at the last tick */
    float per;                           /* the ticks' spacing, frames (smoothed, 1/8) */
    float oph;                           /* the clock's place at this block's start in its beat: its ticks (128 a beat) */
    float e;                             /* the error's estimate, s (+: ahead of the master's place + CLK OFFSET) */
    float ri;                            /* the integral, s / s (+: slower) */
    float ps;                            /* the errors of the ticks after a long gap, summed (counted as they came) */
} xk;                                    /* (RAM, 32 B: the pool's headroom is at its floor) */
static UI_LATE void xk_rt(uint32_t st)   /* audio interrupt, CLOCK EXTERNAL: a clock or transport message taken */
{
    if (st == 0xFAu || st == 0xFBu) { xk.st = 1; xk.lk = 0; xk.tl = xk.t; return; }
    if (st == 0xFCu) { xk.st = xk.lk = 0; return; }
    if (st != 0xF8u || !xk.st) return;
    if (xk.st == 1u) {                   /* (the first after START: the master's 0) */
        xk.st = 2;
        xk.cnt = 0;
        xk.pend = 0;
    } else {
        const float g = (float)(xk.t - xk.tl), r = xk.per > 0.f ? g / xk.per : 1.f, k = (float)(int32_t)(r + 0.5f);
        xk.cnt = (uint8_t)((xk.cnt + 1u) % 24u);
        if (xk.pend && (r < 0.5f || r > 1.5f)) xk.pend = 0;   /* (no even run after it: a tempo change, no loss) */
        else if (!xk.pend && xk.lk && r > 1.8f && r < 5.2f && r - k < 0.2f && k - r < 0.2f) {   /* (a gap of k
                                          * spacings: k - 1 ticks lost? the next ticks' errors decide) */
            xk.pend = (uint8_t)((uint32_t)k - 1u);
            xk.pn = 0;
            xk.ps = 0.f;
        }
        if (!xk.pend) xk.per = xk.per > 0.f ? xk.per + (g - xk.per) * 0.125f : g;   /* (held while one is open: the
                                          * master's tempo before it) */
    }
    xk.tl = xk.t;
    if (ck.on && (int32_t)ck.mt >= 0) {  /* the error: the clock's place at this block's start less the master's (its
                                          * tick taken at that block) */
        const float bpm = S.bpm_now > 1.f ? S.bpm_now : (float)S.patch[128 + 16] + 63.f;   /* (sq_block's) */
        float d = xk.oph - (float)xk.cnt * (16.f / 3.f), e;
        if (d >= 64.f) d -= 128.f; else if (d < -64.f) d += 128.f;
        e = d * 15.f / (32.f * bpm) + (float)set.ck_ofs * 1e-3f;
        if (!xk.lk) { xk.e = e; xk.ri = 0.f; xk.lk = 1; }
        else if (!xk.pend) xk.e += (e - xk.e) * XK_A;
        else if (xk.ps += e - xk.e, ++xk.pn >= XK_PN) {   /* (their errors as counted, against the estimate (which
                                          * meanwhile follows the clock's tempo against the master's before the gap,
                                          * sq_block): the gap's ticks lost if they are later by more than (pend - 0.3)
                                          * ticks of 60 / 24 / bpm s) */
            if (xk.ps * (1.f / (float)XK_PN) > ((float)xk.pend - 0.3f) * 2.5f / bpm)
                xk.cnt = (uint8_t)((xk.cnt + xk.pend) % 24u);
            xk.pend = 0;
        }
    }
}
#endif
static UI_LATE float dr_decay(float t)   /* the factor a frame for -60 dB in t seconds (~exp(-6.9 / (t fs)), > 0) */
{
    float x = 6.9078f * S.ifs / (t > 1e-4f ? t : 1e-4f);
    return 1.f / (1.f + x + 0.5f * x * x);
}
/* DRUM DRIVE: LEVEL above DR_LCLEAN drives the drums' bus (their mix, before it is added to
 * the synth) into a saturator, the level that of 100, a makeup after it; up to DR_LCLEAN nothing of it runs (the
 * output as before, to the bit). 1.18C42 (DR_KDRIVE): the kick alone is driven (its voice at the
 * full rate, before its pan, send and the mix; COMP after it on the whole kit), every other instrument at the level of
 * 100, clean; the strip says KICK DRIVE. The curve (GLO > DRUM KIT > BUS: DRIVE, kept with the kit: drk.drv): WARM a cubic soft
 * clip (the 909 BD's), TANH the engine's rational tanh (sy_math.h), then the engine's distortion types as stateless
 * functions (sy_fx.c distortion; sy_dist_tab.c): LIGHT, SOFT, MIDDLE, HARD the measured static curves (fitted),
 * DIGITAL the hard clip, SHAPER the sine folder, BIT RED the quantiser. (Not the voices' saturation types: their curve
 * is a table built per patch, sy_sat_build.) The drive d = (LEVEL - 100) / 27: WARM and TANH x (1 + 3d) into the
 * curve (+12 dB at 127), the engine's at intensity 127 d. The makeup holds a decaying sine's RMS (a hit's) as it was
 * dry, + 2 dB at most (1 + 0.25 d): the loudness grows gently (the demo beat's RMS at 127: +0.4 .. +2.9 dB). Set by the main loop (dr_level), read per chunk by
 * dr_render (dr_drive) */
#include "sy_math.h"                     /* (the engine's float helpers: sy_tanh, sy_sin2pi, sy_floori) */
extern const float SY_DIST5_G[128], SY_DIST5_L[128];
#include "sy_cln_d14s.h"                 /* (the engine's curves 1-4: shapes and scales, sy_dist_tab.c; sy_d14_s) */
/* (1.19.2) the app's exp2, log2, tanpi, db2lin, lin2db: the engine's shared flash functions (sy_tables.c under
 * SY_HOT_RAM, the engine's units' flag: the same arithmetic as sy_math.h's inline ones, so the same bits) instead of
 * an inlined copy at every site (-892 B). Only code in the flash calls them (the drums' compressor, DRIVE's setup, the
 * hat's filters); sy_sin2pi stays inline (DRIVE's curve, sample by sample). */
float sy_exp2_f(float x);
float sy_log2_f(float x);
float sy_tanpi_f(float f);
#define sy_exp2(x) sy_exp2_f(x)
#define sy_log2(x) sy_log2_f(x)
#define sy_tanpi(x) sy_tanpi_f(x)
#define sy_db2lin(x) sy_exp2_f((x) * 0.166096405f)
#define sy_lin2db(x) (6.020599913f * sy_log2_f(x))
enum { DV_WARM, DV_TANH, DV_LIGHT, DV_SOFT, DV_MIDDLE, DV_HARD, DV_DIGITAL, DV_SHAPER, DV_BITRED, DV_N };
static struct {
    volatile uint8_t on;                 /* LEVEL above 100: dr_drive runs */
    uint8_t type;                        /* DV_* (set last: the parameters below are the type's or the one before's) */
    float pre, mk;                       /* the gain into the curve, the makeup */
    const float *ts;                     /* the distortion curves (LIGHT .. HARD): the type's shapes [2][CLN_D14_N] */
    float ca[2], cb[2], ka[2], kb[2];    /* per half: the two intensities' out scales (their mix folded in), in scales */
    float g, lim;                        /* DIGITAL: the slope, the limit; SHAPER: the amplitude (g), the factor (lim);
                                          * BIT RED: the step (g), its inverse (lim) */
    volatile uint8_t comp;               /* DRUM COMP (DC_*; 0 OFF: nothing of it runs) */
    uint8_t cidle;                       /* the drums were silent: the detector starts again */
    float env, cg;                       /* the compressor's detector (peak, full scale 1) and its last gain */
} dv PLAT_POOL;
/* DRUM COMP (GLO > DRUM KIT > BUS: COMP, kept with the kit: drk.drv bits 4-5): a feed-forward compressor on
 * the drums' bus (1.18C42: the kick driven in it, DR_KDRIVE; before: before the bus's drive). The detector the chunk's peak (DR_CHUNK frames; both sides, linked), attack and
 * release at the chunk's rate; the gain (env / threshold)^(1 / ratio - 1) above the threshold, ramped over the next
 * chunk, then the makeup. LIGHT a gentle 2:1, PUNCH 4:1 with a 10 ms attack (the transients through) and 100 ms
 * release, HEAVY 8:1 fast. Nothing of it runs while OFF or while no hit sounds */
enum { DC_OFF, DC_LIGHT, DC_PUNCH, DC_HEAVY, DC_N };
typedef struct { float thr, slope, att, rel, mk; } dc_pre_t;   /* threshold, 1 / ratio - 1, attack and release (s),
                                          * makeup (a factor) */
static const char *const DV_NAME[DV_N] = {"WARM", "TANH", "LIGHT", "SOFT", "MIDDLE", "HARD", "DIGITAL", "SHAPER",
                                          "BIT RED"};   /* DRIVE's curves (GLO > DRUM KIT's BUS, dr_level; the engine's distortion
                                          * types' names from LIGHT on) */
static const char *const DC_NAME[DC_N] = {"OFF", "LIGHT", "PUNCH", "HEAVY"};   /* COMP's presets (the BUS) */
static const dc_pre_t DC_PRE[DC_N] = {
    {1.f, 0.f, 0.01f, 0.1f, 1.f},
    {0.5f, -0.5f, 0.015f, 0.15f, 1.26f},     /* LIGHT: -6 dB, 2:1, 15 / 150 ms, +2 dB */
    {0.35f, -0.75f, 0.010f, 0.10f, 1.58f},   /* PUNCH: -9 dB, 4:1, 10 / 100 ms, +4 dB */
    {0.2f, -0.875f, 0.003f, 0.06f, 2.24f},   /* HEAVY: -14 dB, 8:1, 3 / 60 ms, +7 dB */
};
static UI_LATE void dr_comp(float *d, float *sd, uint32_t nn, uint32_t side)   /* audio interrupt: a chunk of the
                                          * drums' mix through the compressor */
{
    const dc_pre_t *c = &DC_PRE[dv.comp < DC_N ? dv.comp : DC_OFF];
    float pk = 0.f, env = dv.env, g0 = dv.cg, g1, dg;
    uint32_t k;
    if (dv.cidle) { dv.cidle = 0; env = 0.f; g0 = 1.f; }
    for (k = 0; k < nn; k++) {
        const float a = sy_absf(d[k]) + (side ? sy_absf(sd[k]) : 0.f);   /* (|L| or |R|, the larger: |d| + |sd|) */
        if (a > pk) pk = a;
    }
    env += (pk > env ? 1.f - sy_exp2(-1.4427f * (float)nn * S.ifs / c->att)
                     : 1.f - sy_exp2(-1.4427f * (float)nn * S.ifs / c->rel)) * (pk - env);
    g1 = env > c->thr ? sy_exp2(c->slope * sy_log2(env / c->thr)) : 1.f;
    dv.env = env;
    dv.cg = g1;
    dg = (g1 - g0) / (float)nn;
    for (k = 0; k < nn; k++) {
        const float g = c->mk * (g0 + dg * (float)(k + 1u));
        d[k] *= g;
        if (side) sd[k] *= g;
    }
}
static inline __attribute__((always_inline)) float dv_curve(uint32_t t, float x)   /* the curve t at x (the gain into
                                          * it included, no makeup; inlined: a loop per curve, dr_drive) */
{
    x *= dv.pre;
    switch (t) {
    case DV_WARM:
        x = x > 1.5f ? 1.5f : x < -1.5f ? -1.5f : x;
        return x * (1.f - x * x * (1.f / 6.75f));
    case DV_TANH:
        return sy_tanh(x);
    case DV_DIGITAL:
        return sy_clampf(dv.g * x, -dv.lim, dv.lim);
    case DV_SHAPER:
        return dv.g * sy_sin2pi(dv.lim * sy_clampf(x, -1.f, 1.f));
    case DV_BITRED:
        return dv.g * (float)sy_floori(x * dv.lim + 0.5f);
    default: {                           /* (LIGHT .. HARD: as sy_fx.c, the shape of x's half at both scales) */
        const float v = sy_clampf(x, -1.f, 1.f), z = sy_absf(v);
        const int h = v < 0.f;
        const float *S = dv.ts + h * CLN_D14_N;
        return dv.ca[h] * sy_d14_s(S, dv.ka[h] * z) + dv.cb[h] * sy_d14_s(S, dv.kb[h] * z);
    }
    }
}
static UI_LATE void dr_drive(float *d, float *sd, uint32_t nn, uint32_t side)   /* audio interrupt: a chunk of the
                                          * drums' mix (d; side: the side sd too, each side driven) through the curve */
{
    const float mk = dv.mk;
    uint32_t k;
#define DV_LOOP(T)                                                                                  \
    if (!side)                                                                                      \
        for (k = 0; k < nn; k++) d[k] = mk * dv_curve(T, d[k]);                                     \
    else                                                                                            \
        for (k = 0; k < nn; k++) {                                                                  \
            const float l = mk * dv_curve(T, d[k] + sd[k]), r = mk * dv_curve(T, d[k] - sd[k]);     \
            d[k] = 0.5f * (l + r);                                                                  \
            sd[k] = 0.5f * (l - r);                                                                 \
        }
    switch (dv.type) {                   /* (a loop of its own per curve) */
    case DV_WARM: DV_LOOP(DV_WARM); break;
    case DV_TANH: DV_LOOP(DV_TANH); break;
    case DV_DIGITAL: DV_LOOP(DV_DIGITAL); break;
    case DV_SHAPER: DV_LOOP(DV_SHAPER); break;
    case DV_BITRED: DV_LOOP(DV_BITRED); break;
    default: DV_LOOP(DV_LIGHT); break;
    }
#undef DV_LOOP
}
static inline __attribute__((always_inline)) float dr_swet(uint32_t v)   /* the send law's wet at SEND v (sy_fx.c
                                          * send_law: its points every 16, 1 from 96; 0 at 0) */
{
    static const float W[7] = {0.f, 0.056f, 0.145f, 0.258f, 0.415f, 0.675f, 1.f};
    const uint32_t i = v >> 4;
    return i >= 6u ? 1.f : W[i] + (float)(v & 15u) * (1.f / 16.f) * (W[i + 1u] - W[i]);
}
static UI_LATE void dr_sends(void)       /* main loop: the kit's SENDs to the bus and part A's space (dr_level). Each
                                          * instrument goes into the bus at its send law's wet, the drums' LEVEL (up
                                          * to 100) and 1 / the part's route (sy_fx.c ROUTE_G, -0.86: the space's
                                          * output goes through it), so its echo stands to the dry drum as a patch's
                                          * to its own dry signal at that send; the space takes the bus as B's send,
                                          * at the largest wet (S.dr_w). A change wakes the effects at rest. (S.dr_w
                                          * above 0 while a factor is: the audio interrupt clears the bus by it) */
{
    float w[DR_NI], dw = 0.f;
    const float was = S.dr_w, k = dr.gain * (1.f / -0.86f);
    uint32_t i;
#ifdef __clang__
#define DS_NO_UNROLL _Pragma("clang loop unroll(disable)")   /* (loops kept loops: the flash's bytes) */
#else
#define DS_NO_UNROLL
#endif
    DS_NO_UNROLL for (i = 0; i < DR_NI; i++) {
        w[i] = dr_swet(drk.k[i].snd);
        if (w[i] > dw) dw = w[i];
    }
    if (dw > 0.f) S.dr_w = dw;
    DS_NO_UNROLL for (i = 0; i < DR_NI; i++) ds.ig[i] = w[i] * k;
    S.dr_w = dw;
    S.dr_b = ds.b;
    if (dw != was) S.fx_wake = 1;
}
static UI_LATE void dr_level(void)       /* main loop: LEVEL (set.dr_lev) and DRUM DRIVE (drk.drv) to the drums */
{
    const uint32_t lv = set.dr_lev > DR_LCLEAN ? DR_LCLEAN : set.dr_lev, t = (drk.drv & 15u) < DV_N ? drk.drv & 15u : DV_WARM;
    float dd, it, w, sw = 0.f, sd = 0.f;
    uint32_t i, j, k;
    dr.gain = (float)lv * (float)lv * 1e-4f;   /* (as before) */
#ifdef HYB_MULTI
    if (ln_mute & 4u) dr.gain = 0.f;     /* (the launcher's MUTE) */
#endif
    if (fv_aud) dr.gain = 0.f;           /* (1.18C33: the favorites editor's audition) */
    dr_sends();
    if ((uint32_t)(drk.drv >> 4) != dv.comp) {   /* (DRUM COMP: from a fresh detector) */
        dv.cidle = 1;
        dv.comp = (uint8_t)(drk.drv >> 4 < DC_N ? drk.drv >> 4 : DC_OFF);
    }
    if (set.dr_lev <= DR_LCLEAN) { dv.on = 0; return; }
    dd = (float)(set.dr_lev - DR_LCLEAN) * (1.f / (float)(DR_LMAX - DR_LCLEAN));
    it = 127.f * dd;
    k = it >= 112.f ? 7u : (uint32_t)(it * (1.f / 16.f));   /* (the curves' intensities 0, 16, .., 112, 127) */
    w = k == 7u ? (it - 112.f) * (1.f / 15.f) : it * (1.f / 16.f) - (float)k;
    dv.pre = t <= DV_TANH ? 1.f + 3.f * dd : 1.f;
    if (t >= DV_LIGHT && t <= DV_HARD) {
        for (j = 0; j < 2u; j++) {
            dv.ca[j] = (1.f - w) * CLN_D14_A[t - DV_LIGHT][j][k]; dv.cb[j] = w * CLN_D14_A[t - DV_LIGHT][j][k + 1u];
            dv.ka[j] = CLN_D14_K[t - DV_LIGHT][j][k]; dv.kb[j] = CLN_D14_K[t - DV_LIGHT][j][k + 1u];
        }
        dv.ts = CLN_D14_S[t - DV_LIGHT][0];
    } else if (t == DV_DIGITAL) {
        i = (uint32_t)(it + 0.5f);
        dv.g = SY_DIST5_G[i];
        dv.lim = SY_DIST5_L[i];
    } else if (t == DV_SHAPER) {         /* (sy_fx.c's: the amplitude per 16 of intensity, the folds 4 x 1.68 per 32) */
        static const float A[9] = {0.06248f, 0.06054f, 0.05858f, 0.05663f, 0.05466f, 0.05278f, 0.05071f, 0.04879f,
                                   0.04689f};
        dv.g = A[k] + w * (A[k + 1u] - A[k]);
        dv.lim = 4.f * sy_exp2(0.748f * it * (1.f / 32.f));
    } else if (t == DV_BITRED) {         /* (sy_fx.c's quantiser: the step 0.117 at 127, an octave less per 23) */
        dv.g = 0.117f * sy_exp2((it - 127.f) * (1.f / 23.f));
        dv.lim = 1.f / dv.g;
    }
    dv.type = (uint8_t)t;
    for (i = 0; i < 24u; i++) {          /* the makeup: a decaying sine (0.45 down by 1.1 dB a period), its RMS kept
                                          * (0.45: fitted on the demo beat's bus, the RMS at 127 within 0.4 .. 2.9 dB
                                          * of 100's for every curve) */
        const float a = 0.45f * sy_exp2(-0.18f * (float)i);
        for (j = 0; j < 16u; j++) {
            const float x = a * sy_sin2pi((float)j * (1.f / 16.f)), y = dv_curve(t, x);
            sd += x * x;
            sw += y * y;
        }
    }
    dv.mk = sw > 1e-12f ? fm_sqrtf(sd / sw) * (1.f + 0.25f * dd) : 1.f;
    dv.on = 1;
}
static UI_LATE void dr_init(void)
{
    uint32_t i;
    dr.nz = 0x12345679u;
    dr_level();
    for (i = 0; i < 6u; i++) dr.mph[i] = i * 0x2A3B4C5Du;
}
/* The sampled hi-hat (0.98, the kit's MODEL SMP on CH and OH; docs/drum-hat-sample.md): the user's own closed and
 * open hit, generated by tools/drums/mk_hat.py into hat/hat_tab.h (the Bluetooth build takes
 * the smaller hat/hat_tab4.h, HYB_HAT_SMALL; HYB_HAT_FULL the 8-bit one there too; the Bluetooth two-part build the
 * 8-bit one since 1.13, none before). Each hit: its low part at HAT_RATE (8-bit mu-law, or 4 bits with a scale a block of 16) played
 * at TUNE's rate, linearly interpolated, then a 4th-order low-pass; above it white noise through a
 * 4th-order high-pass at the level of the stored envelope (3 ms frames, ramped); both filters follow TUNE. DECAY below
 * 100 %: an exponential fade over both parts; above: from the hit's anchor on, a tail of noise (in the noise band, and
 * band-passed for the low part) at the level the hit had there, down to -60 dB of its peak in (DECAY - 1) x its own
 * decay from there; then the voice ends. At 100 % the hit as stored, ending with it. Without the table (not
 * generated, or HYB_NO_HAT) none of this is built: MODEL stays "--" on the hats and a kit's bit plays the synthesized
 * hat. Its state in the pool (one voice: the hats'; the Bluetooth two-part build: HYB_SPARE) */
/* (1.13) HYB_RT_SPARE (the Bluetooth two-part build: build_variant.py --rt-spare): state its pool has no room for
 * (the sampled hat's, RANDOMIZE's) in the RAM code region's unused end, a NOLOAD section after the RAM code
 * (.rt_spare), zeroed by app_init before anything runs; elsewhere the pool */
#if defined(HYB_RT_SPARE) && !defined(PLAT_HOST)
#define HYB_SPARE __attribute__((section(".rt_spare")))
#else
#define HYB_SPARE PLAT_POOL
#endif
#if !defined(HYB_NO_HAT) && !defined(HYB_HAT_SMALL) && !defined(HYB_HAT_FULL) && defined(FM1B_BLE) && FM1B_BLE
#ifndef HYB_MULTI
#define HYB_HAT_SMALL 1                  /* (the Bluetooth build: the 4-bit table; the 8-bit one does not fit) */
#endif
#endif
#if !defined(HYB_NO_HAT) && defined(HYB_HAT_SMALL) && __has_include("hat_tab4.h")
#define HAT_HAVE 1
#elif !defined(HYB_NO_HAT) && !defined(HYB_HAT_SMALL) && __has_include("hat_tab.h")
#define HAT_HAVE 1
#else
#define HAT_HAVE 0
#endif
#if HAT_HAVE
typedef struct {                         /* a hit (mk_hat.py) */
    uint32_t d, n;                       /* its low part: its first byte in HAT_D, its samples */
    uint16_t e, nf, kt, kp, s, cut;      /* its envelopes' first byte in HAT_E (nf frames of the noise band, then nf of
                                          * the low part); the tail's anchor and the peak (frames); 4 bit: its first
                                          * block's scale in HAT_SB; 1: the sample cut it (its tail from DECAY 100 %
                                          * up), 0: it decays into its floor (from the anchor on, where it is) */
    float pk, ehs, els;                  /* the low part's scale; the envelopes' (a byte b: x 2^(-b / 16), 255 none) */
    float sl, th, tl, te;                /* the tail: the slope before the anchor (log2 a frame), the two levels there,
                                          * its end (log2 below the larger one: -60 dB of the hit's peak) */
} hat_hit_t;
#ifdef HYB_HAT_SMALL
#include "hat_tab4.h"
#else
#include "hat_tab.h"
#endif
#define HAT_G 0.75f                      /* the level (x the kit's LEVEL and the accent's): CH at its peak (5 ms RMS) about
                                          * +2 dB over the synthesized CH, OH the sample's own 7 dB below its CH */
static struct {
    uint8_t on;                          /* the hats' voice plays the sample (its hit's instrument's MODEL) */
    uint8_t h;                           /* the hit (0 closed, 1 open) */
    uint8_t tail;                        /* DECAY above 100 %: the tail from the anchor on */
    float sc;                            /* 4 bit: the block's step */
    uint32_t pos, inc;                   /* the place in the low part and its step an output frame (16.16 samples) */
    uint32_t di, k;                      /* the next sample to decode; the envelopes' frame */
    float s0, s1;                        /* the low part's samples at pos's integer part and after it */
    float eh, ehd, el, eld;              /* the noise band's level (its gain in it) and its step an output frame; the
                                          * low part's tail (sample units x its noise's factor), its step */
    float slf;                           /* the tail's slope (log2 a frame of the envelopes) */
    float fd, fdc;                       /* the fade (DECAY below 100 %), its factor an output frame */
    float gl, gh, nl;                    /* the gains: the low part's, the noise band's (TONE, the noise's factor in
                                          * it), the low tail's noise's factor */
    float c[5][3];                       /* the sections (b0, a1, a2): the low-pass (2), the high-pass (2), the low
                                          * tail's high-pass */
    float z[5][2];
} hs HYB_SPARE;
static UI_LATE float hat_exp2(float x) { return sy_exp2(x); }   /* (out of line: the flash's bytes) */
static UI_LATE float hat_sqrt(float x) { return fm_sqrtf(x); }
static UI_LATE void hat_bq(float *c, float fc, float q, int hp)   /* a Butterworth section at fc Hz (bilinear) */
{
    const float k = sy_tanpi(fc * S.ifs), n = 1.f / (1.f + k / q + k * k);
    c[0] = hp ? n : k * k * n;
    c[1] = 2.f * (k * k - 1.f) * n;
    c[2] = (1.f - k / q + k * k) * n;
}
static UI_LATE float hat_env(uint32_t e, uint32_t k, uint32_t nf, float s)   /* an envelope's level at frame k */
{
    const uint32_t b = k < nf ? HAT_E[e + k] : 255u;
    return b == 255u ? 0.f : s * hat_exp2((float)b * -0.0625f);
}
static UI_LATE void hat_frame(void)      /* frame hs.k begins: the levels' ramps to the next one's */
{
    const hat_hit_t *p = &HAT_H[hs.h];
    const uint32_t k = hs.k;
    const float st = (float)hs.inc * (1.f / (65536.f * (float)HAT_FLS));   /* (a frame's part an output frame) */
    float a = hat_env(p->e, k, p->nf, p->ehs), b = hat_env(p->e, k + 1u, p->nf, p->ehs), la, lb, ta = 0.f, tb = 0.f;
    if (hs.tail && k + 1u >= p->kt) {    /* (the tail: the hit's levels at its anchor, down by slf a frame) */
        tb = hat_exp2(hs.slf * (float)(k + 1u - p->kt));
        ta = k >= p->kt ? tb * hat_exp2(-hs.slf) : 0.f;
    }
    ta *= p->th;
    tb *= p->th;
    a = a > ta ? a : ta;                 /* (the noise band: one noise, the larger level) */
    b = b > tb ? b : tb;
    hs.eh = a * hs.gh;
    hs.ehd = (b - a) * hs.gh * st;
    la = hat_env(p->e + p->nf, k, p->nf, p->els);   /* (the low part: noise for what its own level lacks) */
    lb = hat_env(p->e + p->nf, k + 1u, p->nf, p->els);
    ta *= p->tl / p->th;
    tb *= p->tl / p->th;
    la = ta * ta - la * la;
    lb = tb * tb - lb * lb;
    la = la > 0.f ? hat_sqrt(la) * hs.nl : 0.f;
    lb = lb > 0.f ? hat_sqrt(lb) * hs.nl : 0.f;
    hs.el = la;
    hs.eld = (lb - la) * st;
}
static UI_LATE float hat_next(const hat_hit_t *p)   /* the low part's next sample (x 32767 of pk; 0 after its end;
                                          * out of line: one call each two output frames at TUNE 0) */
{
    const uint32_t i = hs.di++;
    if (i >= p->n) return 0.f;
#if HAT_BITS == 8u
    {
        const uint32_t c = HAT_D[p->d + i];
        const float m = (float)HAT_MU[c & 127u];
        return c & 128u ? -m : m;
    }
#else
    {                                    /* (4 bit: (2 c - 15) / 15 x the block's scale 2^(-e / 8), mk_hat.py) */
        const uint32_t c = (uint32_t)HAT_D[p->d + (i >> 1)] >> ((i & 1u) << 2) & 15u;
        if (!(i & (HAT_BLK - 1u))) hs.sc = (32767.f / 15.f) * hat_exp2((float)HAT_SB[p->s + i / HAT_BLK] * -0.125f);
        return (float)((int32_t)(c << 1) - 15) * hs.sc;
    }
#endif
}
static UI_LATE void hat_hit(uint32_t h, float f, float g, uint32_t tune, uint32_t tone)   /* audio interrupt: hit h
                                          * struck (dr_hit): f DECAY's factor (the step's LENGTH and accent in it), g
                                          * the level; the sections' states kept (CH cutting OH: no step in them) */
{
    const hat_hit_t *p = &HAT_H[h];
    const float r = DR_SEMI[tune], lim = 0.45f * S.fs;
    float fl = HAT_LPK * HAT_RATE * r, fh = HAT_HPK * 0.5f * HAT_RATE * r;
    const float ft = 0.136f * HAT_RATE * r;   /* (the low tail's band: from 3 kHz at TUNE 0) */
    fl = fl < lim ? fl : lim;
    fh = fh < lim ? fh : lim;
    hat_bq(hs.c[0], fl, 0.54119610f, 0);
    hat_bq(hs.c[1], fl, 1.30656296f, 0);
    hat_bq(hs.c[2], fh, 0.54119610f, 1);
    hat_bq(hs.c[3], fh, 1.30656296f, 1);
    hat_bq(hs.c[4], ft, 0.70710678f, 1);
    hs.h = (uint8_t)h;
    hs.inc = (uint32_t)(r * HAT_RATE * S.ifs * 65536.f + 0.5f);
    hs.pos = hs.di = hs.k = 0;
    hs.s0 = 0.f;
    hs.s1 = hat_next(p);
    hs.gl = g * p->pk * (1.f / 32767.f);
    hs.gh = g * DR_SEMI[tone] * 1.7320508f / hat_sqrt(1.f - 2.f * fh * S.ifs);   /* (TONE: the noise band; the
                                          * noise of -1..1 at its power 1 / 3, the part the high-pass passes) */
    hs.nl = 32767.f / p->pk * 1.7320508f / hat_sqrt(2.f * (fl - ft) * S.ifs);
    hs.fd = 1.f;                         /* (DECAY below 100 %: from the hit's peak on, to -60 dB in f / (1 - f) x the
                                          * hit's length after it) */
    hs.fdc = f < 1.f ? dr_decay(f / (1.f - f) * (float)(p->n - p->kp * HAT_FLS) * (1.f / HAT_RATE) / r) : 1.f;
    hs.tail = f > 1.f;
    f = p->cut ? f - 1.f : f;            /* (the tail to -60 dB in f x the hit's own decay from the anchor: a hit that
                                          * decays by itself as at 100 %, longer above; a hit cut: none at 100 %) */
    hs.slf = p->sl / (f > 0.0156f ? f : 0.0156f);
    hs.on = 1;
    hat_frame();
}
static UI_LATE int hat_render(float *o, const float *nz, uint32_t nn)   /* audio interrupt: the sampled hat added
                                          * into o (nn frames; nz the drums' white noise): 0 once it has ended */
{
    const hat_hit_t *p = &HAT_H[hs.h];
    const uint32_t inc = hs.inc;
    const float gl = hs.gl, fdc = hs.fdc;
    uint32_t pos = hs.pos, fe = (hs.k + 1u) * HAT_FLS, k, j;
    float s0 = hs.s0, s1 = hs.s1, eh = hs.eh, ehd = hs.ehd, el = hs.el, eld = hs.eld, fd = hs.fd;
    float (*c)[3] = hs.c, (*z)[2] = hs.z;
    for (k = 0; k < nn; k++) {
        float x, y, w;
        while ((pos >> 16) + 1u >= hs.di) { s0 = s1; s1 = hat_next(p); }
        x = s0 + (float)(pos & 0xFFFFu) * (1.f / 65536.f) * (s1 - s0);
        if (el != 0.f) {                 /* (the low tail: noise high-passed here, low-passed with the sample) */
            w = c[4][0] * nz[k] + z[4][0];
            z[4][0] = -2.f * c[4][0] * nz[k] - c[4][1] * w + z[4][1];
            z[4][1] = c[4][0] * nz[k] - c[4][2] * w;
            x += el * w;
        }
        for (j = 0; j < 2u; j++) {       /* (the low-pass: two sections, transposed direct form II) */
            y = c[j][0] * x + z[j][0];
            z[j][0] = 2.f * c[j][0] * x - c[j][1] * y + z[j][1];
            z[j][1] = c[j][0] * x - c[j][2] * y;
            x = y;
        }
        w = nz[k];
        for (j = 2u; j < 4u; j++) {      /* (the noise's high-pass) */
            y = c[j][0] * w + z[j][0];
            z[j][0] = -2.f * c[j][0] * w - c[j][1] * y + z[j][1];
            z[j][1] = c[j][0] * w - c[j][2] * y;
            w = y;
        }
        o[k] += (gl * x + eh * w) * fd;
        eh += ehd;
        el += eld;
        if (hs.k >= p->kp) fd *= fdc;
        pos += inc;
        if ((pos >> 16) >= fe) {         /* (the next frame) */
            hs.k++;
            fe += HAT_FLS;
            hat_frame();
            eh = hs.eh; ehd = hs.ehd; el = hs.el; eld = hs.eld;
        }
    }
    hs.pos = pos; hs.s0 = s0; hs.s1 = s1; hs.eh = eh; hs.el = el; hs.fd = fd;
    if (fd < 1e-3f) return 0;            /* (faded: -60 dB) */
    if (hs.k <= p->nf) return 1;         /* (the hit itself) */
    return hs.tail && hs.slf * (float)(hs.k - p->kt) > p->te;   /* (its tail, to -60 dB) */
}
#define HAT_ON hs.on
#else
#define HAT_ON 0u
#endif
#define KIT_MODEL(i) ((i) <= DR_HT || (HAT_HAVE && ((i) == DR_CH || (i) == DR_OH)))   /* instrument i has two
                                          * models (the hats: with the sample's table, 0.98) */

static UI_LATE void dr_hit(uint32_t ins, uint32_t sb)   /* audio interrupt: instrument ins struck (sb: its step byte):
                                          * its voice from the start (CH and OH: the other one cut), the kit's
                                          * settings now (a factor of 1 where they are as shipped: the sound as before
                                          * the kit, to the bit) */
{
    const dr_kp_t *q = &drk.k[ins];
    const uint32_t m9 = ins <= DR_HT && q->m909, vi = DR_VOICE[ins], acc = (sb & 3u) > 2u ? 2u : sb & 3u;
    const dr_ins_t *p = m9 ? &DR_909[ins] : &DR_INS[ins];
    dr_voice_t *v = &dr.v[vi];
#if DR_HALF
    uint32_t full = (DR_KDRIVE && ins == DR_BD && dv.on) || (DR_HATF && vi >= DR_HH && vi <= 8u);   /* (the kick
                                          * driven, DR_HATF the hats and CY: at the full rate) */
    const float ts = full ? 1.f : 0.5f;  /* (the decays' times in the voice's frames) */
#else
#define ts 1.f
#endif
    const float df = DR_SEMI[q->dec] * DR_SEMI[q->dec],
                lm = DR_LMUL[sb >> 2 & 7u] * (1.f + 0.15f * (float)acc) * df,
                g = p->g * DR_ACCG[acc] * ((float)(q->lev * q->lev) / 10000.f);
    v->a = g;
    v->ad = dr_decay(p->ta * lm * ts);
    v->b = 0.f;
    v->bd = dr_decay(p->tb * ts);
    v->ph = v->ph2 = v->z1 = v->z2 = v->e = 0.f;
    v->m = (uint8_t)(m9 | q->att << 1);
    v->tune = q->tune;
    v->tone = q->tone;
    v->pan = q->pan;
    ds.vi[vi] = (uint8_t)ins;
    switch (ins) {
    case DR_BD: case DR_LT: case DR_MT: case DR_HT:
        v->b = 1.f + 0.12f * (float)acc; /* (the sweep; the accent: higher) */
        if (m9 && ins == DR_BD) {        /* (the 909's BD: the triangle from its zero crossing, rising; the attack's
                                          * pulse, ph2, 1.6 ms) */
            v->ph = 0.75f;
            v->ph2 = 0.6f * g;
        }
        if (q->att || m9) {              /* (the click: noise for a few ms, CLICK's and the 909's) */
            v->z1 = g * (0.028f * (float)q->att + (m9 ? 0.12f : 0.f));
            v->z2 = dr_decay((ins == DR_BD ? (m9 ? 0.008f : 0.004f) : m9 ? 0.014f : 0.006f) * ts);
            dr.ck = (uint16_t)(dr.ck | 1u << vi);
        } else
            dr.ck = (uint16_t)(dr.ck & ~(1u << vi));
        break;
    case DR_SD:                          /* (the tone; the noise, its decay the length's) */
        v->a = 0.45f * g;
        v->b = 0.6f * g * (1.f + 0.1f * (float)acc);
        v->bd = dr_decay(p->tb * lm * ts);
        break;
    case DR_RS: {                        /* (TONE: the click and the higher sine up, the lower down, the sum's level
                                          * about kept: s) */
        const float s = 2.f / (1.f + DR_SEMI[q->tone]);
        v->a = g * s;
        v->b = 0.5f * g * DR_SEMI[q->tone] * s;
        break;
    }
    case DR_CP: {                        /* (a burst; the tail, its decay the length's; the next burst's frame. TUNE
                                          * moves the band-pass: the level kept about, / ~tune^0.5) */
        const float t = DR_SEMI[(q->tune + DR_K0) / 2u];
        v->a = g / t;
        v->ad = dr_decay(p->ta * ts);
        v->b = 0.5f * g / t;
        v->bd = dr_decay(p->tb * lm * ts);
        v->ph2 = DR_CP_GAP * S.fs * ts;
        break;
    }
    case DR_CH: case DR_OH: case DR_CY:  /* (the accent: the high-pass a little higher; TUNE moves the high-passes:
                                          * the level kept about, x ~tune^1.5) */
        v->ph2 = (float)acc;
        v->a = g * DR_SEMI[q->tune] * DR_SEMI[(q->tune + DR_K0) / 2u];
        break;
    default:                             /* CB: the attack's fast part, the tail */
        v->a = 0.6f * g;
        v->b = 0.4f * g;
        break;
    }
#if HAT_HAVE
    if (vi == DR_HH) {                   /* (the hats: MODEL SMP, the sample; the voice's level a flag then) */
        hs.on = (uint8_t)q->m909;
        if (hs.on) {
            hat_hit(ins - DR_CH, lm, HAT_G * DR_ACCG[acc] * ((float)(q->lev * q->lev) / 10000.f), q->tune, q->tone);
            v->a = 1.f;
#if DR_HALF
            full = 1;                    /* (the sample at the full rate, its own filters') */
#endif
        }
    }
#endif
    if (q->att && !DR_SWEPT(ins)) v->e = 1.f;   /* (ATTACK: the fade-in; BD and the toms have CLICK instead) */
#if DR_HALF
    if (full) {
        v->m |= DR_MF;
        dr.fm = (uint16_t)(dr.fm | 1u << vi);
    } else
        dr.fm = (uint16_t)(dr.fm & ~(1u << vi));
#else
#undef ts
#endif
    dr.act = (uint16_t)(dr.act | 1u << vi);
}
static volatile uint32_t dr_mute PLAT_POOL;   /* main loop: a step recorded ahead of its hit (dr_rec_hit), its
                                          * instruments not struck again there: the step counted from the clock's
                                          * start (21 bits) << 11 | the instruments (11 bits); 0 none */
static volatile uint8_t dr_metro PLAT_POOL;   /* main loop: RECORD is on (sqv.rec, dr_idle): the metronome */
#ifdef APP_HOST_CMD
static void host_click(uint32_t m, uint32_t acc);   /* (1.18C40, hostcmd.c: the metronome's clicks, "mcis") */
#endif
#ifdef APP_HOST_CMD
static uint16_t dr_lastt[DR_NI];         /* (the host simulator: each instrument's last hit, its tick in the loop + 1) */
static uint32_t dr_jn, dr_jl, dr_jmax, dr_jsum, dr_jst, dr_jbd, dr_jw, dr_jc, dr_jog;   /* (the host, hostcmd.c "drjit":
                                          * the hits, those late, the latest (the clock's ticks after their place),
                                          * their sum; the steps passed, BD's hits, the store's writes before, the
                                          * changes at their line, BD's hits off the grid) */
#endif
#ifdef HYB_MULTI
/* (1.19.3) GLO > SEQ > DRUM OUT ON: every hit played here (the drum sequencer's steps, the black keys and the steps
 * heard on the drums' page, dr.prev; not the RECORD metronome's, not a note in on DRUM CHANNEL: never sent back) goes
 * out over MIDI as a note on DRUM CHANNEL (OFF: none) to OUT PORTS, DR_GM's note (the map the notes in take; BD 36 of
 * its two), the velocity from the accent (64, 100, 127: dr_mi_in's law takes each back to its accent). Its note-off
 * at the next block (dr_moff: 3 .. 11 ms later; a drum module plays the hit whole), so none is left on by a stop, a
 * PANIC or a setting changed. In every ENGINE (MIDI ONLY too, where the drums are silent); the launcher's MUTE silences
 * the audio alone, unless the drums have no sound of their own (MIDI ONLY): then their MIDI (LN_MMUTE) */
static const uint8_t DR_GMO[11] = {36, 38, 41, 45, 48, 37, 39, 42, 46, 49, 56};   /* (the instruments' order, DR_BD ..) */
static uint16_t dr_mo PLAT_POOL;         /* the instruments sent, their note-offs due */
static uint8_t dr_moc PLAT_POOL;         /* their channel (0 .. 15) */
static UI_LATE void dr_moff(void)        /* audio interrupt: the note-offs due */
{
    const uint32_t to = midi_to(), st = 0x80u | dr_moc;
    uint32_t m = dr_mo;
    dr_mo = 0;
    for (; m && to; m &= m - 1u) plat_midi_send(st >> 4 | st << 8 | (uint32_t)DR_GMO[__builtin_ctz(m)] << 16, to);
}
static UI_LATE void dr_out(uint32_t i, uint32_t sb)   /* audio interrupt: instrument i struck here (sb: its step byte) */
{
    const uint32_t c = (set.dr_ch - 1u) & 15u, st = 0x90u | c, a = (sb & 3u) > 2u ? 2u : sb & 3u, to = midi_to();
    if (!(set.sq_def & SQ_DOUT) || !set.dr_ch || !to || LN_MMUTE(2u)) return;
    if ((dr_mo >> i & 1u) || dr_moc != c) dr_moff();   /* (struck again before its off; the channel changed) */
    plat_midi_send(st >> 4 | st << 8 | (uint32_t)DR_GMO[i] << 16 | (64u + 36u * a - (a >> 1) * 9u) << 24, to);
    dr_mo |= (uint16_t)(1u << i);
    dr_moc = (uint8_t)c;
}
#else
#define dr_out(i, sb) ((void)0)
#endif
static UI_LATE void dr_tick(uint32_t t, uint32_t fl)   /* audio interrupt: the drums' tick t (1/8 step, from their
                                          * origin): its step's hits there (ALIGN 0 .. 7/8 late), the next step's early
                                          * ones (ALIGN -7/8 .. -1/8: 8 - its eighths into this step; over the loop's end
                                          * its first step's), as sq_tick. fl (0.96): bit 0 the next step's early ones
                                          * left out (a pattern waits for the line after this step), bit 1 the line
                                          * (step 1's early ones now) */
{
    const uint32_t m = SQ_STEPS(drp.lc) - 1u, st = t >> 3 & m, sub = t & 7u, mu = dr_mute;
    uint32_t i, j, h = 0;
    if ((t & 7u) == 0u) { dr.step = (uint8_t)st; dr.hit = 0; }
    if (dr_metro && sub == 0u && !(st & 3u)) {   /* (RECORD: the rim shot as the metronome on each beat, the bar's
                                          * first accented; heard only, never recorded) */
        dr_hit(DR_RS, ((st & 15u) ? 0u : 2u) | 4u << 2);
#ifdef APP_HOST_CMD
        host_click(dr_qm, !(st & 15u));
#endif
    }
    for (j = 0; j < 2u - (fl & 1u); j++) {   /* (this step's, the next's early) */
        const uint32_t s = (st + j) & m, skip = mu >> 11 == (((t >> 3) + j) & 0x1FFFFFu) ? mu : 0u;   /* (the
                                          * instruments recorded at it: heard) */
        for (i = 0; i < DR_NI; i++) {    /* (CH before OH: on one step OH sounds) */
            const uint32_t sb = DR_ST(i, s), a = sb >> 5, e = a && DR_NEG(i, s);
            if (DR_ON(i, s) && ((e == j && (e ? 8u - a : a) == sub) || ((fl & 2u) && !j && e)) && !(skip >> i & 1u)
                && !(sq_pvm & 4u)) {     /* (a
                                          * step held heard as it is turned: its hit alone, 0.94) */
                dr_hit(i, sb);
                dr_out(i, sb);                  /* (1.19.3: DRUM OUT) */
                h |= 1u << i;
#ifdef APP_HOST_CMD
                dr_lastt[i] = (uint16_t)((t & (m * 8u + 7u)) + 1u);
                dr_hon++;
                dr_hm = dr_qm;
                {                        /* (drjit: its lateness, the clock's tick now against tick t's own) */
                    const uint32_t lt = dr_qm - (dr_org + (t << CK_SH(drp.div)));
                    dr_jn++;
                    if (i == DR_BD) dr_jbd++;
                    if (i == DR_BD && !a && (dr_qm & ((8u << CK_SH(drp.div)) - 1u))) dr_jog++;   /* (off the
                                          * shared clock's step grid) */
                    if ((int32_t)lt > 0) { dr_jl++; dr_jsum += lt; if (lt > dr_jmax) dr_jmax = lt; }
                }
#endif
            }
        }
    }
    if (h) dr.hit = (uint16_t)(dr.hit | h);
}
static UI_LATE void dr_swap(void)        /* audio interrupt, a line: the pattern in the spare (un_shd) and drp swapped */
{
    uint16_t *a = (uint16_t *)(void *)&drp, *b = (uint16_t *)(void *)&un_shd;
    uint32_t i;
    for (i = 0; i < (uint32_t)sizeof drp / 2u; i++) { const uint16_t t = a[i]; a[i] = b[i]; b[i] = t; }
}
_Static_assert(sizeof(dr_pat_t) % 2u == 0u, "dr_swap: in halfwords");
static UI_LATE void dr_clk(uint32_t m, uint32_t sh)   /* audio interrupt: the drums' clock tick m (on their grid,
                                          * >> sh their tick), from their origin (dr_org); 0.96: a pattern waiting, its
                                          * line (sq_q*, as sq_tick). After 0.99 the pattern chosen is loaded ahead into
                                          * the spare by the main loop (sq_qm[2]: offered); the bar's last step takes it
                                          * at its first tick (SQ_QS_LOAD: the old pattern's last step played out, its
                                          * step 1's early hits left out) and the line swaps it in (SQ_QS_DONE): step 1
                                          * on the line, however late the main loop. Not offered by then: the old
                                          * pattern plays on, the change at the next line (never late, never silent) */
{
    const uint32_t rq = sq_qr[2];
    uint32_t s = sq_qs[2], r = (m - dr_org) >> sh, line = 0;
    const uint32_t p8 = s == SQ_QS_LOAD ? SQ_QP8(sq_qd[2], sq_ql[2]) : SQ_QP8(drp.div, drp.lc);
    dr_qm = m;
#ifdef APP_HOST_CMD
    if (!(r & 7u)) dr_jst++;             /* (drjit: a step passed, played or not) */
#endif
    if (rq && (s == SQ_QS_OFF || s == SQ_QS_LOAD) && !(r & (p8 - 1u)) && (rq != SQ_QR_SWAP || s == SQ_QS_LOAD)) {
        if (s == SQ_QS_LOAD) dr_swap();
        dr_org = m;
        r = 0;
        line = 2u;
        s = rq == SQ_QR_NONE ? SQ_QS_HALT : SQ_QS_DONE;
        sq_qs[2] = (uint8_t)s;
#ifdef APP_HOST_CMD
        if (rq == SQ_QR_SWAP) dr_jc++;   /* (drjit: a change at its line) */
#endif
    }
    if (s == SQ_QS_HALT || (rq == SQ_QR_JOIN && s == SQ_QS_OFF)) return;
    dr_tick(r, ((rq == SQ_QR_NONE || s == SQ_QS_LOAD) && !(((r >> 3) + 1u) * 8u & (p8 - 1u))) | line);   /* (the
                                          * line next: not the next step's early ones; at the line, step 1's) */
    if (rq == SQ_QR_SWAP && s == SQ_QS_OFF && sq_qm[2] && !(((r | 7u) + 1u) & (p8 - 1u)) && !(r & 7u)) {   /* (the
                                          * bar's last step, its first tick: the spare taken for the line) */
        sq_qd[2] = drp.div;
        sq_ql[2] = drp.lc;
        sq_qs[2] = SQ_QS_LOAD;
    }
}
static UI_LATE void dr_qcatch(void)      /* audio interrupt, a block's start: the request gone, the state cleared (NONE
                                          * from a line, or a stop at the line, left it held, every later PATTERN
                                          * refused as waiting) */
{
    if (!sq_qr[2]) sq_qs[2] = SQ_QS_OFF;
}
static UI_LATE void arp_realign(sy_synth *p)   /* audio interrupt, the shared clock's start (PLAY): a part's engine
                                          * arpeggiator playing held keys starts again from its first step now, with the
                                          * sequencers' first step (as a first key does: synth.c arp_note_on, without
                                          * the wait for its free clock's next tick) */
{
    if (!p->arp_on || !p->arp_n) return;
    p->arp_i = -1;
    p->arp_t = 1e9f;
    p->arp_dir = 1;
    p->arp_lt = 0.f;
    p->arp_wait = 0.f;
}
static UI_LATE void sq_block(uint32_t frames)   /* audio interrupt, a block's start: the sequencers on and off, the
                                          * shared clock's ticks, the arp sequencer's steps; the drums' steps of
                                          * DR_LAT blocks ago */
{
    uint32_t i, m0, n = 0, lat, to;
    float bpm;
#ifdef HYB_MULTI
    if (dr_mo) dr_moff();                /* (1.19.3: DRUM OUT's note-offs) */
#endif
    if (dr.prev) {                       /* (an instrument to hear: a black key on the drums' page) */
        const uint32_t p = dr.prev;
        dr.prev = 0;
        if ((p & 15u) - 1u < DR_NI) {
            dr_hit((p & 15u) - 1u, (p >> 4 ^ DR_LEN1 << 2) & 0x1Fu);
            dr_out((p & 15u) - 1u, (p >> 4 ^ DR_LEN1 << 2) & 0x1Fu);   /* (1.19.3: DRUM OUT) */
        }
#ifdef APP_HOST_CMD
        dr_hpv = (uint16_t)(((p & 15u) - 1u) | ((p >> 4 ^ DR_LEN1 << 2) & 0x1Fu) << 8);
        dr_hpn++;
#endif
    }
#ifdef HYB_MULTI
    for (sq_ii = 0; sq_ii < SQ_NI; sq_ii++)   /* (SEQ A, SEQ B: each plays its part) */
#endif
    if (SQI.run && !SQI.on && !(sq_pva[2] & 4u)) {   /* on (a preview's note ended in this block: at the next,
                                          * sq_pv_cut_a): the arpeggiator held off, the MIDI notes held stopped (they
                                          * are its input now); it plays from the clock's next tick on its grid (at
                                          * once if the clock starts now) */
        SQI.on = 1;
        SQI.join = 1;
#ifdef HYB_MULTI
        if (SQI.hold) eng_to(sq_ii ? EM_B : EM_A, SY_EV_PARAM, 129u, 0u);   /* (its part's arpeggiator; PATCH ARP ON:
                                          * not held, sq_hold) */
        for (i = 0; i < 128u; i++)
            if (SQI.midi[i >> 5] >> (i & 31u) & 1u) eng_to(sq_ii ? EM_B : EM_A, 0x80u, i, 0u);
#else
        if (sq.hold) eng_midi(SY_EV_PARAM, 129u, 0u);
        for (i = 0; i < 128u; i++)
            if (sq.midi[i >> 5] >> (i & 31u) & 1u) eng_midi(0x80u, i, 0u);
#endif
        for (i = 0; i < SQ_NT; i++) SQI.nt[i].live = 0;
        sq_lat_clr();                    /* (KEY LATCH: from the BASE NOTE, or the notes held now) */
    } else if (!SQI.run && SQI.on) {     /* off: its notes off (the main loop gives the arpeggiator back) */
        SQI.on = 0;
        sq_all_off();
    }
#ifdef HYB_MULTI
    for (sq_ii = 0; sq_ii < SQ_NI; sq_ii++)
#endif
    if (sq_cut[SQ_IIX]) {                /* (its pattern replaced: the notes of the one before off) */
        sq_cut[SQ_IIX] = 0;
        if (SQI.on) { sq_all_off(); sq_pva[2] |= 4u; }   /* (a preview's note-on waits a block: one key's off and on
                                          * never in one block, sq_pv_cut_a) */
    }
    if (!(sq_pva[2] & 4u) && dr.on != dr.run) {
        dr.on = dr.run;
        dr_org = 0;
    }   /* (0.96: the origin the clock's start)
                                          * (as the arp sequencers: a start together, the block after a preview's
                                          * note-off) */
#ifdef HYB_MULTI
    if (!sqr[0].on && !sqr[1].on && !dr.on && !ln.emp) ck.on = 0;
#else
    if (!sq.on && !dr.on && !ln.emp) ck.on = 0;
#endif
    else if (!ck.on) {                   /* the clock starts: its first tick now (1.18C40: PRECOUNT's, pc_tk ticks before
                                          * 0) */
        ck.on = 1;
        ck.mt = 0u - pc_tk;
        ck.pre = (uint8_t)(pc_tk != 0u);
        pc_tk = 0;
        ck.frac = 0.f;
#ifdef HYB_MULTI
        ck.ofs = 0;                      /* (1.19.4: CLK OFFSET from the start's first tick, below) */
        ck.adj = 0.f;
#endif
        n = 1;
        if (eng.syn) {                   /* (the engine's arpeggiators in phase with it) */
            arp_realign(&S);
#ifdef HYB_MULTI
            arp_realign(&SB);
#endif
        }
    }
    m0 = ck.mt;
    bpm = S.bpm_now > 1.f ? S.bpm_now : (float)S.patch[128 + 16] + 63.f;
#ifdef HYB_MULTI
    xk.t += frames;                      /* (1.19.5: CLK LOCK's time base; a new run's estimate afresh, the clock gone: no
                                          * place known until the next START) */
    if (!ck.on) xk.lk = 0;
    if (xk.st && (float)(xk.t - xk.tl) > S.fs) xk.st = xk.lk = 0;   /* (no tick for 1 s, as the engine's tempo) */
#endif
    to = midi_to();
    if (set.clk == CLK_MASTER && to) {   /* CLOCK MASTER: the MIDI clock out at the tempo, 24 a beat, from the block
                                          * clock with the fraction carried, as the shared clock's ticks below (a tick
                                          * at the start of the block it falls in, no drift: the sequencers' notes and
                                          * the clock stay together); START with the shared clock's start (its first
                                          * tick, the bar's first step: a clock tick with it), the clock's phase from
                                          * there; STOP when both sequencers are off */
        if (ck.on && !ck.mrun && (ck.pre ? (int32_t)ck.mt >= 0 : n != 0u)) {   /* (a count: at its end, the block
                                          * after the clock's 0) */
            ck.mrun = 1;
            ck.mfrac = 1.f;              /* (a tick at once, after START: the first beat; the phase before dropped) */
            plat_midi_send(0x0Fu | 0xFAu << 8, to);
        } else if (!ck.on && ck.mrun) {
            ck.mrun = 0;
            plat_midi_send(0x0Fu | 0xFCu << 8, to);
        }
        ck.mfrac += (float)frames * bpm * (24.f / 60.f) / S.fs;
        for (i = 0; ck.mfrac >= 1.f && i < 8u; i++) {
            ck.mfrac -= 1.f;
            plat_midi_send(0x0Fu | 0xF8u << 8, to);
        }
    } else if (ck.mrun) {                /* (MASTER left, or OUT PORTS OFF, while it ran: STOP, where it can go) */
        ck.mrun = 0;
        if (to) plat_midi_send(0x0Fu | 0xFCu << 8, to);
    }
    if ((int32_t)ck.mt >= 0) ck.pre = 0;
    if (pc_cut) {                        /* (1.18C40: RECORD off in the count: the clock's 0 at once, step 1 now) */
        pc_cut = 0;
        if (ck.on && (int32_t)ck.mt < 0) { m0 = ck.mt = ~0u; ck.frac = 1.f; }
    }
    if (ck.on) {
#ifdef HYB_MULTI
        xk.oph = (float)(ck.mt & 127u) + ck.frac;   /* (1.19.5: its place at this block's start, for CLK LOCK) */
#endif
        ck.frac += (float)frames * bpm * (16.f * 2.f / 15.f) / S.fs;   /* (a tick at 1x: fs * 15 / bpm / 8 frames;
                                          * the clock's at 4x) */
#ifdef HYB_MULTI
        if (xk.lk && set.clk == CLK_EXT && S.xclk_on && !(set.ck_fl & CK_FL_UNLOCK)) {   /* (1.19.5) CLK LOCK: the
                                          * clock steered onto the master's place (xk_rt) */
            const float dt = (float)frames * S.ifs, m = dt * 0.25f;
            float d = -(XK_K * xk.e + xk.ri) * dt;
            if (d > m) d = m;
            else if (d < -m) d = -m;
            else {                       /* (the integral held while the limit acts) */
                xk.ri += XK_KI * xk.e * dt;
                xk.ri = xk.ri > XK_IMAX ? XK_IMAX : xk.ri < -XK_IMAX ? -XK_IMAX : xk.ri;
            }
            xk.e += d;
            if (xk.pend) xk.e += dt * (bpm * xk.per * 0.4f * S.ifs - 1.f);   /* (a gap open: the clock's tempo against
                                          * the master's before it, 60 fs / 24 / per) */
            ck.frac += d * bpm * (16.f * 2.f / 15.f);
            ck.adj = 0.f;                /* (CLK OFFSET is the loop's: no one-off shift beside it) */
            ck.ofs = 0;
        } else {   /* (1.19.4) CLK OFFSET, CLOCK EXTERNAL with a clock coming in alone: the clock's phase shifted by it (- earlier), by running at most
             * 25 % faster or slower until it is made: from the start (its first step at once, with START, as before;
             * the shift made within 4x the offset, 200 ms at most) and after a change while it runs (no tick lost or
             * doubled: the ticks stay the fraction's) */
            const int32_t o = set.clk == CLK_EXT && S.xclk_on ? set.ck_ofs : 0;   /* (a clock coming in) */
            if (o != ck.ofs) { ck.adj += (float)(ck.ofs - o) * 1e-3f; ck.ofs = (int8_t)o; }
            if (ck.adj != 0.f) {
                const float m = (float)frames * S.ifs * 0.25f, d = ck.adj > m ? m : ck.adj < -m ? -m : ck.adj;
                ck.adj -= d;
                ck.frac += d * bpm * (16.f * 2.f / 15.f);
            }
        }
#endif
        if (!n) m0++;
        for (; ck.frac >= 1.f && n < 32u; n++) {
            ck.frac -= 1.f;
            ck.mt++;
        }
    }
#ifdef HYB_MULTI
    for (sq_ii = 0; sq_ii < SQ_NI; sq_ii++)   /* (both on the one grid: each tick's place from the clock's) */
#endif
    {
        sq_qcatch();                     /* (0.96: a pattern loaded after its line) */
        sq_endp();                       /* (1.02: a chord latched that its steps replaced) */
        if (SQI.on && sq_latch() && SQG.fp) sq_follow();   /* (1.08: a key struck: FING POLY's chord follows it) */
        for (i = 0; i < n && SQI.on; i++) {   /* the arp sequencer's ticks */
            const uint32_t m = m0 + i, sh = CK_SH(sq_qs[SQ_IIX] == SQ_QS_LOAD ? sq_qd[SQ_IIX] : SQPI.div);   /* (a
                                          * pattern being loaded before the line: the old one's grid to it) */
            if ((m & ((1u << sh) - 1u)) || (int32_t)m < 0) continue;   /* (1.18C40: PRECOUNT's ticks: none) */
            if (SQI.join) {              /* (on the clock's grid from its start: 0.96's origin there) */
                SQI.join = 0;
                SQI.tick = (uint16_t)(m >> sh);
                sq_org[SQ_IIX] = 0;
            }
            else SQI.tick++;
#ifdef APP_HOST_CMD
            sq_mt_now = m;
#endif
            sq_tick();
            sq_endp();                   /* (the notes ending in it that no step took on again, 1.02) */
        }
    }
    ck.q[ck.blk % CK_Q].m0 = m0;
    ck.q[ck.blk % CK_Q].n = n;
    lat = S.opt & SY_OPT_LATENCY ? 2u : 0u;   /* (DR_LAT: the engine's note latency, its blocks) */
    dr_qcatch();                         /* (the request gone: its state cleared; after 0.99 no late load to catch) */
    if (dr.on) {                         /* the drums' ticks: the clock's of lat blocks ago (on its grid: the tick's
                                          * place in the bar from the clock, whenever the drums were turned on) */
        const uint32_t q = (ck.blk + CK_Q - lat) % CK_Q;
        for (i = 0; i < ck.q[q].n; i++) {
            const uint32_t m = ck.q[q].m0 + i, sh = CK_SH(sq_qs[2] == SQ_QS_LOAD ? sq_qd[2] : drp.div);   /* (each
                                          * tick: a pattern loaded before its line, the old grid to it, 0.96) */
            if (!(m & ((1u << sh) - 1u)) && (int32_t)m >= 0) dr_clk(m, sh);   /* (PRECOUNT's: none) */
        }
    }
    if ((sq_metro || dr_metro) && ck.on) {   /* RECORD on the arp sequencer's page: the rim shot on its beats (every 4
                                          * steps at its time division), the bar's first accented, heard with its
                                          * notes (lat blocks later, as the drums'); heard only. 1.18C40: the drums'
                                          * page, PRECOUNT's beats alone on the drums' grid (from 0 on: dr_tick's) */
#ifdef HYB_MULTI
        const uint32_t sh = CK_SH(sq_metro ? sqpa[(sq_metro - 1u) & 1u].div : drp.div);
#else
        const uint32_t sh = CK_SH(sq_metro ? sqp.div : drp.div);
#endif
        const uint32_t q = (ck.blk + CK_Q - lat) % CK_Q;
        for (i = 0; i < ck.q[q].n; i++) {
            const uint32_t m = ck.q[q].m0 + i, st = m >> sh >> 3;
            if (!(m & ((8u << sh) - 1u)) && !(st & 3u) && (sq_metro || (int32_t)m < 0)) {
                dr_hit(DR_RS, ((st & 15u) ? 0u : 2u) | 4u << 2);
#ifdef APP_HOST_CMD
                host_click(m, !(st & 15u));
#endif
            }
        }
    }
    ck.blk = (uint8_t)((ck.blk + 1u) % CK_Q);
}
/* The 909's BD (the kit's MODEL 909), this project's own design after published descriptions of
 * the classic machine's circuit: an oscillator whose wave is a triangle flattened by a soft clip (odd harmonics, not
 * a sine: the third at -20 dB, the 808's -29), driven harder at the attack (4.2x into the clip, 2.2x late: squarer);
 * its pitch from an exponential envelope, f0 (1 + sw e), e falling to -60 dB in 0.14 s (from ~5.5 f0 at the hit to
 * f0 + 50 % at 50 ms, + 10 % at 80 ms: the "thud" gliding down, where the 808's sweep is over in a few ms); at the
 * attack a pulse (1.6 ms) and a burst of noise (8 ms, gently low-passed: two taps). c: the amplitude, the pitch
 * envelope, the phase, the pulse (in, out); nz 0: the click over (no noise) */
static UI_LATE void dr_bd909(float *o, const float *nz, uint32_t nn, float f, float sw, float ad, float bd, float *c,
                             float *zc, float zd)   /* (nn a power of 2: the noise's second tap wraps) */
{
    float a = c[0], b = c[1], ph = c[2], pu = c[3], z1 = *zc, x;
    const float pd = zd * zd * zd * zd * zd;   /* (the pulse: five times the noise's rate) */
    uint32_t k;
    for (k = 0; k < nn; k++) {
        ph += f * (1.f + sw * b);
        if (ph >= 1.f) ph -= 1.f;
        x = 4.f * (ph < 0.5f ? 0.5f - ph : ph - 0.5f) - 1.f;   /* (the triangle) */
        x *= 2.2f + 2.f * b;
        x = x > 1.5f ? 1.5f : x < -1.5f ? -1.5f : x;
        x *= 1.f - x * x * (1.f / 6.75f);        /* (the soft clip: 1 at 1.5) */
        x = a * x + pu;
        if (nz) {
            x += z1 * 0.5f * (nz[k] + nz[(k + 1u) & (nn - 1u)]);
            z1 *= zd;
        }
        o[k] += x;
        a *= ad;
        b *= bd;
        pu *= pd;
    }
    c[0] = a; c[1] = b; c[2] = ph; c[3] = pu < 1e-6f ? 0.f : pu;
    *zc = z1;
}
#if !DR_HALF
static UI_LATE void dr_render(int32_t *lr, uint32_t n, float g)   /* audio interrupt: the voices sounding added to the
                                          * block (24 bit, g: MASTER's scale), clipped. The kit's settings as taken at
                                          * the hit (dr_hit): a voice with an ATTACK still fading in or a PAN goes
                                          * through a buffer of its own (pv) to the mix; a PAN makes the mix stereo
                                          * (sd: the side, L - R over 2) for the chunk */
{
    uint32_t c0, k, vi, snd = 0;         /* (snd: a voice sent into the bus this block) */
    float d[DR_CHUNK], nz[DR_CHUNK], mt[DR_CHUNK], pv[DR_CHUNK], sd[DR_CHUNK];
    if (!dr.act) { dv.cidle = 1; S.dr_on = 0; return; }   /* (silent: DRUM COMP's detector starts again with the next
                                          * hit; the bus empty) */
    g *= dr.gain;
    if (S.dr_w > 0.f)                    /* (an instrument sends: the bus from zeros; never an ig above 0 without it,
                                          * dr_sends) */
        for (k = 0; k < 2u * SY_BLOCK; k++) ds.b[k] = 0.f;
    for (c0 = 0; c0 < n; c0 += DR_CHUNK) {
        const uint32_t nn = n - c0 < DR_CHUNK ? n - c0 : DR_CHUNK;
        const uint32_t act = dr.act;
        uint32_t side = 0;
        for (k = 0; k < nn; k++) d[k] = 0.f;
        if (act & (0x1E2u | dr.ck)) {    /* the noise, one for all voices that use it (SD RS CP, the hats, CY; the
                                          * clicks) */
            uint32_t x = dr.nz;
            for (k = 0; k < nn; k++) {
                x ^= x << 13; x ^= x >> 17; x ^= x << 5;
                nz[k] = (float)(int32_t)x * (1.f / 2147483648.f);
            }
            dr.nz = x;
        }
        if ((act >> DR_HH & 3u) & ~HAT_ON) {   /* the metal (the hats, CY): six squares at inharmonic pitches (not
                                          * for the sampled hat alone) */
            uint32_t q0 = dr.mph[0], q1 = dr.mph[1], q2 = dr.mph[2], q3 = dr.mph[3], q4 = dr.mph[4], q5 = dr.mph[5];
            const float r = S.ifs * 4294967296.f;   /* (Hz to a phase increment) */
            const uint32_t i0 = (uint32_t)(DR_MHZ[0] * r), i1 = (uint32_t)(DR_MHZ[1] * r), i2 = (uint32_t)(DR_MHZ[2] * r),
                           i3 = (uint32_t)(DR_MHZ[3] * r), i4 = (uint32_t)(DR_MHZ[4] * r), i5 = (uint32_t)(DR_MHZ[5] * r);
            for (k = 0; k < nn; k++) {
                const uint32_t n6 = (q0 >> 31) + (q1 >> 31) + (q2 >> 31) + (q3 >> 31) + (q4 >> 31) + (q5 >> 31);
                mt[k] = (float)(int32_t)n6 * (1.f / 3.f) - 1.f;
                q0 += i0; q1 += i1; q2 += i2; q3 += i3; q4 += i4; q5 += i5;
            }
            dr.mph[0] = q0; dr.mph[1] = q1; dr.mph[2] = q2; dr.mph[3] = q3; dr.mph[4] = q4; dr.mph[5] = q5;
        }
        for (vi = 0; vi < DR_NV; vi++) {
            dr_voice_t *v = &dr.v[vi];
            float a, b, ph, ph2, z1, z2, ad, bd, *o = d;
            float tf, tn;                /* (the kit's TUNE and TONE as factors: 1 as shipped) */
            float gs;                    /* (its SEND's factor into the bus: 0 none) */
            uint32_t post;
            if (!(act >> vi & 1u)) continue;
            gs = ds.ig[ds.vi[vi]];
            a = v->a; b = v->b; ph = v->ph; ph2 = v->ph2; z1 = v->z1; z2 = v->z2; ad = v->ad; bd = v->bd;
            tf = DR_SEMI[v->tune];
            tn = DR_SEMI[v->tone];
            post = v->e > 0.f || v->pan || gs != 0.f;
            if (post) {                  /* (its own buffer: the fade-in, the pan, the send) */
                o = pv;
                for (k = 0; k < nn; k++) pv[k] = 0.f;
            }
            if (DR_SWEPT(vi)) {           /* BD, the toms: a sine swept down (TONE: the sweep's depth, x 1/4 .. 4) */
                const dr_ins_t *p = v->m & 1u ? &DR_909[vi] : &DR_INS[vi];
                const float f = p->f0 * tf * S.ifs, sw = p->sw * tn * tn;
                if ((v->m & 1u) && vi == 0u) {   /* (the 909's BD: dr_bd909) */
                    float c[4] = {a, b, ph, ph2};
                    if (z1 > 0.f) {
                        dr_bd909(o, nz, nn, f, sw, ad, bd, c, &z1, z2);
                        if (z1 < 1e-5f) {    /* (the click over) */
                            z1 = 0.f;
                            dr.ck = (uint16_t)(dr.ck & ~(1u << vi));
                        }
                    } else
                        dr_bd909(o, 0, nn, f, sw, ad, bd, c, &z1, z2);
                    a = c[0]; b = c[1]; ph = c[2]; ph2 = c[3];
                } else if (z1 > 0.f) {   /* (a click: noise) */
                    for (k = 0; k < nn; k++) {
                        float x;
                        ph += f * (1.f + sw * b * b);
                        if (ph >= 1.f) ph -= 1.f;
                        x = 2.f * ph - 1.f;
                        x = 4.f * x * (1.f - (x < 0.f ? -x : x));
                        o[k] += a * x + z1 * nz[k];
                        a *= ad;
                        b *= bd;
                        z1 *= z2;
                    }
                    if (z1 < 1e-5f) {    /* (the click over) */
                        z1 = 0.f;
                        dr.ck = (uint16_t)(dr.ck & ~(1u << vi));
                    }
                } else
                    for (k = 0; k < nn; k++) {
                        float x;
                        ph += f * (1.f + sw * b * b);
                        if (ph >= 1.f) ph -= 1.f;
                        x = 2.f * ph - 1.f;
                        x = 4.f * x * (1.f - (x < 0.f ? -x : x));
                        o[k] += a * x;
                        a *= ad;
                        b *= bd;
                    }
            } else if (vi == 1u) {       /* SD: two sines (f0, 1.6 f0) and the noise band-passed (TONE: the noise's
                                          * high-pass and low-pass); the 909's: the second tone higher and louder,
                                          * the noise brighter (less low-passed) and louder */
                const uint32_t m9 = v->m & 1u;
                const dr_ins_t *p = m9 ? &DR_909[DR_SD] : &DR_INS[DR_SD];
                const float f = p->f0 * tf * S.ifs, c = 6.2832f * p->f1 * tn * S.ifs, r2 = m9 ? 1.78f : 1.6f,
                            t2 = m9 ? 0.8f : 0.5f, lp0 = (m9 ? 0.85f : 0.6f) * tn, lp = lp0 < 0.98f ? lp0 : 0.98f,
                            nm = m9 ? 1.8f : 1.3f;
                for (k = 0; k < nn; k++) {
                    float x, y, w = nz[k];
                    ph += f; if (ph >= 1.f) ph -= 1.f;
                    ph2 += r2 * f; if (ph2 >= 1.f) ph2 -= 1.f;
                    x = 2.f * ph - 1.f; x = 4.f * x * (1.f - (x < 0.f ? -x : x));
                    y = 2.f * ph2 - 1.f; y = 4.f * y * (1.f - (y < 0.f ? -y : y));
                    z1 += c * (w - z1);  /* (the noise high-passed, then a little low-passed) */
                    z2 += lp * (w - z1 - z2);
                    o[k] += a * (x + t2 * y) + nm * b * z2;
                    a *= ad;
                    b *= bd;
                }
            } else if (vi == 5u) {       /* RS: two sines pinged, a click of noise (TONE: the click and the higher
                                          * sine: dr_hit, h) */
                const float f = DR_INS[DR_RS].f0 * tf * S.ifs, f2 = DR_INS[DR_RS].f1 * tf * S.ifs, h = 0.6f * tn;
                for (k = 0; k < nn; k++) {
                    float x, y, w = nz[k];
                    ph += f; if (ph >= 1.f) ph -= 1.f;
                    ph2 += f2; if (ph2 >= 1.f) ph2 -= 1.f;
                    x = 2.f * ph - 1.f; x = 4.f * x * (1.f - (x < 0.f ? -x : x));
                    y = 2.f * ph2 - 1.f; y = 4.f * y * (1.f - (y < 0.f ? -y : y));
                    o[k] += a * (h * x + 0.4f * y) + b * w;
                    a *= ad;
                    b *= bd;
                }
            } else if (vi == 6u) {       /* CP: noise through a band-pass (a state-variable filter); three bursts
                                          * DR_CP_GAP apart (a), the tail (b). TUNE: the band; TONE: its width (the
                                          * level about kept) */
                const float f = 6.2832f * DR_INS[DR_CP].f0 * tf * S.ifs, gap = DR_CP_GAP * S.fs, q = 0.45f * tn,
                            cg = 1.2f * (0.75f + 0.25f * tn);
                for (k = 0; k < nn; k++) {
                    float w = nz[k], hp;
                    ph += 1.f;
                    if (ph >= ph2 && ph < 2.5f * gap) { a = 2.f * b; ph2 += gap; }
                    z1 += f * z2;
                    hp = w - z1 - q * z2;
                    z2 += f * hp;
                    o[k] += (a + b) * cg * z2;
                    a *= ad;
                    b *= bd;
                }
#if HAT_HAVE
            } else if (vi == DR_HH && hs.on) {   /* the sampled hat (hat_render; its end: a 0) */
                a = (float)hat_render(o, nz, nn);
#endif
            } else if (vi == DR_HH || vi == 8u) {   /* the hats, the cymbal: the metal and noise, high-passed,
                                                     * rectified, high-passed (TUNE: the high-passes; TONE: the
                                                     * low-pass after them) */
                const float x0 = 6.2832f * DR_INS[vi == DR_HH ? DR_CH : DR_CY].f0 * (1.f + 0.1f * ph2) * tf * S.ifs,
                            c = 1.f - 1.f / (1.f + x0 + 0.5f * x0 * x0),   /* (a one-pole low-pass's ~1 - exp(-x0)) */
                            mix = vi == DR_HH ? 0.45f : 0.65f, lc = 1.f - 0.25f / tn;
                for (k = 0; k < nn; k++) {
                    const float x = mix * mt[k] + (1.f - mix) * nz[k];
                    float y;
                    z1 += c * (x - z1);  /* (a high-pass: the input less its low-pass; half-wave rectified, the
                                          * grit of the classic hats; high-passed again for the DC; a little low-passed) */
                    y = x - z1;
                    y = y > 0.f ? y : 0.f;
                    z2 += c * (y - z2);
                    ph += lc * (y - z2 - ph);   /* (then a low-pass at ~12 kHz: ph its state) */
                    o[k] += a * ph;
                    a *= ad;
                }
            } else {                     /* CB: two squares, low-passed; a fast part and the tail (TONE: the
                                          * low-pass) */
                const float f = DR_INS[DR_CB].f0 * tf * S.ifs, f2 = DR_INS[DR_CB].f1 * tf * S.ifs,
                            c = 6.2832f * 2400.f * tn * S.ifs;
                for (k = 0; k < nn; k++) {
                    float x;
                    ph += f; if (ph >= 1.f) ph -= 1.f;
                    ph2 += f2; if (ph2 >= 1.f) ph2 -= 1.f;
                    x = (ph < 0.5f ? 0.5f : -0.5f) + (ph2 < 0.5f ? 0.5f : -0.5f);
                    z1 += c * (x - z1);
                    z2 += c * (z1 - z2);
                    o[k] += (a + b) * z2;
                    a *= ad;
                    b *= bd;
                }
            }
            if (post) {                  /* the fade-in (ATTACK: 1 - e, e to -20 dB in its time) and the pan (a
                                          * balance: the far side down, the near one as centred) */
                const int32_t pn = v->pan;
                const float gl = pn > 0 ? 1.f - (float)pn * (1.f / DR_KPAN) : 1.f,
                            gr = pn < 0 ? 1.f + (float)pn * (1.f / DR_KPAN) : 1.f,
                            cm = 0.5f * (gl + gr), cs = 0.5f * (gl - gr);
                float e = v->e;
                const float ed = e > 0.f ? dr_decay(0.006f * (float)(v->m >> 1)) : 0.f;   /* (90 % at 2 ms a step) */
                if (pn && !side) {
                    side = 1;
                    for (k = 0; k < nn; k++) sd[k] = 0.f;
                }
                if (e > 0.f) {
                    for (k = 0; k < nn; k++) {
                        pv[k] *= 1.f - e;
                        e *= ed;
                    }
                    v->e = e < 1e-4f ? 0.f : e;
                }
                if (gs != 0.f) {         /* (the send: after the level, the fade-in and the pan) */
                    float *sb = ds.b + c0;
                    const float ga = gl * gs, gb = gr * gs;
                    snd = 1;
                    for (k = 0; k < nn; k++) {
                        sb[k] += ga * pv[k];
                        sb[SY_BLOCK + k] += gb * pv[k];
                    }
                }
                if (!pn)
                    for (k = 0; k < nn; k++) d[k] += pv[k];
                else
                    for (k = 0; k < nn; k++) {
                        d[k] += cm * pv[k];
                        sd[k] += cs * pv[k];
                    }
            }
            if (b < 1e-5f) b = 0.f;      /* (no denormals: CP's bursts end long before its tail, a too) */
            if (a < 1e-30f) a = 0.f;
            if (a < 1e-4f && (DR_SWEPT(vi) ? z1 < 1e-4f : b < 1e-4f)) {   /* (silent: the voice
                                          * ends; BD and the toms: their click over too) */
                dr.act = (uint16_t)(dr.act & ~(1u << vi));
                dr.ck = (uint16_t)(dr.ck & ~(1u << vi));
                a = b = z1 = z2 = 0.f;
                v->e = 0.f;
            }
            v->a = a; v->b = b; v->ph = ph; v->ph2 = ph2; v->z1 = z1; v->z2 = z2;
        }
        if (dv.comp) dr_comp(d, sd, nn, side);   /* (DRUM COMP, then the drive) */
        if (!DR_KDRIVE && dv.on) dr_drive(d, sd, nn, side);   /* (LEVEL above 100: the bus driven) */
        if (!side)
            for (k = 0; k < nn; k++) {   /* (centred: the same into both sides, then clipped) */
                const int32_t x = (int32_t)(d[k] * g);
                int32_t *o = lr + 2u * (c0 + k), l = o[0] + x, r = o[1] + x;
                o[0] = l > 8388607 ? 8388607 : l < -8388607 ? -8388607 : l;
                o[1] = r > 8388607 ? 8388607 : r < -8388607 ? -8388607 : r;
            }
        else
            for (k = 0; k < nn; k++) {   /* (a voice panned: the mid and the side) */
                int32_t *o = lr + 2u * (c0 + k), l = o[0] + (int32_t)((d[k] + sd[k]) * g),
                        r = o[1] + (int32_t)((d[k] - sd[k]) * g);
                o[0] = l > 8388607 ? 8388607 : l < -8388607 ? -8388607 : l;
                o[1] = r > 8388607 ? 8388607 : r < -8388607 ? -8388607 : r;
            }
    }
    S.dr_on = (uint8_t)snd;              /* (the bus to the synth's effects, the next block) */
}
#else
static UI_LATE void dr_up(float *x, float *y, uint32_t nk, uint32_t add)   /* audio interrupt (1.18C42): nk frames
                                          * of the half rate (x: after DR_UPH of history) to 2 nk of the full rate
                                          * into y (add: added to it), the history kept. DR_UP 1: linear, a frame of
                                          * the full rate late; 2: four points (the cubic's), 4 frames late; 3: a
                                          * half-band of 15 taps, 8 frames late */
{
    uint32_t j;
#if DR_UP == 1
#define DR_UPO(j, e, o) e = 0.5f * (x[j] + x[(j) + 1u]); o = x[(j) + 1u]
#elif DR_UP == 2
#define DR_UPO(j, e, o) e = x[(j) + 1u]; o = 0.5625f * (x[(j) + 1u] + x[(j) + 2u]) - 0.0625f * (x[j] + x[(j) + 3u])
#else
#define DR_UPO(j, e, o)                                                                                    \
    e = x[(j) + 3u];                                                                                        \
    o = DR_HB1 * (x[(j) + 3u] + x[(j) + 4u]) + DR_HB2 * (x[(j) + 2u] + x[(j) + 5u])                          \
        + DR_HB3 * (x[(j) + 1u] + x[(j) + 6u]) + DR_HB4 * (x[j] + x[(j) + 7u])
#endif
    if (add)
        for (j = 0; j < nk; j++) {
            float e, o;
            DR_UPO(j, e, o);
            y[2u * j] += e;
            y[2u * j + 1u] += o;
        }
    else
        for (j = 0; j < nk; j++) {
            float e, o;
            DR_UPO(j, e, o);
            y[2u * j] = e;
            y[2u * j + 1u] = o;
        }
#undef DR_UPO
    for (j = 0; j < DR_UPH; j++) x[j] = x[nk + j];
}
static UI_LATE void dr_render(int32_t *lr, uint32_t n, float g)   /* audio interrupt: the voices sounding added to the
                                          * block (24 bit, g: MASTER's scale), clipped. 1.18C42 (DR_HALF): the voices
                                          * at half the rate into hx (its side hsx, the send's hbx), at the full rate
                                          * those with DR_MF into d; the half rate's upsampled into d, sd and the bus
                                          * (dr_up; their histories kept in drb), then COMP. The kit's settings as taken at the hit
                                          * (dr_hit): a voice with an ATTACK still fading in, a PAN, a SEND or the
                                          * kick driven goes through a buffer of its own (pv) to the mix; a PAN
                                          * makes the mix stereo (the side, L - R over 2) for the chunk */
{
    uint32_t c0, k, vi, snd = 0;         /* (snd: a voice sent into the bus this block) */
    float d[DR_CHUNK], sd[DR_CHUNK], pv[DR_CHUNK], nz[DR_CHUNK], mt[DR_HATF ? DR_CHUNK : DR_HK];   /* (full rate) */
    float hx[DR_UPH + DR_HK], hsx[DR_UPH + DR_HK], hbx[2][DR_UPH + DR_HK];   /* (half rate, after the histories) */
    float *const h = hx + DR_UPH, *const hsd = hsx + DR_UPH;
    if (!dr.act) {                       /* (silent: DRUM COMP's detector starts again with the next hit; the bus
                                          * empty; the upsampler's histories cleared once) */
        dv.cidle = 1;
        S.dr_on = 0;
        if (!drb.upz) {
            for (k = 0; k < DR_UPH; k++) drb.h[k] = drb.hs[k] = drb.hb[0][k] = drb.hb[1][k] = 0.f;
            drb.hsl = drb.hbl = 0;
            drb.upz = 1;
        }
        return;
    }
    drb.upz = 0;
    g *= dr.gain;
    if (S.dr_w > 0.f)                    /* (an instrument sends: the bus from zeros; never an ig above 0 without it,
                                          * dr_sends) */
        for (k = 0; k < 2u * SY_BLOCK; k++) ds.b[k] = 0.f;
    else if (drb.hbl) {                  /* (no SEND now: the half rate's send history dropped) */
        for (k = 0; k < DR_UPH; k++) drb.hb[0][k] = drb.hb[1][k] = 0.f;
        drb.hbl = 0;
    }
    for (k = 0; k < DR_UPH; k++) {       /* (the histories in) */
        hx[k] = drb.h[k];
        hsx[k] = drb.hs[k];
        hbx[0][k] = drb.hb[0][k];
        hbx[1][k] = drb.hb[1][k];
    }
    for (c0 = 0; c0 < n; c0 += DR_CHUNK) {
        const uint32_t nn = n - c0 < DR_CHUNK ? n - c0 : DR_CHUNK, nh = nn >> 1;
        const uint32_t act = dr.act, fv = act & dr.fm;   /* (fv: the voices at the full rate) */
        uint32_t side = 0, hside = 0, hsnd = 0;   /* (hsnd: a voice of the half rate sent this chunk) */
        if (fv)
            for (k = 0; k < nn; k++) d[k] = 0.f;
        for (k = 0; k < nh; k++) h[k] = 0.f;
        if (act & (0x1E2u | dr.ck)) {    /* the noise, one for all voices that use it (SD RS CP, the hats, CY; the
                                          * clicks): at the half rate's frames, the full rate's where a voice there
                                          * needs it */
            uint32_t x = dr.nz;
            const uint32_t m = fv ? nn : nh;
            for (k = 0; k < m; k++) {
                x ^= x << 13; x ^= x >> 17; x ^= x << 5;
                nz[k] = (float)(int32_t)x * (1.f / 2147483648.f);
            }
            dr.nz = x;
        }
        if ((act >> DR_HH & 3u) & ~HAT_ON) {   /* the metal (the hats, CY; at the half rate, DR_HATF the full):
                                          * six squares at inharmonic pitches (not for the sampled hat alone) */
            uint32_t q0 = dr.mph[0], q1 = dr.mph[1], q2 = dr.mph[2], q3 = dr.mph[3], q4 = dr.mph[4], q5 = dr.mph[5];
            const float r = (DR_HATF ? 1.f : 2.f) * S.ifs * 4294967296.f;   /* (Hz to a phase increment) */
            const uint32_t i0 = (uint32_t)(DR_MHZ[0] * r), i1 = (uint32_t)(DR_MHZ[1] * r), i2 = (uint32_t)(DR_MHZ[2] * r),
                           i3 = (uint32_t)(DR_MHZ[3] * r), i4 = (uint32_t)(DR_MHZ[4] * r), i5 = (uint32_t)(DR_MHZ[5] * r);
            for (k = 0; k < (DR_HATF ? nn : nh); k++) {
                const uint32_t n6 = (q0 >> 31) + (q1 >> 31) + (q2 >> 31) + (q3 >> 31) + (q4 >> 31) + (q5 >> 31);
                mt[k] = (float)(int32_t)n6 * (1.f / 3.f) - 1.f;
                q0 += i0; q1 += i1; q2 += i2; q3 += i3; q4 += i4; q5 += i5;
            }
            dr.mph[0] = q0; dr.mph[1] = q1; dr.mph[2] = q2; dr.mph[3] = q3; dr.mph[4] = q4; dr.mph[5] = q5;
        }
        for (vi = 0; vi < DR_NV; vi++) {
            dr_voice_t *v = &dr.v[vi];
            float a, b, ph, ph2, z1, z2, ad, bd, *o;
            float tf, tn;                /* (the kit's TUNE and TONE as factors: 1 as shipped) */
            float gs;                    /* (its SEND's factor into the bus: 0 none) */
            float ifs;                   /* (a frame's time at the voice's rate, x 1 / s) */
            uint32_t post, hr, nk, drv;  /* (hr: the half rate; nk: its frames this chunk; drv: the kick driven) */
            if (!(act >> vi & 1u)) continue;
            hr = !(v->m & DR_MF);
            nk = hr ? nh : nn;
            ifs = hr ? 2.f * S.ifs : S.ifs;
            o = hr ? h : d;
            gs = ds.ig[ds.vi[vi]];
            a = v->a; b = v->b; ph = v->ph; ph2 = v->ph2; z1 = v->z1; z2 = v->z2; ad = v->ad; bd = v->bd;
            tf = DR_SEMI[v->tune];
            tn = DR_SEMI[v->tone];
            drv = DR_KDRIVE && !hr && vi == 0u && dv.on;
            post = v->e > 0.f || v->pan || gs != 0.f || drv;
            if (post) {                  /* (its own buffer: the fade-in, the pan, the send, the drive) */
                o = pv;
                for (k = 0; k < nk; k++) pv[k] = 0.f;
            }
            if (DR_SWEPT(vi)) {           /* BD, the toms: a sine swept down (TONE: the sweep's depth, x 1/4 .. 4) */
                const dr_ins_t *p = v->m & 1u ? &DR_909[vi] : &DR_INS[vi];
                const float f = p->f0 * tf * ifs, sw = p->sw * tn * tn;
                if ((v->m & 1u) && vi == 0u) {   /* (the 909's BD: dr_bd909) */
                    float c[4] = {a, b, ph, ph2};
                    if (z1 > 0.f) {
                        dr_bd909(o, nz, nk, f, sw, ad, bd, c, &z1, z2);
                        if (z1 < 1e-5f) {    /* (the click over) */
                            z1 = 0.f;
                            dr.ck = (uint16_t)(dr.ck & ~(1u << vi));
                        }
                    } else
                        dr_bd909(o, 0, nk, f, sw, ad, bd, c, &z1, z2);
                    a = c[0]; b = c[1]; ph = c[2]; ph2 = c[3];
                } else if (z1 > 0.f) {   /* (a click: noise) */
                    for (k = 0; k < nk; k++) {
                        float x;
                        ph += f * (1.f + sw * b * b);
                        if (ph >= 1.f) ph -= 1.f;
                        x = 2.f * ph - 1.f;
                        x = 4.f * x * (1.f - (x < 0.f ? -x : x));
                        o[k] += a * x + z1 * nz[k];
                        a *= ad;
                        b *= bd;
                        z1 *= z2;
                    }
                    if (z1 < 1e-5f) {    /* (the click over) */
                        z1 = 0.f;
                        dr.ck = (uint16_t)(dr.ck & ~(1u << vi));
                    }
                } else
                    for (k = 0; k < nk; k++) {
                        float x;
                        ph += f * (1.f + sw * b * b);
                        if (ph >= 1.f) ph -= 1.f;
                        x = 2.f * ph - 1.f;
                        x = 4.f * x * (1.f - (x < 0.f ? -x : x));
                        o[k] += a * x;
                        a *= ad;
                        b *= bd;
                    }
            } else if (vi == 1u) {       /* SD: two sines (f0, 1.6 f0) and the noise band-passed (TONE: the noise's
                                          * high-pass and low-pass); the 909's: the second tone higher and louder,
                                          * the noise brighter (less low-passed) and louder. (The one-poles' factors
                                          * at the half rate: two of the full rate's steps, c (2 - c)) */
                const uint32_t m9 = v->m & 1u;
                const dr_ins_t *p = m9 ? &DR_909[DR_SD] : &DR_INS[DR_SD];
                const float f = p->f0 * tf * ifs, c0f = 6.2832f * p->f1 * tn * S.ifs, r2 = m9 ? 1.78f : 1.6f,
                            t2 = m9 ? 0.8f : 0.5f, lp0 = (m9 ? 0.85f : 0.6f) * tn, lp1 = lp0 < 0.98f ? lp0 : 0.98f,
                            lpx = lp1 > 0.6f ? lp1 - 0.6f : 0.f,
                            nm = (m9 ? 1.8f : 1.3f) * (hr ? 0.92f * (1.f + 4.f * lpx * lpx) : 1.f),
                            c = hr ? c0f * (2.f - c0f) : c0f, lp = hr ? lp1 * (2.f - lp1) : lp1;   /* (the noise at the
                                          * half rate: its density twice the full's, its band's level -0.7 dB; the
                                          * brighter noise (the 909's, TONE up) up to +4 dB for its top lost: the
                                          * A-weighted level as before) */
                for (k = 0; k < nk; k++) {
                    float x, y, w = nz[k];
                    ph += f; if (ph >= 1.f) ph -= 1.f;
                    ph2 += r2 * f; if (ph2 >= 1.f) ph2 -= 1.f;
                    x = 2.f * ph - 1.f; x = 4.f * x * (1.f - (x < 0.f ? -x : x));
                    y = 2.f * ph2 - 1.f; y = 4.f * y * (1.f - (y < 0.f ? -y : y));
                    z1 += c * (w - z1);  /* (the noise high-passed, then a little low-passed) */
                    z2 += lp * (w - z1 - z2);
                    o[k] += a * (x + t2 * y) + nm * b * z2;
                    a *= ad;
                    b *= bd;
                }
            } else if (vi == 5u) {       /* RS: two sines pinged, a click of noise (TONE: the click and the higher
                                          * sine: dr_hit, h) */
                const float f = DR_INS[DR_RS].f0 * tf * ifs, f2 = DR_INS[DR_RS].f1 * tf * ifs, hh = 0.6f * tn;
                for (k = 0; k < nk; k++) {
                    float x, y, w = nz[k];
                    ph += f; if (ph >= 1.f) ph -= 1.f;
                    ph2 += f2; if (ph2 >= 1.f) ph2 -= 1.f;
                    x = 2.f * ph - 1.f; x = 4.f * x * (1.f - (x < 0.f ? -x : x));
                    y = 2.f * ph2 - 1.f; y = 4.f * y * (1.f - (y < 0.f ? -y : y));
                    o[k] += a * (hh * x + 0.4f * y) + b * w;
                    a *= ad;
                    b *= bd;
                }
            } else if (vi == 6u) {       /* CP: noise through a band-pass (a state-variable filter); three bursts
                                          * DR_CP_GAP apart (a), the tail (b). TUNE: the band; TONE: its width (the
                                          * level about kept). (Its factor at the half rate: the full rate's band,
                                          * 2 sin(pi fc / fs), its sine doubled) */
                const float f1 = 6.2832f * DR_INS[DR_CP].f0 * tf * S.ifs,
                            f = hr ? 2.f * f1 * fm_sqrtf(1.f - 0.25f * f1 * f1) : f1,
                            gap = DR_CP_GAP * S.fs * (hr ? 0.5f : 1.f), q = 0.45f * tn,
                            cg = 1.2f * (0.75f + 0.25f * tn) * (hr ? 0.75f : 1.f);   /* (the noise's density at the
                                          * half rate twice the full's: -2.5 dB) */
                for (k = 0; k < nk; k++) {
                    float w = nz[k], hp;
                    ph += 1.f;
                    if (ph >= ph2 && ph < 2.5f * gap) { a = 2.f * b; ph2 += gap; }
                    z1 += f * z2;
                    hp = w - z1 - q * z2;
                    z2 += f * hp;
                    o[k] += (a + b) * cg * z2;
                    a *= ad;
                    b *= bd;
                }
#if HAT_HAVE
            } else if (vi == DR_HH && hs.on) {   /* the sampled hat (hat_render, at the full rate; its end: a 0) */
                a = (float)hat_render(o, nz, nk);
#endif
            } else if (vi == DR_HH || vi == 8u) {   /* the hats, the cymbal: the metal and noise, high-passed,
                                                     * rectified, high-passed (TUNE: the high-passes; TONE: the
                                                     * low-pass after them) */
                const float x0 = 6.2832f * DR_INS[vi == DR_HH ? DR_CH : DR_CY].f0 * (1.f + 0.1f * ph2) * tf * S.ifs
                                 * (hr ? 1.4f : 1.f),
                            c = 1.f - 1.f / (1.f + x0 + 0.5f * x0 * x0),   /* (a one-pole low-pass's ~1 - exp(-x0).
                                          * The half rate: x0 x 1.4, not 2: the high-passes' corners lower, the band
                                          * up to 11 kHz and the A-weighted level as they were) */
                            mix = vi == DR_HH ? 0.45f : 0.65f, lc1 = 1.f - 0.25f / tn, lc = hr ? lc1 * (2.f - lc1) : lc1,
                            hg = hr && vi == DR_HH ? 1.15f : 1.f,   /* (the hats at the half rate: +1.2 dB, their
                                          * A-weighted level as before; in the input: the chain is homogeneous) */
                            m1 = mix * hg, m2 = (1.f - mix) * hg;
                for (k = 0; k < nk; k++) {
                    const float x = m1 * mt[k] + m2 * nz[k];
                    float y;
                    z1 += c * (x - z1);  /* (a high-pass: the input less its low-pass; half-wave rectified, the
                                          * grit of the classic hats; high-passed again for the DC; a little low-passed) */
                    y = x - z1;
                    y = y > 0.f ? y : 0.f;
                    z2 += c * (y - z2);
                    ph += lc * (y - z2 - ph);   /* (then a low-pass at ~12 kHz: ph its state) */
                    o[k] += a * ph;
                    a *= ad;
                }
            } else {                     /* CB: two squares, low-passed; a fast part and the tail (TONE: the
                                          * low-pass) */
                const float f = DR_INS[DR_CB].f0 * tf * ifs, f2 = DR_INS[DR_CB].f1 * tf * ifs,
                            c1 = 6.2832f * 2400.f * tn * S.ifs, c = hr ? c1 * (2.f - c1) : c1;
                for (k = 0; k < nk; k++) {
                    float x;
                    ph += f; if (ph >= 1.f) ph -= 1.f;
                    ph2 += f2; if (ph2 >= 1.f) ph2 -= 1.f;
                    x = (ph < 0.5f ? 0.5f : -0.5f) + (ph2 < 0.5f ? 0.5f : -0.5f);
                    z1 += c * (x - z1);
                    z2 += c * (z1 - z2);
                    o[k] += (a + b) * z2;
                    a *= ad;
                    b *= bd;
                }
            }
            if (post) {                  /* the drive (the kick, LEVEL above 100: dr_drive, before its pan and send),
                                          * the fade-in (ATTACK: 1 - e, e to -20 dB in its time) and the pan (a
                                          * balance: the far side down, the near one as centred) */
                const int32_t pn = v->pan;
                const float gl = pn > 0 ? 1.f - (float)pn * (1.f / DR_KPAN) : 1.f,
                            gr = pn < 0 ? 1.f + (float)pn * (1.f / DR_KPAN) : 1.f,
                            cm = 0.5f * (gl + gr), cs = 0.5f * (gl - gr);
                float e = v->e, *md = hr ? h : d, *ms = sd;
                const float ed = e > 0.f ? dr_decay(0.006f * (float)(v->m >> 1 & 31u) * (hr ? 0.5f : 1.f)) : 0.f;   /* (90 %
                                          * at 2 ms a step) */
                if (drv) dr_drive(pv, pv, nk, 0);
                if (pn) {
                    if (hr) {
                        ms = hsd;
                        if (!hside) {
                            hside = 1;
                            for (k = 0; k < nh; k++) hsd[k] = 0.f;
                        }
                    } else if (!side) {
                        side = 1;
                        for (k = 0; k < nn; k++) sd[k] = 0.f;
                    }
                }
                if (e > 0.f) {
                    for (k = 0; k < nk; k++) {
                        pv[k] *= 1.f - e;
                        e *= ed;
                    }
                    v->e = e < 1e-4f ? 0.f : e;
                }
                if (gs != 0.f) {         /* (the send: after the level, the drive, the fade-in and the pan; at the
                                          * half rate into hbx, upsampled after the chunk's voices) */
                    float *sb = hr ? hbx[0] + DR_UPH : ds.b + c0, *sr = hr ? hbx[1] + DR_UPH : ds.b + SY_BLOCK + c0;
                    const float ga = gl * gs, gb = gr * gs;
                    if (hr && !hsnd) {   /* (the chunk's first: its half-rate send from zeros) */
                        hsnd = 1;
                        for (k = 0; k < nh; k++) sb[k] = sr[k] = 0.f;
                    }
                    snd = 1;
                    for (k = 0; k < nk; k++) {
                        sb[k] += ga * pv[k];
                        sr[k] += gb * pv[k];
                    }
                }
                if (!pn)
                    for (k = 0; k < nk; k++) md[k] += pv[k];
                else
                    for (k = 0; k < nk; k++) {
                        md[k] += cm * pv[k];
                        ms[k] += cs * pv[k];
                    }
            }
            if (b < 1e-5f) b = 0.f;      /* (no denormals: CP's bursts end long before its tail, a too) */
            if (a < 1e-30f) a = 0.f;
            if (a < 1e-4f && (DR_SWEPT(vi) ? z1 < 1e-4f : b < 1e-4f)) {   /* (silent: the voice
                                          * ends; BD and the toms: their click over too) */
                dr.act = (uint16_t)(dr.act & ~(1u << vi));
                dr.ck = (uint16_t)(dr.ck & ~(1u << vi));
                a = b = z1 = z2 = 0.f;
                v->e = 0.f;
            }
            v->a = a; v->b = b; v->ph = ph; v->ph2 = ph2; v->z1 = z1; v->z2 = z2;
        }
        dr_up(hx, d, nh, fv != 0u);      /* (the half rate's mix to the full rate) */
        if (hside || drb.hsl) {          /* (its side, and once more after its last: the history played out) */
            if (!hside)
                for (k = 0; k < nh; k++) hsd[k] = 0.f;
            dr_up(hsx, sd, nh, side);
            side = 1;
            drb.hsl = (uint8_t)hside;
        }
        if (hsnd || drb.hbl) {           /* (the send of the half rate to the bus, and once more after its last) */
            if (!hsnd)
                for (k = 0; k < nh; k++) hbx[0][DR_UPH + k] = hbx[1][DR_UPH + k] = 0.f;
            dr_up(hbx[0], ds.b + c0, nh, 1);
            dr_up(hbx[1], ds.b + SY_BLOCK + c0, nh, 1);
            drb.hbl = (uint8_t)hsnd;
            snd = 1;
        }
        if (dv.comp) dr_comp(d, sd, nn, side);   /* (DRUM COMP: the whole kit, the kick driven included) */
        if (!DR_KDRIVE && dv.on) dr_drive(d, sd, nn, side);   /* (as 1.18C41: LEVEL above 100, the bus driven) */
        if (!side)
            for (k = 0; k < nn; k++) {   /* (centred: the same into both sides, then clipped) */
                const int32_t x = (int32_t)(d[k] * g);
                int32_t *o = lr + 2u * (c0 + k), l = o[0] + x, r = o[1] + x;
                o[0] = l > 8388607 ? 8388607 : l < -8388607 ? -8388607 : l;
                o[1] = r > 8388607 ? 8388607 : r < -8388607 ? -8388607 : r;
            }
        else
            for (k = 0; k < nn; k++) {   /* (a voice panned: the mid and the side) */
                int32_t *o = lr + 2u * (c0 + k), l = o[0] + (int32_t)((d[k] + sd[k]) * g),
                        r = o[1] + (int32_t)((d[k] - sd[k]) * g);
                o[0] = l > 8388607 ? 8388607 : l < -8388607 ? -8388607 : l;
                o[1] = r > 8388607 ? 8388607 : r < -8388607 ? -8388607 : r;
            }
    }
    for (k = 0; k < DR_UPH; k++) {       /* (the histories out) */
        drb.h[k] = hx[k];
        drb.hs[k] = hsx[k];
        drb.hb[0][k] = hbx[0][k];
        drb.hb[1][k] = hbx[1][k];
    }
    S.dr_on = (uint8_t)snd;              /* (the bus to the synth's effects, the next block) */
}
#endif
static UI_LATE void sq_lat_on(sq_rt_t *q, uint32_t w, uint32_t b)   /* audio interrupt: a MIDI note in for q's
                                          * input: in its latch at once (a note shorter than a block counts too), with
                                          * the notes held now (the grip, 1.08) */
{
    uint32_t k;
    for (k = 0; k < 4u; k++) q->lat[k] = q->keys[k] | q->midi[k];
    q->lat[w] |= b;
    q->ldn = 1;
}
/* (1.18C31) RECORD on the arp sequencer's page takes the MIDI notes in for its part (sq_metro: armed, 1 + the
 * sequencer): the audio interrupt hands them to the main loop (srq: note | velocity << 7 | sequencer << 14, velocity
 * 0 an off; sr_mi_poll records them as keys); they are neither the sequencer's input nor played here. srm: the notes
 * handed over and not let go yet (their off goes the same way while RECORD is armed; disarmed, the usual way) */
#define SRQ 32u
static uint16_t srq[SRQ];
static volatile uint32_t srq_w, srq_r;
static volatile uint32_t srm[4];
static UI_LATE uint32_t sr_mi_in(uint32_t t, uint32_t d1, uint32_t d2, uint32_t to)   /* audio interrupt: a note
                                          * (t 0x80 / 0x90) for the parts to (1 A, 2 B); the part RECORD took it for
                                          * (1 or 2), 0 none */
{
    const uint32_t b = 1u << (d1 & 31u), w = d1 >> 5 & 3u, m = sq_metro, p = (m - 1u) & 1u;
    const uint32_t arm = m && (to >> p & 1u), on = t == 0x90u && d2, q = srq_w;
    if (!on) {
        const uint32_t h = srm[w] & b;
        srm[w] &= ~b;
        if (!h || !arm) return 0;
    } else if (!arm) return 0;
    if (q - srq_r >= SRQ) return 0;      /* (full: the usual way) */
    if (on) srm[w] |= b;
    srq[q % SRQ] = (uint16_t)(d1 | (on ? d2 : 0u) << 7 | p << 14);
    RING_PUBLISH();
    srq_w = q + 1u;
    return 1u << p;
}
/* (1.18C31) GLO > MIDI > DRUM CHANNEL (the two-part builds): the notes on it play the drums, a
 * GM-like map (DR_GM: the lowest note DR_GM0), the accent from the velocity by the black keys' law (dr_key: under 70
 * none, under 110 ACC, else ACC+), LENGTH as shipped; heard at once, after thru (unchanged), before the listen
 * channel's filter. Every note on and off of that channel is the drums' (none reaches a part or a sequencer's input,
 * whatever CHANNEL and B CHANNEL are); its other messages go on as before. A note on while the drums' RECORD is armed
 * (dr_metro) also goes to the main loop (srq, bit 15: the instrument, the accent << 7; sr_mi_poll: dr_rec_hit as a
 * black key's). ENGINE (MIDI ONLY silent) and the launcher's MUTE act on the drums' output as on the keys' hits */
#ifdef HYB_MULTI
#define DR_MCH set.dr_ch
#else
#define DR_MCH 0u                        /* (the single build: no DRUM CHANNEL, no room in its settings) */
#endif
#define DR_GM0 35u
#define DR_GM_N 23u
static const uint8_t DR_GM[DR_GM_N] = {   /* notes 35 .. 57: the instrument, DR_NI none */
    DR_BD, DR_BD, DR_RS, DR_SD, DR_CP, DR_SD, DR_LT, DR_CH, DR_LT, DR_CH,   /* 35 .. 44 */
    DR_MT, DR_OH, DR_MT, DR_HT, DR_CY, DR_HT, DR_CY, DR_NI, DR_NI, DR_NI,   /* 45 .. 54 */
    DR_NI, DR_CB, DR_CY};                /* 55 .. 57 */
#ifdef APP_HOST_CMD
static uint32_t dr_mhn, dr_mhv;          /* (the host: the drum channel's hits, the last one's instrument | accent << 8) */
#endif
static UI_LATE uint32_t dr_mi_in(uint32_t pkt)   /* audio interrupt: a MIDI packet in; 1 = the drum channel's note,
                                          * taken (struck if a note on of the map) */
{
    const uint32_t st = pkt >> 8 & 0xFFu, d1 = (pkt >> 16 & 0x7Fu) - DR_GM0, v = pkt >> 24 & 0x7Fu;
    uint32_t i, acc, q;
    if (!DR_MCH || (st & 0xE0u) != 0x80u || (st & 0x0Fu) + 1u != DR_MCH) return 0;
    if (!(st & 0x10u) || !v || d1 >= DR_GM_N || (i = DR_GM[d1]) >= DR_NI) return 1;   /* (an off, a note not mapped) */
    acc = v >= 110u ? 2u : v >= 70u ? 1u : 0u;
    dr_hit(i, acc | DR_LEN1 << 2);
#ifdef APP_HOST_CMD
    dr_mhn++;
    dr_mhv = i | acc << 8;
#endif
    if (dr_metro && (q = srq_w) - srq_r < SRQ) {   /* (RECORD armed: to be recorded; full: heard alone) */
        srq[q % SRQ] = (uint16_t)(i | acc << 7 | 1u << 15);
        RING_PUBLISH();
        srq_w = q + 1u;
    }
    return 1;
}
#ifdef HYB_MULTI
static UI_LATE void sq_panic(void)       /* audio interrupt: all notes off (CC 123): the MIDI notes held in forgotten
                                          * (both sequencers) */
{
    srm[0] = srm[1] = srm[2] = srm[3] = 0;
    for (sq_ii = 0; sq_ii < SQ_NI; sq_ii++) {
        SQI.midi[0] = SQI.midi[1] = SQI.midi[2] = SQI.midi[3] = 0;
        sq_lat_clr();                    /* (KEY LATCH's notes too) */
        if (SQI.on) sq_all_off();
    }
}
static UI_LATE uint32_t in_parts(uint32_t st)   /* (1.19.4) audio interrupt: the parts a channel message in is for by
                                          * its channel: bit 0 A (A CHANNEL > IN: OFF none, OMNI all), bit 1 B (CHANNELS
                                          * alone: B CHANNEL > IN; its channel B alone, OMNI every channel, A's too) */
{
    const uint32_t c = st & 15u, a = !(set.mo & MO_AINOFF) && (!set.midi_ch || c + 1u == set.midi_ch);
    if (M.mode != SY_M_CHANNELS) return a;
    if (set.mo & MO_BOMNI) return a | 2u;
    return M.ch_b >= 0 && c == (uint32_t)M.ch_b ? 2u : a;
}
static UI_LATE uint32_t sq_parts_in(uint32_t st, uint32_t d1)   /* audio interrupt: the parts a MIDI note in goes to
                                          * (1 A, 2 B, 3 both), as the engine's setup routes it (synth.c multi_route):
                                          * SINGLE A; LAYER both; SPLIT by the split; CHANNELS A, part B's channel B.
                                          * 1.18C31 (the user's rule: MIDI in plays as the panel's keys, CHANNELS
                                          * alone by channel): B CHANNEL no longer B alone in LAYER and SPLIT */
{
    if (M.mode == SY_M_SINGLE) return sqr[1].on || sq_metro == 2u ? 3u : 1u;   /* (1.19.3: SEQ B in SINGLE, MIDI
                                          * only: running, its input; RECORD armed on it, recorded, as the keys) */
    if (M.mode == SY_M_CHANNELS) return in_parts(st);   /* (1.19.4: B IN OMNI too) */
    return M.mode == SY_M_LAYER ? 3u : M.mode == SY_M_SPLIT ? (d1 < (uint32_t)M.split ? 2u : 1u) : 1u;
}
static UI_LATE int sq_midi_in(uint32_t st, uint32_t d1, uint32_t d2)   /* audio interrupt: a MIDI message in; 1 = taken
                                          * (a note for a part whose sequencer runs: its input; for the other part
                                          * too (LAYER): played there alone) */
{
    const uint32_t t = st & 0xF0u, b = 1u << (d1 & 31u), w = d1 >> 5 & 3u;
    uint32_t to, run, k, rec;
    if (t == 0xB0u && d1 == 123u) sq_panic();
    if (t != 0x80u && t != 0x90u) return 0;
    to = sq_parts_in(st, d1);
    rec = sr_mi_in(t, d1, d2, to);       /* (1.18C31: RECORD armed on a part: recorded and heard there alone, as
                                          * RECORD's keys: not its input, not the other part's in LAYER) */
    for (k = 0; k < SQ_NI; k++)          /* (on: the parts' sequencers; off: both forget it) */
        if (t == 0x90u && d2 && (to >> k & 1u) && !rec) {
            sqr[k].midi[w] |= b;
            sq_lat_on(&sqr[k], w, b);
        } else if (t == 0x80u || !d2) sqr[k].midi[w] &= ~b;
    if (rec) return 1;
    run = (sqr[0].on ? 1u : 0u) | (sqr[1].on ? 2u : 0u);
    if (!(to & run)) return 0;           /* (no sequencer of its parts runs: as before) */
    if (to & ~run) eng_to(to & ~run & 1u ? EM_A : EM_B, st, d1, d2);   /* (the part not sequenced plays it) */
    return 1;
}
#else
static UI_LATE void sq_panic(void)       /* audio interrupt: all notes off (CC 123): the MIDI notes held in forgotten */
{
    srm[0] = srm[1] = srm[2] = srm[3] = 0;
    sq.midi[0] = sq.midi[1] = sq.midi[2] = sq.midi[3] = 0;
    sq_lat_clr();                        /* (KEY LATCH's notes too) */
    if (sq.on) sq_all_off();
}
static UI_LATE int sq_midi_in(uint32_t st, uint32_t d1, uint32_t d2)   /* audio interrupt: a MIDI message in; 1 = taken
                                          * (a note while it runs: input, not played) */
{
    uint32_t t = st & 0xF0u, b = 1u << (d1 & 31u), w = d1 >> 5 & 3u;
    if ((t == 0x80u || t == 0x90u) && sr_mi_in(t, d1, d2, 1u)) {   /* (1.18C31: RECORD armed: recorded, not input) */
        if (t == 0x80u || !d2) sq.midi[w] &= ~b;
        return 1;
    }
    if (t == 0x90u && d2) { sq.midi[w] |= b; sq_lat_on(&sq, w, b); }
    else if (t == 0x80u || t == 0x90u) sq.midi[w] &= ~b;
    else if (t == 0xB0u && d1 == 123u) sq_panic();
    return sq.on && (t == 0x80u || t == 0x90u);
}
#endif

/* the engine's measurement of a block's parts (SY_PROF in fm1.toml's o2_cflags) */
extern uint32_t (*sy_prof_clock)(void);
extern uint32_t sy_prof[7], sy_prof_v[3], sy_split[2];
static float prof_avg[10];
static volatile uint8_t prof_shown;      /* HOME held: the times are smoothed (else not: nothing reads them) */

#ifndef HYB_ONE_CORE
#define HYB_ONE_CORE 0                   /* 1: the second core left off (a comparison build) */
#endif

/* the governor (app_render): the share of the block it keeps the voices under, % */
#define GOV_PCT 95.f                     /* (A013, one key, three voices: about 94 % on the device, played clean) */
#define GOV_XR_BLOCKS 69u                /* the governor: one step for the xruns of ~200 ms at most (128-frame blocks) */
#define GOV_SLOT_MIN 4u                  /* the voices sounding for a voice's cost to be taken */
#define GOV_CALM_PCT 60u                 /* calm: the load under this, */
#define GOV_CALM_PK 75u                  /* no block over this (% of its time), */
#define GOV_CALM_BLOCKS 690u             /* and no dropout, for ~2 s */
#define GOV_FLOOR 4                      /* the limit after dropouts at the least (or the cap) */
#ifndef GOV_FLOOR2
#define GOV_FLOOR2 1                     /* (after 1.18C34) 1: the floor two notes' worth at the least, a note's voices as
                                          * the patch and the unison caps give them now (sy_note_voices: a test patch with
                                          * UNISON OFF takes 4 a note, the floor 8; UNISON MAX 2: 4), never over the cap.
                                          * Under it a second key could only steal the first's voices. 0: GOV_FLOOR */
#endif
#ifndef GOV_FIX
#define GOV_FIX 1                        /* (after 1.18C32) 1: the voices that fit from the two cores' model: a voice's
                                          * cost c (the first core's voices' time over its count, sy_split), the fixed
                                          * part F (the load less the voices' window: the part's head, the control
                                          * blocks, the effects' end) and the second core's other work x beside its
                                          * voices (the effects with SY_FX_PIPE: from the split's balance and the wait,
                                          * x = (2 n0 - nv) c + wait): nv = (2 (GOV_PCT - pen - F) - x) / c (one core:
                                          * (GOV_PCT - pen - F) / c). 1.18C32 took a voice slot as the voices' share
                                          * over half the voices: with few voices, the first core taking most of them
                                          * (the second has the effects) and the wait, a voice looked 2-3 times dearer
                                          * (EURO at the device's rate: 8 voices at 70 %, the limit held at its floor
                                          * of 4, calm never reached over GOV_CALM_PCT). MED and HIGH: a block's own
                                          * time counts against calm from GOV_CALM_PKF on (the FIFO plays a longer
                                          * block without a dropout; the dropouts count as before). 0: as 1.18C32 */
#endif
#define GOV_CALM_PKF 150u                /* (GOV_FIX, MED and HIGH) a block not calm from this (% of its time) */
#ifndef GOV_BUD
#define GOV_BUD 1                        /* (1.18C49) 1: the budget: a patch whose sounding voices really
                                          * cost more than the block allows (the load averaged at BUD_ON or more for
                                          * BUD_ONN blocks in a row) is held under BUD_PCT instead of GOV_PCT (the
                                          * room for a chord's onset: its new voices' first blocks, the fades over the
                                          * cap), and the voices over the governor's limit then fade out early over
                                          * BUD_FADE blocks, BUD_GAP blocks apart (sy_shed's order: the quietest
                                          * release tails first, then held notes' extra unison voices; a note's last
                                          * voice never; the engine's sy_bud: no voice over the limit meanwhile). It
                                          * ends with the governor's calm (the load under GOV_CALM_PCT for ~2 s) or a
                                          * patch loaded. Below BUD_ON nothing changes (the output to the bit). 0: as
                                          * 1.18C48 */
#endif
#if GOV_BUD
#define BUD_ON 88u                       /* (% of a block, the average) the budget acts from this, */
#define BUD_ONN 32u                      /* for this many blocks in a row (~90 ms: a chord's onset alone never) */
#define BUD_PCT 72.f                     /* the governor's share of the block while it acts */
#define BUD_FADE 15                      /* an early fade's blocks (~41 ms) */
#define BUD_GAP 8u                       /* blocks between two early fades */
static uint8_t bud_on;                   /* the budget acts */
static uint32_t bud_run, bud_blk;        /* blocks in a row at BUD_ON; the last early fade's */
static uint16_t bud_n;                   /* the voices faded early since power-on (GLO > ABOUT > DIAGNOSIS's T) */
#endif
#ifdef PLAT_HOST
static int host_gov[4] = {-1, 0, 0, 0};  /* (the host simulator, hostcmd.c "hgov": the governor's figures faked, % of a
                                          * block: the control blocks, a voice slot on the first core, the wait for the
                                          * second, the rest; -1: the computer's own) */
static int host_pk = -1;                 /* (hostcmd.c "hpk": a block's own time the governor's calm sees, %; -1 none) */
#endif
static float gov_rest = 30.f, gov_slot;
static uint32_t gov_busy;                /* the last block that was not calm (app_render) */
static uint32_t gov_xr;                  /* the xruns it has seen */

/* GLO > AUDIO > REVERB AUTO and the overload guard (after 1.18C30). rvq_tick runs after every
 * block (app_render) with p, that block's render time, and pct, the platform's load smoothed over ~16 blocks (both % of
 * the block's time: the first core's render with its wait for the second, the path a block takes). A block's own time
 * counts with LATENCY LOW alone (it renders in the audio interrupt: a block past its time is a dropout); MED and HIGH
 * render ahead (a block may run past its time as long as the average stays under it): pct and the dropouts there.
 * AUTO: the reverb FULL while the load allows it; HALF (about a third of FULL's reverb for
 * the figures) from a block at RVA_UP_PK or the load at RVA_UP; FULL again after RVA_HOLD blocks at least and RVA_CALM
 * blocks in a row with the load under RVA_DOWN (RVA_UP less FULL's own share and a margin: back in FULL the load stays
 * under RVA_UP, no switching to and fro). Each switch is the reverb's own safe path (hyb_rv_set_full: the tail faded out
 * over a block, the ring cleared, the other path from silence, as a REVERB chosen in GLO), its tail cut: under load
 * many notes are being struck, which cover it.
 * The guard (every mode): a block at GRD_PK or more, or an xrun, sheds load at once: the reverb HALF if it runs FULL
 * (FULL chosen too: back as AUTO comes back, the line reads HALF (LOAD) meanwhile), and GRD_N voices faded out over
 * GRD_FADE blocks (sy_shed: the release tails first, the quietest first, then held notes' unison voices), the
 * governor's limit under what sounds now; again after GRD_GAP blocks (what it shed is gone by then) while it lasts. So
 * the blocks do not stay past their time: no run of dropouts, the main loop and the second core's job never starved
 * for long (the emulator at the device's rate) */
#define RVA_UP_PK 85u                    /* (% of a block) AUTO: HALF from a block over this, */
#define RVA_UP 70u                       /* or the load over this */
#define RVA_DOWN 55u                     /* FULL again: the load under this */
#define RVA_CALM 345u                    /* for ~1 s in a row (blocks), */
#define RVA_HOLD 1035u                   /* and HALF held ~3 s at least */
#define GRD_PK 95u                       /* the guard: a block over this (% of its time) */
#define GRD_N 2                          /* the voices it sheds at a time (a voice a core) */
#define GRD_FADE 2                       /* over these blocks (~6 ms: straight in power, no click) */
#define GRD_GAP 4u                       /* blocks before it sheds again */
#ifndef GRD_FIX
#define GRD_FIX 1                        /* (after 1.18C34) 1: MED and HIGH (the guard sees the load smoothed over ~16
                                          * blocks) shed again only after GRD_GAP_AVG blocks: what it shed shows in the
                                          * average that late, every GRD_GAP blocks it shed on and on for the same
                                          * overload (two keys of a test patch on the device: the first note shed whole,
                                          * then the second's unison voice). The reverb first: the block that takes a
                                          * FULL reverb HALF (AUTO's or the guard's) sheds no voices, GRD_GAP blocks
                                          * later it may (the reverb's load on the second core counts only through the
                                          * wait). The engine keeps each held note a voice (synth.c SY_SHED_KEEP). 0: as
                                          * 1.18C34 */
#endif
#define GRD_GAP_AVG 16u                  /* (GRD_FIX, MED and HIGH) blocks between two sheds */
#ifndef RVA_FIX
#define RVA_FIX 1                        /* (1.18C36, the user's device test: a reverb tail cut while playing) 1: a switch
                                          * of the reverb's path clears its ring (the tail cut), and HALF saves little on
                                          * the device (the reverb 11.3 -> 9.6 % of a block, devbench): REVERB FULL stays
                                          * FULL (the guard sheds voices only, no HALF (LOAD)); AUTO goes HALF only on a
                                          * sustained load (the average at RVA_UP for RVA_UPN blocks in a row: no single
                                          * block or dropout) and back to FULL only while the reverb is silent (the
                                          * effects at rest, the output's zeros known, for RVA_REST blocks in a row, or
                                          * no voice for RVA_IDLE blocks), after RVA_HOLD. 0: as 1.18C35 */
#endif
#define RVA_UPN 64u                      /* (RVA_FIX) AUTO: HALF after the average at RVA_UP this many blocks (~0.18 s) */
#define RVA_REST 16u                     /* (RVA_FIX) FULL again: the effects at rest this many blocks in a row (~44 ms), */
#define RVA_IDLE 3450u                   /* (RVA_FIX) or no voice sounding this long (~10 s: a tail that never rests) */
static volatile uint8_t rva_half PLAT_POOL;   /* AUTO (or the guard over FULL) runs the reverb HALF */
static uint8_t rva_told PLAT_POOL;       /* what the engine was told last (hyb_rv_set_full): FULL 2, HALF 1, 0 nothing */
static uint32_t rva_blk PLAT_POOL, rva_calm PLAT_POOL;   /* the block of the last switch; the calm blocks in a row */
static uint32_t grd_blk PLAT_POOL;       /* the block the guard last shed */
#if RVA_FIX
static uint32_t rva_up PLAT_POOL, rva_rest PLAT_POOL, rva_idle PLAT_POOL;   /* blocks in a row: the load at RVA_UP; the
                                          * output's zeros known (app_render); no voice */
#endif
static volatile uint16_t grd_n PLAT_POOL;   /* the voices the guard shed since power-on (GLO > ABOUT > DIAGNOSIS) */
static void rvq_apply(void)              /* the reverb's path as REVERB and AUTO's state say (taken at its next block) */
{
    const uint32_t q = RVQ_Q(set.rv_q);
    const int full = q == RVQ_HALF ? 0 : !rva_half;
    if ((uint8_t)(full + 1) != rva_told) {
        rva_told = (uint8_t)(full + 1);
        hyb_rv_set_full(full);
    }
}

/* the load and the voices, for the screen */
static volatile uint32_t stat_voices, blocks;
static int gov_floor(void)               /* the limit's floor: GOV_FLOOR, with GOV_FLOOR2 two notes' worth (or the cap) */
{
    int f = GOV_FLOOR;
#if GOV_FLOOR2
    static int nv = 1;
    static uint32_t seen = ~0u;
    if (seen != blocks >> 6) {           /* (the engine asked every 64 blocks: a patch or a cap changed takes ~0.2 s) */
        seen = blocks >> 6;
        nv = sy_note_voices(&S);
    }
    if (2 * nv > f) f = 2 * nv;
#endif
    return voice_cap() < f ? voice_cap() : f;
}
#ifdef HYB_MULTI
static volatile uint32_t stat_vab;        /* (the VOICES tile: A's and B's counts of one block, stat_voices') */
#endif

static volatile uint32_t blk_peak;      /* the longest block since the screen looked, % of its time (the load figure is
                                          * smoothed over 16 blocks: a chord's first block that clicks hides in it) */
void sy_fx_reset(sy_synth *s);           /* (the engine's sy_int.h: the effects' buffers and states cleared) */
void hyb_rv_clear(void);                 /* (the reverb's: its ring and its input filter cleared at its next block, the
                                          * shimmer's buffer too: sy_cln_rv.c) */
#ifndef HYB_HEALTH_LOG
#define HYB_HEALTH_LOG 0                 /* 1: the debug log's health lines (0.64; hl_*); off since
                                          * 0.67, none of its code, data or work in the image (-DHYB_HEALTH_LOG=1) */
#endif
#if HYB_HEALTH_LOG
int hyb_rv_health(uint32_t *h);          /* (its state for the debug log: hl_tick) */
static UI_LATE void hl_paint(void);
#endif
static UI_LATE void fx_clear(void)       /* audio interrupt: the effects' tails cut (the chorus's, the delay's and the
                                          * phaser's buffers, the EQ's and the reverb's states: sy_fx_reset, as at the
                                          * engine's start; the reverb's lines) */
{
    uint32_t i;
    sy_fx_reset(&S);
#ifdef HYB_MULTI
    sy_fx_reset(&SB);
    if (S.rv_buf)                        /* (the reverb's lines: SH's, by pointer; none with the exact reverb) */
        for (i = 0; i < 4096u; i++) S.rv_buf[i] = 0.f;
#else
    for (i = 0; i < sizeof S.rv_buf / sizeof S.rv_buf[0]; i++) S.rv_buf[i] = 0.f;
#endif
    {   /* (the reverb's ring is the delay's memory: sy_fx_reset cleared its first 80 KB, not the rest nor the reverb's
         * input filter. By a pointer: in RAM, beyond a direct call's reach from here) */
        void (*volatile rs)(void) = hyb_rv_clear;   /* (1.18C49: the shimmer's buffer too) */
        rs();
    }
}
static UI_LATE void fx_cut(void)         /* audio interrupt: the effects' tails cut, the drums' voices silenced */
{
    fx_clear();
    dr.act = 0;
}
static UI_LATE void eng_wake(void)       /* audio interrupt: the synth on again (ENGINE), from silence: its voices freed
                                          * at once (as the engine's mono mode frees them), its arpeggiator's keys
                                          * gone, nothing left in its queue; its first block not heard and the
                                          * effects cleared after it (app_render: their tails from before; the
                                          * effects' pipe, SY_FX_PIPE, still holds a block of voices from before it
                                          * stopped, which that block takes into them) */
{
    uint32_t i;
    sy_all_off(&S);
#ifdef HYB_MULTI
    sy_all_off(&SB);
    SB.ev_r = SB.ev_w;                   /* (S.v: the whole pool, both parts' voices, below) */
#endif
    for (i = 0; i < SY_MAX_VOICES; i++) { S.v[i].on = S.v[i].gate = 0; S.v[i].aenv.stage = 0; }
    S.ev_r = S.ev_w;
    S.xclk_n = 0;                        /* (the arpeggiator's tempo glides in again from the next tick) */
    S.xclk_arp = 0.f;
    eng.mute = 1;
}
static UI_LATE void eng_ramp(int32_t *lr)   /* audio interrupt: a block of the fade (eng.fade's: from fade / ENG_FADE
                                          * to a step less, frame by frame) */
{
    uint32_t k;
    const float f0 = (float)eng.fade * (1.f / (float)ENG_FADE), d = -1.f / (float)(ENG_FADE * SY_BLOCK);
    for (k = 0; k < SY_BLOCK; k++) {
        const float f = f0 + d * (float)k;
        lr[2u * k] = (int32_t)((float)lr[2u * k] * f);
        lr[2u * k + 1u] = (int32_t)((float)lr[2u * k + 1u] * f);
    }
}
static UI_LATE void eng_panic(void)      /* audio interrupt (1.18C49, a device report: PANIC left the reverb ringing):
                                          * all sound off. The whole output fades out over ENG_FADE blocks (~46 ms,
                                          * the drums' too) as ENGINE's MIDI ONLY would, then comes back as ENGINE's
                                          * own way back does (eng_wake: every voice freed, the queue's notes gone, a
                                          * silent block, then every effect cleared: both parts' delay, chorus,
                                          * phaser, EQ, the reverb's ring and the shimmer's buffer, hyb_rv_clear): the
                                          * next note into a fresh, empty reverb. Notes struck during the fade are not
                                          * heard. The synth off (ENGINE) or fading already: the tails cut, as before */
{
    if (eng.now == EN_ALL && !eng.fade) {
        eng.to = EN_MIDI;
        eng.fade = ENG_FADE;
    } else fx_cut();
}
static UI_LATE void eng_step(void)       /* audio interrupt, a block's start: ENGINE's mode (set.eng) taken: a part
                                          * going off fades (eng.fade), one coming on starts at once */
{
    const uint32_t want = set.eng < EN_N ? set.eng : EN_ALL;
    if (eng.fade) {
        if (--eng.fade == 0u) {          /* (faded out: off) */
            eng.now = eng.to;
            if (eng.now == EN_MIDI) dr.act = 0;
        }
    } else if (want != eng.now) {
        eng.to = (uint8_t)want;
        if (want > eng.now) {            /* (the synth, the drums or both go off: the synth's notes released now) */
            eng.fade = ENG_FADE;
            if (eng.now == EN_ALL) sy_all_off(&S);
#ifdef HYB_MULTI
            if (eng.now == EN_ALL) sy_all_off(&SB);
#endif
        } else {                         /* (the synth, the drums or both come on) */
            if (want == EN_ALL) eng_wake();
            if (eng.now == EN_MIDI) dr.act = 0;   /* (the drums' hits while silent not heard late) */
            eng.now = (uint8_t)want;
        }
    }
    eng.syn = (uint8_t)(eng.now == EN_ALL && !(eng.fade && eng.to != EN_ALL));
}
static UI_LATE void tp_set(uint32_t v)   /* audio interrupt (and before it starts): the tempo running, byte v (bpm -
                                          * 63), to the engine, both parts in the USB build (one tempo: the sequencers'
                                          * clock and each part's LFO and arpeggiator sync) */
{
    if (S.patch[TP_BYTE] != v) sy_set_param(&S, TP_BYTE, (int)v);
#ifdef HYB_MULTI
    if (SB.patch[TP_BYTE] != v) sy_set_param(&SB, TP_BYTE, (int)v);
#endif
    tp_run = (uint8_t)(v + 1u);
}
#ifdef HYB_MULTI
static UI_LATE void pv_glide(void)       /* audio interrupt, a block: the parts' volumes to the engine, glided as
                                          * SYNTH VOL (at 100 exactly 1: the output to the bit) */
{
    static float pg[2] = {1.f, 1.f};
    uint32_t k;
    for (k = 0; k < 2u; k++) {
        const float pt = (fv_aud ? 2u : ln_mute) >> k & 1u ? 0.f : (float)pv_vol[k] * (1.f / (float)PV_UNITY), d = pt * pt - pg[k];   /* (the
                                          * launcher's MUTE, glided) */
        pg[k] = d < 1e-5f && d > -1e-5f ? pt * pt : pg[k] + d * 0.25f;
    }
    S.rt_g = pg[0];
    SB.rt_g = pg[1];
}
#endif
/* after a block (app_render): AUTO's choice and the guard (rvq_apply's notes). p: the block's render time, pct: the load
 * smoothed (% of the block); xr: an xrun since the last block (not a flash write's) */
#ifdef PLAT_HOST
static int host_load = -1;               /* (the host simulator, hostcmd.c "hload": the blocks' load rvq_tick sees, % of a
                                          * block; -1: AUTO and the guard idle there, its clock is the computer's) */
#endif
static void rvq_tick(uint32_t p, uint32_t pct, int xr)
{
#ifdef PLAT_HOST
    if (host_load < 0) { rvq_apply(); return; }
    p = pct = (uint32_t)host_load;
    xr = 0;
#endif
    const uint32_t q = RVQ_Q(set.rv_q);
    const uint32_t pk = set.lat == LAT_LOW ? p : pct;   /* (MED, HIGH: the average; a block may run past its time) */
    const int over = pk >= GRD_PK || xr;
#if GRD_FIX
    int rv_now = 0;                      /* (the reverb taken HALF by this block: no voices shed with it) */
    static uint32_t grd_gap = GRD_GAP;   /* (the blocks before the guard sheds again) */
#else
    const uint32_t grd_gap = GRD_GAP;
#endif
    if (eng.now == EN_ALL) {
#if RVA_FIX
        rva_up = pct >= RVA_UP ? rva_up + 1u : 0u;
        rva_idle = stat_voices ? 0u : rva_idle + 1u;
#endif
        if (!rva_half) {
#if RVA_FIX
            if (q == RVQ_AUTO && rva_up >= RVA_UPN) {   /* (FULL chosen: never HALF; AUTO: a sustained load alone) */
#else
            if ((q == RVQ_AUTO && (pk >= RVA_UP_PK || pct >= RVA_UP || xr)) || (q == RVQ_FULL && over)) {
#endif
                rva_half = 1;            /* (the reverb HALF from its next block) */
                rva_blk = blocks;
                rva_calm = 0;
#if GRD_FIX
                rv_now = 1;
#endif
            }
        } else {
            rva_calm = pct < RVA_DOWN && !over ? rva_calm + 1u : 0u;
#if RVA_FIX
            if (q != RVQ_AUTO || (blocks - rva_blk >= RVA_HOLD && (rva_rest >= RVA_REST || rva_idle >= RVA_IDLE))) {
#else
            if (q == RVQ_HALF || (blocks - rva_blk >= RVA_HOLD && rva_calm >= RVA_CALM)) {
#endif
                rva_half = 0;            /* (FULL again; or HALF chosen: nothing held) */
                rva_blk = blocks;
            }
        }
#if GRD_FIX
        if (over && rv_now && blocks - grd_blk >= grd_gap) {   /* (the reverb HALF first: the next shed GRD_GAP on) */
            grd_blk = blocks;
            grd_gap = GRD_GAP;
        } else
#endif
        if (over && blocks - grd_blk >= grd_gap) {   /* the guard: voices faded out, the limit under what sounds */
            int k = sy_shed(&S, GRD_N, GRD_FADE);
#ifdef HYB_MULTI
            if (M.mode_now != SY_M_SINGLE && k < GRD_N) k += sy_shed(&SB, GRD_N - k, GRD_FADE);
#endif
            if (k > 0) {
                const int fl = gov_floor(), left = (int)stat_voices - k;
                grd_blk = blocks;
#if GRD_FIX
                grd_gap = set.lat == LAT_LOW ? GRD_GAP : GRD_GAP_AVG;
#endif
                if (grd_n < 0xFFFFu) grd_n = (uint16_t)(grd_n + (uint32_t)k);
                if (GOV_MAX > left) GOV_MAX = left > fl ? left : fl;
            }
        }
    }
    rvq_apply();
}

void app_render(int32_t *lr, uint32_t n, uint32_t master_q12)
{
    uint32_t i, pkt, pct, t_in = plat_ticks24(), nz = 0u;   /* (nz: the blocks not known to be zeros) */
    int port;
    const float g = (float)master_q12 * (8388607.f / 4096.f);   /* full scale 1.0 -> 24 bit, at MASTER */
    static float vg = 1.f;               /* the synth's volume (set.vol) as a gain, glided a block at a time */
    if (patch_req >= 0) {                /* a patch change: between two blocks */
#ifdef HYB_MULTI
        sy_synth *pp = patch_req & PATCH_REQ_B ? &SB : &S;   /* (two parts: PATCH_REQ_B, part B's) */
        sy_all_off(pp);
        const uint32_t tb = S.patch[TP_BYTE];   /* (the tempo running) */
        sy_all_off(pp);
        sy_load_patch(pp, pbuf);
        if (sqr[pp == &SB].hold) sy_set_param(pp, 129, 0);   /* (the part's sequencer has a pattern: its
                                          * arpeggiator off) */
        tp_set(tb);                      /* (1.18: the global tempo kept, no patch sets it, either part's) */
#else
        const uint32_t tb = S.patch[TP_BYTE];
        sy_all_off(&S);
        sy_load_patch(&S, pbuf);         /* (always PATCH_REQ_BUF: a user patch is read from the store in the main
                                          * loop, never here) */
        if (sq.hold) sy_set_param(&S, 129, 0);   /* (the sequencer has a pattern: the patch's arpeggiator held off) */
        tp_set(sq.on || dr.on ? tb : S.patch[TP_BYTE]);   /* (a sequencer runs: its tempo kept, the patch's byte not
                                          * changed; stopped: the new patch's) */
#endif
        patch_req = -1;
#if GOV_BUD && GOV_FIX
        bud_on = 0;                      /* (a patch loaded: its own cost decides again) */
#endif
    } else if (uw_seq != uw_seen)
        uw_render();                     /* (a user wave changed: a table holding it built again, as a patch load does) */
#ifdef HYB_SHIMMER
    {   /* FX > SHIMMER: part A's values to the reverb (shx_sync) */
        static uint8_t seen;
        if (shx_seq != seen) {
            uint32_t k;
            seen = shx_seq;
            hyb_rv_shim(shx_eng);
            for (k = 0; k < SHX_NR; k++) S.shx_rt[k] = shx_eng[SHX_N + k];
        }
    }
#endif
    if (lat_req >= 0) {
        if (lat_req) S.opt |= SY_OPT_LATENCY; else S.opt &= ~(uint32_t)SY_OPT_LATENCY;
        if (GOV_MAX > voice_cap()) GOV_MAX = voice_cap();
        lat_req = -1;
    }
    eng_step();                          /* ENGINE's mode */
    sq_pv_cut_a();                       /* (a step's preview: its note before ends, 0.85) */
    sq_block(n);                         /* the step sequencer */
    while (ev_r != ev_w) {               /* the panel's */
        uint32_t e = evq[ev_r % EVQ];
#ifdef HYB_MULTI
        if ((e >> 8 & 0xFFu) == TP_BYTE && (e & 0xFFu) - EV_PARAM <= EV_PARAM_A - EV_PARAM)   /* (the tempo: the
                                          * global one, both parts', from either part's byte, 1.18) */
            tp_set(e >> 16 & 0x7Fu);
        else if ((e & 0xFFu) - EV_PARAM <= EV_PARAM_A - EV_PARAM) {   /* (a part's byte; its arpeggiator held
                                          * off while its sequencer runs) */
            const uint32_t b = (e & 0xFFu) == EV_PARAM_B;
            eng_to(b ? EM_B : EM_A, SY_EV_PARAM, e >> 8 & 0xFFu, sqr[b].hold && (e >> 8 & 0xFFu) == 129u ? 0u : e >> 16 & 0xFFu);
        } else if ((e & 0xFFu) == EV_KEY_A || (e & 0xFFu) == EV_KEY_B)   /* (a key's note to one part alone) */
            eng_to((e & 0xFFu) == EV_KEY_B ? EM_B : EM_A, e >> 16 & 0xFFu ? 0x90u : 0x80u, e >> 8 & 0xFFu, e >> 16 & 0xFFu);
#else
        if ((e & 0xFFu) - 1u < 2u && (e >> 8 & 0xFFu) == TP_BYTE) tp_set(e >> 16 & 0x7Fu);   /* (the tempo: the
                                          * running one) */
        else if ((e & 0xFFu) == EV_PARAM) eng_midi(SY_EV_PARAM, e >> 8 & 0xFFu,
                                              sq.hold && (e >> 8 & 0xFFu) == 129u ? 0u : e >> 16 & 0xFFu);
#endif
        else {
            if ((e & 0xFFFFu) == (0xB0u | 123u << 8)) sq_panic();   /* (PANIC: the sequencer's notes and input too; CC
                                          * 120, the tails and the drums: eng_panic, from eng_midi) */
            eng_midi(e & 0xFFu, e >> 8 & 0xFFu, e >> 16 & 0xFFu);
        }
        ev_r++;
    }
    sq_pv_on_a();                        /* (its note now, after a hold of the arpeggiator) */
    while (plat_midi_in(&pkt, &port)) {  /* USB, the TRS jack, Bluetooth (GLO > MIDI: the ports, the channel, thru,
                                          * the clock) */
        uint32_t st = (pkt >> 8) & 0xFFu, cin = pkt & 0x0Fu;
        const uint32_t rt = st == 0xF8u || st == 0xFAu || st == 0xFBu || st == 0xFCu;   /* (clock and transport) */
        if ((uint32_t)port < NPORTS && (set.in_off >> port & 1u))
            continue;                    /* (an IN port off: nothing from it, not even thru; SysEx comes another way,
                                          * the platform's, and is not touched) */
        if (set.thru && (st & 0x80u) && (st < 0xF0u || st >= 0xF8u) && (cin < 4u || cin > 7u)
            && !(rt && set.clk == CLK_MASTER))
            plat_midi_send(pkt, midi_to() & ~plat_midi_in_from());   /* thru: the channel messages and the real-time
                                                     * ones (no SysEx; MASTER: not the clock, the FM-1 sends its own),
                                                     * to the other ports of OUT PORTS, never back */
        if (st >= 0xF0u && !rt)
            continue;                    /* (clock and transport to the engine; SysEx and the rest not) */
        if (dr_mi_in(pkt)) continue;     /* (1.18C31: DRUM CHANNEL's notes, the drums') */
#ifdef HYB_MULTI
        if (rt ? set.clk != CLK_EXT || (clk_src_eff() != CS_ANY && (uint32_t)port != CS_PORT[clk_src_eff() & 3u])
               : (st & 0x80u) && !in_parts(st))   /* (B's channel too: CHANNELS alone, 1.18C31; 1.19.4: A IN OFF,
                                          * B IN OMNI) */
            continue;
#else
        if (rt ? set.clk != CLK_EXT || (clk_src_eff() != CS_ANY && (uint32_t)port != CS_PORT[clk_src_eff() & 3u])
               : (st & 0x80u) && set.midi_ch && (st & 0x0Fu) + 1u != set.midi_ch)
            continue;
#endif                    /* (the clock not taken: INTERNAL, MASTER, another port than SOURCE's; a
                                          * channel not listened to) */
        if (st == 0xFAu || st == 0xFBu) seq_req = 1u;   /* (CLOCK EXTERNAL's START / CONTINUE: the sequencers with a pattern
                                          * start; STOP: they stop; input() takes it) */
        else if (st == 0xFCu) seq_req = 2u;
#ifdef HYB_MULTI
        if (rt) xk_rt(st);               /* (1.19.5: CLK LOCK's count) */
#endif
        if (st < 0xF0u && sq_midi_in(st, pkt >> 16 & 0x7Fu, pkt >> 24 & 0x7Fu))
            continue;                    /* (the sequencer runs: a note is its input) */
#ifdef HYB_MULTI
        if ((st & 0xF0u) == 0xB0u && cm_in(pkt >> 16 & 0x7Fu, pkt >> 24 & 0x7Fu)) continue;   /* (1.19.6: the MIDI CC
                                          * map's moved lines, learn) */
        if ((st & 0xF0u) == 0xB0u && pv_cc(pkt >> 16 & 0x7Fu, pkt >> 24 & 0x7Fu)) continue;   /* (VOL A / B CC) */
#endif
        if ((st & 0xF0u) == 0xB0u && (pkt >> 16 & 0x7Fu) == 7u && set.vol_cc) {
            set.vol = (uint8_t)(pkt >> 24 & 0x7Fu);   /* (CC 7: the synth's volume, not saved by itself; GLO shows it) */
            continue;
        }
        if ((st & 0xF0u) == 0xB0u) re_cc = 1u;   /* (the remote editor told, remote.c: the engine's page A) */
#ifdef HYB_MULTI
        if ((st & 0xF0u) == 0xB0u && set.cm_show)   /* (CC SHOW: cp_run says it) */
            fm1cp_post(&cc_pop, pkt >> 16 & 0x7Fu, pkt >> 24 & 0x7Fu, in_parts(st));
#endif
#ifdef HYB_MULTI
        if (M.mode == SY_M_CHANNELS && (set.mo & MO_BOMNI)) {   /* (1.19.4: B IN OMNI: part B every channel, A by
                                          * A IN) */
            if (in_parts(st) & 1u) eng_to(EM_A, st, pkt >> 16 & 0x7Fu, pkt >> 24 & 0x7Fu);
            eng_to(EM_B, st, pkt >> 16 & 0x7Fu, pkt >> 24 & 0x7Fu);
        } else
        eng_to(M.mode == SY_M_CHANNELS ? EM_IN : EM_KEYS, st, pkt >> 16 & 0x7Fu, pkt >> 24 & 0x7Fu);   /* (1.18C31:
                                          * as the keys but in CHANNELS, where B CHANNEL plays B alone; SPLIT by the
                                          * note number) */
#else
        eng_midi(st, pkt >> 16 & 0x7Fu, pkt >> 24 & 0x7Fu);
#endif
    }
    for (i = 0; i + SY_BLOCK <= n; i += SY_BLOCK) {
        uint32_t k, z = 0u;              /* (z: the synth's block is zeros, known: its effects at rest, or it is off) */
        const float v = (float)set.vol * (1.f / 127.f);
        vg += (v * v - vg) * 0.25f;      /* (a square law; ~10 ms to settle, no zipper steps from a fader) */
#ifdef HYB_MULTI
        pv_glide();                      /* (the parts' volumes the same way) */
#endif
        if (eng.now == EN_ALL) {
#ifdef HYB_MULTI
            z = (uint32_t)sy_multi_render_i24(&M, lr + 2u * i, g * vg);   /* (two parts, or part A alone: SINGLE) */
#else
            z = (uint32_t)sy_render_i24(&S, lr + 2u * i, g * vg);   /* (the clip and the scaling in one pass) */
#endif
            if (eng.fade && eng.to == EN_DRUMS) eng_ramp(lr + 2u * i);   /* (ENGINE: the synth alone fading) */
            if (prof_shown)              /* the parts' times, smoothed (the engine's SY_PROF; while HOME is held) */
                for (k = 0; k < 10u; k++) prof_avg[k] += ((float)(k < 7u ? sy_prof[k] : sy_prof_v[k - 7u]) - prof_avg[k]) * (1.f / 32.f);
        }
#if RVA_FIX
        rva_rest = z || eng.now != EN_ALL ? rva_rest + 1u : 0u;   /* (the reverb silent: AUTO may take FULL again) */
#endif
        if (eng.now != EN_ALL || eng.mute) {   /* (the synth off, or its first block after it came on: silence) */
            for (k = 0; k < 2u * SY_BLOCK; k++) lr[2u * i + k] = 0;
            z = 1u;
            if (eng.mute && !--eng.mute) {   /* (eng_wake: the effects' tails and the old block in them gone: the
                                          * chorus's, the phaser's, the EQ's states here, the delay's and the
                                          * reverbs' memory, the exact reverb's network too, by the engine's own
                                          * reset at an effect mode's change, at the next block: synth.h
                                          * fx_mode_last no mode) */
                fx_clear();
                S.fx_mode_last = 0xFFu;
                for (k = 0; k < 2u; k++)         /* (and what sy_fx_reset leaves: the delay's feedback, the
                                                  * distortion's filters) */
                    S.dl_fbv[k] = S.ds_hp[k] = S.ds_lp[k] = S.ds_hold[k] = 0.f;
#ifdef HYB_MULTI
                for (k = 0; k < 2u; k++)
                    SB.ds_hp[k] = SB.ds_lp[k] = SB.ds_hold[k] = 0.f;
#endif
            }
        }
        if (eng.now != EN_ALL) {         /* (the synth off: the engine's MIDI clock reading goes on, as at the end of
                                          * its block, synth.c sy_render: no tick for 1 s, stopped; its tempo) */
            S.xclk_frames += (float)SY_BLOCK;
            if (S.xclk_on && S.xclk_frames > S.fs) { S.xclk_on = 0; S.xclk_per = 0.f; }
            S.bpm_now = S.xclk_on && S.xclk_per > 0.f ? 60.f * S.fs / (24.f * S.xclk_per)
                                                      : (float)S.patch[128 + 16] + 63.f;
        }
        nz += z ^ 1u;
        if (eng.now != EN_MIDI) {
            dr_render(lr + 2u * i, SY_BLOCK, g);   /* (the drums after it: dry, centred) */
            if (eng.fade && eng.to == EN_MIDI) eng_ramp(lr + 2u * i);   /* (ENGINE: all fading) */
        } else dr.act = S.dr_on = 0;     /* (MIDI ONLY: the drums' hits not heard, now or later) */
    }
    /* the governor: as many voices as fit in GOV_PCT of the block, from what a voice costs now and what the rest
     * costs. The voices' share (their control blocks, the first core's voices, the wait for the second) goes by
     * the first core's slots, half the voices rounded up (the second core takes the other half beside it); the
     * rest is the load less that share. A voice's cost is taken only while GOV_SLOT_MIN voices or more sound (with
     * one or two the share's fixed part, the control blocks and the wait, charged to them made a voice look several
     * times dearer: the limit stayed at 6 in HIGH at a load of 27 %). The limit falls at once and rises a voice at a
     * time; it acts on the next note's voices (a unison note takes fewer), not on those sounding. Calm (the load
     * under GOV_CALM_PCT, no block over GOV_CALM_PK, no dropout, for GOV_CALM_BLOCKS): the limit rises to the cap
     * whatever the estimate says. Dropouts take it to GOV_FLOOR at the least. */
    {
        const uint32_t p = (plat_ticks24() - t_in) * 100u / (uint32_t)(128.f * 1000000.f / 44100.f * (float)PLAT_TICKS_PER_US);
        static uint32_t xr_seen;         /* (the xruns rvq_tick has seen) */
        const uint32_t xr = plat_xruns();
        if (p > blk_peak) blk_peak = p;
#if GOV_FIX
#ifdef PLAT_HOST
        if (host_pk >= 0 ? (uint32_t)host_pk >= (set.lat == LAT_LOW ? GOV_CALM_PK : GOV_CALM_PKF) : p >= GOV_CALM_PK)
            gov_busy = blocks;           /* (hpk: the blocks' time faked; else the computer's, as LOW's) */
#else
        if (p >= (set.lat == LAT_LOW ? GOV_CALM_PK : GOV_CALM_PKF)) gov_busy = blocks;
#endif
#else
#ifdef PLAT_HOST
        if (host_pk >= 0 ? (uint32_t)host_pk >= GOV_CALM_PK : p >= GOV_CALM_PK) gov_busy = blocks;
#else
        if (p >= GOV_CALM_PK) gov_busy = blocks;
#endif
#endif
        pct = plat_cpu_pct();
        rvq_tick(p, pct, xr != xr_seen && !store_recent());   /* (REVERB AUTO, the overload guard) */
        xr_seen = xr;
    }
    blocks++;
#ifdef HYB_MULTI
    {   /* (1.18C25: both parts' counts of the same block, one word for the main loop: A's in the low byte, B's
         * above; read apart, a block between them could show A's as A + B) */
        const uint32_t va = eng.now == EN_ALL ? (uint32_t)sy_voices_active(&S) : 0u,
            vb = eng.now == EN_ALL && M.mode_now != SY_M_SINGLE ? (uint32_t)sy_voices_active(&SB) : 0u;
        stat_voices = va + vb;
        stat_vab = va | vb << 8;
    }
#else
    stat_voices = eng.now == EN_ALL ? (uint32_t)sy_voices_active(&S) : 0u;
#endif
    {   /* busy for the deferred saves (store_wait): a voice, the drums, the transport, the output heard (the tails).
         * The output's peak only while none of the others is: not for blocks known to be zeros (the synth's effects at
         * rest or the synth off, the drums silent), every fourth block else (11 ms against STORE_IDLE_MS) */
        if (stat_voices || dr.act || dr.run || SQ_RUNS) st_busy_t = plat_ms();
        else if (nz && !(blocks & 3u)) {
            uint32_t k, pk = 0;
            for (k = 0; k < 2u * n; k++) {
                const uint32_t a = (uint32_t)(lr[k] < 0 ? -lr[k] : lr[k]);
                if (a > pk) pk = a;
            }
            if (pk > STORE_QUIET_PK) st_busy_t = plat_ms();
        }
    }
    if (eng.now != EN_ALL) {             /* (the synth off: the governor holds, its estimates and the limit as they
                                          * were; the dropouts meanwhile not its; calm counted from the synth's start) */
        gov_xr = plat_xruns();
        gov_busy = blocks;
    } else {
        const float blk = 128.f * 1000000.f / 44100.f * (float)PLAT_TICKS_PER_US, a = 1.f / 16.f;
        float vw = (float)(sy_prof[1] + sy_prof[2] + sy_prof[3]) / blk * 100.f;
#if GOV_FIX
        static float gov_c, gov_f = 30.f, gov_x;   /* (GOV_FIX: a voice's cost, the fixed part, the second core's other
                                          * work, % of a block, smoothed) */
        float v0 = (float)sy_prof[2] / blk * 100.f, wt = (float)sy_prof[3] / blk * 100.f;
        int n0 = (int)sy_split[0], nv = (int)sy_split[1];
#endif
        int want = voice_cap(), vfloor = gov_floor();
        static uint32_t xr_blk;
        static float gov_pen;            /* % below GOV_PCT after dropouts: each one a voice less at once and 6 % more
                                          * room, given back over ~10 s (the load figure is an average; a chord's
                                          * onset or a voice's attack can still run a block over). A burst of xruns
                                          * is one dropout (one step per GOV_XR_BLOCKS at most); those just after a
                                          * flash write (store_recent) none */
        if (plat_xruns() != gov_xr) {
            gov_xr = plat_xruns();
            if (!store_recent() && blocks - xr_blk >= GOV_XR_BLOCKS) {
                xr_blk = gov_busy = blocks;
                gov_pen = gov_pen + 6.f > 30.f ? 30.f : gov_pen + 6.f;
                if (GOV_MAX > vfloor) GOV_MAX--;
            }
        } else if (gov_pen > 0.f)
            gov_pen -= 0.0018f;          /* (6 % in ~3400 blocks) */
#ifdef PLAT_HOST
        if (host_gov[0] >= 0) {          /* (hgov: the figures faked: the control blocks, a voice on the first core
                                          * (half the voices there, rounded up), the wait, the rest) */
            const float w = (float)host_gov[0] + (float)host_gov[1] * (float)((stat_voices + 1u) / 2u);
            pct = (uint32_t)(w + (float)host_gov[2] + (float)host_gov[3]);
            vw = w + (float)host_gov[2];
#if GOV_FIX
            n0 = (int)((stat_voices + 1u) / 2u); nv = (int)stat_voices;
            v0 = (float)host_gov[1] * (float)n0; wt = (float)host_gov[2];
#endif
        }
#endif
        if (pct >= GOV_CALM_PCT) gov_busy = blocks;
        gov_rest += ((float)pct - vw - gov_rest) * a;
        if (stat_voices >= GOV_SLOT_MIN) gov_slot += (vw / (float)((stat_voices + 1u) / 2u) - gov_slot) * a;
#if GOV_FIX
        if (n0 > 0 && nv >= n0) {        /* (the two cores' model: a block with voices on the first core) */
            const float c = v0 / (float)n0, x = (float)(2 * n0 - nv) * c + wt;
            gov_c += (c - gov_c) * a;
            gov_f += ((float)pct - v0 - wt - gov_f) * a;
            gov_x += ((x > 0.f ? x : 0.f) - gov_x) * a;
        }
#endif
#if GOV_BUD && GOV_FIX
        bud_run = pct >= BUD_ON ? bud_run + 1u : 0u;
        if (bud_run >= BUD_ONN) bud_on = 1;
#endif
        if (blocks - gov_busy >= GOV_CALM_BLOCKS) {   /* calm: up to the cap, a voice at a time */
#if GOV_BUD && GOV_FIX
            bud_on = 0;
#endif
            if (GOV_MAX < voice_cap() && (blocks & 63u) == 0u) GOV_MAX++;
        } else {
#if GOV_FIX
            if (gov_c > 0.25f) {
#if GOV_BUD
                const float room = (bud_on ? BUD_PCT : GOV_PCT) - gov_pen - gov_f,
#else
                const float room = GOV_PCT - gov_pen - gov_f,
#endif
                    n = (plat_cpu2_on() ? 2.f * room - gov_x : room) / gov_c;
                want = n < 1.f ? 1 : (int)n;
                if (want > voice_cap()) want = voice_cap();
                if (want < vfloor) want = vfloor;
            }
#else
            if (gov_slot > 0.5f) {
                float slots = (GOV_PCT - gov_pen - gov_rest) / gov_slot;
                want = slots < 1.f ? 1 : 2 * (int)slots;
                if (want > voice_cap()) want = voice_cap();
                if (want < vfloor) want = vfloor;
            }
#endif
            if (want < GOV_MAX) GOV_MAX = want;
            else if (want > GOV_MAX && (blocks & 63u) == 0u) GOV_MAX++;
        }
#if GOV_BUD && GOV_FIX
        if (bud_on && blocks - bud_blk >= BUD_GAP) {   /* the voices over the limit (those fading not counted) */
            int k, live = 0;
#ifdef HYB_MULTI
            for (k = 0; k < S.nv; k++) live += S.v[k].on && !S.v[k].ufade;   /* (the parts' slices) */
            for (k = 0; k < SB.nv; k++) live += SB.v[k].on && !SB.v[k].ufade;
#else
            for (k = 0; k < SY_MAX_VOICES; k++) live += S.v[k].on && !S.v[k].ufade;
#endif
            if (live > GOV_MAX) {
                k = sy_shed(&S, 1, BUD_FADE);
#ifdef HYB_MULTI
                if (!k && M.mode_now != SY_M_SINGLE) k = sy_shed(&SB, 1, BUD_FADE);
#endif
                if (k) {
                    bud_blk = blocks;
                    if (bud_n < 0xFFFFu) bud_n++;
                }
            }
        }
        sy_bud = bud_on;                 /* (the engine: no voice over the limit while it acts) */
#endif
    }
}

void app_panic(void)
{
    sy_all_off(&S);
#ifdef HYB_MULTI
    sy_all_off(&SB);
#endif
}


static UI_LATE void fv_load(void);
static uint8_t shadow[SH_LEN];           /* the patch as the screen and the knobs see it (the engine's copy follows
                                          * through the queue; its extension past it: SH_LEN) */
static uint8_t sx_edit;                  /* shadow holds a single from SysEx (app_sysex), not set.patch: no edits are
                                          * kept for it (em_set), SAVE stores it; another patch chosen ends it */

/* the edits not saved, kept in RAM per patch: a byte's value that differs from the bank's (a patch left and chosen
 * again comes back as it was left; until power-off). Full: the oldest edit of another patch makes room */
#define EM_N 128u
static struct { uint16_t patch; uint8_t byte, val; } em[EM_N] PLAT_POOL;
static uint32_t em_n;
#ifdef HYB_MULTI
#define EM_P(p) ((p) | (uint32_t)ed_part << 15)   /* (two parts: each part's edits of a patch its own, the part edited's:
                                          * part B on A001 does not change part A's A001) */
#else
#define EM_P(p) (p)
#endif
#ifdef HYB_SHIMMER
#define EM_X 0x4000u                     /* em[].patch: the byte is the extension's (256 + byte; a patch's index < 0x4000) */
#define EM_K(p, b) (EM_P(p) | ((b) >= 256u ? EM_X : 0u))
#define EM_PM(e) ((uint32_t)(e) & ~EM_X)  /* (an edit's patch) */
#define EM_B(i) ((uint32_t)em[i].byte | (em[i].patch & EM_X ? 256u : 0u))   /* (an edit's byte) */
#define EM_STORED(p, b) shx_stored((p), (b))
#else
#define EM_K(p, b) EM_P(p)
#define EM_PM(e) ((uint32_t)(e))
#define EM_B(i) ((uint32_t)em[i].byte)
#define EM_STORED(p, b) patch_bytes(p)[b]
#endif

static int em_find(uint32_t p, uint32_t b)
{
    uint32_t i;
    for (i = 0; i < em_n; i++)
        if (em[i].patch == EM_K(p, b) && em[i].byte == (uint8_t)b)
            return (int)i;
    return -1;
}
static void em_drop(uint32_t i)
{
    for (; i + 1u < em_n; i++)
        em[i] = em[i + 1u];
    em_n--;
}
static void em_set(uint32_t p, uint32_t b, uint32_t v)   /* patch p's byte b is v now */
{
    int i;
    if (sx_edit && p == set.patch)
        return;                          /* (the edit buffer holds a SysEx single: the edit is in shadow alone) */
    i = em_find(p, b);
    if (i >= 0) em_drop((uint32_t)i);
    if (v == EM_STORED(p, b))
        return;                          /* (as the bank has it: no edit) */
    if (em_n >= EM_N) {
        uint32_t k;
        for (k = 0; k < em_n && EM_PM(em[k].patch) == EM_P(p); k++)
            ;
        em_drop(k < em_n ? k : 0u);      /* (all this patch's: its oldest) */
    }
    em[em_n].patch = (uint16_t)EM_K(p, b); em[em_n].byte = (uint8_t)b; em[em_n].val = (uint8_t)v;
    em_n++;
}
static int em_edited(uint32_t p)
{
    uint32_t i;
    for (i = 0; i < em_n; i++)
        if (EM_PM(em[i].patch) == EM_P(p))
            return 1;
    return 0;
}

/* patch i with its edits into shadow, and to the engine (at the next block). (The audio interrupt and the main loop
 * share the first core: pbuf is not read while it is written, patch_req being off meanwhile) */
static void shadow_send(void);
#ifdef HYB_SHIMMER
static void shx_sync(void)               /* part A's extension to the engine (at its next block) */
{
    uint32_t k;
#ifdef HYB_MULTI
    const uint8_t *a = ed_part ? shadow_o : shadow;
#else
    const uint8_t *a = shadow;
#endif
    if (fv_aud) return;                  /* (1.18C33: the favorites editor's audition is part A's: fv_ext) */
    for (k = 0; k < SHX_N; k++)
        shx_eng[k] = a[256u + k];
    shx_eng[SHX_TAIL] = shx_tl(a + 256u, a[256u + SHX_GL]);
    for (k = 0; k < SHX_NR; k++)
        shx_eng[SHX_N + k] = (uint8_t)(a[SHX_RB[k]] >= SHX_VD ? a[SHX_RB[k]] - SHX_VD + 1u : 0u);
    RING_PUBLISH();
    shx_seq++;
}
static UI_LATE int shx_forced(void)      /* part A's shimmer on in a reverb mode: the reverb runs HALF (GLO > AUDIO >
                                          * REVERB's line says so) */
{
#ifdef HYB_MULTI
    const uint8_t *a = ed_part ? shadow_o : shadow;
#else
    const uint8_t *a = shadow;
#endif
    return SHX_LIVE && a[256] && a[112] >= 2u && a[112] <= 4u;   /* (1.04: none with Bluetooth on, SHX_LIVE) */
}
#endif
static void load_patch(uint32_t i)
{
    const uint8_t *b = patch_bytes(i);
    uint32_t k;
#ifdef APP_HOST_CMD
    host_pl++;
#endif
    sx_edit = 0;
    for (k = 0; k < 256u; k++)
        shadow[k] = b[k];
#ifdef HYB_SHIMMER
    shx_put(patch_ext(i), shadow);       /* (its extension: as stored, or the defaults) */
#endif
    for (k = 0; k < em_n; k++)
        if (EM_PM(em[k].patch) == EM_P(i))
            shadow[EM_B(k)] = em[k].val;
    shadow_send();
}
static void shadow_send(void)            /* shadow to the engine, whole (at the next block) */
{
    uint32_t k;
    patch_req = -1;
#ifdef HYB_MULTI
    shadow[TP_BYTE] = (uint8_t)(tp_run - 1u);   /* (1.18: a patch loaded on either part, its tempo byte the global
                                          * tempo's: no patch sets it, SAVE stores it, the remote editor shows it) */
#endif
    RING_PUBLISH();
    for (k = 0; k < 256u; k++)
        pbuf[k] = (uint8_t)SHX_EB(k, shadow[k]);
    RING_PUBLISH();
#ifdef HYB_MULTI
    patch_req = PATCH_REQ_BUF | (ed_part ? PATCH_REQ_B : 0);   /* (the part edited) */
#else
    patch_req = PATCH_REQ_BUF;
#endif
    SHX_SYNC();                          /* (its extension: part A's to the reverb) */
}

static void settings_defaults(settings_t *s)
{
    uint32_t k;
    for (k = 0; k < sizeof *s; k++)
        ((uint8_t *)s)[k] = 0;           /* (no device either) */
    s->magic = SET_MAGIC;
#if defined(HYB_MULTI) && FM1B_BLE
    s->lat = LAT_HIGH;                   /* (1.18C42, the user's choice) the Bluetooth two-part build: HIGH as shipped
                                          * (two blocks ahead: the most room for the load's peaks); a LATENCY saved
                                          * stays (the settings object holds it) */
#else
    s->lat = LAT_LOW;                    /* LOW: the standard (the user's choice) */
#endif
#ifdef APP_HOST_CMD
    if (host_old_def) s->lat = LAT_LOW;  /* (the host's "glold") */
#endif
    s->patch = 0;                        /* (A001) */
    s->pt_ok = PT_V2;
    qa_defaults(s);
    glo_defaults(s);
    pm_defaults(s);
    s->sq_ok = SQ_V1;
    s->sq_pat = 0;
    s->dr_ok = DR_V1;
    s->dr_pat = 0;
    s->dr_lev = DR_LEV1;
    s->vo_ok = VO_V1;
    s->vol = 127u;
    s->vol_cc = 1u;
    s->sq_def = (uint8_t)(DEF47 ? 0u : SQ_DEF0);   /* (1.18C48: GLO > SEQ as shipped) */
    md_defaults(s);
#ifdef HYB_MULTI
    mx_defaults(s);
    sqb_defaults(s);
    ve_check(s, 0);
#endif
}

/* the settings object loaded into s: the first release's (SET_LEN_V1 bytes, no device), the second's (SET_LEN_V2, no
 * quick-access list), the third's (SET_LEN_V3, no GLO), the fourth's (SET_LEN_V4, no key lights), the fifth's
 * (SET_LEN_V4K, no PLAY mode), the sixth's (SET_LEN_V5, the patch only as patch8), the seventh's and eighth's (SET_LEN_V6,
 * SET_LEN_V7: their later fields as shipped), ..., the tenth's (SET_LEN_V9: MIDI in and out as shipped, the clock's mode
 * from its CLOCK IN) or this one's; 0 = none or not valid
 * (then the defaults) */
static UI_SMALL int settings_load(settings_t *s)
{
    int n = plat_store_load(OBJ_SETTINGS, s, sizeof *s), ok;
    uint32_t k;
#ifdef HYB_MULTI
    ok = (n == (int)sizeof *s || n == SET_LEN_V18 || n == SET_LEN_V17 || n == SET_LEN_V16 || n == SET_LEN_V15 || n == SET_LEN_V14 || n == SET_LEN_V13 || n == SET_LEN_V12
          || n == SET_LEN_V11 || n == SET_LEN_V10 || n == SET_LEN_V9 ||   /* (1.18C26: V14 and V13 too; 1.18C22-C25
                                          * took a V13 object for none, its settings the defaults) */
#else
    ok = (n == (int)sizeof *s || n == SET_LEN_V9 ||
#endif
          n == SET_LEN_V8 || n == SET_LEN_V7 || n == SET_LEN_V6 || n == SET_LEN_V5 || n == SET_LEN_V4K || n == SET_LEN_V4 || n == SET_LEN_V3 || n == SET_LEN_V2 || n == SET_LEN_V1) && s->magic == SET_MAGIC && s->oct >= -2
         && s->oct <= 2 && s->patch8 <= OLD_INIT && s->lat <= LAT_HIGH && s->ble_off <= 1u;
    if (!ok) {
        settings_defaults(s);
        return 0;
    }
    if (s->lat < LAT_LOW) s->lat = LAT_LOW;   /* (the dropped LOW of the first releases) */
    if (n == SET_LEN_V1 || s->dev_ok != 1u)   /* no device saved */
        for (k = SET_LEN_V1; k < __builtin_offsetof(settings_t, qa_ok); k++)
            ((uint8_t *)s)[k] = 0;
    s->dev_name[20] = 0;
    ok = n >= SET_LEN_V3 && (s->qa_ok == 1u || s->qa_ok == QA_V2);   /* the quick-access list: each slot a
                                                     * block (the first list: one of its twelve sections) or empty */
    for (k = 0; ok && k < QA_SLOTS; k++)
        ok = s->qa[k] < (s->qa_ok == 1u ? 12u : QA_NSEC) || s->qa[k] == QA_EMPTY;
    if (!ok)
        qa_defaults(s);
    else if (s->qa_ok == 1u) {           /* (the first list's sections: their blocks now) */
        static const uint8_t OLD[12] = {10, 11, 7, 8, 16, 17, 18, 19, 0, 6, 21, 22};
        for (k = 0; k < QA_SLOTS; k++)
            if (s->qa[k] != QA_EMPTY) s->qa[k] = OLD[s->qa[k]];
        s->qa_ok = QA_V2;
    }
    if (n < SET_LEN_V4 || s->glo_ok != GLO_V1 || s->tr < -TR_MAX || s->tr > TR_MAX || s->midi_ch > 16u
        || s->thru > 1u || s->clk >= CLK_N || s->ble_auto > 1u)
        glo_defaults(s);
    if (n < SET_LEN_V4K || s->key_dark > 2u) s->key_dark = 0;   /* (KL_ON, KL_OFF; KL_NIGHT 1.18C43) */
    if (n < SET_LEN_V4K || s->rv_q >= RVQ_N) s->rv_q = RVQ_DEF;
    s->sq_def = (uint8_t)(n < SET_LEN_V4K ? (DEF47 ? 0u : SQ_DEF0) : s->sq_def & (SQ_PADM | SQ_PCM | SQ_DOUT));   /* (GLO > SEQ,
                                          * 1.14: an older object as shipped; 1.18C40: PRECOUNT 0 .. 2) */
    if ((s->sq_def & SQ_PCM) == SQ_PCM) s->sq_def &= (uint8_t)~SQ_PCM;
#ifdef HYB_MULTI
    if (n < SET_LEN_V4K || s->tempo > 128u) s->tempo = 0;   /* (1.18: none saved) */
#endif
    ok = n >= SET_LEN_V5 && s->pm_ok == PM_V1;
    for (k = 0; ok && k < 7u; k++)
        ok = s->pm[k] <= PM_SMART;
    if (!ok)
        pm_defaults(s);
    if (n >= SET_LEN_V6 && s->pt_ok == PT_V1) {   /* (1.18C29 and before: three user banks, the init patch 640; the
                                          * slots keep their numbers, the init patch moves: PT_V2) */
        if (s->patch == OLD_PATCH_INIT) s->patch = (uint16_t)PATCH_INIT;
        else if (s->patch > OLD_PATCH_INIT) s->patch = (uint16_t)PATCH_N;   /* (none such: as a broken one, below) */
#ifdef HYB_MULTI
        if (n >= SET_LEN_V11 && s->mx_patch == OLD_PATCH_INIT) s->mx_patch = (uint16_t)PATCH_INIT;
        else if (n >= SET_LEN_V11 && s->mx_patch > OLD_PATCH_INIT) s->mx_patch = (uint16_t)PATCH_N;
#endif
        s->pt_ok = PT_V2;
    }
    if (n < SET_LEN_V6 || s->pt_ok != PT_V2 || s->patch >= PATCH_N) {   /* (a release before the user banks) */
        s->patch = (uint16_t)patch_of_old(s->patch8);
        s->pt_ok = PT_V2;
    }
    s->sq_pad &= SQ_DISARM | DR_DISARM;  /* (PATTERN NONE; an older object: its padding, 0) */
    if (n < SET_LEN_V7 || s->sq_ok != SQ_V1 || s->sq_pat >= SQ_PATS) {
        s->sq_ok = SQ_V1;
        s->sq_pat = 0;
        s->sq_pad = 0;
    }
    if (n < SET_LEN_V8 || s->dr_ok != DR_V1 || s->dr_pat >= DR_PATS || s->dr_lev > DR_LMAX) {
        s->dr_ok = DR_V1;
        s->dr_pat = 0;
        s->dr_lev = DR_LEV1;
    }
    if (n < SET_LEN_V9 || s->vo_ok != VO_V1 || s->vol > 127u || s->vol_cc > 1u) {
        s->vo_ok = VO_V1;
        s->vol = 127u;
        s->vol_cc = 1u;
    }
    if (n < SET_LEN_V10 || s->md_ok != MD_V1 || s->out_ch > 15u || s->out_to >= OP_N || s->in_off > 7u
        || s->clk_src >= CS_N || (s->sq_base && (s->sq_base < SQ_BASE_LO || s->sq_base > SQ_BASE_HI)))
        md_defaults(s);
    if (s->eng >= EN_N) s->eng = EN_ALL;
#if FM1B_BLE && !defined(HYB_MULTI)
    if ((s->md_pad >> MD_UNIM_SH & 7u) > 5u) s->md_pad &= (uint8_t)MD_AUTO;   /* (the unison settings OFF) */
#endif
#ifdef HYB_MULTI
    if (n < SET_LEN_V11 || s->mx_ok != MX_V1 || s->mx_mode >= SY_M_N || s->mx_split > 127u || s->mx_chb > 16u
        || s->mx_nva < 1u || s->mx_nva >= SY_MAX_VOICES || s->mx_send > MX_SEND_PATCH || s->mx_patch >= PATCH_N
        || s->mx_dyn > 1u || s->mx_maxv < 4u || s->mx_maxv > SY_MAX_VOICES || s->mx_uni > 3u || s->mx_unim > 5u
        || s->mx_uauto > 1u || s->mx_steal > 1u)
        mx_defaults(s);
    else if (n < SET_LEN_V16 || (s->mx_unx & MX_UNX_L) > 5u || s->mx_unx > 31u)
        s->mx_unx = 0;                   /* (1.18C48: an older object's: UNI MAX MULTI as shipped, the others as stored) */
    if (s->mx_uni != 3u) s->mx_unx &= (uint8_t)~MX_UNX_S4;
    if (s->mx_unim != 5u) s->mx_unx &= (uint8_t)~MX_UNX_M6;
    sqb_check(s, n);
    if (n < SET_LEN_V12 || s->mx_vcc[0] > PV_CC_OFF || s->mx_vcc[1] > PV_CC_OFF) s->mx_vcc[0] = s->mx_vcc[1] = 0;
    ln_check(s, n);
    if (n < SET_LEN_V14 || s->osc_ok != OSC_V1 || s->osc_cl > 1u) {   /* (an older object: ORIGINAL) */
        s->osc_ok = OSC_V1;
        s->osc_cl = 0;
    }
    if (n < SET_LEN_V14 || s->flt_sm > 1u) s->flt_sm = 0;   /* (FILT KNOBS: an older object's, ORIGINAL) */
    s->ck_fl = n < SET_LEN_V14 ? 0u : s->ck_fl & CK_FL_UNLOCK;   /* (1.19.5: CLK LOCK; an older object's ON) */
    ve_check(s, n);
#endif
    return 1;
}

/* the quick-access list as it is now, saved at once into the settings object as saved (what SAVE keeps, the octave
 * or the patch, stays as it was saved: dev_remember's way) */
static int qa_save(void)
{
    settings_t s;
    uint32_t k;
    settings_load(&s);
    s.qa_ok = QA_V2;
    for (k = 0; k < QA_SLOTS; k++)
        s.qa[k] = set.qa[k];
    return store_save(OBJ_SETTINGS, &s, sizeof s);
}

#define VOICES_MED 14
#define VOICES_HIGH 16
#ifdef HYB_MULTI
static int lat_cap(void) { return set.lat == LAT_HIGH ? VOICES_HIGH : set.lat == LAT_MED ? VOICES_MED : VOICES_STD; }
static int voice_cap(void) { return set.mx_maxv && set.mx_maxv < lat_cap() ? set.mx_maxv : lat_cap(); }   /* (MAX VOICES) */
#else
static int voice_cap(void) { return set.lat == LAT_HIGH ? VOICES_HIGH : set.lat == LAT_MED ? VOICES_MED : VOICES_STD; }
#endif
static int lat_fifo(void) { return set.lat == LAT_HIGH ? 2 : set.lat == LAT_MED ? 1 : 0; }   /* the platform's blocks */

static UI_LATE void kit_load(void);
static UI_LATE void kit_bus_text(uint32_t c, char *v);
void app_init(void)
{
#if defined(HYB_RT_SPARE) && !defined(PLAT_HOST)
    {   /* (.rt_spare, NOLOAD: the boot clears .bss and the pool, not it) */
        extern uint32_t _rts_start[], _rts_end[];
        uint32_t *w;
        for (w = _rts_start; w < _rts_end; w++) *w = 0u;
    }
#endif
#if HYB_HEALTH_LOG
    hl_paint();                          /* (the stacks marked for the debug log, before the interrupts) */
#endif
#ifdef HYB_MULTI
    sy_multi_init(&M, &S, &SB, &SH, CH_A, CH_B, SAT_B, 44100.f);   /* (both parts; SINGLE until mx_apply) */
#else
    sy_init_hz(&S, 44100u);
#endif
    { extern const unsigned sy_synth_size; if (sy_synth_size != (unsigned)sizeof S) app_say("BUILD", "VOICES MISMATCH"); }
    S.channel = -1;                      /* any MIDI channel */
    sy_prof_clock = plat_ticks24;
    if (!HYB_ONE_CORE && plat_cpu2_set(1) == 0)   /* the second core: half the voices (only here, at power-on) */
        sy_set_cpu2(plat_cpu2_run, plat_cpu2_wait);
    settings_load(&set);
#ifdef HYB_MULTI
    pv_vol[0] = (uint8_t)pv_saved(0);    /* (1.19.7: VOL A / B as saved) */
    pv_vol[1] = (uint8_t)pv_saved(1);
#endif
#ifdef HYB_TEST_BT_OFF
    set.ble_off = 1;                     /* (a test build: Bluetooth off at power-on, as if saved so: the emulator) */
#endif
#ifdef HYB_TEST_LAT_HIGH
    set.lat = LAT_HIGH;                  /* (a test build: GLO > AUDIO > LATENCY HIGH, its voices: the emulator's load) */
#endif
#ifdef HYB_TEST_DLEV
    set.dr_lev = HYB_TEST_DLEV;          /* (1.18C42, a test build: the drums' LEVEL, above 100 the drive: the emulator) */
#endif
#if defined(HYB_TEST_OSC_CLEAN) && defined(HYB_MULTI)
    set.osc_cl = 1;                      /* (a test build: GLO > AUDIO > OSC CLEAN: the emulator's load) */
#endif
#ifdef HYB_RV_LEND
    bt_lend_init();                      /* (Bluetooth off: its memory to REVERB FULL, before the first block) */
#endif
    rvq_apply();                         /* (GLO > AUDIO > REVERB, before the first block) */
    S.opt |= SY_OPT_FXPIPE;              /* the effects on the second core a block later, the notes a block earlier (the
                                          * reference's two blocks; SY_OPT_LATENCY: the engine's default) */
    S.max_voices = voice_cap();
#ifdef HYB_MULTI
    M.max_voices = (int16_t)voice_cap();
#endif
#if FM1B_BLE && !defined(HYB_MULTI)
    uni_apply();                         /* (AUDIO's unison settings) */
#endif
#ifdef HYB_MULTI
    osc_apply();                         /* (AUDIO's OSC) */
#endif
#ifdef HYB_VELO
    ve_apply();                          /* (GLO > VELOCITY: the keys' sensing) */
#endif
    plat_audio_fifo(lat_fifo());         /* MED, HIGH: the platform renders one or two blocks ahead */
    ub_scan();
    fv_load();                           /* (1.18C33: PRESET's favorites) */
#ifdef HYB_MULTI
    sq_mi = 1;                           /* (SEQ B's pattern) */
    sq_load(set.sqb_pat);
    sq_mi = 0;
#endif
    sq_load(set.sq_pat);                 /* (the sequencer's pattern; it starts off) */
    dr_load(set.dr_pat);                 /* (the drum sequencer's; it starts off) */
    kit_load();                          /* (the drum kit) */
    dr_init();
    if (!patch_filled(set.patch)) {      /* (a slot emptied since, or a factory slot: the first filled user slot, else
                                          * the init patch) */
        uint32_t q = PU_0;
        while (q < PATCH_INIT && !patch_filled(q)) q++;
        set.patch = (uint16_t)q;
    }
    uw_scan();                           /* the user waves (waves.c), before the first wave table is built */
    sy_user_wave = uw_get;
    {
        const uint8_t *b = patch_bytes(set.patch);
        uint32_t k;
        for (k = 0; k < 256u; k++)
            shadow[k] = b[k];
        sy_load_patch(&S, shadow);       /* (before the audio starts) */
#ifdef HYB_SHIMMER
        shx_put(patch_ext(set.patch), shadow);
#endif
    }
#ifdef HYB_MULTI
    {   /* part B: its patch (an emptied slot: the init patch), the setup */
        const uint8_t *b;
        uint32_t k;
        patch_o = (uint16_t)(patch_filled(set.mx_patch) ? set.mx_patch : PATCH_INIT);
        b = patch_bytes(patch_o);
        for (k = 0; k < 256u; k++)
            shadow_o[k] = b[k];
        sy_load_patch(&SB, shadow_o);
#ifdef HYB_SHIMMER
        shx_put(patch_ext(patch_o), shadow_o);
#endif
        mx_apply();
    }
#endif
    SHX_SYNC();                          /* (part A's shimmer: the first block takes it) */
#ifdef HYB_MULTI
    if (!set.tempo) set.tempo = (uint8_t)(shadow[TP_BYTE] + 1u);   /* (1.18: none saved yet: part A's patch's) */
    shadow[TP_BYTE] = shadow_o[TP_BYTE] = (uint8_t)(set.tempo - 1u);   /* (the global tempo, both parts) */
    tp_set(set.tempo - 1u);
#else
    tp_set(S.patch[TP_BYTE]);            /* (the tempo: part A's patch's) */
#endif
}

/* -------------------------------------------------------------------- UI --- */
static struct {
    char msg[2][24];                     /* a message on the LCD for a moment (Bluetooth, saved...) */
    uint32_t msg_until;
    uint32_t keys, btn;
    uint8_t held[NKEYS];                 /* the note each held key started */
    uint32_t sec;                        /* the row K1..K4 act on (U_ROW), when a row is lit (row_on) */
    int32_t last;                        /* the parameter turned last: its column in the lit row, -1 none */
    const struct u_param_s *lastp;       /* the parameter turned last (the LCD strip), 0 none */
    uint32_t edit_t;                     /* when a parameter was turned last (the LCD shows it for LCD_EDIT_MS) */
    uint32_t alt_t;                      /* the LCD's alternation of the patch and the module starts here */
    uint8_t looking;                     /* Bluetooth MIDI devices: the scan's list up (lib/btdev) */
    uint32_t rec_t;                      /* REC went down (0: not held, or its hold already acted) */
    uint8_t d_lcd, d_tabs, d_rows, d_col;   /* what to draw again (d_col: the lit row's turned column) */
    uint8_t view;                        /* V_*: HOME and the quick-access list, the list's slots, a menu, its EDIT
                                          * groups, a group */
    uint8_t qpos;                        /* V_QUICK: 0 HOME, 1 .. n the list's blocks, n + 1 EDIT */
    uint8_t menu, pos, gcur, grp;        /* the menu (U_MENU); the place in its blocks or its group's rows (rl_*); the
                                          * cursor in its EDIT list; the group (U_GRP) */
    uint8_t mpos[7];                     /* the place each menu was left at (U_NMENU) */
    const struct u_param_s *vl_p;        /* the parameter whose list of values is up (until vl_until) */
    uint32_t vl_until;
    uint8_t shift_used;                  /* OCT- held as SHIFT has been used (its release then no octave) */
    uint8_t oc_pair;                     /* OCT- and OCT+ pressed together (quick_panic): 1 the pair, 2 its PANIC done;
                                          * 0 once both are up */
    uint8_t up_used;                     /* OCT+ was part of the pair (its release then no octave) */
    uint32_t shift_t;                    /* OCT- went down */
    uint32_t up_t;                       /* OCT+ went down */
    uint32_t plist_until;                /* the patch list over the rows until then (PRESET turned) */
    uint8_t plist_new;                   /* the patch list shown by a first detent, no patch loaded yet (0.91: the
                                          * browse's undo step at its first load) */
    uint8_t gpage, gpos, gch;            /* GLO (V_GLO): its page (GP_*), the cursor, the value being chosen (G_*, or the
                                          * device list; 0 none) */
    int8_t gold;                         /* the value before its list opened (HOME: back to it) */
    uint8_t glook;                       /* GLO's device list is up: Bluetooth MIDI devices looked for (ui.looking) */
    uint8_t ps_b;                        /* (0.96) the bank PRESET browses on INIT (0 A .. 4 E): the one INIT was
                                          * reached from (pb_*) */
    uint8_t rp_ask;                      /* (0.96) "RESET PATCH ?" asked: INIT reached with the patch edited (rp_*) */
    uint16_t rp_park;                    /* (after 0.98) RESET PATCH's patch left (EM_P + 1; 0 none): its edits kept
                                          * for UNDO alone, forgotten when the list loads it again (rp_rec, ps_go) */
    uint32_t glo_t;                      /* a GLO setting changed then: saved GLO_SAVE_MS later (0: nothing to save) */
    uint8_t pcur, pch;                   /* PLAY mode's assignment editor (V_PLAYED): the cursor (a control, PC_N: BACK);
                                          * choosing its parameter (the list of all) */
    uint16_t psel;                       /* the parameter chosen meanwhile: 0 DEFAULT, 1 SMART, 2 + the list's (pl_*) */
    uint8_t smart;                       /* the smart choice's byte now (play_smart): 18 PW, 27 FM amount, 28 sync */
    uint16_t ppatch;                     /* the patch it was resolved for */
    uint16_t scur;                       /* SAVE (V_SAVE): the user slot under the cursor (0 .. UB_N - 1) */
    uint8_t sfrom;                       /* the view SAVE was pressed in (HOME goes back to it) */
    uint8_t ncur, nask;                  /* the name editor (V_NAME): the cursor (0 .. 9); REC asked once (a slot
                                          * filled: REC again overwrites it) */
    char nm[10];                         /* the name being edited */
    uint8_t ls_end;                      /* (1.18C35) this turn of a list reached a section's end: a line a detent until
                                          * a pause (ls_acc) */
    uint16_t ps_cur, ls_fly;             /* (1.18C35) a fast turn of a list (ls_acc): the patch list's cursor (patch
                                          * + 1; 0: the patch up's place), its last detent (ms | 1; 0: none); the
                                          * patch there loaded (the favorites editor's heard) once it settles, fv_idle */
    uint32_t ls_t;                       /* (1.18C35) a list's last detent (ls_acc) */
} ui;
#define PLIST_MS 3000u                   /* the patch list's time after PRESET's last turn */
#define SHIFT_HOLD_MS 500u               /* OCT- held this long: SHIFT (released sooner, nothing else pressed: the
                                          * octave down) */
#define OC_PAIR_MS 300u                  /* OCT- and OCT+: the second within this of the first is the pair (quick_panic) */
#define OC_HOLD_MS 300u                  /* the pair held this long (both down): PANIC */
enum { V_QUICK, V_LIST, V_MENU, V_GLIST, V_GROUP, V_GLO, V_PLAY, V_PLAYED, V_SAVE, V_NAME, V_SEQ, V_DRUM, V_KIT, V_LNCH };   /* (V_QUICK
                                          * at power-on: HOME; V_PLAY PLAY mode, V_PLAYED its assignment editor; V_SAVE
                                          * SAVE's slot list, V_NAME its name editor; V_SEQ the sequencer's page, V_DRUM
                                          * the drum sequencer's; V_KIT GLO's drum kit; V_LNCH the pattern launcher,
                                          * the two-part builds, ln_*) */
#define GLO_SAVE_MS 2000u                /* GLO's settings saved this long after the last change */
#define LCD_ALT_MS 3000u                 /* the patch and the module in turn, each this long */
#define LCD_EDIT_MS 10000u               /* a parameter turned: its name and value this long */

static void text_cat(char *d, const char *s)
{
    while (*d)
        d++;
    while (*s)
        *d++ = *s++;
    *d = 0;
}

static void put_num(char *p, uint32_t v, int w)   /* right-aligned decimal, w places, zero-padded */
{
    int i;
    for (i = w - 1; i >= 0; i--) {
        p[i] = (char)('0' + v % 10u);
        v /= 10u;
    }
}

#if !defined(HYB_NO_RAND) && (!FM1B_BLE || defined(HYB_MULTI))
#define HYB_RAND                         /* (after 0.98) PRESET's RANDOMIZE (app.c rz_*): the USB build and (1.13) the
                                          * Bluetooth two-part build (~0.9 KB of flash; the Bluetooth single build,
                                          * kept at 1.12, has not that much left); HYB_NO_RAND: none */
#define RZ1 1u                           /* (the patch list's places: RANDOMIZE first) */
#else
#define RZ1 0u
#endif
#include "ui.c"

void app_say(const char *a, const char *b)
{
    uint32_t i;
    char t[48];
#ifdef HYB_MULTI
    ln_vq = 0;                           /* (a message of another kind: the strip's own colours) */
#endif
    for (i = 0; i < 23u && a && a[i]; i++)
        ui.msg[0][i] = a[i];
    ui.msg[0][i] = 0;
    for (i = 0; i < 23u && b && b[i]; i++)
        ui.msg[1][i] = b[i];
    ui.msg[1][i] = 0;
    ui.msg_until = plat_ms() + 1300u;
    ui.d_lcd = 1;
    if (ann_on()) {
        t[0] = 0;
        text_cat(t, ui.msg[0]);
        if (ui.msg[1][0]) {
            text_cat(t, " ");
            text_cat(t, ui.msg[1]);
        }
        ann_say(t, 0, ANN_NOW);
    }
}
static UI_LATE void say_on(const char *a, uint32_t on) { app_say(a, on ? "ON" : "OFF"); }   /* "<a>  ON" / "OFF" */
#ifdef HYB_MULTI
static int text_eq(const char *a, const char *b);
#define MH_MS 1500u                      /* the muted-part hint stays this long after the last key or start */
static uint8_t mh_ext PLAT_POOL;         /* main loop: a MIDI START (CLOCK EXTERNAL) is being run: no hint for it */
static UI_LATE void mute_hint(uint32_t to)   /* a local key or a local start reaches the parts "to" (bits: SYNTH 1, SYNTH 2,
                                          * the drums): those of them the launcher muted are named in the strip ("A MUTED",
                                          * "A+DR MUTED"), in the first one's colour, MH_MS after the last such action. The
                                          * strip is drawn on a change only: the same hint on screen just has its time
                                          * renewed. Main loop only (a key's or a transport's handler, never the audio) */
{
    static const char *const nm[3] = {"A", "B", "DR"};
    const uint32_t m = to & ln_mute & 7u, now = plat_ms();
    uint32_t q;
    char t[24];
    if (!m) return;
    t[0] = 0;
    for (q = 0; q < 3u; q++)
        if (m >> q & 1u) {
            if (t[0]) text_cat(t, "+");
            text_cat(t, m == 4u ? "DRUMS" : q == 0u && SQ_BMO ? "SYNTH" : nm[q]);
        }
    text_cat(t, " MUTED");
    if (now < ui.msg_until && ui.msg[1][0] == 0 && text_eq(ui.msg[0], t)) {
        ui.msg_until = now + MH_MS;      /* (shown already: only its time) */
        return;
    }
    app_say(t, "");
    ui.msg_until = now + MH_MS;
    for (q = 0; !(m >> q & 1u); q++)
        ;
    ln_vq = (uint8_t)(q + 1u);           /* (its part's colour: A red, B green, the drums amber) */
}
static UI_LATE void mute_hint_run(void)  /* app_frame, after input and the launcher: a sequencer that began to run (PLAY, a
                                          * page, the launcher; not a MIDI START) with its part muted: the hint */
{
    static uint8_t last PLAT_POOL;
    const uint32_t run = (sqr[0].run ? 1u : 0u) | (sqr[1].run ? 2u : 0u) | (dr.run ? 4u : 0u), up = run & ~(uint32_t)last;
    last = (uint8_t)run;
    if (mh_ext) { mh_ext = 0; return; }
    if (up) mute_hint(up);
}
#else
#define mute_hint(to) ((void)0)
#endif

static void patch_tag(char *s, uint32_t p)   /* "C012": patch p's bank and slot; "INIT" (4 characters and a 0) */
{
    if (p >= PATCH_INIT) { s[0] = 'I'; s[1] = 'N'; s[2] = 'I'; s[3] = 'T'; }   /* (the init patch) */
    else { s[0] = (char)('A' + (p - PU_0) / 128u); put_num(s + 1, p % 128u + 1u, 3); }   /* (the user banks: A .. F) */
    s[4] = 0;
}
static void name_label(char *s, uint32_t p, const char *nm)   /* "A012 NAME": patch p, its name nm (10 bytes) */
{
    uint32_t i, n = 5;
    patch_tag(s, p);
    s[4] = ' ';
    for (i = 0; i < 10u; i++) {
        char c = nm[i];
        s[n++] = c >= 32 && c < 127 ? c : ' ';
    }
    while (n > 5 && s[n - 1] == ' ')
        n--;
    s[n] = 0;
}
static void patch_label(char *s, uint32_t p, const uint8_t *b)   /* (its name from b: bytes 240..249; the edit
                                                     * buffer holding a SysEx single: "SysEx NAME") */
{
    name_label(s, p, (const char *)b + 240);
#ifdef HYB_MULTI
    if ((sx_edit == 1u && b == shadow) || (sx_o == 1u && b == shadow_o)) {   /* (the other part's too: HOME's rows;
                                          * 2: a RANDOMIZE roll, its own name) */
#else
    if (sx_edit == 1u && b == shadow) {
#endif
        uint32_t i = 0;
        char t[24];
        while ((t[i] = s[i]) != 0) i++;
        s[0] = 0;
        text_cat(s, "SysEx");
        text_cat(s, t + 4);
    }
}
#ifdef HYB_MULTI
static void patch_line(char *s)          /* the patch up (two parts: "B:" and the part edited's) */
{
    if (set.mx_mode == SY_M_SINGLE) { patch_label(s, set.patch, shadow); return; }
    s[0] = ed_part ? 'B' : 'A';
    s[1] = ':';
    patch_label(s + 2, set.patch, shadow);
}
#else
static void patch_line(char *s) { patch_label(s, set.patch, shadow); }   /* the patch up */
#endif

/* PRESET's order (0.96): within the bank of the patch up, INIT, then its filled slots (001 .. 128, the empty ones left
 * out); on INIT the bank it was reached from (ui.ps_b). Up to 0.95 the order ran over all the banks (INIT, A001 ..
 * E128): PRESET past a bank's last patch went on into the next bank's first. The bank changes with OCT- / OCT+ and the
 * list up (ps_bank) alone */
static UI_LATE uint32_t pb_b0(void)      /* the bank's first slot */
{
    return (set.patch < PATCH_INIT ? set.patch / PA_N : ui.ps_b < PU_0 / PA_N ? PU_0 / PA_N : ui.ps_b) * PA_N;
}
static UI_LATE uint32_t pb_n(void)       /* the places: INIT and the bank's filled slots */
{
    const uint32_t b0 = pb_b0();
    uint32_t q, n = 1;
    for (q = b0; q < b0 + PA_N; q++)
        n += (uint32_t)patch_filled(q);
    return n;
}
static UI_LATE uint32_t pb_pos(uint32_t p)   /* patch p's place (INIT 0) */
{
    uint32_t q, n = 1;
    if (p >= PATCH_INIT) return 0;
    for (q = p / PA_N * PA_N; q < p; q++)
        n += (uint32_t)patch_filled(q);
    return n;
}
static UI_LATE uint32_t pb_patch(uint32_t pos)   /* the patch at place pos */
{
    const uint32_t b0 = pb_b0();
    uint32_t q;
    for (q = b0; pos && q < b0 + PA_N; q++)
        if (patch_filled(q) && !--pos) return q;
    return PATCH_INIT;
}
/* the patch list (PRESET turned): the patch up in the middle (INIT, then the bank's patches in PRESET's order, pb_*);
 * a patch with edits not saved has a dot (ui.c ls_*) */
static UI_LATE uint32_t fe_patch(uint32_t i);
static UI_LATE int fv_find(uint32_t p);
static void pl_item(uint32_t i, char *t, char *pre, uint32_t *fl)   /* (1.18C33: the favorites' section over RANDOMIZE:
                                          * EDIT FAVORITES, favorite n .. 1 numbered; the favorites editor: its favorites,
                                          * then the bank's patches as stored, fe_*) */
{
    uint32_t p;
    if (fv.ed) {
        p = fe_patch(i);
        if (i < fv.n) put_sint(pre, (int32_t)(fv.n - i), 0);
    } else if (!i) {
        text_cat(t, "EDIT FAVORITES");
        return;
    } else if (i <= fv.n) {
        p = fv.p[fv.n - i];
        put_sint(pre, (int32_t)(fv.n + 1u - i), 0);
    }
#ifdef HYB_RAND
    else if (i == PL_RZ) { text_cat(t, "RANDOMIZE"); return; }   /* (below INIT, after 0.98: rz_go) */
#endif
    else p = pb_patch(i - PL_INIT);
    if (p == set.patch && !fv.ed) patch_label(t, p, shadow);
    else if (p >= PU_0 && p < PATCH_INIT) {   /* (a user patch: its name alone, not the whole patch read) */
        char nm[10];
        ub_name(p - PU_0, nm);
        name_label(t, p, nm);
    } else
        patch_label(t, p, patch_bytes(p));
    if (em_edited(p) && !fv.ed) *fl |= LS_DOT;
    if (fv_find(p) >= 0) *fl |= LS_STAR;
}

/* HOLD (REC, where REC has no other meaning): released keys keep their notes until REC is pressed again (lit while on); a held note's
 * key struck again restarts it */
static uint8_t hold_on;
#ifdef HYB_MULTI
static uint8_t hold_o;                   /* (1.00) the other part's HOLD (part_edit swaps it in): each part keeps its
                                          * own while it is not edited; not saved (a power-on: both off). (1.16) LAYER:
                                          * one HOLD for both parts, hold_o kept equal to hold_on (hold_flip, MODE) and
                                          * not swapped */
#endif
static uint32_t hold_bits[4];            /* the notes HOLD keeps, a bit a note (hold_note) */
#define hold_note(n) (hold_bits[(n) >> 5 & 3u] >> ((n) & 31u) & 1u)
#define hold_set(n) (hold_bits[(n) >> 5 & 3u] |= 1u << ((n) & 31u))
#define hold_clr(n) (hold_bits[(n) >> 5 & 3u] &= ~(1u << ((n) & 31u)))
#ifdef HYB_MULTI
static uint32_t hold_bo[4] PLAT_POOL;    /* (1.15) CHANNELS: the notes the other part's HOLD keeps sounding there while
                                          * the part edited plays the keys (part_edit swaps them with hold_bits); empty
                                          * in the other modes */
#define hold_po(n) (hold_bo[(n) >> 5 & 3u] >> ((n) & 31u) & 1u)
#define HOLD_PO(n) hold_po(n)
#else
#define HOLD_PO(n) 0u
#endif
#define KEY_NONE 0xFFu                   /* ui.held: the key played no note (a SHIFT function) */
#ifdef HYB_MULTI
static uint32_t key_zb[4];               /* SPLIT: the notes a key below the split started (part B's zone), a bit a
                                          * note (key_zone) */
static UI_LATE void key_zone(uint32_t k, uint32_t note)   /* key k starts note: its zone kept with the note. The panel's
                                          * keys split by the key itself (its note at OCTAVE 0 and TRANSPOSE 0: F3 +
                                          * k), not by the note: OCTAVE and TRANSPOSE move both zones' pitch, the split
                                          * stays at its key. A note held (HOLD, the sequencers' input, the engine's
                                          * arpeggiator) keeps the zone of the key that started it; MIDI in splits by
                                          * the note number (sq_parts_in, the engine's multi_route) */
{
    if (53u + k < set.mx_split) key_zb[note >> 5 & 3u] |= 1u << (note & 31u);
    else key_zb[note >> 5 & 3u] &= ~(1u << (note & 31u));
}
static UI_LATE uint32_t sq_parts_key(uint32_t note)   /* the parts a key's note goes to (1 A, 2 B, 3 both): SINGLE
                                          * A, CHANNELS the part edited (0.67; red A, green B: part_edit releases the
                                          * keys' notes first), LAYER both, SPLIT by the zone of the key that started
                                          * it (key_zone) */
{
    if (set.mx_mode == SY_M_LAYER) return 3u;
    if (set.mx_mode == SY_M_CHANNELS) return ed_part ? 2u : 1u;
    if (SQ_BMO) return sqr[1].run ? 3u : 1u;   /* (1.19.3: SEQ B running in SINGLE takes the keys as its input, as in
                                          * LAYER; part A plays them; SEQ A, if it runs, takes them too) */
    return set.mx_mode == SY_M_SPLIT && (key_zb[note >> 5 & 3u] >> (note & 31u) & 1u) ? 2u : 1u;
}
static UI_LATE void key_send(uint32_t st, uint32_t note, uint32_t v);
static UI_LATE void key_out(uint32_t st, uint32_t note, uint32_t v)   /* a key's note on (0x90) or off (0x80): to its
                                          * parts, those whose sequencer runs leave it out (its input there, sq_keys_put);
                                          * none runs: as before (the setup routes it), out over MIDI too */
{
    const uint32_t to = sq_parts_key(note), run = (sqr[0].run ? 1u : 0u) | (sqr[1].run ? 2u : 0u), d = to & ~run;
    if (fv_aud) {                        /* (1.18C33: the favorites editor: the keys play its audition, part A) */
        ev_put(EV_KEY_A, note, st == 0x80u ? 0u : v);
        key_send(st, note, v);
        return;
    }
    if (run && !d) return;               /* (only sequencers' input) */
    if (!run && set.mx_mode != SY_M_SPLIT && to != 2u) ev_put(st, note, v);   /* (SPLIT, CHANNELS' part B: to the
                                          * key's part itself, below) */
    else ev_put(d & 1u ? EV_KEY_A : EV_KEY_B, note, st == 0x80u ? 0u : v);   /* (the part not sequenced alone) */
    key_send(st, note, v);
}
static UI_LATE void key_send(uint32_t st, uint32_t note, uint32_t v)   /* a key's note out over MIDI on its part's
                                          * OUT (1.19.4): part B's alone (SPLIT's B zone, CHANNELS with part B edited)
                                          * on B's, the rest (LAYER's both too: sent once) on A's */
{
    midi_send_p(sq_parts_key(note) == 2u, st, note, v);
}
static UI_LATE void note_off_out(uint32_t note) { key_out(0x80u, note, 0); }   /* (its parts' sequencers run: it did not sound
                                          * there, nothing to end) */
#else
static void note_off_out(uint32_t note)   /* (the sequencer runs: the keys' notes did not sound, nothing to end) */
{
    if (sq.run && !fv_aud) return;
    ev_put(0x80u, note, 0);
    midi_send_ch(0x80u, note, 0);
}
#endif
/* (after 1.08) HOLD's restart: a held note's key struck again ends the note held, and its new note goes to the engine a
 * block later (key_rs_run), never in the block of that note-off. The engine takes a block's note-ons before its
 * note-offs (the reference's order, synth.c EV_ONS_FIRST): an off with no older voice of its key left (the held note's
 * voices stolen, as a full pool of unison voices does) ended the new note at once, and the key stayed silent while
 * HOLD kept its note (a field report: HOLD on with a patch of decays alone, two parts of unison 7: more and more keys
 * dead). key_rs: the keys waiting (bit k), key_rs_w: the panel's queue (ev_w) after their note-offs */
static uint32_t key_rs, key_rs_w;
static UI_LATE void key_note_on(uint32_t k, uint32_t note, uint32_t v)   /* key k's note on: to the engine, out */
{
#ifdef HYB_MULTI
    key_out(0x90u, note, v);             /* (to the parts whose sequencer does not run) */
#else
    if (!sq.run || fv_aud) {             /* (the sequencer runs: an input note, sq_keys_put; 1.18C33: not while the
                                          * favorites editor auditions) */
        ev_put(0x90u, note, v);
        midi_send_ch(0x90u, note, v);
    }
#endif
    (void)k;
}
static UI_LATE void key_rs_run(void)     /* after input: the restarts waiting, once the audio interrupt took their
                                          * note-offs; a key let go meanwhile: none (its note-off went with the
                                          * restart; HOLD on: its note marked held, harmless, a strike sends an off) */
{
    uint32_t k;
    if (!key_rs || (int32_t)(ev_r - key_rs_w) < 0) return;
    for (k = 0; k < NKEYS; k++)
        if ((key_rs >> k & 1u) && (ui.keys >> k & 1u) && ui.held[k] != KEY_NONE) {   /* (still down, its note) */
            uint32_t v = key_vel(k);
            key_note_on(k, ui.held[k], v < 1u ? 1u : v);
        }
    key_rs = 0;
}
#ifdef HYB_VELO
static UI_LATE void ve_cal_key(uint32_t k);
#endif
static UI_LATE void key_on(uint32_t k)   /* key k struck (input; 0.83: also a sequencer's page with PATTERN NONE,
                                          * seq_none_key): its note on (OCTAVE, TRANSPOSE), held in ui.held */
{
    uint32_t v = key_vel(k), note = (uint32_t)(53 + (int32_t)k + 12 * set.oct + set.tr);   /* (5 .. 127) */
    int rs = 0;
#ifdef HYB_MULTI
    if (ui.view == V_LNCH) { ui.held[k] = KEY_NONE; return; }   /* (the launcher: the part keys and all others, no note; keys held on entry still get their off) */
#endif
    if (v < 1u) v = 1u;
#ifdef HYB_VELO
    ve_cal_key(k);                       /* (GLO > VELOCITY > CALIBRATE open: shown, learned from) */
#endif
    if (note < 128u && hold_note(note) && !fv_aud) {   /* (held by HOLD: struck again, it restarts: its new note a
                                          * block later; 1.18C33: not the favorites editor's audition) */
        hold_clr(note);
        note_off_out(note);
        rs = 1;
    }
    ui.held[k] = (uint8_t)note;
#ifdef HYB_MULTI
    if (note < 128u) {
        key_zone(k, note);
        if (!fv_aud) mute_hint(sq_parts_key(note));   /* (its part muted: the strip says so) */
    }
#endif
    if (rs) {
        key_rs |= 1u << k;
        key_rs_w = ev_w;
    } else
        key_note_on(k, note, v);
}
#ifdef HYB_MULTI
#define HOLD_PART(n) sq_parts_key(n)     /* (the parts a held note sounds on: SPLIT its key's zone's) */
#else
#define HOLD_PART(n) 1u
#endif
#define hold_release() hold_release_m(3u)
static void hold_release_m(uint32_t m)   /* the held notes off, but those whose key is still down; (1.16) m: the
                                          * parts (1 A, 2 B, 3 both) whose notes, SPLIT's zones (the other modes: all) */
{
    uint32_t n, k;
    for (n = 0; n < 128u; n++) {
        if (!hold_note(n) || !(HOLD_PART(n) & m))
            continue;
        hold_clr(n);
        for (k = 0; k < NKEYS; k++)
            if ((ui.keys >> k & 1u) && ui.held[k] == n)
                break;
        if (k == NKEYS)
            note_off_out(n);
    }
}

static void all_notes_off(void)          /* the keys held and the notes HOLD keeps: off (a patch change; while the
                                          * sequencer runs they are its input and stay) */
{
    uint32_t k;
    if (SQ_RUNS) return;
    key_rs = 0;                          /* (HOLD's restarts waiting: none sent; their note-offs went already) */
    for (k = 0; k < NKEYS; k++)
        if ((ui.keys >> k & 1u) && ui.held[k] != KEY_NONE) {
            hold_clr(ui.held[k]);
            note_off_out(ui.held[k]);
        }
    hold_release();
}
#ifdef HYB_MULTI
static UI_LATE void hold_park_end(void)  /* (1.15) the notes the other part's HOLD keeps (CHANNELS, hold_bo): off there
                                          * (its note-offs as its keys' would go: to it, out on its channel) */
{
    uint32_t k;
    ed_part ^= 1u;                       /* (key_out, key_send: the part edited's; main loop alone reads it) */
    for (k = 0; k < 128u; k++)
        if (hold_po(k)) note_off_out(k);
    ed_part ^= 1u;
    hold_bo[0] = hold_bo[1] = hold_bo[2] = hold_bo[3] = 0;
}
static UI_LATE void all_off_both(void) { hold_park_end(); all_notes_off(); }   /* (both parts' keys' and HOLD's) */
static UI_LATE void key_up_off(uint32_t note)   /* (1.16) a key let go, the part edited's HOLD off (input()): SPLIT,
                                          * a key of the other part's zone (key_zb) whose HOLD is on: held there; else
                                          * its note off. (A call as note_off_out's: input() keeps its size) */
{
    if (set.mx_mode == SY_M_SPLIT && hold_o && sq_parts_key(note) == (ed_part ? 1u : 2u)) hold_set(note);
    else note_off_out(note);
}
static UI_LATE void hold_zone_fix(void)  /* (1.16) SPLIT, the part edited's HOLD on, the other's off: input() latched
                                          * every key let go (hold_on); the notes of the other part's zone end here,
                                          * before the sequencers' input is taken (key_rs_run, sq_keys_put) */
{
    uint32_t k, a, x = 0;
    if (set.mx_mode != SY_M_SPLIT || hold_on == hold_o) return;
    a = ed_part ? hold_o : hold_on;      /* (part A's HOLD; B's the other) */
    for (k = 0; k < 4u; k++) x |= hold_bits[k] & (a ? key_zb[k] : ~key_zb[k]);
    if (x) hold_release_m(a ? 2u : 1u);
}
#else
#define all_off_both all_notes_off
#define hold_zone_fix() ((void)0)
#define key_up_off note_off_out
#endif

/* Bluetooth MIDI devices: back to the last one connected (set.dev_*), by itself: once Bluetooth is up after
 * power-on, and whenever the device's link is down (not scanning, not connecting); at once (1.5 s after Bluetooth is
 * up, 1 s after the device was lost), then 2 s after each failed attempt for AR_BUSY_MS, 10 s later on. An attempt pauses the FM-1's
 * advertising up to 5 s (a computer finds it in between). Looking for devices (REC) or letting the device go stops
 * the attempts until the next connection or power-on; a device connected by hand replaces the saved one. Every
 * other attempt looks for the saved name instead (AR_SCAN_MS): a device that comes back under another address (an
 * SMK-25 II after a power cycle) never answers its old one */
#define AR_SCAN_MS 10000u                /* a look for the name */
#define AR_BUSY_MS 180000u               /* after power-on or a loss: the attempts close together (AR_GAP_BUSY) so long */
#define AR_GAP_BUSY 2000u
#define AR_GAP_CALM 10000u
static struct {
    uint8_t stop;                        /* stopped by the player */
    uint32_t scan_t, scan_len;           /* looking for the saved name since then (0: not), how long */
    uint8_t seen;                        /* names logged in this look (the radio log: what advertises) */
    uint32_t since, n;                   /* the series' start (power-on, a loss), its attempts: every third by address,
                                          * the others by the name (a device may advertise its name only a while) */
    uint8_t addr_n;                      /* attempts by address: each takes the next address type (a bonded device
                                          * may answer only under its identity, types 2 / 3: the radio resolves it) */
    uint8_t trying;                      /* an attempt of ours is on (its NO ANSWER not shown) */
    uint8_t was_ready, said, up;
    uint32_t next, fails;                /* the next attempt's time; failed attempts in a row */
} ar;

static int bytes_eq(const uint8_t *a, const uint8_t *b, uint32_t n)
{
    while (n--)
        if (*a++ != *b++)
            return 0;
    return 1;
}

static int text_eq(const char *a, const char *b)
{
    while (*a && *a == *b)
        a++, b++;
    return *a == *b;
}

/* the device READY now becomes the one to reconnect to: saved at once if it is a new one, into the settings object
 * as saved (what SAVE keeps stays as it was saved: a turned octave is not saved by connecting a device) */
static void dev_remember(void)
{
    uint8_t a[6];
    int t = 0, k;
    char nm[21];
    settings_t s;
    if (!plat_ble_device_info(a, &t, nm))
        return;
    nm[20] = 0;
    if (set.dev_ok && set.dev_type == (uint8_t)t && bytes_eq(set.dev_addr, a, 6)
        && (!nm[0] || text_eq(set.dev_name, nm)))
        return;                          /* (the saved one, or by its address: the name kept) */
    if (!nm[0]) {                        /* (by address: the saved name if it is the saved device, else a word) */
        const char *w = set.dev_ok && bytes_eq(set.dev_addr, a, 6) ? set.dev_name : "MIDI device";
        for (k = 0; k < 20 && w[k]; k++)
            nm[k] = w[k];
        nm[k] = 0;
    }
    set.dev_ok = 1;
    set.dev_type = (uint8_t)t;
    for (k = 0; k < 6; k++)
        set.dev_addr[k] = a[k];
    for (k = 0; k < 21; k++)
        set.dev_name[k] = nm[k];
    settings_load(&s);
    s.dev_ok = set.dev_ok;
    s.dev_type = set.dev_type;
    for (k = 0; k < 6; k++)
        s.dev_addr[k] = set.dev_addr[k];
    for (k = 0; k < 21; k++)
        s.dev_name[k] = set.dev_name[k];
    store_save(OBJ_SETTINGS, &s, sizeof s);
}

static void ar_log(const char *a, const char *b)   /* a line on the radio log (fm1_remote.py log) */
{
    char t[48];
    t[0] = 0;
    text_cat(t, a);
    text_cat(t, b);
    plat_log(t);
}
static void auto_reconnect(uint32_t now)   /* every frame */
{
    int d = plat_ble_device();
    if (d == BLE_DEV_READY) {
        if (!ar.was_ready)
            dev_remember();
        ar.was_ready = 1;
        ar.trying = ar.stop = ar.said = 0;
        ar.fails = 0;
        return;
    }
    if (ar.was_ready) {                  /* lost or let go: again soon (unless let go) */
        ar.was_ready = 0;
        ar.said = 0;
        ar.next = now + 1000u;
        ar.since = now;
        ar.n = 0;
    }
    if (ar.trying) {
        if (d != BLE_DEV_OFF)
            return;                      /* (on it) */
        ar.trying = 0;                   /* it did not answer, or its set-up failed */
        ar.fails++;
        ar.next = now + (now - ar.since < AR_BUSY_MS ? AR_GAP_BUSY : AR_GAP_CALM);
    }
    if (plat_ble_state() != BLE_READY || set.ble_off) {
        ar.up = 0;
        return;
    }
    if (!ar.up) {                        /* Bluetooth just up (power-on, switched on): soon */
        ar.up = 1;
        ar.fails = 0;
        ar.n = 0;                        /* (by the name first: a device's address may change across power) */
        ar.since = now;
        ar.next = now;                   /* (at once: a device switched on with the FM-1 advertises only a while) */
    }
    if (ar.scan_t) {                     /* looking for the saved name: connect to it as soon as it shows */
        char nm[21];
        int i;
        if (ui.looking || ar.stop || d == BLE_DEV_CONNECTING || d == BLE_DEV_READY) { ar.scan_t = 0; return; }
        for (i = 0; plat_ble_found(i, nm, 0); i++) {
            nm[20] = 0;
            if (i >= ar.seen && i < 8) {     /* (each name found, once: the first 8) */
                ar.seen = (uint8_t)(i + 1);
                ar_log("dev seen ", nm[0] ? nm : "(no name)");
            }
            if ((set.dev_name[0] && text_eq(nm, set.dev_name)) || text_eq(nm, "(DIRECT)")) {   /* (DIRECT: a device asking
                                                     * for the FM-1 itself, the saved one that bonded) */
                ar_log("dev name match ", nm);
                ar.scan_t = 0;
                if (plat_ble_connect(i) == 0) {
                    ar.trying = 1;
                    if (!ar.said) { ar.said = 1; app_say("RECONNECTING", set.dev_name); }
                } else
                    ar.next = now + 3000u;
                return;
            }
        }
        if ((int32_t)(now - ar.scan_t) >= (int32_t)ar.scan_len) {   /* not seen: the next attempt by address (signed:
                                          * scan_t is now | 1, a pass in the same millisecond, 1.18C46) */
            ar_log("dev name not seen ", set.dev_name);
            ar.scan_t = 0;
            ar.fails++;
            ar.next = now + (now - ar.since < AR_BUSY_MS ? AR_GAP_BUSY : AR_GAP_CALM);
        }
        return;
    }
    if (ar.stop || !set.ble_auto || !set.dev_ok || ui.looking || d != BLE_DEV_OFF || (int32_t)(now - ar.next) < 0)
        return;
    if (ar.n++ % 3u != 2u && set.dev_name[0]) {   /* two of three attempts: by the name (btdev_page scans meanwhile) */
        ar.scan_len = AR_SCAN_MS;
        ar.scan_t = now | 1u;
        ar.seen = 0;
        ar_log("dev auto by name ", set.dev_name);
        return;
    }
    {
        const int t = (int)((set.dev_type + ar.addr_n++) & 3u);
        char w[4] = {'t', (char)('0' + t), 0, 0};
        ar_log("dev auto by address ", w);
        if (plat_ble_connect_addr(set.dev_addr, t) != 0) {
            ar.next = now + 15000u;      /* (refused: not now) */
            return;
        }
    }
    ar.trying = 1;
    if (!ar.said) {                      /* the first of a series on the screen; the rest quiet */
        ar.said = 1;
        app_say("RECONNECTING", set.dev_name);
    }
}

static void ble_device_say(void)
{
    char name[21];
    if (btdev_found(btdev_pick(), name, 0))
        app_say("DEVICE", name);
    else
        app_say("BLE", "SCANNING");
}
static void ble_device_button(void)
{
    int ev;
    if (ar.trying && !ui.looking) {      /* an automatic attempt on: given up, and the scan instead */
        ar.stop = 1;
        plat_ble_disconnect();
        ui.looking = 1;
        app_say("BLE", "SCANNING");
        return;
    }
    if (!btdev_busy() && !ui.looking) {
        ar.stop = 1;                     /* (looking by hand: no automatic attempts until the next connection) */
        ui.looking = 1;
        app_say("BLE", "SCANNING");
        return;
    }
    ev = btdev_save();
    ui.looking = 0;
    if (ev == BTDEV_LET_GO)
        ar.stop = 1;                     /* let go by hand: it stays gone until the next connection */
    app_say(ev == BTDEV_CONNECTING ? btdev_word(ev) : "BLE", ev == BTDEV_CONNECTING ? btdev_name() : btdev_word(ev));
}

/* UNDO (SHIFT = OCT- held, then HOME): the last steps back, newest first. A step is a knob's turn (one parameter,
 * turned without a pause of UNDO_GAP_MS: back to its value before) or a patch change (PRESET turned while the patch
 * list is up counts once: back to the patch before, with its edits) */
static uint32_t home_sig[2];             /* (the HOME screen's parts as drawn: below) */
#define UNDO_N 5u                        /* (the player's choice: five steps are enough, the RAM is short) */
#define UNDO_GAP_MS 1500u
#define UNDO_PATCH 0xFFFFu               /* a step's byte: a patch change (patch: the one before) */
#ifdef HYB_RAND
#define UNDO_RAND 0xFFFEu                /* a step's byte: a RANDOMIZE roll (old: the roll before, 0 the original) */
static UI_LATE void rz_apply(uint32_t seed);
#endif
static struct { uint16_t patch, byte; uint8_t old; } undo[UNDO_N] PLAT_POOL;
static uint32_t undo_n, undo_t;          /* the steps kept (the newest last); the last step's time */
static void undo_push(uint32_t patch, uint32_t byte, uint32_t old)
{
    uint32_t i;
    if (undo_n >= UNDO_N) {              /* (full: the oldest goes) */
        for (i = 0; i + 1u < UNDO_N; i++)
            undo[i] = undo[i + 1u];
        undo_n--;
    }
    undo[undo_n].patch = (uint16_t)patch; undo[undo_n].byte = (uint16_t)byte; undo[undo_n].old = (uint8_t)old;
    undo_n++;
}
static const u_param_t *param_of(uint32_t byte)   /* the panel's parameter on this byte, 0 none */
{
    uint32_t r, k;
    for (r = 0; r < U_NROW; r++)
        for (k = 0; k < 4u; k++) {
            const u_param_t *p = &U_ROW[r].p[k];
            if (p->label[0] && (P_BYTE0(p) == byte || (p->kind == V_WPW && p->byte == byte + 1u)))
                return p;
        }
    return 0;
}
#ifdef HYB_MULTI
static UI_LATE const u_param_t *cm_line(uint32_t c)   /* (1.19.6) the MIDI CC map's line c's parameter, 0: not one
                                          * that can move */
{
    return c < 120u ? (pm_cc_bad(c) ? 0 : param_of(c)) : c >= 128u && c < CM_NT ? param_of(c) : 0;
}
#endif
static void undo_step(void)
{
    char a[24], v[12];
    uint32_t p, b;
    if (!undo_n) {
        app_say("Undo", "NOTHING");
        return;
    }
    undo_n--;
    p = undo[undo_n].patch;
    b = undo[undo_n].byte;
#ifdef HYB_RAND
    if (b == UNDO_RAND) rz_apply(undo[undo_n].old);   /* (the roll before, from the same original) */
    else
#endif
    if (b == UNDO_PATCH || p != set.patch) {   /* (a parameter of a patch not up: that patch first) */
        all_notes_off();
        set.patch = (uint16_t)p;
        load_patch(p);
    }
    a[0] = v[0] = 0;
#ifdef HYB_RAND
    if (b >= 0xFFFEu) {                  /* (UNDO_PATCH, UNDO_RAND) */
#else
    if (b == UNDO_PATCH) {
#endif
        patch_line(a);
    } else {
        const u_param_t *q = param_of(b);
        shadow[b] = undo[undo_n].old;
        em_set(p, b, shadow[b]);
#ifdef HYB_SHIMMER
        if (b >= 256u || shx_ri(b) >= 0) shx_sync();   /* (the extension's, a target's) */
        if (b < 256u)
#endif
        ev_put(EV_PARAM, b, SHX_EB(b, shadow[b]));
        text_cat(a, "UNDO ");
        if (q) { p_name(q, a); value_text(q, shadow[b], v); }
    }
    undo_t = 0;                          /* (the next turn is a new step) */
    ui.plist_until = 0;
    app_say(a, b >= UNDO_PATCH - RZ1 ? "UNDO" : v);
    ui.last = -1;
    ui.edit_t = 0;
    ui.d_rows = ui.d_lcd = 1;
    home_sig[0] = home_sig[1] = 0;
}

static void overlay_off(void)            /* the patch list or a value list away (the view again) */
{
    if (ui.plist_until || ui.vl_until) {
        ui.plist_until = ui.vl_until = 0;
        ui.d_rows = 1;
        home_sig[0] = home_sig[1] = 0;
    }
}

/* the quick-access list (set.qa): SELECT on HOME steps through its blocks, the empty slots left out, then EDIT */
static uint32_t qa_count(void)
{
    uint32_t k, n = 0;
    for (k = 0; k < QA_SLOTS; k++)
        n += set.qa[k] != QA_EMPTY;
    return n;
}
static int32_t qa_nth(uint32_t i)        /* the list's block i (from 0), -1 none */
{
    uint32_t k;
    for (k = 0; k < QA_SLOTS; k++)
        if (set.qa[k] != QA_EMPTY && !i--)
            return set.qa[k];
    return -1;
}
static int qa_on_module(void) { return ui.view == V_QUICK && ui.qpos >= 1u && ui.qpos <= qa_count(); }

#ifdef HYB_MULTI
#define HOME_TAP_MS 400u                 /* HOME on HOME let go sooner (not the profile line): the other part edited */
static uint32_t home_tap PLAT_POOL;      /* HOME pressed on HOME then (| 1), 0 none */
#endif
static struct { int16_t lfo; int16_t x, y; uint8_t on, st; } led[2];   /* a row place's light: lfo -1 none, st 2 unknown */
#ifdef HYB_MULTI
/* the part the panel edits: its patch's bytes, number and SysEx state are shadow, set.patch and
 * sx_edit (so every page, PRESET and SAVE act on it), the other part's are kept in shadow_o, patch_o, sx_o; a switch
 * swaps them (the engine's parts keep their patches). The undo steps go (they are the other part's) */
static void view_reset(void);
static UI_LATE void part_edit(uint32_t p)
{
    uint32_t k;
    if (p && set.mx_mode == SY_M_SINGLE) { app_say("Part B", "MULTI OFF"); return; }
    if (p != ed_part) {
        if (set.mx_mode == SY_M_CHANNELS) {   /* (CHANNELS: the keys play the part edited. 1.15: the old part's
                                          * HOLD on, its held notes go on sounding there (hold_bo) and the keys still
                                          * down stay latched on it, as if let go under HOLD; HOLD off, the keys'
                                          * notes end there (0.67). Either way a key still down then plays nothing
                                          * until struck again: no note hangs) */
            for (k = 0; k < NKEYS; k++)
                if ((ui.keys >> k & 1u) && ui.held[k] != KEY_NONE) {
                    if (hold_on && !(key_rs >> k & 1u)) hold_set(ui.held[k]);   /* (a restart waiting: its note
                                          * ended already) */
                    else {
                        hold_clr(ui.held[k]);
                        note_off_out(ui.held[k]);
                    }
                    ui.held[k] = KEY_NONE;
                }
            key_rs = 0;
            for (k = 0; k < 4u; k++) {   /* (the new part's own held notes back, the old part's kept) */
                const uint32_t t = hold_bits[k];
                hold_bits[k] = hold_bo[k];
                hold_bo[k] = t;
            }
        }
        for (k = 0; k < SH_LEN; k++) { const uint8_t t = shadow[k]; shadow[k] = shadow_o[k]; shadow_o[k] = t; }
        { const uint16_t t = set.patch; set.patch = patch_o; patch_o = t; }
        { const uint8_t t = sx_edit; sx_edit = sx_o; sx_o = t; }
        if (set.mx_mode != SY_M_LAYER) {   /* (1.00) the part's own HOLD back; off there: the notes held let go as
                                          * REC would (the keys still down keep theirs). 1.16: LAYER, one HOLD for both
                                          * parts, nothing changes; SPLIT, each zone's notes stay on its part's HOLD
                                          * (hold_bits both zones', key_zb), none ends */
            const uint8_t t = hold_on;
            hold_on = hold_o;
            hold_o = t;
            if (!hold_on && set.mx_mode != SY_M_SPLIT) hold_release();
        }
        ed_part = (uint8_t)p;
        undo_n = 0;
        ui.plist_until = 0;
        u_theme(p);                      /* (part B: the screen green) */
        view_reset();
        home_sig[0] = home_sig[1] = 0;
        led[0].st = led[1].st = 2u;      /* (the LFOs' lights: drawn again) */
    }
    app_say("Edit", p ? "PART B" : "PART A");
}
static UI_LATE void mx_slot_other(void)  /* SAVE wrote the patch edited into the slot the other part plays (patch_o):
                                          * the other part takes the slot as it is now (its bytes, sound and name), as it
                                          * would after a restart (its patch number is the slot's); its edits of the old
                                          * patch there go. Without this it played and showed the old patch until a
                                          * restart, then the patch saved (the part edited's: both parts the same) */
{
    uint32_t k;
    const uint32_t ep = (uint32_t)patch_o | (uint32_t)!ed_part << 15;   /* (the other part's EM_P) */
    for (k = em_n; k-- > 0;)
        if (EM_PM(em[k].patch) == ep) em_drop(k);
    for (k = 0; k < SH_LEN; k++)
        shadow_o[k] = shadow[k];
    sx_o = 0;
    SHX_SYNC();                          /* (part A's extension, if A is the other part) */
    patch_req = -1;
    RING_PUBLISH();
    for (k = 0; k < 256u; k++)
        pbuf[k] = (uint8_t)SHX_EB(k, shadow_o[k]);
    RING_PUBLISH();
    patch_req = PATCH_REQ_BUF | (ed_part ? 0 : PATCH_REQ_B);   /* (the other part) */
    home_sig[0] = home_sig[1] = 0;
}
static UI_LATE int mx_save_set(void)     /* SAVE's settings: part A's patch in patch, part B's in mx_patch */
{
    settings_t o;
    uint32_t k;
    pv_keep();                           /* (1.19.7: VOL A / B saved with the rest) */
    o = set;
    o.patch = ed_part ? patch_o : set.patch;
    o.patch8 = (uint8_t)old_of_patch(o.patch);
    o.mx_patch = ed_part ? set.patch : patch_o;
    o.tempo = tp_run;                    /* (1.18: the global tempo, GLO's settings saved with the rest) */
    for (k = 0; k < sizeof o.ln; k++) ((uint8_t *)o.ln)[k] = 0;   /* (1.18C27: the launcher's table not kept here) */
    o.ln_ok = 0;
    set.mx_patch = o.mx_patch;
    return store_save(OBJ_SETTINGS, &o, sizeof o);
}
static UI_LATE void mx_apply(void)       /* GLO > MULTI to the engine (taken at its next block; a new mode or VOICES
                                          * stops all its voices) */
{
    M.split = set.mx_split;
    M.ch_b = (int8_t)(set.mx_chb ? (int)set.mx_chb - 1 : -1);
    M.nv_a = set.mx_nva;
    M.send_b = (int16_t)(set.mx_send >= MX_SEND_PATCH ? -1 : (int)set.mx_send);
    M.dyn = set.mx_dyn;
    M.uni_max = (uint8_t)SET_UNI;        /* (1.18C48: SINGLE's cap or the multi modes') */
    M.uni_mono = (uint8_t)SET_UNIM;
    M.uni_auto = set.mx_uauto;
    M.uni_steal = set.mx_steal;
    M.mode = set.mx_mode;
}
#endif
#if FM1B_BLE && !defined(HYB_MULTI)
static UI_LATE void uni_apply(void)      /* the Bluetooth build: GLO > AUDIO's unison settings to the engine (a note
                                          * takes them at its start) */
{
    S.uni_max = (uint8_t)SET_UNI;
    S.uni_mono = (uint8_t)SET_UNIM;
    S.uni_auto = (uint8_t)SET_UAUTO;
    S.uni_steal = (uint8_t)SET_STEAL;
}
#endif
static void view_reset(void)             /* a new place: no parameter turned yet, all drawn again */
{
    ui.last = -1;
    ui.edit_t = 0;
    ui.alt_t = plat_ms() - LCD_ALT_MS;  /* (the block first) */
    ui.d_rows = ui.d_lcd = 1;
}

static void qa_go(uint32_t pos)          /* HOME (0), the list's block pos, or EDIT (past the last) */
{
    uint32_t n = qa_count();
    ui.view = V_QUICK;
    ui.qpos = (uint8_t)(pos > n + 1u ? n + 1u : pos);
    if (qa_on_module())
        ui.sec = (uint32_t)qa_nth(ui.qpos - 1u);
    view_reset();
}

/* the menus (FX, SEL, ENV, LFO, EDIT, ARP): a menu's blocks, SELECT a step (K1..K4 change the lit one, the next
 * below it, dim), then EDIT (REC: the list of its groups) and BACK. A group: its rows the same way, then BACK. The
 * matrix (EDIT) has no blocks: its button opens the list of its six assigns */
enum { RL_NONE = -1, RL_EDIT = -2, RL_BACK = -3 };
static uint32_t rl_n(void)               /* the places of the menu's or the group's list */
{
    const u_menu_t *M = &U_MENU[ui.menu];
    return ui.view == V_GROUP ? U_GRP[ui.grp].nr + 1u : menu_nb(ui.menu, shadow) + (M->ng ? 1u : 0u) + 1u;
}
static int32_t rl_item(uint32_t pos)     /* place pos: a row (U_ROW), EDIT, BACK or none (FX: its blocks by the
                                          * delay / reverb mode, ui.c menu_blk) */
{
    uint32_t nr = ui.view == V_GROUP ? U_GRP[ui.grp].nr : menu_nb(ui.menu, shadow);
    if (pos < nr) return (int32_t)(ui.view == V_GROUP ? group_r0(ui.grp) + pos : menu_blk(ui.menu, pos, shadow));
    if (pos >= rl_n()) return RL_NONE;
    return pos + 1u == rl_n() ? RL_BACK : RL_EDIT;
}
static int32_t row_on(void)              /* the row K1..K4 act on, -1 none */
{
    if (qa_on_module()) return (int32_t)ui.sec;
    if (ui.view == V_MENU || ui.view == V_GROUP) return rl_item(ui.pos) >= 0 ? rl_item(ui.pos) : -1;
    return -1;
}
static void rl_go(uint32_t pos)          /* a place of the menu's or the group's list */
{
    ui.pos = (uint8_t)pos;
    if (ui.view == V_MENU) ui.mpos[ui.menu] = (uint8_t)pos;
    if (rl_item(pos) >= 0) ui.sec = (uint32_t)rl_item(pos);
    view_reset();
}
static void glist_go(uint32_t cur)
{
    ui.view = V_GLIST;
    ui.gcur = (uint8_t)cur;
    view_reset();
}
static void menu_go(uint32_t m)          /* a menu's blocks, at the place last left (no blocks: its groups) */
{
    if (ui.view == V_MENU && ui.menu == m) ui.mpos[m] = 0;   /* (its button again: the first block) */
    ui.menu = (uint8_t)m;
    if (!U_MENU[m].nb) { glist_go(0); return; }
    ui.view = V_MENU;
    rl_go(ui.mpos[m] < rl_n() ? ui.mpos[m] : 0u);
}
static void group_go(uint32_t g, uint32_t pos)
{
    ui.view = V_GROUP;
    ui.grp = (uint8_t)g;
    rl_go(pos);
}
static void menu_up(void)                /* one level up: a group to its menu's list, the list to its EDIT, a menu to
                                          * HOME */
{
    if (ui.view == V_GROUP) glist_go(ui.grp - menu_g0(ui.menu));
    else if (ui.view == V_GLIST && U_MENU[ui.menu].nb) { ui.view = V_MENU; rl_go(rl_n() - 2u); }
    else qa_go(0);
}
static int play_on(void);
static void played_rec(void);
static uint32_t menu_button(uint32_t b)  /* the menu a button opens (SEL: its pages in turn), U_NMENU none */
{
    uint32_t m;
    if (b == B_SEL)
        return ui.view != V_QUICK && ui.view != V_LIST && ui.view != V_GLO && ui.view != V_SAVE && ui.view != V_NAME
               && !play_on() && ui.menu == M_SEL1 ? M_SEL2 : M_SEL1;
    for (m = 0; m < U_NMENU; m++)
        if (U_MENU[m].btn == b) return m;
    return U_NMENU;
}

/* GLO: the global settings, a list of pages: TRANSPOSE, MIDI, BLUETOOTH, AUDIO, KEY LIGHTS, DRUM
 * KIT (a page of its own: V_KIT, kit_*), ABOUT, BACK; MIDI has
 * pages of its own (IN PORTS, OUT, CLOCK). SELECT moves the cursor, REC acts on its line: a page opens (a page's line
 * shows a summary of it), an on / off turns, PANIC and FORGET do it, a value with more choices (TRANSPOSE, CHANNEL,
 * LATENCY, SYNTH VOL, ENGINE, OUT's CHANNEL and PORTS, CLOCK's MODE and SOURCE) opens the list of its values (SELECT or K1
 * chooses, taken at once; REC keeps it, HOME goes back to the value before), SEARCH the list of Bluetooth MIDI devices
 * (REC connects or lets go). BACK or HOME: a level up (to the page's line). Saved by themselves GLO_SAVE_MS after the
 * last change */
enum { G_NONE, G_TR, G_MIDI, G_BT, G_AUDIO, G_BACK, G_CH, G_THRU, G_CLK, G_PANIC, G_BLE, G_SEARCH, G_FORGET, G_AUTO,
       G_LAT, G_CC, G_KEYL, G_ABOUT, G_VER, G_BY, G_KOFI, G_DIAG, G_TRSD, G_VOL, G_VCC, G_INP, G_OUT, G_CLKP, G_IUSB, G_ITRS, G_IBT,
       G_OCH, G_OTO, G_CSRC, G_KIT, G_ENG, G_ASAVE, G_RVQ,
       G_SEQ, G_SPARP, G_SKLAT, G_SRQ, G_SBASE, G_SBASEB,   /* (1.14: GLO > SEQ, the arp patterns' defaults as one is
                                          * cleared or never saved, set.sq_def; the BASE NOTE(s), the page's SELECT's) */
       G_SDOUT,                          /* (1.19.3: GLO > SEQ > DRUM OUT, set.sq_def's SQ_DOUT: its bit's place) */
       G_SPCNT,                          /* (1.18C40: GLO > SEQ > PRECOUNT, both sequencers' RECORD: OFF, 1 BAR, 2 BARS) */
#ifdef HYB_MULTI
       G_MULTI, G_MMODE, G_MEDIT, G_MSPLIT, G_MCHB, G_MVOI, G_MSEND,   /* (the USB build's MULTI page) */
       G_MDYN, G_MAXV, G_UNI, G_UNIL, G_UNIM, G_UAUTO, G_STEAL,   /* (ALLOC; AUDIO's voice settings; 1.18C48: G_UNI UNI
                                          * MAX SINGLE, G_UNIL UNI MAX MULTI) */
       G_MVOA, G_MVOB,                   /* (1.19.7: MULTI's VOL A, VOL B: pv_vol, saved with the settings) */
       G_MVCA, G_MVCB,                   /* (MULTI's VOL A CC, VOL B CC) */
       G_OSC,                            /* (1.18C22: AUDIO's OSC, ORIGINAL / CLEAN) */
       G_FLT,                            /* (after 1.18C34: AUDIO's FILT KNOBS, ORIGINAL / SMOOTH) */
       G_VELO, G_VON, G_VFIX, G_VCRV, G_VSENS, G_VDRUM, G_VCAL,   /* (1.18C26: GLO > VELOCITY, ve_*) */
       G_VK0, G_VK1, G_VK2, G_VK3, G_VK4, G_VK5, G_VK6, G_VNORM, G_VRST, G_VSKIP,   /* (its CALIBRATE page: the top
                                          * row's keys; G_VNORM: CHECK) */
        G_DRCH,                           /* (1.18C31: MIDI's DRUM CHANNEL, set.dr_ch) */
       G_BOUT, G_ACH, G_BCH, G_CLKO,     /* (1.19.4: B CHANNEL > OUT; MIDI's A CHANNEL, B CHANNEL (their pages: IN, G_CH /
                                          * G_MCHB, and OUT, G_OCH / G_BOUT), CLK OFFSET) */
       G_CLKL,                           /* (1.19.5: MIDI's CLK LOCK, ON / OFF) */
       G_CCSHOW,                         /* (MIDI's CC SHOW, ON / OFF: set.cm_show) */
#elif FM1B_BLE
       G_UNI, G_UNIM, G_UAUTO, G_STEAL,  /* (AUDIO's unison settings, in md_pad) */
#endif
       G_N };
enum { GP_TOP, GP_MIDI, GP_BT, GP_AUDIO, GP_ABOUT, GP_IN, GP_OUT, GP_CLK,
#ifdef HYB_MULTI
       GP_MULTI, GP_ACH, GP_BCH,         /* (1.19.4: A CHANNEL's and B CHANNEL's IN and OUT) */
#endif
       GP_SEQ,
#ifdef HYB_VELO
       GP_VELO, GP_VCAL,                 /* (1.18C26: VELOCITY, its CALIBRATE) */
#endif
       GP_N };
_Static_assert(SQ_PARP == 1u << (G_SPARP - G_SPARP) && SQ_NLAT == 1u << (G_SKLAT - G_SPARP)
               && SQ_NRQ == 1u << (G_SRQ - G_SPARP) && SQ_DOUT == 1u << (G_SDOUT - G_SPARP),
               "GLO > SEQ: the pad's bits in their order (DRUM OUT's too)");
#ifdef HYB_MULTI
#define GLO_PMAX 14u                     /* a page's lines at most (MIDI's 14: CC SHOW) */
#elif defined(HYB_VELO)
#define GLO_PMAX 13u                     /* a page's lines at most (1.18C48: AUDIO's 13) */
#else
#define GLO_PMAX 11u
#endif
#if FM1B_BLE && defined(HYB_MULTI)       /* the Bluetooth build with two parts: the Bluetooth build's
                                          * pages, VELOCITY and MULTI after TRANSPOSE and AUDIO's voice settings as the
                                          * USB build's. 1.19.1 (a player's report): VELOCITY under TRANSPOSE, KEY
                                          * LIGHTS and AUTO SAVE above ABOUT; part B's channel (B CHANNEL) under MIDI
                                          * after CHANNEL, no longer on MULTI (1.19.1, 1.19.2: dimmed and fixed in
                                          * SINGLE; 1.19.3: SEQ B's channel there too, never dimmed). Nothing stored follows a line's place (ui.gpos is not saved) */
static const uint8_t GLO_PAGE[GP_N][GLO_PMAX] = {{G_TR, G_VELO, G_MULTI, G_MIDI, G_BT, G_AUDIO, G_SEQ, G_KIT, G_KEYL,
                                                  G_ASAVE, G_ABOUT, G_BACK},
                                                 {G_ACH, G_BCH, G_DRCH, G_INP, G_OUT, G_CLKP, G_CLKO, G_CLKL, G_THRU,
                                                  G_VCC, G_CC, G_CCSHOW, G_PANIC, G_BACK},
                                                 {G_BLE, G_SEARCH, G_FORGET, G_AUTO, G_BACK},
                                                 {G_LAT, G_MAXV, G_UNI, G_UNIL, G_UNIM, G_UAUTO, G_STEAL, G_VOL, G_ENG,
                                                  G_RVQ, G_OSC, G_FLT, G_BACK},
                                                 {G_VER, G_BY, G_KOFI, G_DIAG, G_TRSD, G_BACK}, {G_IUSB, G_ITRS, G_IBT, G_BACK},
                                                 {G_OTO, G_BACK}, {G_CLK, G_CSRC, G_BACK},
                                                 {G_MMODE, G_MEDIT, G_MSPLIT, G_MDYN, G_MVOI, G_MSEND, G_MVOA, G_MVOB,
                                                  G_MVCA, G_MVCB, G_BACK},
                                                 {G_CH, G_OCH, G_BACK}, {G_MCHB, G_BOUT, G_BACK},
                                                 {G_SPARP, G_SKLAT, G_SRQ, G_SPCNT, G_SBASE, G_SBASEB, G_SDOUT, G_BACK},
                                                 {G_VON, G_VFIX, G_VCRV, G_VSENS, G_VDRUM, G_VCAL, G_BACK},
                                                 {G_VK0, G_VK1, G_VK2, G_VK3, G_VK4, G_VK5, G_VK6, G_VSKIP, G_VNORM,
                                                  G_VRST, G_BACK}};
static const uint8_t GLO_PN[GP_N] = {12, 14, 5, 13, 6, 4, 2, 3, 11, 3, 3, 8, 7, 11};
#elif FM1B_BLE
static const uint8_t GLO_PAGE[GP_N][GLO_PMAX] = {{G_TR, G_MIDI, G_BT, G_AUDIO, G_KEYL, G_ASAVE, G_SEQ, G_KIT, G_ABOUT,
                                                  G_BACK},
                                                 {G_CH, G_INP, G_OUT, G_CLKP, G_THRU, G_VCC, G_CC, G_PANIC, G_BACK},
                                                 {G_BLE, G_SEARCH, G_FORGET, G_AUTO, G_BACK},
                                                 {G_LAT, G_UNI, G_UNIM, G_UAUTO, G_STEAL, G_VOL, G_ENG, G_RVQ, G_BACK},
                                                 {G_VER, G_BY, G_KOFI, G_DIAG, G_TRSD, G_BACK}, {G_IUSB, G_ITRS, G_IBT, G_BACK},
                                                 {G_OCH, G_OTO, G_BACK}, {G_CLK, G_CSRC, G_BACK},
                                                 {G_SPARP, G_SKLAT, G_SRQ, G_SPCNT, G_SBASE, G_BACK}};
static const uint8_t GLO_PN[GP_N] = {10, 9, 5, 9, 6, 4, 3, 3, 6};   /* (AUDIO: USB audio on / off left out, the platform has
                                          * no way to refuse the USB audio stream while it runs) */
#elif defined(HYB_MULTI)   /* the USB build with two parts: VELOCITY and MULTI after TRANSPOSE, its
                                          * page (1.19.1: the order and B CHANNEL as the Bluetooth build's) */
static const uint8_t GLO_PAGE[GP_N][GLO_PMAX] = {{G_TR, G_VELO, G_MULTI, G_MIDI, G_AUDIO, G_SEQ, G_KIT, G_KEYL, G_ASAVE,
                                                  G_ABOUT, G_BACK},
                                                 {G_ACH, G_BCH, G_DRCH, G_INP, G_OUT, G_CLKP, G_CLKO, G_CLKL, G_THRU,
                                                  G_VCC, G_CC, G_CCSHOW, G_PANIC, G_BACK},
                                                 {G_BACK},
                                                 {G_LAT, G_MAXV, G_UNI, G_UNIL, G_UNIM, G_UAUTO, G_STEAL, G_VOL, G_ENG,
                                                  G_RVQ, G_OSC, G_FLT, G_BACK},
                                                 {G_VER, G_BY, G_KOFI, G_DIAG, G_TRSD, G_BACK}, {G_IUSB, G_ITRS, G_BACK},
                                                 {G_OTO, G_BACK}, {G_CLK, G_CSRC, G_BACK},
                                                 {G_MMODE, G_MEDIT, G_MSPLIT, G_MDYN, G_MVOI, G_MSEND, G_MVOA, G_MVOB,
                                                  G_MVCA, G_MVCB, G_BACK},
                                                 {G_CH, G_OCH, G_BACK}, {G_MCHB, G_BOUT, G_BACK},
                                                 {G_SPARP, G_SKLAT, G_SRQ, G_SPCNT, G_SBASE, G_SBASEB, G_SDOUT, G_BACK},
                                                 {G_VON, G_VFIX, G_VCRV, G_VSENS, G_VDRUM, G_VCAL, G_BACK},
                                                 {G_VK0, G_VK1, G_VK2, G_VK3, G_VK4, G_VK5, G_VK6, G_VSKIP, G_VNORM,
                                                  G_VRST, G_BACK}};
static const uint8_t GLO_PN[GP_N] = {11, 14, 1, 13, 6, 3, 2, 3, 11, 3, 3, 8, 7, 11};
#else   /* the USB build (no Bluetooth): no BLUETOOTH page (GP_BT stays, out of reach), no BT port in */
static const uint8_t GLO_PAGE[GP_N][GLO_PMAX] = {{G_TR, G_MIDI, G_AUDIO, G_KEYL, G_ASAVE, G_SEQ, G_KIT, G_ABOUT, G_BACK},
                                                 {G_CH, G_INP, G_OUT, G_CLKP, G_THRU, G_VCC, G_CC, G_PANIC, G_BACK},
                                                 {G_BACK}, {G_LAT, G_VOL, G_ENG, G_RVQ, G_BACK},
                                                 {G_VER, G_BY, G_KOFI, G_DIAG, G_TRSD, G_BACK}, {G_IUSB, G_ITRS, G_BACK},
                                                 {G_OCH, G_OTO, G_BACK}, {G_CLK, G_CSRC, G_BACK},
                                                 {G_SPARP, G_SKLAT, G_SRQ, G_SPCNT, G_SBASE, G_BACK}};
static const uint8_t GLO_PN[GP_N] = {9, 9, 1, 5, 6, 3, 3, 3, 6};
#endif
#ifdef HYB_MULTI
static const uint8_t GLO_OPEN[GP_N] = {G_NONE, G_MIDI, G_BT, G_AUDIO, G_ABOUT, G_INP, G_OUT, G_CLKP, G_MULTI, G_ACH,
                                       G_BCH, G_SEQ, G_VELO, G_VCAL};
static const uint8_t GLO_UP[GP_N] = {GP_TOP, GP_TOP, GP_TOP, GP_TOP, GP_TOP, GP_MIDI, GP_MIDI, GP_MIDI, GP_TOP, GP_MIDI,
                                     GP_MIDI, GP_TOP, GP_TOP, GP_VELO};
static const char *const GLO_TITLE[GP_N] = {"GLOBAL", "MIDI", "BLUETOOTH", "AUDIO", "ABOUT", "MIDI IN", "MIDI OUT",
                                            "CLOCK", "MULTI", "A CHANNEL", "B CHANNEL", "SEQ", "VELOCITY", "CALIBRATE"};
#else
static const uint8_t GLO_OPEN[GP_N] = {G_NONE, G_MIDI, G_BT, G_AUDIO, G_ABOUT, G_INP, G_OUT, G_CLKP, G_SEQ};   /* the
                                          * line that opens a page */
static const uint8_t GLO_UP[GP_N] = {GP_TOP, GP_TOP, GP_TOP, GP_TOP, GP_TOP, GP_MIDI, GP_MIDI, GP_MIDI, GP_TOP};   /* its
                                          * page */
static const char *const GLO_TITLE[GP_N] = {"GLOBAL", "MIDI", "BLUETOOTH", "AUDIO", "ABOUT", "MIDI IN", "MIDI OUT",
                                            "CLOCK", "SEQ"};   /* (short: the strip has the line's value beside it) */
#endif
#ifdef HYB_MULTI
#define G_CH_NAME "IN"                   /* (1.19.4: A CHANNEL's page: IN (the channel listened to), OUT (OUT CHANNEL's)) */
#define G_OCH_NAME "OUT"
#else
#define G_CH_NAME "CHANNEL"
#define G_OCH_NAME "CHANNEL"
#endif
static const char *const G_NAME[G_N] = {"", "TRANSPOSE", "MIDI", "BLUETOOTH", "AUDIO", "BACK", G_CH_NAME, "THRU",
                                        "MODE", "PANIC", "BLUETOOTH", "SEARCH", "FORGET", "AUTO CONNECT", "LATENCY",
                                        "MIDI CC", "KEY LIGHTS", "ABOUT", APP_NAME " " HYB_VERSION HYB_RVTAG HYB_EDITION,
                                        "BY J0KERPACK", "KO-FI.COM/J0KERPACK", "DIAGNOSIS", "TRS IN", "SYNTH VOL", "CC 7", "IN PORTS", "OUT",
                                        "CLOCK", "USB", "TRS", "BLUETOOTH", G_OCH_NAME, "PORTS", "SOURCE", "DRUM KIT",
                                        "ENGINE", "AUTO SAVE", "REVERB",
#ifdef HYB_MULTI
                                        "SEQ", "PATCH ARP", "KEY LATCH", "REC QUANT", "BASE A", "BASE B", "DRUM OUT",
#else
                                        "SEQ", "PATCH ARP", "KEY LATCH", "REC QUANT", "BASE NOTE", "", "",
#endif
                                        "PRECOUNT",
#ifdef HYB_MULTI
                                        "MULTI", "MODE", "EDIT", "SPLIT", "IN", "VOICES", "B SEND",
                                        "ALLOC", "MAX VOICES", "UNI MAX SINGLE", "UNI MAX MULTI", "MONO UNI MAX",
                                        "UNISON AUTO", "STEAL",
                                        "VOL A", "VOL B", "VOL A CC", "VOL B CC", "OSC", "FILT KNOBS",
                                        "VELOCITY", "VELOCITY", "FIXED", "CURVE", "SENSITIVITY", "DRUM ACC", "CALIBRATE",
                                        "C#5", "D5", "D#5", "E5", "F5", "F#5", "G5", "CHECK", "RESET", "SKIP KEY",
                                        "DRUM CHANNEL", "OUT", "A CHANNEL", "B CHANNEL", "CLK OFFSET",
                                        "CLK LOCK", "CC SHOW",
#elif FM1B_BLE
                                        "UNISON MAX", "MONO UNI MAX", "UNISON AUTO", "STEAL",
#endif
                                        };
static UI_LATE uint32_t glo_page_of(uint32_t g)   /* the page line g opens (GP_TOP: none) */
{
    uint32_t p;
    for (p = 1; p < GP_N && GLO_OPEN[p] != g; p++)
        ;
    return p < GP_N ? p : GP_TOP;
}
static UI_LATE uint32_t glo_line(uint32_t p, uint32_t g)   /* g's line on page p (0: not there) */
{
    uint32_t i;
    for (i = 0; i < GLO_PN[p] && GLO_PAGE[p][i] != g; i++)
        ;
    return i < GLO_PN[p] ? i : 0u;
}

static void glo_touch(void) { ui.glo_t = plat_ms() | 1u; }   /* (saved GLO_SAVE_MS later: app_frame) */
#ifdef HYB_VELO
/* GLO > VELOCITY > CALIBRATE (1.18C26; a guide after 1.18C30): the top row's keys (C#5 .. G5, no converter: the LED-line
 * probe's peak, latched for each press by fm1-base's lib/touch.c, the same peak its note plays) given ends of their
 * own, one key at a time, the strip saying what to do:
 *   GUIDE  each key in turn: VG_K light strikes ("C#5 LIGHT 1/3"), then VG_K firm ones ("C#5 FIRM 2/3"). A strike
 *          with no reading is not counted ("NOT READ" "AGAIN"), another key's is not either ("WRONG KEY" "PLAY C#5").
 *          Three strikes of a kind more than VG_SPREAD apart: that kind again ("UNEVEN"); the firm median less than
 *          VG_SPAN above the light one: the key again ("TOO CLOSE"). Else the ends put the light median at about
 *          VG_VL and the firm median at about VG_VF (linear between, touch_curve still bending it; the analog keys'
 *          light and firm strikes land about there, an assumption), "C#5 OK" "24-41", and the next key.
 *   CHECK  each key once at medium force ("C#5 MEDIUM" "CHECK 1/7"); a key more than VG_TOL from the median of the
 *          seven is flagged ("V 30 REDO" on its line).
 *   DONE   "ALL DONE" "PRESS SAVE" (or the key to redo); any key struck shows the velocity it plays (a test).
 * The lines: each key's state or ends ("LIGHT 2/3", "OK 24-41", "ROW": the row's), SKIP KEY (the key guided keeps
 * the ends it has), CHECK (the check again), RESET (every key's ends the row's, the guide from C#5). REC on a key's
 * line: that key again (alone once the guide is through). Nothing is written until SAVE (set.ve_lo / ve_hi, GLO's;
 * its light blinks while they wait). The measured basis of the numbers (Velo's lab notes, lib/sense.c): a bare
 * contact lifts the probe ~20, a light press 27..36, a firm one 43..45 (saturated): one count of the probe is ~10..15
 * of velocity on a calibrated key, so VG_TOL is wide. */
#define VG_K 3u                          /* strikes of each kind */
#define VG_SPAN 6u                       /* the least distance of the light and the firm median (probe counts) */
#define VG_SPREAD 8u                     /* the most distance of three strikes of a kind */
#define VG_VL 20u                        /* the velocity the light median plays at (assumption: comparable to the */
#define VG_VF 120u                       /* analog keys' light and firm strikes), and the firm median's */
#define VG_TOL 30u                       /* CHECK: the most distance of a key's velocity from the seven's median */
enum { VG_GUIDE, VG_CHECK, VG_DONE };
static struct {
    uint8_t st, key, ph, n;              /* the stage; the key guided (0 C#5 .. 6 G5); light / firm; strikes counted */
    uint8_t one;                         /* that key alone (REC on its line once the guide was through) */
    uint8_t ok, bad;                     /* bit i: key i calibrated since the page opened; flagged by CHECK */
    uint8_t pk[2][VG_K];                 /* the key's light and firm peaks */
    uint8_t cv[VE_NTOP];                 /* CHECK: each key's velocity (0: none) */
    uint8_t lk, lv;                      /* DONE: the key struck last + 1, the velocity it played */
} vc PLAT_POOL;
static UI_LATE void vg_go(uint32_t st, uint32_t key)   /* the guide to stage st on key */
{
    vc.st = (uint8_t)st;
    vc.key = (uint8_t)key;
    vc.ph = vc.n = 0;
    vc.lk = 0;
    if (st == VG_CHECK && !key) {        /* (a check from the start: the last one's velocities gone) */
        uint32_t k;
        for (k = 0; k < VE_NTOP; k++) vc.cv[k] = 0;
        vc.bad = 0;
    }
    if (st != VG_DONE) ui.gpos = (uint8_t)key;   /* (the cursor to its line) */
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void ve_cal_clear(void)   /* the page opened: the guide from C#5 */
{
    vc.one = vc.ok = vc.bad = 0;
    vg_go(VG_GUIDE, 0);
}
static UI_LATE void vg_check_end(void)   /* CHECK through: the keys far from the seven's median flagged */
{
    uint32_t k, j, n = 0, m;
    uint8_t s[VE_NTOP];
    for (k = 0; k < VE_NTOP; k++)        /* (the velocities checked, sorted) */
        if (vc.cv[k]) {
            for (j = n++; j && s[j - 1u] > vc.cv[k]; j--) s[j] = s[j - 1u];
            s[j] = vc.cv[k];
        }
    vc.bad = 0;
    if (n) {
        m = s[n / 2u];
        for (k = 0; k < VE_NTOP; k++)
            if (vc.cv[k] && (vc.cv[k] > m + VG_TOL || vc.cv[k] + VG_TOL < m)) vc.bad |= (uint8_t)(1u << k);
    }
    vg_go(VG_DONE, 0);
}
static UI_LATE void vg_next(void)        /* the key guided done or skipped: the next (CHECK after the last) */
{
    if (vc.st == VG_CHECK) {
        if (vc.key + 1u < VE_NTOP) vg_go(VG_CHECK, vc.key + 1u); else vg_check_end();
    } else if (vc.one || vc.key + 1u >= VE_NTOP) {
        if (vc.one) {                   /* (that key alone: its check's velocity gone, done) */
            vc.one = 0;
            vc.bad &= (uint8_t)~(1u << vc.key);
            vc.cv[vc.key] = 0;
            vg_go(VG_DONE, 0);
        }
        else vg_go(VG_CHECK, 0);
    } else {
        vg_go(VG_GUIDE, vc.key + 1u);
    }
}
static UI_LATE uint32_t vg_med(uint8_t *p)   /* three peaks: sorted in place, their median */
{
    uint32_t i, j;
    for (i = 1; i < VG_K; i++)
        for (j = i; j && p[j - 1u] > p[j]; j--) { const uint8_t t = p[j]; p[j] = p[j - 1u]; p[j - 1u] = t; }
    return p[VG_K / 2u];
}
static UI_LATE void vg_ends(uint32_t i, char *v)   /* key i's ends: "24-41", "ROW" (the row's) */
{
    if (!set.ve_hi[i]) { text_cat(v, "ROW"); return; }
    put_sint(v + text_len(v), set.ve_lo[i], 0);
    text_cat(v, "-");
    put_sint(v + text_len(v), set.ve_hi[i], 0);
}
static UI_LATE void vg_learn(uint32_t i)   /* key i's light and firm strikes in: its ends, or the reason not */
{
    const char *nm = G_NAME[G_VK0 + i];
    const uint32_t L = vg_med(vc.pk[0]), F = vg_med(vc.pk[1]);
    float lo, hi, d;
    char t[16] = "";
    if (F < L + VG_SPAN) {               /* (firm hardly firmer: the key again) */
        app_say(nm, "TOO CLOSE: AGAIN");
        vc.ph = vc.n = 0;
        return;
    }
    d = (float)(F - L) / (float)(VG_VF - VG_VL);   /* (probe counts a velocity step) */
    lo = (float)L - (float)(VG_VL - 1u) * d;
    hi = lo + 126.f * d;
    set.ve_lo[i] = (uint8_t)(lo > 0.f ? (uint32_t)(lo + .5f) : 0u);
    set.ve_hi[i] = (uint8_t)(hi < 254.f ? (uint32_t)(hi + .5f) : 254u);
    if (set.ve_hi[i] <= set.ve_lo[i]) set.ve_hi[i] = (uint8_t)(set.ve_lo[i] + 1u);
    touch_top_ends(VE_TOP0 + i, set.ve_lo[i], set.ve_hi[i]);
    glo_touch();
    vc.ok |= (uint8_t)(1u << i);
    text_cat(t, nm);
    text_cat(t, " OK");
    vg_ends(i, t + text_len(t) + 1u);    /* (two texts in one: the name, the ends) */
    app_say(t, t + text_len(t) + 1u);
    vg_next();
}
static UI_LATE void ve_cal_key(uint32_t k)   /* key k struck (key_on, its velocity taken): on CALIBRATE, counted or
                                          * told why not; DONE: shown */
{
    const uint32_t i = k - VE_TOP0;
    char t[16] = "";
    int pk;
    if (ui.view != V_GLO || ui.gpage != GP_VCAL || ui.gch) return;
    ui.d_rows = ui.d_lcd = 1;
    if (vc.st == VG_DONE) {              /* (a test: any key, the velocity it plays) */
        vc.lk = (uint8_t)(k + 1u);
        vc.lv = (uint8_t)key_vel(k);
        return;
    }
    if (i != vc.key) {                   /* (another key: not counted) */
        text_cat(t, "PLAY ");
        text_cat(t, G_NAME[G_VK0 + vc.key]);
        app_say("WRONG KEY", t);
        return;
    }
    pk = touch_top_peak(k);
    if (pk < 0) { app_say(G_NAME[G_VK0 + i], "NOT READ: AGAIN"); return; }   /* (the probe never saw it closed) */
    if (vc.st == VG_CHECK) {
        vc.cv[i] = touch_top_vel(k, (uint32_t)pk);
        vg_next();
        return;
    }
    vc.pk[vc.ph][vc.n++] = (uint8_t)(pk > 254 ? 254 : pk);
    if (vc.n < VG_K) return;
    vg_med(vc.pk[vc.ph]);                /* (sorted: the spread) */
    if (vc.pk[vc.ph][VG_K - 1u] > vc.pk[vc.ph][0] + VG_SPREAD) {
        app_say(G_NAME[G_VK0 + i], vc.ph ? "UNEVEN: FIRM AGAIN" : "UNEVEN: LIGHT AGAIN");
        vc.n = 0;
        return;
    }
    if (!vc.ph) { vc.ph = 1; vc.n = 0; return; }
    vg_learn(i);
}
static UI_LATE void vg_rec(uint32_t g)   /* REC on CALIBRATE's line g (a key's, SKIP KEY, CHECK, RESET) */
{
    uint32_t k;
    if (g >= G_VK0 && g <= G_VK6) {      /* (that key again: alone once the guide is through) */
        vc.one = (uint8_t)(vc.st != VG_GUIDE || vc.one);
        vg_go(VG_GUIDE, g - G_VK0);
    } else if (g == G_VSKIP) {
        if (vc.st == VG_DONE) return;
        app_say(G_NAME[G_VK0 + vc.key], "SKIPPED");
        vg_next();
    } else if (g == G_VNORM) {           /* (CHECK again) */
        vc.one = 0;
        vg_go(VG_CHECK, 0);
    } else if (g == G_VRST) {            /* (every key's ends the row's, the guide from C#5) */
        for (k = 0; k < VE_NTOP; k++) {
            set.ve_lo[k] = set.ve_hi[k] = 0;
            touch_top_ends(VE_TOP0 + k, 0, 0);
        }
        glo_touch();
        ve_cal_clear();
        app_say("Calibration", "RESET");
    }
}
static UI_LATE void vg_line(uint32_t g, char *v)   /* CALIBRATE's line g's value */
{
    const uint32_t i = g - G_VK0;
    if (g >= G_VK0 && g <= G_VK6) {
        if (vc.st == VG_GUIDE && i == vc.key) {   /* ("LIGHT 2/3", "FIRM 0/3") */
            text_cat(v, vc.ph ? "FIRM " : "LIGHT ");
            put_sint(v + text_len(v), vc.n, 0);
            text_cat(v, "/3");
        } else if (vc.st != VG_GUIDE && vc.cv[i]) {   /* ("V 63", "V 20 REDO") */
            text_cat(v, "V ");
            put_sint(v + 2, vc.cv[i], 0);
            if (vc.bad >> i & 1u) text_cat(v, " REDO");
        } else if (vc.st == VG_CHECK && i == vc.key) {
            text_cat(v, "MEDIUM");
        } else {                         /* ("OK 24-41": calibrated since the page opened; "24-41", "ROW") */
            if (vc.ok >> i & 1u) text_cat(v, "OK ");
            vg_ends(i, v);
        }
    } else if (g == G_VNORM && vc.st == VG_DONE) {   /* (the check's verdict, once there was one) */
        uint32_t k, n = 0;
        for (k = 0; k < VE_NTOP; k++) n += vc.cv[k] != 0;
        if (n) text_cat(v, vc.bad ? "DIFFERS" : "EVEN");
    }
}
static UI_LATE void ve_cal_lcd(char *a, char *v)   /* the strip on CALIBRATE: what to do now */
{
    uint32_t k;
    if (vc.st == VG_GUIDE) {             /* ("C#5 LIGHT 1/3", "KEY 1/7") */
        text_cat(a, G_NAME[G_VK0 + vc.key]);
        text_cat(a, vc.ph ? " FIRM " : " LIGHT ");
        put_sint(a + text_len(a), vc.n, 0);
        text_cat(a, "/3");
        text_cat(v, "KEY ");
    } else if (vc.st == VG_CHECK) {      /* ("C#5 MEDIUM", "CHECK 1/7") */
        text_cat(a, G_NAME[G_VK0 + vc.key]);
        text_cat(a, " MEDIUM");
        text_cat(v, "CHECK ");
    } else if (vc.lk) {                  /* (DONE, a key struck: "F4" "V 57") */
        note_text(53u + vc.lk - 1u, a);
        text_cat(v, "V ");
        put_sint(v + 2, vc.lv, 0);
        return;
    } else {                             /* ("ALL DONE" "PRESS SAVE"; "E5 DIFFERS" "REC: REDO") */
        for (k = 0; k < VE_NTOP && !(vc.bad >> k & 1u); k++)
            ;
        if (k < VE_NTOP) { text_cat(a, G_NAME[G_VK0 + k]); text_cat(a, " DIFFERS"); text_cat(v, "REC: REDO"); }
        else { text_cat(a, "ALL DONE"); text_cat(v, set.ve_fl & VE_OFF ? "VELOCITY IS OFF" : ui.glo_t ? "PRESS SAVE" : "SAVED"); }
        return;
    }
    put_sint(v + text_len(v), vc.key + 1, 0);   /* (the key's place of seven) */
    text_cat(v, "/7");
}
#endif

/* GLO's settings into the settings object as saved (what SAVE keeps, the octave and the patch, stays as it was saved:
 * dev_remember's way); the saved device too (FORGET) */
static int glo_save(void)
{
    settings_t s;
    uint32_t k;
    settings_load(&s);
    s.lat = set.lat;
    s.ble_off = set.ble_off;
    for (k = SET_LEN_V1; k < __builtin_offsetof(settings_t, qa_ok); k++)
        ((uint8_t *)&s)[k] = ((const uint8_t *)&set)[k];
    s.glo_ok = GLO_V1;
    s.tr = set.tr;
    s.midi_ch = set.midi_ch;
    s.thru = set.thru;
    s.clk = set.clk;
    s.ble_auto = set.ble_auto;
    s.key_dark = set.key_dark;
    s.rv_q = set.rv_q;                   /* (REVERB) */
    s.sq_def = set.sq_def;               /* (GLO > SEQ, 1.14) */
    s.pm_ok = PM_V1;
    for (k = 0; k < 7u; k++)
        s.pm[k] = set.pm[k];
    s.sq_ok = SQ_V1;
    s.sq_pat = set.sq_pat;
    s.sq_pad = set.sq_pad;
    s.dr_ok = DR_V1;
    s.dr_pat = set.dr_pat;
    s.dr_lev = set.dr_lev;
    s.vo_ok = VO_V1;
    s.vol = set.vol;
    s.vol_cc = set.vol_cc;
    s.md_ok = MD_V1;
    s.out_ch = set.out_ch;
    s.out_to = set.out_to;
    s.in_off = set.in_off;
    s.clk_src = set.clk_src;
    s.sq_base = set.sq_base;
    s.eng = set.eng;
    s.md_pad = set.md_pad;               /* (AUTO SAVE) */
#ifdef HYB_MULTI
    s.mx_ok = MX_V1;                     /* (GLO > MULTI; part B's patch: SAVE's, as part A's) */
    s.mx_mode = set.mx_mode;
    s.mx_split = set.mx_split;
    s.mx_chb = set.mx_chb;
    s.mx_nva = set.mx_nva;
    s.mx_send = set.mx_send;
    s.mx_dyn = set.mx_dyn;
    s.mx_maxv = set.mx_maxv;
    s.mx_uni = set.mx_uni;
    s.mx_unim = set.mx_unim;
    s.mx_unx = set.mx_unx;               /* (1.18C48: the unison caps' rest) */
    s.mx_uauto = set.mx_uauto;
    s.mx_steal = set.mx_steal;
    s.sqb_ok = SQB_V1;                   /* (SEQ B) */
    s.sqb_pat = set.sqb_pat;
    s.sqb_pad = set.sqb_pad;
    s.sqb_base = set.sqb_base;
    s.mx_vcc[0] = set.mx_vcc[0];         /* (VOL A / B CC) */
    s.mx_vcc[1] = set.mx_vcc[1];
    s.tempo = tp_run;                    /* (1.18: the global tempo, the one running) */
    s.osc_ok = OSC_V1;                   /* (1.18C22: OSC) */
    s.osc_cl = set.osc_cl;
    s.flt_sm = set.flt_sm;               /* (FILT KNOBS, in OSC's padding) */
    s.ck_fl = set.ck_fl;                 /* (1.19.5: CLK LOCK, in OSC's padding too) */
    s.ve_ok = VE_V1;                     /* (1.18C26: VELOCITY, CALIBRATE's ends) */
    s.ve_fl = set.ve_fl;
    s.ve_fix = set.ve_fix;
    s.ve_crv = set.ve_crv;
    s.ve_sens = set.ve_sens;
    for (k = 0; k < VE_NTOP; k++) { s.ve_lo[k] = set.ve_lo[k]; s.ve_hi[k] = set.ve_hi[k]; }
    s.dc_ok = DC_V1;                     /* (1.18C31: DRUM CHANNEL) */
    s.dr_ch = set.dr_ch;
    s.mo = set.mo;                       /* (1.19.4: the parts' OUT, the new IN values; CLK OFFSET) */
    s.ck_ofs = set.ck_ofs;
    s.cm_ok = CM_V1;                     /* (1.19.6: the MIDI CC map) */
    for (k = 0; k < 128u; k++) s.cm[k] = set.cm[k];
    pv_keep();                           /* (1.19.7: VOL A / B) */
    s.pv_st[0] = set.pv_st[0];
    s.pv_st[1] = set.pv_st[1];
#endif
    return store_save(OBJ_SETTINGS, &s, sizeof s);
}

static void ble_toggle(void)             /* Bluetooth off / on (REC held; GLO's BLUETOOTH) */
{
    set.ble_off = (uint8_t)!set.ble_off;
#ifdef HYB_RV_LEND
    if (bt_lent) {                       /* (its memory is the reverb's until a restart: on then, as saved; 1.14: SAVE
                                          * named, GLO's settings are written by SAVE alone with AUTO SAVE off) */
        app_say(set.ble_off ? "Bluetooth" : "BT ON", set.ble_off ? "OFF" : "SAVE+RESTART");   /* ("Bluetooth"
                                          * and "ON: SAVE+RESTART" do not fit the strip) */
        glo_touch();
        return;
    }
    if (set.ble_off) {                   /* (1.14) off, not lent yet: its memory goes to the shimmer and REVERB FULL at
                                          * the next power-on, once the setting is saved */
        plat_ble_enable(0);
        app_say("BT OFF", "SAVE+RESTART");
        glo_touch();
        return;
    }
#endif
    plat_ble_enable(!set.ble_off);
    say_on("Bluetooth", !set.ble_off);
    glo_touch();
}

static void glo_panic(void)              /* all notes off: the keys' and HOLD's (and out over MIDI), then every voice;
                                          * 1.18C49: all sound off too (eng_panic: the tails faded and cleared) */
{
    all_off_both();
    ev_put(0xB0u, 123u, 0u);             /* (the engine: sy_all_off, the arpeggiator's keys too) */
    ev_put(0xB0u, 120u, 0u);
    midi_send_ch(0xB0u, 123u, 0);
#ifdef HYB_MULTI
    midi_send_p(1u, 0xB0u, 123u, 0);     /* (1.19.4: part B's OUT too) */
#endif
    app_say("Panic", "ALL NOTES OFF");
}

/* the values of TRANSPOSE, CHANNEL, LATENCY, SYNTH VOL, ENGINE, OUT's CHANNEL and PORTS, CLOCK's MODE and SOURCE as a list:
 * their number, the one now, set one, its text */
static const char *const OP_NAME[OP_N] = {"USB+BT", "USB", "BT", "OFF"};
#if !FM1B_BLE   /* the USB build: OUT PORTS a list of two (USB, OFF), CLOCK SOURCE of three (no BT). The saved values stay
                 * as they are (the settings object is the Bluetooth build's too) until one is chosen here: USB+BT
                 * works as USB, BT as OFF (nothing behind it), an external clock from BT as one from ANY (out_to_eff,
                 * clk_src_eff) */
#define GV_OTO_N 2u
#define GV_OTO_OP(i) ((i) ? OP_OFF : OP_USB)
#define GV_CSRC_N (CS_N - 1u)
#endif
static const char *const CLK_NAME[CLK_N] = {"INTERNAL", "EXTERNAL", "MASTER"};
static const char *const CS_NAME[CS_N] = {"ANY", "USB", "TRS", "BT"};
static const char *const EN_NAME[EN_N] = {"SYNTH+DRUMS", "DRUMS ONLY", "MIDI ONLY"};   /* (ENGINE's list; its line
                                          * and the strip, short: EN_SHORT, as HOME's tile) */
static const char *const EN_SHORT[EN_N] = {"ALL", "DRUMS", "MIDI"};
#ifdef HYB_MULTI
static const char *const MX_NAME[SY_M_N] = {"SINGLE", "LAYER", "SPLIT", "CHANNELS"};   /* MODE (SY_M_*) */
#endif
#ifdef HYB_MULTI
#define GV_BASE(g) (*((g) == G_SBASEB ? &set.sqb_base : &set.sq_base))   /* (GLO > SEQ's BASE A / B) */
#else
#define GV_BASE(g) set.sq_base
#endif
static UI_LATE uint32_t gv_n(uint32_t g)
{
    if (g == G_RVQ || g == G_SPCNT) return 3u;   /* (REVERB: FULL, HALF, AUTO; PRECOUNT: OFF, 1 BAR, 2 BARS) */
    if (g == G_SBASE || g == G_SBASEB) return SQ_BASE_HI - SQ_BASE_LO + 2u;   /* (GLO > SEQ's BASE NOTE: OFF, C1 .. C7) */
#ifdef HYB_MULTI
    if (g == G_MMODE) return SY_M_N;
    if (g == G_MEDIT) return 2u;
    if (g == G_MSPLIT) return 128u;
    if (g == G_CH || g == G_MCHB) return 18u;   /* (1.19.4: the INs, OFF, OMNI, 1 .. 16) */
    if (g == G_DRCH || g == G_OCH || g == G_BOUT) return 17u;   /* (OFF, 1 .. 16) */
    if (g == G_CLKO) return 2u * CK_OFS_MAX + 1u;   /* (-50 .. +50 ms) */
    if (g == G_MVOI) return SY_MAX_VOICES - 1u;   /* (part A's 1 .. 15) */
    if (g == G_MSEND) return MX_SEND_PATCH + 1u;   /* (PATCH, 0 .. 127) */
    if (g == G_MDYN || g == G_UAUTO || g == G_STEAL) return 2u;
    if (g == G_MVCA || g == G_MVCB) return 121u;   /* (OFF, 0 .. 119) */
    if (g == G_MVOA || g == G_MVOB) return 128u;   /* (VOL A / B 0 .. 127) */
    if (g == G_OSC || g == G_FLT) return 2u;   /* (ORIGINAL, CLEAN / SMOOTH) */
    if (g == G_VFIX) return 127u;        /* (VELOCITY's FIXED 1 .. 127, CURVE, SENSITIVITY -5 .. +5) */
    if (g == G_VCRV) return VC_N;
    if (g == G_VSENS) return 2u * VE_SENS_MAX + 1u;
    if (g == G_MAXV) return (uint32_t)lat_cap() - 3u;   /* (4 .. the latency's voices) */
    if (g == G_UNI || g == G_UNIL) return 5u;   /* (1.18C48: OFF, 1 .. 4; MONO UNI MAX OFF, 1 .. 6) */
    if (g == G_UNIM) return 7u;
#elif FM1B_BLE
    if (g == G_UAUTO || g == G_STEAL) return 2u;
    if (g == G_UNI) return 4u;
    if (g == G_UNIM) return 6u;
#endif
#if !FM1B_BLE
    if (g == G_OTO) return GV_OTO_N;
    if (g == G_CSRC) return GV_CSRC_N;
#endif
    return g == G_TR ? 2u * TR_MAX + 1u : g == G_CH ? 17u : g == G_VOL ? 128u : g == G_OCH ? 16u : g == G_OTO ? OP_N
         : g == G_CSRC ? CS_N : g == G_ENG ? EN_N : 3u;   /* (LATENCY, MODE: 3) */
}
static UI_LATE uint32_t gv_get(uint32_t g)
{
    if (g == G_RVQ) return RVQ_Q(set.rv_q) - RVQ_FULL;
    if (g == G_SPCNT) return SQ_PCB;
    if (g == G_SBASE || g == G_SBASEB) return GV_BASE(g) ? GV_BASE(g) - SQ_BASE_LO + 1u : 0u;
#ifdef HYB_MULTI
    if (g == G_MMODE) return set.mx_mode;
    if (g == G_OSC) return set.osc_cl;
    if (g == G_FLT) return set.flt_sm;
    if (g == G_VFIX) return set.ve_fix - 1u;
    if (g == G_VCRV) return set.ve_crv;
    if (g == G_VSENS) return (uint32_t)(set.ve_sens + VE_SENS_MAX);
    if (g == G_MEDIT) return ed_part;
    if (g == G_MSPLIT) return set.mx_split;
    if (g == G_CH) return set.mo & MO_AINOFF ? 0u : set.midi_ch + 1u;   /* (1.19.4: OFF, OMNI, 1 .. 16) */
    if (g == G_MCHB) return set.mo & MO_BOMNI ? 1u : set.mx_chb ? set.mx_chb + 1u : 0u;
    if (g == G_OCH) return set.mo & MO_AOFF ? 0u : set.out_ch + 1u;   /* (OFF, 1 .. 16) */
    if (g == G_BOUT) return (set.mo & MO_B) - 1u;
    if (g == G_CLKO) return (uint32_t)(set.ck_ofs + CK_OFS_MAX);
    if (g == G_DRCH) return set.dr_ch;
    if (g == G_MVOI) return set.mx_nva - 1u;
    if (g == G_MSEND) return set.mx_send >= MX_SEND_PATCH ? 0u : set.mx_send + 1u;
    if (g == G_MDYN) return set.mx_dyn;
    if (g == G_MAXV) return (uint32_t)voice_cap() - 4u;
    if (g == G_UNI) return SET_UNIS;
    if (g == G_UNIL) return SET_UNIL;
    if (g == G_UNIM) return SET_UNIM;
    if (g == G_UAUTO) return set.mx_uauto;
    if (g == G_STEAL) return set.mx_steal;
    if (g == G_MVCA || g == G_MVCB) { const uint32_t c = pv_cc_of(g == G_MVCB); return c > 119u ? 0u : c + 1u; }
    if (g == G_MVOA || g == G_MVOB) return pv_vol[g == G_MVOB];
#elif FM1B_BLE
    if (g == G_UNI) return SET_UNI;
    if (g == G_UNIM) return SET_UNIM;
    if (g == G_UAUTO) return SET_UAUTO;
    if (g == G_STEAL) return SET_STEAL;
#endif
#if !FM1B_BLE
    if (g == G_OTO) return out_to_eff() == OP_OFF;
    if (g == G_CSRC) return clk_src_eff();
#endif
    return g == G_TR ? (uint32_t)(set.tr + TR_MAX) : g == G_CH ? set.midi_ch : g == G_VOL ? set.vol : g == G_OCH ? set.out_ch
         : g == G_OTO ? set.out_to : g == G_CLK ? set.clk : g == G_CSRC ? set.clk_src : g == G_ENG ? set.eng : set.lat - LAT_LOW;
}
static UI_LATE char *pat_text(char *t, const char *w, int32_t n)   /* "<w>n" into t ("Pattern 3"), t */
{
    t[0] = 0;
    text_cat(t, w);
    put_sint(t + text_len(t), n, 0);
    return t;
}
#ifdef HYB_MULTI
static UI_LATE void sq_on_i(uint32_t i, int on);
static UI_LATE void sq_show(uint32_t i);
#endif
static UI_LATE void gv_set(uint32_t g, uint32_t i)
{
    if (g == G_SPCNT) {                  /* (1.18C40: PRECOUNT, the next start's: pc_ticks) */
        set.sq_def = (uint8_t)((set.sq_def & ~SQ_PCM) | (i < 3u ? i : 0u) << SQ_PC_SH);
        glo_touch();
        return;
    }
    if (g == G_SBASE || g == G_SBASEB) { /* (GLO > SEQ's BASE NOTE: as the page's SELECT, the latch giving way) */
        GV_BASE(g) = (uint8_t)(i ? i + SQ_BASE_LO - 1u : 0u);
#ifdef HYB_MULTI
        sqr[g == G_SBASEB].lclr = 1;
#else
        sq.lclr = 1;
#endif
        glo_touch();
        return;
    }
#ifdef HYB_MULTI
    if (g == G_OSC || g == G_FLT) {      /* OSC, FILT KNOBS: both parts' next block takes it (sy_synth osc_clean,
                                          * flt_sm: synth.c) */
        if (g == G_OSC) set.osc_cl = (uint8_t)(i ? 1u : 0u);
        else set.flt_sm = (uint8_t)(i ? 1u : 0u);
        osc_apply();
        glo_touch();
        return;
    }
    if (g == G_VFIX || g == G_VCRV || g == G_VSENS) {   /* VELOCITY (the next key struck takes it) */
        if (g == G_VFIX) set.ve_fix = (uint8_t)(i < 127u ? i + 1u : 127u);
        else if (g == G_VCRV) set.ve_crv = (uint8_t)(i < VC_N ? i : VC_NORMAL);
        else set.ve_sens = (int8_t)clamp((int32_t)i - VE_SENS_MAX, -VE_SENS_MAX, VE_SENS_MAX);
        ve_apply();
        glo_touch();
        return;
    }
#endif
    if (g == G_RVQ) {                    /* REVERB: the reverb's next block takes it (its tail cut, the ring cleared;
                                          * AUTO: FULL or HALF as the load is now, rvq_tick) */
        set.rv_q = (uint8_t)(RVQ_FULL + (i < 3u ? i : 2u));
        rva_half = 0;                    /* (a FULL or AUTO chosen: FULL now, AUTO and the guard watch from here) */
        rvq_apply();
        glo_touch();
        return;
    }
#if FM1B_BLE && !defined(HYB_MULTI)
    if (g >= G_UNI && g <= G_STEAL) {    /* AUDIO's unison settings (a note takes them at its start) */
        const uint32_t sh = g == G_UNI ? MD_UNI_SH : g == G_UNIM ? MD_UNIM_SH : g == G_UAUTO ? 6u : 7u;
        const uint32_t m = (g == G_UNI ? 3u : g == G_UNIM ? 7u : 1u) << sh;
        if (g == G_UNIM && i > 5u) i = 5u;
        set.md_pad = (uint8_t)((set.md_pad & ~m) | (i << sh & m));
        uni_apply();
        ui.d_rows = 1;                   /* (UNISON's hint, "5>2") */
        glo_touch();
        return;
    }
#endif
#ifdef HYB_MULTI
    if (g == G_MVOA || g == G_MVOB) {    /* (1.19.7: VOL A / B, as SHIFT + ALGO sets them) */
        pv_vol[g == G_MVOB] = (uint8_t)(i & 127u);
        glo_touch();
        return;
    }
    if (g == G_MVCA || g == G_MVCB) {    /* (VOL A / B CC: the list's 0 OFF, i controller i - 1) */
        set.mx_vcc[g == G_MVCB] = (uint8_t)(i ? (i > 120u ? 120u : i) : PV_CC_OFF);
        glo_touch();
        return;
    }
    if (g >= G_MDYN && g <= G_STEAL) {   /* ALLOC, AUDIO's voice settings (the engine: its next block) */
        if (g == G_MDYN && i != set.mx_dyn) { all_off_both(); set.mx_dyn = (uint8_t)(i & 1u); }   /* (the voices
                                                     * change hands) */
        else if (g == G_MAXV) {
            set.mx_maxv = (uint8_t)(i + 4u);
            lat_req = set.lat;           /* (the governor's limit under the cap: app_render) */
        } else if (g == G_UNI || g == G_UNIM) {   /* (1.18C48: SINGLE's 4 and the mono 6 in mx_unx, the old field at
                                          * its old most, 3 and 5, as an older firmware reads it) */
            const uint32_t m = g == G_UNI ? 4u : 6u, v = i > m ? m : i, b = g == G_UNI ? MX_UNX_S4 : MX_UNX_M6;
            *(g == G_UNI ? &set.mx_uni : &set.mx_unim) = (uint8_t)(v == m ? m - 1u : v);
            set.mx_unx = (uint8_t)((set.mx_unx & ~b) | (v == m ? b : 0u));
        } else if (g == G_UNIL) set.mx_unx = (uint8_t)((set.mx_unx & ~MX_UNX_L) | ((i > 4u ? 4u : i) + 1u));
        else if (g == G_UAUTO) set.mx_uauto = (uint8_t)(i & 1u);
        else if (g == G_STEAL) set.mx_steal = (uint8_t)(i & 1u);
        mx_apply();
        ui.d_rows = 1;                   /* (UNISON's hint, "5>2") */
        glo_touch();
        return;
    }
    if (g == G_CH) {                     /* (1.19.4) A CHANNEL > IN: OFF (MO_AINOFF, midi_ch 0), OMNI, 1 .. 16; the
                                          * notes of the channel left: off, their note-offs no longer come */
        set.midi_ch = (uint8_t)(i > 1u ? i - 1u : 0u);
        set.mo = (uint8_t)((set.mo & ~MO_AINOFF) | (i ? 0u : MO_AINOFF));
        ev_put(0xB0u, 123u, 0u);
        glo_touch();
        return;
    }
    if (g == G_OCH || g == G_BOUT) {     /* (1.19.4) A CHANNEL > OUT, B CHANNEL > OUT: OFF, 1 .. 16; all notes off on
                                          * the channel left (its notes' offs go elsewhere now) */
        midi_send_p(g == G_BOUT, 0xB0u, 123u, 0);
        if (g == G_BOUT) set.mo = (uint8_t)((set.mo & ~MO_B) | (i + 1u));
        else {
            set.mo = (uint8_t)((set.mo & ~MO_AOFF) | (i ? 0u : MO_AOFF));
            if (i) set.out_ch = (uint8_t)(i - 1u);
        }
        glo_touch();
        return;
    }
    if (g == G_CLKO) {                   /* (1.19.4) CLK OFFSET: ck_shift takes it at its next block */
        set.ck_ofs = (int8_t)((int32_t)i - CK_OFS_MAX);
        glo_touch();
        return;
    }
    if (g == G_DRCH) {                   /* DRUM CHANNEL (1.18C31): its notes the drums' from the next one in; the notes
                                          * of the channel now taken: off (their note-offs no longer reach the parts) */
        set.dr_ch = (uint8_t)(i > 16u ? 0u : i);
        ev_put(0xB0u, 123u, 0u);
        glo_touch();
        return;
    }
    if (g >= G_MMODE && g <= G_MSEND) {  /* MULTI (the engine takes it at its next block) */
        if (g == G_MEDIT) { part_edit(i); return; }   /* (not saved: the part edited) */
        if (g == G_MMODE && i != set.mx_mode) {
            all_off_both();             /* (the keys' and HOLD's: the voices change hands) */
            set.mx_mode = (uint8_t)(i < SY_M_N ? i : SY_M_SINGLE);
            if (set.mx_mode == SY_M_LAYER) hold_o = hold_on;   /* (1.16: LAYER's one HOLD: the part edited's) */
            if (set.mx_mode == SY_M_SINGLE) part_edit(0);   /* (SINGLE: part A alone, edited. 1.19.3: SEQ B
                                          * runs on, MIDI only; the voices are new, multi_setup) */
            home_sig[0] = home_sig[1] = 0;
            ui.d_lcd = 1;
        } else if (g == G_MSPLIT) set.mx_split = (uint8_t)(i & 127u);
        else if (g == G_MCHB) {
            if (!ed_part) hold_park_end();   /* (1.15: B's held notes out on the channel left: off) */
            ev_put(0xB0u | (set.mx_chb ? set.mx_chb - 1u : 0u), 123u, 0u);   /* (B's notes on the channel left: off) */
            set.mx_chb = (uint8_t)(i > 1u ? i - 1u : 0u);   /* (1.19.4: OFF, OMNI (MO_BOMNI, mx_chb 0), 1 .. 16) */
            set.mo = (uint8_t)((set.mo & ~MO_BOMNI) | (i == 1u ? MO_BOMNI : 0u));
        } else if (g == G_MVOI && i + 1u != set.mx_nva) {
            all_off_both();
            set.mx_nva = (uint8_t)(i + 1u);
        } else if (g == G_MSEND) set.mx_send = (uint8_t)(i ? i - 1u : MX_SEND_PATCH);
        mx_apply();
        glo_touch();
        return;
    }
#endif
#if !FM1B_BLE
    if (g == G_OTO) i = GV_OTO_OP(i);
#endif
    if (g == G_TR) set.tr = (int8_t)((int32_t)i - TR_MAX);
    else if (g == G_OCH || g == G_OTO) {
        midi_send_ch(0xB0u, 123u, 0);    /* (all notes off where they went: their note-offs go elsewhere now) */
        if (g == G_OCH) set.out_ch = (uint8_t)i; else set.out_to = (uint8_t)i;
    } else if (g == G_CLK || g == G_CSRC) {
        if (g == G_CLK) set.clk = (uint8_t)i; else set.clk_src = (uint8_t)i;
        ev_put(0xFCu, 0, 0);             /* (the engine's MIDI clock stopped: its tempo the patch's again until the
                                          * clock it takes now ticks) */
    }
    else if (g == G_CH) {
        set.midi_ch = (uint8_t)i;
        ev_put(0xB0u, 123u, 0u);         /* (the notes of the channel left: off, their note-offs no longer come) */
    } else if (g == G_VOL) {
        set.vol = (uint8_t)i;
    } else if (g == G_ENG) {
        if (i != set.eng) all_off_both();   /* (the keys' and HOLD's notes stopped, out over MIDI too: none hangs; the
                                          * audio interrupt fades the part going off, eng_step) */
        set.eng = (uint8_t)(i < EN_N ? i : EN_ALL);
    } else {
        set.lat = (uint8_t)(i + LAT_LOW);
        lat_req = set.lat;
        plat_audio_fifo(lat_fifo());     /* (the blocks ahead: the room for the voices' peaks) */
    }
    glo_touch();
}
/* (1.18C42) REVERB's names say the ring that runs: HALF32 (HALF on the float ring: every build since 1.18C41,
 * RV_HALF_F), FULL16 (FULL on the 16-bit ring: the Bluetooth two-part build with Bluetooth's memory not lent, i.e.
 * Bluetooth on at power-on) or FULL32 (the float ring: that build after a power-on with Bluetooth off, and the other
 * builds). The name is the state's now: a Bluetooth change takes the ring at the next power-on, the name with it */
#if defined(RV_HALF_F) && !RV_HALF_F && defined(HYB_MULTI) && FM1B_BLE
#define RVQ_HNAME "HALF16"
#else
#define RVQ_HNAME "HALF32"
#endif
#if defined(HYB_MULTI) && FM1B_BLE && defined(HYB_RV_LEND)
#define RVQ_FNAME (bt_lent ? "FULL32" : "FULL16")
#elif defined(HYB_MULTI) && FM1B_BLE
#define RVQ_FNAME "FULL16"
#else
#define RVQ_FNAME "FULL32"
#endif
static UI_LATE void gv_text(uint32_t g, uint32_t i, char *t)
{
    static const char *const LAT[3] = {"LOW", "MED 14", "HIGH 16"};   /* (codes LAT_LOW..LAT_HIGH) */
    if (g == G_RVQ) { text_cat(t, i == 2u ? "AUTO" : i ? RVQ_HNAME : RVQ_FNAME); return; }
    if (g == G_SBASE || g == G_SBASEB) { if (i) note_text(i + SQ_BASE_LO - 1u, t); else text_cat(t, "OFF"); return; }
    if (g == G_SPCNT) { text_cat(t, i == 2u ? "2 BARS" : i ? "1 BAR" : "OFF"); return; }
#ifdef HYB_MULTI
    if (g == G_LAT) {                    /* ("HIGH 14": the latency's voices under MAX VOICES) */
        static const char *const LN[3] = {"LOW ", "MED ", "HIGH "};
        const uint32_t c = (i == 2u ? VOICES_HIGH : i == 1u ? VOICES_MED : VOICES_STD);
        text_cat(t, LN[i < 3u ? i : 0u]);
        put_sint(t + text_len(t), (int32_t)(set.mx_maxv && set.mx_maxv < c ? set.mx_maxv : c), 0);
        return;
    }
    if (g == G_MMODE) { text_cat(t, MX_NAME[i < SY_M_N ? i : 0u]); return; }
    if (g == G_MEDIT) { text_cat(t, i ? "PART B" : "PART A"); return; }
    if (g == G_MSPLIT) { text_cat(t, "B < "); note_text(i, t); return; }
    if (g == G_CH || g == G_MCHB) {      /* (1.19.4: the INs, OFF, OMNI, 1 .. 16) */
        if (i > 1u) put_sint(t, (int32_t)i - 1, 0); else text_cat(t, i ? "OMNI" : "OFF");
        return;
    }
    if (g == G_DRCH || g == G_OCH || g == G_BOUT) { if (i) put_sint(t, (int32_t)i, 0); else text_cat(t, "OFF"); return; }
    if (g == G_CLKO) { put_sint(t, (int32_t)i - CK_OFS_MAX, i != CK_OFS_MAX); text_cat(t, " MS"); return; }
    if (g == G_MVOI) {                   /* ("A 8 B 8") */
        text_cat(t, "A "); put_sint(t + text_len(t), (int32_t)i + 1, 0);
        text_cat(t, " B "); put_sint(t + text_len(t), SY_MAX_VOICES - 1 - (int32_t)i, 0);
        return;
    }
    if (g == G_MSEND) { if (i) put_sint(t, (int32_t)i - 1, 0); else text_cat(t, "PATCH"); return; }
    if (g == G_MDYN) { text_cat(t, i ? "DYNAMIC" : "FIXED"); return; }
    if (g == G_MAXV) { put_sint(t, (int32_t)i + 4, 0); return; }
    if (g == G_MVCA || g == G_MVCB) { if (i) put_sint(t, (int32_t)i - 1, 0); else text_cat(t, "OFF"); return; }
    if (g == G_MVOA || g == G_MVOB) { put_sint(t, (int32_t)i, 0); return; }
    if (g == G_OSC || g == G_FLT) { text_cat(t, !i ? "ORIGINAL" : g == G_OSC ? "CLEAN" : "SMOOTH"); return; }
    if (g == G_VFIX) { put_sint(t, (int32_t)i + 1, 0); return; }
    if (g == G_VCRV) {
        static const char *const VC_NAME[VC_N] = {"SOFT2", "SOFT", "NORMAL", "HARD", "HARD2"};
        text_cat(t, VC_NAME[i < VC_N ? i : VC_NORMAL]);
        return;
    }
    if (g == G_VSENS) { put_sint(t, (int32_t)i - VE_SENS_MAX, 1); return; }
#endif
#if FM1B_BLE || defined(HYB_MULTI)
#ifdef HYB_MULTI
    if (g == G_UNIL) g = G_UNI;          /* (1.18C48: UNI MAX MULTI as SINGLE's) */
#endif
    if (g == G_UNI || g == G_UNIM) { if (i) put_sint(t, (int32_t)i, 0); else text_cat(t, "OFF"); return; }
    if (g == G_UAUTO) { text_cat(t, i ? "ON" : "OFF"); return; }
    if (g == G_STEAL) { text_cat(t, i ? "UNISON FIRST" : "NOTES"); return; }
#endif
#if !FM1B_BLE
    if (g == G_OTO) i = GV_OTO_OP(i);
#endif
    if (g == G_TR) put_sint(t, (int32_t)i - TR_MAX, 1);
    else if (g == G_CH && !i) text_cat(t, "OMNI");
    else if (g == G_CH || g == G_VOL) put_sint(t, (int32_t)i, 0);
    else if (g == G_OCH) put_sint(t, (int32_t)i + 1, 0);
    else if (g == G_OTO) text_cat(t, OP_NAME[i & 3u]);
    else if (g == G_CLK) text_cat(t, CLK_NAME[i < CLK_N ? i : 0u]);
    else if (g == G_CSRC) text_cat(t, CS_NAME[i & 3u]);
    else if (g == G_ENG) text_cat(t, EN_SHORT[i < EN_N ? i : 0u]);
    else text_cat(t, LAT[i < 3u ? i : 0u]);
}
static UI_LATE int glo_value(uint32_t g)         /* a value with its list */
{
#ifdef HYB_MULTI
    if ((g >= G_MMODE && g <= G_MVCB) || g == G_OSC || g == G_FLT || g == G_VFIX || g == G_VCRV || g == G_VSENS || g == G_DRCH
        || g == G_BOUT || g == G_CLKO)
        return 1;
#elif FM1B_BLE
    if (g >= G_UNI && g <= G_STEAL) return 1;
#endif
    return g == G_TR || g == G_CH || g == G_LAT || g == G_VOL || g == G_OCH || g == G_OTO || g == G_CLK || g == G_CSRC
        || g == G_ENG || g == G_RVQ || g == G_SBASE || g == G_SBASEB || g == G_SPCNT;
}

static UI_LATE void glo_val(uint32_t g, char *v)   /* a line's value ("" none) */
{
#ifdef HYB_SHIMMER
    if (g == G_RVQ && shx_forced()) { text_cat(v, RVQ_HNAME " SHIMMER"); return; }   /* (part A's patch: HALF
                                          * whatever REVERB says) */
#endif
    if (g == G_RVQ && RVQ_Q(set.rv_q) == RVQ_AUTO) {   /* (AUTO: the ring it runs now, rvq_tick; 1.18C42) */
        text_cat(v, "AUTO (");
        text_cat(v, rva_half ? RVQ_HNAME : RVQ_FNAME);
        text_cat(v, ")");
        return;
    }
    if (g == G_RVQ && rva_half && RVQ_Q(set.rv_q) != RVQ_HALF) {   /* (the overload guard over FULL, running HALF now:
                                          * rvq_tick) */
        text_cat(v, RVQ_HNAME " (LOAD)");
        return;
    }
#ifdef HYB_MULTI
    if (g == G_MULTI) { text_cat(v, MX_NAME[set.mx_mode < SY_M_N ? set.mx_mode : 0u]); return; }   /* (its page's summary) */
    if (g == G_ACH || g == G_BCH) {      /* (1.19.4: "OMNI>1": IN > OUT) */
        gv_text(g == G_ACH ? G_CH : G_MCHB, gv_get(g == G_ACH ? G_CH : G_MCHB), v);
        text_cat(v, ">");
        gv_text(g == G_ACH ? G_OCH : G_BOUT, gv_get(g == G_ACH ? G_OCH : G_BOUT), v + text_len(v));
        return;
    }
#endif
#ifdef HYB_VELO
    if (g == G_VELO) {                   /* (its page's summary: "NORMAL", "HARD +2", "OFF 100") */
        if (set.ve_fl & VE_OFF) { text_cat(v, "OFF "); put_sint(v + 4, set.ve_fix, 0); return; }
        gv_text(G_VCRV, set.ve_crv, v);
        if (set.ve_sens) { text_cat(v, " "); put_sint(v + text_len(v), set.ve_sens, 1); }
        return;
    }
    if (g == G_VON) { text_cat(v, set.ve_fl & VE_OFF ? "OFF" : "ON"); return; }
    if (g == G_VDRUM) { text_cat(v, set.ve_fl & VE_DRUM ? "VELOCITY" : "RAW"); return; }
    if (g == G_VCAL) {                   /* (the keys with ends of their own: "NONE", "3 KEYS") */
        uint32_t k, n = 0;
        for (k = 0; k < VE_NTOP; k++) n += set.ve_hi[k] != 0;
        if (!n) text_cat(v, "NONE");
        else { put_sint(v, (int32_t)n, 0); text_cat(v, n > 1u ? " KEYS" : " KEY"); }
        return;
    }
    if ((g >= G_VK0 && g <= G_VK6) || g == G_VNORM || g == G_VSKIP) { vg_line(g, v); return; }   /* (CALIBRATE) */
#endif
    if (glo_value(g)) gv_text(g, gv_get(g), v);
    else if (g == G_KEYL)
        text_cat(v, set.key_dark == KL_NIGHT ? "NIGHT" : set.key_dark ? "OFF" : "ON");
    else if (g == G_ASAVE)
        text_cat(v, set.md_pad & MD_AUTO ? "ON" : "OFF");
    else if ((g >= G_SPARP && g <= G_SRQ) || g == G_SDOUT) /* (GLO > SEQ's defaults, 1.14; DRUM OUT, 1.19.3) */
        text_cat(v, SQ_FON(set.sq_def, 1u << (g - G_SPARP)) ? "ON" : "OFF");
    else if (g == G_VCC)
        text_cat(v, set.vol_cc ? "SYNTH VOL" : "PART VOL");
#ifdef HYB_RV_LEND
    else if (g == G_BLE && bt_lent && !set.ble_off)
        text_cat(v, "RESTART");          /* (on at the next power-on: the reverb has its memory now) */
#endif
#ifdef HYB_MULTI
    else if (g == G_CLKL) text_cat(v, set.ck_fl & CK_FL_UNLOCK ? "OFF" : "ON");
    else if (g == G_CCSHOW) text_cat(v, set.cm_show ? "ON" : "OFF");
#endif
    else if (g == G_THRU || g == G_AUTO || g == G_BLE)
        text_cat(v, (g == G_THRU ? set.thru : g == G_AUTO ? set.ble_auto : !set.ble_off) ? "ON" : "OFF");
    else if (g >= G_IUSB && g <= G_IBT)  /* (an IN port: ON taken, OFF ignored) */
        text_cat(v, set.in_off >> (g == G_IUSB ? PORT_USB : g == G_ITRS ? PORT_TRS : PORT_BLE) & 1u ? "OFF" : "ON");
    else if (g == G_INP) {               /* (the pages' summaries: the ports in; the channel and ports out; the clock) */
        uint32_t k;
        static const uint8_t PT[3] = {PORT_USB, PORT_TRS, PORT_BLE};
        for (k = 0; k < IN_PORTS; k++)
            if (!(set.in_off >> PT[k] & 1u)) { if (v[0]) text_cat(v, " "); text_cat(v, CS_NAME[k + 1u]); }
        if (!in_off_eff()) { v[0] = 0; text_cat(v, "ALL"); }
        else if (!v[0]) text_cat(v, "NONE");
    } else if (g == G_OUT) {
#ifdef HYB_MULTI
        text_cat(v, OP_NAME[out_to_eff() & 3u]);   /* (1.19.4: the ports alone, the channels A's and B's) */
#else
        if (out_to_eff() == OP_OFF) text_cat(v, "OFF");
        else {                           /* ("10 USB+BT": the channel, the ports; no "CH", the line's room) */
            put_sint(v, set.out_ch + 1, 0);
            text_cat(v, " ");
            text_cat(v, OP_NAME[out_to_eff() & 3u]);
        }
#endif
    } else if (g == G_CLKP) {            /* ("EXT TRS": a source other than ANY, short) */
        if (set.clk == CLK_EXT && clk_src_eff() != CS_ANY) { text_cat(v, "EXT "); text_cat(v, CS_NAME[clk_src_eff() & 3u]); }
        else text_cat(v, CLK_NAME[set.clk < CLK_N ? set.clk : 0u]);
    }
    else if (g == G_SEARCH && btdev_ready())
        text_cat(v, btdev_name()[0] ? btdev_name() : set.dev_ok ? set.dev_name : "ON");
    else if (g == G_FORGET)
        text_cat(v, set.dev_ok ? set.dev_name : "NONE");
}

/* the device list (SEARCH): the device connected or connecting (REC lets it go), else those found (REC connects to
 * one; none yet: SEARCHING), then BACK */
static uint32_t gd_n(void)               /* its lines before BACK */
{
    int n = btdev_count();
    return btdev_busy() || n < 1 ? 1u : (uint32_t)n;
}
static UI_SMALL void gd_item(uint32_t i, char *t, char *pre, uint32_t *fl)
{
    char nm[21];
    (void)pre; (void)fl;
    nm[0] = 0;
    if (i >= gd_n()) text_cat(t, "BACK");
    else if (btdev_busy()) {
        text_cat(t, btdev_name()[0] ? btdev_name() : set.dev_ok ? set.dev_name : "DEVICE");
        text_cat(t, btdev_ready() ? "\tON" : "\t...");
    } else if (btdev_found((int)i, nm, 0)) {
        nm[20] = 0;
        text_cat(t, nm[0] ? nm : "MIDI DEVICE");
    } else
        text_cat(t, "SEARCHING...");
}
static int gd_idle(void) { return ui.gch == G_SEARCH && ui.gpos < gd_n() && !btdev_busy() && !btdev_count(); }   /* on
                                                     * SEARCHING: REC does nothing */
static void gd_close(void)
{
    ui.looking = ui.glook = 0;
    ui.gch = G_NONE;
    ui.gpos = 1u;                        /* (SEARCH) */
}
static void gd_rec(void)
{
    int ev;
    if (ui.gpos >= gd_n()) { gd_close(); return; }
    if (btdev_busy()) {                  /* the device: let go (no automatic attempts until the next connection) */
        btdev_save();
        ar.stop = 1;
        app_say("Device", "DISCONNECTED");
        return;
    }
    if (!btdev_count())
        return;                          /* (still looking) */
    btdev_choose(ui.gpos);
    ev = btdev_save();
    if (ev == BTDEV_CONNECTING) {
        app_say(btdev_word(ev), btdev_name());
        gd_close();
    } else
        app_say("Device", btdev_word(ev));
}

/* MIDI CC: the controllers the engine takes and what each one changes (page A: a controller sets the patch byte of its
 * number; 1 and 64 are the wheel and the pedal, 120 and 123 all off), a line each, then BACK; nothing to change */
static UI_LATE int cc_name(uint32_t c, char *t)   /* controller c's target after t; 0 = none */
{
    const u_param_t *p;
    if (c == 1u) text_cat(t, "Mod wheel (matrix)");
    else if (c == 64u) text_cat(t, "Sustain pedal");
    else if (c == 120u) text_cat(t, "All sound off");
    else if (c == 123u) text_cat(t, "All notes off");
    else if (c == 7u) text_cat(t, set.vol_cc ? "Synth volume" : "Part volume");
#ifdef HYB_MULTI
    else if (c == pv_cc_of(0)) text_cat(t, "Part A volume");   /* (VOL A / B CC: not to the engine) */
    else if (c == pv_cc_of(1)) text_cat(t, "Part B volume");
#endif
    else if ((c < 120u || (c >= 128u && c < CM_NT)) && (p = param_of(c)) != 0) p_name(p, t);   /* (1.19.6: the
                                          * parameters past the controllers' bytes too, a number given by hand) */
    else return 0;
    return 1;
}
static UI_LATE uint32_t cc_nth(uint32_t i)       /* the controller on line i (CM_NT: none) */
{
    uint32_t c;
    char t[32];
    for (c = 0; c < CM_NT; c++) {
        t[0] = 0;
        if (cc_name(c, t) && !i--) return c;
    }
    return CM_NT;
}
static UI_LATE uint32_t cc_n(void)
{
    static uint32_t n, sig;
#ifdef HYB_MULTI
    const uint32_t g = 1u | (uint32_t)set.mx_vcc[0] << 8 | (uint32_t)set.mx_vcc[1] << 16;   /* (VOL A / B CC: lines
                                          * come and go) */
#else
    const uint32_t g = 1u;
#endif
    if (g != sig) { sig = g; n = 0; while (cc_nth(n) < CM_NT) n++; }
    return n;
}
#ifdef HYB_MULTI
static UI_LATE uint32_t cc_num(uint32_t i)       /* (1.19.6) line i's number now (the one being set), 128 none */
{
    const uint32_t c = cc_nth(i);
    return cm_ed == 1u && i == ui.gpos ? cm_v : c < 128u && pm_cc_bad(c) ? c : cm_num(set.cm, c);
}
#else
#define cc_num(i) cc_nth(i)
#define cm_ed 0u
#endif
static void cc_item(uint32_t i, char *t, char *pre, uint32_t *fl)   /* "74  Cutoff" (1.19.6: the number now, "--"
                                          * none; moved: the dot), then RESET ALL, BACK */
{
    const uint32_t c = cc_nth(i), n = cc_num(i);
    (void)pre;
    if (c >= CM_NT) { text_cat(t, i > cc_n() ? "BACK" : "RESET ALL"); return; }
    if (n < 128u) put_sint(t, (int32_t)n, 0);
    else text_cat(t, "--");
    text_cat(t, "  ");
    cc_name(c, t);
    if (n != c && (n < 128u || c < 128u)) *fl |= LS_DOT;   /* (moved, or a number given) */
}
#ifdef HYB_MULTI
static UI_LATE void glo_up(void);
static UI_LATE void cm_set(uint32_t n)           /* (1.19.6) the line on the cursor to number n (128 none), said */
{
    const uint32_t c = cc_nth(ui.gpos), u = cm_put(set.cm, n, c);
    char a[48] = "", v[40] = "";
    glo_touch();                         /* (saved with GLO's settings) */
    cm_ed = 0;
    if (n < 128u) { text_cat(a, "CC"); put_sint(a + 2, (int32_t)n, 0); }
    else text_cat(a, "--");
    if (u < CM_NT) { text_cat(v, "FROM "); if (!cc_name(u, v)) v[0] = 0; }   /* (the line that lost it: "CC45" "FROM
                                          * Filt2 Env Amt"; the strip has no room for both names) */
    if (!v[0]) { text_cat(a, " > "); cc_name(c, a); }   /* ("CC74 > Cutoff") */
    app_say(a, v);
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void cm_rec(void)                 /* REC in the list: a line's number to change (learn armed), taken;
                                          * RESET ALL asked, done; BACK */
{
    const uint32_t n = cc_n();
    uint32_t c, k;
    if (cm_ed == 1u) { cm_set(cm_v); return; }
    if (ui.gpos > n) { glo_up(); return; }
    if (ui.gpos == n) {
        if (cm_ed == 2u) {
            for (k = 0; k < 128u; k++) set.cm[k] = 0;
            glo_touch();
            app_say("MIDI CC", "ALL DEFAULT");
            cm_ed = 0;
        } else cm_ed = 2u;
    } else if (!cm_line(c = cc_nth(ui.gpos))) {
        app_say("THIS LINE", "IS FIXED");
    } else {
        cm_ed = 1u;
        cm_v = (uint8_t)cm_num(set.cm, c);
    }
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void cm_turn(int32_t e)           /* SELECT while a line's number is changed: --, 0 .. 127, the numbers
                                          * never mapped skipped */
{
    int32_t v = cm_v >= 128u ? 0 : cm_v + 1, w, k = e < 0 ? -e : e;
    for (; k; k--) {
        w = v;
        do w += e < 0 ? -1 : 1; while (w > 0 && w <= 128 && pm_cc_bad((uint32_t)w - 1u));
        if (w < 0 || w > 128) break;
        v = w;
    }
    cm_v = (uint8_t)(v ? v - 1 : 128);
    ui.d_rows = ui.d_lcd = 1;
}
#endif

/* the list's lines (ui.c ls_*): a page's (the name, a tab, the value) and a value's choices */
static void glo_item(uint32_t i, char *t, char *pre, uint32_t *fl)
{
    const uint32_t g = GLO_PAGE[ui.gpage][i];
    (void)pre;
    text_cat(t, G_NAME[g]);
    if (g != G_BACK && !(g >= G_MIDI && g <= G_AUDIO) && g != G_PANIC && g != G_CC && !(g >= G_ABOUT && g <= G_TRSD)
        && g != G_KIT) {                 /* (the
                                          * MIDI pages' lines: their summary) */
        char v[24];
        v[0] = 0;
        glo_val(g, v);
        if (v[0]) { text_cat(t, "\t"); text_cat(t, v); }
    }
}
static void gv_item(uint32_t i, char *t, char *pre, uint32_t *fl)
{
    (void)pre; (void)fl;
    if (ui.gch == G_ENG) text_cat(t, EN_NAME[i < EN_N ? i : 0u]);   /* (the names in full here: the list's room) */
#ifdef HYB_MULTI
    else if (ui.gch == G_MMODE && i == SY_M_SINGLE) text_cat(t, "SINGLE MAX QUALITY");   /* (one part: all voices and
                                                     * the effects for it alone) */
#endif
    else gv_text(ui.gch, i, t);
}

static void glo_open(void)               /* GLO's button: its first page (again: the same) */
{
    if (ui.view == V_GLO && ui.gch == G_SEARCH) gd_close();
    ui.view = V_GLO;
    ui.gpage = GP_TOP;
    ui.gpos = 0;
    ui.gch = G_NONE;
    view_reset();
}
static UI_LATE void glo_up(void)                 /* HOME (BACK: REC): a value's list closed (the value as before), a page to
                                          * the one above (on its line), the first to HOME */
{
    if (ui.gch == G_SEARCH) gd_close();
#ifdef HYB_MULTI
    else if (ui.gch == G_CC && cm_ed) cm_ed = 0;   /* (1.19.6: a number being changed, RESET ALL asked: cancelled) */
#endif
    else if (ui.gch == G_CC) {
        ui.gch = G_NONE;
        ui.gpos = (uint8_t)glo_line(GP_MIDI, G_CC);
    } else if (ui.gch) {
        if (gv_get(ui.gch) != (uint32_t)(uint8_t)ui.gold) gv_set(ui.gch, (uint8_t)ui.gold);
        ui.gch = G_NONE;
    } else if (ui.gpage != GP_TOP) {     /* (up to the page above, on this page's line) */
        const uint32_t p = ui.gpage;
        ui.gpage = GLO_UP[p];
        ui.gpos = (uint8_t)glo_line(ui.gpage, GLO_OPEN[p]);
    } else {
        qa_go(0);
        return;
    }
    view_reset();
}
static UI_LATE void kit_open(void);
static UI_LATE void sq_def_sync(void);
static UI_LATE void glo_rec(void)                /* REC on the cursor's line */
{
    uint32_t g;
    if (ui.gch == G_SEARCH) { gd_rec(); return; }
    if (ui.gch == G_CC) {                /* (BACK: up; 1.19.6: a line's number, RESET ALL: cm_rec) */
#ifdef HYB_MULTI
        cm_rec();
#else
        if (ui.gpos >= cc_n()) glo_up();
#endif
        return;
    }
    if (ui.gch) { ui.gch = G_NONE; return; }   /* (the value kept) */
    g = GLO_PAGE[ui.gpage][ui.gpos];
    if (glo_page_of(g) != GP_TOP) {      /* (a page) */
        ui.gpage = (uint8_t)glo_page_of(g);
        ui.gpos = 0;
#ifdef HYB_VELO
        if (ui.gpage == GP_VCAL) ve_cal_clear();   /* (CALIBRATE: nothing struck yet) */
#endif
#ifdef HYB_VELO
    } else if (g == G_VON || g == G_VDRUM) {   /* (VELOCITY ON / OFF, DRUM ACC RAW / VELOCITY) */
        set.ve_fl ^= (uint8_t)(g == G_VON ? VE_OFF : VE_DRUM);
        glo_touch();
    } else if ((g >= G_VK0 && g <= G_VK6) || g == G_VSKIP || g == G_VNORM || g == G_VRST) {   /* (CALIBRATE's) */
        vg_rec(g);
        return;
#endif
    } else if (g >= G_VER && g <= G_TRSD) {   /* (ABOUT's lines: text only) */
        return;
    } else if (g == G_BACK) {
        glo_up();
        return;
    } else if (g == G_KIT) {             /* (the drum kit's page) */
        kit_open();
        return;
    } else if (glo_value(g)) {
        ui.gch = (uint8_t)g;
        ui.gold = (int8_t)gv_get(g);
    } else if (g == G_THRU) {
        set.thru = (uint8_t)!set.thru;
        glo_touch();
#ifdef HYB_MULTI
    } else if (g == G_CLKL) {            /* (1.19.5) CLK LOCK: ck_lock takes it at its next block */
        set.ck_fl ^= CK_FL_UNLOCK;
        glo_touch();
    } else if (g == G_CCSHOW) {          /* CC SHOW: the audio interrupt reads it at the next controller */
        set.cm_show = (uint8_t)!set.cm_show;
        glo_touch();
#endif
    } else if (g == G_KEYL) {
        set.key_dark = (uint8_t)(set.key_dark == KL_ON ? KL_NIGHT : set.key_dark == KL_NIGHT ? KL_OFF : KL_ON);   /* (ON,
                                          * NIGHT, OFF: 1.18C43) */
        glo_touch();
    } else if ((g >= G_SPARP && g <= G_SRQ) || g == G_SDOUT) {   /* (GLO > SEQ's defaults, 1.14: no pattern saved
                                          * changes; 1.19.3: DRUM OUT, no pattern's) */
        set.sq_def ^= (uint8_t)(1u << (g - G_SPARP));
        sq_def_sync();
        glo_touch();
    } else if (g == G_ASAVE) {           /* (AUTO SAVE: itself saved at once, alone; the rest as the rule says) */
        settings_t s;
        set.md_pad ^= MD_AUTO;
        settings_load(&s);
        s.md_pad = (uint8_t)((s.md_pad & ~MD_AUTO) | (set.md_pad & MD_AUTO));
        if (store_save(OBJ_SETTINGS, &s, sizeof s) != 0) app_say("Save", "FAILED");
    } else if (g == G_VCC) {
        set.vol_cc = (uint8_t)!set.vol_cc;
        glo_touch();
    } else if (g >= G_IUSB && g <= G_IBT) {
        set.in_off ^= (uint8_t)(1u << (g == G_IUSB ? PORT_USB : g == G_ITRS ? PORT_TRS : PORT_BLE));
        ev_put(0xB0u, 123u, 0u);         /* (a port off: its notes off, their note-offs no longer come) */
        glo_touch();
    } else if (g == G_AUTO) {
        set.ble_auto = (uint8_t)!set.ble_auto;
        if (set.ble_auto) { ar.stop = 0; ar.fails = 0; ar.next = plat_ms(); }   /* (on: an attempt soon) */
        glo_touch();
    } else if (g == G_BLE) {
        ble_toggle();
    } else if (g == G_PANIC) {
        glo_panic();
    } else if (g == G_FORGET) {          /* the saved device: no more reconnecting to it, and the radio's pairing
                                          * records cleared */
        plat_ble_forget();               /* (the radio's pairing records too: a device that forgot its bond
                                          * otherwise stalls every reconnection) */
        app_say("Device", set.dev_ok ? "FORGOTTEN" : "PAIRINGS CLEARED");
        if (set.dev_ok) {
            uint32_t k;
            for (k = SET_LEN_V1; k < __builtin_offsetof(settings_t, qa_ok); k++)
                ((uint8_t *)&set)[k] = 0;
            glo_touch();
        }
    } else if (g == G_CC) {
        ui.gch = G_CC;
        ui.gpos = 0;
    } else if (g == G_SEARCH) {
        if (!FM1B_BLE_CENTRAL || set.ble_off || plat_ble_state() != BLE_READY) {
            app_say("Bluetooth", set.ble_off ? "OFF" : "NOT READY");
            return;
        }
        if (ar.trying) plat_ble_disconnect();   /* (an automatic attempt: given up, the scan instead) */
        ar.stop = 1;                     /* (looking by hand: no automatic attempts until the next connection) */
        ui.looking = ui.glook = 1;
        ui.gch = G_SEARCH;
        ui.gpos = 0;
    }
    view_reset();
}
static int glo_choosing(void) { return ui.view == V_GLO && ui.gch && ui.gch != G_SEARCH && ui.gch != G_CC; }
static void glo_select(int32_t e)        /* SELECT (K1 in a value's list) */
{
    if (glo_choosing()) {
        int32_t i = clamp((int32_t)gv_get(ui.gch) + e, 0, (int32_t)gv_n(ui.gch) - 1);
        if ((uint32_t)i != gv_get(ui.gch)) gv_set(ui.gch, (uint32_t)i);
    } else if (ui.gch == G_CC) {
#ifdef HYB_MULTI
        if (cm_ed == 1u) { cm_turn(e); return; }
        cm_ed = 0;
#endif
        ui.gpos = (uint8_t)clamp((int32_t)ui.gpos + e, 0, (int32_t)cc_n() + 1);
    } else if (ui.gch == G_SEARCH) {
        ui.gpos = (uint8_t)clamp((int32_t)ui.gpos + e, 0, (int32_t)gd_n());
        if (ui.gpos < gd_n() && !btdev_busy() && btdev_count()) btdev_choose(ui.gpos);
    } else
        ui.gpos = (uint8_t)clamp((int32_t)ui.gpos + e, 0, (int32_t)GLO_PN[ui.gpage] - 1);
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void bi_about(char *v);   /* (splash.c: the boot image's check, GLO > ABOUT's last line) */
static UI_LATE void flash_about(char *v) /* (1.12) the flash chip the boot found, by its JEDEC id: "FLASH 856014" (the
                                          * Puya P25Q80 of the first FM-1s), "FLASH B36014" (the UC25HQ80 of newer
                                          * ones; both save), " RAM ONLY" after one fm1-base does not know (nothing
                                          * is kept); after the version (the USB build) "FL B36014", " RAM" (v: 24
                                          * bytes) */
{
    static const char hex[] = "0123456789ABCDEF";
    const uint32_t id = plat_flash_id();
    const int after = v[0] != 0;
    char *e;
    uint32_t k;
    text_cat(v, after ? "FL " : "FLASH ");
    e = v + text_len(v);
    for (k = 0; k < 6u; k++) e[k] = hex[id >> (20u - 4u * k) & 15u];
    e[6] = 0;
    if (!plat_store_ok()) text_cat(v, after ? " RAM" : " RAM ONLY");
}
static UI_LATE void about_diag(uint32_t g, char *v)   /* GLO > ABOUT's lines without a text of their own: diagnoses,
                                          * refreshed each second (app_frame). BACK (after 1.05): the USB audio input's
                                          * glitches since power-on (plat_usb_audio_stats), "USB U0 O0 M0 S0": U
                                          * underruns (a hold, a click in the recording), O overruns (a block dropped),
                                          * M IN tokens with no packet (a flash write), S the computer's stops (> 20 ms),
                                          * each up to 999 (v: 24 bytes); DIAGNOSIS (after 1.08): the voices, keys, HOLD
                                          * (below); the others (0.84): the battery's reading as the levels see it,
                                          * the latest single one, the level (4: a USB host). All of them: gui-spec.md
                                          * 8, ABOUT */
{
    if (g == G_TRSD) {                   /* (1.19.8) "TRS B12 M4 D0 E9": the TRS MIDI jack since power-on: the bytes
                                          * received, the messages made of them, those dropped (the queue full or a
                                          * lapped ring), the UART's receive events (plat_trs_stats); counts mod 1000,
                                          * drops up to 99: a note on shows B+3 M+1 at once, its off the same */
        uint32_t c[4];
        plat_trs_stats(c);
        text_cat(v, "TRS B");
        put_sint(v + text_len(v), (int32_t)(c[0] % 1000u), 0);
        text_cat(v, " M");
        put_sint(v + text_len(v), (int32_t)(c[1] % 1000u), 0);
        text_cat(v, " D");
        put_sint(v + text_len(v), (int32_t)(c[2] > 99u ? 99u : c[2]), 0);
        text_cat(v, " E");
        put_sint(v + text_len(v), (int32_t)(c[3] % 1000u), 0);
        return;
    }
    if (g == G_DIAG) {                   /* (after 1.08) "V9/12 G6 K2 H5 Q0 S2": the voices on (sounding, releases too) of the
                                          * limit now (the governor's, both parts'), those gated (held by a key or
                                          * HOLD), the keys down, the notes HOLD keeps, the messages to the engine
                                          * dropped (its queue full) since power-on, the voices the overload guard
                                          * shed (none: not shown) */
        uint32_t k, on = 0, gt = 0, h = 0;
#ifdef HYB_MULTI
        for (k = 0; k < (uint32_t)S.nv; k++) { on += S.v[k].on; gt += S.v[k].on && S.v[k].gate; }   /* (the parts' slices) */
        for (k = 0; k < (uint32_t)SB.nv; k++) { on += SB.v[k].on; gt += SB.v[k].on && SB.v[k].gate; }
#else
        for (k = 0; k < SY_MAX_VOICES; k++) { on += S.v[k].on; gt += S.v[k].on && S.v[k].gate; }
#endif
        for (k = 0; k < 128u; k++) h += hold_note(k) + HOLD_PO(k);   /* (1.15: the other part's too) */
        text_cat(v, "V");
        put_sint(v + 1, (int32_t)on, 0);
        text_cat(v, "/");
        put_sint(v + text_len(v), GOV_MAX, 0);
        text_cat(v, " G");
        put_sint(v + text_len(v), (int32_t)gt, 0);
        text_cat(v, " K");
        put_sint(v + text_len(v), __builtin_popcount(ui.keys), 0);
        text_cat(v, " H");
        put_sint(v + text_len(v), (int32_t)h, 0);
        text_cat(v, " Q");
        put_sint(v + text_len(v), ev_drop, 0);
        if (grd_n && text_len(v) <= 19u) {   /* (after 1.18C30) " S4": the voices the overload guard shed (rvq_tick), up
                                          * to 99, while the line has room (v: 24 bytes) */
            text_cat(v, " S");
            put_sint(v + text_len(v), grd_n > 99u ? 99 : (int32_t)grd_n, 0);
        }
#if GOV_BUD && GOV_FIX
        if (bud_n && text_len(v) <= 19u) {   /* (1.18C49) " T7": the voices the budget faded early (app_render), up to
                                          * 99, while the line has room */
            text_cat(v, " T");
            put_sint(v + text_len(v), bud_n > 99u ? 99 : (int32_t)bud_n, 0);
        }
#endif
        return;
    }
    if (g == G_BACK) {                   /* (1.18C35) "U0 O0 M0 S0 A1 F70-140": the four glitch counts, then the
                                          * host's stream starts (A, each a re-prime: a short silence) and the ring's
                                          * fill range since the last start (F, frames; healthy within ~40..180), each
                                          * while the line has room (v: 24 bytes). (usb-audio2) Every other 2 s
                                          * "G2 W45 X2 P0" instead (plat_usb_audio_more): G the host's gaps (a packet
                                          * it took >= 2.5 ms late), W the longest wait since the last start (ms), X
                                          * the skips after a gap (crossfaded), P the overruns while the host read
                                          * (the device's own feed ahead: should stay 0); an overrun (O) now counts a
                                          * full ring once, faded, not each block dropped */
        static const char K[2][5] = {{'U', 'O', 'M', 'S', 'A'}, {'G', 'W', 'X', 'P', 0}};
        const uint32_t b = plat_ms() >> 11 & 1u;
        uint32_t c[8], k;
        if (b) {
            plat_usb_audio_more(c);
        } else {
            plat_usb_audio_stats(c);
            plat_usb_audio_stream(c + 4);
        }
        for (k = 0; k < 5u && K[b][k]; k++) {
            char *e = v + text_len(v);
            if (k == 4u && text_len(v) > 18u) return;
            if (k) *e++ = ' ';
            e[0] = K[b][k];
            e[1] = 0;
            put_sint(e + 1, (int32_t)(c[k] > 999u ? 999u : c[k]), 0);
        }
        if (!b && c[6] && text_len(v) <= 14u) {
            text_cat(v, " F");
            put_sint(v + text_len(v), (int32_t)c[5], 0);
            text_cat(v, "-");
            put_sint(v + text_len(v), (int32_t)(c[6] > 999u ? 999u : c[6]), 0);
        }
        return;
    }
    text_cat(v, "BATT ");
    put_sint(v + 5, plat_battery_raw(0), 0);
    text_cat(v, " ");
    put_sint(v + text_len(v), plat_battery_raw(1), 0);
    text_cat(v, " L");
    put_sint(v + text_len(v), plat_battery(), 0);
}
static void glo_lcd(char *a, char *v)    /* the LCD strip in GLO: the page and the line's value; a value's name and
                                          * the value chosen; the device */
{
    if (ui.gch == G_SEARCH) {
        if (btdev_busy()) {
            text_cat(a, btdev_name()[0] ? btdev_name() : set.dev_ok ? set.dev_name : "DEVICE");
            text_cat(v, btdev_ready() ? "CONNECTED" : "CONNECTING");
        } else {
            text_cat(a, "DEVICES");
            if (!btdev_count()) text_cat(v, "SEARCHING");
        }
    } else if (ui.gch == G_CC) {
        const uint32_t n = ui.gpos < cc_n() ? cc_num(ui.gpos) : 129u;
#ifdef HYB_MULTI
        if (cm_ed == 2u) { text_cat(a, "RESET ALL?"); text_cat(v, "REC: YES"); } else
#endif
        text_cat(a, cm_ed == 1u ? "LEARN: SEND CC" : "MIDI CC");
        if (n < 128u) { text_cat(v, "CC "); put_sint(v + 3, (int32_t)n, 0); }
        else if (n == 128u) text_cat(v, "--");
        else if (!cm_ed) text_cat(v, ui.gpos > cc_n() ? "BACK" : "RESET ALL");
    } else if (ui.gch) {
        text_cat(a, G_NAME[ui.gch]);
        gv_text(ui.gch, gv_get(ui.gch), v);
    } else {
        const uint32_t g = GLO_PAGE[ui.gpage][ui.gpos];
        text_cat(a, GLO_TITLE[ui.gpage]);
        glo_val(g, v);
#if !FM1B_BLE
        if (g == G_VER) text_cat(v, HYB_VERSION HYB_RVTAG HYB_EDITION " ");   /* (the strip's room: "ABOUT" and the version) */
#endif
#ifdef HYB_RV_LEND
        if (g == G_VER) text_cat(v, bt_lent ? "BT>SHIM " : "BT ON ");   /* (1.14) Bluetooth's memory: lent to the
                                          * shimmer and REVERB FULL (Bluetooth off at power-on) or the radio's */
#endif
        if (g == G_VER) flash_about(v);  /* (1.12: the flash chip, after the version in the USB build) */
        if (g == G_KOFI) bi_about(v);    /* (after 0.99, a diagnosis: the boot image as the power-on found it) */
#ifdef HYB_VELO
        if (ui.gpage == GP_VCAL) { a[0] = v[0] = 0; ve_cal_lcd(a, v); }   /* (CALIBRATE: the key struck last) */
#endif
        if (ui.gpage == GP_ABOUT && !v[0]) about_diag(g, v);
        if (!v[0]) text_cat(v, G_NAME[g]);
    }
}

static int qa_rec(void)                  /* REC in the lists and on EDIT / BACK: 1 = taken (no Bluetooth then) */
{
    if (ui.view == V_GLO) {
        glo_rec();
        ui.d_rows = ui.d_lcd = 1;
        return 1;
    }
    if (ui.view == V_PLAYED) {
        played_rec();
        return 1;
    }
    if (ui.view == V_MENU || ui.view == V_GROUP) {
        int32_t it = rl_item(ui.pos);
        if (it == RL_EDIT) glist_go(0);
        else if (it == RL_BACK) menu_up();
        else return 0;
        return 1;
    }
    if (ui.view == V_GLIST) {
        if (ui.gcur >= U_MENU[ui.menu].ng) menu_up();
        else group_go(menu_g0(ui.menu) + ui.gcur, 0);
        return 1;
    }
    if (ui.view == V_QUICK && ui.qpos == qa_count() + 1u) {   /* EDIT: the list's slots */
        ui.view = V_LIST;
        ul.cur = 0;
        ul.choosing = 0;
    } else if (ui.view == V_LIST && ul.cur >= QA_SLOTS) {     /* BACK */
        qa_go(QA_SLOTS + 1u);
        return 1;
    } else if (ui.view == V_LIST && !ul.choosing) {           /* a slot: the blocks to choose from */
        ul.choosing = 1;
        ul.ch = set.qa[ul.cur] == QA_EMPTY ? -1 : (int8_t)set.qa[ul.cur];
    } else if (ui.view == V_LIST) {                           /* the block chosen: kept, and saved */
        set.qa[ul.cur] = ul.ch < 0 ? QA_EMPTY : (uint8_t)ul.ch;
        ul.choosing = 0;
        if (qa_save() == 0) app_say("Quick list", "SAVED");
        else app_say("Save", "FAILED");
    } else
        return 0;
    ui.d_rows = ui.d_lcd = 1;
    return 1;
}

static UI_LATE void kit_up(void);
static void qa_home(void)                /* HOME: back a step in the menus; anywhere else, the HOME screen */
{
    if (ui.view == V_LIST && ul.choosing) {   /* the slot as it was */
        ul.choosing = 0;
        ui.d_rows = ui.d_lcd = 1;
    } else if (ui.view == V_LIST)
        qa_go(QA_SLOTS + 1u);            /* (EDIT) */
    else if (ui.view == V_GROUP || ui.view == V_GLIST)
        menu_up();
    else if (ui.view == V_GLO)
        glo_up();
    else if (ui.view == V_KIT)
        kit_up();
    else
        qa_go(0);
}

static void qa_select(int32_t e)         /* SELECT */
{
    if (ui.view == V_GLO) {
        glo_select(e);
        return;
    }
    if (ui.view == V_QUICK) {
        int32_t p = clamp((int32_t)ui.qpos + e, 0, (int32_t)qa_count() + 1);
        if (p != ui.qpos) qa_go((uint32_t)p);
        return;
    }
    if (ui.view == V_MENU || ui.view == V_GROUP) {
        int32_t p = clamp((int32_t)ui.pos + e, 0, (int32_t)rl_n() - 1);
        if (p != ui.pos) rl_go((uint32_t)p);
        return;
    }
    if (ui.view == V_GLIST) ui.gcur = (uint8_t)clamp((int32_t)ui.gcur + e, 0, (int32_t)U_MENU[ui.menu].ng);
    else if (ul.choosing) ul.ch = (int8_t)clamp(ul.ch + e, -1, (int32_t)U_NBLK - 1);   /* EMPTY, then the blocks */
    else ul.cur = (uint8_t)clamp((int32_t)ul.cur + e, 0, (int32_t)QA_SLOTS);
    ui.d_rows = ui.d_lcd = 1;
}

/* the lists' lines (ui.c ls_*) */
static void qa_item(uint32_t i, char *t, char *pre, uint32_t *fl)   /* the quick-access slots, BACK */
{
    if (i >= QA_SLOTS) { text_cat(t, "BACK"); return; }
    pre[0] = (char)('1' + i); pre[1] = 0;
    if (set.qa[i] == QA_EMPTY) *fl |= LS_EMPTY;
    else text_cat(t, U_ROW[set.qa[i]].tag);
}
static void ch_item(uint32_t i, char *t, char *pre, uint32_t *fl)   /* a slot's choice: EMPTY, the blocks */
{
    (void)pre; (void)fl;
    if (!i) { text_cat(t, "EMPTY"); return; }
    text_cat(t, U_ROW[i - 1u].tag);
}
static void gl_item(uint32_t i, char *t, char *pre, uint32_t *fl)   /* a menu's EDIT groups, BACK */
{
    (void)pre; (void)fl;
    text_cat(t, i < U_MENU[ui.menu].ng ? U_GRP[menu_g0(ui.menu) + i].name : "BACK");
}
static UI_LATE void mxl_item(uint32_t i, char *t, char *pre, uint32_t *fl)   /* (0.92) the matrix's slots: the
                                          * group's name, a light (lit: in use), the destination's short name */
{
    char nm[12];
    gl_item(i, t, pre, fl);
    if (i >= MX_N) return;
    *fl |= LS_LED;
    if (mx_slot(i, nm)) {
        *fl |= LS_LIT;
        text_cat(t, "\t");
        text_cat(t, nm);
    }
}
static void vl_item(uint32_t i, char *t, char *pre, uint32_t *fl)   /* a parameter's values */
{
    (void)pre; (void)fl;
    u_vraw = 1;                          /* (the list: the modes themselves) */
    value_text(ui.vl_p, i, t);
    u_vraw = 0;
}

/* K1..K4 feel like a pot: turned slowly, a value a detent; turned fast, the screen's knob turns as far as the real one
 * (its 270 degrees are 0..127, so a turn of KNOB_DETENTS detents moves 127 x 360 / 270). Between the two the step
 * follows the speed on an S curve (no corner to feel), the speed itself smoothed over ~80 ms (one uneven detent does
 * not jump), the fractions carried so nothing is lost */
#ifndef KNOB_DETENTS
#define KNOB_DETENTS 20                  /* detents per turn of the FM-1's encoders (counted on a device) */
#endif
#define KNOB_SLOW 3.f                    /* detents per second: up to here one value a detent */
#define KNOB_FAST 20.f                   /* from here the screen's knob follows the real one */
static int32_t knob_steps(uint32_t k, int32_t e)
{
    static float speed[7], frac[7];      /* (K1..K4, ALGO; in PLAY mode SELECT and PRESET too: play_turn) */
    static uint32_t last_t[7];
    const float sync = 127.f * 360.f / (270.f * (float)KNOB_DETENTS);   /* values a detent, 1:1 */
    uint32_t now = plat_ms(), dt = now - last_t[k];
    float inst, a, x, g, d;
    int32_t n;
    last_t[k] = now;
    inst = (float)(e < 0 ? -e : e) * 1000.f / (float)(dt < 5u ? 5u : dt);
    if (dt > 250u) {                     /* a new turn: it starts slow */
        speed[k] = 0.f;
        frac[k] = 0.f;
        inst = 0.f;
    }
    a = (float)dt * (1.f / 80.f);        /* the smoothing: the share of the new reading after dt ms */
    if (a > 1.f) a = 1.f;
    speed[k] += a * (inst - speed[k]);
    x = (speed[k] - KNOB_SLOW) * (1.f / (KNOB_FAST - KNOB_SLOW));
    x = x < 0.f ? 0.f : x > 1.f ? 1.f : x;
    g = 1.f + (sync - 1.f) * x * x * (3.f - 2.f * x);
    d = (float)e * g + frac[k];
    n = (int32_t)d;                      /* (towards zero; the rest carried) */
    frac[k] = d - (float)n;
    if (n == 0) {                        /* a detent always moves at least one value */
        n = e > 0 ? 1 : -1;
        frac[k] = 0.f;
    }
    return n;
}
#define LS_SETTLE 200u                   /* (1.18C35) a fast turn of a list rested this long: settled (fv_idle) */
/* (1.18C35) a list's cursor i turned by e detents through its section lo .. hi (a bank, the favorites) as K1..K4 turn
 * (knob_steps, encoder k): slow, a line a detent; fast, several, never past the section's ends: there it stops, and
 * from there (or from outside the section: INIT, RANDOMIZE, ...) a line a detent until the turn pauses (as
 * knob_steps: 250 ms). A fast turn (ui.ls_fly) loads nothing on its way: the line it settles on, LS_SETTLE after its
 * last detent */
static UI_LATE int32_t ls_acc(uint32_t k, int32_t i, int32_t e, int32_t lo, int32_t hi)
{
    const uint32_t now = plat_ms();
    const int32_t n = knob_steps(k, e);
    if (now - ui.ls_t > 250u) ui.ls_end = 0;   /* (a new turn) */
    ui.ls_t = now;
    if (ui.ls_end || i < lo || i + e < lo || i + e > hi) { ui.ls_end = 1; return i + e; }
    if (i + n < lo || i + n > hi) ui.ls_end = 1;
    if (n != e || ui.ls_fly) ui.ls_fly = (uint16_t)(now | 1u);
    return clamp(i + n, lo, hi);
}

/* a parameter turned (K1..K4; ALGO: the tempo) by e detents: its byte to the engine, the edit memory and the undo (a
 * new turn: a new step); its list of names up over the rows while it turns. k: the encoder (knob_steps), col: its
 * column in the lit row (framed), -1 none */
/* a knob turned on the lit row draws its column again, nothing else, unless the turn changes more than its column:
 * the shape (wave / PW's label), the delay / reverb mode (FX's blocks after DLY/REV, TIME / PREDLY's label, the
 * block's count in the LCD strip), a byte the row below shows too, a matrix assign (the names below), a value's font
 * changing its neighbour's (draw: row_big) */
static struct { int16_t it, dim; int8_t hl; uint8_t big; } lit = {-1, -1, -1, 0};   /* the rows as drawn */
static int col_ok(uint32_t by)
{
    uint32_t k;
    if (lit.it < 0 || by == 17u || by == 22u || by == 112u || (ui.view == V_GROUP && ui.menu == M_MTX))
        return 0;
    if (lit.dim >= 0)
        for (k = 0; k < 4u; k++)
            if (U_ROW[lit.dim].p[k].label[0] && P_BYTE0(&U_ROW[lit.dim].p[k]) == by) return 0;
    return 1;
}

static UI_LATE int pv_col_ok(uint32_t by)        /* PLAY mode: a turn of byte by changes its control's box alone (not a label
                                          * elsewhere: the shapes' wave / PW, the delay / reverb mode's TIME / PREDLY;
                                          * not a byte two controls show) */
{
    uint32_t c, n = 0;
    if (by == 17u || by == 22u || by == 112u)
        return 0;
    for (c = 0; c < PC_N; c++)
        n += pv.p[c] && p_byte(pv.p[c]) == by;
    return n <= 1u;
}

static uint8_t o3_last = 2u;             /* osc 3's mode before it was switched off (saw when none) */
/* a list knob (0.91): the first detent after a pause shows the parameter's list of values alone, its value as it
 * is; the detents while that list is up change it. Armed = the list up for this parameter (ui.vl_p, ui.vl_until):
 * the list's time over, another knob turned or a button that closes it (overlay_off) and the next turn shows again.
 * The numbers (no list) change at once as ever; MIDI never comes here. 1: the turn's first detent taken */
static UI_LATE int vl_first(const u_param_t *p, int32_t col, uint32_t now)
{
    if (!p_list(p) || (ui.vl_until && now < ui.vl_until && ui.vl_p == p))
        return 0;
    ui.plist_until = 0;                  /* (the patch list away: this list instead) */
    ui.vl_p = p;
    ui.vl_until = now + PLIST_MS;
    ui.lastp = p;
    ui.last = col;
    ui.edit_t = now | 1u;
    ui.d_lcd = ui.d_rows = 1;
    if (ui.view == V_QUICK && !ui.qpos) home_sig[0] = 0;
    return 1;
}
/* the edit buffer's byte by to v, as an edit (the panel's knobs, the remote editor's: remote.c): an undo step (the same
 * byte turned again within UNDO_GAP_MS: the same step), the edit memory (SAVE's light), the engine (its queue; the
 * extension and the shimmer's targets: shx_sync; the name, 240..249: the panel's alone) */
static UI_LATE void edit_set(uint32_t by, uint32_t v, uint32_t now)
{
#ifdef HYB_MULTI
    if (by == TP_BYTE) {                 /* (1.18: the global tempo, either part's knob or the remote editor: both
                                          * parts' panel bytes, the engines, GLO's settings to save; no undo step, no
                                          * edit memory: no patch holds it) */
        shadow[by] = shadow_o[by] = (uint8_t)v;
        tp_run = (uint8_t)(v + 1u);
        ev_put(EV_PARAM, by, v);
        glo_touch();
        return;
    }
#endif
    if (!undo_n || undo[undo_n - 1u].byte != by || undo[undo_n - 1u].patch != set.patch
        || now - undo_t > UNDO_GAP_MS)   /* (a new turn: a new step) */
        undo_push(set.patch, by, shadow[by]);
    undo_t = now;
    shadow[by] = (uint8_t)v;
    em_set(set.patch, by, v);
#ifdef HYB_SHIMMER
    if (by >= 256u || shx_ri(by) >= 0) shx_sync();   /* (the extension, or a target: part A's to the engine) */
    if (by < 256u)
#endif
    if (by < 240u || by >= 250u)
        ev_put(EV_PARAM, by, SHX_EB(by, v));
    if (by == TP_BYTE) tp_run = (uint8_t)(v + 1u);   /* (shown at once; the audio interrupt sets it too) */
    if (by == 128u + 41u && v) o3_last = (uint8_t)v;
}
static void param_turn(const u_param_t *p, uint32_t k, int32_t e, int32_t col)
{
    uint32_t by = p_byte(p), mx = p_max(p), now = plat_ms();
    int32_t v;
#ifdef HYB_MULTI
    if (p_parta(p)) { app_say("CHANGE IN", "PART A"); return; }   /* (after 0.98: part A's delay / reverb, ui.c) */
#ifdef HYB_SHIM_LEND
    if (P_BTOFF(p)) { app_say("SHIMMER:", "BT OFF"); return; }   /* (1.04: Bluetooth on, ui.c) */
#endif
#endif
    st_busy_t = now;                     /* (a knob: busy, store_wait) */
    if (vl_first(p, col, now)) {         /* (a list: its first detent shows it, the rest change the value) */
        e += e > 0 ? -1 : 1;
        if (!e) return;
    }
    if (p->kind == V_O3ON) {             /* (on: the mode it had) */
        if (e < 0 && shadow[by]) o3_last = shadow[by];
        v = e > 0 ? (shadow[by] ? shadow[by] : o3_last) : 0;
    } else if (by == TP_BYTE && tp_run)  /* (the tempo: from the one running, a patch's own or kept, tp_run) */
        v = clamp((int32_t)tp_run - 1 + knob_steps(k, e), 0, (int32_t)mx);
#ifdef HYB_SHIMMER
    else if (P_RT(p))                    /* (a destination: its list, then the shimmer's targets) */
        v = (int32_t)rt_byte(p, (uint32_t)clamp((int32_t)rt_idx(p, shadow[by]) + knob_steps(k, e), 0, (int32_t)mx));
#endif
    else
        v = clamp((int32_t)(shadow[by] > mx ? mx : shadow[by]) + knob_steps(k, e), 0, (int32_t)mx);
    if (v != shadow[by] || (by == TP_BYTE && tp_run && v != tp_run - 1)) {
        edit_set(by, (uint32_t)v, now);
    } else if (p_list(p)) {              /* (the list up, the value at its end: its time again, nothing to draw) */
        ui.vl_until = now + PLIST_MS;
        ui.edit_t = now | 1u;
        return;
    }
    if (ui.plist_until) overlay_off();   /* (the patch list away) */
    if (p_list(p)) {
        ui.vl_p = p;
        ui.vl_until = now + PLIST_MS;
    } else if (ui.vl_until)
        overlay_off();
    ui.lastp = p;
    ui.last = col;
    ui.edit_t = now | 1u;                /* (never 0) */
    ui.d_lcd = 1;
    if (ui.view == V_PLAY) {             /* PLAY mode: its control's box alone (pv_col_ok) */
        if (!p_list(p) && pv_col_ok(by)) ui.d_col = 1;
        else ui.d_rows = 1;
    } else if (col >= 0 && !p_list(p) && col_ok(by)) ui.d_col = 1;   /* (its column alone) */
    else ui.d_rows = 1;
    if (ui.view == V_QUICK && !ui.qpos) home_sig[0] = 0;   /* (HOME: its BPM) */
}

/* PLAY mode (1.18C43 the development notes): PLAY held PM_HOLD_MS turns it on (pm_on, its page
 * V_PLAY) and off again; while it is on it is the HOME screen: every way back to HOME comes to its page (pm_home), the
 * other pages (the menus, GLO, the sequencers, the launcher, SAVE, the lists) as before. SHIFT + PLAY on its page
 * opens its assignment editor (V_PLAYED), again back (until 1.18C42 SHIFT + PLAY turned it on, the editor, off). On
 * its page the seven encoders K1..K4, SELECT, PRESET and ALGO each turn one parameter, as the menus' knobs do
 * (param_turn: the edit memory, undo, the value list), PRESET with SHIFT (1.18C43: PRESET alone the patch, as on
 * HOME: the patch list over HOME's screen, then the page again); ALGO no tempo unless assigned it; the keys play,
 * PLAY starts and stops the transport as everywhere. Not kept over a power-off. The
 * assignments are global, saved with GLO's settings (glo_touch); a control not assigned takes its default:
 * K1 cutoff, K2 resonance, K3 amp attack, K4 amp release, SELECT the smart choice, PRESET LFO 1's rate, ALGO LFO 2's.
 * The smart choice (play_smart, taken when PLAY mode starts and at a patch change in it): the first the patch uses of
 * osc 1's PW (its shape on the square side), the FM amount (above 0), sync (on); else the PW */
#define pl_tab sq_ush.pl                 /* the parameters to choose from (the editor's list), in the menus' order:
                                          * U_ROW's row x 4 + knob, each byte once */
static uint32_t pl_n;
static const u_param_t *pl_p(uint32_t i) { return &U_ROW[pl_row(pl_tab[i])].p[pl_tab[i] & 3u]; }
static UI_LATE void pl_add_row(uint32_t r)
{
    uint32_t k, i;
    if (r == ROW_OUT) return;            /* (1.18C48: LEVEL's bytes again, not counted by PL_ENC: ui.c) */
    for (k = 0; k < 4u; k++) {
        const u_param_t *p = &U_ROW[r].p[k];
        if (!p->label[0])
            continue;
#ifdef HYB_SHIMMER
        if (P_EXT(p)) continue;          /* (EDIT > SHIMMER's, 0.91: the extension's, not a patch byte; as the
                                          * SHIMMER block, not in PLAY mode's list) */
#endif
        for (i = 0; i < pl_n && pl_p(i)->byte != p->byte; i++)
            ;
        if (i == pl_n) pl_tab[pl_n++] = PL_ENC(r, k);
    }
}
static UI_LATE void pl_build(void)               /* (once) each menu's blocks as it shows them, then its EDIT groups' rows */
{
    static const uint8_t FX_ALL[8] = {2, 0, 1, 3, 4, 5, 6, ROW_LEVEL};   /* FX: DLY/REV, the delay's and the reverb's
                                                     * blocks, PHASER, CHORUS, DIST, LEVEL */
    uint32_t m, i, g;
    if (pl_n)
        return;
    for (m = 0; m < U_NMENU; m++) {
        for (i = 0; i < (m == M_FX ? sizeof FX_ALL : U_MENU[m].nb); i++)
            pl_add_row(m == M_FX ? FX_ALL[i] : m == M_SEL2 ? SEL2_BLK[i] : menu_b0(m) + i);
        for (g = menu_g0(m); g < menu_g0(m) + U_MENU[m].ng; g++)
            for (i = 0; i < U_GRP[g].nr; i++)
                pl_add_row(group_r0(g) + i);
    }
}
static UI_LATE int32_t pl_find(uint32_t byte)    /* the list's entry on this byte, -1 none */
{
    uint32_t i;
    pl_build();
    for (i = 0; i < pl_n; i++)
        if (pl_p(i)->byte == byte)
            return (int32_t)i;
    return -1;
}
static const u_param_t P_PW1 = {18, V_NUM, 0, 0, "PW", "Osc1 PW"};   /* (the smart choice: osc 1's PW on either side
                                                     * of its shape; the menus reach it through WAVE / PW) */
static const uint16_t PM_DEFAULT[PC_N] = {40u, 42u, 59u, 63u, PM_SMART, 67u, 79u};
static UI_LATE void play_smart(void)
{
    ui.smart = (uint8_t)(shadow[17] >= 64u ? 18u : shadow[27] ? 27u : shadow[28] ? 28u : 18u);
    ui.ppatch = set.patch;
}
static UI_LATE const u_param_t *pm_code(uint32_t code)   /* an assignment's parameter, 0 none */
{
    int32_t i;
    if (code == PM_SMART && ui.smart == 18u) return &P_PW1;
    i = pl_find(code == PM_SMART ? ui.smart : code);
    return i < 0 ? 0 : pl_p((uint32_t)i);
}
static UI_LATE const u_param_t *pm_param(uint32_t c, int *smart)   /* control c's parameter now */
{
    uint32_t code = set.pm[c];
    const u_param_t *p = code == PM_DEF ? 0 : pm_code(code);
    if (!p) {                            /* (the default; an assignment not in this release's list: too) */
        code = PM_DEFAULT[c];
        p = pm_code(code);
    }
    *smart = code == PM_SMART;
    return p;
}
static UI_LATE void pv_fill(void)                /* the controls' parameters for the screen and the knobs (ui.c pv) */
{
    uint32_t c;
    int sm;
    for (c = 0; c < PC_N; c++) {
        pv.p[c] = pm_param(c, &sm);
        pv.smart[c] = (uint8_t)sm;
    }
}
static UI_LATE int play_on(void) { return ui.view == V_PLAY || ui.view == V_PLAYED; }   /* (its pages) */
#define PM_HOLD_MS 500u                  /* PLAY held this long: PLAY mode on / off (as SHIFT's OCT- hold) */
static uint8_t pm_on PLAT_POOL;          /* PLAY mode on (1.18C43: the HOME screen while on) */
static uint32_t pm_t PLAT_POOL;          /* PLAY down since (odd), 2 its hold taken, 0 up */
static uint8_t pm_rel PLAT_POOL;         /* (1.19.7) PLAY's press: 1 found the transport running (its stop waits for
                                          * the release of a short press), 2 the launcher's PLAY then (ln_play on the
                                          * release), 4 it started the transport (taken back if the hold fires) */
static UI_LATE void play_go(int fresh)           /* PLAY mode's screen (fresh: it starts, the smart choice taken) */
{
    if (fresh) play_smart();
    ui.view = V_PLAY;
    ui.pch = 0;
    pv_fill();
    view_reset();
}
static UI_LATE void pm_toggle(void)              /* (1.18C43) PLAY held: PLAY mode on (its page) or off (HOME) */
{
    pm_on = (uint8_t)!pm_on;
    overlay_off();
    if (pm_on) play_go(1);
    else if (play_on()) qa_go(0);
    app_say("Play mode", pm_on ? "ON" : "OFF");
}
static UI_LATE void pm_home(uint32_t now)        /* (1.18C43) PLAY mode on: HOME's screen is its page (once the patch
                                          * list, a value list, the favorites' editor are away) */
{
    if (pm_on && ui.view == V_QUICK && !ui.qpos && !fv.ed && (!ui.plist_until || now >= ui.plist_until)
        && (!ui.vl_until || now >= ui.vl_until))
        play_go(0);
}
static UI_LATE void play_turn(uint32_t c, int32_t e)   /* control c turned by e detents */
{
    static const uint8_t ENC[PC_N] = {0, 1, 2, 3, 5, 6, 4};   /* (knob_steps' own speed each; ALGO's as before) */
    pv_fill();
    if (pv.p[c]) param_turn(pv.p[c], ENC[c], e, (int32_t)c);
}
static UI_LATE void played_go(void)              /* the assignment editor: the controls, BACK */
{
    ui.view = V_PLAYED;
    ui.pcur = 0;
    ui.pch = 0;
    pv_fill();
    view_reset();
}
static UI_LATE uint32_t pm_sel(uint32_t code)    /* an assignment's place in the list of choices */
{
    int32_t i;
    if (code == PM_DEF) return 0u;
    if (code == PM_SMART) return 1u;
    i = pl_find(code);
    return i < 0 ? 0u : 2u + (uint32_t)i;
}
static UI_LATE void played_rec(void)             /* REC: a control's choices open; the one chosen taken; BACK */
{
    if (!ui.pch && ui.pcur >= PC_N) {    /* BACK: PLAY mode */
        play_go(0);
        return;
    }
    if (!ui.pch) {
        ui.pch = 1;
        ui.psel = (uint16_t)pm_sel(set.pm[ui.pcur]);
    } else {
        char t[24];
        set.pm[ui.pcur] = (uint16_t)(ui.psel == 0u ? PM_DEF : ui.psel == 1u ? PM_SMART : pl_p(ui.psel - 2u)->byte);
        glo_touch();                     /* (saved with GLO's settings) */
        ui.pch = 0;
        pv_fill();
        t[0] = 0;
        if (ui.psel < 2u) text_cat(t, ui.psel ? "SMART" : "DEFAULT");
        else p_name(pv.p[ui.pcur], t);
        app_say(PC_NAME[ui.pcur], t);
    }
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void played_home(void)            /* HOME: the choices closed (the assignment as it was); the editor to PLAY mode */
{
    if (ui.pch) {
        ui.pch = 0;
        ui.d_rows = ui.d_lcd = 1;
    } else
        play_go(0);
}
static UI_LATE void played_select(int32_t e)     /* SELECT (K1 too) */
{
    pl_build();
    if (ui.pch) ui.psel = (uint16_t)clamp((int32_t)ui.psel + e, 0, (int32_t)pl_n + 1);
    else ui.pcur = (uint8_t)clamp((int32_t)ui.pcur + e, 0, (int32_t)PC_N);
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void pe_item(uint32_t i, char *t, char *pre, uint32_t *fl)   /* the editor's lines: "K1  Cutoff", BACK */
{
    (void)pre; (void)fl;
    if (i >= PC_N) { text_cat(t, "BACK"); return; }
    text_cat(t, PC_NAME[i]);
    text_cat(t, "  ");
    if (pv.smart[i]) text_cat(t, "SMART ");
    if (pv.p[i]) p_name(pv.p[i], t);
}
#ifdef HYB_MULTI
static UI_LATE uint32_t cm_mig(uint32_t code, uint32_t k)   /* (1.19.6) a 1.19.2 .. 1.19.5 PLAY mode control's
                                          * parameter (as pm_param, from the object's code): its line, 255 none */
{
    const u_param_t *p = code == PM_DEF ? 0 : pm_code(code);
    if (!p) p = pm_code(PM_DEFAULT[k]);
    return p && cm_line(p_byte(p)) ? p_byte(p) : 255u;
}
static UI_LATE void cm_run(uint32_t now)         /* (1.19.6) main loop: learn's number taken; the moved lines' CCs in,
                                          * each as turning its parameter would set it */
{
    uint32_t q = cm_lrn, v;
    const u_param_t *p;
    cm_lrn = 0;
    if (cm_ed && !(ui.view == V_GLO && ui.gch == G_CC)) cm_ed = 0;   /* (the list left: off) */
    if (cm_ed == 1u && (q & 0x80u)) cm_set(q & 0x7Fu);
    cm_arm = cm_ed == 1u;
    while (cm_t != cm_h) {
        q = cm_q[cm_t];
        cm_t = (uint8_t)((cm_t + 1u) & 7u);
        if ((p = cm_line(q >> 7)) == 0 || p_parta(p)) continue;   /* (part A's effects while B is edited: as its
                                          * knob, nothing) */
#ifdef HYB_SHIM_LEND
        if (P_BTOFF(p)) continue;
#endif
        v = ((q & 0x7Fu) * p_max(p) + 63u) / 127u;
        if (v == shadow[p_byte(p)]) continue;
        edit_set(p_byte(p), v, now);
        ui.d_rows = ui.d_lcd = 1;
    }
}
static UI_LATE void cp_run(uint32_t now)         /* CC SHOW, main loop: the newest fixed map controller moved says its
                                          * parameter and the value the engine took (sy_set_param: the byte itself,
                                          * the list's last past its end), as the panel says a knob's; in CHANNELS its
                                          * part first. Moved lines and learn say theirs through edit_set (cm_run) */
{
    fm1cp_notice n;
    const u_param_t *p;
    char a[24], v[16];
    if (!fm1cp_take(&cc_pop, now, &n) || !n.part || n.id >= 120u || (p = cm_line(n.id)) == 0) return;   /* (no
                                          * part listens: the engine took nothing) */
    a[0] = 0;
    if (M.mode == SY_M_CHANNELS && n.part != 3u) text_cat(a, n.part == 2u ? "B " : "A ");
    p_name(p, a);
    value_text(p, n.value > p_max(p) ? p_max(p) : n.value, v);
    app_say(a, v);
}
#endif
static UI_LATE void pc_item(uint32_t i, char *t, char *pre, uint32_t *fl)   /* a control's choices: DEFAULT, SMART (each with
                                                     * what it is now), every parameter */
{
    (void)pre; (void)fl;
    if (i >= 2u) { p_name(pl_p(i - 2u), t); return; }
    text_cat(t, i ? "SMART\t" : "DEFAULT\t");
    if (!i && PM_DEFAULT[ui.pcur] == PM_SMART) text_cat(t, "SMART");
    else {
        const u_param_t *p = pm_code(i ? PM_SMART : PM_DEFAULT[ui.pcur]);
        if (p) p_name(p, t);
    }
}
static UI_LATE void play_lcd(char *a, char *v)   /* the LCD strip: PLAY mode (HOLD) and the patch; the editor's line */
{
    if (ui.view == V_PLAY) {
        text_cat(a, hold_on ? "PLAY HOLD" : "PLAY");
        patch_line(v);
    } else if (!ui.pch) {
        text_cat(a, "ASSIGN");
        text_cat(v, ui.pcur < PC_N ? PC_NAME[ui.pcur] : "BACK");
    } else {                             /* choosing: the control, the block the parameter is in */
        text_cat(a, PC_NAME[ui.pcur]);
        text_cat(v, ui.psel == 0u ? "DEFAULT" : ui.psel == 1u ? "SMART" : U_ROW[pl_row(pl_tab[ui.psel - 2u])].tag);
    }
}

/* SAVE: the user banks' slots, C001 .. H128 in one list (FONT_M, a filled slot with its name, an
 * empty one with a box), the cursor on the patch up if it is a user patch, else on the first empty slot. SELECT moves
 * it (faster turned faster: knob_steps), K1 a bank (C .. H: the same slot; OCT- / OCT+ too, 1.00, lit for the banks
 * there are before / after); REC opens the name editor with the patch's
 * name: SELECT turns the letter under the cursor (NM_SET), K1 (and OCT- / OCT+) moves the cursor, K2 the letter's
 * case; REC writes the patch (shadow, the new name in it) into the slot (a filled slot: asked once, REC again
 * overwrites it), it becomes the patch up, the edit memory of the patch it was saved from and of the slot are cleared
 * (they are in the user patch now), the undo steps too, the settings saved (the patch, the octave, the rest: what SAVE
 * kept before the user banks). HOME: the editor to the list, the list back to where SAVE was pressed (SAVE again:
 * the same) */
static const char NM_SET[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-./'+#&!()*:_,";
#define NM_N (sizeof NM_SET - 1u)
static UI_LATE int sv_on(void) { return ui.view == V_SAVE || ui.view == V_NAME; }
static UI_LATE void sv_open(void)                /* SAVE: the slot list */
{
    uint32_t u = 0;
    if (ui.view == V_GLO && ui.gch == G_SEARCH) gd_close();
    ui.sfrom = ui.view;
    if (set.patch >= PU_0 && set.patch < PATCH_INIT) u = set.patch - PU_0;
    else {
        while (u < UB_N && ub_filled(u)) u++;
        if (u >= UB_N) u = 0;
    }
    ui.scur = (uint16_t)u;
    ui.view = V_SAVE;
    view_reset();
}
static UI_LATE void sv_back(void)                /* the list closed: back to where SAVE was pressed */
{
    ui.view = ui.sfrom;
    view_reset();
    home_sig[0] = home_sig[1] = 0;
}
static UI_LATE void sv_home(void)                /* HOME (SAVE): the editor to the list, the list closed */
{
    if (ui.view == V_NAME) { ui.view = V_SAVE; view_reset(); }
    else sv_back();
}
static UI_LATE void sv_name_open(void)           /* REC on a slot: the name editor, the patch's name in it */
{
    uint32_t k;
    for (k = 0; k < 10u; k++) {
        const char c = (char)shadow[240 + k];
        ui.nm[k] = c >= 32 && c < 127 ? c : ' ';
    }
    ui.ncur = 0;
    ui.nask = 0;
    ui.view = V_NAME;
    view_reset();
}
static UI_LATE void nm_move(int32_t d)           /* the cursor (K1, OCT- / OCT+) */
{
    ui.ncur = (uint8_t)clamp((int32_t)ui.ncur + d, 0, 9);
    ui.nask = 0;
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void nm_turn(int32_t e)           /* SELECT: the letter under the cursor through NM_SET (round) */
{
    uint32_t i = 0;
    while (i < NM_N && NM_SET[i] != ui.nm[ui.ncur]) i++;
    if (i >= NM_N) i = 0;                /* (a character not in the set: from the space) */
    i = (uint32_t)(((int32_t)i + e % (int32_t)NM_N + (int32_t)NM_N) % (int32_t)NM_N);
    ui.nm[ui.ncur] = NM_SET[i];
    ui.nask = 0;
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void nm_case(void)                /* K2: the letter's case */
{
    char c = ui.nm[ui.ncur];
    if (c >= 'a' && c <= 'z') c = (char)(c - 32);
    else if (c >= 'A' && c <= 'Z') c = (char)(c + 32);
    ui.nm[ui.ncur] = c;
    ui.nask = 0;
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE int sp_runs(void);
static UI_LATE void sv_write(void)               /* REC in the editor: the patch into the slot (the beat running: at
                                          * its stop, "SAVE PENDING", 1.02: no flash write under the beat) */
{
    const uint32_t u = ui.scur, p = PU_0 + u, from = set.patch;
    uint32_t k;
    char t[8];
    if (ub_filled(u) && !ui.nask) {      /* (a filled slot: asked once) */
        ui.nask = 1;
        app_say("REC again", "OVERWRITE");
        ui.d_rows = 1;
        return;
    }
    for (k = 0; k < 10u; k++)
        shadow[240 + k] = (uint8_t)ui.nm[k];   /* (the name: not the engine's, nothing to send) */
    patch_tag(t, p);
    if (sp_runs()) {
        sqv.sv_w |= 4u;
        sqv.svpw[0] = (uint16_t)u;
        sqv.svpw[1] = set.patch;
        sv_back();
        app_say(t, "SAVE PENDING");
        return;
    }
    if (ub_write(u, shadow) != 0) {
        ui.nask = 0;
        ui.d_rows = 1;
        app_say("Save", "FAILED");
        return;
    }
    for (k = em_n; k-- > 0;)             /* the edits are in the user patch now */
        if (EM_PM(em[k].patch) == EM_P(from) || EM_PM(em[k].patch) == EM_P(p)) em_drop(k);
    undo_n = 0;                          /* (steps of patches whose edits are gone) */
    sx_edit = 0;                         /* (a SysEx single in the edit buffer: it is the slot's now) */
    set.patch = (uint16_t)p;
    set.patch8 = (uint8_t)old_of_patch(p);
    set.pt_ok = PT_V2;
    if (sv_on()) sv_back();              /* (at the stop, 1.02: the view as it is) */
#ifdef HYB_MULTI
    if (patch_o == p) mx_slot_other();   /* (the other part's slot written over: it takes what the slot holds now) */
    if (mx_save_set() == 0) {            /* (each part's patch in its field) */
#else
    if (store_save(OBJ_SETTINGS, &set, sizeof set) == 0) {
#endif
        ui.glo_t = 0;                    /* (GLO's settings saved with the rest) */
        app_say("Saved", t);
    } else
        app_say(t, "SETTINGS FAILED");
}
static UI_LATE void sv_rec(void) { if (ui.view == V_SAVE) sv_name_open(); else sv_write(); }
static UI_LATE void sv_select(int32_t e)         /* SELECT */
{
    if (ui.view == V_NAME) { nm_turn(e); return; }
    ui.scur = (uint16_t)ls_acc(4u, ui.scur, e, ui.scur / UB_SLOTS * UB_SLOTS, ui.scur / UB_SLOTS * UB_SLOTS + UB_SLOTS - 1u);
    if (ui.scur >= UB_N) ui.scur = e < 0 ? 0u : UB_N - 1u;   /* (1.18C35: a bank's ends stop a fast turn, ls_acc) */
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void sv_knob(uint32_t k, int32_t e)   /* K1..K4 */
{
    if (ui.view == V_NAME) {
        if (k == 0u) nm_move(e);
        else if (k == 1u) nm_case();
        return;
    }
    if (k == 0u) {                       /* a bank */
        int32_t u = (int32_t)ui.scur + (e > 0 ? 1 : -1) * (int32_t)UB_SLOTS;
        if (u >= 0 && u < (int32_t)UB_N) { ui.scur = (uint16_t)u; ui.d_rows = ui.d_lcd = 1; }   /* (none further: as
                                          * it is, nothing drawn) */
    }
}
static UI_LATE void sv_foot(char *l, char *r)   /* the foot strip: the user banks' letters OCT- / OCT+ go to ("" none) */
{
    const uint32_t b = ui.scur / UB_SLOTS;
    l[0] = r[0] = l[1] = r[1] = 0;
    if (b > 0) l[0] = (char)(UB_L0 + b - 1u);
    if (b + 1u < UB_BANKS) r[0] = (char)(UB_L0 + b + 1u);
}
static UI_LATE void sv_item(uint32_t i, char *t, char *pre, uint32_t *fl)   /* the slot list's lines */
{
    (void)pre;
    patch_tag(t, PU_0 + i);
    if (!ub_filled(i)) { *fl |= LS_EMPTY; return; }
    if (PU_0 + i == set.patch) patch_label(t, PU_0 + i, shadow);
    else {
        char nm[10];
        ub_name(i, nm);
        name_label(t, PU_0 + i, nm);
    }
}
static UI_LATE void sv_lcd(char *a, char *v)     /* the LCD strip: the slot; in the editor the name's slot, or the
                                                  * question */
{
    char t[8];
    patch_tag(t, PU_0 + ui.scur);
    if (ui.view == V_NAME && ui.nask) { text_cat(a, "REC again"); text_cat(v, "OVERWRITE"); return; }
    text_cat(a, ui.view == V_NAME ? "Name" : "Save to");
    text_cat(v, t);
    if (ui.view == V_SAVE && !ub_filled(ui.scur)) text_cat(v, " empty");
}
static UI_LATE void sv_draw(void)                /* the name editor (ui.c nm_body) */
{
    uint32_t k;
    patch_tag(nv.tag, PU_0 + ui.scur);
    nv.filled = (uint8_t)ub_filled(ui.scur);
    nv.ask = ui.nask;
    nv.cur = ui.ncur;
    for (k = 0; k < 10u; k++) nv.nm[k] = ui.nm[k];
    nv.old[0] = 0;
    if (nv.filled) {
        char nm[10];
        ub_name(ui.scur, nm);
        name_label(nv.old, PU_0 + ui.scur, nm);
    }
    draw_body(UI_ROW0_Y, UI_BODY_H, nm_body);
}

/* ------------------------------------------------- the sequencer's panel --- */
/* SEQ: its page (it does not start it); SEQ on the page: the drums' page, again back to where it was pressed. On the
 * page K1..K4 act on the step held (EVENT, LENGTH, ALIGN, VEL; SELECT its offset) or, no step key held, on the
 * sequencer: LENGTH (16, 32, 64 steps), TIME DIV (0.25x .. 4x), PATTERN (NONE, 1 .. 32), CLEAR (turned: "CLEAR? REC";
 * REC clears the pattern); SELECT the BASE NOTE (what it plays while no input note is held: OFF, C1 .. C7; the
 * settings keep it). The 16 white keys are a block of 16 steps (OCT- / OCT+: the block shown, sqv.blk; RECORD: the octave, 1.19.7): a tap turns
 * one on or off, one held shows its values (changed and let go: still SQ_TT_MS, 1.18C31); the black keys do nothing there. The parameter turned last is up large
 * in the strip for SQ_TT_MS. SHIFT + SEQ (one part): off, anywhere. Once on it runs in every view (the input notes from the keys
 * off the page and MIDI). Its edits wait in the journal (un_note) until SAVE (AUTO SAVE ON: SQ_SAVE_MS after the
 * last change, all idle); the settings keep the pattern's number (glo_touch). The runtime and the timing: sq_block (the audio interrupt) */
#define SQ_TAP_MS 400u                   /* a step key let go sooner, nothing changed: on / off */
#define SQ_SAVE_MS 3000u                 /* the pattern saved this long after its last change */
#define SQ_CLR_MS 3000u                  /* CLEAR's question this long */
#define SQ_TT_MS 1500u                   /* the parameter turned: up in the strip this long after the last turn; a
                                          * step changed and let go: its view kept this long (sqv.lin, 1.18C31) */
#define SQ_CE_REST 300u                  /* (1.18C31) EVENT's turns this soon after the one that opened CHORD ENTRY
                                          * (and after each other): swallowed, so a flick does not leave it again */
#define SQ_TT_TEMPO 0xFFu                /* sqv.tt: the tempo (ALGO) */
#ifdef HYB_MULTI
static UI_LATE void sq_keys_put(uint32_t keys)   /* the input notes (the keys held off the page, HOLD's) to each
                                          * sequencer's keys: its part's (sq_parts_key) */
{
    uint32_t w[SQ_NI][4], k, i, j, m;
    hold_zone_fix();                     /* (1.16: SPLIT, a key latched by the other part's HOLD: not input here) */
    for (k = 0; k < 4u; k++) w[0][k] = w[1][k] = 0;
    /* (P8: only the bits set are visited; this runs every pass of the main loop, and the HOLD bits are mostly empty
     * or few: a scan of all 128 notes and the keys cost ~100 k a block with the sequencer running) */
    for (m = keys & ((1u << NKEYS) - 1u); m; m &= m - 1u) {
        k = (uint32_t)__builtin_ctz(m);
        if (ui.held[k] != KEY_NONE) {
            const uint32_t to = sq_parts_key(ui.held[k]);
            for (i = 0; i < SQ_NI; i++)
                if (to >> i & 1u) w[i][ui.held[k] >> 5] |= 1u << (ui.held[k] & 31u);
        }
    }
    for (j = 0; j < 4u; j++)
        for (m = hold_bits[j]; m; m &= m - 1u) {
            const uint32_t n = j << 5 | (uint32_t)__builtin_ctz(m), to = sq_parts_key(n);
            for (i = 0; i < SQ_NI; i++)
                if (to >> i & 1u) w[i][j] |= 1u << (n & 31u);
        }
    for (k = 0; k < 4u; k++)
        w[!ed_part][k] |= hold_bo[k];    /* (1.15: CHANNELS, the other part's HOLD's notes its input) */
    for (i = 0; i < SQ_NI; i++)
        for (k = 0; k < 4u; k++)
            if (sqr[i].keys[k] != w[i][k]) sqr[i].keys[k] = w[i][k];
#else
static UI_LATE void sq_keys_put(uint32_t keys)   /* the input notes (the keys held off the page, HOLD's) to sq.keys */
{
    uint32_t w[4] = {0, 0, 0, 0}, k, m;
    for (m = keys & ((1u << NKEYS) - 1u); m; m &= m - 1u) {   /* (P8: the bits set only) */
        k = (uint32_t)__builtin_ctz(m);
        if (ui.held[k] != KEY_NONE) w[ui.held[k] >> 5] |= 1u << (ui.held[k] & 31u);
    }
    for (k = 0; k < 4u; k++) {
        w[k] |= hold_bits[k];
        if (sq.keys[k] != w[k]) sq.keys[k] = w[k];
    }
#endif
    if (ui.view != V_SEQ && ui.view != V_DRUM) sqv.key = sqv.clr = 0;   /* (the page left with a step key down, CLEAR
                                          * asking) */
}
/* the sequencers' edit journal (after 1.00, 13): every change not saved
 * yet of every pattern (the drums' 16, the arp patterns' 32) as the bytes it changed, newest last, one entry a byte:
 * its pattern and place (jr_a), its old and its new value, an undo level's first (bit 31); a CLEAR one entry (JR_CLR,
 * 1.18C33). A pattern loaded (dr_load_to, sq_load_to: the drums' spare too) is the store's bytes with its entries
 * applied in order (jr_apply), so its edits survive another pattern chosen and this one again until SAVE writes it
 * (jr_store, its entries dropped) or the power goes; with AUTO SAVE OFF nothing else writes a pattern. 1.18C33 (the
 * user's rules): UNDO (SHIFT + HOME on the pages, EDIT > UNDO) for big changes alone: CLEAR, a RECORD pass, DUPLICATE,
 * an instrument ERASED each begin an undo level of the pattern (un.next: UN_ISBIG); UNDO takes the page's pattern back
 * to before its newest level, the small edits made after it with it (its entries from there dropped, the pattern
 * loaded again). A small edit (a step tapped, a value turned, a chord entered) has no undo: its bytes join the
 * pattern's newest level's entries (one a byte). Never refused: full, the oldest levels given up (jr_evict: no longer
 * undone, their bytes' entries joined); still full, the pattern edited is marked (pk: its RAM copy its edits until
 * SAVE, "EDITS FULL" "NO UNDO  SAVE"); 90 % full: "EDITS NEARLY FULL" "SAVE" once (up to 1.18C32 a change past the
 * room was taken back, "EDITS FULL" "SAVE"). A change is found against a copy of the pattern as last seen (un_shd
 * beside drp, un_shs; while un_shd is the drums' spare: the store's bytes and the entries, on the stack) and the copy
 * brought up to date, so the edits themselves need not know. In the pool: JR_N entries (4 B each) and the marked
 * patterns' spare copies (pk_c) */
#ifndef JR_N
#if defined(HYB_MULTI) && defined(HYB_BANK_PACK)
#define JR_N 576u                        /* (1.18C33: 2.25 KB; 1.02 to 1.18C32 136, 192 up to 1.01: the pool's 8 KB spare
                                          * the build insists on kept, with the spare copies, pk_c) */
#else
#define JR_N 640u                        /* (1.18C33: 2.5 KB; 256 up to 1.18C32) */
#endif
#endif
#define JR_K (DR_PATS + SQ_PATS)          /* the patterns' keys: the drums' 0 .. 15, the arp patterns' 16 .. 47 */
_Static_assert(sizeof(dr_pat_t) < 0x3FFu && sizeof(sq_pat_t) < 0x1FFu && DR_PATS * 1024u + SQ_PATS * 512u <= 0x8000u
               && JR_K <= 64u, "jr_a: a place in 15 bits; a pattern's last place (JR_CLR) past its bytes");
#define JR_CLR(k) ((k) < DR_PATS ? 0x3FFu : 0x1FFu)   /* (1.18C33) pattern k's CLEAR: one entry at its last place, old
                                          * 0xFF, new its LENGTH (an arp pattern's: | its pad << 2), sq_clr_to / dr_clr_to */
#define UN_KNOB(c) (1u << 24 | ((c) + 1u) << 8 | (sqv.key ? sqv.sel : 0xFFu))   /* (a small edit's kinds: no undo) */
#define UN_REC (2u << 24)                /* (+ the pass: a RECORD pass, one undo) */
#define UN_BIG (3u << 24)                /* (| un.seq: DUPLICATE, an instrument ERASED, CLEAR: one undo each) */
#define UN_ISBIG(x) ((x) >> 25 == 1u)
static struct {
    uint32_t e[JR_N];                    /* bit 31: an undo level's first (a big change's); the place (jr_a) << 16; the
                                          * old byte << 8; the new */
    uint32_t key, t;                     /* the last big change's kind (its level, joined while it goes on), the last
                                          * change */
    uint32_t next;                       /* the kind of the change being made (input() clears it) */
    uint32_t say_t;                      /* (a message due: say, sq_idle) */
    uint16_t n, seq;                     /* the entries; the big changes counted (UN_BIG) */
    uint8_t k;                           /* the last big change's pattern */
    uint8_t pass;                        /* the recording pass (dr_arm, the transport) */
    uint8_t say, wn;                     /* a message due (UN_SAY_*); NEARLY FULL said */
    uint8_t nou;                         /* the last big change has no undo (its level did not fit) */
} un PLAT_POOL;
#define UN_SAY_NEAR 1u
#define UN_SAY_FULL 2u
/* (1.18C33) the patterns whose unsaved edits the journal cannot hold ("marked", pk.m): their RAM copy (a sequencer's
 * pattern, pk.hk) is theirs until SAVE; one left (another pattern chosen) goes into a spare copy (pk_c, PK_NC of them),
 * or into the journal if it has room by then; else the user is asked (pk_gate: REC saves, HOME drops its edits) */
#ifndef PK_NC
#define PK_NC 2u
#endif
#define PK_NH (1u + UN_NS)               /* the holders: the drums' pattern (drp), the arp sequencers' (un_cur) */
static struct {
    uint64_t m;                          /* the marked patterns (bit k: key k) */
    uint8_t hk[PK_NH];                   /* each holder's pattern + 1 (un_load; 0 none yet) */
    uint8_t ck[PK_NC];                   /* each spare copy's pattern + 1 (0 free, 0xFF kept for the drums' line) */
    uint8_t ask, aq, want;               /* the question: its pattern + 1, its holder, the pattern asked for + 1 */
    uint8_t dres;                        /* the drums' copy kept for their line (its slot + 1) */
    uint8_t ha;                          /* bit h: holder h's copy is its marked pattern's own (not a copy of it) */
    uint8_t raw;                         /* the loads: the store's bytes alone (no journal, no marked copy) */
} pk PLAT_POOL;
static union { dr_pat_t d; sq_pat_t s; } pk_c[PK_NC] PLAT_POOL;
#define JR_ANY (un.n || pk.m)            /* unsaved pattern edits */
/* (the copies as last seen: un_shd, beside drp; un_shs) */
static sq_pat_t un_shs[UN_NS] PLAT_POOL;
static UI_LATE uint8_t *un_cur(uint32_t q)
{
#ifdef HYB_MULTI
    return q ? (uint8_t *)&sqpa[q - 1u] : (uint8_t *)&drp;
#else
    return q ? (uint8_t *)&sqp : (uint8_t *)&drp;
#endif
}
static UI_LATE uint8_t *un_copy(uint32_t q) { return q ? (uint8_t *)&un_shs[q - 1u] : (uint8_t *)&un_shd; }
static UI_LATE uint32_t un_size(uint32_t q) { return q ? (uint32_t)sizeof(sq_pat_t) : (uint32_t)sizeof(dr_pat_t); }
static UI_LATE uint32_t jr_key(uint32_t q)       /* sequencer q's pattern (0 the drums, 1 + i arp sequencer i): its key */
{
    if (!q) return dr_sp && sq_qs[2] == SQ_QS_DONE ? dr_spn : set.dr_pat;   /* (swapped on the line, the main loop
                                          * not there yet: the spare's) */
#ifdef HYB_MULTI
    if (q > 1u) return DR_PATS + set.sqb_pat;
#endif
    return DR_PATS + set.sq_pat;
}
static UI_LATE uint32_t jr_a(uint32_t k) { return k < DR_PATS ? k << 10 : 0x4000u | (k - DR_PATS) << 9; }   /* pattern
                                          * k's first place */
static UI_LATE uint32_t jr_pat(uint32_t e)       /* entry e's pattern */
{
    const uint32_t a = e >> 16 & 0x7FFFu;
    return a < 0x4000u ? a >> 10 : (a >> 9) - 32u + DR_PATS;
}
static UI_LATE uint32_t jr_of(uint32_t e, uint32_t k)   /* entry e's byte in pattern k (~0: another's) */
{
    return jr_pat(e) == k ? (e >> 16) - jr_a(k) & 0x3FFu : ~0u;
}
static UI_LATE int jr_is_clr(uint32_t e) /* (1.18C33) entry e is a pattern's CLEAR (JR_CLR) */
{
    const uint32_t a = e >> 16 & 0x7FFFu;
    return a < 0x4000u ? (a & 0x3FFu) == 0x3FFu : (a & 0x1FFu) == 0x1FFu;
}
static UI_LATE void sq_clr_to(sq_pat_t *q, uint32_t lc, uint32_t pad)   /* an arp pattern CLEARed (sq_rec, its entry):
                                          * all steps off, the rest as shipped; its LENGTH lc and pad kept */
{
    uint32_t k;
    sq_defaults(q, 0);
    q->lc = (uint8_t)lc;
    q->pad = (uint8_t)(pad & SQ_PADM);
    for (k = 0; k < SQ_N; k++) q->st[k].ev = SQ_NOTE;
}
static UI_LATE void dr_clr_to(dr_pat_t *q, uint32_t lc)   /* a drum pattern CLEARed (dr_rec, its entry): all steps
                                          * off, their values and the time division as shipped; its LENGTH lc kept */
{
    dr_defaults(q, 0, 0);
    q->lc = (uint8_t)lc;
}
static UI_LATE void jr_apply(uint8_t *p, uint32_t k)   /* pattern k's entries onto p (as the store holds it) */
{
    uint32_t i, o, v;
    if (pk.raw) return;
    for (i = 0; i < un.n; i++)
        if ((o = jr_of(un.e[i], k)) != ~0u) {
            v = un.e[i] & 0xFFu;
            if (o != JR_CLR(k)) p[o] = (uint8_t)v;
            else if (k < DR_PATS) dr_clr_to((dr_pat_t *)p, v & 3u);
            else sq_clr_to((sq_pat_t *)p, v & 3u, v >> 2);
        }
}
static UI_LATE void jr_sweep(void)       /* the entries cleared (0) and those changed back (old = new; not a level's
                                          * first, not a CLEAR) removed */
{
    uint32_t i, j = 0;
    for (i = 0; i < un.n; i++) {
        const uint32_t e = un.e[i];
        if (!e || (!(e >> 31) && !jr_is_clr(e) && (e >> 8 & 0xFFu) == (e & 0xFFu))) continue;
        un.e[j++] = e;
    }
    un.n = (uint16_t)j;
}
static UI_LATE void pk_unmark(uint64_t w);
static UI_LATE void jr_drop(uint64_t w)          /* the patterns w's entries (bit k: key k) dropped (and their marks) */
{
    uint32_t i;
    for (i = 0; i < un.n; i++)
        if (w >> jr_pat(un.e[i]) & 1u) un.e[i] = 0;
    jr_sweep();
    if (w >> un.k & 1u) un.key = 0;
    pk_unmark(w);
}
static UI_LATE int32_t jr_lvl(uint32_t k, uint32_t i)   /* pattern k's newest undo level's first entry before entry i
                                          * (-1: none) */
{
    while (i--)
        if ((un.e[i] >> 31) && jr_pat(un.e[i]) == k) return (int32_t)i;
    return -1;
}
static UI_LATE int32_t jr_find(uint32_t k, uint32_t pl)   /* pattern k's entry for place pl in its newest level (or
                                          * before any; -1: none there) */
{
    uint32_t i = un.n;
    while (i--) {
        const uint32_t e = un.e[i];
        if (jr_pat(e) != k) continue;
        if ((e >> 16 & 0x7FFFu) == pl) return (int32_t)i;
        if ((e >> 31) || jr_is_clr(e)) return -1;   /* (the level's first; a CLEAR: nothing older counts) */
    }
    return -1;
}
static UI_LATE int jr_evict(void)        /* the oldest undo level given up (no longer undone): its entries joined with
                                          * the level before (a byte's one entry, its newest value); a CLEAR's: the
                                          * entries before it dropped. 0: none left */
{
    uint32_t g, i, j, k, pl, lo;
    int32_t p;
    for (g = 0; g < un.n && !(un.e[g] >> 31); g++)
        ;
    if (g == un.n) return 0;
    k = jr_pat(un.e[g]);
    un.e[g] &= ~(1u << 31);
    if (k == un.k && jr_lvl(k, un.n) < 0) un.nou = 1;   /* (the change going on has no undo: its rest joins none) */
    if (jr_is_clr(un.e[g])) {
        for (i = 0; i < g; i++)
            if (jr_pat(un.e[i]) == k) un.e[i] = 0;
    } else {
        p = jr_lvl(k, g);
        lo = p < 0 ? 0u : (uint32_t)p;
        for (j = g; j < un.n; j++) {
            const uint32_t e = un.e[j];
            if (!e || jr_pat(e) != k) continue;
            if ((j > g && (e >> 31)) || jr_is_clr(e)) break;   /* (the next level) */
            pl = e >> 16 & 0x7FFFu;
            for (i = g; i > lo; ) {
                const uint32_t f = un.e[--i];
                if (f && jr_pat(f) == k && (f >> 16 & 0x7FFFu) == pl && !jr_is_clr(f)) {
                    un.e[i] = (f & ~0xFFu) | (e & 0xFFu);
                    un.e[j] = 0;
                    break;
                }
            }
        }
    }
    jr_sweep();
    return 1;
}
/* (1.18C33) the marked patterns (pk, above) */
static UI_LATE int pk_on(uint32_t k) { return (int)(pk.m >> k & 1u); }
static UI_LATE uint32_t pk_size(uint32_t k) { return k < DR_PATS ? (uint32_t)sizeof(dr_pat_t) : (uint32_t)sizeof(sq_pat_t); }
static UI_LATE void pk_copy(void *d, const void *s, uint32_t k)
{
    uint32_t i;
    for (i = 0; i < pk_size(k); i++) ((uint8_t *)d)[i] = ((const uint8_t *)s)[i];
}
static UI_LATE int pk_line(void) { return dr_sp && sq_qs[2] == SQ_QS_DONE; }   /* (the drums swapped on the line, the
                                          * main loop not there yet: drp holds dr_spn, the spare the one left) */
static UI_LATE uint32_t pk_own(uint32_t h)       /* holder h's pattern + 1 (0: none yet) */
{
    return !h && pk_line() ? dr_spn + 1u : pk.hk[h];
}
static UI_LATE uint8_t *pk_where(uint32_t k, const uint8_t *nt)   /* marked pattern k's copy: a holder's (not nt) or
                                          * a spare (0: none) */
{
    uint32_t h;
    for (h = 0; h < PK_NH; h++) {
        uint8_t *p = un_cur(h);
        if (!h && pk_line()) {           /* (drp: the spare's pattern now, the spare the one left) */
            if (dr_spn == k && p != nt) return p;
            if (pk.hk[0] == k + 1u && (pk.ha & 1u) && (uint8_t *)&un_shd != nt) return (uint8_t *)&un_shd;
        } else if (pk.hk[h] == k + 1u && (pk.ha >> h & 1u) && p != nt) return p;
    }
    for (h = 0; h < PK_NC; h++)
        if (pk.ck[h] == k + 1u) return (uint8_t *)&pk_c[h];
    return 0;
}
static UI_LATE int32_t pk_free(void)    /* a free spare copy (-1: none) */
{
    uint32_t i;
    for (i = 0; i < PK_NC; i++)
        if (!pk.ck[i]) return (int32_t)i;
    return -1;
}
static UI_LATE int32_t pk_slot(uint32_t k)       /* pattern k's spare copy (-1: none) */
{
    uint32_t i;
    for (i = 0; i < PK_NC; i++)
        if (pk.ck[i] == k + 1u) return (int32_t)i;
    return -1;
}
static UI_LATE void pk_unmark(uint64_t w)        /* the patterns w unmarked (saved, dropped, copied onto) */
{
    uint32_t i;
    pk.m &= ~w;
    for (i = 0; i < PK_NC; i++)
        if (pk.ck[i] && pk.ck[i] != 0xFFu && (w >> (pk.ck[i] - 1u) & 1u)) pk.ck[i] = 0;
}
static UI_LATE int pk_journal(uint32_t k, const uint8_t *src)   /* marked pattern k's copy src into the journal
                                          * against the store's bytes (older undo levels given up for it): 1 = done,
                                          * unmarked */
{
    union { dr_pat_t d; sq_pat_t s; } b;
    const uint8_t *s = (const uint8_t *)&b;
    const uint32_t sz = pk_size(k), a = jr_a(k);
    uint32_t i, f = 0;
    pk.raw = 1;
    if (k < DR_PATS) dr_load_to(&b.d, k); else sq_load_to(&b.s, k - DR_PATS);
    pk.raw = 0;
    for (i = 0; i < sz; i++) f += src[i] != s[i];
    while (un.n + f > JR_N)
        if (!jr_evict()) return 0;
    for (i = 0; i < sz; i++)
        if (src[i] != s[i]) un.e[un.n++] = (a + i) << 16 | (uint32_t)s[i] << 8 | src[i];
    pk_unmark((uint64_t)1 << k);
    return 1;
}
static UI_LATE void pk_name(char *t, uint32_t k) /* "DRUMS 3", "PATTERN 5" */
{
    pat_text(t, k < DR_PATS ? "DRUMS " : "PATTERN ", (int32_t)(k < DR_PATS ? k : k - DR_PATS) + 1);
}
static UI_LATE void pk_park(uint32_t k, const uint8_t *src)   /* marked pattern k's copy src leaves its holder: into a
                                          * spare copy, else the journal (pk_gate asked before when neither) */
{
    const int32_t i = pk_free();
    char t[12];
    if (pk_where(k, src)) return;        /* (another holds it) */
    if (i >= 0) { pk_copy(&pk_c[i], src, k); pk.ck[i] = (uint8_t)(k + 1u); return; }
    if (pk_journal(k, src)) return;
    pk_unmark((uint64_t)1 << k);         /* (never silently: the changes that can wait ask first, pk_gate) */
    pk_name(t, k);
    app_say(t, "EDITS LOST");
}
static UI_LATE int pk_load(uint32_t k, uint8_t *q)   /* (sq_load_to, dr_load_to) pattern k into q: a holder's
                                          * marked pattern left kept (pk_park); k marked: its copy (1 = done, nothing
                                          * from the store) */
{
    uint32_t h, o;
    const uint8_t *s;
    if (pk.raw) return 0;
    for (h = 0; h < PK_NH && un_cur(h) != q; h++)
        ;
    o = h < PK_NH ? pk_own(h) : 0u;
    if (o && o != k + 1u && pk_on(o - 1u) && (pk.ha >> h & 1u) && !(!h && pk_line())) {
        const int32_t i = pk_slot(k);
        if (pk_free() < 0 && i >= 0 && pk_on(k) && !pk_where(o - 1u, q)) {   /* (no copy free, k's own
                                          * there: the two exchanged) */
            union { dr_pat_t d; sq_pat_t s; } t;
            pk_copy(&t, &pk_c[i], k);
            pk_copy(&pk_c[i], q, o - 1u);
            pk.ck[i] = (uint8_t)o;
            pk_copy(q, &t, k);
            pk.ha |= (uint8_t)(1u << h);
            return 1;
        }
        pk_park(o - 1u, q);
    }
    if (h < PK_NH && !(o == k + 1u && pk_on(k))) pk.ha &= (uint8_t)~(1u << h);
    if (!pk_on(k)) return 0;
    if (o == k + 1u && (h >= PK_NH || (pk.ha >> h & 1u) || (!h && pk_line()))) return 1;   /* (its holder again: as
                                          * it is) */
    if (!(s = pk_where(k, q))) { pk_unmark((uint64_t)1 << k); return 0; }
    pk_copy(q, s, k);
    if (h < PK_NH && s >= (const uint8_t *)pk_c && s < (const uint8_t *)(pk_c + PK_NC)) pk.ha |= (uint8_t)(1u << h);   /* (from a
                                          * spare copy: its own now, the copy freed, pk_took) */
    return 1;
}
static UI_LATE void pk_took(uint32_t q)  /* holder q holds its pattern now (un_load): its spare copy free again */
{
    const uint32_t k = jr_key(q);
    uint32_t i;
    pk.hk[q] = (uint8_t)(k + 1u);
    if (pk.ha >> q & 1u)
        for (i = 0; i < PK_NC; i++)
            if (pk.ck[i] == k + 1u) pk.ck[i] = 0;
}
static UI_LATE void pk_mark(uint32_t q, uint32_t k)   /* pattern k's edits (holder q's) beyond the journal: its
                                          * entries dropped, its RAM copy its own until SAVE (no undo) */
{
    uint32_t i;
    pk.ha |= (uint8_t)(1u << q);
    for (i = 0; i < un.n; i++)
        if (jr_pat(un.e[i]) == k) un.e[i] = 0;
    jr_sweep();
    if (k == un.k) un.key = 0;
    pk.m |= (uint64_t)1 << k;
    un.say = UN_SAY_FULL;
    un.say_t = plat_ms();
}
static UI_LATE int pk_gate(uint32_t h, uint32_t n)   /* holder h (0 the drums, 1 + i arp sequencer i) about to take
                                          * its sequencer's pattern n: 1 = go; 0 = the user asked (its marked pattern
                                          * fits nowhere: no spare copy free, no room in the journal) */
{
    const uint32_t o = pk.hk[h], kn = h ? DR_PATS + n : n;
    if (!o || !pk_on(o - 1u) || !(pk.ha >> h & 1u) || o == kn + 1u) return 1;
    if (pk_free() >= 0 || pk_slot(kn) >= 0 || pk_where(o - 1u, un_cur(h)) || pk_journal(o - 1u, un_cur(h)))
        return 1;                        /* (a copy free, or the one taken in a copy: exchanged, pk_load) */
    if (sqv.sp_wait == 0xFFu || sqv.sp_wait == o) return 0;   /* (its SAVE waits for the stop: it stays, not asked again) */
    pk.ask = (uint8_t)o;
    pk.aq = (uint8_t)h;
    pk.want = (uint8_t)(n + 1u);
    ui.d_lcd = 1;
    return 0;
}
static UI_LATE int pk_dgate(uint32_t n)  /* (dr_qidle) the drums' pattern n into the spare for the line: pk_gate, a
                                          * spare copy kept for the pattern left, filled on the line (pk_dleft) */
{
    int32_t i;
    if (!pk_gate(0u, n)) return 0;
    if (pk.hk[0] && pk_on(pk.hk[0] - 1u) && (pk.ha & 1u) && !pk.dres && !pk_where(pk.hk[0] - 1u, (uint8_t *)&drp) && (i = pk_free()) >= 0) {
        pk.ck[i] = 0xFFu;
        pk.dres = (uint8_t)(i + 1);
    }
    return 1;
}
static UI_LATE void pk_dfree(void)       /* (dr_spfree) the copy kept for the line given back */
{
    if (pk.dres) pk.ck[pk.dres - 1u] = 0;
    pk.dres = 0;
}
static UI_LATE void pk_dleft(void)       /* (dr_qidle, swapped on the line) the drums' pattern left, in the spare: kept */
{
    const uint32_t o = pk.hk[0];
    if (o && pk_on(o - 1u) && (pk.ha & 1u)) {
        if (pk.dres) { pk_copy(&pk_c[pk.dres - 1u], &un_shd, o - 1u); pk.ck[pk.dres - 1u] = (uint8_t)o; pk.dres = 0; }
        else pk_park(o - 1u, (const uint8_t *)&un_shd);
    }
    pk_dfree();
    pk.ha = (uint8_t)((pk.ha & ~1u) | (uint32_t)pk_on(dr_spn));   /* (drp: the spare's, from its copy if marked) */
}
static UI_LATE void un_load(uint32_t q)  /* pattern q loaded (its entries applied): its copy taken */
{
    const uint8_t *c = un_cur(q);
    uint8_t *s = un_copy(q);
    uint32_t i;
    if (!q && dr_sp) return;             /* (the drums' copy the spare now: taken when it is given back) */
    for (i = 0; i < un_size(q); i++) s[i] = c[i];
    pk_took(q);
}
static UI_LATE void sq_def_sync(void)    /* (1.14) GLO > SEQ's defaults changed: each arp sequencer's pattern in RAM (and
                                          * its copy as last seen) takes its settings as a load would now (store +
                                          * journal): a pattern never saved (its head not written) and not turned in
                                          * EDIT since, GLO's; any other, its own (unchanged). ASSUMED: a pattern never
                                          * saved has no settings of its own yet, so it follows GLO > SEQ (its notes
                                          * and steps, unsaved, are not touched; nothing is journalled) */
{
    sq_pat_t p;
    uint32_t i;
    for (i = 0; i < UN_NS; i++) {
        sq_pat_t *const q = (sq_pat_t *)un_cur(1u + i);
        sq_load_to(&p, jr_key(1u + i) - DR_PATS);
#ifdef HYB_MULTI
        if ((q->pad ^ p.pad) & SQ_NLAT) sqr[i].lclr = 1;
#else
        if ((q->pad ^ p.pad) & SQ_NLAT) sq.lclr = 1;
#endif
        q->pad = un_shs[i].pad = p.pad;
    }
}
static UI_LATE void seq_blk_fix(void);
static UI_LATE int un_note(uint32_t q)   /* pattern q changed: the bytes changed into the journal. Never refused
                                          * (1.18C33): full, the oldest undo levels given up; still full, the pattern
                                          * marked (its RAM copy its own until SAVE, pk_mark). A big change (UN_ISBIG:
                                          * un.next) starts an undo level on its first byte, a RECORD pass's bytes join
                                          * it; a small change's bytes join the pattern's newest level (or none) */
{
    dr_pat_t b;                          /* (the drums' as before while un_shd is their spare) */
    uint8_t *c = un_cur(q), *s = un_copy(q);
    const uint32_t key = un.next, now = plat_ms(), sz = un_size(q), k = jr_key(q), a = jr_a(k);
    uint32_t i, f, lvl;
    un.next = 0;
    un.t = now;
    if (!q && dr_sp) { dr_load_to(&b, k); s = (uint8_t *)&b; }
    if (pk_on(k)) {                      /* (marked: its RAM copy is its edits) */
        for (i = 0; i < sz; i++) s[i] = c[i];
        if (UN_ISBIG(key)) un.nou = 1;
        return 1;
    }
    lvl = UN_ISBIG(key) && (key != un.key || k != un.k);   /* (a big change begins: a level of its own) */
    if (lvl) un.nou = 0;
    for (;;) {                           /* (the entries needed; full: the oldest level given up, counted again) */
        for (f = i = 0; i < sz; i++)
            if (c[i] != s[i]) f += lvl || jr_find(k, a + i) < 0;
        if (un.n + f <= JR_N) break;
        if (!jr_evict()) {
            if (UN_ISBIG(key)) un.nou = 1;
            pk_mark(q, k);
            for (i = 0; i < sz; i++) s[i] = c[i];
            return 1;
        }
    }
    for (f = i = 0; i < sz; i++) {
        int32_t j;
        if (c[i] == s[i]) continue;
        j = lvl ? -1 : jr_find(k, a + i);
        if (j >= 0) { un.e[j] = (un.e[j] & ~0xFFu) | c[i]; f = 1; }   /* (a byte the level has: its old kept) */
        else {
            un.e[un.n++] = (lvl ? 1u << 31 : 0u) | (a + i) << 16 | (uint32_t)s[i] << 8 | c[i];
            if (lvl) { lvl = 0; un.key = key; un.k = (uint8_t)k; }
        }
        s[i] = c[i];
    }
    if (f) jr_sweep();                   /* (a byte back to its value before: its entry gone, the pattern maybe saved) */
    if (!un.wn && un.n * 10u >= JR_N * 9u) {   /* (90 % full: SAVE asked for, once) */
        un.wn = 1;
        un.say = UN_SAY_NEAR;
        un.say_t = now;
    }
    return 1;
}
static UI_LATE void un_clear(uint32_t q) /* (1.18C33) pattern q CLEARed (sq_clr_to, dr_clr_to done): one entry, an
                                          * undo level of its own; the pattern marked: back in the journal (no undo) */
{
    uint8_t *c = un_cur(q), *s = un_copy(q);
    const uint32_t sz = un_size(q), k = jr_key(q), a = jr_a(k);
    uint32_t i, lv = 1;
    un.next = 0;
    un.t = plat_ms();
    un.nou = 0;
    if (pk_on(k)) { pk_unmark((uint64_t)1 << k); lv = 0; }
    while (un.n + 1u > JR_N && jr_evict())
        ;
    if (un.n + 1u > JR_N) {              /* (no level left: its own entries go, the CLEAR replaces them) */
        for (i = 0; i < un.n; i++)
            if (jr_pat(un.e[i]) == k) un.e[i] = 0;
        jr_sweep();
        lv = 0;
    }
    if (!lv) un.nou = 1;
    if (un.n + 1u > JR_N) pk_mark(q, k);
    else un.e[un.n++] = (lv ? 1u << 31 : 0u) | (a + JR_CLR(k)) << 16 | 0xFF00u
                        | (k < DR_PATS ? ((dr_pat_t *)c)->lc : ((sq_pat_t *)c)->lc | ((sq_pat_t *)c)->pad << 2);
    un.key = UN_BIG | ++un.seq;
    un.k = (uint8_t)k;
    if (!(!q && dr_sp))
        for (i = 0; i < sz; i++) s[i] = c[i];
}
#define sq_dirty(st) un_note(1u + UN_SQI)   /* (step st changed: the arp pattern's change found, un_note) */
static UI_LATE int store_wait(void)              /* the patterns', the kit's and the settings' saves wait until all
                                          * is idle (0.65): the transport stopped, nothing sounding, nothing touched for
                                          * STORE_IDLE_MS (st_busy_t; 0.63-0.64: the transport alone, so the erase a few
                                          * seconds after a stop clicked into the tails); then once their change is
                                          * SQ_SAVE_MS, DR_SAVE_MS, GLO_SAVE_MS old */
{
    if (!(set.md_pad & MD_AUTO)) return 1;   /* (AUTO SAVE OFF, as shipped from 0.65: SAVE saves them, sv_all) */
    return dr.run || SQ_RUNS || plat_ms() - st_busy_t < STORE_IDLE_MS;
}
static UI_LATE void sr_end(void);
static UI_LATE void sq_hold(int h);
#ifdef HYB_MULTI
static uint8_t sq_sh PLAT_POOL;          /* the arp sequencer shown (sq_mi; sq_hold_sync's loop moves sq_mi) */
#define SQ_SHOWN (sq_sh == sq_mi)
#else
#define SQ_SHOWN 1
#endif
/* the preview (0.85): a step held on the arp sequencer's page heard as its OFFSET, VEL or LENGTH
 * is turned (each detent that changes it), and a step a tap turns on: its note (BASE NOTE, or KEY LATCH's, + OFFSET:
 * as the roll shows it, rl_pitch; NOTE steps alone) at its VEL on the sequencer's part, out over MIDI as its notes,
 * for its LENGTH (at most SQ_PV_MS; turning LENGTH SQ_PV_LMS and a bar, to hear it). Each cuts the one before; its
 * note-off asked by the main loop (sq_pv_idle): at its end, a turn's also when the step key is let go, and at once
 * when the page is left, the other sequencer shown, PATTERN turned (NONE: none), its transport started or stopped,
 * PANIC; the notes themselves sent by the audio interrupt (sq_pvq). With PATCH ARP OFF its part's arpeggiator is held
 * off from the first one until the step key is let go and the last has ended (SQ_PV_TAIL later; sq_hold_want), as
 * while it runs, so the note sounds as the step will. A few bytes: the note + 1 (0 none), its part (f bit 0), the
 * transport then (bit 1), the hold (bit 2), a turn's (bit 3: ends with the step key), its end (plat_ms, 16 bits) */
#define SQ_PV_MS 400u
#define SQ_PV_LMS 2000u
#define SQ_PV_TAIL 50u
#define SQ_PV_RES 800u                   /* (1.18C33) a turn's preview ended this long ago: its sequencer's steps
                                          * heard again though the step key is still held (sq_pv_idle) */
/* (1.18C44) the audition: a step key held SQ_AUD_MS on the arp sequencer's page, nothing
 * changed, its step on: the step heard as the preview plays it (sq_pv_play with SQ_PV_AUD: its notes, a CHORD step's
 * all, FING POLY's the input notes) at its VEL, sustained while the key is held (SQ_AUD_KEEP renewed, sq_aud_idle),
 * its sequencer muted meanwhile as by a turn's preview (f bit 3) and heard again SQ_PV_TAIL after the key is let go;
 * the other part and the drums play on. Its release toggles nothing (sqv.aud SQ_AUD_ON); a turn while it sounds
 * plays the step changed, again sustained (LENGTH: its timed preview, as before) */
#define SQ_AUD_MS 250u                   /* (1.18C44) a step key held this long: its step heard */
#define SQ_PV_AUD 16u                    /* (1.18C44) sq_pv_play's rel, sq_pv.f bit 4: the audition, sustained */
#define SQ_AUD_KEEP 30000u               /* (1.18C44) its end pushed this far ahead each frame while held (int16) */
#define SQ_AUD_TRY 1u                    /* (1.18C44) sqv.aud: tried this press, nothing heard (off, VEL 0, no note) */
#define SQ_AUD_ON 2u                     /* (1.18C44) sqv.aud: heard this press (its release toggles nothing) */
static struct { uint8_t n, f; uint16_t t; uint8_t k, a, m, b; uint16_t t0, tt; } sq_pv PLAT_POOL;   /* (k .. tt: the
                                          * previews' rate, pvr: one object in the pool) */
#define pvr sq_pv
/* the previews' rate (0.86): a knob's preview (the arp step's OFFSET, LENGTH, VEL, the drum
 * step's ACCENT, LENGTH) starts at most every PVR_GAP ms. A turn inside that time keeps its preview
 * waiting (pvr: the latest one), heard once the knob has rested PVR_REST ms or PVR_GAP has passed since the last
 * (pv_tick, the main loop), if its page, step and instrument are still the ones turned; the values between skipped.
 * A step tapped on is heard at once, as before; the transport's start, PATTERN turned and PANIC drop the waiting
 * one, and so does a page, step or instrument other than the one turned when it is due. k: 0 none, 1 an arp step (a
 * the step, m the sequencer, b LENGTH's limit), 2 a drum step (a the step, m the instrument; 0.86-1.00: 3
 * the kit's TUNE, gone after 1.00); t0 the last preview's start, tt the last turn (plat_ms, 16 bits) */
#define PVR_GAP 120u
#define PVR_REST 80u
static UI_LATE int pv_ask(uint32_t k, uint32_t a, uint32_t m, uint32_t b)   /* a turn's preview: 1 heard now (the
                                          * caller plays it), 0 waiting (pv_tick) */
{
    const uint16_t n = (uint16_t)plat_ms();
    pvr.tt = n;
    if (!pvr.k && (uint16_t)(n - pvr.t0) >= PVR_GAP) { pvr.t0 = n; return 1; }
    pvr.k = (uint8_t)k; pvr.a = (uint8_t)a; pvr.m = (uint8_t)m; pvr.b = (uint8_t)b;
    return 0;
}
static UI_LATE void pv_tick(uint32_t now);
#ifdef APP_HOST_CMD
static uint8_t pv_hn, pv_hlast, pv_hpart, pv_hlive, pv_hbad, pv_hvel;   /* (the host simulator: the previews, hostcmd.c) */
static uint16_t pv_hms;
#endif
static UI_LATE int sq_hold_want(uint32_t run)   /* the sequencer shown running (run) or not: its part's patch
                                          * arpeggiator to be held off: while it runs and while RECORD is armed on it
                                          * (the keys recorded sound as single notes); never with its pattern's PATCH
                                          * ARP ON (0.71: its notes feed the arpeggiator; 0.76: RECORD armed too, the
                                          * keys heard arpeggiated as they will play back, recorded plainly: sq_rec_key;
                                          * arming then toggles nothing, engine arp_switch) */
{
    return !(sqp.pad & SQ_PARP) && (run || sq.on   /* (stopped: given back once the audio interrupt has sent
                                          * its notes' offs, a block after them, 0.94: in one block the engine takes the
                                          * arpeggiator's byte before the offs, which went to it then and hung) */
                                    || (sqv.arec && sqv.arec_mi == UN_SQI)
                                    || (sq_pv.f & 5u) == (4u | UN_SQI)   /* (a step heard as it plays: sq_pv_play) */
                                    || (ui.view == V_SEQ && SQ_SHOWN && SQ_PATSH < 0));   /* (PATTERN NONE on its
                                          * page: the keys practised as the steps would play them) */
}
static UI_LATE void sq_hold_sync(void)   /* app_frame: each arp sequencer's hold as it should be (a pattern chosen,
                                          * PATCH ARP turned, undone or copied onto it, RECORD armed or not) */
{
#ifdef HYB_MULTI
    const uint8_t m = sq_mi;
    sq_sh = m;
    for (sq_mi = 0; sq_mi < SQ_NI; sq_mi++) sq_hold(sq_hold_want(sq.run));
    sq_mi = m;
#else
    sq_hold(sq_hold_want(sq.run));
#endif
}
static UI_LATE void sq_pv_note(uint32_t v)       /* the preview's note on (v) / off (0): its part, out as sq_emit's
                                          * (the audio interrupt's, sq_pvq) */
{
    const uint32_t n = sq_pv.n - 1u;
#ifdef APP_HOST_CMD
    if (v) { pv_hbad = (uint8_t)(pv_hbad + pv_hlive); pv_hn++; pv_hlast = (uint8_t)n; pv_hpart = sq_pv.f & 1u;
             pv_hvel = (uint8_t)v; pv_hms = (uint16_t)(sq_pv.t - plat_ms()); }
    pv_hlive = v != 0;
    if (sq_log) printf("  sqpv %s %u vel %u part %u ms %u\n", v ? "on " : "off", (unsigned)n, (unsigned)v,
                       (unsigned)(sq_pv.f & 1u), (unsigned)plat_ms());
#endif
    sq_pvq = ((sq_pvq & 0xFF000000u) + 0x01000000u) | (v ? n | 0x80u | v << 8 | (sq_pv.f & 1u) << 15 : 0u);
}
static UI_LATE void sq_pv_cut(void)      /* the preview sounding: off */
{
    if (sq_pv.n) { sq_pv_note(0u); sq_pv.n = 0; sq_pv.t = (uint16_t)plat_ms(); }   /* (the hold's tail from here) */
}
static UI_LATE void sq_pv_mute(void)    /* (0.94) the arp sequencer whose step held is heard as it is turned (a turn's
                                          * preview, f bit 3, on its page, its transport as then): muted from its
                                          * first preview until the step key is let go and the last has ended
                                          * (SQ_PV_TAIL later, sq_pv_idle), its notes sounding cut at the start
                                          * (sq_cut); the other part's and the drums play on */
{
    const uint32_t m = (sq_pv.f & 8u) && ui.view == V_SEQ && SQ_PATSH >= 0
                       && (sq_pv.f & 3u) == (UN_SQI | (uint32_t)sq.run << 1) ? 1u << (sq_pv.f & 1u) : 0u;
    if (m & ~(uint32_t)sq_pvm) sq_cut[UN_SQI] = 1u;
    sq_pvm = (uint8_t)((sq_pvm & 4u) | m);
}
static UI_LATE uint32_t sq_aud_poly(int32_t *p)   /* (1.18C44) the audition of a FING POLY step: the input notes
                                          * held (HOLD's, MIDI's), none held the notes latched (KEY LATCH, as sq_live
                                          * shows them), else the base (rl_base); lowest first, SQ_CHV at most */
{
    uint32_t w, b, n = 0, m[4], any = 0;
    for (w = 0; w < 4u; w++) any |= m[w] = sq.keys[w] | sq.midi[w];
    if (!any && sq_live())
        for (w = 0; w < 4u; w++) m[w] = sq.lat[w];
    for (w = 0; w < 4u; w++)
        for (b = m[w]; b && n < SQ_CHV; b &= b - 1u) p[n++] = (int32_t)(w * 32u + (uint32_t)__builtin_ctz(b));
    if (!n) p[n++] = rl_base();
    return n;
}
static UI_LATE void sq_pv_play(const sq_step_t *t, uint32_t lim, uint32_t rel)   /* step t of the sequencer shown
                                          * heard now, for its LENGTH, a bar and lim ms at most (rel 8: and until the
                                          * step key is let go; | SQ_PV_AUD, 1.18C44: only then) */
{
    int32_t k = -1, p[SQ_CHV];
    uint32_t w = SQ_LEN(t->la & 31u), ms, i, j = 0, n;
    sq_pv_cut();
    if (!SQ_OFS(t->ev & 15u) && (t->ev & 15u) != SQ_CHORD
        && ((t->ev & 15u) != SQ_POLY || !(rel & SQ_PV_AUD))) return;   /* (1.18C31: FING n too, at the base + OFFSET
                                          * as the roll shows it: its finger's note unknown here; 1.18C44: FING POLY
                                          * in the audition alone) */
    n = (t->ev & 15u) == SQ_POLY ? sq_aud_poly(p) : rl_notes(t, p);   /* (1.02: a chord, its notes on the base as it
                                          * plays them) */
    for (i = 0; i < n; i++)
        if ((uint32_t)p[i] < 128u) { if (k < 0) k = p[i]; else SQ_PVX[j++] = (uint8_t)(p[i] + 1); }
    while (j < SQ_CHV - 1u) SQ_PVX[j++] = 0;
    if (!t->vel || k < 0 || SQ_PATSH < 0) return;
    if (w > SQ_LEN(SQ_LBAR)) w = SQ_LEN(SQ_LBAR);
    ms = (uint32_t)((float)(w * 7500u >> sqp.div) / (S.bpm_now > 1.f ? S.bpm_now : 120.f));   /* (a tick at 1x:
                                          * 1875 / bpm ms) */
    sq_pv.n = (uint8_t)(k + 1);
    sq_pv.f = (uint8_t)(UN_SQI | (uint32_t)sq.run << 1 | 4u | rel);
    sq_pv.t = (uint16_t)(plat_ms() + (rel & SQ_PV_AUD ? SQ_AUD_KEEP : ms < lim ? ms : lim));
    sq_hold(sq_hold_want(sq.run));       /* (PATCH ARP OFF: its part's arpeggiator held off before the note) */
    sq_pv_mute();                        /* (a turn's: the sequencer's own notes off first, 0.94) */
    sq_pv_note(t->vel);
}
static UI_LATE uint32_t sr_on(void);
static UI_LATE void sq_pv_keep(void)     /* (1.11) a key tapped in CHORD ENTRY, heard while it is down (sq_ch_key):
                                          * with PATCH ARP OFF its part's arpeggiator held off before its note, as
                                          * for a preview (the hold bit, f bit 2), until CHORD ENTRY is over and the
                                          * last of its keys' notes has ended (SQ_PV_TAIL later, sq_pv_idle); up to
                                          * 1.10 the notes went through the patch's arpeggiator with the transport
                                          * stopped. PATCH ARP ON: into it, as the sequencer plays the chord */
{
    if (!sq_pv.n) { sq_pv.f = (uint8_t)(UN_SQI | (uint32_t)sq.run << 1 | 4u); sq_pv.t = (uint16_t)plat_ms(); }
    sq_hold(sq_hold_want(sq.run));
}
static UI_LATE void sq_pv_idle(uint32_t now)     /* sq_idle: the preview ended (its time, or the page's state changed) */
{
    const uint32_t ok = ui.view == V_SEQ && SQ_PATSH >= 0 && (sq_pv.f & 3u) == (UN_SQI | (uint32_t)sq.run << 1);
    if (sq_pv.n && (!ok || ((sq_pv.f & 8u) && !SQ_HELD) || (int16_t)(uint16_t)(now - sq_pv.t) >= 0)) sq_pv_cut();
    if (!sq_pv.n && (sq_pv.f & 4u) && sr_on()) sq_pv.t = (uint16_t)now;   /* (1.11: CHORD ENTRY's keys sounding,
                                          * sq_pv_keep: the tail from their last note-off) */
    if (!sq_pv.n && !(ok && SQ_HELD) && !sr_on() && (int16_t)(uint16_t)(now - sq_pv.t) >= (int16_t)SQ_PV_TAIL)
        sq_pv.f = 0;                     /* (the hold given back by sq_hold_sync SQ_PV_TAIL after the note-off: a
                                          * margin, the arpeggiator never back before the engine has the note-off) */
    if (!sq_pv.n && (sq_pv.f & 8u) && (int16_t)(uint16_t)(now - sq_pv.t) >= (int16_t)SQ_PV_RES
        && (int32_t)(now - sqv.tt_t) >= (int32_t)SQ_PV_RES)   /* (signed: tt_t is now | 1, 1.18C46) */
        sq_pv.f &= (uint8_t)~8u;         /* (1.18C33: a turn's preview over and the knobs resting, the step key still
                                          * held: the sequencer's part heard again; the next turn mutes it again) */
    sq_pv_mute();                        /* (and the sequencer's own steps with it, 0.94) */
}
static UI_LATE void sq_aud_idle(uint32_t now)   /* (1.18C44) sq_idle: the audition started once a step key has
                                          * been held SQ_AUD_MS on the page with nothing changed (not in CHORD
                                          * ENTRY, RECORD), its step on; sustained while it is held */
{
    if (sq_pv.n && (sq_pv.f & SQ_PV_AUD)) sq_pv.t = (uint16_t)(now + SQ_AUD_KEEP);   /* (its end: the key let go,
                                          * the page's state changed, sq_pv_idle) */
    if (ui.view != V_SEQ || !sqv.key || sqv.aud || sqv.edited || sqv.lin || sqv.ce || sqv.arec
        || now - sqv.down_t < SQ_AUD_MS) return;
    sqv.aud = SQ_AUD_TRY;                /* (once a press) */
    if (!(sqp.st[sqv.sel].ev & SQ_ON)) return;   /* (an off step: nothing, nothing muted; a tap still turns it on) */
    sq_pv_play(&sqp.st[sqv.sel], SQ_PV_MS, 8u | SQ_PV_AUD);
    if (sq_pv.n) sqv.aud = SQ_AUD_ON;
}
static UI_LATE void sq_qidle(void);
static UI_LATE void sq_ce_end(void);
static UI_LATE int jr_store(uint64_t w);
static UI_LATE uint64_t jr_pats(void);
static UI_LATE void sq_idle(uint32_t now)        /* app_frame: AUTO SAVE ON, the patterns edited saved once they rest
                                          * and all is idle (store_wait); CLEAR's question over */
{
    sq_qidle();                          /* (0.96: a pattern waiting for its line, sq_q*) */
    pv_tick(now);                        /* (a turn's preview waiting: now, 0.86) */
    sq_aud_idle(now);                    /* (a step key held: its step heard, 1.18C44) */
    sq_pv_idle(now);                     /* (before sq_hold_sync: the preview's note-off, then the hold back) */
    if (sqv.arec && (ui.view != V_SEQ || (sqv.arec_run && !sq.run) || sqv.arec_mi != UN_SQI || SQ_PATSH < 0)) {   /* (RECORD off: the
                                          * page left, the transport stopped, the other sequencer shown) */
        sqv.arec = 0;
        sr_end();
    }
    if (ui.view != V_SEQ && !sqv.arec) sr_end();   /* (1.02: a chord's keys still down as the page was left: off) */
    if (sqv.lin && (!sqv.key || ui.view != V_SEQ)) sqv.lin = 0;
    else if (sqv.lin && !SQ_CE && (uint16_t)(now - sqv.lin_t) >= SQ_TT_MS) sqv.lin = sqv.key = 0;   /* (1.18C31: a
                                          * step changed and let go: its view gone SQ_TT_MS later, the page as it was;
                                          * CHORD ENTRY stays until REC / HOME) */
    if (sqv.ce && (!sqv.key || ui.view != V_SEQ)) sq_ce_end();   /* (1.03: CHORD ENTRY's page, step or sequencer
                                          * left, its EDIT list opened: over, nothing stored) */
    sq_metro = (uint8_t)(sqv.arec ? 1u + UN_SQI : 0u);
    if (sqv.arec && sq.run && (uint32_t)(sq.step >> 4) != sqv.blk[0]) {   /* (1.19.7: RECORD running, OCT- / OCT+
                                          * the octave: the block shown follows the step playing) */
        sqv.blk[0] = (uint8_t)(sq.step >> 4);
        seq_blk_fix();
    }
    sq_hold_sync();                      /* (RECORD off: its part's arpeggiator back unless it runs; PATCH ARP turned,
                                          * a pattern chosen or copied onto it, undone) */
    if (sqv.arec && sqv.arec_run != sq.run) un.pass++;   /* (a recording pass: from a start to a stop) */
    sqv.arec_run = sq.run;
    if (un.say && now - un.say_t >= 1400u) {   /* (1.18C33: after the change's own word) */
        if (un.say == UN_SAY_NEAR) app_say("EDITS NEARLY FULL", "SAVE");
        else app_say("EDITS FULL", "NO UNDO  SAVE");
        un.say = 0;
    }
    if (un.wn && un.n * 4u < JR_N * 3u) un.wn = 0;
    if (JR_ANY && !store_wait() && now - un.t >= SQ_SAVE_MS) {
        un.t = now;                      /* (failed: again SQ_SAVE_MS later, not every frame) */
        (void)jr_store(jr_pats());
    }
    if (sqv.clr && now - sqv.clr_t >= SQ_CLR_MS) sqv.clr = 0;
}
#define SQ_BUSY_MS 2500u                 /* "SEQ A IS PLAYING": up this long */
#define SQ_BUSY_GAP 1000u                /* said again on a further key at most this often */
static UI_LATE int sq_silent(const sq_pat_t *p)   /* no step on (with a velocity) within its LENGTH: it plays nothing */
{
    uint32_t i;
    for (i = 0; i < SQ_STEPS(p->lc); i++)
        if ((p->st[i].ev & SQ_ON) && p->st[i].vel) return 0;
    return 1;
}
static UI_LATE void sq_busy_say(uint32_t now)    /* app_frame (after input): a key struck off the sequencers' pages
                                          * that went only to running arp sequencers as their input (it sounds not
                                          * itself, key_out) whose patterns have no step on: nothing sounds, the strip
                                          * says why ("SEQ A  IS PLAYING") */
{
    static uint32_t last_k PLAT_POOL, said_t PLAT_POOL;
    const uint32_t nw = ui.keys & ~last_k;
    uint32_t k, p = 0;
    last_k = ui.keys;
    if (!nw || !SQ_RUNS || (said_t && (int32_t)(now - said_t) < (int32_t)SQ_BUSY_GAP)) return;   /* (signed: said_t
                                          * is now | 1, 1.18C46) */
    for (k = 0; k < NKEYS; k++) {
        if (!(nw >> k & 1u) || ui.held[k] == KEY_NONE) continue;   /* (a step, a function: not a note) */
#ifdef HYB_MULTI
        {
            const uint32_t to = sq_parts_key(ui.held[k]), run = (sqr[0].run ? 1u : 0u) | (sqr[1].run ? 2u : 0u);
            if ((to & ~run) || ((to & 1u) && !sq_silent(&sqpa[0])) || ((to & 2u) && !sq_silent(&sqpa[1]))) continue;
            p |= to;                     /* (sounds nowhere: every part it goes to has its sequencer silent) */
        }
#else
        if (sq_silent(&sqp)) p = 1u;
#endif
    }
    if (!p) return;
    said_t = now | 1u;
#ifdef HYB_MULTI
    if (set.mx_mode != SY_M_SINGLE || p > 1u)
        app_say(p == 3u ? "SEQ A+B" : p == 2u ? "SEQ B" : "SEQ A", "IS PLAYING");
    else
#endif
        app_say("ARP SEQ", "IS PLAYING");   /* (no room for a hint "PLAY STOPS" beside it) */
    ui.msg_until = now + SQ_BUSY_MS;
}
static UI_LATE int rp_on(void);
static UI_LATE void rp_go(void);
static UI_LATE void rp_back(void);
#ifdef HYB_RAND
static UI_LATE void rz_go(void);
#endif
#ifdef HYB_RAND
static uint16_t rz_from HYB_SPARE;       /* (after 0.98) the patch INIT was reached from in this browse (+ 1; 0 none):
                                          * RANDOMIZE below INIT varies it (rz_go) */
#endif
static UI_LATE void fv_to(int32_t i);
static UI_LATE void fv_valid(void);
static UI_LATE void fv_check(void);
static UI_LATE void fe_turn(int32_t e);
static UI_LATE void fe_bank(int32_t d);
static UI_LATE void ps_go(uint32_t q)   /* PRESET: patch q loaded, the patch list up (PLIST_MS again) */
{
    uint32_t i;
    fv.sec = fv.ask = 0;                 /* (1.18C33: out of the favorites' section; fv_to puts it back) */
    if (ui.rp_park == EM_P(q) + 1u)      /* (the patch reset: as stored, its edits gone) */
        for (i = em_n; i--;)
            if (EM_PM(em[i].patch) == EM_P(q)) em_drop(i);
    ui.rp_park = ui.ps_cur = ui.ls_fly = 0;   /* (1.18C35: a fast turn's place: this one) */
#ifdef HYB_RAND
    rz_from = (uint16_t)(q >= PATCH_INIT && set.patch < PATCH_INIT ? set.patch + 1u : 0u);
#endif
    if (q >= PATCH_INIT && set.patch < PATCH_INIT) ui.ps_b = (uint8_t)(set.patch / PA_N);   /* (INIT: the bank left) */
    ui.rp_ask = 0;
    all_notes_off();
    if (ui.plist_new || plat_ms() >= ui.plist_until)   /* (a browse with the list up: one undo step) */
        undo_push(set.patch, UNDO_PATCH, 0);
    ui.plist_new = 0;
    ui.plist_until = plat_ms() + PLIST_MS;
    set.patch = (uint16_t)q;
    load_patch(set.patch);
    ui.last = -1;
    ui.edit_t = 0;
    ui.alt_t = plat_ms();                /* (the patch first) */
    ui.d_lcd = ui.d_rows = 1;
}
#define PS_AT (ui.ps_cur ? ui.ps_cur - 1u : set.patch)   /* (1.18C35) the patch list's cursor: a fast turn's place */
#define ps_nb(d, to) ps_nb_from(fv.sec ? PATCH_INIT : set.patch, (d), (to))   /* (the favorites' section: as INIT) */
static UI_LATE int32_t ps_nb_from(uint32_t p, int32_t d, uint32_t *to)   /* the bank d (-1 before, 1 after) the patch
                                          * list's OCT- / OCT+ reach from patch p, in PRESET's order (INIT, A .. E; the
                                          * empty ones skipped): 1 .. and its patch in *to (its slot of the same number
                                          * when filled, else its first filled); -1 none (the hint strip and the LEDs use
                                          * it too; 1.18C33: the favorites editor's banks) */
{
    int32_t b = p >= PATCH_INIT ? 0 : (int32_t)(p / PA_N) + 1;
    uint32_t q;
    while ((b += d) > 0 && b <= (int32_t)(PATCH_INIT / PA_N)) {
        const uint32_t b0 = (uint32_t)(b - 1) * PA_N;
        if (patch_filled(b0 + p % PA_N)) { *to = b0 + p % PA_N; return b; }
        for (q = b0; q < b0 + PA_N; q++)
            if (patch_filled(q)) { *to = q; return b; }
    }
    return -1;
}
static UI_LATE void ps_bank(int32_t d)   /* OCT- / OCT+ tapped with the patch list up (0.78): the bank before / after;
                                          * none further: as it is */
{
    uint32_t q;
    if (fv.ed) { fe_bank(d); return; }   /* (1.18C33: the favorites editor's banks) */
    if (ps_nb(d, &q) > 0) ps_go(q);
    else if (d < 0 && !fv.sec && set.patch < PATCH_INIT) rp_go();   /* (A's OCT-: INIT; the patch edited, asked first) */
    else if (d < 0 && !fv.sec) fv_to((int32_t)PL_RZ - 1);   /* (1.18C33: INIT's OCT-: favorite 1, none: EDIT FAVORITES) */
    else ui.plist_until = plat_ms() + PLIST_MS;   /* (none further: the list's time again) */
}
static UI_LATE void ps_turn(int32_t e)   /* PRESET turned on HOME by e detents: within the patch's bank, the order
                                          * INIT, 001 .. 128 (pb_*: the empty slots left out), not round: below the
                                          * bank's lowest the init patch, the ends stop (0.96: no bank change; up to
                                          * 0.95 INIT, A001 .. E128 over all the banks). 0.91: the first detent after a
                                          * pause shows the patch list alone (the patch stays), the detents while it is
                                          * up browse. 0.96: INIT reached with the patch edited asks first (rp_*) */
{
    const uint32_t now = plat_ms();
    int32_t np, p;
    if (fv.ed) { fe_turn(e); return; }   /* (1.18C33: the favorites editor) */
    if (rp_on()) {                       /* (asked: up takes it back; RESET PATCH's down: RANDOMIZE, after 0.98;
                                          * RANDOMIZE's down (1.18C33): the favorites) */
        if (e > 0) rp_back();
#ifdef HYB_RAND
        else if (ui.rp_ask == 1u) rz_go();
#endif
        else fv_to((int32_t)(ui.rp_ask == 2u ? PL_RZ : PL_INIT) + e);
        return;
    }
    if (!ui.plist_until || now >= ui.plist_until) {
        ui.vl_until = 0;                 /* (a value's list away: the patch list instead) */
        ui.plist_until = now + PLIST_MS;
        ui.plist_new = 1;
        ui.rp_ask = 0;                   /* (a question left by the list's close: gone) */
#ifdef HYB_RAND
        rz_from = 0;                     /* (a new browse) */
#endif
        fv_valid();                      /* (1.18C33: a favorite whose slot was emptied: gone) */
        fv_check();
        ui.last = -1;
        ui.edit_t = 0;
        ui.alt_t = now;                  /* (the patch first) */
        ui.d_lcd = ui.d_rows = 1;
        e += e > 0 ? -1 : 1;
        if (!e) return;
    }
    fv_check();
    if (fv.sec) { fv_to((int32_t)fv.sec - 1 + e); return; }   /* (1.18C33: in the favorites' section) */
    np = (int32_t)pb_n();
    p = ls_acc(5u, (int32_t)pb_pos(PS_AT), e, 1, np - 1);   /* (1.18C35: fast, several a detent within the bank) */
    if (ui.ls_fly && p > 0 && p < np) {  /* (a fast turn: the cursor alone, the patch loaded where it settles) */
        ui.ps_cur = (uint16_t)(pb_patch((uint32_t)p) + 1u);
        ui.plist_until = now + PLIST_MS;
        ui.d_rows = ui.d_lcd = 1;
        return;
    }
    if (ui.ps_cur) ps_go(ui.ps_cur - 1u);   /* (from its place on past the bank's end: that place loaded first) */
    if (p < 0 && !(RZ1 && p == -1)) { fv_to((int32_t)PL_INIT + p); return; }   /* (1.18C33: past RANDOMIZE: the
                                          * favorites' section) */
#ifdef HYB_RAND
    if (p < 0) { rz_go(); return; }     /* (below INIT: RANDOMIZE, the patch up its original) */
#endif
    if (p > np - 1) p = np - 1;          /* (ASSUMED, 0.96: the top end stops at the bank's last patch, no wrap) */
    if (pb_patch((uint32_t)p) == set.patch) { ui.plist_until = now + PLIST_MS; return; }   /* (an end: the list's
                                          * time again, nothing loaded or drawn) */
    if (!p) rp_go(); else ps_go(pb_patch((uint32_t)p));
}
#ifdef HYB_MULTI
static UI_LATE void ln_preset(int32_t e) /* PRESET turned on the launcher: the patch of the part edited (LN_EP: SYNTH 1 or 2,
                                          * as ps_go on HOME: the same load, part B's a block later, the running tempo
                                          * kept) stepped within the bank, no HOME, no patch list: its name on the strip a
                                          * moment, in the part's colour. The drums have no patch (ASSUMED: their kit is
                                          * GLO's, no stepping): a hint. Not down to INIT (its question, rp_*, is HOME's) */
{
    const uint32_t now = plat_ms();
    int32_t p;
    char t[24];
    if (LN_EP == 2u) { app_say("Drums", "NO PATCH"); return; }
    p = (int32_t)pb_pos(set.patch) + e;
    if (p < 1) p = 1;
    if (p > (int32_t)pb_n() - 1) p = (int32_t)pb_n() - 1;
    if (p >= 1 && pb_patch((uint32_t)p) != set.patch) {
        ui.plist_new = ui.msg_until == 0 || now >= ui.msg_until || !ln_vq;   /* (a browse: one undo step) */
        ps_go(pb_patch((uint32_t)p));
        ui.plist_until = 0;              /* (no list over the launcher) */
    }
    t[0] = 0;
    patch_label(t, set.patch, shadow);
    app_say(t, "");
    ui.msg_until = plat_ms() + 1500u;
    ln_vq = (uint8_t)(LN_EP + 1u);       /* (the part's colour: A red, B green) */
    ui.d_lcd = 1;
}
#endif
/* (0.96) INIT reached (PRESET below a bank's lowest patch, OCT- on bank A) with the patch up edited (edits not saved,
 * or a SysEx single in the edit buffer): "RESET PATCH ?" over the patch list (the strip, a box in the list, the cursor
 * on INIT; ui.c ls_ask), the patch still up and sounding. REC loads INIT (an undo step of its own), HOME takes it back
 * (the patch stays, the list's cursor on it again), as does PRESET turned up; a button that closes the list or the
 * time out (RP_MS) the same. Not edited: INIT at once */
#define RP_MS 5000u
static UI_LATE int rp_on(void) { return ui.rp_ask && ui.plist_until && plat_ms() < ui.plist_until; }
static UI_LATE void rp_q(uint32_t k)     /* question k asked (1 RESET PATCH, 2 RANDOMIZE) */
{
    ui.rp_ask = (uint8_t)k;
    ui.plist_until = ui.msg_until = plat_ms() + RP_MS;
    app_say(k == 2u ? "RANDOMIZE" : "RESET PATCH ?", k == 2u ? "REC ROLLS" : "REC");   /* (1.18C33: RANDOMIZE a
                                          * place REC acts on, no question, no box) */
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void rp_go(void)
{
    if (set.patch >= PATCH_INIT) return;
    if (!em_edited(set.patch) && !sx_edit) { ps_go(PATCH_INIT); return; }
    rp_q(1);
}
#ifdef HYB_RAND
/* (after 0.98) RANDOMIZE, the patch list's place below INIT (PRESET turned below INIT, or down from RESET PATCH's
 * question): "RANDOMIZE ?" asked as RESET PATCH is (ui.rp_ask 2: HOME, PRESET up, a button that closes the list, the
 * time out take it back), asked edited or not; the edit buffer as it is then its original (rz_org: the stored patch
 * with its edits, or a single; INIT passed on the way down in this browse, not by RESET PATCH: the patch before it,
 * loaded again, rz_from). REC rolls a variation of the original (each roll from the original, not from the roll
 * before; the question stays up for the next one, its time again), an undo step each (UNDO_RAND: the roll before, the
 * original last). The roll is the edit buffer as a SysEx single is (sx_edit 2: its edits in shadow alone, SAVE stores
 * it, another patch chosen drops it), its name marked "~". Per byte of the panel's parameters (param_of): a number
 * moved by a triangular offset of up to +-20 % of its range (RZ_SMALL, the levels, the amp's times, the detunes:
 * +-8 %; the cutoff, the amp's sustain and the oscillators' volume a quarter of that downwards: never silent); of
 * the lists only the LFOs' and the effects' shapes and the saturation, one in eight times a neighbour; kept: RZ_KEEP
 * (the patch's volume and pan, the arpeggiator's pattern, length and swing, the input) and every other switch and
 * list (modes, routing, key mode, unison, semitones, the matrix's sources and destinations: its amounts vary) */
static uint8_t rz_org[256] HYB_SPARE, rz_sx HYB_SPARE, rz_seed HYB_SPARE;
static uint8_t rz_new HYB_SPARE;         /* (1.18C33) RANDOMIZE reached, no roll yet: the first REC takes the original
                                          * (rz_org) and starts UNDO's steps again; turned past, nothing changes */
static const uint8_t RZ_KEEP[] = {10, 91, 102, PB_(2), PB_(5), PB_(6), PB_(99)};
static const uint8_t RZ_SMALL[] = {26, 33, 34, 36, 37, 38, 59, 60, 61, 63, 105, 113, PB_(42), PB_(44), PB_(85),
                                   PB_(92), PB_(95), PB_(96), PB_(97)};
static UI_LATE int rz_in(const uint8_t *t, uint32_t n, uint32_t b)
{
    while (n--)
        if (t[n] == b) return 1;
    return 0;
}
static UI_LATE void rz_apply(uint32_t seed)      /* roll seed (1..255) of the original into the edit buffer; 0: the
                                          * original again */
{
    uint32_t b, x = seed * 2654435761u | 1u;
    for (b = 0; b < 256u; b++)
        shadow[b] = rz_org[b];
    sx_edit = (uint8_t)(seed ? 2u : rz_sx);
    rz_seed = (uint8_t)seed;
    for (b = 0; seed && b < 240u; b++) {
        const u_param_t *p = param_of(b);
        const int32_t o = shadow[b], mx = p ? (int32_t)p_max(p) : 0;
        int32_t d;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        if (!p || o > mx || rz_in(RZ_KEEP, sizeof RZ_KEEP, b)) continue;
        if (p->kind == V_ENUM) {         /* (a shape, the saturation: now and then a neighbour) */
            if ((p->en == E_LFOSH || p->en == E_FXSH || p->en == E_SAT) && !(x & 7u))
                shadow[b] = (uint8_t)clamp(o + (x & 8u ? 1 : -1), 0, mx);
            continue;
        }
        if (p->kind != V_NUM && p->kind != V_BI && p->kind != V_SHAPE) continue;
        d = ((int32_t)(x >> 8 & 255u) + (int32_t)(x >> 16 & 255u) - 255) * mx
            * (rz_in(RZ_SMALL, sizeof RZ_SMALL, b) ? 8 : 20) / 25500;
        if (d < 0 && (b == 36u || b == 40u || b == 61u)) d /= 4;   /* (never silent) */
        shadow[b] = (uint8_t)clamp(o + d, 0, mx);
    }
    if (seed && rz_org[240] != '~') {    /* (the name marked: "~" before it, its last letter off) */
        for (b = 249u; b > 240u; b--)
            shadow[b] = rz_org[b - 1u];
        shadow[240] = '~';
    }
    shadow_send();
    ui.last = -1;
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void rz_go(void)          /* RANDOMIZE reached, the edit buffer its original (taken at the first
                                          * roll, 1.18C33: rz_new; UNDO's steps start again then, as with a single from
                                          * SysEx) */
{
    if (ui.rp_ask == 2u && rp_on()) return;   /* (there already) */
    if (set.patch >= PATCH_INIT && rz_from) {   /* (INIT passed on the way down: the patch before it, back up) */
        set.patch = (uint16_t)(rz_from - 1u);
        load_patch(set.patch);
    }
    rz_new = 1;
    rp_q(2);
}
#endif
static UI_LATE void rp_back(void)        /* taken back: the patch's list again */
{
    ui.rp_ask = 0;
    ui.msg_until = 0;
    ui.plist_until = plat_ms() + PLIST_MS;
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE int rp_rec(void)          /* REC: INIT loaded (1 = asked); the patch left loses its edits (after 0.98:
                                          * the list brought them back), UNDO alone takes it back with them.
                                          * RANDOMIZE asked: a roll (rz_*) */
{
    const uint32_t p = EM_P(set.patch) + 1u;
    if (!rp_on()) return 0;
#ifdef HYB_RAND
    if (ui.rp_ask == 2u) {
#ifdef APP_HOST_CMD
        static uint32_t host_roll;       /* (host tests: a fixed sequence of rolls, not the clock's: the same patches every run) */
        uint32_t r = ((++host_roll * 37u) & 255u) | 1u;
#else
        uint32_t r = (plat_ticks24() >> 3 & 255u) | 1u;   /* (odd: 128 rolls) */
#endif
        if (rz_new) {                    /* (the first roll here: the original taken now) */
            uint32_t k;
            for (k = 0; k < 256u; k++)
                rz_org[k] = shadow[k];
            rz_sx = sx_edit;
            rz_seed = 0;
            rz_new = 0;
            undo_n = 0;                  /* (no step of an older original's rolls left) */
        }
        if (r == rz_seed) r ^= 2u;       /* (a new one) */
        undo_push(set.patch, UNDO_RAND, rz_seed);
        rz_apply(r);
        ui.plist_until = ui.msg_until = plat_ms() + RP_MS;
        return 1;
    }
#endif
    ui.msg_until = 0;
    ui.plist_new = 1;                    /* (its own undo step back to the patch edited) */
    ps_go(PATCH_INIT);
    ui.rp_park = (uint16_t)p;
#ifdef HYB_RAND
    rz_from = 0;                         /* (reset: RANDOMIZE below it varies INIT) */
#endif
    return 1;
}
static UI_LATE int rp_home(void)         /* HOME: taken back (1 = asked) */
{
    if (!rp_on()) return 0;
    rp_back();
    return 1;
}
/* (1.18C33) the favorites (fv). The patch list over RANDOMIZE: favorite 1 (one
 * detent up from RANDOMIZE) .. favorite n, then EDIT FAVORITES (the lines top down: EDIT FAVORITES, favorite n .. 1,
 * RANDOMIZE, INIT, the bank's patches). A favorite reached loads as PRESET loads a patch (ps_go: its edits kept, the
 * browse's undo step); OCT+ there goes to the first bank, OCT- on INIT (or RANDOMIZE's patch INIT) to favorite 1;
 * REC on a favorite asks "REMOVE FAV ?", REC again removes it; REC on EDIT FAVORITES opens the editor (fe_*): the list
 * in grey (ui.c U_TH_GREY), the favorites in their order (numbered) over the bank's patches, PRESET or SELECT scroll,
 * OCT- / OCT+ the banks (OCT+ from the favorites: the first bank; OCT- from the first bank: the favorites), the patch
 * under the cursor auditioned as stored on part A's engine with its own effects (fv_aud: part B and the drums muted,
 * SEQ A's notes kept from the engine; the panel's buffers, the edit memory, undo, HOLD's switch and the modes never
 * touched), REC adds it (as the last) or removes it, K1 moves a favorite up or down; HOME (or a button that leaves
 * HOME) leaves it: part A's patch as it is on the panel sent again, the list's cursor on EDIT FAVORITES.
 * Stored at FV_OFF in OBJ_LSET (past the launcher's 16 sets: an older firmware reads and writes only those, and
 * plat_store_write keeps the rest of an object) at each change; the beat running, at its stop ("SAVE PENDING", the
 * patterns' rule); a move FV_MOVE_MS after the last one, or as the editor is left. A favorite is a slot's number: the
 * slot written over keeps it (the new patch, its name), a slot emptied (the bank organizer, a bank replaced) drops it
 * (fv_valid, as the list opens) */
#define FV_OFF 3072u                     /* (LS_SLOTS * LS_REC: the launcher's sets before it) */
#define FV_REC (4u + 2u * FV_N)          /* "F" "V" FV_V1 n, then n patch numbers (low byte first), zeros */
#define FV_M0 0x46u
#define FV_M1 0x56u
#define FV_V1 1u                         /* (a later version keeps this much as it is: read as FV_V1, any version) */
#define FV_MOVE_MS 1000u
#define FV_ASK_MS 4000u
#ifdef LS_SLOTS
_Static_assert(FV_OFF == LS_SLOTS * LS_REC && FV_OFF + FV_REC <= LS_MAXB, "the favorites past the sets, short of 0xF00");
#endif
static UI_LATE int fv_find(uint32_t p)   /* p's favorite (0 ..), -1 none */
{
    uint32_t k;
    for (k = 0; k < fv.n; k++)
        if (fv.p[k] == p) return (int)k;
    return -1;
}
static UI_LATE void fv_drop(uint32_t k)
{
    for (; k + 1u < fv.n; k++)
        fv.p[k] = fv.p[k + 1u];
    fv.n--;
    fv.wr |= 2u;
}
static UI_LATE int fv_put(void)          /* the favorites into the store now: 0 = done */
{
    uint8_t r[FV_REC];
    uint32_t k;
    for (k = 0; k < FV_REC; k++)
        r[k] = 0;
    r[0] = FV_M0; r[1] = FV_M1; r[2] = FV_V1; r[3] = fv.n;
    for (k = 0; k < fv.n; k++) {
        r[4u + 2u * k] = (uint8_t)fv.p[k];
        r[5u + 2u * k] = (uint8_t)(fv.p[k] >> 8);
    }
    return store_write(OBJ_LSET, FV_OFF, r, FV_REC);
}
static UI_LATE void fv_load(void)        /* app_init: the favorites from the store (none there: none) */
{
    uint8_t r[FV_REC];
    uint32_t k, q;
    fv.n = 0;
    if (plat_store_read(OBJ_LSET, FV_OFF, r, FV_REC) != (int)FV_REC || r[0] != FV_M0 || r[1] != FV_M1) return;
    for (k = 0; k < r[3] && k < FV_N; k++)
        if ((q = (uint32_t)r[4u + 2u * k] | (uint32_t)r[5u + 2u * k] << 8) >= PU_0 && q < PATCH_INIT)
            fv.p[fv.n++] = (uint16_t)q;  /* (a slot's number; filled or not: fv_valid) */
}
static UI_LATE void fv_valid(void)       /* the favorites whose slot is empty now: dropped (written) */
{
    uint32_t k = fv.n;
    while (k--)
        if (!patch_filled(fv.p[k])) fv_drop(k);
}
static UI_LATE void fv_check(void)       /* the list's cursor in the section: still the patch up's favorite */
{
    if (fv.sec > fv.n + 1u || (fv.sec > 1u && fv.p[fv.n + 1u - fv.sec] != set.patch)) fv.sec = fv.ask = 0;
}
static UI_LATE void fv_said(const char *a)   /* the strip: a, and the write waiting if the beat runs */
{
    const int r = sp_runs();
    if (r) fv.wr |= 4u;
    app_say(a, r ? "SAVE PENDING" : "");
}
static UI_LATE void fv_pbuf(const uint8_t *b)   /* patch b (the panel's form) to part A's engine at its next block,
                                          * the tempo running kept */
{
    uint32_t k;
    patch_req = -1;
    RING_PUBLISH();
    for (k = 0; k < 256u; k++)
        pbuf[k] = (uint8_t)SHX_EB(k, b[k]);
    if (tp_run) pbuf[TP_BYTE] = (uint8_t)(tp_run - 1u);
    RING_PUBLISH();
    patch_req = PATCH_REQ_BUF;           /* (part A) */
}
static UI_LATE void fv_play(uint32_t p)  /* the audition: patch p as stored on part A (its extension to the reverb) */
{
#ifdef APP_HOST_CMD
    host_pl++;
#endif
    fv_pbuf(patch_bytes(p));
#ifdef HYB_SHIMMER
    {
        const uint8_t *x = patch_ext(p);
        uint32_t k;
        for (k = 0; k < SHX_N; k++)
            shx_eng[k] = x[k];
        shx_eng[SHX_TAIL] = shx_tl(x, x[SHX_GX]);
        for (k = 0; k < SHX_NR; k++)
            shx_eng[SHX_N + k] = (uint8_t)(x[SHX_N + k] >= 1u && x[SHX_N + k] <= SHX_NT ? x[SHX_N + k] : 0u);
        RING_PUBLISH();
        shx_seq++;
    }
#endif
}
/* the editor's list: the favorites (top down n .. 1), then bank fv.eb's filled slots */
static UI_LATE uint32_t fe_nb(void)
{
    uint32_t q, n = 0;
    for (q = fv.eb * PA_N; q < fv.eb * PA_N + PA_N; q++)
        n += (uint32_t)patch_filled(q);
    return n;
}
static UI_LATE uint32_t fe_patch(uint32_t i)
{
    uint32_t q;
    if (i < fv.n) return fv.p[fv.n - 1u - i];
    i -= fv.n;
    for (q = fv.eb * PA_N; q < fv.eb * PA_N + PA_N; q++)
        if (patch_filled(q) && !i--) return q;
    return PATCH_INIT;
}
static UI_LATE uint32_t fe_place(uint32_t p)   /* p's line in its bank's part (that bank shown) */
{
    uint32_t q, n = fv.n;
    fv.eb = (uint8_t)(p / PA_N);
    for (q = fv.eb * PA_N; q < p; q++)
        n += (uint32_t)patch_filled(q);
    return n;
}
static UI_LATE void fe_show(void)        /* the patch under the cursor heard (the strip: its hint) */
{
    fv_play(fe_patch(fv.ei));
    ui.msg_until = 0;
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void fe_turn(int32_t e)   /* PRESET, SELECT: the cursor */
{
    const int32_t n = (int32_t)(fv.n + fe_nb()), f = fv.ei < fv.n;
    int32_t i = ls_acc(5u, fv.ei, e, f ? 0 : (int32_t)fv.n, f ? (int32_t)fv.n - 1 : n - 1);   /* (1.18C35: fast, several
                                          * a detent within the favorites or the bank, heard where it settles) */
    if (i > n - 1) i = n - 1;
    if (i < 0) i = 0;
    if (i == (int32_t)fv.ei) return;
    fv.ei = (uint8_t)i;
    if (ui.ls_fly) ui.d_rows = ui.d_lcd = 1; else fe_show();
}
static UI_LATE void fe_bank(int32_t d)   /* OCT- / OCT+: the bank before / after (the favorites: the first bank; the
                                          * first bank's OCT-: favorite 1) */
{
    uint32_t q;
    const int f = fv.ei < fv.n;
    if (ps_nb_from(f ? PATCH_INIT : fe_patch(fv.ei), d, &q) > 0) fv.ei = (uint8_t)fe_place(q);
    else if (d < 0 && !f && fv.n) fv.ei = (uint8_t)(fv.n - 1u);
    else return;
    fe_show();
}
static UI_LATE void fe_k1(int32_t e)     /* K1: the favorite under the cursor moved down the list (right: a lower
                                          * number) or up, a place a detent */
{
    uint32_t j = fv.n - 1u - fv.ei;      /* (its place in the order) */
    if (fv.ei >= fv.n) return;           /* (a bank's patch: nothing to move) */
    for (; e > 0 && j > 0u; e--, j--) { const uint16_t m = fv.p[j]; fv.p[j] = fv.p[j - 1u]; fv.p[j - 1u] = m; }
    for (; e < 0 && j + 1u < fv.n; e++, j++) { const uint16_t m = fv.p[j]; fv.p[j] = fv.p[j + 1u]; fv.p[j + 1u] = m; }
    fv.ei = (uint8_t)(fv.n - 1u - j);
    fv.wr |= 1u;
    fv.t = plat_ms();
    ui.msg_until = 0;
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void fe_rec(void)         /* REC: the patch under the cursor a favorite (the last) or not */
{
    const uint32_t p = fe_patch(fv.ei);
    const int k = fv_find(p);
    if (k >= 0) {
        fv_drop((uint32_t)k);
        if (fv.ei >= fv.n + 1u) fv.ei--;   /* (in the bank: the same patch) */
        else fe_show();                  /* (a favorite: the next one down, heard) */
        fv_said("FAV REMOVED");
    } else if (fv.n >= FV_N) {
        app_say("FAVORITES", "FULL");
        return;
    } else {
        fv.p[fv.n++] = (uint16_t)p;
        fv.ei++;
        fv.wr |= 2u;
        fv_said("FAV ADDED");
    }
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void fe_open(void)        /* EDIT FAVORITES: the editor, on the patch up (a user patch) or the first */
{
    uint32_t q = set.patch;
    fv_valid();
    if ((q < PU_0 || q >= PATCH_INIT) && ps_nb_from(PATCH_INIT, 1, &q) <= 0) {
        app_say("Favorites", "NO PATCHES");
        return;
    }
    all_off_both();                      /* (the keys' and HOLD's notes end, as at a patch change; HOLD stays) */
    fv.ask = 0;
    fv.ed = 1;
    fv_aud = 1;
    dr_level();                          /* (the drums muted) */
    fv.ei = (uint8_t)fe_place(q);
    fe_show();
    ui.msg_until = 0;
    ui.plist_until = plat_ms() + PLIST_MS;
}
static UI_LATE void fv_leave(void)       /* the editor left: part A as the panel has it again */
{
    uint32_t k;
    if (!fv.ed) return;
    for (k = 0; k < NKEYS; k++)          /* (a key still down: its audition's note off) */
        if ((ui.keys >> k & 1u) && ui.held[k] != KEY_NONE) {
            note_off_out(ui.held[k]);
            ui.held[k] = KEY_NONE;
        }
    fv.ed = 0;
    fv_aud = 0;
#ifdef HYB_MULTI
    fv_pbuf(ed_part ? shadow_o : shadow);
#else
    fv_pbuf(shadow);
#endif
    SHX_SYNC();
    dr_level();
    fv.t = plat_ms() - FV_MOVE_MS;       /* (a move: written now) */
    fv.sec = 1u;
    ui.msg_until = 0;
    if (ui.view == V_QUICK && !ui.qpos) ui.plist_until = plat_ms() + PLIST_MS;   /* (HOME: the list again; left by
                                          * another button, fv_idle: its view) */
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void fv_to(int32_t i)     /* the patch list's cursor to line i, from the favorites' section or into it */
{
    const uint32_t now = plat_ms();
    ui.rp_ask = fv.ask = 0;
    ui.msg_until = 0;
    ui.plist_until = now + PLIST_MS;
    if (i < 0) i = 0;
    if ((uint32_t)i >= PL_RZ) {          /* (down out of it: RANDOMIZE, INIT, the bank) */
        fv.sec = 0;
#ifdef HYB_RAND
        if ((uint32_t)i == PL_RZ) { rz_go(); return; }
#endif
        i -= (int32_t)PL_INIT;
        if (i > (int32_t)pb_n() - 1) i = (int32_t)pb_n() - 1;
        if (!i) rp_go(); else ps_go(pb_patch((uint32_t)i));
        ui.d_rows = ui.d_lcd = 1;
        return;
    }
    if (i && fv.p[fv.n - (uint32_t)i] != set.patch) ps_go(fv.p[fv.n - (uint32_t)i]);   /* (a favorite: loaded) */
    fv.sec = (uint8_t)(i + 1);
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE int fv_rec(void)          /* REC: the editor's; the patch list's favorites' section (1 = taken) */
{
    const uint32_t now = plat_ms();
    uint32_t k;
    if (fv.ed) { fe_rec(); return 1; }
    fv_check();
    if (!fv.sec || !ui.plist_until || now >= ui.plist_until) return 0;
    if (fv.sec == 1u) { fe_open(); return 1; }
    k = fv.n + 1u - fv.sec;
    ui.d_rows = ui.d_lcd = 1;
    if (fv.ask != k + 1u) {              /* (asked first) */
        fv.ask = (uint8_t)(k + 1u);
        fv.t = now;
        app_say("REMOVE FAV ?", "REC");
        ui.msg_until = ui.plist_until = now + FV_ASK_MS;
        return 1;
    }
    fv_drop(k);
    fv.ask = fv.sec = 0;                 /* (the patch stays up: the cursor on its place in the bank) */
    fv_said("FAV REMOVED");
    ui.plist_until = now + PLIST_MS;
    return 1;
}
static UI_LATE int fv_home(void)         /* HOME: the editor left, the question taken back (1 = taken) */
{
    if (fv.ed) { fv_leave(); return 1; }
    if (!fv.ask || !ui.plist_until) return 0;
    fv.ask = 0;
    ui.msg_until = 0;
    ui.plist_until = plat_ms() + PLIST_MS;
    ui.d_rows = ui.d_lcd = 1;
    return 1;
}
static UI_LATE void fv_idle(uint32_t now)   /* app_frame: the editor kept up, the question's time, the write */
{
    if (fv.ed) {
        if (ui.view != V_QUICK || ui.qpos) fv_leave();   /* (HOME left another way) */
        else ui.plist_until = now + PLIST_MS;
    }
    if (fv.ask && now - fv.t >= FV_ASK_MS) { fv.ask = 0; ui.d_rows = ui.d_lcd = 1; }
    if (ui.ls_fly && (!ui.plist_until || (int16_t)(uint16_t)(now - ui.ls_fly) >= (int16_t)LS_SETTLE)) {   /* (1.18C35: a fast turn
                                          * settled: its line loaded, heard; the list closed before: as it is;
                                          * signed, 1.18C46: ls_fly is now | 1, a pass in the same millisecond saw
                                          * 0xFFFF and settled it at once) */
        const uint32_t q = ui.ps_cur;
        ui.ls_fly = ui.ps_cur = 0;
        if (ui.plist_until) { if (fv.ed) fe_show(); else if (q) ps_go(q - 1u); }
    }
    if ((fv.wr & 3u) && !((fv.wr & 1u) && now - fv.t < FV_MOVE_MS)) {
        if (sp_runs()) {
            if (!(fv.wr & 4u)) { fv.wr |= 4u; app_say("Favorites", "SAVE PENDING"); }
        } else {
            const uint32_t said = fv.wr & 4u;
            fv.wr = 0;
            if (fv_put()) app_say("Favorites", "SAVE FAILED");
            else if (said) app_say("Favorites", "SAVED");
        }
    }
}
static UI_LATE void pl_lcd(char *a, char *v)   /* the strip over the patch list (1.18C33: the hints) */
{
    if (fv.ed) {
        const uint32_t p = fe_patch(fv.ei);
        if (fv.ei < fv.n) {
            text_cat(a, "FAV ");
            put_sint(a + 4, (int32_t)(fv.n - fv.ei), 0);
            text_cat(v, "K1: MOVE");
        } else {
            patch_tag(a, p);
            text_cat(v, fv_find(p) >= 0 ? "REC: DEL" : "REC: ADD");
        }
        return;
    }
    if (fv.sec == 1u) { text_cat(a, "EDIT FAVORITES"); text_cat(v, "REC"); return; }
#ifdef HYB_RAND
    if (ui.rp_ask == 2u && rp_on()) { text_cat(a, "RANDOMIZE"); text_cat(v, "REC ROLLS"); return; }
#endif
    patch_line(a);
}
static UI_LATE void pl_foot(char *l, char *r)   /* the foot strip: the banks' letters OCT- / OCT+ go to ("" none;
                                          * 1.18C33: "FAV" to the favorites; the favorites editor's: its banks) */
{
    uint32_t q;
    char t[5];
    const int f = fv.ed && fv.ei < fv.n;
    const uint32_t p = !fv.ed ? (fv.sec ? PATCH_INIT : set.patch) : f ? PATCH_INIT : fe_patch(fv.ei);
    l[0] = r[0] = l[1] = r[1] = 0;
    if (ps_nb_from(p, -1, &q) > 0) { patch_tag(t, q); l[0] = t[0]; }
    else if (fv.ed ? !f && fv.n : !fv.sec && set.patch >= PATCH_INIT) text_cat(l, "FAV");
    if (ps_nb_from(p, 1, &q) > 0) { patch_tag(t, q); r[0] = t[0]; }
}
static UI_LATE void pl_set(void)         /* the patch list as ls's (asked: the cursor on INIT; the box, ls_body;
                                          * RANDOMIZE's place over INIT, 1.18C33: the favorites' section over it; the
                                          * favorites editor's list) */
{
    if (fv.ed) ls_set(pl_item, fv.n + fe_nb(), fv.ei);
    else {
        fv_check();
        ls_set(pl_item, pb_n() + PL_INIT, fv.sec ? fv.sec - 1u : rp_on() ? (ui.rp_ask == 2u ? PL_RZ : PL_INIT)
                                                                        : pb_pos(PS_AT) + PL_INIT);
    }
    ls_foot = pl_foot;
}
static void octave(int32_t d);
static UI_LATE void oct_tap(int32_t d)   /* OCT- / OCT+ tapped off the name editor and the sequencers' pages: the
                                          * octave; the patch list up, the bank; SAVE's slot list (1.00), its bank as K1
                                          * (the same slot) */
{
    if (ui.view == V_LNCH) return;       /* (1.18C17: the launcher: no octave change) */
    if (ui.view == V_SAVE) sv_knob(0u, d);
    else if (ui.plist_until) ps_bank(d);
    else octave(d);
}
static UI_LATE uint32_t oct_lights(void) /* OCT- / OCT+ lit: the octave (off at 0); the patch list up, the banks (OCT-
                                          * dark on INIT); SAVE's slot list, the banks there are before / after (1.00) */
{
    if (ui.view == V_LNCH) return 0u;    /* (the launcher: dark) */
    if (ui.view == V_SAVE)
        return (ui.scur >= UB_SLOTS ? 1u << B_OCTDN : 0u) | (ui.scur + UB_SLOTS < UB_N ? 1u << B_OCTUP : 0u);
    if (ui.plist_until) {                /* (1.18C8: lit where a bank is to go to; 1.18C33: the favorites too) */
        char l[4], r[4];
        pl_foot(l, r);
        return (l[0] ? 1u << B_OCTDN : 0u) | (r[0] ? 1u << B_OCTUP : 0u);
    }
    return set.oct < 0 ? 1u << B_OCTDN : set.oct > 0 ? 1u << B_OCTUP : 0u;
}
static UI_LATE void sq_title(uint32_t w)         /* the parameter turned (a column, SQ_TT_TEMPO): up in the strip */
{
    sqv.sp_ask = 0;                      /* (a knob turned: SAVE's question taken back) */
    sqv.tt = (uint8_t)w;
    sqv.tt_t = plat_ms() | 1u;
}
static UI_LATE void seq_blk_fix(void)            /* the blocks shown within the patterns' lengths */
{
    if (sqv.blk[0] >= 1u << sqp.lc) sqv.blk[0] = (uint8_t)((1u << sqp.lc) - 1u);
    if (sqv.blk[1] >= 1u << drp.lc) sqv.blk[1] = (uint8_t)((1u << drp.lc) - 1u);
    sqv.sel = (uint8_t)(sqv.blk[ui.view == V_DRUM] * SQ_N + (sqv.sel & (SQ_N - 1u)));   /* (the step shown: in the
                                          * block shown, at its key) */
}
static UI_LATE void seq_blk(int32_t d)           /* OCT- / OCT+ on a sequencer's page: the block of 16 steps the white
                                          * keys show, left / right within the pattern's length */
{
    const uint32_t p = ui.view == V_DRUM, n = 1u << (p ? drp.lc : sqp.lc);
    const int32_t b = clamp((int32_t)sqv.blk[p] + d, 0, (int32_t)n - 1);
    if (!p && (SQ_CE || sqv.arec)) { octave(d); return; }   /* (CHORD ENTRY, 1.03; RECORD armed or running, 1.19.7:
                                          * the keys are notes there: the keyboard's octave, as on HOME, its message
                                          * too. The notes down keep their pitch for their off, sr.note) */
    if (b == sqv.blk[p]) return;
    sqv.blk[p] = (uint8_t)b;
    sqv.sel = (uint8_t)(b * (int32_t)SQ_N + (sqv.sel & (SQ_N - 1u)));
    if (sqv.key) sqv.key = 0;            /* (a step key down: its block left, its release toggles nothing) */
}
static UI_LATE void seq_none(int drums, int none);
#ifdef HYB_MULTI
static UI_LATE void sq_key_end(uint32_t n)       /* a key's note sounding in the part of the sequencer shown, about to
                                          * start: off there (out over MIDI too unless it goes on in the other part) */
{
    const uint32_t to = sq_parts_key(n), run = (sqr[0].run ? 1u : 0u) | (sqr[1].run ? 2u : 0u);
    if (!(to >> sq_mi & 1u) || (run >> sq_mi & 1u)) return;
    ev_put(sq_mi ? EV_KEY_B : EV_KEY_A, n, 0);
    if (!(to & ~run & ~(1u << sq_mi))) key_send(0x80u, n, 0);
}
#endif
static UI_LATE void sq_hold(int h);
static UI_LATE void sq_start(void)               /* on: the keys' notes sounding now stop (they are its input now) */
{
    uint32_t k;
    if (sq.run) return;
#ifdef HYB_MULTI
    for (k = 0; k < 128u; k++) {         /* (its part's alone: the other's go on) */
        uint32_t j;
        for (j = 0; j < NKEYS && !((ui.keys >> j & 1u) && ui.held[j] == k); j++)
            ;
        if (j < NKEYS || hold_note(k)) sq_key_end(k);
        if (hold_po(k)) {                /* (1.15: CHANNELS, the other part's HOLD's notes, if it is this one's) */
            ed_part ^= 1u;
            sq_key_end(k);
            ed_part ^= 1u;
        }
    }
#else
    for (k = 0; k < NKEYS; k++)
        if ((ui.keys >> k & 1u) && ui.held[k] != KEY_NONE) note_off_out(ui.held[k]);
    for (k = 0; k < 128u; k++)
        if (hold_note(k)) note_off_out(k);
#endif
    sq_keys_put(ui.keys);
    sq_hold(sq_hold_want(1u));           /* (its part's arpeggiator off while it runs; PATCH ARP ON: on) */
    sq_pv_cut();                         /* (a step's preview: ended before the first step, sq_pv_cut_a) */
    pvr.k = 0;                           /* (one waiting: dropped) */
    sq.run = 1;                          /* (the audio interrupt: the first step) */
}
static UI_LATE void sq_stop(void)
{
    if (!sq.run) return;
    sq.run = 0;                          /* (the audio interrupt: its notes off at the next block) */
    sq_hold(sq_hold_want(0u));           /* (its part's patch's arpeggiator back, unless RECORD is armed on it with
                                          * PATCH ARP OFF; 0.94: after its notes' offs, sq_hold_sync) */
    sq.keys[0] = sq.keys[1] = sq.keys[2] = sq.keys[3] = 0;
}
static UI_LATE void sq_hold(int h)       /* the sequencer shown running (h) or stopped: its part's engine arpeggiator
                                          * held off (B 1 at 0 in the engine through the parameter path, the patch keeps
                                          * its byte; the ARP menu shows "SEQ") or given the patch's mode back */
{
    if (sq.hold == (uint8_t)(h != 0)) return;
    sq.hold = (uint8_t)(h != 0);         /* (before the event: the audio interrupt holds B 1 at 0 from then on) */
#ifdef HYB_MULTI
    ev_put(sq_mi ? EV_PARAM_B : EV_PARAM_A, 129u, h ? 0u : sq_mi == ed_part ? shadow[129] : shadow_o[129]);   /* (its
                                          * part's, whichever part is edited) */
#else
    ev_put(EV_PARAM, 129u, h ? 0u : shadow[129]);
#endif
}
#ifdef HYB_MULTI
static UI_LATE void sq_on_i(uint32_t i, int on)  /* SEQ A (0) or B (1) on or off, whichever is shown */
{
    const uint8_t m = sq_mi;
    sq_mi = (uint8_t)i;
    if (on) sq_start(); else sq_stop();
    sq_mi = m;
}
static UI_LATE void sq_stop_all(void) { sq_on_i(0, 0); sq_on_i(1, 0); }
static UI_LATE int32_t sq_other(void)            /* the pattern the other arp sequencer uses, -1 none (NONE; 1.19.3:
                                          * SEQ B in SINGLE too, its MIDI-only sequencer: 1.18C19 .. 1.19.2 none there) */
{
    return sq_mi ? (set.sq_pad & SQ_DISARM ? -1 : (int32_t)set.sq_pat) : (set.sqb_pad & SQ_DISARM ? -1 : (int32_t)set.sqb_pat);
}
static UI_LATE int sq_busy(void)                 /* the one shown is NONE and the pattern it keeps the other's: not
                                          * edited here (1: said so) */
{
    char t[12];
    if (!(SQ_PADV & SQ_DISARM) || (int32_t)SQ_PATV != sq_other()) return 0;
    app_say(pat_text(t, "Pattern ", SQ_PATV + 1), sq_mi ? "IN USE BY A" : "IN USE BY B");
    return 1;
}
static UI_LATE int32_t sq_pat_step(int32_t p, int32_t e)   /* PATTERN turned e detents from p (-1 NONE): the other's
                                          * pattern skipped, the ends stop */
{
    const int32_t u = sq_other();
    while (e) {
        const int32_t d = e > 0 ? 1 : -1;
        int32_t q = p + d;
        if (q == u && u >= 0) q += d;   /* (NONE is always there) */
        if (q < -1 || q > (int32_t)SQ_PATS - 1) break;
        p = q;
        e -= d;
    }
    return p;
}
static UI_LATE void sq_show(uint32_t i)          /* the page shows SEQ A (0) or B (1), each its block shown (the
                                          * edits in the journal, un_note); the colours follow (ui_draw) */
{
    if (i != sq_mi) {
        const uint8_t b = sqv.blk[0];
        sqv.blk[0] = sqv.blk_o;
        sqv.blk_o = b;
        sq_mi = (uint8_t)i;
    }
    if ((SQ_PADV & SQ_DISARM) && (int32_t)SQ_PATV == sq_other())
        sq_load(SQ_PATV);                /* (NONE, its pattern the other's: shown as the other has it) */
    sqv.key = sqv.clr = sqv.tt = sqv.menu = 0;
    seq_blk_fix();
    view_reset();
}
#else
#define sq_stop_all() sq_stop()
#endif
static UI_LATE void sq_page(uint32_t i)          /* the arp sequencer's page shown (SEQ's way), SEQ i (A 0, B 1;
                                          * SINGLE: A) in the USB build */
{
    sqv.key = sqv.clr = sqv.tt = 0;
    ui.view = V_SEQ;
#ifdef HYB_MULTI
    sq_show(i);
#else
    (void)i;
    seq_blk_fix();
    view_reset();
#endif
    sqv.last = 1;
}
static UI_LATE void kit_seq(void);
static UI_LATE void dr_page(void)                /* the drums' page shown (SEQ's way, the kit's SEQ): SEQ's last page */
{
    sqv.key = sqv.clr = sqv.tt = 0;
    sqv.last = 2;
    ui.view = V_DRUM;
    seq_blk_fix();
    view_reset();
}
#ifdef HYB_MULTI
static uint8_t sq_retv;                 /* RAM only: 1 when the launcher was the sequencer view left last (the others: sqv.last, sq_mi), SEQ's way back */
#endif
static UI_LATE void sq_button(int shift)         /* SEQ: the arp sequencer's page, then the drum sequencer's, round
                                          * again (0.79: no way back; HOME or another button leaves); pressed off them
                                          * the last of these shown (sqv.last, 0.77) */
{
    if (shift) {                         /* SHIFT + SEQ: both sequencers off (PLAY does that) */
        ui.shift_used = 1;
        sq_stop_all();
        dr.run = 0;
        ln.emp = 0;
        app_say("Seq", "STOP");          /* (the transport, as PLAY says it; 0.76: was "OFF") */
    } else if (ui.view == V_DRUM)        /* (the way round: the arp sequencer's page again, the first one) */
#ifdef HYB_MULTI
        sq_page(sqv.first);
#else
        sq_page(0u);
#endif
    else if (ui.view == V_KIT)           /* (the kit: the drums' page) */
        kit_seq();
#ifdef HYB_MULTI
    else if (ui.view == V_LNCH) {        /* (the launcher: the drums' page; 1.18C21: a part chosen there by its key held, that
                                          * part's sequencer) */
        if (ln.esel && !ln.epd) { ln.esel = 0; sq_page(sqv.first); }   /* (1.19.3: SINGLE too, SEQ B's MIDI-only
                                          * page with SYNTH 2's key: ln_esel) */
        else { ln.esel = 0; dr_page(); }
    }
#endif
    else if (ui.view == V_SEQ) {         /* the drums' page */
#ifdef HYB_MULTI
        if (sq_mi == sqv.first && (SQB_CYC || !SQ_BMO)) {   /* (the other part's arp sequencer first; 1.19.3:
                                          * SINGLE too, SEQ B MIDI only) */
            sq_show(!sq_mi);
            return;
        }
        if (LN_CYC) ln_open();           /* (1.18C11: then the pattern launcher, then the drums' page again) */
        else dr_page();
#else
        dr_page();
#endif
    } else {                             /* the page (the last shown: its way on from there) */
        if (ui.view == V_GLO && ui.gch == G_SEARCH) gd_close();
        overlay_off();
        pl_n = 0;                        /* (pl_tab's room: the pages' parts now; PLAY mode builds its list again) */
        sqv.menu = 0;                    /* (an EDIT list left up: closed) */
#ifdef HYB_MULTI
        if (sq_retv && LN_CYC) { ln_open(); return; }   /* (the launcher left last: back to it; the other sequencer views: sqv.last below) */
#endif
        if (sqv.last == 2) {
            dr_page();
            return;
        }
#ifdef HYB_MULTI
        if (!sqv.last)                   /* (the way's start: the part edited's first; else the one shown last) */
            sqv.first = ed_part;
        sq_page(!sqv.last ? sqv.first : sq_mi);
#else
        sq_page(0u);
#endif
    }
}
static UI_LATE int seq_none_key(uint32_t k);
static UI_LATE int sq_ch_key(uint32_t k, int down);
static UI_LATE void sq_ce_begin(void);
static UI_LATE int sq_ch_put(void);
static UI_LATE void sq_key(uint32_t k, int down)   /* a key on the page: a white key a step of the block shown (CHORD
                                          * ENTRY: every key a note of the chord, 1.03) */
{
    uint32_t i, s;
    if (sq_ch_key(k, down)) return;
    for (i = 0; i < SQ_N && SQ_KEY[i] != k; i++)
        ;
    if (down && seq_none_key(k)) return;   /* (PATTERN NONE: a note) */
    if (i == SQ_N) return;               /* (a black key) */
#ifdef HYB_MULTI
    if (down && sq_busy()) return;
#endif
    s = sqv.blk[0] * SQ_N + i;
    if (down) {
        sqv.key = (uint8_t)(i + 1u);
        sqv.sel = (uint8_t)s;
        sqv.edited = sqv.clr = sqv.ce = sqv.lin = sqv.aud = 0;
        sqv.dzw = 0;                     /* (a new step: its 0s sticky afresh) */
        sqv.down_t = plat_ms();
    } else if (sqv.key == i + 1u && !sqv.lin) {
        if (!sqv.edited && sqv.aud != SQ_AUD_ON && plat_ms() - sqv.down_t < SQ_TAP_MS) {   /* (1.18C44: a hold heard,
                                          * sq_aud_idle: nothing toggled) */
            if (sqp.st[s].ev & SQ_ON) sq_step0(&sqp.st[s]);   /* (off: its values as shipped, so it comes on
                                          * fresh) */
            else { sqp.st[s].ev |= SQ_ON; sq_pv_play(&sqp.st[s], SQ_PV_MS, 0u); }   /* (on: heard, 0.85) */
            sq_dirty(s);
        }
        if (sqv.edited) {
            sqv.lin = 1; sqv.lin_t = (uint16_t)plat_ms();
            if (!sq_tt_on()) sqv.tt = 0;   /* (1.18C33: the roll gone back to the columns while held: not again) */
        }                                /* (1.18C31: changed: its view kept
                                          * SQ_TT_MS, sq_idle; a CHORD step's let go no longer opens CHORD ENTRY, 1.03
                                          * to 1.18C30 it did: EVENT turned does, sq_knob) */
        else sqv.key = 0;
    }
}
#define SQ_DZ 2                          /* the detents 0 holds on ALIGN and OFFSET before it lets go (0.78) */
static UI_LATE int32_t sq_dz(int32_t v, int32_t e, uint32_t w)   /* v turned by e, 0 sticky: a turn through 0 stops
                                          * there; from 0 SQ_DZ detents more to leave it (column w's: sqv.dz) */
{
    const int32_t n = v + e;
    if (sqv.dzw != w) sqv.dz = 0;
    sqv.dzw = (uint8_t)w;
    if (v) { sqv.dz = 0; return (v ^ n) < 0 ? 0 : n; }
    if ((sqv.dz ^ e) < 0) sqv.dz = 0;    /* (turned back: afresh) */
    sqv.dz = (int8_t)clamp(sqv.dz + e, -SQ_DZ - 1, SQ_DZ + 1);
    if (sqv.dz > SQ_DZ || sqv.dz < -SQ_DZ) { sqv.dz = 0; return e < 0 ? -1 : 1; }
    return 0;
}
/* (0.96) PATTERN turned while the transport runs (sq_qt, sq_qr: the audio interrupt's side above sq_tick): the
 * sequencer playing waits for its bar's line, the one joining from NONE starts silent and plays from its first line;
 * the PATTERN column shows the target meanwhile, in the accent (0.97: no blinking title, nothing drawn again while
 * it waits). The knob turned again moves the target, the pattern playing chosen again takes it back; stopped, at once
 * as before */
static UI_LATE int sq_qrun(uint32_t q)           /* sequencer q (0, 1 the arp sequencers, 2 the drums) runs */
{
#ifdef HYB_MULTI
    return q == 2u ? dr.run != 0 : sqr[q & 1u].run != 0;
#else
    return q == 2u ? dr.run != 0 : sq.run != 0;
#endif
}
static UI_LATE int sq_qload(uint32_t q, int32_t n)   /* sequencer q's pattern n (-1: none, the one before kept)
                                          * loaded in place now (the audio interrupt plays nothing of it before the
                                          * line, or it is stopped) with its edits (the journal; the one left keeps
                                          * its own there, nothing written), its number (1 = done; 0: the other arp
                                          * sequencer's pattern meanwhile) */
{
    if (q == 2u) {
        if (n >= 0) { set.dr_pat = (uint8_t)n; dr_load_to(&drp, (uint32_t)n); un_load(0u); }
        if (ui.view == V_DRUM) seq_blk_fix();
    } else {
#ifdef HYB_MULTI
        const uint8_t mi = sq_mi;
        const int32_t o = q ? (set.sq_pad & SQ_DISARM ? -1 : (int32_t)set.sq_pat)
                            : (set.sqb_pad & SQ_DISARM ? -1 : (int32_t)set.sqb_pat);   /* (the other's) */
        if (n >= 0 && n == o && !(sq_qr[!q] == SQ_QR_NONE || (sq_qr[!q] == SQ_QR_SWAP && sq_qt[!q] != (uint32_t)n + 1u)))
            return 0;                    /* (taken by the other meanwhile; 1.18C10: unless it leaves it at this line, a
                                          * launcher row's exchange) */
        sq_mi = (uint8_t)q;
        if (n >= 0) { SQ_PATV = (uint8_t)n; sq_load_to(&sqp, (uint32_t)n); un_load(1u + q); }
        sq_mi = mi;
        if (q == mi && ui.view == V_SEQ) seq_blk_fix();
#else
        if (n >= 0) { set.sq_pat = (uint8_t)n; sq_load_to(&sqp, (uint32_t)n); un_load(1u); }
        if (ui.view == V_SEQ) seq_blk_fix();
#endif
    }
    glo_touch();
    ui.d_lcd = 1;                        /* (the page: its parts whose signatures changed alone, seq_draw; 0.96
                                          * drew it all again here, a flicker on the device: 0.97) */
    return 1;
}
static UI_LATE void sq_qoff(uint32_t q)          /* the request done or taken back (the strip again; the PATTERN
                                          * column by its signature) */
{
    sq_qr[q] = SQ_QR_OFF;
    sq_qm[q] = 0;
    sq_qt[q] = 0;
    ui.d_lcd = 1;
}
/* (after 0.99) the drums' change while they run, sample-exact on the line however busy the main loop: 0.96-0.99
 * loaded the pattern in place in the bar's last step, after its hits; a main loop late there (a frame, the screen,
 * the store's reads; at 4x or a late ALIGN that step's rest is a few ms) played step 1 late and dropped the steps
 * the wait took, and the pattern left was saved at once (a flash erase: the audio stopped). Now sq_qidle loads the
 * target with its edits (the journal) into the spare (un_shd, the undo's copy: changes to drp meanwhile are found
 * against the store's bytes and the journal, un_note) as soon as it is chosen and offers it (sq_qm[2]); the audio
 * interrupt takes it at the first tick of the bar's last step and swaps it in on the line (dr_clk). One offered later
 * plays at the next line. The pattern left keeps its edits in the journal (1.00 parked them in the spare and wrote
 * them after the stop); nothing is written */
static UI_LATE void dr_spfree(void)              /* the spare given back to the undo: a copy of drp again */
{
    dr_sp = 0;
    pk_dfree();
    un_load(0u);
}
static UI_LATE int dr_qwant(int32_t p0, int32_t p1)   /* sq_qwant, the drums running */
{
    if (sq_qs[2] == SQ_QS_HALT) return 1;    /* (NONE at its line: sq_qidle first) */
    if (!sq_qt[2]) sq_qo[2] = (uint8_t)p0;   /* (the one playing) */
    sq_qt[2] = (uint8_t)(p1 < 0 ? SQ_QNONE : (uint32_t)p1 + 1u);
    if (sq_qs[2] == SQ_QS_LOAD || sq_qs[2] == SQ_QS_DONE) return 1;   /* (the spare taken for this line: the target
                                          * after it, sq_qidle) */
    sq_qm[2] = 0;                        /* (the spare withdrawn while the request changes; taken just before: as
                                          * above) */
    if (sq_qs[2] == SQ_QS_LOAD) return 1;
    if (p1 == (int8_t)sq_qo[2]) { sq_qr[2] = SQ_QR_OFF; sq_qt[2] = 0; ui.d_lcd = 1; return 1; }   /* (taken back) */
    sq_qr[2] = (uint8_t)(p1 < 0 ? SQ_QR_NONE : SQ_QR_SWAP);
    return 1;
}
static UI_LATE void dr_qidle(uint32_t st, uint32_t run)   /* sq_qidle, the drums' SQ_QR_SWAP */
{
    const uint32_t t = sq_qt[2];
    if (st == SQ_QS_DONE) {              /* (swapped on the line: the spare holds the pattern left) */
        const uint32_t n = dr_spn;
        pk_dleft();                      /* (1.18C33: the one left, marked: kept) */
        set.dr_pat = (uint8_t)n;
        sq_qm[2] = 0;
        sq_qs[2] = SQ_QS_OFF;            /* (the audio interrupt leaves DONE alone while the request stands) */
        if (ui.view == V_DRUM) seq_blk_fix();
        glo_touch();
        if (t && t != n + 1u) {          /* (turned on meanwhile: that one at the next line) */
            sq_qo[2] = (uint8_t)n;
            sq_qr[2] = (uint8_t)(t == SQ_QNONE ? SQ_QR_NONE : SQ_QR_SWAP);
            ui.d_lcd = 1;
        } else sq_qoff(2u);
        dr_spfree();
        return;
    }
    if (!run) {                          /* (stopped: at once, in place) */
        sq_qm[2] = 0;
        if (dr_sp) dr_spfree();
        if (!pk_gate(0u, t - 1u)) { sq_qoff(2u); return; }   /* (1.18C33: its edits fit nowhere: asked) */
        (void)sq_qload(2u, (int32_t)t - 1);
        sq_qoff(2u);
        return;
    }
    if (sq_qm[2] || st == SQ_QS_LOAD) return;
    if (!dr_sp || dr_spn != t - 1u) {    /* (the target into the spare) */
        if (!pk_dgate(t - 1u)) { if (dr_sp) dr_spfree(); sq_qoff(2u); return; }   /* (1.18C33: the one left fits
                                          * nowhere: not changed, asked) */
        dr_sp = 1;
        dr_spn = (uint8_t)(t - 1u);
        dr_load_to(&un_shd, t - 1u);
    }
    sq_qm[2] = 1;                        /* (offered: the line takes it) */
}
static UI_LATE int sq_qwant(uint32_t q, int32_t p0, int32_t p1)   /* PATTERN from p0 (-1 NONE) to p1 (the knob's,
                                          * from the target waiting): 1 = waiting (nothing else to do now); 0 = at once
                                          * (stopped; a sequencer joining: silent to its line, SQ_QR_JOIN) */
{
    if (!(SQ_RUNS || dr.run)) return 0;
    if (!sq_qr[q] && sq_qs[q]) return 1; /* (the last one's end not seen yet by the audio interrupt: a block) */
    if (!sq_qrun(q) || sq_qr[q] == SQ_QR_JOIN) {   /* (joining, or not yet heard: at once, silent until the line) */
        sq_qm[q] = 0;
        sq_qt[q] = (uint8_t)(p1 < 0 ? 0u : (uint32_t)p1 + 1u);
        sq_qr[q] = (uint8_t)(p1 < 0 ? SQ_QR_OFF : SQ_QR_JOIN);
        return 0;
    }
    if (q == 2u) return dr_qwant(p0, p1);
    if (sq_qs[q] >= SQ_QS_WAIT) return 1;   /* (at the line this moment: sq_qidle first) */
    if (!sq_qt[q]) sq_qo[q] = (uint8_t)p0;   /* (the pattern left: its edits kept in the journal) */
    p0 = (int8_t)sq_qo[q];               /* (the one playing: loaded already in its last step, p0 is the target's) */
    if (p1 == p0) {                      /* (the one playing again: taken back; loaded already: back again) */
        if (sq_qm[q]) (void)sq_qload(q, p0);
        sq_qoff(q);
        return 1;
    }
    sq_qt[q] = (uint8_t)(p1 < 0 ? SQ_QNONE : (uint32_t)p1 + 1u);
    if (sq_qm[q] && !sq_qload(q, p1 < 0 ? p0 : p1)) { (void)sq_qload(q, p0); sq_qoff(q); return 1; }
    sq_qr[q] = (uint8_t)(p1 < 0 ? SQ_QR_NONE : SQ_QR_SWAP);
    return 1;
}
static UI_LATE void sq_qidle(void)               /* app_frame: the target loaded in the bar's last step (or at the
                                          * line, late); the line passed; the transport stopped: at once */
{
    uint32_t q;
#ifdef APP_HOST_CMD
    if (sq_qbusy) return;                /* (the host: the main loop kept busy, the late load's way) */
#endif
    for (q = 0; q < 3u; q++) {
        const uint32_t r = sq_qr[q], st = sq_qs[q], run = sq_qrun(q);
#ifndef HYB_MULTI
        if (q == 1u) continue;
#endif
        if (!r) continue;
        if (r == SQ_QR_JOIN) { if (st == SQ_QS_DONE || !run) sq_qoff(q); continue; }
        if (r == SQ_QR_NONE && (st == SQ_QS_HALT || !run)) {   /* (NONE from the line; stopped: at once) */
            if (q == 2u) seq_none(1, 1);
            else {
#ifdef HYB_MULTI
                const uint8_t mi = sq_mi;
                sq_mi = (uint8_t)q;
                seq_none(0, 1);
                sq_mi = mi;
#else
                seq_none(0, 1);
#endif
            }
            sq_qoff(q);
            continue;
        }
        if (r != SQ_QR_SWAP) continue;
        if (q == 2u) { dr_qidle(st, run); continue; }
        if (!sq_qm[q] && (st == SQ_QS_LOAD || st == SQ_QS_WAIT || !run)) {   /* (its last step: loaded now) */
            if (sq_qt[q] && sq_qt[q] != SQ_QNONE && !pk_gate(1u + q, sq_qt[q] - 1u)) { sq_qoff(q); continue; }   /* (1.18C33:
                                          * the pattern left fits nowhere: not changed, asked) */
            if (!sq_qload(q, (int32_t)sq_qt[q] - 1)) { app_say("Pattern", "IN USE"); sq_qoff(q); continue; }
            sq_qm[q] = 1;
        }
        if (st == SQ_QS_DONE || !run) sq_qoff(q);   /* (playing from the line; stopped: done) */
    }
    if (dr_sp && !sq_qr[2]) dr_spfree();   /* (the spare not needed: back to the undo) */
}
static UI_LATE void sq_knob(uint32_t c, int32_t e)   /* SELECT (c 4), K1..K4 (c 0..3): the step held, else the sequencer */
{
    sq_step_t *t = &sqp.st[sqv.sel];
    const uint32_t w0 = (uint32_t)(uint8_t)t->off | (uint32_t)t->la << 8 | (uint32_t)t->vel << 16;   /* (the preview:
                                          * OFFSET, LENGTH, VEL before) */
    const uint32_t ev0 = t->ev;          /* (EVENT before: the audition's, 1.18C44) */
    const uint16_t now = (uint16_t)(plat_ms() | 1u);
    if (sqv.lin && !SQ_CE) sqv.lin = sqv.key = 0;   /* (1.18C31: a step's view kept after its key was let go: gone,
                                          * the knob the sequencer's as before) */
    un.next = UN_KNOB(c);                /* (a small edit) */
    if (pk.ask) { pk.ask = 0; ui.d_lcd = 1; return; }   /* (1.18C33: the question up: a turn takes it back) */
    if (SQ_CE && (c || (sqv.chn && (sqv.ce & SQ_CET)))) return;   /* (CHORD ENTRY: EVENT alone, with no note chosen,
                                          * 1.03; 1.18C31: or none toggled yet, the step's own chord shown) */
    if (SQ_CE && sqv.cek && (uint16_t)(now - sqv.cek) < SQ_CE_REST) { sqv.cek = now; return; }   /* (1.18C31: the
                                          * turn that opened it still going: swallowed) */
#ifdef HYB_MULTI
    if ((sqv.key || c == 0u || c == 1u || c == 3u) && sq_busy()) return;   /* (the other's pattern: PATTERN, BASE NOTE
                                          * alone) */
#endif
    if (!sqv.key) {
        if (c == 0u) {                   /* LENGTH: 16, 32, 64 (the steps past it kept) */
            const uint32_t l = (uint32_t)clamp((int32_t)sqp.lc + e, 0, SQ_NLC - 1);
            if (l != sqp.lc) { sqp.lc = (uint8_t)l; seq_blk_fix(); sq_dirty(SQ_NMAX); }
        } else if (c == 1u) { sqp.div = (uint8_t)clamp((int32_t)sqp.div + e, 0, SQ_NDIV - 1); sq_dirty(SQ_NMAX); }
        else if (c == 2u) {
#ifdef HYB_MULTI
            const int32_t p0 = SQ_PADV & SQ_DISARM ? -1 : (int32_t)SQ_PATV, p1 = sq_pat_step(SQ_QPAT(UN_SQI, p0), e);   /* (the
                                          * other's pattern skipped; from the one waiting, 0.96) */
#else
            const int32_t p0 = set.sq_pad & SQ_DISARM ? -1 : (int32_t)set.sq_pat;
            const int32_t p1 = clamp(SQ_QPAT(0u, p0) + e, -1, SQ_PATS - 1);
#endif
            const uint32_t n = p1 < 0 ? SQ_PATV : (uint32_t)p1;
            if (sq_qwant(UN_SQI, p0, p1)) { sqv.clr = 0; sq_title(7u + c); return; }   /* (0.96: running, at the bar's line) */
            if (p1 >= 0 && n != SQ_PATV && !pk_gate(1u + UN_SQI, n)) return;   /* (1.18C33: its pattern's edits fit
                                          * nowhere: asked) */
            sq_pv_cut();                 /* (a step tapped on still heard: off) */
            pvr.k = 0;
            if (p1 != p0) seq_none(0, p1 < 0);   /* (NONE below 1: no pattern, the patterns kept; the edits of the
                                          * one left in the journal) */
            if (n != SQ_PATV) {
                SQ_PATV = (uint8_t)n;
                sq_load(n);
                seq_blk_fix();
                glo_touch();             /* (its number in the settings) */
            }
#ifdef HYB_MULTI
            else if (p0 < 0 && p1 >= 0) {   /* (NONE to the pattern kept: again, the other may have changed it) */
                sq_load(n);
                seq_blk_fix();
            }
#endif
        } else if (c == 3u) {
            sqv.clr = 1;
            sqv.clr_t = plat_ms();
            sqv.tt = 0;
            app_say("CLEAR?", "REC");
            return;
        } else {                         /* SELECT: the BASE NOTE (OFF below C1; a setting, not the pattern's) */
            const int32_t b = SQ_BASEV ? (int32_t)SQ_BASEV : (int32_t)SQ_BASE_LO - 1;
            const int32_t n = clamp(b + e, (int32_t)SQ_BASE_LO - 1, (int32_t)SQ_BASE_HI);
            if (n != b) { SQ_BASEV = (uint8_t)(n < (int32_t)SQ_BASE_LO ? 0 : n); glo_touch(); }
            sq.lclr = 1;                 /* (KEY LATCH: the notes latched give way to it, heard at once) */
        }
        if (c < 4u) sqv.clr = 0;
        sq_title(7u + c);
        return;
    }
    if (c == 4u && !SQ_OFS(t->ev & 15u)) return;   /* (1.18C31: SELECT, OFFSET, NOTE's and FING n's (its finger's
                                          * note transposed; up to 1.18C30 turned, shown, unused): on FING POLY and
                                          * CHORD nothing changed or shown) */
    if (c == 0u && !SQ_CE && (t->ev & 15u) == SQ_CHORD) {   /* (1.18C31: EVENT on a CHORD step: CHORD ENTRY, its
                                          * chord chosen; the event changed from there, EVENT turned with no note) */
        sq_ce_begin();
        sqv.cek = now;
        sqv.tt = 0;                      /* (the strip: "CHORD STEP n" at once) */
        return;
    }
    if (c == 4u) t->off = (int8_t)clamp(sq_dz(t->off, -e, 5u), -24, 24);   /* (NOTE, FING n: above; after 1.18C34, the user's
                                          * wish: counter-clockwise (e < 0) raises it, clockwise lowers it) */
    else if (c == 0u) {                  /* (EVENT in the list's order, 1.02: CHORD, NOTE, FING 1-8, FING POLY; with
                                          * PATCH ARP ON FING POLY too, the chord fed to the arpeggiator; 0.72-1.01 it
                                          * was refused there) */
        const uint32_t v = SQ_EVU((uint32_t)clamp((int32_t)SQ_EVU(t->ev & 15u) + e, 0, SQ_NEV - 1));
        if ((v == SQ_CHORD) != ((t->ev & 15u) == SQ_CHORD)) t->off = (int8_t)(v == SQ_CHORD ? -1 : 0);   /* (to CHORD:
                                          * none recorded yet, the base note; back: OFFSET 0) */
        t->ev = (uint8_t)((t->ev & (SQ_ON | SQ_ALNEG)) | v);
        if (v != SQ_CHORD && sqv.ce) {   /* (CHORD ENTRY left by another event, 1.03; 1.18C31: the step's view
                                          * as after any change: held, or let go and kept SQ_TT_MS) */
            const uint8_t k = sqv.key;
            sq_ce_end();
            sqv.key = k;
            sqv.lin_t = now;
        }
    }
    else if (c == 1u) {                  /* (LENGTH: up to the pattern's LENGTH; one set longer before it was shortened
                                          * turned down from where it is) */
        const int32_t l0 = t->la & 31, lm = (int32_t)SQ_LMAXP(sqp.lc);
        t->la = (uint8_t)((t->la & 0xE0u) | (uint32_t)clamp(l0 + e, 0, l0 > lm ? l0 : lm));
    }
    else if (c == 2u) {                  /* (ALIGN -7/8 .. +7/8: the eighths and the sign) */
        const int32_t a = clamp(sq_dz(SQ_AL(t), e, 3u), -7, 7);
        t->la = (uint8_t)((t->la & 31u) | (uint32_t)(a < 0 ? -a : a) << 5);
        t->ev = (uint8_t)((t->ev & ~SQ_ALNEG) | (a < 0 ? SQ_ALNEG : 0u));
    }
    else t->vel = (uint8_t)clamp((int32_t)t->vel + knob_steps(c, e), 0, 127);
    sqv.edited = 1;
    sq_title(c == 4u ? 5u : 1u + c);
    sq_dirty(sqv.sel);
    if (((c != 0u && c != 2u             /* (OFFSET, LENGTH, VEL changed: the step heard, 0.85) */
          && (w0 ^ ((uint32_t)(uint8_t)t->off | (uint32_t)t->la << 8 | (uint32_t)t->vel << 16))
             & (c == 4u ? 0xFFu : c == 1u ? 0x1F00u : 0xFF0000u))
         || (c == 0u && sqv.aud == SQ_AUD_ON && !sqv.lin && ((ev0 ^ t->ev) & 15u) && (t->ev & 15u) != SQ_CHORD))   /* (1.18C44:
                                          * in the audition EVENT changed too, the key held; to CHORD: CHORD ENTRY
                                          * ends it) */
        && pv_ask(1u, sqv.sel, UN_SQI, c == 1u))   /* (at most every PVR_GAP: else the latest later, 0.86) */
        sq_pv_play(t, c == 1u ? SQ_PV_LMS : SQ_PV_MS, 8u | (sqv.aud == SQ_AUD_ON && c != 1u ? SQ_PV_AUD : 0u));   /* (1.18C44:
                                          * in the audition sustained again; LENGTH's timed, to hear it) */
    if (c == 0u && (t->ev & 15u) == SQ_CHORD && !SQ_CE) {   /* (1.18C31: EVENT turned to CHORD: CHORD ENTRY at once,
                                          * the key held or not; up to 1.18C30 on the key's release) */
        sq_ce_begin();
        sqv.cek = now;
        sqv.tt = 0;
    }
}
static UI_LATE int sq_rec(void)                  /* REC on the page: CLEAR asked, the pattern cleared (1 = taken) */
{
    const uint8_t lc = sqp.lc, pa = sqp.pad;
    char t[12];
    if (ui.view == V_SEQ && SQ_CE) {     /* (CHORD ENTRY: the chord into the step, 1.03) */
        if (sq_ch_put()) sq_ce_end();
        return 1;
    }
    if (ui.view != V_SEQ || !sqv.clr) return 0;
    sqv.clr = 0;
    sq_clr_to(&sqp, lc, set.sq_def);    /* (all steps off, the rest as shipped, its LENGTH kept; 1.14: PATCH ARP, KEY
                                          * LATCH, REC QUANT GLO > SEQ's; up to 1.13 kept) */
    if ((pa ^ sqp.pad) & SQ_NLAT) sq.lclr = 1;   /* (KEY LATCH changed: as EDIT's, the latch cleared) */
    sq_cut[UN_SQI] = 1u;                 /* (its notes sounding off) */
    un_clear(1u + UN_SQI);               /* (into the journal, one entry: undone by UNDO, written by SAVE, 1.18C33) */
    app_say(pat_text(t, "Pattern ", SQ_PATV + 1), un.nou ? "CLEARED NO UNDO" : "CLEARED");
    return 1;
}
static UI_LATE int sq_tt_on(void) { return sqv.tt && (int32_t)(plat_ms() - sqv.tt_t) < (int32_t)SQ_TT_MS; }   /* (signed:
                                          * tt_t is now | 1, 1.18C46) */
static UI_LATE int sq_tt_text(char *a, char *v)  /* the parameter turned (sq_title), its name and value: 1 = there */
{
    uint32_t mx, bi;
    if (!sq_tt_on()) return 0;
    if (sqv.tt == SQ_TT_TEMPO) {
        text_cat(a, "TEMPO");
        put_sint(v, (int32_t)((tp_run ? tp_run - 1u : shadow[TP_BYTE]) + 63u), 0);
    } else if (ui.view == V_DRUM) {
        text_cat(a, DR_LB[sqv.tt]);
        dr_cv(sqv.tt, v, &mx, &bi);
    } else {
        text_cat(a, SQ_LB[sqv.tt]);
        sq_cv(sqv.tt, v, &mx, &bi);
    }
    return 1;
}
static UI_LATE int se_on(void) { return sqv.menu && ui.view == sqv.menu_v; }   /* the EDIT list up (se_*) */
static UI_LATE int se_lcd(char *a, char *v);
static UI_LATE uint32_t pc_beat(uint32_t div)    /* (1.18C40) PRECOUNT's count running: 1 + the beat counted now (bar
                                          * 4 + beat), at time division div (the page's); 0 none */
{
    const uint32_t m = ck.mt, sh = 5u + CK_SH(div), n = SQ_PCB * 4u;
    uint32_t r;
    if (!ck.on || (int32_t)m >= 0) return 0;
    r = (0u - m + (1u << sh) - 1u) >> sh;   /* (the beats left, this one's included) */
    return r && r <= n ? n - r + 1u : 0u;
}
static UI_LATE void seq_state(char *v, int32_t p, uint32_t run, uint32_t div)   /* a sequencer's strip value (0.76):
                                          * its pattern, the transport and the time division ("P3 PLAY 1x", "P3 STOP
                                          * 1x"); PATTERN NONE: "NONE". (Before, "ON" / "OFF" for the transport read
                                          * as the pattern on or off). 1.18C40: PRECOUNT's count, its bar and beat
                                          * ("P3 COUNT 2.3") */
{
    const uint32_t c = run ? pc_beat(div) : 0u;
    if (p < 0) { text_cat(v, "NONE"); return; }
    text_cat(v, "P");
    put_sint(v + 1, p + 1, 0);
    if (c) {
        text_cat(v, " COUNT ");
        put_sint(v + text_len(v), (int32_t)(c + 3u) / 4, 0);
        text_cat(v, ".");
        put_sint(v + text_len(v), (int32_t)((c - 1u) & 3u) + 1, 0);
        return;
    }
    text_cat(v, run ? " PLAY " : " STOP ");
    text_cat(v, SQ_DIVT[div]);
}
static UI_LATE int sq_step_title(char *a)        /* (0.94) a step key held: the strip's title its step, "STEP 12"
                                          * (1 = so) */
{
    if (!sqv.key) return 0;
    pat_text(a, sqv.ce ? "CHORD STEP " : "STEP ", (int32_t)sqv.sel + 1);   /* (CHORD ENTRY, 1.03) */
    return 1;
}
static UI_LATE void sq_lcd(char *a, char *v)     /* the strip: the parameter turned; else ARP SEQ (a step key held:
                                          * STEP n, 0.94), the pattern, PLAY / STOP and the time division ("ARP SEQ
                                          * P3 PLAY 1x") */
{
    const uint32_t d = ui.view == V_DRUM;   /* (1.03: the drums' page too, dr_lcd up to 1.02: "DRUM SEQ  P2 STOP 1x BD";
                                          * RECORD armed "DRUM REC") */
    if (se_lcd(a, v) || sq_tt_text(a, v)) return;
    seq_state(v, d ? DR_PATSH : SQ_PATSH, d ? dr.run : sq.run, d ? drp.div : sqp.div);
    if (d) { text_cat(v, " "); text_cat(v, DR_NAME[sqv.ins]); }
    if (sq_step_title(a)) return;
    if (d) { text_cat(a, sqv.rec ? "DRUM REC" : "DRUM SEQ"); return; }
#ifdef HYB_MULTI
    if (sq_mi && SQ_BMO) { text_cat(a, (set.mo & MO_B) < 2u ? "B OUT OFF" : sqv.arec ? "B MIDI REC" : "B MIDI ONLY"); return; }   /* (1.19.3: SEQ B in
                                          * SINGLE, MIDI out alone) */
#endif
    text_cat(a, sqv.arec ? "ARP" : "ARP SEQ");
#ifdef HYB_MULTI
    if (set.mx_mode != SY_M_SINGLE) text_cat(a, sq_mi ? " B" : " A");   /* (two parts: the one shown; SINGLE: SEQ A
                                          * "ARP SEQ" as before, SEQ B above) */
#endif
    if (sqv.arec) text_cat(a, " REC");   /* (RECORD on: "ARP REC", "ARP B REC") */
}
static UI_LATE uint32_t sq_tt_sig(void)          /* what the strip's title shows (0: none) */
{
    if (!sq_tt_on()) return 0;
    if (sqv.tt == SQ_TT_TEMPO) return 0x80000000u | tp_run;
    return 0x80000000u | (uint32_t)sqv.tt << 24 | ((ui.view == V_DRUM ? dr_sig(SQ_PG + DR_NI + sqv.tt)
                                                                       : sq_sig(SQ_PG + sqv.tt)) & 0xFFFFFFu);
}
#define SQ_KEYT (sqv.key ? sqv.sel + 1u : 0u)   /* (the strip's title: the step held, 0.94) */
static UI_LATE void seq_frame(void)              /* a sequencer's page (the drums' on V_DRUM), each frame: the strip
                                          * when it changes (1.03: one for both, sq_frame and dr_frame up to 1.02) */
{
    const uint32_t t = sq_tt_sig(), d = ui.view == V_DRUM;
#ifdef HYB_MULTI
    const uint32_t m = (uint32_t)sq_mi << 4;
#else
    const uint32_t m = 0;
#endif
    const uint32_t l = t ? t : (d ? dr.run | (uint32_t)drp.div << 1 | (uint32_t)sqv.ins << 4 | (uint32_t)sqv.rec << 8
                                    | (uint32_t)(DR_PATSH + 1) << 9 | 1u << 15
                                  : sq.run | (uint32_t)sqp.div << 1 | m | (uint32_t)sqv.arec << 5
                                    | (uint32_t)(SQ_PATSH + 1) << 6) | SQ_KEYT << 16
                       | pc_beat(d ? drp.div : sqp.div) << 24;   /* (1.18C40: PRECOUNT's beat) */
    if (l != sqv.lcd) { sqv.lcd = l; ui.d_lcd = 1; }
}
static UI_LATE uint32_t dr_lights(uint32_t *dim, uint32_t *mid);
#define ARP_FLASH 0.15f                  /* the ARP light: lit for this part of each beat (0.67) */
static UI_LATE uint32_t arp_beat(uint32_t now)   /* the ARP light lit now (as 1 << B_ARP): while the part edited's
                                          * patch has its arpeggiator on (its byte), the first ARP_FLASH of each beat at
                                          * the running tempo (MIDI clock in: the clock's; else the patch's); between
                                          * the beats the middle level (input: mb), the arpeggiator off the glow */
{
    static uint32_t t0 PLAT_POOL;
    static float ph PLAT_POOL;           /* (the beat's phase, 0..1) */
    const uint32_t dt = now - t0;
    const float bpm = S.xclk_on && S.bpm_now > 1.f ? S.bpm_now : (float)shadow[128 + 16] + 63.f;   /* (two parts,
                                          * 1.18: both parts' panel bytes the global tempo) */
    t0 = now;
    if (!shadow[129]) { ph = 0.f; return 0; }   /* (on again: lit at once, the beat from there) */
    if (dt < 1000u) ph += (float)dt * bpm * (1.f / 60000.f);
    if (ph >= 1.f) ph -= (float)(int32_t)ph;
    return ph < ARP_FLASH ? 1u << B_ARP : 0u;
}
static UI_LATE uint32_t kit_lights(uint32_t *mid);
static UI_LATE uint32_t sq_ce_lights(void)       /* (1.03) CHORD ENTRY's key lights: the keys of the notes chosen
                                          * (sq_lights: its own function, input's code unchanged) */
{
    const int32_t b = 53 + 12 * set.oct + set.tr - rl_base() + 64;   /* (key 0's note, as sq_rec_key plays it, as
                                          * the chords hold it) */
    uint32_t j, m = 0;
    for (j = 0; j < sqv.chn; j++) {
        const uint32_t k = (uint32_t)(sqv.cht[j] - b);
        if (k < NKEYS) m |= 1u << k;
    }
    return m;
}
static UI_LATE uint32_t sq_blk_lights(uint32_t p)   /* OCT- / OCT+ on a sequencer's page (p 1: the drums'): lit where
                                          * a block of 16 steps is left to go to (a pattern of 16 steps: both dark) */
{
    const uint32_t n = 1u << (p ? drp.lc : sqp.lc), b = sqv.blk[p];
    if (!p && sqv.arec) return oct_lights();   /* (RECORD: the octave, 1.19.7; REC lit as RECORD's) */
    if (!p && SQ_CE) return oct_lights() | 1u << B_REC | 1u << B_HOME;   /* (CHORD ENTRY: the octave, 1.03; 1.05: REC
                                          * and HOME lit, here: input()'s lights take this function's buttons, its code
                                          * stays as it is) */
    return (b > 0u ? 1u << B_OCTDN : 0u) | (b + 1u < n ? 1u << B_OCTUP : 0u);
}
#define SQ_BEATS (1u << SQ_KEY[0] | 1u << SQ_KEY[4] | 1u << SQ_KEY[8] | 1u << SQ_KEY[12])   /* (steps 1, 5, 9, 13 of a
                                          * block) */
static UI_LATE uint32_t sq_beat(void)   /* (1.18C43) the first 1/16 of a beat while a sequencer runs (the arp
                                          * sequencer shown, else the other, else the drums; the clock's, MIDI's
                                          * too): PLAY mode's PLAY light, as sq_lights' SEQ light */
{
#ifdef HYB_MULTI
    const sq_rt_t *q = sq.run ? &sq : &sqr[!sq_mi];
    return q->run ? !(q->step & 3u) : dr.run && !(dr.step & 3u);
#else
    return sq.run ? !(sq.step & 3u) : dr.run && !(dr.step & 3u);
#endif
}
static UI_LATE uint32_t sq_lights(uint32_t lk, uint32_t *lb, uint32_t *dim, uint32_t *mid)   /* the SEQ light (the
                                          * first 1/16 of each beat, any view) and the keys: on the page the block
                                          * shown, its steps on lit, the beats' steps off at the middle level, the
                                          * step playing (in that block) inverted (PATTERN NONE: all keys at the
                                          * middle level); elsewhere while it runs the step playing dark among
                                          * the keys lit (KEY LIGHTS off: dark, no dim keys) */
{
    uint32_t i, m = 0, cur;
    const uint32_t b0 = sqv.blk[0] * SQ_N;
    *dim = *mid = 0;
    {   /* (a sequencer's page: lit, dark on the beat's first 1/16 while it runs; elsewhere lit on it) */
#ifdef HYB_MULTI
        const sq_rt_t *q = sq.run ? &sq : &sqr[!sq_mi];   /* (the one shown's, else the other's) */
        const uint32_t beat = q->run ? !(q->step & 3u) : dr.run && !(dr.step & 3u);
#else
        const uint32_t beat = sq.run ? !(sq.step & 3u) : dr.run && !(dr.step & 3u);   /* (the arp sequencer's beat,
                                          * else the drums') */
#endif
        if (beat != (ui.view == V_SEQ || ui.view == V_DRUM)) *lb |= 1u << B_SEQ;
    }
#ifdef HYB_MULTI
    if (ui.view == V_LNCH) return ln_lights(lb, mid);   /* (the launcher: the part keys, HOME, EDIT, REC) */
#endif
    if (ui.view == V_DRUM) {
        m = dr_lights(dim, mid);
        if (KEYS_DARK) *dim = 0;         /* (the instruments without steps; the beats are at the middle level) */
        return m;
    }
    if (ui.view != V_SEQ) {              /* (elsewhere: the running light, the step playing of the arp sequencer
                                          * shown, else of the other, else of the drums; KEY LIGHTS off: that key
                                          * alone lit, else dark among the keys lit) */
#ifdef HYB_MULTI
        const sq_rt_t *q = sq.run ? &sq : &sqr[!sq_mi];
#else
        const sq_rt_t *q = &sq;
#endif
        const uint32_t r = q->run ? 1u << SQ_KEY[q->step & 15u] : dr.run ? 1u << SQ_KEY[dr.step & 15u] : 0u;
        return KEYS_DARK ? r : lk & ~r;
    }
    if (SQ_CE) return sq_ce_lights();    /* (CHORD ENTRY, 1.03: the keys of the notes chosen, KEY LIGHTS off too) */
    cur = sq.run && (uint32_t)(sq.step >> 4) == sqv.blk[0] ? 1u << SQ_KEY[sq.step & 15u] : 0u;
    if (sqv.arec) return ((1u << NKEYS) - 1u) ^ cur;   /* (RECORD: every key a note, lit; the step playing dark) */
    if (SQ_PATSH < 0) { *mid = (1u << NKEYS) - 1u; return 0; }   /* (PATTERN NONE: the keys play, all at the
                                          * middle level) */
    for (i = 0; i < SQ_N; i++)
        if (sqp.st[b0 + i].ev & SQ_ON) m |= 1u << SQ_KEY[i];
    m ^= cur;
    *mid = SQ_BEATS & ~m & ~cur;         /* (the beats off: the middle level, also with KEY LIGHTS off; 0.83: was the
                                          * glow, too faint beside the buttons' middle level since 0.63) */
    return m;
}

/* ----------------------------------------------- the drum sequencer's panel --- */
/* The drums' page (V_DRUM: SEQ on the arp sequencer's page; SEQ again the arp sequencer's, 0.79). The black
 * keys choose the instrument (and play it: the accent from the key's velocity); the 16 white keys are its steps of the
 * block shown (OCT- / OCT+): a tap turns one on or off, one held shows its values on K1..K4 (ACCENT, LENGTH, ALIGN,
 * STEP on / off). No step key held, K1..K4 act on the drum sequencer (LENGTH 16 / 32 / 64, TIME DIV, PATTERN NONE,
 * 1 .. 16, CLEAR, REC confirms) and SELECT on the level. Its edits wait in the journal (un_note) until SAVE; the
 * settings keep its number and the level (glo_touch). SHIFT + SEQ stops both sequencers. The runtime: sq_block, dr_render (the audio interrupt) */
#define DR_SAVE_MS 3000u
#define dr_dirty(st) un_note(0u)        /* (step st changed: the drum pattern's change found, un_note) */
static UI_LATE void dr_idle(uint32_t now)        /* app_frame: RECORD off once the page is left or the transport
                                          * stops (the pattern's edits: the journal, sq_idle) */
{
    if (sqv.rec && (ui.view != V_DRUM || (sqv.rec_run && !dr.run) || DR_PATSH < 0)) sqv.rec = 0;
    if ((sq_pvm & 4u) && !(ui.view == V_DRUM && sqv.key && DR_PATSH >= 0)) sq_pvm &= 3u;   /* (the step key let go,
                                          * the page left: the drums' steps heard again, 0.94) */
    dr_metro = sqv.rec;
    if (sqv.rec_run != dr.run) un.pass++;   /* (a recording pass: from a start to a stop) */
    sqv.rec_run = dr.run;
    if (!sqv.arec && !sqv.rec && (SQ_RUNS || dr.run) && ck.on && (int32_t)ck.mt < 0) pc_cut = 1;   /* (1.18C40:
                                          * RECORD off in PRECOUNT's count, the page left: the sequencers from step 1
                                          * now, sq_block) */
    if (sqv.menu && ui.view != sqv.menu_v) sqv.menu = 0;   /* (the page left: its EDIT list closed) */
}
#define DR_REC_OUT 1u                    /* RECORD's compensation: the blocks from a render to the speaker (the DMA's
                                          * other half; the FIFO's blocks, lat_fifo, on top) */
#define DR_REC_TOUCH 0.005f              /* and a key's press to the main loop, s (sense.c: two samples, ~4.4 ms) */
#ifdef APP_HOST_CMD
static uint32_t rec_m, rec_back;         /* (the host's print) */
static float rec_hf;                     /* (1.14, the host: the clock's place heard, unrounded: recqt) */
#endif
#define REC_PRE (~0u)                    /* rec_abs: heard in PRECOUNT's count, before step 1 (1.18C40): not recorded */
static UI_LATE uint32_t rec_abs(uint32_t div, uint32_t e8)   /* RECORD (the drums', the arp sequencer's): the step at
                                          * time division div nearest to where the beat was heard when a key went down
                                          * now, counted from the clock's start (the clock's place less the drums' /
                                          * the engine's delay, DR_LAT, the speaker's and the key's; a step's second
                                          * half: the next); e8 (1.14, REC QUANT OFF): the nearest eighth of a step
                                          * instead (its ticks, as the sequencer's) */
{
    const uint32_t sh = CK_SH(div) + (e8 ? 0u : 3u), lat = S.opt & SY_OPT_LATENCY ? 2u : 0u;
    const float bpm = S.bpm_now > 1.f ? S.bpm_now : (float)S.patch[128 + 16] + 63.f;
    const float tk = bpm * (16.f * 2.f / 15.f);   /* (the clock's ticks a second, sq_block) */
    uint32_t m, back;
    float f;
    do { m = *(volatile uint32_t *)&ck.mt; f = *(volatile float *)&ck.frac; } while (m != *(volatile uint32_t *)&ck.mt);
    f = tk * ((float)SY_BLOCK * (float)(lat + DR_REC_OUT + (uint32_t)lat_fifo()) / S.fs + DR_REC_TOUCH) - f;
    back = f > 0.f ? (uint32_t)(f + 0.5f) : 0u;   /* (the clock's ticks back to the beat heard: m + frac is the end
                                          * of the block rendered last) */
#ifdef APP_HOST_CMD
    rec_m = m; rec_back = back; rec_hf = (float)m - f;
#endif
    if ((int32_t)m < 0)                  /* (1.18C40: PRECOUNT's: heard before step 1's rounding, none; within it, step 1) */
        return (int32_t)(m - back + (1u << sh >> 1)) < 0 ? REC_PRE : 0u;
    return m > back ? (m - back + (1u << sh >> 1)) >> sh : 0u;   /* (the step (eighth) counted from the clock's start,
                                          * rounded) */
}
static UI_LATE void dr_rec_hit(uint32_t i, uint32_t acc)   /* RECORD (the transport running): instrument i struck now
                                          * (heard at once, dr.prev) onto the step nearest to where the beat was heard
                                          * (the clock's place less the drums' delay, DR_LAT, the speaker's and the
                                          * key's: dr_tick's grid; a hit in a step's second half goes to the next,
                                          * round at LENGTH); a step off is turned on with the accent (the length and
                                          * align as shipped), one on kept; struck ahead of its step, that step does
                                          * not strike it again this pass (dr_mute) */
{
    uint32_t a = rec_abs(drp.div, 0u), s, mu;
    if (a == REC_PRE) return;            /* (1.18C40: PRECOUNT's count: heard only) */
    a -= dr_org >> CK_SH(drp.div) >> 3;  /* (from the pattern's origin, 0.96) */
    s = a & (SQ_STEPS(drp.lc) - 1u);
    mu = (a & 0x1FFFFFu) << 11;
    dr_mute = (dr_mute & ~0x7FFu) == mu ? dr_mute | 1u << i : mu | 1u << i;
    if (!DR_ON(i, s)) {
        drp.on[s >> 4][i] |= (uint16_t)(1u << (s & 15u));
        drp.neg[s >> 4][i] &= (uint16_t)~(1u << (s & 15u));
        DR_ST(i, s) = DR_SB(acc, DR_LEN1, 0u);
        un.next = UN_REC + un.pass;      /* (a pass's hits: one undo) */
        dr_dirty(s);
    }
#ifdef APP_HOST_CMD
    printf("  rec ins %u step %u (abs %u, m %u back %u: %.3f steps)\n", (unsigned)i, (unsigned)s, (unsigned)a,
           (unsigned)rec_m, (unsigned)rec_back, (double)((float)rec_m - (float)rec_back) / (double)(8u << CK_SH(drp.div)));
#endif
}
static UI_LATE uint32_t dr_ikey(uint32_t k)     /* key k's instrument (DR_NI: none) */
{
    uint32_t i;
    for (i = 0; i < DR_NI && DR_KEY[i] != k; i++)
        ;
    return i;
}
static UI_LATE void seq_pick_say(void) { app_say("NONE", "SELECT PATTERN"); }   /* a sequencer's page with PATTERN
                                          * NONE: a key or REC says so (the keys play, RECORD stays off) */
static UI_LATE void key_on(uint32_t k);
static UI_LATE int seq_none_key(uint32_t k)      /* a key struck on the arp sequencer's page (sq_key): with PATTERN
                                          * NONE it plays its note as off the pages, no step (0.83; 1 = played), and
                                          * says SELECT PATTERN once the strip is free (the drums' page: dr_key) */
{
    if (SQ_PATSH >= 0) return 0;
    if (!ui.msg_until) seq_pick_say();
    key_on(k);
    return 1;
}
static UI_LATE void dr_arm(void)                 /* REC on the drums' page: RECORD on / off */
{
    if (!sqv.rec && DR_PATSH < 0) { seq_pick_say(); return; }
    sqv.rec = (uint8_t)!sqv.rec;
    dr_metro = sqv.rec;
    sqv.rec_run = dr.run;
    un.pass++;
    say_on("Record", sqv.rec);
}
static UI_LATE void dr_erase(uint32_t i)         /* SHIFT + an instrument's key while recording: its steps all off */
{
    uint32_t b;
    for (b = 0; b < SQ_NB; b++) drp.on[b][i] = 0;
    un.next = UN_BIG | ++un.seq;         /* (an undo level of its own, 1.18C33) */
    dr_dirty(0);
    app_say(DR_NAME[i], un.nou ? "ERASED NO UNDO" : "ERASED");
}
/* RECORD on the arp sequencer's page (0.63): REC arms it (instead of HOLD there; while CLEAR asks
 * REC still clears); every key is then a note (all lit, the step playing shown in the block shown), heard at once on
 * the sequencer's part (not its input); while the transport runs a note is written onto the step nearest to where it
 * was heard (rec_abs, as the drums' RECORD), wrapping at LENGTH, in whichever block plays: the step on, EVENT NOTE,
 * OFFSET the note less the BASE NOTE (folded by octaves into -24 .. +24), VEL the key's velocity, ALIGN 0, LENGTH from
 * how long the key is held (whole steps, 1/16 at least, to its block's end at most). Keys struck together (within
 * SQ_RCH_MS of the step's first, 1.05: sq_rec_win, sq_rec_ch): a CHORD step of their notes (VEL the loudest, LENGTH the
 * longest held; the pattern's chords full: its lowest note); a key past that onto the same step: its lowest note (a
 * chord kept). A step recorded ahead of its tick is not played again that pass (sq_mute). The rim shot
 * clicks the beats (sq_metro). OCT- / OCT+ the keyboard's octave while armed (1.19.7, as on HOME; the block shown
 * follows the step playing while the transport runs). Disarmed on leaving the page, when the transport stops, when the page shows the other
 * arp sequencer. A pass (arm, start, stop) is one undo. The pattern is saved after the stop (store_wait).
 * MIDI in (1.18C31; sr_mi_in, sr_mi_poll): a note on the sequencer's part (its channel, as the notes in reach it,
 * sq_parts_in) while RECORD is armed is recorded as a key is (the note as it comes: no OCTAVE, no TRANSPOSE; its
 * velocity), in a slot of its own after the keys' (SR_NM, found by its note), heard on the part, not its input */
#define SR_NM 10u                        /* (1.18C31) the MIDI notes in held at once while RECORD is armed */
#define SR_N (NKEYS + SR_NM)             /* sr's slots: the keys' (0 .. NKEYS - 1), then the MIDI notes' */
static struct {
    uint8_t note[SR_N];                  /* a key's note sounding + 1 (0 none) */
    uint8_t st[SR_N];                    /* the step its note went onto (0xFF none) */
    int8_t off[SR_N];                    /* the offset written there (SQ_ROCH: a chord's note, 1.05) */
    uint32_t t[SR_N];                    /* the key went down then */
    uint16_t abs;                        /* the last step written (counted from the clock's start) + 1 */
    uint8_t pass, lo;                    /* its pass (un.pass) and note: a chord's lowest kept */
} sr PLAT_POOL;
static UI_LATE void hold_flip(uint32_t keys);
static UI_LATE void sr_note(uint32_t note, uint32_t v)   /* a recorded key's note on (v > 0) / off (0) on the arp
                                          * sequencer's part, and out */
{
#ifdef HYB_MULTI
    ev_put(sq_mi ? EV_KEY_B : EV_KEY_A, note, v);
    midi_send_p(sq_mi, v ? 0x90u : 0x80u, note, v);   /* (1.19.4: on the sequencer's part's OUT) */
#else
    ev_put(v ? 0x90u : 0x80u, note, v);
    midi_send_ch(v ? 0x90u : 0x80u, note, v);
#endif
}
static UI_LATE void sr_play(uint32_t k, uint32_t note, uint32_t v)   /* (1.18C31) slot k's note on / off: a key's
                                          * as sr_note; a MIDI note in's on the part alone (not sent out again: thru
                                          * sends it on as configured, as any note in) */
{
    if (k < NKEYS) { sr_note(note, v); return; }
#ifdef HYB_MULTI
    ev_put(sq_mi ? EV_KEY_B : EV_KEY_A, note, v);
#else
    ev_put(v ? 0x90u : 0x80u, note, v);
#endif
}
static UI_LATE uint32_t sr_on(void)      /* (1.11) a key's note sounding (RECORD's, CHORD ENTRY's) */
{
    uint32_t k, n = 0;
    for (k = 0; k < SR_N; k++) n |= sr.note[k];
    return n;
}
static UI_LATE void sr_end(void)         /* RECORD off: the notes of the keys still down off (MIDI's too) */
{
    uint32_t k;
    for (k = 0; k < SR_N; k++)
        if (sr.note[k]) { sr_play(k, sr.note[k] - 1u, 0u); sr.note[k] = 0; }
}
static UI_LATE void sq_arm(void)                 /* REC on the arp sequencer's page: RECORD on / off */
{
    if (!sqv.arec && SQ_PATSH < 0) { seq_pick_say(); return; }   /* (NONE: nothing to record into) */
    if (!sqv.arec && !SQ_BASEV) { app_say("Record", "BASE OFF");  return; }   /* (no base: NOTE plays nothing) */
    if (!sqv.arec && hold_on) hold_flip(ui.keys);   /* (REC is RECORD here: HOLD off) */
    sqv.arec = (uint8_t)!sqv.arec;
    sqv.arec_run = sq.run;
    sqv.arec_mi = (uint8_t)UN_SQI;
    un.pass++;
    if (!sqv.arec) sr_end();
    sq_hold_sync();
    sq_metro = (uint8_t)(sqv.arec ? 1u + UN_SQI : 0u);
    say_on("Record", sqv.arec);
}
#define SQ_RCH_MS 50u                    /* (1.05) RECORD: keys struck within this of a step's first key: one chord */
#define SQ_ROCH 127                      /* sr.off: the key's note is in its step's chord (its LENGTH: the longest) */
static UI_LATE uint32_t sq_ch_slot(uint32_t s, const uint8_t *v);
static UI_LATE int sq_rec_ch(uint32_t k, uint32_t note, uint32_t v, uint32_t s)   /* (1.05) RECORD: key k's note
                                          * (velocity v) into step s's chord, the step written last this pass (its
                                          * NOTE becomes a CHORD of both, as CHORD ENTRY keeps it: offsets from the
                                          * BASE NOTE + 64, -48 .. +48, 6 at most). 1: taken (or nothing to do); 0:
                                          * the single note's way (out of range, CHORDS FULL on a NOTE step: its
                                          * lowest note kept, as before 1.05) */
{
    sq_step_t *const p = &sqp.st[s];
    const uint32_t base = SQ_BASEV, c = note + 64u - base, ev = p->ev & 15u, lo = sr.lo + 64u - base;
    uint8_t w[SQ_CHV] = {0};
    uint32_t i, j, n = 0, f;
    if (c - 16u > 96u) return 0;
    if (ev == SQ_CHORD && (uint8_t)p->off < SQ_CHN)
        for (; n < SQ_CHV && sqp.ch[(uint8_t)p->off][n]; n++) w[n] = sqp.ch[(uint8_t)p->off][n];
    else if (ev == SQ_NOTE && lo - 16u <= 96u) w[n++] = (uint8_t)lo;
    else return 0;
    for (i = 0; i < n && w[i] < c; i++)
        ;
    if (i < n && w[i] == c) return 1;    /* (in it already) */
    if (n >= SQ_CHV) { app_say("CHORD", "MAX 6 NOTES"); return 1; }
    for (j = n; j > i; j--) w[j] = w[j - 1u];
    w[i] = (uint8_t)c;
    if ((f = sq_ch_slot(s, w)) == SQ_CHN) return ev == SQ_CHORD;   /* (CHORDS FULL: a chord stays as it is) */
    p->ev = SQ_ON | SQ_CHORD;
    p->off = (int8_t)f;
    if (v > p->vel) p->vel = (uint8_t)v;   /* (VEL: the loudest; LENGTH: the longest held, sq_rec_key) */
    if (note < sr.lo) sr.lo = (uint8_t)note;
    sr.st[k] = (uint8_t)s;
    for (j = 0; j < SR_N; j++)           /* (the keys on it still down: their LENGTH the chord's) */
        if (sr.note[j] && sr.st[j] == s) sr.off[j] = SQ_ROCH;
    un.next = UN_REC + un.pass;
    sq_dirty(s);
#ifdef APP_HOST_CMD
    printf("  arec key %u note %u vel %u step %u chord %u (%u notes)\n", (unsigned)k, (unsigned)note, (unsigned)v,
           (unsigned)s, (unsigned)f, (unsigned)n + 1u);
#endif
    return 1;
}
static UI_LATE int sq_rec_win(uint32_t s, uint32_t now)   /* (1.05) RECORD: a key down now is struck with step s's
                                          * (the step written last): its keys still down went down within SQ_RCH_MS,
                                          * one at least (a key held from further back, a pass before: not counted;
                                          * no state of its own: sr's times. A chord's first key let go before the
                                          * window ends: the keys after it on their own) */
{
    uint32_t j, ok = 0;
    for (j = 0; j < SR_N; j++)
        if (sr.note[j] && sr.st[j] == s && now - sr.t[j] < 4u * SQ_RCH_MS) {
            if (now - sr.t[j] > SQ_RCH_MS) return 0;
            ok = 1;
        }
    return (int)ok;
}
static UI_LATE void sq_rec_hit(uint32_t k, uint32_t note, uint32_t v)   /* RECORD, running: key k's note onto its step
                                          * (REC QUANT OFF, 1.14: the step it falls in, its eighth there the ALIGN) */
{
    const uint32_t nq = sqp.pad & SQ_NRQ, r = rec_abs(sqp.div, nq), e = r - ((uint32_t)sq_org[UN_SQI] >> (nq ? 0u : 3u));
    uint32_t a = nq ? e >> 3 : e, s;
    const uint32_t base = SQ_BASEV, now = plat_ms(), m = SQ_STEPS(sqp.lc) - 1u;
    int32_t off = (int32_t)note - (int32_t)base;
    sq_step_t *p;
    if (!base || r == REC_PRE) return;   /* (1.18C40: PRECOUNT's count: heard only, sr_down) */
    if (sr.abs && sr.pass == un.pass && sq_rec_win((sr.abs - 1u) & m, now)) {   /* (1.05: struck with the step
                                          * written last: its chord, wherever this key's own place) */
        a = sr.abs - 1u;
        if (sq_rec_ch(k, note, v, a & m)) return;
    }
    s = a & m;
    p = &sqp.st[s];
    while (off > 24) off -= 12;
    while (off < -24) off += 12;
    sq_mute[UN_SQI] = (uint16_t)((a & 0x1FFFu) + 1u);
    if (sr.abs == (uint16_t)(a + 1u) && sr.pass == un.pass && (sr.lo <= note || (p->ev & 15u) == SQ_CHORD)) return;   /* (a
                                          * key past the window: the step's lowest; its chord kept) */
    sr.abs = (uint16_t)(a + 1u);
    sr.pass = un.pass;
    sr.lo = (uint8_t)note;
    p->ev = SQ_ON | SQ_NOTE;
    p->off = (int8_t)off;
    p->la = (uint8_t)(2u | (nq ? (e & 7u) << 5 : 0u));   /* (1/16 until the key is let go; ALIGN 0, REC QUANT OFF
                                          * the key's eighth late, 0 .. 7/8: a chord's keys keep the first's, sq_rec_ch) */
    p->vel = (uint8_t)v;
    sr.st[k] = (uint8_t)s;
    sr.off[k] = (int8_t)off;
    un.next = UN_REC + un.pass;          /* (a pass's notes: one undo) */
    sq_dirty(s);
#ifdef APP_HOST_CMD
    printf("  arec key %u note %u vel %u step %u off %d align %u (abs %u, m %u back %u: %.3f steps)\n", (unsigned)k,
           (unsigned)note, (unsigned)v, (unsigned)s, (int)off, (unsigned)(p->la >> 5), (unsigned)a, (unsigned)rec_m,
           (unsigned)rec_back, (double)((float)rec_m - (float)rec_back) / (double)(8u << CK_SH(sqp.div)));
    host_rq_h = rec_hf;                  /* (recqt: where it was heard, the clock's ticks) */
#endif
}
static UI_LATE void sr_down(uint32_t k, uint32_t note, uint32_t v)   /* slot k's note struck (velocity v, 1 ..
                                          * 127): heard, and recorded while RECORD runs (a key's or a MIDI note's) */
{
    if (sr.note[k]) sr_play(k, sr.note[k] - 1u, 0u);
    sr.note[k] = (uint8_t)(note + 1u);
    sr.st[k] = 0xFFu;
    sr.t[k] = plat_ms();
    sr_play(k, note, v);                 /* (heard at once, on its part; not the sequencer's input. PATCH ARP ON:
                                          * into its part's arpeggiator, 0.76; recorded as the key's note still) */
    if (sq.run && ck.on && sqv.arec) sq_rec_hit(k, note, v);   /* (CHORD ENTRY's notes, 1.03: heard alone,
                                          * sq_ch_key, arec off) */
}
static UI_LATE void sr_up(uint32_t k);
static UI_LATE void sq_rec_key(uint32_t k, int down)   /* a key on the arp sequencer's page while RECORD is on */
{
    if (down) {
        uint32_t v = key_vel(k);
        const int32_t note = 53 + (int32_t)k + 12 * set.oct + set.tr;   /* (as the key plays elsewhere) */
        if (note < 0 || note > 127) return;
        if (v < 1u) v = 1u;
        sr_down(k, (uint32_t)note, v);
        return;
    }
    sr_up(k);
}
static UI_LATE void sr_up(uint32_t k)    /* slot k's note let go: off, its step's LENGTH from how long it was held */
{
    if (!sr.note[k]) return;
    sr_play(k, sr.note[k] - 1u, 0u);
    sr.note[k] = 0;
    if (sr.st[k] != 0xFFu) {             /* its LENGTH: how long it was held, whole steps (1/16 at least; past a bar
                                          * in beats, 4 steps; the pattern's LENGTH at most, 0.71) */
        const uint32_t s = sr.st[k], d = sqp.div;
        sq_step_t *p = &sqp.st[s];
        const float bpm = S.bpm_now > 1.f ? S.bpm_now : (float)S.patch[128 + 16] + 63.f;
        const float stp = 60000.f / (bpm * (float)(1u << d));   /* (a step, ms: 1/16 at 1x, d 2) */
        uint32_t n = (uint32_t)((float)(plat_ms() - sr.t[k]) / stp + 0.5f), li;
        if (n < 1u) n = 1u;
        li = n == 1u ? 2u : n <= SQ_N ? n + 1u : 13u + (n + 2u) / 4u;   /* (17 steps: 16; 18 .. 21: 20; 22 .. 25:
                                          * 24; ...) */
        if (li > SQ_LMAXP(sqp.lc)) li = SQ_LMAXP(sqp.lc);
        if (sr.off[k] == SQ_ROCH ? p->ev == (SQ_ON | SQ_CHORD) && (p->la & 31u) < li   /* (a chord's: the longest
                                          * held, 1.05) */
            : p->ev == (SQ_ON | SQ_NOTE) && p->off == sr.off[k] && (p->la & 31u) != li) {
            p->la = (uint8_t)((p->la & ~31u) | li);
            un.next = UN_REC + un.pass;
            sq_dirty(s);
        }
        sr.st[k] = 0xFFu;
    }
}
static UI_LATE void sr_mi_poll(void)     /* (1.18C31) main loop, before the keys: the MIDI notes in that RECORD
                                          * took (sr_mi_in) as keys struck and let go, each in a slot of its own found
                                          * by its note (a note on again: its slot, as a key struck again; all slots
                                          * held: not heard, not recorded). A note on that comes after RECORD went
                                          * off, or for the other sequencer, is dropped (its off then finds no slot).
                                          * The drum channel's hits (dr_mi_in) onto the drums' steps */
{
    while (srq_r != srq_w) {
        const uint32_t e = srq[srq_r % SRQ], note = e & 127u, v = e >> 7 & 127u;
        uint32_t k, f = SR_N;
        srq_r++;
        if (e >> 15) {                   /* (1.18C31: DRUM CHANNEL's hit, dr_mi_in: recorded as a black key's) */
            if (sqv.rec && dr.run && dr.on && ck.on) dr_rec_hit(note, v);
            continue;
        }
        for (k = NKEYS; k < SR_N && sr.note[k] != note + 1u; k++)
            if (!sr.note[k] && f == SR_N) f = k;
        if (!v) {                        /* (off: its slot's LENGTH, as a key let go) */
            if (k < SR_N) sr_up(k);
            continue;
        }
        if (!sqv.arec || (e >> 14 & 1u) != UN_SQI || ui.view != V_SEQ) continue;
        if (k == SR_N) k = f;
        if (k == SR_N) continue;
        sr_down(k, note, v);
    }
}
/* CHORD ENTRY (1.03; 1.02 gathered the notes while the step key was held): EVENT turned on the
 * step held (1.18C31; 1.03 to 1.18C30 a CHORD step's key let go opened it, a change of its LENGTH too): onto CHORD, or
 * on a CHORD step its first detent (its chord then chosen; the event unchanged; the turns SQ_CE_REST after it
 * swallowed), keeps the step's place (sqv.key, sqv.ce: "CHORD STEP n"), its key let go or not. Every key (white and black) then toggles a
 * note in or out of the chord (SQ_CHV at most; one more: "MAX 6 NOTES"), heard as RECORD's keys are (sr: off when let
 * go); the notes chosen lit on the keys (the octave shown) and drawn in the roll at their pitch (rl_notes); OCT- / OCT+
 * move the keyboard's octave (set back at the end). REC: the chord into the step (sq_ch_put); HOME: left, the step as
 * it was; EVENT turned with no note chosen (1.18C31: or none toggled yet, the step's own chord shown; so a CHORD step's
 * EVENT changed: its first detent opens ENTRY, the next ones SQ_CE_REST later leave it): left with that event. A chord the step has: its notes chosen first.
 * 1.05: while it runs the step plays the notes chosen (sq_cel: on, the chord sqv.cht; nothing stored, so HOME needs
 * nothing back), REC and HOME lit (sq_blk_lights). The
 * chord: the notes less the base the step plays from (rl_base: BASE NOTE, KEY LATCH's note, C4 with BASE OFF; as NOTE's
 * OFFSET, so the chord follows the base; past -48 .. +48 left out), lowest first, into the pattern's chords: the same
 * chord shared, else one no other step uses (the step's own too); none left: "CHORDS FULL", ENTRY stays. One change
 * (sq_dirty: the step and the chord's bytes in the journal), one undo with the EVENT turned to CHORD before it.
 * 1.07: every note toggled out (SQ_CEE): the choice empty, not "none chosen yet": none shown or lit, the step silent
 * (sq_cel), REC a rest (sq_ch_put: off, chord none, its entry emptied unless another step uses it; one change too) */
static UI_LATE void sq_ce_begin(void)            /* CHORD ENTRY on for step sqv.sel (sq_knob, 1.18C31: EVENT) */
{
    const uint32_t c = (uint8_t)sqp.st[sqv.sel].off;
    uint32_t i;
    sqv.ce = (uint8_t)(set.oct + 3);     /* (-2 .. +2: 1 .. 5) */
    if (sq_pv.f & SQ_PV_AUD) sq_pv_cut();   /* (1.18C44: the audition over, ENTRY's keys heard alone) */
    sq_pv.f &= (uint8_t)~(8u | SQ_PV_AUD);   /* (a turn's preview: not ended by the key's place kept, nor the sequencer
                                          * muted through ENTRY) */
    ui.d_lcd = sqv.edited = 1;           /* (the strip: "CHORD STEP n"; the roll shown) */
    for (i = sqv.chn = 0; i < SQ_CHV; i++)   /* (a chord recorded: its notes chosen; none yet: none) */
        if ((sqv.cht[i] = c < SQ_CHN ? sqp.ch[c][i] : 0u) != 0) sqv.chn++;
}
static UI_LATE void sq_ce_end(void)              /* CHORD ENTRY over (nothing stored here) */
{
    if (sqv.ce) set.oct = (int8_t)((sqv.ce & 7u) - 3);   /* (SQ_CEE aside, 1.07) */
    sqv.ce = sqv.key = sqv.chn = 0;
}
static UI_LATE const sq_step_t *sq_cel(const sq_step_t *p, uint32_t s, sq_step_t *t)   /* audio interrupt (1.05):
                                          * step s (p) as it plays; in CHORD ENTRY with notes chosen, on its page, its
                                          * sequencer's: the chord chosen now (t, its chord SQ_CHN: sq_chord reads
                                          * sqv.cht), on, at its LENGTH, ALIGN, VEL; nothing stored (HOME: the step as
                                          * it was; REC stores it). A toggle is heard the next time the step plays.
                                          * 1.07: every note toggled out (SQ_CEE): off, silent (REC: a rest) */
{
    if (!SQ_CE || !SQ_CECH || s != sqv.sel || ui.view != V_SEQ || SQ_IIX != UN_SQI) return p;
    t->ev = (uint8_t)((p->ev & SQ_ALNEG) | (sqv.chn ? SQ_ON : 0u) | SQ_CHORD);
    t->off = (int8_t)SQ_CHN;
    t->la = p->la;
    t->vel = p->vel;
    return t;
}
static UI_LATE const uint8_t *sq_cech(void) { return sqv.cht; }   /* (sq_chord: CHORD ENTRY's notes, 1.05) */
static UI_LATE int sq_ch_key(uint32_t k, int down)   /* a key on the page: in CHORD ENTRY a note in or out (1 = taken) */
{
    uint32_t c, i, j;
    if (!SQ_CE) return 0;
    if (!down) {                         /* (1.18C31: the step key let go: CHORD ENTRY stays until REC / HOME) */
        if (!sqv.lin && k == SQ_KEY[sqv.key - 1u]) { sqv.lin = 1; sqv.lin_t = (uint16_t)plat_ms(); }
        return 1;
    }
    sq_pv_keep();                        /* (PATCH ARP OFF: its part's arpeggiator held off first, 1.11) */
    sq_rec_key(k, 1);                    /* (heard as RECORD's keys, nothing recorded: off when let go) */
    sqv.ce |= SQ_CET;                    /* (1.18C31: a note toggled: EVENT no longer leaves it while notes are chosen) */
    c = (uint32_t)((int32_t)sr.note[k] - 1 - rl_base() + 64);   /* (as the chords hold it; 0 .. 127: none) */
    if (!sr.note[k] || c - 16u > 96u) return 1;   /* (past -48 .. +48 of the base: heard alone) */
    for (i = 0; i < sqv.chn && sqv.cht[i] < c; i++)
        ;
    if (i < sqv.chn && sqv.cht[i] == c) {   /* (chosen: out; the last one out: the choice empty, SQ_CEE, 1.07) */
        sqv.ce |= SQ_CEE;
        for (sqv.chn--; i < sqv.chn; i++) sqv.cht[i] = sqv.cht[i + 1u];
        sqv.cht[i] = 0;
        return 1;
    }
    if (sqv.chn >= SQ_CHV) { app_say("CHORD", "MAX 6 NOTES"); return 1; }
    for (j = sqv.chn++; j > i; j--) sqv.cht[j] = sqv.cht[j - 1u];
    sqv.cht[i] = (uint8_t)c;
    return 1;
}
static UI_LATE uint32_t sq_ch_slot(uint32_t s, const uint8_t *v)   /* chord v (SQ_CHV offsets + 64, lowest first, 0
                                          * past the last) for step s into the pattern's chords: its index (the same
                                          * chord shared, else the first one no other step uses, s's own too);
                                          * SQ_CHN: none left, "CHORDS FULL" said, nothing changed */
{
    sq_pat_t *const q = &sqp;
    uint32_t i, j, u = 0, f = SQ_CHN;
    for (i = 0; i < SQ_NMAX; i++)        /* (the chords the other steps use) */
        if (i != s && (q->st[i].ev & 15u) == SQ_CHORD && (uint8_t)q->st[i].off < SQ_CHN)
            u |= 1u << q->st[i].off;
    for (i = SQ_CHN; i--; ) {            /* (the same chord; else the first one no other step uses) */
        for (j = 0; j < SQ_CHV && q->ch[i][j] == v[j]; j++)
            ;
        if (j == SQ_CHV) break;
        if (!(u >> i & 1u)) f = i;
    }
    if (i < SQ_CHN) f = i;
    if (f == SQ_CHN) { app_say("CHORDS", "FULL"); return f; }
    i = (q->st[s].ev & 15u) == SQ_CHORD ? (uint8_t)q->st[s].off : SQ_CHN;   /* (s's chord before: emptied if it is
                                          * left and no other step uses it, 1.05: no stale entry) */
    for (j = 0; j < SQ_CHV; j++) {
        if (i < SQ_CHN && i != f && !(u >> i & 1u)) q->ch[i][j] = 0;
        q->ch[f][j] = v[j];
    }
    return f;
}
static UI_LATE int sq_ch_put(void)               /* REC in CHORD ENTRY: the notes chosen into its step (0: CHORDS FULL,
                                          * ENTRY stays; none chosen yet: the step as it was; 1.07: every note
                                          * toggled out, SQ_CEE: no chord, a rest: the step off, its chord none, its
                                          * entry in the table emptied if no other step uses it) */
{
    sq_step_t *t = &sqp.st[sqv.sel];
    uint32_t f, i, j;
    if (!SQ_CECH) return 1;
    if (!sqv.chn) {                      /* (1.07: a rest) */
        f = (uint8_t)t->off;
        for (i = 0; f < SQ_CHN && i < SQ_NMAX; i++)   /* (the chord another step uses: kept) */
            if (i != sqv.sel && (sqp.st[i].ev & 15u) == SQ_CHORD && (uint8_t)sqp.st[i].off == f) f = SQ_CHN;
        for (j = 0; f < SQ_CHN && j < SQ_CHV; j++) sqp.ch[f][j] = 0;
        t->off = -1;
        t->ev &= (uint8_t)~SQ_ON;
    } else {
        if ((f = sq_ch_slot(sqv.sel, sqv.cht)) == SQ_CHN) return 0;
        t->off = (int8_t)f;
        t->ev |= SQ_ON;
    }
    sq_dirty(sqv.sel);                   /* (a small edit: no undo of its own, 1.18C33) */
    return 1;
}
static UI_LATE void dr_pv(uint32_t sb)   /* the preview (0.85): the instrument chosen heard
                                          * with step byte sb's ACCENT and LENGTH (its ALIGN not), as a black key
                                          * plays it (dr.prev: its voice struck again, one-shot, nothing to end); a
                                          * turn's (the step key held) mutes the drums' steps until it is let go
                                          * (0.94, sq_pvm bit 2; dr_idle) */
{
    if (DR_PATSH < 0) return;
    if (sqv.key && sqv.edited) sq_pvm |= 4u;   /* (a turn's; a tap's not) */
    dr.prev = (uint16_t)((sqv.ins + 1u) | ((sb ^ DR_LEN1 << 2) & 0x1Fu) << 4);
}
static UI_LATE void pv_tick(uint32_t now)        /* sq_idle: a turn's preview waiting (pvr), heard once its knob
                                          * rested PVR_REST or PVR_GAP passed, if still on its page, step, instrument */
{
    const uint16_t n = (uint16_t)now;
    const uint32_t k = pvr.k;
    if (!k || ((uint16_t)(n - pvr.tt) < PVR_REST && (uint16_t)(n - pvr.t0) < PVR_GAP)) return;
    pvr.k = 0;
    pvr.t0 = n;
    if (k == 1u) {
        if (ui.view == V_SEQ && sqv.key && sqv.sel == pvr.a && UN_SQI == pvr.m)
            sq_pv_play(&sqp.st[pvr.a], pvr.b ? SQ_PV_LMS : SQ_PV_MS,
                       8u | (sqv.aud == SQ_AUD_ON && !pvr.b ? SQ_PV_AUD : 0u));   /* (1.18C44: the audition's) */
    } else if (ui.view == V_DRUM && sqv.sel == pvr.a && sqv.ins == pvr.m) dr_pv(DR_ST(sqv.ins, sqv.sel));
}
static UI_LATE void dr_key(uint32_t k, int down)   /* a key on the drums' page: a white key a step of the block shown,
                                          * a black key an instrument */
{
    uint32_t i, s;
    for (i = 0; i < DR_NI && DR_KEY[i] != k; i++)
        ;
    if (i < DR_NI) {                     /* an instrument: chosen, heard */
        if (down) {
            const uint32_t v = key_vel_drum(k), acc = v >= 110u ? 2u : v >= 70u ? 1u : 0u;
            sqv.ins = (uint8_t)i;
            dr.prev = (uint8_t)((i + 1u) | acc << 4);
            mute_hint(4u);
            if (sqv.rec && dr.run && dr.on && ck.on) dr_rec_hit(i, acc);   /* (RECORD, running: onto its step) */
        }
        return;
    }
    for (i = 0; i < SQ_N && SQ_KEY[i] != k; i++)
        ;
    if (i == SQ_N) return;
    if (DR_PATSH < 0) {                  /* (PATTERN NONE, 0.84: a white key nothing, no step and no note; the black
                                          * keys play the instruments as with a pattern) */
        if (down && !ui.msg_until) seq_pick_say();
        return;
    }
    s = sqv.blk[1] * SQ_N + i;
    if (down) {
        sqv.key = (uint8_t)(i + 1u);
        sqv.sel = (uint8_t)s;
        sqv.edited = sqv.clr = 0;
        sqv.down_t = plat_ms();
    } else if (sqv.key == i + 1u) {
        if (!sqv.edited && plat_ms() - sqv.down_t < SQ_TAP_MS) {
            drp.on[s >> 4][sqv.ins] ^= (uint16_t)(1u << (s & 15u));
            if (drp.on[s >> 4][sqv.ins] >> (s & 15u) & 1u) dr_pv(DR_ST(sqv.ins, s));   /* (on: heard, 0.85) */
            dr_dirty(s);
        }
        sqv.key = 0;
    }
}
static UI_LATE void dr_knob(uint32_t c, int32_t e)   /* SELECT (c 4), K1..K4 (c 0..3): the step held, else the drum
                                          * sequencer */
{
    uint8_t *b = &DR_ST(sqv.ins, sqv.sel);
    const uint8_t b0 = *b;
    un.next = UN_KNOB(c) | (uint32_t)sqv.ins << 16;   /* (a small edit) */
    if (pk.ask) { pk.ask = 0; ui.d_lcd = 1; return; }   /* (1.18C33: the question up: a turn takes it back) */
    if (!sqv.key) {
        if (c == 0u) {                   /* LENGTH: 16, 32, 64 (the steps past it kept) */
            const uint32_t l = (uint32_t)clamp((int32_t)drp.lc + e, 0, SQ_NLC - 1);
            if (l != drp.lc) { drp.lc = (uint8_t)l; seq_blk_fix(); dr_dirty(SQ_NMAX); }
        } else if (c == 1u) { drp.div = (uint8_t)clamp((int32_t)drp.div + e, 0, SQ_NDIV - 1); dr_dirty(SQ_NMAX); }
        else if (c == 2u) {
            const int32_t p0 = set.sq_pad & DR_DISARM ? -1 : (int32_t)set.dr_pat;
            const int32_t p1 = clamp(SQ_QPAT(2u, p0) + e, -1, DR_PATS - 1);
            const uint32_t n = p1 < 0 ? set.dr_pat : (uint32_t)p1;
            if (sq_qwant(2u, p0, p1)) { sqv.clr = 0; sq_title(6u + c); return; }   /* (0.96: running, at the bar's
                                          * line) */
            if (p1 >= 0 && n != set.dr_pat && !pk_gate(0u, n)) return;   /* (1.18C33: its edits fit nowhere: asked) */
            if (p1 != p0) seq_none(1, p1 < 0);
            if (n != set.dr_pat) {
                set.dr_pat = (uint8_t)n;
                dr_load(n);
                seq_blk_fix();
                glo_touch();
            }
        } else if (c == 3u) {
            sqv.clr = 1;
            sqv.clr_t = plat_ms();
            sqv.tt = 0;
            app_say("CLEAR?", "REC");
            return;
        } else {                         /* SELECT: the level */
            set.dr_lev = (uint8_t)clamp((int32_t)set.dr_lev + knob_steps(c, e), 0, DR_LMAX);
            dr_level();                  /* (the gain; above 100 the drive) */
            glo_touch();
        }
        if (c < 4u) sqv.clr = 0;
        sq_title(6u + c);
        return;
    }
    if (c == 0u) *b = (uint8_t)((*b & ~3u) | (uint32_t)clamp((int32_t)(*b & 3u) + e, 0, DR_NACC - 1));
    else if (c == 1u) *b = (uint8_t)((*b & ~0x1Cu) | (uint32_t)clamp((int32_t)(*b >> 2 & 7u) + e, 0, DR_NLEN - 1) << 2);
    else if (c == 2u) {                  /* (ALIGN -7/8 .. +7/8, 0 sticky: as the arp sequencer's, sq_dz) */
        const int32_t a = clamp(sq_dz(DR_AL(sqv.ins, sqv.sel), e, 3u), -7, 7);
        const uint16_t g = (uint16_t)(1u << (sqv.sel & 15u));
        uint16_t *n = &drp.neg[sqv.sel >> 4][sqv.ins];
        *b = (uint8_t)((*b & 0x1Fu) | (uint32_t)(a < 0 ? -a : a) << 5);
        *n = (uint16_t)(a < 0 ? *n | g : *n & ~g);
    } else if (c == 3u) {
        uint16_t *o = &drp.on[sqv.sel >> 4][sqv.ins];
        if (e > 0) *o |= (uint16_t)(1u << (sqv.sel & 15u));
        else *o &= (uint16_t)~(1u << (sqv.sel & 15u));
    } else return;                       /* (SELECT: nothing for a step) */
    sqv.edited = 1;
    sq_title(1u + c);
    dr_dirty(sqv.sel);
    if (c < 2u && *b != b0 && pv_ask(2u, sqv.sel, sqv.ins, 0u)) dr_pv(*b);   /* (ACCENT, LENGTH changed: the hit
                                          * heard, 0.85; at most every PVR_GAP, else the latest later, 0.86) */
}
static UI_LATE int dr_rec(void)                  /* REC on the page: CLEAR asked, the pattern cleared (1 = taken) */
{
    const uint8_t lc = drp.lc;
    char t[12];
    if (ui.view != V_DRUM || !sqv.clr) return 0;
    sqv.clr = 0;
    dr_clr_to(&drp, lc);                 /* (all steps off, their values and the time division as shipped, its LENGTH
                                          * kept) */
    un_clear(0u);                        /* (into the journal, one entry: undone by UNDO, written by SAVE, 1.18C33) */
    app_say(pat_text(t, "Drums ", set.dr_pat + 1), un.nou ? "CLEARED NO UNDO" : "CLEARED");
    return 1;
}
static UI_LATE uint32_t un_step(uint32_t q, uint32_t o, uint32_t old)   /* the step record o (old byte) is of,
                                          * SQ_NMAX: the pattern's settings */
{
    const uint32_t no = SQ_NB * DR_NI * 2u, ns = SQ_NB * DR_NI * SQ_N;
    if (q) return o < SQ_NMAX * 4u ? o / 4u : o < SQ_NMAX * 4u + 4u ? SQ_NMAX : SQ_NMAX + 2u;   /* (+ 2: a chord's,
                                          * 1.02: the head's, no step of its own) */
    if (o < no || o - no - ns < no) {    /* (an instrument's on bits or align signs: the lowest changed) */
        const uint32_t p = o < no ? o : o - no - ns;
        uint32_t d = (old ^ un_cur(0)[o]) & 0xFFu, k = 0;
        while (d && !(d & 1u)) { d >>= 1; k++; }
        return p / (DR_NI * 2u) * SQ_N + (p & 1u) * 8u + k;
    }
    return o - no < ns ? (o - no) / (DR_NI * SQ_N) * SQ_N + (o - no) % SQ_N : SQ_NMAX;
}
static UI_LATE void un_undo(void)                /* (1.18C33) the page's pattern back to before its last big change
                                          * (CLEAR, a RECORD pass, DUPLICATE, an instrument ERASED) and the small edits
                                          * after it: its entries from that level on dropped, the pattern loaded again
                                          * (the store's bytes and the entries before); another pattern's never */
{
    const uint32_t q = ui.view == V_DRUM ? 0u : 1u + UN_SQI, k = jr_key(q);
    int32_t g = -1;
    uint32_t i, clr;
#ifdef HYB_MULTI
    if (q && sq_busy()) return;          /* (NONE on the other's pattern: its edits the other's) */
#endif
    if (!pk_on(k)) g = jr_lvl(k, un.n);
    if (g < 0) { app_say("Undo", "NOTHING"); return; }
    clr = (uint32_t)jr_is_clr(un.e[g]);
    for (i = (uint32_t)g; i < un.n; i++)
        if (jr_pat(un.e[i]) == k) un.e[i] = 0;
    jr_sweep();
    if (k == un.k) un.key = 0;
    if (q) sq_load(k - DR_PATS); else dr_load(k);
    seq_blk_fix();
    app_say("UNDO", clr ? "CLEAR" : "PATTERN");
}
static UI_LATE void seq_dup(void)                /* EDIT > DUPLICATE: the pattern twice as long, its steps again after
                                          * them (all of each step; the drums' every instrument) */
{
    const uint32_t d = ui.view == V_DRUM, lc = d ? drp.lc : sqp.lc, nb = 1u << lc;
    uint32_t b, i, k;
    char t[12];
    if (lc + 1u >= SQ_NLC) { app_say("Duplicate", "MAX 64"); return; }
#ifdef HYB_MULTI
    if (!d && sq_busy()) return;
#endif
    un.next = UN_BIG | ++un.seq;         /* (an undo level of its own, 1.18C33) */
    if (d) {
        for (b = 0; b < nb; b++)
            for (i = 0; i < DR_NI; i++) {
                drp.on[nb + b][i] = drp.on[b][i];
                drp.neg[nb + b][i] = drp.neg[b][i];
                for (k = 0; k < SQ_N; k++) drp.st[nb + b][i][k] = drp.st[b][i][k];
            }
        drp.lc = (uint8_t)(lc + 1u);     /* (after the steps: the audio interrupt plays them at once) */
        dr_dirty(SQ_N);
    } else {
        for (k = 0; k < nb * SQ_N; k++) sqp.st[nb * SQ_N + k] = sqp.st[k];
        sqp.lc = (uint8_t)(lc + 1u);
        sq_dirty(SQ_N);
    }
    seq_blk_fix();
    app_say("DUPLICATED", un.nou ? "NO UNDO" : pat_text(t, "LENGTH ", (int32_t)(SQ_STEPS(lc + 1u))));
}
/* EDIT > COPY TO (0.66, 13): the page's pattern as it is in RAM (its unsaved edits too: every
 * step of every block, LENGTH, DIV; the drums' every instrument) onto another pattern of the same sequencer, written
 * into the store at once (an explicit save, as SAVE and CLEAR: also while the transport runs, the erases' moment of
 * silence then heard; one erase per object, the head's and the rest's, an object none of whose bytes change not
 * written). The pattern edited stays the one chosen and its own edits stay unsaved (AUTO SAVE OFF's rule): the
 * objects it shares with the destination get the destination's bytes alone, its own as stored. Not undone by UNDO
 * (another pattern changed). The chooser, a list over the page: the patterns but the one edited, then BACK; a
 * pattern saved once "USED", the other arp sequencer's (two parts) "SEQ A" / "SEQ B"; SELECT moves, REC copies (onto
 * those after "OVERWRITE n? REC", REC again), HOME or BACK goes back to the EDIT list, EDIT closes both. Onto the
 * other arp sequencer's pattern: its copy in RAM replaced in place (it plays the new steps from its next step, as a
 * pattern loaded: sq_load), its parked edits dropped, its undo started again */
static UI_LATE uint32_t cp_n(void) { return ui.view == V_DRUM ? DR_PATS : SQ_PATS; }   /* the chooser's lines (the
                                          * patterns but one, BACK) */
static UI_LATE uint32_t cp_pat(uint32_t i)       /* line i's pattern (i < cp_n() - 1) */
{
    return i < (ui.view == V_DRUM ? set.dr_pat : SQ_PATV) ? i : i + 1u;
}
static UI_LATE int32_t cp_other(void)            /* the pattern the other arp sequencer plays (-1: none, the drums') */
{
#ifdef HYB_MULTI
    if (ui.view != V_DRUM) return sq_other();
#endif
    return -1;
}
static UI_LATE int cp_saved(uint32_t n)          /* pattern n of the page's sequencer was saved (its head's or its
                                          * rest's mark in the store) */
{
    uint8_t m;
    if (ui.view == V_DRUM)
        return (plat_store_read(OBJ_DRUM, n * DR_REC + DR_REC - 2u, &m, 1u) == 1 && m == DR_MARK)
               || (plat_store_read(OBJ_DRUMX + n / DR_XPER, (n % DR_XPER) * DR_XREC + DR_XREC - 2u, &m, 1u) == 1
                   && m == DR_XMARK);
    return (plat_store_read(OBJ_SEQ, n * SQ_REC + SQ_N * 4u, &m, 1u) == 1 && m == SQ_MARK)
           || (plat_store_read(OBJ_SEQX + n / SQ_XPER, (n % SQ_XPER) * SQ_XREC + SQ_XREC - 4u, &m, 1u) == 1
               && m == SQ_XMARK);
}
static UI_LATE void cp_item(uint32_t i, char *t, char *pre, uint32_t *fl)   /* "PATTERN 7  USED", BACK */
{
    const uint32_t n = cp_pat(i);
    (void)pre; (void)fl;
    if (i + 1u >= cp_n()) { text_cat(t, "BACK"); return; }
    pat_text(t, "PATTERN ", (int32_t)n + 1);
    if ((int32_t)n == cp_other()) text_cat(t, UN_SQI ? "\tSEQ A" : "\tSEQ B");
    else if (sqv.cp_used >> n & 1u) text_cat(t, "\tUSED");
}
static UI_LATE void cp_open(void)                /* EDIT > COPY TO: the chooser, on the first pattern but the one
                                          * edited; the patterns saved or with edits waiting (the journal) looked up
                                          * once */
{
    const uint64_t w = jr_pats() >> (ui.view == V_DRUM ? 0u : DR_PATS);
    uint32_t n;
    sqv.cp_used = 0;
    for (n = 0; n < cp_n(); n++)
        if ((w >> n & 1u) || cp_saved(n)) sqv.cp_used |= 1u << n;
    sqv.cp = 1;
    sqv.cp_ask = 0;
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void cp_say(const char *a, uint32_t n)   /* a message, "PATTERN n" */
{
    char t[12];
    app_say(a, pat_text(t, "PATTERN ", (int32_t)n + 1));
}
static UI_LATE int cp_copy(uint32_t n)           /* the page's pattern onto pattern n, written (n's edits waiting
                                          * dropped: its bytes replaced): 0 = done */
{
    const uint32_t d = ui.view == V_DRUM;
    const int r = d ? dr_store(&drp, n, 3u) : sq_store(&sqp, n, 3u);
    if (r == 0) jr_drop((uint64_t)1 << (d ? n : DR_PATS + n));
#ifdef HYB_MULTI
    {                                    /* (the other arp sequencer's pattern, chosen or kept under NONE: its RAM) */
        const uint32_t o = !sq_mi;
        if (!d && r == 0 && (o ? set.sqb_pat : set.sq_pat) == n) {
            const uint8_t *s = (const uint8_t *)&sqp;
            uint8_t *t = (uint8_t *)&sqpa[o];
            uint32_t k;
            for (k = 0; k < sizeof(sq_pat_t); k++) t[k] = s[k];
            sq_cut[o] = 1u;              /* (its notes of the pattern before off) */
            un_load(1u + o);
        }
    }
#endif
    return r;
}
static UI_LATE void cp_rec(void)                 /* REC in the chooser: the line's */
{
    const uint32_t i = sqv.cp - 1u, n = cp_pat(i);
    if (i + 1u >= cp_n()) { sqv.cp = 0; ui.d_rows = ui.d_lcd = 1; return; }   /* (BACK: the EDIT list) */
    if (SQ_RUNS || dr.run) { app_say("COPY TO", "STOP FIRST"); return; }   /* (after 1.00: no flash write while
                                          * the beat runs) */
    if (!sqv.cp_ask && ((sqv.cp_used >> n & 1u) || (int32_t)n == cp_other())) {
        char t[16];
        text_cat(pat_text(t, "OVERWRITE ", (int32_t)n + 1), "?");
        sqv.cp_ask = 1;
        app_say(t, "REC");
        return;
    }
    sqv.cp = sqv.cp_ask = 0;
    sqv.menu = 0;                        /* (done: the list closed, as DUPLICATE) */
    ui.d_rows = ui.d_lcd = 1;
    if (cp_copy(n) != 0) cp_say("COPY FAILED", n);
    else cp_say("COPIED TO", n);
}
/* EDIT on a sequencer's page (13): a list over the page, SAVE PATTERN (0.67, the cursor's first
 * line), DUPLICATE, COPY TO, UNDO, PATCH ARP (0.71), KEY LATCH (0.75), REC QUANT (1.14; the three the arp sequencers'
 * alone, the pattern's settings), DRUM KIT (0.74, the drums' alone: the kit's page, kit_dseq), BACK; SELECT moves, REC
 * does the line's (SAVE PATTERN and DUPLICATE close the list, COPY TO opens its chooser, UNDO and the settings keep it), BACK,
 * HOME or EDIT closes it. K1..K4 do nothing while it is up; the keys keep playing / editing the page. The page left:
 * closed (dr_idle) */
enum { SE_SAVE, SE_DUP, SE_COPY, SE_UNDO, SE_PARP, SE_KLAT, SE_RQ, SE_KIT, SE_BACK, SE_N };
static const char *const SE_ITEM[SE_N] = {"SAVE PATTERN", "DUPLICATE", "COPY TO", "UNDO", "PATCH ARP", "KEY LATCH",
                                          "REC QUANT", "DRUM KIT", "BACK"};
_Static_assert(SQ_PARP == 1u << (SE_PARP - SE_PARP) && SQ_NLAT == 1u << (SE_KLAT - SE_PARP)
               && SQ_NRQ == 1u << (SE_RQ - SE_PARP), "EDIT's settings: the pad's bits in their order");
static UI_LATE void kit_dseq(void);
static UI_LATE void sp_edit(void);
static UI_LATE int sp_lcd(char *a, char *v);
static UI_LATE uint32_t se_n(void) { return ui.view == V_DRUM ? SE_N - 3u : SE_N - 1u; }   /* the list's lines (the
                                          * drums': no PATCH ARP, KEY LATCH, REC QUANT; the arp sequencers': no DRUM
                                          * KIT) */
static UI_LATE uint32_t se_line(uint32_t i)   /* line i's item (SE_*) */
{
    if (ui.view == V_DRUM) return i >= SE_PARP ? i + 3u : i;
    return i >= SE_KIT ? i + 1u : i;
}
#define SE_FLAG(i) ((i) >= SE_PARP && (i) <= SE_RQ)   /* (a pattern's setting, ON / OFF: SQ_FON's bit 1 << (i - SE_PARP)) */
static UI_LATE void se_item(uint32_t i, char *t, char *pre, uint32_t *fl)   /* "PATCH ARP  OFF" */
{
    (void)pre; (void)fl;
    i = se_line(i);
    text_cat(t, SE_ITEM[i]);
    if (SE_FLAG(i)) text_cat(t, SQ_FON(sqp.pad, 1u << (i - SE_PARP)) ? "\tON" : "\tOFF");
}
static UI_LATE void se_flag(uint32_t i)  /* EDIT > PATCH ARP (SQ_PARP: the patch's arpeggiator not held off, its hold
                                          * follows, sq_hold_sync), KEY LATCH (SQ_NLAT; the latch cleared: OFF the old
                                          * way, the BASE NOTE once the keys are let go), REC QUANT (SQ_NRQ, 1.14:
                                          * RECORD's keys onto the nearest step, or OFF their eighth kept as ALIGN):
                                          * the pattern's ON / OFF, kept in it (an edit: the journal, UNDO, SAVE) */
{
    const uint32_t b = 1u << (i - SE_PARP);
#ifdef HYB_MULTI
    if (sq_busy()) return;
#endif
    sqp.pad ^= (uint8_t)b;
    if (b == SQ_NLAT) sq.lclr = 1;
    sq_dirty(SQ_NMAX);
    say_on(SE_ITEM[i], SQ_FON(sqp.pad, b));
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void se_close(void)
{
    sqv.menu = sqv.cp = sqv.cp_ask = 0;
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void se_home(void)                /* HOME with the list up: the chooser back to the list, the list closed */
{
    if (sqv.cp) { sqv.cp = sqv.cp_ask = 0; ui.d_rows = ui.d_lcd = 1; }
    else se_close();
}
static UI_LATE void se_open(void)                /* EDIT on the page: the list, or closed again */
{
    if (se_on()) { se_close(); return; }
    sqv.menu = 1;
    sqv.cp = sqv.cp_ask = 0;
    sqv.menu_v = (uint8_t)ui.view;
    sqv.key = sqv.clr = sqv.tt = 0;
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void se_select(int32_t e)
{
    if (sqv.cp) {
        sqv.cp = (uint8_t)clamp((int32_t)sqv.cp + e, 1, (int32_t)cp_n());
        sqv.cp_ask = 0;
    } else
        sqv.menu = (uint8_t)clamp((int32_t)sqv.menu + e, 1, (int32_t)se_n());
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE int se_rec(void)                  /* REC with the list up (1 = taken) */
{
    uint32_t i;
    if (!se_on()) return 0;
    i = se_line(sqv.menu - 1u);
    if (sqv.cp) cp_rec();
    else if (i == SE_SAVE) { se_close(); sp_edit(); }
    else if (i == SE_DUP) { seq_dup(); se_close(); }
    else if (i == SE_COPY) cp_open();
    else if (i == SE_UNDO) { un_undo(); ui.d_lcd = 1; }
    else if (SE_FLAG(i)) se_flag(i);
    else if (i == SE_KIT) { se_close(); kit_dseq(); }
    else se_close();
    return 1;
}
static UI_LATE int se_lcd(char *a, char *v)      /* the strip with the list up: the page's EDIT, the line; the chooser:
                                          * COPY TO, the line, or its question (1 = so) */
{
    if (sp_lcd(a, v)) return 1;
    if (!se_on()) return 0;
    if (sqv.cp) {
        const uint32_t i = sqv.cp - 1u;
        if (sqv.cp_ask) {
            text_cat(pat_text(a, "OVERWRITE ", (int32_t)cp_pat(i) + 1), "?");
            text_cat(v, "REC");
        } else {
            text_cat(a, "COPY TO");
            if (i + 1u >= cp_n()) text_cat(v, "BACK");
            else put_sint(v, (int32_t)cp_pat(i) + 1, 0);
        }
        return 1;
    }
    text_cat(a, ui.view == V_DRUM ? "DRUM EDIT" : "SEQ EDIT");
    se_item(sqv.menu - 1u, v, 0, 0);
    if (SE_FLAG(se_line(sqv.menu - 1u))) v[9] = ' ';   /* ("PATCH ARP ON", "KEY LATCH ON", "REC QUANT ON": the tab a
                                          * space) */
    return 1;
}
static UI_LATE uint32_t dr_has(uint32_t i)       /* instrument i has a step on within the pattern's length */
{
    uint32_t b, h = 0;
    for (b = 0; b < 1u << drp.lc; b++) h |= drp.on[b][i];
    return h;
}
#define DR_BLINK_MS 60u                  /* an instrument struck: its key dark this long */
static UI_LATE uint32_t dr_lights(uint32_t *dim, uint32_t *mid)   /* the keys on the drums' page: the white keys the
                                          * chosen instrument's steps of the block shown (on: lit; the beats' steps
                                          * off: the middle level; the step sounding inverted while it runs, in that block); the
                                          * black keys the instruments (with steps: lit, a hit at the step sounding
                                          * dark DR_BLINK_MS; without: dim; the chosen one without steps: the middle
                                          * level; PATTERN NONE: all keys at the middle level) */
{
    static uint16_t seen PLAT_POOL;     /* (in the pool: the Bluetooth build's RAM is full) */
    static uint32_t t_hit PLAT_POOL;
    const uint32_t bs = sqv.blk[1];
    uint32_t i, m = 0, d = 0, h = dr.run ? dr.hit : 0u, blink;
    const uint32_t cur = dr.run && (uint32_t)(dr.step >> 4) == bs ? 1u << SQ_KEY[dr.step & 15u] : 0u;
    const uint16_t on = drp.on[bs][sqv.ins];
    const uint32_t now = plat_ms();
    if (DR_PATSH < 0) {                  /* (PATTERN NONE, 0.84: the black keys play the instruments, the chosen one
                                          * lit, the others at the middle level, as on the kit's page; the white keys
                                          * dark, they do nothing) */
        for (i = 0; i < DR_NI; i++) m |= 1u << DR_KEY[i];
        *dim = 0;
        *mid = m & ~(1u << DR_KEY[sqv.ins]);
        return 1u << DR_KEY[sqv.ins];
    }
    if (h & ~(uint32_t)seen) t_hit = now;   /* (a new hit: the blink from now) */
    seen = (uint16_t)h;
    blink = now - t_hit < DR_BLINK_MS ? h : 0u;
    for (i = 0; i < SQ_N; i++)
        if (on >> i & 1u) m |= 1u << SQ_KEY[i];
    m ^= cur;
    *mid = SQ_BEATS & ~m & ~cur;         /* (the beats off: the middle level, as on the arp sequencer's page) */
    for (i = 0; i < DR_NI; i++) {
        const uint32_t k = 1u << DR_KEY[i];
        if (!dr_has(i)) { if (i == sqv.ins) *mid |= k; else d |= k; }
        else if (!(blink >> i & 1u)) m |= k;
    }
    *dim = d;
    return m;
}

/* ----------------------------------------------------------- the drum kit --- */
/* GLO > DRUM KIT (V_KIT): each instrument's sound, one kit for all patterns. The black keys
 * choose the instrument and play it (the accent from the key's velocity, as on the drums' page), the white keys play
 * the one chosen; neither plays the synth here, and the drum sequencer goes on. Three blocks (SELECT; two rows, the
 * page scrolls to the third): SOUND, LEVEL, TUNE, DECAY, ATTACK (BD and the toms: CLICK); VOICE, TONE (BD and the
 * toms: SWEEP), MODEL 808 / 909 (BD SD LT MT HT), PAN, SEND (0.88: into part A's delay / reverb, kit_send); BUS,
 * the drums' bus (not an instrument's): LEVEL (set.dr_lev, the drum sequencer's LEVEL), DRIVE (the curve, DV_*), COMP
 * (DC_*), heard at once (dr_level), the one turned up in the strip, and K4 the instrument's RESET (0.88, VOICE's K4
 * before; turned: "REC?" for SQ_CLR_MS, REC: the instrument as shipped). An instrument's change sounds
 * from the next hit (dr_hit). The kit is saved after the patterns in OBJ_DRUM DR_SAVE_MS after its last change (the
 * level with GLO's settings). HOME: back to GLO's
 * DRUM KIT line (opened from the drums' EDIT list: to the drums' page, 0.74); SEQ: the drums' page (0.74). The page: ui.c kit_draw */
#define KIT_M9 (1u << DR_BD | 1u << DR_CH | 1u << DR_OH)   /* (1.18C48, the user's choice) MODEL as shipped: 909 (BD),
                                          * SMP (the hats: the sampled hat where built, HAT_HAVE); the others 808 / SYN.
                                          * Until 1.18C47 all 808 / SYN. A new kit (none stored), an instrument's RESET
                                          * and an instrument stored not valid take it; a kit stored keeps its own */
static UI_LATE dr_kp_t kit_p0(uint32_t i)   /* instrument i as shipped */
{
    dr_kp_t p = DR_KP0;
    p.m909 = (uint8_t)(!DEF47 && (KIT_M9 >> i & 1u));
    return p;
}
static UI_LATE int kit_valid(const dr_kp_t *q)
{
    return q->lev <= DR_KLMAX && q->tune <= DR_KN && q->dec <= DR_KN && q->att <= DR_KATT && q->tone <= DR_KN
           && q->pan >= -DR_KPAN && q->pan <= DR_KPAN;
}
static UI_LATE void kit_load(void)       /* app_init: the kit from the store (none, or an instrument not valid: as
                                          * shipped) */
{
    dr_kit_t q;
    uint32_t i;
    const int ok = plat_store_read(OBJ_DRUM, DR_KIT_OFF, &q, sizeof q) == (int)sizeof q && q.mark == DR_KMARK
                   && q.ver == DR_KV1;
    for (i = 0; i < DR_NI; i++) {
        drk.k[i] = ok && kit_valid(&q.k[i]) ? q.k[i] : kit_p0(i);
#ifdef HYB_TEST_DSEND                    /* (a test build: every instrument's SEND, for the emulator's load) */
        drk.k[i].snd = HYB_TEST_DSEND;
#endif
#ifdef HYB_TEST_HAT                      /* (a test build: the hats' MODEL, for the emulator's load; 0.98) */
        if (i == DR_CH || i == DR_OH) drk.k[i].m909 = HYB_TEST_HAT;
#endif
    }
    drk.mark = DR_KMARK;
    drk.ver = DR_KV1;
    drk.drv = (uint8_t)(ok && (q.drv & 15u) < DV_N && (q.drv >> 4) < DC_N ? q.drv : DV_WARM);
#ifdef HYB_TEST_DRV                      /* (1.18C42, a test build: the bus's DRIVE and COMP, for the emulator's load) */
    drk.drv = HYB_TEST_DRV;
#endif
    kv.dirty_t = 0;
    kv.ask = kv.blk = kv.top = kv.tt = 0;
}
static UI_LATE void kit_save(void)       /* the kit into its place, if changed */
{
    if (!kv.dirty_t) return;
    kv.dirty_t = 0;
    if (store_write(OBJ_DRUM, DR_KIT_OFF, &drk, sizeof drk) != 0) app_say("Drum kit", "SAVE FAILED");
}
/* AUTO SAVE OFF (0.65, as shipped): nothing is saved by itself; SAVE saves what waits (sv_all: the patterns with
 * edits in the journal, the kit, GLO's settings), its light lit while anything waits (sv_unsaved) */
static UI_LATE int sv_unsaved(void)              /* a pattern, the kit or GLO's settings changed and not saved */
{
    return JR_ANY || kv.dirty_t || ui.glo_t;
}
static UI_LATE int sp_runs(void) { return SQ_RUNS || dr.run; }   /* the beat runs: no flash write (after 1.00) */
#ifdef HYB_MULTI
static UI_LATE void pv_pend(void)        /* (1.19.7) SAVE pressed: a volume moved by VOL A / B CC (which marks nothing, so
                                          * a pedal never lights SAVE) or a launcher set is pending now, as a turn */
{
    if (pv_vol[0] != pv_saved(0) || pv_vol[1] != pv_saved(1)) ui.glo_t = plat_ms() | 1u;
}
#else
#define pv_pend() ((void)0)
#endif
static UI_LATE int sv_all(void)                  /* all that waits saved now (1: something was); the patterns while
                                          * the beat runs at its stop (sp_idle) */
{
    int any;
    pv_pend();
    any = sv_unsaved();
    if (sp_runs()) {                     /* (the kit and GLO's settings too: sqv.sv_w) */
        if (JR_ANY) sqv.sp_wait = 0xFFu;
        sqv.sv_w |= 3u;
        return any;
    }
    if (JR_ANY && jr_store(jr_pats()) != 0) app_say("Pattern", "SAVE FAILED");
    kit_save();
    if (ui.glo_t) {
        ui.glo_t = 0;
        if (glo_save() != 0) app_say("Save", "FAILED");
    }
    return any;
}
/* The sequencers' patterns saved by hand (0.67, 12, 13; after 1.00 the development notes): EDIT >
 * SAVE PATTERN, the page's pattern; SAVE on a sequencer's page, every pattern with edits waiting in the journal (the
 * drums', the arp patterns'), after "SAVE PATTERN 3?  REC" (REC saves; any other button, a knob turned or SQ_CLR_MS
 * cancel). They write the patterns' objects alone: never the patch, the kit or GLO's settings. While the beat runs
 * (after 1.00, the user's choice: timing first) nothing is written: "SAVE PENDING", saved at the transport's stop
 * (sp_idle; 0.68-1.00 wrote at once, a moment's dropout). A pattern written: its entries dropped. sp_* take w: 0 all
 * the patterns with edits, else a pattern's key + 1 (jr_key) */
static UI_LATE uint64_t jr_pats(void)            /* the patterns with entries (bit k: key k) */
{
    uint64_t w = 0;
    uint32_t i;
    for (i = 0; i < un.n; i++) w |= (uint64_t)1 << jr_pat(un.e[i]);
    return w | pk.m;                     /* (1.18C33: the marked ones, their RAM copies) */
}
static UI_LATE int jr_store(uint64_t w)          /* the patterns w (bit k: key k) written as loaded (the store's bytes
                                          * and their entries), each its parts with entries (the head, the rest);
                                          * their entries dropped: 0 = done */
{
    union { dr_pat_t d; sq_pat_t s; } p;
    uint32_t k, i, o, x, m;
    int r = 0;
    for (k = 0; k < JR_K; k++) {
        if (!(w >> k & 1u)) continue;
        for (i = 0, m = pk_on(k) ? 3u : 0u; i < un.n; i++)   /* (marked: all of it, 1.18C33) */
            if ((o = jr_of(un.e[i], k)) != ~0u) {
                if (o == JR_CLR(k)) { m = 3u; continue; }   /* (a CLEAR: all of it) */
                x = un_step(k >= DR_PATS, o, 0u);
                m |= x < SQ_N || x >= SQ_NMAX ? 1u : 2u;
            }
        if (k < DR_PATS) { dr_load_to(&p.d, k); r |= dr_parts(&p.d, k, m, store_stage); }
        else { sq_load_to(&p.s, k - DR_PATS); r |= sq_parts(&p.s, k - DR_PATS, m, store_stage); }
    }
    r |= store_flush();
    if (r == 0) jr_drop(w);
    return r;
}
static UI_LATE uint64_t sp_set(uint32_t w)       /* what w names that has edits (bit k: key k) */
{
    return w ? jr_pats() & (uint64_t)1 << (w - 1u) : jr_pats();
}
static UI_LATE uint32_t sp_page(void) { return 1u + jr_key(ui.view == V_DRUM ? 0u : 1u + UN_SQI); }   /* the page's */
static UI_LATE void sp_name(char *t, uint32_t w) /* "PATTERN 3" (the page's), "DRUM PAT 2", "ARP PAT 1"; several:
                                          * "2 PATTERNS"; none: the page's */
{
    const uint64_t m = sp_set(w);
    uint32_t n = 0, k = w ? w - 1u : sp_page() - 1u, i;
    for (i = JR_K; i--; )
        if (m >> i & 1u) { n++; k = i; }
    if (n > 1u) { put_sint(t, (int32_t)n, 0); text_cat(t, " PATTERNS"); return; }
    text_cat(t, k + 1u == sp_page() ? "PATTERN " : k < DR_PATS ? "DRUM PAT " : "ARP PAT ");
    while (*t) t++;
    put_sint(t, (int32_t)(k < DR_PATS ? k : k - DR_PATS) + 1, 0);
}
static UI_LATE void sp_go(uint32_t w)            /* what w names saved now; the beat running: at its stop */
{
    char t[20];
    const uint64_t m = sp_set(w);
    sqv.sp_ask = 0;
    t[0] = 0;
    sp_name(t, w);
    if (!m) { app_say(t, "UNCHANGED"); return; }
    if (sp_runs()) {
        sqv.sp_wait = (uint8_t)(sqv.sp_wait && sqv.sp_wait != (w ? w : 0xFFu) ? 0xFFu : w ? w : 0xFFu);
        app_say(t, "SAVE PENDING");
        return;
    }
    app_say(t, jr_store(m) != 0 ? "SAVE FAILED" : "SAVED");
}
static UI_LATE void sp_edit(void) { sp_go(sp_page()); }   /* EDIT > SAVE PATTERN: the page's */
static UI_LATE int sp_on(void);
static UI_LATE void sp_button(void)              /* SAVE on a sequencer's page: the question (nothing waits: said) */
{
    char t[24];
    if (sp_on()) { sqv.sp_ask = 0; ui.d_lcd = 1; return; }   /* (SAVE again: taken back) */
    if (!JR_ANY) { sp_go(0); return; }
    sqv.sp_ask = 1;
    sqv.sp_t = plat_ms();
    sqv.sp_b = plat_buttons();           /* (those down now: sp_idle) */
    t[0] = 0;
    text_cat(t, "SAVE ");
    sp_name(t + 5, 0);
    text_cat(t, "?");
    app_say(t, "REC");
}
static UI_LATE int sp_on(void)                   /* SAVE's question up */
{
    return sqv.sp_ask && (ui.view == V_SEQ || ui.view == V_DRUM) && plat_ms() - sqv.sp_t < SQ_CLR_MS;
}
#ifdef HYB_MULTI
static UI_LATE int pa_rec(void)                  /* (after 0.98) REC on part B's row of part A's delay / reverb (ui.c
                                          * p_parta): part A edited, the same page (1 = taken) */
{
    const int32_t r = row_on();
    uint32_t k;
    if (r < 0) return 0;
    for (k = 0; k < 4u && !p_parta(&U_ROW[r].p[k]); k++)
        ;
    if (k == 4u) return 0;
    part_edit(0);
    return 1;
}
#endif
static UI_LATE int sp_rec(void)                  /* REC: the question answered (1 = taken; 0.96: the patch list's
                                          * RESET PATCH too, rp_rec: input's REC chain unchanged; after 0.98 part B's
                                          * way to part A's delay / reverb, pa_rec) */
{
    if (fv_rec()) return 1;              /* (1.18C33: the favorites) */
    if (rp_rec()) return 1;
#ifdef HYB_MULTI
    if (pa_rec()) return 1;
#endif
    if (!sp_on()) return 0;
    sp_go(0);
    return 1;
}
/* (1.18C33) a marked pattern that fits nowhere as its sequencer is about to leave it (pk_gate): the strip asks
 * "PATTERN 3 UNSAVED" "REC SAVE  HOME DROP" until answered; REC saves it (the beat running: at its stop, "SAVE
 * PENDING", the change not made), then the change; HOME drops its edits not in the journal (it is loaded again: the
 * store's bytes and its entries), then the change; another button or PATTERN turned: nothing, it stays */
#ifdef HYB_MULTI
static UI_LATE void ln_part(uint32_t q, int32_t p1);
#endif
static UI_LATE void pk_set(uint32_t h, uint32_t n)   /* holder h's sequencer to pattern n, PATTERN's way */
{
#ifdef HYB_MULTI
    ln_part(h ? h - 1u : 2u, (int32_t)n);
#else
    if (!h) {
        const int32_t p0 = DR_PATSH;
        if (sq_qwant(2u, p0, (int32_t)n)) return;
        if ((int32_t)n != p0) seq_none(1, 0);
        if (n != set.dr_pat) { set.dr_pat = (uint8_t)n; dr_load(n); glo_touch(); }
    } else {
        const int32_t p0 = SQ_PATSH;
        if (sq_qwant(0u, p0, (int32_t)n)) return;
        sq_pv_cut();
        pvr.k = 0;
        if ((int32_t)n != p0) seq_none(0, 0);
        if (n != SQ_PATV) { SQ_PATV = (uint8_t)n; sq_load(n); glo_touch(); }
    }
#endif
    seq_blk_fix();
}
static UI_LATE int pk_rec(void)          /* REC on the question (1 = taken) */
{
    const uint32_t k = pk.ask - 1u, h = pk.aq, n = pk.want;
    char t[12];
    if (!pk.ask) return 0;
    pk.ask = 0;
    ui.d_lcd = 1;
    if (sp_runs()) { sp_go(k + 1u); return 1; }   /* (the beat running: at its stop, as SAVE's) */
    pk_name(t, k);
    if (jr_store((uint64_t)1 << k) != 0) { app_say(t, "SAVE FAILED"); return 1; }
    app_say(t, "SAVED");
    if (n) pk_set(h, n - 1u);
    return 1;
}
static UI_LATE int pk_home(void)         /* HOME on the question (1 = taken) */
{
    const uint32_t k = pk.ask - 1u, h = pk.aq, n = pk.want;
    char t[12];
    if (!pk.ask) return 0;
    pk.ask = 0;
    ui.d_lcd = 1;
    pk_unmark((uint64_t)1 << k);
    if (pk.hk[h] == k + 1u) {            /* (its holder: the store's bytes and its entries again) */
        if (!h) { if (!pk_line()) dr_load(k); }
        else {
#ifdef HYB_MULTI
            const uint8_t mi = sq_mi;
            sq_mi = (uint8_t)(h - 1u);
            sq_load(k - DR_PATS);
            sq_mi = mi;
#else
            sq_load(k - DR_PATS);
#endif
        }
    }
    pk_name(t, k);
    app_say(t, "DISCARDED");
    if (n) pk_set(h, n - 1u);
    return 1;
}
static UI_LATE int pk_lcd(char *a, char *v)      /* the strip: the question (1 = up) */
{
    if (!pk.ask) return 0;
    pk_name(a, pk.ask - 1u);
    text_cat(a, " UNSAVED");
    text_cat(v, "REC SAVE  HOME DROP");
    return 1;
}
static UI_LATE int sp_lcd(char *a, char *v)      /* the strip: the question (1 = up) */
{
    if (!sp_on()) return 0;
    text_cat(a, "SAVE ");
    sp_name(a + 5, 0);
    text_cat(a, "?");
    text_cat(v, "REC");
    return 1;
}
static UI_LATE void sp_idle(void)                /* app_frame: the question over (its time out, the page left, a button
                                          * but REC pressed: input did that button's own); the patterns asked saved
                                          * while the beat ran saved at its stop (what they are then), the kit and
                                          * GLO's settings too (sqv.sv_w) */
{
    if (ui.rp_ask && !rp_on()) {         /* (0.96: RESET PATCH's question gone with the list: its strip too) */
        ui.rp_ask = 0;
        ui.msg_until = 0;
        ui.d_lcd = 1;
    }
    sqv.sp_b &= ui.btn;                  /* (let go: pressed again, new) */
    if (sqv.sp_ask && (!sp_on() || (ui.btn & ~sqv.sp_b & ~(1u << B_REC)))) { sqv.sp_ask = 0; ui.d_lcd = 1; }
    if (sqv.sp_wait && !sp_runs()) {
        const uint32_t w = sqv.sp_wait == 0xFFu ? 0u : sqv.sp_wait;
        sqv.sp_wait = 0;
        if (sp_set(w)) sp_go(w);         /* (all undone meanwhile: nothing) */
    }
    if (sqv.sv_w && !sp_runs() && !sv_on()) {   /* (1.02: once the slot list is closed) */
        if (sqv.sv_w & 4u) {             /* (1.02: SAVE's slot asked while the beat ran; the patch changed since: not
                                          * written) */
            ui.scur = sqv.svpw[0];
            ui.nask = 1;                 /* (asked then) */
            if (set.patch == sqv.svpw[1]) sv_write(); else app_say("Save", "CANCELLED");
        }
        if (sqv.sv_w & 1u) kit_save();
        if ((sqv.sv_w & 2u) && ui.glo_t) {
            ui.glo_t = 0;
            if (glo_save() != 0) app_say("Save", "FAILED");
        }
        sqv.sv_w = 0;
    }
}
static UI_LATE int sv_lit(void)                  /* SAVE's light (AUTO SAVE OFF): what SAVE saves on this page waits
                                          * (a sequencer's: the patterns with edits, the journal; the kit's: the kit;
                                          * GLO: the settings; elsewhere: all of them, as SAVE's slot list saves them
                                          * along) */
{
    if (ui.view == V_SEQ || ui.view == V_DRUM) return JR_ANY;
    if (ui.view == V_KIT) return kv.dirty_t != 0;
    if (ui.view == V_GLO) return ui.glo_t != 0 && (plat_ms() & 512u) != 0;   /* (unsaved settings: SAVE blinks, ~1 Hz, 50 %,
                                                     * unlike the steady light of patch / pattern edits) */
    return sv_unsaved();
}
static UI_LATE void sv_page(void)                /* SAVE on a sequencer's page, the kit's or in GLO: that page's alone
                                          * (0.67; 0.65-0.66: all that waited, "Saved ALL"); the beat running: at its
                                          * stop (after 1.00, sp_idle) */
{
    if (ui.view == V_GLO) pv_pend();
    if (ui.view == V_SEQ || ui.view == V_DRUM) sp_button();
    else if (ui.view == V_KIT) {
        const int w = kv.dirty_t != 0;
        if (w && sp_runs()) { sqv.sv_w |= 1u; app_say("Drum kit", "SAVE PENDING"); return; }
        ui.msg[1][0] = 0;
        kit_save();
        if (!ui.msg[1][0]) app_say("Drum kit", w ? "SAVED" : "UNCHANGED");
    } else if (!ui.glo_t)
        app_say("Settings", "UNCHANGED");
    else if (sp_runs()) { sqv.sv_w |= 2u; app_say("Settings", "SAVE PENDING"); }
    else {
        ui.glo_t = 0;
        app_say("Settings", glo_save() != 0 ? "SAVE FAILED" : "SAVED");
    }
}
static UI_LATE void kit_idle(uint32_t now)       /* app_frame: the kit saved once it rests; RESET's question over */
{
    if (kv.dirty_t && !store_wait() && (int32_t)(now - kv.dirty_t) >= (int32_t)DR_SAVE_MS) kit_save();   /* (idle:
                                          * store_wait) */
    if (kv.ask && now - kv.ask_t >= SQ_CLR_MS) kv.ask = 0;
}
static UI_LATE void kit_open(void)       /* GLO's DRUM KIT line: the page */
{
    pl_n = 0;                            /* (the parts' signatures share PLAY mode's room: its list built again) */
    kv.ask = kv.tt = kv.dseq = 0;
    kv.lcd = ~0u;
    ui.view = V_KIT;
    view_reset();
}
static UI_LATE void kit_dseq(void)       /* the drums' EDIT > DRUM KIT: the page, HOME back to the drums' (0.74) */
{
    kit_open();
    kv.dseq = 1;
}
static UI_LATE void kit_seq(void)        /* SEQ on the page: the drums' page (0.74), SEQ's way on from there */
{
    kv.ask = kv.dseq = 0;
    dr_page();
}
static UI_LATE void kit_up(void)         /* HOME: back to the drums' page if opened there (kit_seq), else GLO, on the
                                          * DRUM KIT line */
{
    if (kv.dseq) {
        kit_seq();
        return;
    }
    kv.ask = 0;
    ui.view = V_GLO;
    ui.gpage = GP_TOP;
    ui.gpos = (uint8_t)glo_line(GP_TOP, G_KIT);
    ui.gch = G_NONE;
    view_reset();
}
static UI_LATE void kit_key(uint32_t k)  /* a key down on the page: a black key chooses its instrument; either plays
                                          * the one chosen (the accent from the velocity) */
{
    uint32_t i;
    const uint32_t v = key_vel_drum(k);
    mute_hint(4u);
    for (i = 0; i < DR_NI && DR_KEY[i] != k; i++)
        ;
    if (i < DR_NI && i != sqv.ins) {
        sqv.ins = (uint8_t)i;
        kv.ask = 0;
    }
    dr.prev = (uint8_t)((sqv.ins + 1u) | (v >= 110u ? 2u : v >= 70u ? 1u : 0u) << 4);
}
static UI_LATE void kit_reset_ask(void)  /* BUS's K4 turned (0.88): "RESET BD  REC" (REC: kit_rec) */
{
    char t[12];
    t[0] = 0;
    text_cat(t, "RESET ");
    text_cat(t, DR_NAME[sqv.ins]);
    kv.ask = 1;
    kv.ask_t = plat_ms();
    app_say(t, "REC");
}
static UI_LATE void kit_send(dr_kp_t *q, int32_t e)   /* VOICE's K4 (0.88): SEND, heard at once (dr_sends; after
                                          * 1.00 the instrument no longer struck by the turn); part A's patch with
                                          * its delay / reverb off (mode 0): the send does nothing, the strip says so */
{
    q->snd = (uint8_t)clamp((int32_t)q->snd + knob_steps(3u, e), 0, 127);
    dr_sends();
#ifdef HYB_MULTI
    if (!S.patch[112]) app_say("FX OFF IN PATCH A", "");
#else
    if (!S.patch[112]) app_say("FX OFF IN PATCH", "");
#endif
}
static UI_LATE void kit_knob(uint32_t c, int32_t e)   /* SELECT (c 4): the block; K1..K4 (c 0..3): its four */
{
    dr_kp_t *q = &drk.k[sqv.ins];
    if (c == 4u) {
        kv.blk = (uint8_t)clamp((int32_t)kv.blk + e, KB_SOUND, KB_N - 1);
        if (kv.blk < kv.top) kv.top = kv.blk;   /* (the rows follow: the block chosen on the page) */
        else if (kv.blk > kv.top + 1u) kv.top = (uint8_t)(kv.blk - 1u);
        kv.ask = kv.tt = 0;
        return;
    }
    if (kv.blk == KB_BUS) {              /* (the drums' bus: LEVEL with GLO's settings, DRIVE and COMP with the kit) */
        if (c == 0u) {
            set.dr_lev = (uint8_t)clamp((int32_t)set.dr_lev + knob_steps(c, e), 0, DR_LMAX);
            glo_touch();
        } else if (c == 1u)
            drk.drv = (uint8_t)((drk.drv & 0xF0u) | (uint32_t)clamp((int32_t)(drk.drv & 15u) + e, 0, DV_N - 1));
        else if (c == 2u)
            drk.drv = (uint8_t)((drk.drv & 15u) | (uint32_t)clamp((int32_t)(drk.drv >> 4) + e, 0, DC_N - 1) << 4);
        else {                           /* (K4: the instrument's RESET, 0.88; up to 0.87 VOICE's K4) */
            kit_reset_ask();
            return;
        }
        if (c) kv.dirty_t = plat_ms() | 1u;
        dr_level();                      /* (heard at once) */
        kv.tt = (uint8_t)(1u + c);
        kv.tt_t = plat_ms() | 1u;
        return;
    }
    if (kv.blk == KB_SOUND) {
        if (c == 0u) q->lev = (uint8_t)clamp((int32_t)q->lev + knob_steps(c, e), 0, DR_KLMAX);
        else if (c == 1u) q->tune = (uint8_t)clamp((int32_t)q->tune + e, 0, DR_KN);   /* (TUNE: 0.85-1.00 struck
                                          * the instrument at each turn; after 1.00 no kit knob strikes it) */
        else if (c == 2u) q->dec = (uint8_t)clamp((int32_t)q->dec + e, 0, DR_KN);
        else q->att = (uint8_t)clamp((int32_t)q->att + e, 0, DR_KATT);
    } else if (c == 0u)
        q->tone = (uint8_t)clamp((int32_t)q->tone + e, 0, DR_KN);
    else if (c == 1u) {                  /* (MODEL: right 909, left 808; the five that have two; the hats with their
                                          * sample (0.98): right SMP, left SYN) */
        if (!KIT_MODEL(sqv.ins)) return;
        q->m909 = (uint8_t)(e > 0);
    } else if (c == 2u)
        q->pan = (int8_t)clamp((int32_t)q->pan + knob_steps(c, e), -DR_KPAN, DR_KPAN);
    else kit_send(q, e);
    kv.ask = 0;
    kv.dirty_t = plat_ms() | 1u;
}
static UI_LATE int kit_rec(void)         /* REC on the page: RESET asked, the instrument as shipped (1 = taken) */
{
    if (ui.view != V_KIT || !kv.ask) return 0;
    kv.ask = 0;
    drk.k[sqv.ins] = kit_p0(sqv.ins);
    dr_sends();                          /* (its SEND 0) */
    kv.dirty_t = plat_ms() | 1u;
    app_say(DR_NAME[sqv.ins], "RESET");
    return 1;
}
static UI_LATE int kit_tt_on(void) { return kv.tt && (int32_t)(plat_ms() - kv.tt_t) < (int32_t)SQ_TT_MS; }   /* (signed:
                                          * tt_t is now | 1, 1.18C46) */
static UI_LATE void kit_bus_text(uint32_t c, char *v)   /* a BUS knob's value (c 0..2: LEVEL, DRIVE, COMP) */
{
    if (c == 0u) put_sint(v, set.dr_lev, 0);
    else if (c == 1u) text_cat(v, DV_NAME[(drk.drv & 15u) < DV_N ? drk.drv & 15u : 0u]);
    else text_cat(v, DC_NAME[(drk.drv >> 4) < DC_N ? drk.drv >> 4 : 0u]);
}
static UI_LATE void kit_lcd(char *a, char *v)    /* the strip: a BUS knob turned, its name and value; else DRUM KIT,
                                          * the instrument and its model */
{
    static const char *const TT[3] = {"DRUM LEVEL", DR_KDRIVE ? "KICK DRIVE" : "DRUM DRIVE", "DRUM COMP"};   /* (1.18C42:
                                          * LEVEL above 100 drives the kick alone, DR_KDRIVE) */
    if (kit_tt_on()) {
        text_cat(a, TT[kv.tt - 1u]);
        kit_bus_text(kv.tt - 1u, v);
        return;
    }
    text_cat(a, "DRUM KIT");
    text_cat(v, DR_NAME[sqv.ins]);
    if (sqv.ins <= DR_HT) text_cat(v, drk.k[sqv.ins].m909 ? " 909" : " 808");
    else if (KIT_MODEL(sqv.ins)) text_cat(v, drk.k[sqv.ins].m909 ? " SMP" : " SYN");
}
static UI_LATE void kit_frame(void)      /* the page, each frame: the strip when what it shows changes (the
                                          * instrument, its model; a BUS knob's value) */
{
    const uint32_t l = kit_tt_on() ? 1u << 31 | (uint32_t)kv.tt << 16 | (uint32_t)drk.drv << 8 | set.dr_lev
                                   : sqv.ins | (uint32_t)drk.k[sqv.ins].m909 << 4;
    if (l != kv.lcd) { kv.lcd = l; ui.d_lcd = 1; }
}
static UI_LATE uint32_t kit_lights(uint32_t *mid)   /* the keys on the page: the instrument's black key lit, the
                                          * other black keys at the middle level (plat_lights_mid) */
{
    uint32_t i, m = 0;
    for (i = 0; i < DR_NI; i++) m |= 1u << DR_KEY[i];
    *mid = m & ~(1u << DR_KEY[sqv.ins]);
    return 1u << DR_KEY[sqv.ins];
}

/* SHIFT + the top C# (MONO) or D# (POLY): the patch's key mode (94: 0 poly, 1-4 mono), an undo step */
#define KEY_MONO 20u                     /* C#5 */
#define KEY_POLY 22u                     /* D#5 */
#ifdef HYB_MULTI
#define KEY_EDA 17u                      /* A#4: SHIFT + it, part A edited */
#define KEY_EDB 25u                      /* F#5: part B */
#endif
static void key_mode(int mono)
{
    static uint8_t mono_last = 1u;       /* the mono mode a patch had (or plain mono) */
    uint32_t v;
    if (shadow[94]) mono_last = shadow[94] > 4u ? 1u : shadow[94];
    v = mono ? mono_last : 0u;
    if (v != shadow[94]) {
        all_notes_off();
        undo_push(set.patch, 94u, shadow[94]);
        undo_t = 0;
        shadow[94] = (uint8_t)v;
        em_set(set.patch, 94u, v);
        ev_put(EV_PARAM, 94u, v);
        ui.d_rows = 1;
    }
    app_say("Key mode", mono ? "MONO" : "POLY");
}

static void octave(int32_t d)
{
    int32_t o = clamp(set.oct + d, -2, 2);
    char t[4] = {0};
    if (o != set.oct) {
        set.oct = (int8_t)o;
        t[0] = o < 0 ? '-' : '+';
        t[1] = (char)('0' + (o < 0 ? -o : o));
        app_say("Octave", t);
    }
}

/* the quick PANIC (OCT- and OCT+ together, held OC_HOLD_MS, 8): everything stops, in any view.
 * The keys' and HOLD's notes off (HOLD off), both sequencers off (their input notes forgotten: CC 123 at the audio
 * interrupt, sq_panic), the engine's voices released and its arpeggiator's keys cleared (CC 120, the engine's
 * sy_all_off), the effects' tails cut and the drums' voices silenced (the app's CC 120 at the audio interrupt: fx_cut);
 * out over MIDI all notes off and all sound off on OUT CHANNEL to OUT PORTS, a STOP with CLOCK MASTER */
static UI_LATE void quick_panic(void)
{
    sq_stop_all();                       /* (its notes off at the next block, a STOP with CLOCK MASTER: sq_block) */
    ln.emp = 0;
    sq_pv_cut();                         /* (a step's preview) */
    pvr.k = 0;                           /* (one waiting) */
    dr.run = 0;
    hold_on = 0;
#ifdef HYB_MULTI
    hold_o = 0;                          /* (the other part's too) */
#endif
    all_off_both();                      /* (the keys' and HOLD's, both parts' (1.15): after the sequencer's stop,
                                          * they are notes again) */
    ev_put(0xB0u, 123u, 0u);
    ev_put(0xB0u, 120u, 0u);
    midi_send_ch(0xB0u, 123u, 0);
    midi_send_ch(0xB0u, 120u, 0);
#ifdef HYB_MULTI
    midi_send_p(1u, 0xB0u, 123u, 0);     /* (1.19.4: part B's OUT too) */
    midi_send_p(1u, 0xB0u, 120u, 0);
#endif
    if (set.clk == CLK_MASTER && midi_to() && !ck.mrun)   /* (no sequencer ran: STOP here; else sq_block's) */
        plat_midi_send(0x0Fu | 0xFCu << 8, midi_to());
    app_say("PANIC", "ALL STOPPED");
}

static UI_LATE void hold_flip(uint32_t keys)     /* HOLD on and off */
{
    hold_on = (uint8_t)!hold_on;
#ifdef HYB_MULTI
    if (set.mx_mode == SY_M_LAYER) hold_o = hold_on;   /* (1.16: LAYER, one HOLD for both parts) */
#endif
    if (!hold_on) {
        ui.keys = keys;                  /* (the keys as now: the ones still down keep their notes) */
#ifdef HYB_MULTI
        hold_release_m(set.mx_mode == SY_M_SPLIT ? 1u << ed_part : 3u);   /* (1.16: SPLIT, the part edited's zone's
                                          * alone; the other part's HOLD keeps its own) */
#else
        hold_release();
#endif
    }
    say_on("Hold", hold_on);
}
static UI_LATE void hold_toggle(uint32_t keys)   /* REC with no other meaning: HOLD, on HOME and in PLAY mode alone
                                          * (after 1.09: in the menus REC pressed on the way, one press too many after
                                          * EDIT's list, switched HOLD on unnoticed; there it only says where HOLD is;
                                          * HOLD itself stays as it is, REC lit while it is on) */
{
    if (ui.view == V_QUICK || ui.view == V_PLAY) hold_flip(keys);
    else app_say("Hold", "HOME ONLY");
}
static UI_LATE uint32_t pc_ticks(void)   /* (1.18C40) PLAY now (from a stop): PRECOUNT's ticks, 0 none: RECORD armed
                                          * on the page shown, its sequencer to start (a pattern; SEQ B in the multi
                                          * modes), the clock stopped (the launcher's silent run: none) */
{
    uint32_t d;
    if (!SQ_PCB || ck.on) return 0;
    if (ui.view == V_SEQ && sqv.arec && SQ_PATSH >= 0) {
        d = sqp.div;
    } else if (ui.view == V_DRUM && sqv.rec && DR_PATSH >= 0) d = drp.div;
    else return 0;
    return SQ_PCB << (7u + CK_SH(d));    /* (the bars' ticks: 16 steps of 8 ticks a bar, the clock's at the division) */
}
static UI_LATE void seq_play(int go)     /* PLAY (or MIDI START / STOP with CLOCK EXTERNAL): those with a pattern start
                                          * together on the shared clock, or both stop. go 2 (1.18C40): MIDI START, no
                                          * PRECOUNT (the transport follows the clock's master) */
{
    pc_cut = 0;
    pc_tk = 0;
    if (go) {
        const uint32_t pc = go == 1 ? pc_ticks() : 0u;
        pc_tk = (uint16_t)pc;            /* (before the sequencers: the audio interrupt's next start takes it, whenever
                                          * it comes) */
        sq_pvm = 0;                      /* (a step held and heard: the start plays every sequencer, its preview cut,
                                          * 0.94) */
        sq_qr[0] = sq_qr[1] = sq_qr[2] = SQ_QR_OFF;   /* (0.96: nothing waits at a start: all from the clock's) */
#ifdef HYB_MULTI
        if (!(set.sq_pad & SQ_DISARM)) sq_on_i(0, 1);
        if (!(set.sqb_pad & SQ_DISARM)) sq_on_i(1, 1);   /* (SEQ B: in the multi modes) */
#else
        if (!(set.sq_pad & SQ_DISARM)) sq_start();
#endif
        if (!(set.sq_pad & DR_DISARM)) dr.run = 1;
        dr_mute = 0;                     /* (RECORD's step from a run before: the clock counts from 0 again) */
        sq_mute[0] = sq_mute[1] = 0;
        if (!SQ_RUNS && !dr.run) pc_tk = 0;   /* (nothing started: none for a later start) */
        else if (pc) return;             /* (the count: the page's strip shows it at once, "P1 COUNT 1.1", seq_state) */
        app_say("Seq", SQ_RUNS || dr.run ? "PLAY" : "NO PATTERN");
    } else {
        sq_stop_all();
        dr.run = 0;
        ln.emp = 0;
        app_say("Seq", "STOP");
    }
}
static UI_LATE void seq_none(int drums, int none)   /* PATTERN NONE (below 1) on or off: PLAY leaves the sequencer
                                          * out; chosen while it runs, it stops (a pattern again: it joins) */
{
    const uint32_t bit = drums ? DR_DISARM : SQ_DISARM, run = SQ_RUNS || dr.run;
#ifdef HYB_MULTI
    if (!drums) SQ_PADV = (uint8_t)(none ? SQ_PADV | bit : SQ_PADV & ~bit);   /* (the arp sequencer shown's) */
    else
#endif
    set.sq_pad = (uint8_t)(none ? set.sq_pad | bit : set.sq_pad & ~bit);
    glo_touch();                         /* (saved with GLO's settings) */
    if (drums) dr.run = (uint8_t)(!none && run);
    else if (!none && run) sq_start();
    else if (none) sq_stop();
}
#ifdef HYB_MULTI
static UI_LATE void pv_turn(int32_t e)   /* SHIFT + ALGO: the part edited's volume, up in the strip ("VOL A 100") */
{
    char t[8], v[8];
    pv_vol[ed_part] = (uint8_t)clamp((int32_t)pv_vol[ed_part] + knob_steps(4u, e), 0, 127);
    ui.shift_used = 1;                   /* (OCT- let go: no octave) */
    glo_touch();                         /* (1.19.7: GLO's settings unsaved, as a GLO value turned) */
    t[0] = v[0] = 0;
    text_cat(t, ed_part ? "VOL B" : "VOL A");
    put_sint(v, pv_vol[ed_part], 0);
    app_say(t, v);
}
#endif
static UI_LATE int home_undo(uint32_t shift, uint32_t btn)   /* HOME pressed: SHIFT + HOME, the undo (1: done). A
                                          * sequencer's page: its pattern's alone, never the patch's ("Undo NOTHING"),
                                          * and OCT- down at all is SHIFT there (0.70: OCT- pressed with HOME sooner
                                          * than SHIFT_HOLD_MS left the page, and SHIFT + HOME again undid the patch) */
{
    const int pg = ui.view == V_SEQ || ui.view == V_DRUM;
    if (rp_home()) return 1;             /* (0.96: RESET PATCH asked, HOME takes it back) */
    if (ui.view == V_SEQ && SQ_CE) {     /* (1.03: CHORD ENTRY left, the step as it was) */
        sq_ce_end();
        return 1;
    }
#ifdef HYB_MULTI
    if (!shift && ln_back()) return 1;   /* (1.18C10: a sequencer's page opened from the launcher: back there) */
#endif
    if (!shift && !(pg && (btn >> B_OCTDN & 1u) && !ui.oc_pair)) return 0;
    ui.shift_used = 1;                   /* (OCT- let go: no block moved) */
    if (pg) un_undo(); else undo_step();
    return 1;
}
#ifdef HYB_MULTI
/* (1.18C10) the pattern launcher (SHIFT + SEQ): a table of LN_ROWS rows (set.ln), each SYNTH 1's
 * (SEQ A, part A), SYNTH 2's (SEQ B, part B) and the drums' pattern and a mode (NO, REP, 1x .. 8x). PLAY starts the
 * row selected (stopped: all its patterns together on the shared clock's start); PLAY on the row playing stops; another
 * row while it runs waits for the bar's line (its patterns asked for through the sequencers' own change, sq_qwant: the
 * audio interrupt switches them on the line, the drums sample-exact from the spare). A row lasts its longest pattern
 * (the shorter ones loop meanwhile); REP loops it, Nx plays it N times and goes on to the next row not NO (wrapping;
 * an empty row is a silent row by its mode; 1.18C18: a lone row stops after its Nx), NO (every row's default, REP at its first pattern) is skipped. The main loop asks for the next row's patterns in the last
 * quarter bar before the line (LN_ASK0 .. LN_ASK1 ticks before it: every pattern's line period, SQ_QP8, is at least
 * 128 ticks, so the line they wait for is that one; a main loop later than that: the change at the line after, never
 * a step dropped). The lowest three keys (on the launcher alone) are the parts' keys: a tap MUTE (half lit) and
 * back, held with K1..K3 the part's volume (VOL A / B, the drums' LEVEL); held with EDIT the part's sequencer page
 * with the row's pattern (HOME back here). The table is written by SAVE alone (ln_save, glo_save) */
#define LN_BAR 512u                      /* a bar in the shared clock's ticks (4 << div steps of 8 << CK_SH(div)) */
#define LN_ASK0 112u                     /* the ticks before the line from which the next row's patterns are asked */
#define LN_ASK1 48u                      /* ... until (a queued row nearer to its bar's line than this: the next bar) */
static const uint8_t LN_NX[LN_NMODE] = {0u, 0u, 1u, 2u, 4u, 8u};   /* a mode's repetitions (NO, REP: 0) */
static UI_LATE int32_t ln_cur(uint32_t q)   /* part q's pattern now, or the one it waits for (-1 NONE) */
{
    if (q == 2u) return SQ_QPAT(2u, set.sq_pad & DR_DISARM ? -1 : (int32_t)set.dr_pat);
    return SQ_QPAT(q, (q ? set.sqb_pad : set.sq_pad) & SQ_DISARM ? -1 : (int32_t)(q ? set.sqb_pat : set.sq_pat));
}
static UI_LATE int ln_has(uint32_t r) { return (set.ln[r][0] | set.ln[r][1] | set.ln[r][2]) != 0u; }
#define ln_act() (ln.play ? ln.play : ln.prow ? ln.prow : 1u)   /* 1 + the active row (1.18C21: none yet, row 1): its cells are
                                          * what the sequencers play, both ways (ln_cell_step, ln_sync) */
#define LN_RUN (SQ_RUNS || dr.run || ln.emp)   /* the transport runs (1.18C16: the launcher's silent run too) */
static UI_LATE void ln_part(uint32_t q, int32_t p1)   /* part q's pattern p1 (-1 NONE), PATTERN's way (sq_knob,
                                          * dr_knob): running, at its line; else at once */
{
    int32_t p0;
    if (p1 == ln_cur(q)) return;
    if (q == 2u) {
        p0 = set.sq_pad & DR_DISARM ? -1 : (int32_t)set.dr_pat;
        if (sq_qwant(2u, p0, p1)) return;
        if (p1 >= 0 && (uint32_t)p1 != set.dr_pat && !pk_gate(0u, (uint32_t)p1)) return;   /* (1.18C33: asked) */
        if (p1 != p0) seq_none(1, p1 < 0);
        if (p1 >= 0 && (uint32_t)p1 != set.dr_pat) { set.dr_pat = (uint8_t)p1; dr_load((uint32_t)p1); glo_touch(); }
        return;
    }
    {
        const uint8_t mi = sq_mi;
        sq_mi = (uint8_t)q;
        p0 = SQ_PADV & SQ_DISARM ? -1 : (int32_t)SQ_PATV;
        if (!sq_qwant(q, p0, p1) && (p1 < 0 || (uint32_t)p1 == SQ_PATV || pk_gate(1u + q, (uint32_t)p1))) {   /* (1.18C33:
                                          * its edits fit nowhere: asked) */
            sq_pv_cut();
            pvr.k = 0;
            if (p1 != p0) seq_none(0, p1 < 0);
            if (p1 >= 0) { SQ_PATV = (uint8_t)p1; sq_load((uint32_t)p1); glo_touch(); }
        }
        sq_mi = mi;
    }
}
static UI_LATE void ln_ask(uint32_t r)   /* row r's patterns asked for, the parts not asked yet (ln.iss) */
{
    uint32_t q;
    for (q = 0; q < 3u; q++) {
        const int32_t p = (int32_t)set.ln[r][q] - 1;
        if (ln.iss >> q & 1u) continue;
        ln_part(q, p);
        if (p >= 0 && ln_cur(q) == p && !sq_qrun(q) && LN_RUN) {   /* (1.18C18: the part stopped in a row before that left it NONE or was
                                          * silent: running again, joining silent to the row's line as the cells' knobs do) */
            if (q == 2u) dr.run = 1; else sq_on_i(q, 1);
        }
        if (ln_cur(q) == p) ln.iss |= (uint8_t)(1u << q);   /* (refused for a block, sq_qwant: again next frame) */
    }
}
static UI_LATE uint32_t ln_after(uint32_t r)   /* the row after r that plays: 1 + it (r itself, the only one), 0 none */
{
    uint32_t i;
    for (i = 1; i <= LN_ROWS; i++) {
        const uint32_t c = (r + i) % LN_ROWS;
        if (set.ln[c][3] != LN_NO) return c + 1u;   /* (an empty row plays silent by its mode: REP for ever, Nx N bars; only NO is skipped) */
    }
    return 0;
}
static UI_LATE int ln_solo(uint32_t r)   /* 1.18C18: no other row plays (only NO rows are none): r alone ends at its Nx */
{
    uint32_t c;
    for (c = 0; c < LN_ROWS; c++)
        if (c != r && set.ln[c][3] != LN_NO) return 0;   /* (as ln_after: an empty row is a silent rest, a row) */
    return 1;
}
static UI_LATE void ln_fix(void)         /* 1.18C18: the row playing since its line: every part on the row's cell, whatever was asked
                                          * before (an edit of the waiting row after its ask, a block refused at the line): till they are */
{
    uint32_t q, ok = 1;
    for (q = 0; q < 3u; q++) {
        const int32_t p = (int32_t)set.ln[ln.play - 1u][q] - 1;
        if (ln_cur(q) != p) { ln_part(q, p); if (ln_cur(q) != p) ok = 0; }
    }
    ln.fix = ok ? 0u : (uint8_t)(ln.fix - 1u);   /* (a countdown: a hand on PATTERN meanwhile is not fought for long) */
}
static UI_LATE uint32_t ln_plen(uint32_t q, int32_t p)   /* part q's pattern p (-1 none) in ticks: its LENGTH and time division
                                          * (the pattern loaded, else its head in the store; the default one as sq_load_to /
                                          * dr_load_to take it), 0 none */
{
    uint32_t dv = SQ_DIV1, lc = 0;
    uint8_t t[4];
    if (p < 0) return 0;
    if (q == 2u) {
        if ((uint32_t)p == set.dr_pat && !sq_qr[2]) { dv = drp.div; lc = drp.lc; }
        else if (plat_store_read(OBJ_DRUM, (uint32_t)p * DR_REC + DR_NI * 2u + DR_NI * SQ_N, t, 2u) == 2 && t[0] == DR_MARK
                 && (t[1] & 15u) < SQ_NDIV && (t[1] >> 4) < SQ_NLC) { dv = t[1] & 15u; lc = t[1] >> 4; }
    } else {
        if ((uint32_t)p == (q ? set.sqb_pat : set.sq_pat) && !sq_qr[q]) { dv = sqpa[q].div; lc = sqpa[q].lc; }
        else if (plat_store_read(OBJ_SEQ, (uint32_t)p * SQ_REC + SQ_N * 4u, t, 4u) == 4 && t[0] == SQ_MARK && t[1] < SQ_NDIV
                 && t[2] < SQ_NLC) { dv = t[1]; lc = t[2]; }
    }
    return SQ_STEPS(lc) * 8u << CK_SH(dv);
}
static UI_LATE uint32_t ln_len(uint32_t r)   /* row r's length (ticks): its longest assigned pattern in its own LENGTH and time
                                          * division (the shorter ones loop meanwhile; SYNTH 2 left out with one synth), a bar
                                          * none. From the table, not from what runs: at the row's first frame the parts
                                          * may not be running yet (PLAY) or still be on the row before (1.18C15) */
{
    uint32_t q, l = 0, t;
    for (q = 0; q < 3u; q++) {
        t = ln_plen(q, (int32_t)set.ln[r][q] - 1);
        if (t > l) l = t;
    }
    return l ? l : LN_BAR;
}
static UI_LATE void ln_queue(uint32_t r) /* row r at the running bar's line (nearer to it than LN_ASK1: the next) */
{
    const uint32_t t = ck.mt, d = t - ln.r0 < 0x80000000u ? t - ln.r0 : 0u;
    ln.next = (uint8_t)(r + 1u);
    ln.qd = 1;
    ln.iss = 0;
    ln.sw = ln.r0 + ((d + LN_ASK1) / LN_BAR + 1u) * LN_BAR;
    ln.lcd = 0xFFu;
}
static UI_LATE void ln_play(void)        /* PLAY on the launcher */
{
    ln.pst = 0;
    ln.fix = 0;
    const uint32_t r = ln.sel;
    ln.prow = (uint8_t)(r + 1u);          /* (the active row: edits there are heard) */
    if (LN_RUN) {
        if (ln.play == r + 1u) {         /* (the row playing: all stop) */
            seq_play(0);
            ln.play = ln.next = 0;
        } else {
            if (!ln.play) ln.r0 = 0;     /* (the transport started elsewhere: its bars from the clock's start) */
            ln.emp = 1;
            ln_queue(r);
            app_say("Row", "NEXT BAR");
        }
        ln.lcd = 0xFFu;
        return;
    }
    ln.iss = 0;
    ln_ask(r);                           /* (stopped: at once) */
    if (!ck.on) ck.mt = 0;               /* (the clock starts from 0 at its next block: the row's start) */
    seq_play(1);
    ln.emp = 1;                          /* (an empty row: the clock runs, silent) */
    if (!ln_has(r)) app_say("Launcher", "PLAY");
    ln.play = (uint8_t)(r + 1u);
    ln.next = 0;
    ln.r0 = 0;
    ln.len = 0;
    ln.lcd = 0xFFu;
}
static UI_LATE void ln_esel(uint32_t q)  /* 1.18C21: part key q held: that part edited (SYNTH 1 part A, SYNTH 2 part B, as SHIFT +
                                          * A#4 / F#5; the drums: their sequencer), SEQ opening its sequencer (sqv.last) */
{
    ln.esel = 1;
    if (q == 2u) {
        ln.epd = 1;
        sqv.last = 2;
        app_say("Edit", "DRUMS");
        return;
    }
    ln.epd = 0;
    sqv.last = 0;                        /* (SEQ's way from its start: the part's arp sequencer, sq_button) */
    sqv.first = (uint8_t)q;
    if (q && SQ_BMO) app_say("Seq B", (set.mo & MO_B) > 1u ? "MIDI ONLY" : "B OUT OFF");   /* (1.19.3: SINGLE, part B not edited) */
    else part_edit(q);
}
#define LN_HOLD_MS 300u                  /* (1.18C21) a part key down this long is held, not tapped: no MUTE */
static UI_LATE void ln_keys(void)        /* the part keys: down, up (a tap: MUTE and back; 1.18C21: a tap only, released before
                                          * LN_HOLD_MS with no knob turned) */
{
    const uint32_t now = plat_ms();
    const uint32_t kk = ui.view == V_LNCH ? ui.keys : 0u,   /* (the three lowest white keys: 0, 2, 4) */
                   k = ((kk & 1u) | (kk >> 1 & 2u) | (kk >> 2 & 4u)), dn = k & ~(uint32_t)ln.keys, up = ln.keys & ~k;
    uint32_t q;
    ln.kused &= (uint8_t)~dn;
    ln.khold &= (uint8_t)~dn;
    for (q = 0; q < 3u; q++) {
        if (dn >> q & 1u) ln.kt[q] = now;
        if ((k >> q & 1u) && !(ln.khold >> q & 1u) && now - ln.kt[q] >= LN_HOLD_MS) {
            ln.khold |= (uint8_t)(1u << q);
            if (ln.play) ln.sel = (uint8_t)(ln.play - 1u);   /* (held, not tapped: the display jumps to the row playing; a tap MUTE leaves the display alone) */
            ln_esel(q);                  /* (held: that part edited) */
        }
        if ((up >> q & 1u) && !(ln.kused >> q & 1u) && !(ln.khold >> q & 1u)) {
            ln_mute ^= (uint8_t)(1u << q);
            if (q == 2u) dr_level();     /* (the knobs' strip: drawn by its signature, ln_draw) */
            else if (LN_MMUTE(q)) sq_cut[q] = 1u;   /* (1.19.3: a row without a sound of its own muted: its notes
                                          * sounding off over MIDI at the next block, sq_block) */
        }
    }
    ln.keys = (uint8_t)k;
}
static UI_LATE void ln_sync(void)        /* 1.18C21: a part's pattern changed elsewhere (its sequencer's page, PATTERN): the
                                          * active row's cell follows it. Not while the next row's patterns are asked for
                                          * or the row just switched to is set (ln_fix): then the change is the launcher's.
                                          * 1.18C48: also a part that starts running (bit 7 of ln.sc), whatever started
                                          * it (PLAY on HOME or the DRUM KIT page, MIDI START, a sequencer joining): the
                                          * cell shows what plays, not only a pattern changed since */
{
    const uint32_t a = ln_act() - 1u, hold = ln.fix || (ln.next && ln.next != ln.play && ln.iss);
    uint32_t q;
    for (q = 0; q < 3u; q++) {
        const uint8_t c = (uint8_t)(ln_cur(q) + 1), s = (uint8_t)(c | (sq_qrun(q) ? 0x80u : 0u)), o = ln.sc[q];
        if (s == o && ln.sc0) continue;
        ln.sc[q] = s;
        if (c == (o & 0x7Fu) && !(s & ~o & 0x80u) && ln.sc0) continue;   /* (stopped, nothing new: the cell stays) */
        if (!ln.sc0 || hold || set.ln[a][q] == c) continue;
        if (q < 2u && c && set.ln[a][!q] == c) continue;   /* (SYNTH 1's and 2's never the same in a row) */
        set.ln[a][q] = c;
    }
    ln.sc0 = 1;
}
#ifdef APP_HOST_CMD
static void host_ln_tap(void);           /* (hostcmd.c: the row changes and the parts' origins traced, "lntr") */
#endif
#define ln_rec() (sqv.arec || sqv.rec)   /* (1.18C30) RECORD on in SEQ A / B or the drums' page */
static UI_LATE void ln_idle(void)        /* app_frame, after input: the row playing, the next one's patterns asked for
                                          * before its line, the switch at it */
{
    const uint32_t run = LN_RUN, t = ck.mt;
    ln_keys();
    ls_idle();                           /* (1.18C27: the sets: part B's patch of a LOAD, a write at the stop) */
    if (ln.ret && ui.view != V_SEQ && ui.view != V_DRUM && ui.view != V_LNCH) ln.ret = 0;
    if (!run) {                          /* (stopped: by PLAY elsewhere, PANIC, MIDI STOP) */
        ln.pst = 0;
        ln.fix = 0;
        ln.emp = 0;
        ln.rh = 0;
        if (ln.play || ln.next) { ln.play = ln.next = 0; ln.lcd = 0xFFu; }
    } else if (!ln.play && !ln.next) {
        ln.emp = 0;                      /* (no row of the launcher's: the clock is the parts') */
    } else {
        const uint32_t rec = ln_rec();   /* (1.18C30) RECORD on: the row is held, its patterns loop for as long as it is on: no
                                          * row end by length / Nx / NO, no asks, no switch. A row asked for by the auto order is
                                          * taken back (its asked patterns too, ln_fix); one queued by PLAY waits (its line moves
                                          * on to the next bar) and is taken after RECORD is off. RECORD off: the row's count
                                          * restarts, with the pattern phase kept: the remaining cycle, then the mode's rest */
        if (rec) {
            ln.rh = 1;
            if (ln.next && !ln.qd) {
                if (ln.iss && ln.next != ln.play) ln.fix = 40u;
                ln.next = ln.iss = 0;
            } else if (ln.next) {
                const uint32_t d = t - ln.r0 < 0x80000000u ? t - ln.r0 : 0u;
                if (ln.iss) ln.fix = 40u;
                ln.iss = 0;
                ln.sw = ln.r0 + ((d + LN_ASK1) / LN_BAR + 1u) * LN_BAR;
            }
        } else if (ln.rh) {
            ln.rh = 0;
            if (ln.play && !ln.next && t - ln.r0 < 0x80000000u) {
                const uint32_t l = ln_len(ln.play - 1u);
                ln.r0 = t - (t - ln.r0) % l;
                ln.len = 0;
            }
        }
        if (!rec && !ln.next && ln.play) {   /* (the row's length while nothing waits: the patterns as they are now) */
            const uint32_t r = ln.play - 1u, nx = LN_NX[set.ln[r][3] < LN_NMODE ? set.ln[r][3] : 0u];
            if (!ln.len && t - ln.r0 < 0x80000000u) ln.len = ln_len(r);   /* (once: the row's cycles counted from its start) */
            if (set.ln[r][3] == LN_NO) { /* (1.18C21: set to NO while it plays: it ends at the next bar's line (nearer to it
                                          * than LN_ASK1: the one after), the next row not NO on from there; none: the stop) */
                const uint32_t d = t - ln.r0 < 0x80000000u ? t - ln.r0 : 0u, a = ln_after(r);
                ln.next = (uint8_t)(a ? a : r + 1u);
                ln.sw = ln.r0 + ((d + LN_ASK1) / LN_BAR + 1u) * LN_BAR;
                ln.iss = a ? 0u : 7u;
            } else if (nx && ln.len && (ln.next = (uint8_t)(ln_solo(r) ? r + 1u : ln_after(r))) != 0u) {   /* (1.18C18: alone, it plays its Nx and stops) */
                ln.sw = ln.r0 + nx * ln.len;
                ln.iss = 0;
                if (ln.next == ln.play) ln.iss = 7u;   /* (the only row: on as it is) */
            }
        }
        if (!rec && ln.next && ln.iss != 7u && ln.sw - t <= LN_ASK0) ln_ask(ln.next - 1u);   /* (its last quarter bar, or
                                          * later: then at the line after) */
        if (ln.play && ck.on) ln.emp = 1;   /* (1.18C21: started from a sequencer's page, ln_seq_play: once the parts' start
                                          * has started the clock) */
        if (ln.fix && ln.play && (!ln.next || !ln.iss)) ln_fix();   /* (1.18C21: not against the next row's asks) */
        if (!rec && ln.next && t - ln.sw < 0x80000000u) {   /* (the line passed: that row playing from it) */
            if (ln.next == ln.play && !ln.qd) {   /* (1.18C18: the only row, its Nx played: the launcher stops) */
                seq_play(0);
                ln.play = ln.next = 0;
                ln.fix = 0;
            } else {
                if (ln.iss != 7u) ln_ask(ln.next - 1u);   /* (asked again late: an edit after the ask cleared its part's bit) */
                ln.play = ln.next;
                ln.next = ln.qd = 0;
                ln.r0 = ln.sw;
                ln.len = 0;
                ln.fix = 40u;
            }
            ln.lcd = 0xFFu;
        }
    }
    ln_sync();
    if (ui.view == V_LNCH) {             /* (the strip: the row playing, the one waiting, a list's column) */
        const uint32_t g = (uint32_t)(ln.play | ln.next << 5 | 0u) & 0xFFu;
        if (g != ln.lcd) { ln.lcd = (uint8_t)g; ui.d_lcd = 1; }
    }
#ifdef APP_HOST_CMD
    host_ln_tap();
#endif
}
static UI_LATE void ln_open(void)        /* SHIFT + SEQ: the launcher */
{
    if (ui.view == V_GLO && ui.gch == G_SEARCH) gd_close();
    overlay_off();
    pl_n = 0;                            /* (pl_tab's room: the launcher's parts drawn now, SQ_PART) */
    sqv.menu = 0;
    ln.ret = 0;
    ln.lcd = 0xFFu;
    lset.on = lset.ask = 0;              /* (1.18C27: the rows first) */
    ui.view = V_LNCH;
    view_reset();
}
static UI_LATE int ln_back(void)         /* HOME on a sequencer's page opened by EDIT here: back (1) */
{
    if (!ln.ret || (ui.view != V_SEQ && ui.view != V_DRUM) || se_on()) return 0;
    ln_open();
    return 1;
}
static UI_LATE void ln_edit(void)        /* a part key held, EDIT: that part's sequencer page, always (the cell's pattern, else the current one) */
{
    const uint32_t q = ln.keys & 1u ? 0u : ln.keys & 2u ? 1u : 2u;
    int32_t p;
    if (ln.play) ln.sel = (uint8_t)(ln.play - 1u);   /* (EDIT before the hold time: the row playing's cell, as when held) */
    p = (int32_t)set.ln[ln.sel][q] - 1;
    ln.kused |= (uint8_t)(1u << q);
    if (p >= 0) ln_part(q, p);           /* (EDIT always opens the part's page: on the cell's pattern, running at its line; NONE: on the part's current one) */
    overlay_off();
    pl_n = 0;
    sqv.menu = 0;
    if (q == 2u) dr_page();
    else { sqv.first = (uint8_t)q; sq_page(q); }
    ln.ret = 1;
}
static UI_LATE void ln_cell_step(uint32_t k, int32_t e)   /* K1..K3: the row's cell k stepped in place (NONE, 1..N; SYNTH
                                          * 1's and 2's never the same pattern), set at once and played: running, at
                                          * the next bar's line (sq_qwant); stopped, that part's sequencer started
                                          * with it (HOME stops it) */
{
    const uint32_t n = k == 2u ? DR_PATS : SQ_PATS;
    const int32_t o0 = k == 2u ? -2 : (int32_t)set.ln[ln.sel][!k] - 1, o = o0 < 0 ? -2 : o0, d = e > 0 ? 1 : -1;
    const int32_t v0 = (int32_t)set.ln[ln.sel][k] - 1;
    int32_t v = v0;
    for (; e; e -= d) {
        int32_t w = clamp(v + d, -1, (int32_t)n - 1);
        if (w == o) w = clamp(w + d, -1, (int32_t)n - 1);
        if (w != o) v = w;
    }
    if (v == v0) return;
    set.ln[ln.sel][k] = (uint8_t)(v + 1);
    if (v >= 0 && set.ln[ln.sel][3] == LN_NO) set.ln[ln.sel][3] = LN_REP;   /* (1.18C18: a row's first pattern: NO, the default, becomes REP) */
    if (ln_act() != ln.sel + 1u) {       /* (1.18C17: only the active row is audible (the row playing, else the last
                                          * played or previewed, prow; 1.18C21 none yet: row 1): an edit in any other row, also
                                          * with nothing sounding, changes the table alone; the row waiting is asked again) */
        if (ln.next == ln.sel + 1u) ln.iss &= (uint8_t)~(1u << k);
        return;
    }
    {
        const int stopped = !LN_RUN;
        ln_part(k, v);
        if (v >= 0 && !sq_qrun(k)) {     /* (the part not running: it starts, alone from a stop, else joining silent
                                          * to the bar's line, as PATTERN does) */
            if (stopped) {
                if (!ck.on) ck.mt = 0;
                sq_pvm = 0;              /* (as PLAY starts them, seq_play) */
                sq_qr[0] = sq_qr[1] = sq_qr[2] = SQ_QR_OFF;
                dr_mute = 0;
                sq_mute[0] = sq_mute[1] = 0;
                ln.pst = 1;
                ln.prow = (uint8_t)(ln.sel + 1u);
            } else (void)sq_qwant(k, -1, v);
            if (k == 2u) dr.run = 1; else sq_on_i(k, 1);
        }
    }
}
static UI_LATE void ln_knob(uint32_t k, int32_t e)   /* K1..K4 on the launcher */
{
    if (k < 3u && (ln.keys & 7u)) {      /* (a part key held: its volume, VOL A / B, the drums' LEVEL) */
        char t[8], v[8];
        uint32_t q;
        for (q = 0; q < 3u; q++) {
            if (!(ln.keys >> q & 1u)) continue;
            ln.kused |= (uint8_t)(1u << q);
            t[0] = v[0] = 0;
            if (q == 2u) {
                set.dr_lev = (uint8_t)clamp((int32_t)set.dr_lev + knob_steps(k, e), 0, DR_LMAX);
                dr_level();
                glo_touch();
                text_cat(t, "DRUMS");
                put_sint(v, set.dr_lev, 0);
            } else {
                pv_vol[q] = (uint8_t)clamp((int32_t)pv_vol[q] + knob_steps(k, e), 0, 127);
                glo_touch();             /* (1.19.7: saved with GLO's settings) */
                text_cat(t, q ? "VOL B" : "VOL A");
                put_sint(v, pv_vol[q], 0);
            }
            app_say(t, v);
            ln_vq = (uint8_t)(q + 1u);
        }
        return;
    }
    if (k == 3u) {                       /* K4: the row's mode */
        const uint32_t m = (uint32_t)clamp((int32_t)set.ln[ln.sel][3] + e, 0, LN_NMODE - 1);   /* (1.18C20: all the steps, not one) */
        if (m == set.ln[ln.sel][3]) return;
        set.ln[ln.sel][3] = (uint8_t)m;
        if (ln.play == ln.sel + 1u && ln.next && !ln.qd && (!ln.iss || ln.next == ln.play || m == LN_NO)) {   /* (the row playing:
                                          * its next row and line again from the new mode; 1.18C21: NO, also with
                                          * the next row asked for: the next bar's line then) */
            if (ln.iss && ln.next != ln.play) ln.fix = 40u;   /* (the asks taken back: the parts on the row playing again, ln_fix) */
            ln.next = 0;
            ln.iss = 0;
        } else if (m == LN_NO && ln.next == ln.sel + 1u && ln.next != ln.play) {   /* (1.18C21: the row waiting set to NO:
                                          * skipped, the row playing's next one found again) */
            if (ln.iss) ln.fix = 40u;
            ln.next = ln.qd = ln.iss = 0;
        }
        return;
    }
    ln_cell_step(k, e);
}
/* (1.18C27) the launcher's sets (ls_*, the record: ls_rec_t): their page comes up when SELECT
 * goes on up past row 1 (above the table; below row 16 SELECT stops); the list reads upwards from the launcher, SET 1 at
 * the bottom (the cursor starts there), SELECT up (like K1 counter-clockwise) moves to the higher SET numbers, SELECT
 * down back, and down past SET 1 (the turn begun on it after LN_EDGE_MS of calm) it returns to the rows; HOME too.
 * K1 the slot (SET 1 .. LS_SLOTS: filled, its patches' names; empty), K4 the action, REC does it:
 *   LOAD    the slot into the launcher: its rows, the parts' volumes, the drum kit, part A's and B's patch (loaded as
 *           PRESET loads one: the patch as stored, with its edits kept in this session; part B's a block later), the
 *           tempo (stopped only: a sequencer running keeps the tempo running). No flash written (the kit, the drums'
 *           LEVEL and the tempo are changes waiting for SAVE, as the kit's page's and GLO's)
 *   SAVE    the launcher now into the slot ("OVERWRITE SET n?" REC again for a filled one); SAVE on the page too
 *   CLEAR   the launcher's rows emptied (asked; no flash)
 *   DELETE  the slot emptied (asked)
 * A write while the transport runs waits for its stop ("SAVE PENDING", as the patterns'). The launcher's own SAVE (on
 * its rows) opens the page with SAVE. Nothing of the launcher is restored at power-on (ln_check): its rows come back
 * only by a LOAD */
#define LS_ASK_MS 4000u
#define LN_EDGE_MS 300u
static UI_LATE int ls_v1(ls_rec_t *r)    /* a record of LS_V1 (or LS_V2) as one of LS_V2 (1.18C30: the init patch's
                                          * number moved); 0: neither */
{
    uint32_t q;
    if (r->m0 != LS_M0 || r->m1 != LS_M1 || (r->ver != LS_V1 && r->ver != LS_V2)) return 0;
    if (r->ver == LS_V1)
        for (q = 0; q < 2u; q++) {
            const uint32_t i = (uint32_t)r->pt[q][0] | (uint32_t)r->pt[q][1] << 8;
            const uint32_t j = i == OLD_PATCH_INIT ? PATCH_INIT : i > OLD_PATCH_INIT ? PATCH_N : i;   /* (past it: none,
                                          * as a slot gone) */
            r->pt[q][0] = (uint8_t)j; r->pt[q][1] = (uint8_t)(j >> 8);
        }
    r->ver = LS_V2;
    return 1;
}
static UI_LATE int ls_read(uint32_t k, ls_rec_t *r)   /* slot k's record into r (1: a set) */
{
    return k < LS_SLOTS && plat_store_read(OBJ_LSET, k * LS_REC, r, LS_REC) == (int)LS_REC && ls_v1(r);
}
static UI_LATE uint32_t ls_full(void)    /* the slots filled (bits; read once: their heads) */
{
    uint32_t k;
    uint8_t h[3];
    if (lset.scan) return lset.full;
    lset.full = 0;
    for (k = 0; k < LS_SLOTS; k++)
        if (plat_store_read(OBJ_LSET, k * LS_REC, h, 3u) == 3 && h[0] == LS_M0 && h[1] == LS_M1
            && (h[2] == LS_V1 || h[2] == LS_V2))
            lset.full |= (uint16_t)(1u << k);
    lset.scan = 1;
    return lset.full;
}
static UI_LATE void ls_make(ls_rec_t *r) /* the launcher as it is now as a record */
{
    const uint8_t *pa = ed_part ? shadow_o : shadow, *pb = ed_part ? shadow : shadow_o;
    const uint32_t ia = ed_part ? patch_o : set.patch, ib = ed_part ? set.patch : patch_o;
    uint32_t k;
    for (k = 0; k < LS_REC; k++) ((uint8_t *)r)[k] = 0;
    r->m0 = LS_M0; r->m1 = LS_M1; r->ver = LS_V2;
    for (k = 0; k < sizeof set.ln; k++) ((uint8_t *)r->ln)[k] = ((const uint8_t *)set.ln)[k];
    r->vol[0] = pv_vol[0]; r->vol[1] = pv_vol[1];
    r->dlev = set.dr_lev;
    r->tempo = tp_run;
    r->pt[0][0] = (uint8_t)ia; r->pt[0][1] = (uint8_t)(ia >> 8);
    r->pt[1][0] = (uint8_t)ib; r->pt[1][1] = (uint8_t)(ib >> 8);
    for (k = 0; k < 10u; k++) { r->nm[0][k] = (char)pa[240u + k]; r->nm[1][k] = (char)pb[240u + k]; }
    r->kit = drk;
}
static UI_LATE int ls_put(uint32_t k)   /* ls_buf into slot k now (zeros: emptied): 0 = done */
{
    const int e = store_write(OBJ_LSET, k * LS_REC, &ls_buf, LS_REC);
    if (!e) lset.full = (uint16_t)(ls_buf.m0 ? ls_full() | 1u << k : ls_full() & ~(1u << k));
    lset.gen++;
    return e;
}
static UI_LATE void ls_said(uint32_t k, const char *v)   /* the strip: "SET n" and v */
{
    char t[8] = "SET ";
    put_sint(t + 4, (int32_t)k + 1, 0);
    app_say(t, v);
}
static UI_LATE void ls_write(int del)    /* SAVE (del: DELETE) of the slot: at once, or at the transport's stop */
{
    const uint32_t k = lset.slot;
    uint32_t i;
    if (del) for (i = 0; i < LS_REC; i++) ((uint8_t *)&ls_buf)[i] = 0;
    else ls_make(&ls_buf);
    if (sp_runs()) { lset.pend = (uint8_t)(k + 1u); ls_said(k, "SAVE PENDING"); return; }
    lset.pend = 0;
    ls_said(k, ls_put(k) ? "SAVE FAILED" : del ? "DELETED" : "SAVED");
}
static UI_LATE void ln_reset_run(void)   /* the table changed under the launcher (LOAD, CLEAR): the next row looked for
                                          * again, the row playing's cells taken (ln_fix) */
{
    ln.next = ln.qd = ln.iss = 0;
    ln.len = 0;
    ln.fix = ln.play ? 40u : 0u;
    ln.lcd = 0xFFu;
}
static UI_LATE void ls_swap(void)        /* the part edited and the other one swapped (the panel's buffers, as part_edit;
                                          * nothing else: ls_patch's alone) */
{
    uint32_t k;
    for (k = 0; k < SH_LEN; k++) { const uint8_t t = shadow[k]; shadow[k] = shadow_o[k]; shadow_o[k] = t; }
    { const uint16_t t = set.patch; set.patch = patch_o; patch_o = t; }
    { const uint8_t t = sx_edit; sx_edit = sx_o; sx_o = t; }
    ed_part = (uint8_t)!ed_part;
}
static UI_LATE void ls_patch(uint32_t q, uint32_t i)   /* part q's patch i, as PRESET loads it (its edits of this
                                          * session kept; the tempo the global one) */
{
    const int sw = q != ed_part;
    if (sw) ls_swap();
    set.patch = (uint16_t)i;
    load_patch(i);
    if (sw) ls_swap();
    home_sig[0] = home_sig[1] = 0;
    ui.d_lcd = 1;
}
static UI_LATE uint32_t ls_pnum(const ls_rec_t *r, uint32_t q, uint32_t *chg)   /* part q's patch of r: its number,
                                          * the init patch for one gone (slot emptied); chg bit q: not as saved */
{
    uint32_t i = (uint32_t)r->pt[q][0] | (uint32_t)r->pt[q][1] << 8, k;
    const uint8_t *b;
    if (i >= PATCH_N || !patch_filled(i)) { *chg |= 1u << q; return PATCH_INIT; }
    b = patch_bytes(i);
    for (k = 0; k < 10u; k++)
        if (b[240u + k] != (uint8_t)r->nm[q][k]) { *chg |= 1u << q; break; }
    return i;
}
static UI_LATE void ls_load(uint32_t k)  /* LOAD: slot k into the launcher */
{
    uint32_t i, chg = 0, ia, ib;
    if (!ls_read(k, &ls_buf)) { ls_said(k, "EMPTY"); return; }
    if (ln_rows_bad(&ls_buf.ln[0][0])) for (i = 0; i < sizeof ls_buf.ln; i++) ((uint8_t *)ls_buf.ln)[i] = 0;
    for (i = 0; i < sizeof set.ln; i++) ((uint8_t *)set.ln)[i] = ((const uint8_t *)ls_buf.ln)[i];
    ln_reset_run();
    pv_vol[0] = ls_buf.vol[0] & 127u;
    pv_vol[1] = ls_buf.vol[1] & 127u;
    set.dr_lev = (uint8_t)(ls_buf.dlev > DR_LMAX ? DR_LMAX : ls_buf.dlev);
    for (i = 0; i < DR_NI; i++) drk.k[i] = kit_valid(&ls_buf.kit.k[i]) ? ls_buf.kit.k[i] : kit_p0(i);
    drk.drv = (uint8_t)((ls_buf.kit.drv & 15u) < DV_N && (ls_buf.kit.drv >> 4) < DC_N ? ls_buf.kit.drv : DV_WARM);
    kv.dirty_t = plat_ms() | 1u;         /* (the kit changed: saved as the kit's page's changes are) */
    dr_level();
    glo_touch();                         /* (the drums' LEVEL: GLO's) */
    if (ls_buf.tempo && ls_buf.tempo <= 128u && !sp_runs() && !LN_RUN && ls_buf.tempo != tp_run)
        edit_set(TP_BYTE, ls_buf.tempo - 1u, plat_ms());   /* (stopped: the set's tempo; running: the one running) */
    ia = ls_pnum(&ls_buf, 0u, &chg);
    ib = ls_pnum(&ls_buf, 1u, &chg);
    all_notes_off();
    ls_patch(0u, ia);
    lset.pb = (uint16_t)(ib + 1u);         /* (part B's when the engine took A's: one patch_req at a time) */
    lset.chg = (uint8_t)chg;
    ls_said(k, chg == 3u ? "A+B CHANGED" : chg ? (chg & 1u ? "A CHANGED" : "B CHANGED") : "LOADED");
    lset.gen++;
}
static UI_LATE void ls_clear(void)       /* CLEAR: the launcher's rows emptied */
{
    uint32_t i;
    for (i = 0; i < sizeof set.ln; i++) ((uint8_t *)set.ln)[i] = 0;
    ln_reset_run();
    app_say("Launcher", "CLEARED");
}
static UI_LATE void ls_idle(void)        /* app_frame (ln_idle): part B's patch after A's, a write at the stop */
{
    if (lset.pb && patch_req < 0) { ls_patch(1u, lset.pb - 1u); lset.pb = 0; }
    if (lset.pend && !sp_runs()) {
        const uint32_t k = lset.pend - 1u;
        lset.pend = 0;
        ls_said(k, ls_put(k) ? "SAVE FAILED" : ls_buf.m0 ? "SAVED" : "DELETED");
    }
    if (lset.ask && plat_ms() - lset.ask_t > LS_ASK_MS) { lset.ask = 0; lset.gen++; }
}
static UI_LATE void ls_open(void)        /* the sets' page (above row 1; on = 1) */
{
    lset.on = 1u;
    lset.rsel = ln.sel;
    lset.ask = 0;
    ln.selt = plat_ms();                 /* (the turn that opened it does not also leave it) */
    lset.slot = 0u;                      /* (the cursor on SET 1, the slot next to the launcher: drawn at the bottom, the
                                          * numbers rise upwards (ui.c ls_*); the stored slot n-1 is SET n as ever) */
    ls_full();
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void ls_close(void)       /* the rows again (the row the page was reached from) */
{
    ln.sel = lset.rsel < LN_ROWS ? lset.rsel : 0u;
    lset.on = 0;
    lset.ask = 0;
    ln.lcd = 0xFFu;                      /* (the strip: the launcher's again) */
    ui.d_rows = ui.d_lcd = 1;
}
static UI_LATE void ls_do(uint32_t a)    /* REC (SAVE: a = LS_SAVE) on the page: the action, asked first where it
                                          * would lose something */
{
    const uint32_t k = lset.slot, f = ls_full() >> k & 1u;
    uint32_t i, rows = 0;
    if (lset.pend) { ls_said(lset.pend - 1u, "SAVE PENDING"); return; }   /* (a write waits in ls_buf: nothing else meanwhile) */
    for (i = 0; i < sizeof set.ln; i++) rows |= ((const uint8_t *)set.ln)[i];
    if (lset.ask != a + 1u && ((a == LS_SAVE && f) || (a == LS_DEL && f) || (a == LS_CLEAR && rows))) {
        lset.ask = (uint8_t)(a + 1u);
        lset.ask_t = plat_ms();
        lset.gen++;
        ls_said(k, a == LS_SAVE ? "OVERWRITE? REC" : a == LS_DEL ? "DELETE? REC" : "CLEAR ROWS? REC");
        return;
    }
    lset.ask = 0;
    lset.gen++;
    if (a == LS_LOAD) ls_load(k);
    else if (a == LS_SAVE) ls_write(0);
    else if (a == LS_CLEAR) ls_clear();
    else if (!f) ls_said(k, "EMPTY");
    else ls_write(1);
}
static UI_LATE void ls_pre(uint32_t nw)  /* ln_pre on the sets' page: its buttons (nw: new ones) and knobs */
{
    uint32_t k;
    int32_t e;
    if (nw >> B_HOME & 1u) { ui.btn |= 1u << B_HOME; ls_close(); return; }
    if (nw >> B_REC & 1u) { ui.btn |= 1u << B_REC; ls_do(lset.ask ? lset.ask - 1u : lset.act); }   /* (asked: its yes) */
    if (nw >> B_SAVE & 1u) { ui.btn |= 1u << B_SAVE; ls_do(LS_SAVE); }
    if ((e = plat_enc(EN_SELECT)) != 0) {   /* (1.18C28: SELECT moves the cursor through the slots, up (e < 0) to the higher
                                          * SET numbers, as K1 counter-clockwise; it leaves only down past SET 1, a turn begun
                                          * there after LN_EDGE_MS of calm: a fast scroll stops at SET 1) */
        const uint32_t now = plat_ms(), still = now - ln.selt >= LN_EDGE_MS;
        ln.selt = now;
        if (lset.slot == 0u && e > 0 && still && e <= 2) { ls_close(); return; }
        lset.slot = (uint8_t)clamp((int32_t)lset.slot - e, 0, LS_SLOTS - 1);
        lset.ask = 0;                    /* (a confirm asked is cancelled by the cursor moving) */
        ui.d_lcd = 1;
    }
    for (k = 0; k < 4u; k++) {
        if ((e = plat_enc(EN_K1 + (int)k)) == 0 || (k != 0u && k != 3u)) continue;
        if (k == 0u) lset.slot = (uint8_t)clamp((int32_t)lset.slot - e, 0, LS_SLOTS - 1);   /* (K1 like SELECT: counter-clockwise = higher) */
        else lset.act = (uint8_t)clamp((int32_t)lset.act + e, 0, LS_NACT - 1);
        lset.ask = 0;
        ui.d_lcd = 1;                    /* (the page: by its cells' signatures, ls_draw) */
    }
}
static UI_LATE void ln_save(void)        /* SAVE on the launcher's rows (1.18C27): the sets' page (SAVE again there saves) */
{
    ls_open();
    app_say("Sets", "SAVE: K1 SLOT, SAVE");
}
static UI_LATE void ln_seq_play(void)    /* 1.18C21: PLAY from a stop on a sequencer's page runs the launcher too: the active
                                          * row (not NO; else the first row not NO); no row defined (all NO): row 1 set to
                                          * the sequencers' patterns now, REP (none at all: the sequencers' PLAY as before) */
{
    uint32_t r, q;
    for (r = 0; r < LN_ROWS && set.ln[r][3] == LN_NO; r++) {}
    if (r == LN_ROWS) {
        if (ln_cur(0) < 0 && ln_cur(1) < 0 && ln_cur(2) < 0) { seq_play(1); return; }
        r = 0;
        for (q = 0; q < 3u; q++)
            set.ln[0][q] = (uint8_t)(ln_cur(q) + 1);
        if (set.ln[0][1] && set.ln[0][1] == set.ln[0][0]) set.ln[0][1] = 0;
        set.ln[0][3] = LN_REP;
    } else if (set.ln[ln_act() - 1u][3] != LN_NO) r = ln_act() - 1u;
    ln.sel = (uint8_t)r;
    ln_play();
    if (SQ_RUNS || dr.run) ln.emp = 0;   /* (the clock started by the parts, as their own PLAY: a preview's cut a block
                                          * before their start, the audio's sq_pva; the launcher's hold on it from the next frame, ln_idle) */
}
static UI_LATE void ln_pre(void)         /* app_frame, before input: the launcher's buttons and knobs (taken here,
                                          * input's code left as it is: a button taken is marked down in ui.btn) */
{
    const uint32_t btn = plat_buttons(), nw = btn & ~ui.btn;
    uint32_t k;
    int32_t e;
    if (pk.ask && (nw >> B_REC & 1u)) { ui.btn |= 1u << B_REC; (void)pk_rec(); return; }   /* (1.18C33: the question,
                                          * every page) */
    if (pk.ask && (nw >> B_HOME & 1u)) { ui.btn |= 1u << B_HOME; (void)pk_home(); return; }
    if ((ui.view == V_SEQ || ui.view == V_DRUM) && (nw >> B_PLAY & 1u) && !(btn >> B_OCTDN & 1u) && !LN_RUN) {
        ui.btn |= 1u << B_PLAY;          /* (1.18C21: one PLAY: the launcher runs with the sequencers) */
        ln_seq_play();
        pm_rel = 4;
        return;
    }
    if (ui.view != V_LNCH || (btn >> B_OCTDN & 1u)) return;   /* (SHIFT: input's) */
    if (nw >> B_PLAY & 1u) {             /* (1.19.7: running, on the release of a short press: a hold leaves it) */
        ui.btn |= 1u << B_PLAY;
        if (LN_RUN) pm_rel = 2;
        else { ln_play(); pm_rel = 4; }
    }
    if (lset.on) { ls_pre(nw); return; }   /* (1.18C27: the sets' page) */
    if ((nw >> B_HOME & 1u) && ln.pst && !ln.play && (SQ_RUNS || dr.run)) {   /* (a preview from a stop: stopped) */
        ui.btn |= 1u << B_HOME;
        seq_play(0);
        ln.pst = 0;
    } else if ((nw >> B_HOME & 1u) && ln.play && ln.sel != ln.play - 1u) {   /* (1.18C11: HOME off the row playing:
                                          * the display goes to it; on it, or nothing plays: as before) */
        ui.btn |= 1u << B_HOME;
        ln.sel = (uint8_t)(ln.play - 1u);
    }
    if ((nw >> B_EDIT & 1u) && (ln.keys & 7u)) { ui.btn |= 1u << B_EDIT; ln_edit(); return; }
    if (nw >> B_SAVE & 1u) { ui.btn |= 1u << B_SAVE; ln_save(); return; }
    if ((e = plat_enc(EN_SELECT)) != 0) {   /* (1.18C28: up past row 1, a turn begun there (SELECT still for LN_EDGE_MS
                                          * before; a turn that reached the row stops at it): the sets' page; row 16 stops) */
        const uint32_t now = plat_ms(), still = now - ln.selt >= LN_EDGE_MS;
        ln.selt = now;
        if (ln.sel == 0u && e < 0 && still && e >= -2 && e <= 2 && LS_EDGE) {
            ls_open();
            return;
        }
        ln.sel = (uint8_t)clamp((int32_t)ln.sel + e, 0, LN_ROWS - 1);
    }
    for (k = 0; k < 4u; k++)
        if ((e = plat_enc(EN_K1 + (int)k)) != 0) ln_knob(k, e);
}
static UI_LATE uint32_t ln_lights(uint32_t *lb, uint32_t *mid)   /* sq_lights on the launcher: the part keys lit,
                                          * half lit muted; HOME; EDIT with a part key held; REC on a list */
{
    const uint32_t m = ln_mute & 7u, n = ~m & 7u;   /* (muted: half lit) */
    static const uint8_t BK[8] = {14, 16, 18, 19, 21, 23, 24, 26};   /* (1.18C21: the eight highest white keys, the row's bar) */
    uint32_t bar = 0, i, f = 0;
    *lb |= 1u << B_HOME | (ln.keys & 7u ? 1u << B_EDIT : 0u) | (lset.on ? 1u << B_REC | 1u << B_SAVE : 0u);   /* (the sets' page: REC, SAVE) */
    *mid = (m & 1u) | (m << 1 & 4u) | (m << 2 & 16u);   /* (the keys' bits 0, 2, 4) */
    if (ln.play && LN_RUN) {             /* (the row's run so far, filled left to right: its longest pattern times its
                                          * repeats (REP: one pass), a key an eighth, the first lit from its start; full,
                                          * the next row at the line) */
        const uint32_t r = ln.play - 1u, nx = LN_NX[set.ln[r][3] < LN_NMODE ? set.ln[r][3] : 0u],
                       d = ck.mt - ln.r0 < 0x80000000u ? ck.mt - ln.r0 : 0u;
        uint32_t tot = (ln.len ? ln.len : LN_BAR) * (nx ? nx : 1u);
        if (ln.next && !ln.qd && ln.sw - ln.r0 - 1u < 0x7FFFFFFFu) tot = ln.sw - ln.r0;   /* (NO: up to its bar's line) */
        f = 1u + (uint32_t)((uint64_t)(d % tot) * 8u / tot);
    } else bar = 1u << BK[0] | 1u << BK[4] | 1u << BK[7];   /* (idle: keys 1, 5 and 8 half lit, the bar's marks) */
    *mid |= bar;
    for (bar = 0, i = 0; i < f && i < 8u; i++) bar |= 1u << BK[i];
    return (n & 1u) | (n << 1 & 4u) | (n << 2 & 16u) | bar;
}
static UI_LATE void ln_lcd(char *a, char *v)   /* the strip: a list's column and row; else the row playing, waiting */
{
    if (lset.on) {                       /* (1.18C27: the sets' page: the slot, the action) */
        text_cat(a, "SET ");
        put_sint(a + 4, lset.slot + 1, 0);
        text_cat(v, LS_ACT[lset.act < LS_NACT ? lset.act : 0]);
        return;
    }
    text_cat(a, "LAUNCHER");
    if (!ln.play && !ln.next) { text_cat(v, "STOP"); return; }
    if (ln.play) { text_cat(v, "ROW "); put_sint(v + 4, ln.play, 0); }
    if (ln.next && ln.next != ln.play) { text_cat(v, ">"); put_sint(v + text_len(v), ln.next, 0); }
}
#endif
/* ------------------------------------------------- the lights' power-on show and NIGHT --- */
/* (1.18C43) Two things change the panel's lights on their way out (lt_frame, from input()'s
 * lights; main loop, flash code: nothing in the audio interrupt, no timer of its own, the scan's levels as they are):
 * - the power-on show (LED_BOOT_ANIM): after the boot image, a light runs over the keys from F3 to G5 while the
 *   buttons' columns light from right (GLO / REC) to left (FX / HOME, then OCT- / OCT+), each light dark, glow,
 *   middle, lit, middle, glow, dark, the next one rising while the one before falls (three keys at most); then, KEY
 *   LIGHTS on, the keys' own lights come up from dark in LB_FADE_MS (1.18C46: in LB_NSTEP even steps to the eye, one
 *   level for all keys, plat_lights_fade; NIGHT to its middle level). Anything played ends it at once (a key, a
 *   button, a voice sounding, the drums, a sequencer: the lights as they are then, nothing faded): so does any change
 *   of what the lights would show (a knob that edits, a page, MIDI's notes through the voices);
 * - KEY LIGHTS NIGHT (set.key_dark KL_NIGHT): every light a level down: lit to the middle level, the middle to the
 *   glow, the glow dark (PLAY's green, which has no middle level: lit to the glow).
 * The scan's levels per light (fm1-base plat_lights / plat_lights_mid, felucca-bsp's scan): dark, the glow (a short
 * pulse each scan frame, ~3 % of lit), the middle level (a lit frame in 8, ~12 %; with the glow under it ~15 %),
 * lit; and (1.18C46, plat_lights_fade) one more level common to the keys given, any of 255 (felucca-bsp's fade group,
 * local: up to the glow the glow on a share of the scan's frames, over it whole lit frames among glowing ones, both
 * evenly spread, chosen once a frame).
 * Neither changes a thing while neither runs: then the lights go out as in 1.18C42 (input()). */
#ifndef LED_BOOT_ANIM
#define LED_BOOT_ANIM 1                  /* 0: no power-on show (the lights as 1.18C42's) */
#endif
#define LB_KEY_STEP_MS 18u               /* a key's level held (the show's keys: 5 levels, LB_ENV) */
#define LB_KEY_GAP_MS (2u * LB_KEY_STEP_MS)   /* a key's start to the next's: rises while the one before falls */
#define LB_BTN_STEP_MS 60u               /* the buttons' columns the same, slower (7 columns in the keys' time) */
#define LB_BTN_GAP_MS (2u * LB_BTN_STEP_MS)
#define LB_NCOL 7u
#define LB_CHASE_MS (26u * LB_KEY_GAP_MS + 5u * LB_KEY_STEP_MS)   /* the run: 1 026 ms (the buttons' 1 020) */
#define LB_FADE_MS 2000u                 /* then the keys' lights up from dark (KEY LIGHTS ON or NIGHT) */
static const uint8_t LB_ENV[5] = {1, 2, 3, 2, 1};   /* a light's levels: 1 glow, 2 middle, 3 lit */
/* (1.18C46) the fade: LB_NSTEP steps of LB_FADE_MS / LB_NSTEP (62.5 ms), step i at lightness
 * L* = 100 i / 32 (CIE: even steps to the eye), as plat_lights_fade's level (/256 of lit: (Y 65536 / 254), 0 dark):
 * up to the glow (8) the glow on a share of the frames (from ~1 in 10, ~97 a second); over it whole lit frames, evenly
 * spread: at least one in 16 (~57 a second, as felucca-bsp's own power-on sweep at its lowest; fewer would flicker),
 * so the levels between the glow and 1/16 of lit
 * (L* 21 .. 36) are left out: the steps there stay at the glow (8) or go to the floor (24), the nearer one. NIGHT
 * the first LB_NNIGHT of them, then its middle level (39.5). The keys' own lights at or under the level show as they
 * are (lit, the middle level, the glow: LB_LV_*) */
#define LB_NSTEP 32u
#define LB_NNIGHT 15u                    /* (the steps under NIGHT's middle level: L* 0 .. 43.8) */
#define LB_LV_LIT 256u                   /* (plat_lights_fade's level of a key's own light: lit, the middle level, the
                                          * glow; a key over the fade's level is held at it) */
#define LB_LV_MID 40u
#define LB_LV_GLOW 9u
static const uint8_t LB_LV[LB_NSTEP] = {0, 1, 2, 3, 4, 5, 7, 8, 8, 8, 24, 24, 25, 30, 35, 41, 48, 55, 62, 71, 80, 90, 101,
                                        112, 125, 138, 152, 167, 183, 200, 219, 238};
static const uint16_t LB_COL[LB_NCOL] = {   /* the buttons' columns, right to left (the panel's two rows of six; OCT-
                                          * and OCT+, left of the screen, as the last) */
    1u << B_GLO | 1u << B_REC, 1u << B_EDIT | 1u << B_PLAY, 1u << B_LFO | 1u << B_SEQ, 1u << B_ENV | 1u << B_ARP,
    1u << B_SEL | 1u << B_SAVE, 1u << B_FX | 1u << B_HOME, 1u << B_OCTDN | 1u << B_OCTUP};
static uint8_t lb_st PLAT_POOL;          /* the show: 0 none (or over), 1 to start (app_ui_init's first), 2 running */
static uint32_t lb_fk PLAT_POOL;         /* (1.18C46) the keys out at the fade's level (plat_lights_fade), 0 none */
static uint32_t lb_t0 PLAT_POOL, lb_ref[5] PLAT_POOL;   /* its start; the lights' own picture then (a change ends it) */
static UI_LATE uint32_t lb_lev(uint32_t t, uint32_t step)   /* the show's level t ms into a light's run (0 dark) */
{
    return t < 5u * step ? LB_ENV[t / step] : 0u;
}
/* the lights out (lit, middle, glow: buttons and keys; fk: keys at the fade's level lv, plat_lights_fade) with PLAY's
 * two colours: red (PLAY mode; the show) or green. Only when they change */
static UI_LATE void lt_out(uint32_t now, uint32_t bl, uint32_t bm, uint32_t bd, uint32_t kl, uint32_t km, uint32_t kd,
                           uint32_t fk, uint32_t lv, uint32_t red, uint32_t force)
{
    static uint32_t o[8] PLAT_POOL;      /* (the last ones out, and when: again once a second, as input's; the fade's) */
    const uint32_t pl = 1u << B_PLAY;
    if (!red) {                          /* (green: lit, else the glow; the green LED has no middle level) */
        const uint32_t g = (bl & pl) ? LED_PLAY_GREEN : 0u, gd = !(bl & pl) && ((bm | bd) & pl) ? LED_PLAY_GREEN : 0u;
        bl = (bl & ~pl) | g;
        bm &= ~pl;
        bd = (bd & ~pl) | gd;
    }
    bd |= bm;                            /* (the buttons' middle level with the glow under it, as input's) */
    if (!lv) fk = 0;
    if (fk != lb_fk || (fk ? lv : 0u) != o[7]) plat_lights_fade(fk, lv);   /* (1.18C46: when it changes; before
                                          * plat_lights, the host's trace after it) */
    lb_fk = fk;
    if (!force && o[0] == bl && o[1] == bm && o[2] == bd && o[3] == kl && o[4] == km && o[5] == kd && now - o[6] < 1000u
        && o[7] == (fk ? lv : 0u))
        return;
    o[0] = bl; o[1] = bm; o[2] = bd; o[3] = kl; o[4] = km; o[5] = kd; o[6] = now; o[7] = fk ? lv : 0u;
    plat_lights(bl, kl, bd & ~bl, kd & ~kl, 2);
    plat_lights_mid(bm & ~bl, km & ~kl);
}
/* NIGHT's screen (NIGHT_LCD): the panel's backlight (PA2) is on or off, no PWM (fm1-base lcd.c); the ST7789V's own
 * grey-scale voltage (GVDD = VRH + VDV; reset defaults VRHS 0x0B 4.1 V, VDVS 0x20 0 V, which fm1-base's init leaves)
 * set lower makes the picture's light parts darker: NIGHT ~3.3 V. Two commands when NIGHT changes, nothing else.
 * Not seen on a device yet */
#ifndef NIGHT_LCD
#define NIGHT_LCD 1
#endif
#define NIGHT_VRHS 0x00u                 /* 3.55 V (0x0B: 4.1 V, 0.05 V a step) */
#define NIGHT_VDVS 0x16u                 /* -0.25 V (0x20: 0 V, 0.025 V a step) */
static UI_LATE void lt_lcd(uint32_t night)
{
#if NIGHT_LCD && !defined(PLAT_HOST)
    lcd_cmd(0xC3u);                      /* VRHS (lcd_cmd waits for a transfer still running: then lcd_small is free) */
    lcd_small[0] = (uint8_t)(night ? NIGHT_VRHS : 0x0Bu);
    lcd_data(lcd_small, 1);
    lcd_cmd(0xC4u);                      /* VDVS */
    lcd_small[0] = (uint8_t)(night ? NIGHT_VDVS : 0x20u);
    lcd_data(lcd_small, 1);
    lcd_sync();
#else
    (void)night;
#endif
}
/* input()'s lights through the show and NIGHT: 0 none runs (input's own way, as 1.18C42), 1 done here, -1 back to
 * input's way from here (it sets them now). lb .. mb: what input would show; act: something played */
static UI_LATE int lt_frame(uint32_t now, uint32_t act, uint32_t red, uint32_t lb, uint32_t lk, uint32_t dk, uint32_t mk,
                            uint32_t mb)
{
    static uint8_t was PLAT_POOL, was_n PLAT_POOL;
    const uint32_t night = set.key_dark == KL_NIGHT, all = (1u << NB) - 1u;
    const uint32_t ref[5] = {(lb & ~(1u << B_ARP)) | red << 31, lk, mk, dk, mb};   /* (ARP blinks on its own with the
                                          * beat) */
    uint32_t t = 0, i, bl, bm, bd, kl, km, kd, fk = 0, lv = 0, force = 0;
    if (night != was_n) {                /* (the screen's NIGHT with the lights': lt_lcd) */
        was_n = (uint8_t)night;
        lt_lcd(night);
    }
    if (lb_st == 1u) {
        lb_st = 2u;
        lb_t0 = now;
        for (i = 0; i < 5u; i++) lb_ref[i] = ref[i];
    }
    if (lb_st == 2u) {
        t = now - lb_t0;
        for (i = 0; i < 5u; i++) act |= ref[i] ^ lb_ref[i];
        if (act || t >= LB_CHASE_MS + (KEYS_DARK ? 0u : LB_FADE_MS))
            lb_st = 0;                   /* (played, changed or over: the lights as they are, at once) */
    }
    if (lb_st != 2u && !night) {
        if (!was) return 0;
        was = 0;
        if (lb_fk) { lb_fk = 0; plat_lights_fade(0, 0); }   /* (1.18C46: ended in the fade) */
        return -1;
    }
    if (!was) force = 1;
    was = 1;
    bl = lb;                             /* the lights' own levels (input's: the unlit buttons glow, mb at the middle) */
    bm = mb & ~lb;
    bd = all & ~lb;
    kl = lk;
    km = mk & ~lk;
    kd = dk & ~lk;
    if (night) {                         /* NIGHT: a level down each */
        bd = bm | (red ? 0u : bl & 1u << B_PLAY);   /* (PLAY's green lit: its glow) */
        bm = bl & ~(red ? 0u : 1u << B_PLAY);
        bl = 0;
        kd = km;
        km = kl;
        kl = 0;
    }
    if (lb_st == 2u && t < LB_CHASE_MS) {   /* the run: the show's lights alone */
        bl = bm = bd = kl = km = kd = 0;
        for (i = 0; i < NKEYS; i++) {
            const uint32_t s = i * LB_KEY_GAP_MS, v = t >= s ? lb_lev(t - s, LB_KEY_STEP_MS) : 0u;
            const uint32_t m = 1u << i;
            if (v == 3u && !night) kl |= m;
            else if (v >= 2u) km |= m, kd |= m;
            else if (v) kd |= m;
        }
        for (i = 0; i < LB_NCOL; i++) {
            const uint32_t s = i * LB_BTN_GAP_MS, v = t >= s ? lb_lev(t - s, LB_BTN_STEP_MS) : 0u;
            const uint32_t m = LB_COL[i];
            if (v == 3u && !night) bl |= m;
            else if (v >= 2u) bm |= m;
            else if (v) bd |= m;
        }
        red = 1u;                        /* (PLAY red in the show: the green has no middle level) */
    } else if (lb_st == 2u) {            /* the keys' lights up (1.18C46): one level, LB_LV's step now (NIGHT: its first
                                          * LB_NNIGHT over the time); a key whose own light is higher held at it */
        i = LB_NSTEP * (t - LB_CHASE_MS) / LB_FADE_MS;
        lv = LB_LV[night ? i * LB_NNIGHT / LB_NSTEP : i];
        fk = (lv < LB_LV_LIT ? kl : 0u) | (lv < LB_LV_MID ? km : 0u) | (lv < LB_LV_GLOW ? kd : 0u);
        kl &= ~fk;
        km &= ~fk;
        kd &= ~fk;
    }
    lt_out(now, bl, bm, bd, kl, km, kd, fk, lv, red, force);
    return 1;
}
#ifdef HYB_MULTI
UI_LATE                                  /* (the USB build: after all the code, so the flash's code before the engine's
                                          * stays short enough for the RAM reverb's call into it, UI_LATE's note) */
#endif
static void input(void)
{
    uint32_t keys, btn = plat_buttons(), k, shift, kc;
    int32_t e;
    touch_poll();
    keys = touch_keys();
    un.next = 0;                         /* (the sequencers' undo: a change's kind, set by what makes it) */
    if (seq_req) {                       /* (CLOCK EXTERNAL: START / STOP from MIDI in) */
#ifdef HYB_MULTI
        mh_ext = 1;                      /* (a MIDI START: no muted-part hint, mute_hint_run) */
#endif
        seq_play(seq_req == 1u ? 2 : 0);   /* (MIDI START: no PRECOUNT, 1.18C40) */
        seq_req = 0;
    }
    shift = (btn >> B_OCTDN & 1u) && (ui.btn >> B_OCTDN & 1u) && plat_ms() - ui.shift_t >= SHIFT_HOLD_MS && !ui.oc_pair;
    sr_mi_poll();                        /* (1.18C31: RECORD's MIDI notes in, as keys) */
    for (kc = (keys ^ ui.keys) & ((1u << NKEYS) - 1u); kc; kc &= kc - 1u) {   /* key 0 is F3 (MIDI 53). P8: only the
                                          * keys that changed (every branch below needs a change of its key); ui.keys is
                                          * still read live: a handler (hold_flip) may set it to keys, which only ends
                                          * changes, never makes new ones */
        const uint32_t kk = (uint32_t)__builtin_ctz(kc), bit = 1u << kk;
        k = kk;
        if (!((keys ^ ui.keys) & bit)) continue;
        if ((keys & bit) && !(ui.keys & bit) && shift) {   /* SHIFT: a key is a function, no note */
            ui.held[k] = KEY_NONE;
            ui.shift_used = 1;
            if (ui.view == V_DRUM && sqv.rec && dr_ikey(k) < DR_NI) dr_erase(dr_ikey(k));   /* (RECORD: an
                                                     * instrument's steps erased; not MONO / POLY there) */
            else if (k == KEY_MONO || k == KEY_POLY) key_mode(k == KEY_MONO);
#ifdef HYB_MULTI
            if (k == KEY_EDA || k == KEY_EDB) part_edit(k == KEY_EDB);   /* (the part edited) */
#endif
        } else if ((keys & bit) && !(ui.keys & bit) && ui.view == V_SEQ) {   /* the sequencer's page: a step (RECORD:
                                                     * a note) */
            ui.held[k] = KEY_NONE;
            if (sqv.arec) sq_rec_key(k, 1); else sq_key(k, 1);
        } else if ((keys & bit) && !(ui.keys & bit) && ui.view == V_DRUM) {   /* the drums' page: a step, an
                                                     * instrument */
            ui.held[k] = KEY_NONE;
            dr_key(k, 1);
        } else if ((keys & bit) && !(ui.keys & bit) && ui.view == V_KIT) {   /* the drum kit: an instrument */
            ui.held[k] = KEY_NONE;
            kit_key(k);
        } else if ((keys & bit) && !(ui.keys & bit)) {
            key_on(k);                   /* (its note) */
        } else if (!(keys & bit) && (ui.keys & bit) && ui.held[k] != KEY_NONE) {
            if (hold_on && !fv_aud) hold_set(ui.held[k]);   /* (HOLD: the note stays (1.18C33: not the favorites
                                          * editor's audition); 1.16 SPLIT: its zone's part's off, it
                                          * ends after input, hold_zone_fix) */
            else key_up_off(ui.held[k]);   /* (1.16 SPLIT: the other zone's part's HOLD on, it stays) */
        } else if (!(keys & bit) && (ui.keys & bit) && ui.view == V_SEQ) {
            if (sr.note[k]) sq_rec_key(k, 0); else sq_key(k, 0);
        }
        else if (!(keys & bit) && (ui.keys & bit) && ui.view == V_DRUM)
            dr_key(k, 0);
    }
    for (kc = btn & ~ui.btn & ((1u << NB) - 1u); kc; kc &= kc - 1u) {   /* (P8: the buttons newly down only) */
        uint32_t bit, m;
        k = (uint32_t)__builtin_ctz(kc);
        bit = 1u << k;
        if (!(btn & bit) || (ui.btn & bit))
            continue;
        if (k != B_OCTDN && (btn >> B_OCTDN & 1u))
            ui.shift_used = 1;           /* (pressed with OCT- down: its release no octave) */
        if (pk.ask && k != B_REC && k != B_HOME) { pk.ask = 0; ui.d_lcd = 1; }   /* (1.18C33: the question taken back) */
        if (k == B_HOME && pk_home()) continue;
        if (k == B_HOME && fv_home()) continue;   /* (1.18C33: the favorites editor left; a button that leaves
                                          * HOME leaves it too: fv_idle) */
        if (k == B_EDIT && (ui.view == V_SEQ || ui.view == V_DRUM)) {   /* a sequencer's page: its EDIT list */
            se_open();
        } else if (k == B_EDIT && (ui.view == V_PLAY || (pm_on && (ui.view == V_MENU || ui.view == V_GLIST
                   || ui.view == V_GROUP) && U_MENU[ui.menu].btn == B_EDIT))) {   /* (1.19.6) PLAY mode's page: its
                                          * assignment editor (MATRIX from the editor's EDIT, or any other page's;
                                          * 1.19.7: PLAY mode on, EDIT in MATRIX: the editor again, EDIT toggles) */
            overlay_off();
            played_go();
        } else if ((m = menu_button(k)) < U_NMENU) {   /* a menu's button: its blocks (SEL again: its other page) */
            overlay_off();
            menu_go(m);
        } else if (k == B_HOME && home_undo(shift, btn)) {   /* SHIFT + HOME: undo (a sequencer's page: its
                                          * pattern's) */
        } else if (k == B_HOME && se_on()) {   /* (a sequencer's EDIT list: closed; COPY TO's chooser: the list) */
            se_home();
        } else if ((k == B_HOME || k == B_SAVE) && sv_on()) {   /* SAVE: the editor to the list, the list closed */
            sv_home();
        } else if (k == B_HOME && ui.view == V_PLAYED) {   /* the editor: a level up */
            played_home();
#ifdef HYB_MULTI
        } else if (k == B_HOME && ui.view == V_QUICK && !ui.qpos && set.mx_mode != SY_M_SINGLE && !ui.plist_until
                   && !ui.vl_until) {   /* HOME on HOME, two parts: the part edited, A <-> B, on its release (held:
                                          * the profile line, nothing) */
            home_tap = plat_ms() | 1u;
#endif
        } else if (k == B_HOME) {
            overlay_off();
            qa_home();                   /* (PLAY mode on: its page, pm_home) */
        } else if (k == B_OCTDN || k == B_OCTUP) {   /* SHIFT, the octave on its release; OCT+ the octave on its
                                          * release; the second of the two within OC_PAIR_MS of the first: the pair
                                          * (held: PANIC; neither does its own then) */
            const uint32_t now = plat_ms(), o = k == B_OCTDN ? B_OCTUP : B_OCTDN;
            if (k == B_OCTDN) { ui.shift_t = now; ui.shift_used = 0; }
            else { ui.up_t = now; ui.up_used = 0; }
            if ((btn >> o & 1u) && now - (k == B_OCTDN ? ui.up_t : ui.shift_t) < OC_PAIR_MS) {
                ui.oc_pair = 1;
                ui.shift_used = ui.up_used = 1;
            }
        } else if (k == B_PLAY && shift) {   /* SHIFT + PLAY (1.18C43): PLAY mode's editor and back; PLAY mode is
                                          * PLAY held (pm_toggle) */
            ui.shift_used = 1;
            overlay_off();
            if (ui.view == V_PLAY) played_go();
            else if (pm_on) play_go(0);
            else app_say("Play mode", "HOLD PLAY");
        } else if (k == B_PLAY) {        /* the sequencers' transport: those with a pattern start (at once), or all
                                          * stop (1.19.7: on the release of a short press, pm_rel: a hold, PLAY mode
                                          * on / off, leaves the transport running) */
            if (SQ_RUNS || dr.run) pm_rel = 1;
            else { seq_play(1); pm_rel = 4; }
        } else if (k == B_GLO) {         /* the global settings */
            overlay_off();
            glo_open();
        } else if (k == B_SEQ) {         /* the step sequencer (sq_*) */
            sq_button((int)shift);
        } else if (k == B_SAVE && (ui.view == V_SEQ || ui.view == V_DRUM || ui.view == V_KIT || ui.view == V_GLO)) {
            sv_page();                   /* (a sequencer's page: its patterns, asked; the kit's: the kit; GLO: its
                                          * settings, 0.67) */
        } else if (k == B_SAVE) {        /* the user banks' slots (sv_*); what else waits saved with it (0.65) */
            overlay_off();
            sv_all();
            sv_open();
        } else if (k == B_REC && (pk_rec() || sp_rec() || se_rec() || sq_rec() || dr_rec() || kit_rec())) {   /* (the sequencers' CLEAR, the kit's RESET) */
            ui.rec_t = 0;
        } else if (k == B_REC && sv_on()) {
            ui.rec_t = 0;
            sv_rec();
        } else if (k == B_REC && (overlay_off(), qa_rec())) {
            ui.rec_t = 0;                /* (the menus and lists: not Bluetooth) */
        } else if (k == B_REC && FM1B_BLE) {
            ui.rec_t = plat_ms();        /* on release: a press HOLD, a hold (1 s) switches Bluetooth */
        } else if (k == B_REC && ui.view == V_DRUM) {   /* (the drums' page: RECORD) */
            dr_arm();
        } else if (k == B_REC && ui.view == V_SEQ) {    /* (the arp sequencer's page: RECORD) */
            sq_arm();
        } else if (k == B_REC) {         /* REC with no other meaning here: HOLD */
            hold_toggle(keys);
        }
    }
    {   /* (1.18C43) PLAY held PM_HOLD_MS (not with OCT-): PLAY mode on / off. The press starts a stopped transport at
         * once, as a tap does (no delay for a tap); a start it made is taken back at the hold. 1.19.7: a running
         * transport stops on the release of a short press (pm_rel), so a hold leaves it running (1.18C43 .. 1.19.6:
         * the press stopped it, a hold ended stopped whichever way it began) */
        const uint32_t now = plat_ms();
        if (!(btn >> B_PLAY & 1u)) {
            if ((pm_t & 1u) && pm_rel == 1u) seq_play(0);   /* (1.19.7: a short press's stop, on its release) */
            if ((pm_t & 1u) && pm_rel == 2u) ln_play();
            pm_t = pm_rel = 0;
        }
        else if (!pm_t) pm_t = now | 1u;
        else if ((pm_t & 1u) && (int32_t)(now - pm_t) >= (int32_t)PM_HOLD_MS && !(btn >> B_OCTDN & 1u)) {   /* (signed,
                                          * 1.18C46: pm_t is now | 1, one ms ahead of an even now; the next pass in
                                          * the same millisecond saw 0xFFFFFFFF, a tap turned PLAY mode on) */
            pm_t = 2;
            if ((pm_rel & 4u) && (SQ_RUNS || dr.run)) seq_play(0);   /* (1.19.7: only a start the press made is taken
                                          * back; a transport running before the press runs on) */
            pm_toggle();
        }
        if (btn >> B_OCTDN & 1u && pm_t & 1u) pm_t = 2;   /* (SHIFT + PLAY: no hold) */
        pm_home(now);
    }
    if (!(btn >> B_OCTDN & 1u) && (ui.btn >> B_OCTDN & 1u) && !ui.shift_used
        && plat_ms() - ui.shift_t < SHIFT_HOLD_MS) {   /* OCT- tapped: the octave down (the name editor: the cursor) */
        if (ui.view == V_NAME) nm_move(-1);
        else if (ui.view == V_SEQ || ui.view == V_DRUM) seq_blk(-1);   /* (a sequencer's page: the block shown) */
        else oct_tap(-1);
    }
    if (!(btn >> B_OCTUP & 1u) && (ui.btn >> B_OCTUP & 1u) && !ui.up_used) {   /* OCT+ let go: the octave up (the
                                          * name editor: the cursor) */
        if (ui.view == V_NAME) nm_move(1);
        else if (ui.view == V_SEQ || ui.view == V_DRUM) seq_blk(1);
        else oct_tap(1);
    }
#ifdef HYB_MULTI
    if (home_tap && !(btn >> B_HOME & 1u)) {   /* (HOME let go: a tap toggles the part edited) */
        if ((int32_t)(plat_ms() - home_tap) < (int32_t)HOME_TAP_MS && ui.view == V_QUICK && !ui.qpos) part_edit(!ed_part);
                                          /* (signed: home_tap is now | 1, 1.18C46) */
        home_tap = 0;
    }
#endif
    if (ui.oc_pair) {                    /* the pair: both held OC_HOLD_MS from the second's press, PANIC (once); both
                                          * up, over */
        const uint32_t both = (btn >> B_OCTDN & 1u) && (btn >> B_OCTUP & 1u);
        if (both && ui.oc_pair == 1u
            && plat_ms() - (ui.shift_t - ui.up_t < 0x80000000u ? ui.shift_t : ui.up_t) >= OC_HOLD_MS) {
            ui.oc_pair = 2;
            quick_panic();
        } else if (!(btn >> B_OCTDN & 1u) && !(btn >> B_OCTUP & 1u))
            ui.oc_pair = 0;
    }
    if (FM1B_BLE && ui.rec_t) {
        if (btn >> B_REC & 1u) {
            if (plat_ms() - ui.rec_t >= 1000u) {
                ui.rec_t = 0;
                ble_toggle();
            }
        } else {                         /* (a press: HOLD; looking for a device: GLO > BLUETOOTH > SEARCH) */
            ui.rec_t = 0;
            if (ui.view == V_DRUM) dr_arm(); else if (ui.view == V_SEQ) sq_arm(); else hold_toggle(ui.keys);
        }
    }
    if ((e = plat_enc(EN_PRESET)) != 0 && ui.view == V_PLAY && shift) {
        play_turn(5u, e);                /* (PLAY mode: SHIFT + PRESET its control, the patch stays) */
    } else if (e != 0 && ui.view == V_PLAY) {
        qa_go(0);                        /* (PLAY mode, 1.18C43: the patch as on HOME, its list over HOME's screen;
                                          * then the page again, pm_home) */
        ps_turn(e);
#ifdef HYB_MULTI
    } else if (e != 0 && ui.view == V_LNCH && !lset.on) {
        ln_preset(e);                    /* (the launcher: the part edited's patch, no HOME) */
#endif
    } else if (e != 0 && (ui.view != V_QUICK || ui.qpos != 0u)) {
        app_say("Preset", "HOME ONLY");  /* (in the menus the patch stays: HOME first) */
    } else if (e != 0)
        ps_turn(e);                      /* (the patch list; its first detent shows it alone) */
#ifdef HYB_MULTI
    const int lnk = ui.view == V_LNCH && !(btn >> B_OCTDN & 1u);   /* (1.18C21: the launcher's SELECT and K1..K4 are ln_pre's alone:
                                          * a detent come in after ln_pre's read waits for the next one, not taken here
                                          * as HOME's SELECT (lost, the page drawn again: the flicker)) */
#else
    const int lnk = 0;
#endif
    if ((e = lnk ? 0 : plat_enc(EN_SELECT)) != 0 && ui.view == V_PLAY) {
        play_turn(4u, e);
    } else if (e != 0 && fv.ed) {
        fe_turn(e);                      /* (1.18C33: the favorites editor: as PRESET) */
    } else if (e != 0 && ui.view == V_PLAYED) {
        played_select(e);
    } else if (e != 0 && sv_on()) {
        sv_select(e);
    } else if (e != 0 && se_on()) {
        se_select(e);
    } else if (e != 0 && ui.view == V_SEQ) {
        sq_knob(4u, e);
    } else if (e != 0 && ui.view == V_DRUM) {
        dr_knob(4u, e);
    } else if (e != 0 && ui.view == V_KIT) {
        kit_knob(4u, e);
    } else if (e != 0) {
        overlay_off();
        qa_select(e);
    }
    if ((e = plat_enc(EN_ALGO)) != 0) {
        if (FM1B_BLE_CENTRAL && ui.looking) {
            btdev_choose(btdev_pick() + e);
            ble_device_say();
        } else if (ui.view == V_PLAY)    /* (PLAY mode: its control) */
            play_turn(6u, e);
#ifdef HYB_MULTI
        else if (shift)                  /* SHIFT + ALGO: the part edited's volume */
            pv_turn(e);
#endif
        else {                           /* the tempo (B 16) */
            param_turn(&U_ROW[ROW_GROOVE].p[3], 4u, e, row_on() == (int32_t)ROW_GROOVE ? 3 : -1);
            if (ui.view == V_SEQ || ui.view == V_DRUM || ui.view == V_KIT) ui.d_rows = 0;   /* (the sequencers' pages: the strip
                                                     * alone) */
            if (ui.view == V_SEQ || ui.view == V_DRUM) {   /* (there: the tempo up as the pages' own parameters, sq_title) */
                ui.edit_t = 0;
                sq_title(SQ_TT_TEMPO);
            }
        }
    }
    for (k = 0; k < 4u; k++) {
        int32_t row;
        if (lnk || (e = plat_enc(EN_K1 + (int)k)) == 0)
            continue;
        if (fv.ed) {                     /* (1.18C33: the favorites editor: K1 moves a favorite) */
            if (k == 0u) fe_k1(e);
            continue;
        }
        if (k == 0u && glo_choosing()) { /* (a GLO value's list: K1 as SELECT) */
            overlay_off();
            glo_select(e);
            continue;
        }
        if (ui.view == V_PLAY) {         /* PLAY mode: its controls */
            play_turn(k, e);
            continue;
        }
        if (ui.view == V_PLAYED) {       /* the editor: K1 as SELECT */
            if (k == 0u) played_select(e);
            continue;
        }
        if (sv_on()) {                   /* SAVE: K1 a bank; the name: K1 the cursor, K2 the case */
            sv_knob(k, e);
            continue;
        }
        if (se_on()) continue;           /* (a sequencer's EDIT list up: K1..K4 nothing) */
        if (ui.view == V_SEQ) {          /* the sequencer's page: the step held */
            sq_knob(k, e);
            continue;
        }
        if (ui.view == V_DRUM) {         /* the drums' page */
            dr_knob(k, e);
            continue;
        }
        if (ui.view == V_KIT) {          /* the drum kit's */
            kit_knob(k, e);
            continue;
        }
        if ((row = row_on()) < 0 || !U_ROW[row].p[k].label[0])
            continue;                    /* (HOME, the lists, EDIT and BACK: the knobs do nothing) */
        param_turn(&U_ROW[row].p[k], k, e, (int32_t)k);
    }
    if (keys != ui.keys || btn != ui.btn) st_busy_t = plat_ms();   /* (a key or a button: busy, store_wait) */
    ui.keys = keys;
    ui.btn = btn;
    if ((SQ_RUNS || ui.view == V_SEQ) && !fv_aud) sq_keys_put(keys);   /* (1.18C33: the favorites editor's keys:
                                          * its audition's alone) */
    if (play_on() && set.patch != ui.ppatch) {   /* PLAY mode: another patch (undo): the smart choice again */
        play_smart();
        pv_fill();
        ui.d_rows = ui.d_lcd = 1;
    }
    {   /* the lights: the menu open (GLO too); HOME where it goes up a level (or SHIFT + HOME: undo), REC where it
         * acts; SAVE while the patch has edits not saved, not in the menus (and while its slot list or name editor is open); PLAY red in PLAY mode (lit), else green HOLD. Set only when they change (and once a
         * second): setting them runs the LED driver from the flash, which pushes the audio's code out of the cache */
        static uint32_t last_b = ~0u, last_k, last_t, last_r, last_m;
        static uint32_t last_d PLAT_POOL, last_mb PLAT_POOL;   /* (the pool: RAM is full in the Bluetooth build) */
        static uint32_t lg_ms = ~0u, lg_k, lg_b;   /* (P8: the lights worked out at most once a millisecond, and at once
                                          * when a key or a button changed: input runs every pass of the main loop and
                                          * the lights' code (sq_lights, the launcher's, qa_count, ...) is long; a change
                                          * of the state itself shows at most 1 ms later) */
        uint32_t lb, lk, mk = 0, dk = 0, now = plat_ms();
        if (now != lg_ms || keys != lg_k || btn != lg_b) {
        lg_ms = now; lg_k = keys; lg_b = btn;
        const uint32_t v = ui.view, nav = v != V_PLAY && (v != V_QUICK || ui.qpos != 0u),
                       inmenu = v == V_MENU || v == V_GLIST || v == V_GROUP, run = SQ_RUNS || dr.run,
                       red = pm_on && !run;   /* (1.18C43: PLAY mode on, the transport stopped: PLAY red; running:
                                          * green on each beat, sq_beat) */
        const int32_t it = v == V_MENU || v == V_GROUP ? rl_item(ui.pos) : RL_NONE;
        shift = (btn >> B_OCTDN & 1u) && plat_ms() - ui.shift_t >= SHIFT_HOLD_MS && !ui.oc_pair;
        lb = btn | (inmenu ? 1u << U_MENU[ui.menu].btn : v == V_GLO || v == V_KIT ? 1u << B_GLO : 0u)
                  | (shift || v == V_LIST || v == V_GLIST || v == V_GROUP || v == V_GLO || v == V_PLAYED || v == V_SAVE || v == V_NAME
                     || v == V_KIT ? 1u << B_HOME : 0u)
                  | (v == V_LIST || v == V_GLIST || it == RL_EDIT || it == RL_BACK || (v == V_GLO && !gd_idle()) || v == V_PLAYED
                     || (v == V_KIT && kv.ask)
                     || (v == V_QUICK && ui.qpos == qa_count() + 1u) || v == V_SAVE || v == V_NAME || (v == V_DRUM ? sqv.rec : v == V_SEQ ? sqv.arec : hold_on)
                     ? 1u << B_REC : 0u)
                  | (((em_edited(set.patch) || sx_edit) && !nav) || v == V_SAVE || v == V_NAME
                     || (!(set.md_pad & MD_AUTO) && sv_lit()) ? 1u << B_SAVE : 0u) | (shift || (pm_on ? !run || sq_beat() : run) ? 1u << B_PLAY : 0u)
                  | (v == V_SEQ || v == V_DRUM ? 1u << B_EDIT : 0u)   /* (their EDIT list) */
                  | (v == V_SEQ || v == V_DRUM ? sq_blk_lights(v == V_DRUM)
                     : oct_lights());   /* (the sequencers' pages: the blocks left / right to go to) */
        lb |= arp_beat(now);             /* (the arpeggiator on: ARP lit on each beat) */
        if (fv.ed || (fv.sec && ui.plist_until)) lb |= 1u << B_REC | (fv.ed ? 1u << B_HOME : 0u);   /* (1.18C33: the
                                          * favorites: REC acts, HOME leaves the editor) */
        lk = shift ? 1u << (shadow[94] ? KEY_POLY : KEY_MONO) : sq_lights(KEYS_DARK ? 0u : ~0u, &lb, &dk, &mk);   /* (the key
                                                     * LEDs always on, the user's choice; SHIFT: only the key of the mode
                                                     * not on, MONO or POLY; the sequencers' pages: dim and middle keys) */
        if (shift) dk = mk = 0;
#ifdef HYB_MULTI
        if (shift && set.mx_mode != SY_M_SINGLE) lk |= 1u << (ed_part ? KEY_EDA : KEY_EDB);   /* (the other part's key) */
#endif
        if (v == V_KIT && !shift) lk = kit_lights(&mk);   /* (the drum kit: the other black keys at the middle level) */
        if (v == V_KIT && !shift) dk = 0;
        /* the middle level: the unlit buttons (0.63), ARP only while the part edited's patch has its arpeggiator on
         * (its byte; also while a sequencer holds it off, "SEQ"): off, it glows */
        const uint32_t mb = ((1u << NB) - 1u) & ~(1u << B_PLAY) & ~(shadow[129] ? 0u : 1u << B_ARP);
        const int lt = lt_frame(now, keys | btn | stat_voices | dr.act | dr.run | (uint32_t)SQ_RUNS, red, lb, lk, dk, mk,
                                mb);   /* (1.18C43: the power-on show, NIGHT; 0 neither) */
        if (lt <= 0 && (lt < 0 || lb != last_b || lk != last_k || red != last_r || mk != last_m || dk != last_d
                        || mb != last_mb || now - last_t >= 1000u)) {
            if (last_b == ~0u) plat_glow(1);
            plat_play_red((int)red);     /* (PLAY red: PLAY mode; green: HOLD) */
            if (!dk) plat_leds(lb, lk);
            else {                       /* (plat_leds with dim keys: the unlit buttons glow, PLAY green unless red) */
                uint32_t b = lb, d = ((1u << NB) - 1u) & ~lb;
                const uint32_t pl = 1u << B_PLAY;
                if (!red) { b = (b & ~pl) | (lb & pl ? LED_PLAY_GREEN : 0u); d = (d & ~pl) | (d & pl ? LED_PLAY_GREEN : 0u); }
                plat_lights(b, lk, d, dk & ~lk, 2);
            }
            plat_lights_mid(mb & ~lb, mk);   /* (after the lit ones: the middle level:
                                                     * the unlit buttons, brighter than the glow (0.63: the user's
                                                     * wish; PLAY, two-coloured, left out), and the keys at it) */
            last_b = lb; last_k = lk; last_r = red; last_t = now; last_m = mk; last_d = dk; last_mb = mb;
        }
        }
    }
}

/* (1.18C37) NS_WARM: the engine's note-start code (synth.c SY_NSL, ~3.5 KB of the flash: alloc, unison, voice_start,
 * the first block's set-up) read through by the main loop every NS_WARM_MS, a byte every 16, so that its lines are in
 * the flash's cache when a chord comes (the device's benchmark: a chord's first block over 100 %, mostly that code
 * run cold; the cache serves data reads and instruction fetches alike, as the benchmark's evict probe assumes and its
 * rdwarm probes check). Reads only: nothing else changes (the sound bit-identical). The build gives the bounds
 * (build_variant.py --ns-group); without them nothing is read */
#ifndef NS_WARM
#define NS_WARM 1
#endif
#ifndef NS_WARM_MS
#define NS_WARM_MS 2u
#endif
#if NS_WARM && !defined(PLAT_HOST)
extern const uint8_t _sy_ns_start[] __attribute__((weak)), _sy_ns_end[] __attribute__((weak));
static volatile uint32_t ns_sink;
static __attribute__((noinline)) void ns_pass(void)   /* (one pass; the measurement build times it too) */
{
    const uint8_t *p = _sy_ns_start, *e = _sy_ns_end;   /* (plain reads: their sum goes to a volatile, none is left
                                          * out) */
    uint32_t s = 0;
    if (!p || e <= p)
        return;
    for (; p < e; p += 16)
        s += *p;
    ns_sink = s;
}
static void ns_warm(void)
{
    static uint32_t t;
    const uint32_t now = plat_ms();
    if (now - t < NS_WARM_MS)
        return;
    t = now;
    ns_pass();
}
#else
#define ns_warm() ((void)0)
#endif

void app_input_only(void)                /* (HOLD's restarts: a block after their note-offs) */
{
    ns_warm();
#ifdef HYB_MULTI
    ln_pre();                            /* (the launcher's buttons and knobs first, as app_frame) */
#endif
    input();
    hold_zone_fix();                     /* (1.16: SPLIT, a key let go latched by the part edited's HOLD, its zone's
                                          * part's off: ended) */
    key_rs_run();
}

static int32_t master_db(void)           /* MASTER as the render's gain (its square), in dB */
{
    uint32_t m = plat_master();
    if (m < 8u)
        return -99;
    return (int32_t)(12.0412f * fm_log2f((float)m * (1.f / 4096.f)) - 0.5f);
}

static uint32_t lcd_mode(uint32_t now)   /* 0 a message, 1 the parameter, 2 the patch (or the list's line), 3 the
                                          * block */
{
    if (now < ui.msg_until) return 0u;
    if (ui.plist_until) return 2u;       /* (the patch list: the patch) */
    if (ui.lastp && ui.edit_t && (int32_t)(now - ui.edit_t) < (int32_t)LCD_EDIT_MS) return 1u;   /* (signed: edit_t is
                                          * now | 1, 1.18C46) */
    if (row_on() < 0) return 2u;         /* (HOME and the lists: no block) */
    return 3u;                           /* (a block: its name; the patch only on HOME) */
}

/* the HOME screen's parts, each drawn again only when its figures change (sound before the GUI): the tiles' first
 * line (the load, the voices) at most every HOME_FIG_MS, less often under load and after an xrun; their second line
 * (what the player changes) every 250 ms; the MIDI lights alone (their band only) every HOME_LIGHT_MS; the rest of
 * the connections when they change */
#define HOME_FIG_MS 1000u
#define HOME_LIGHT_MS 250u
static uint32_t fnv(const void *p, uint32_t n)
{
    const uint8_t *b = p;
    uint32_t h = 2166136261u;
    while (n--)
        h = (h ^ *b++) * 16777619u;
    return h | 1u;                       /* (0: draw it) */
}
static uint32_t gui_gap(uint32_t base)   /* a figure's interval: longer under load, longest after an xrun */
{
    static uint32_t xr, xr_t;
    uint32_t now = plat_ms(), pct = plat_cpu_pct(), x = plat_xruns();
    if (x != xr) { xr = x; if (!store_recent()) xr_t = now | 1u; }   /* (a flash write's: no dropout) */
    if (xr_t && (int32_t)(now - xr_t) < 5000) return base * 5u;   /* (signed: xr_t is now | 1, 1.18C46) */
    return pct >= 85u ? base * 3u : pct >= 70u ? base * 2u : base;
}
static void draw_home(uint32_t now)
{
    static uint32_t t_fig, t_row2, t_light, sig_rest;
    uint32_t p, sig, full = !home_sig[0];
    uint8_t a[6];
    int t;
    if (full || now - t_light >= gui_gap(HOME_LIGHT_MS)) {
        uint8_t act = 0;
        for (p = 0; p < NPORTS; p++)     /* (lit: a message since the last look) */
            if (plat_midi_ago((int)p, 0) < now - t_light + 20u)
                act |= (uint8_t)(1u << p);
        t_light = now;
        if (act != hm.act && !full && home_sig[1]) {   /* (the lights alone: their band) */
            hm.act = act;
            draw_body_part(UI_ROW0_Y + UI_HOME_A, UI_BODY_H - UI_HOME_A, 0, 20, home_conn);
        }
        hm.act = act;
    }
    hm.batt = (int8_t)plat_battery();    /* (the bar: with the connections) */
    hm.usb = (uint8_t)(hm.batt == 4 || plat_midi_out_room(TO_USB) != 0u);
    hm.bt_off = set.ble_off;
    hm.bt = (uint8_t)plat_ble_state();
    hm.bt_host = (uint8_t)(plat_ble_connected() ? 2 : plat_ble_linked() ? 1 : 0);
    hm.bt_dev = (uint8_t)plat_ble_device();
    hm.dev[0] = 0;
    if (hm.bt_dev == BLE_DEV_READY && (!plat_ble_device_info(a, &t, hm.dev) || !hm.dev[0]))
        text_cat(hm.dev, set.dev_ok ? set.dev_name : "CONNECTED");   /* (connected by address: the saved name) */
    hm.dev[20] = 0;
#ifdef HYB_MULTI
    {   /* the parts: their patches, the mode, the part edited (the rows instead of Bluetooth's) */
        hm.mx_mode = set.mx_mode;
        hm.mx_ed = ed_part;
        hm.mx_ch = set.mo & MO_BOMNI ? 17u : set.mx_chb;   /* (1.19.4: B IN OMNI, 17) */
        hm.mx_a[0] = hm.mx_b[0] = 0;
        patch_label(hm.mx_a, ed_part ? patch_o : set.patch, ed_part ? shadow_o : shadow);
        patch_label(hm.mx_b, ed_part ? set.patch : patch_o, ed_part ? shadow : shadow_o);
    }
#endif
    sig = fnv(&hm.usb, 1u) ^ fnv(&hm.batt, 1u) ^ fnv(&hm.bt_off, (uint32_t)(sizeof hm - __builtin_offsetof(u_home_t, bt_off)));   /* (not
                                                     * the lights) */
    if (!home_sig[1] || sig != sig_rest) {
        home_sig[1] = 1u;
        sig_rest = sig;
        draw_body(UI_ROW0_Y + UI_HOME_A, UI_BODY_H - UI_HOME_A, home_conn);
    }
    if (full || now - t_fig >= gui_gap(HOME_FIG_MS) || now - t_row2 >= 250u) {
        uint32_t s0, s1;
        static uint32_t last0, last1;
        int fig = full || now - t_fig >= gui_gap(HOME_FIG_MS);
        t_row2 = now;
        if (fig) {
            t_fig = now;
            static uint32_t xr, xr_t;
            hm.cpu = (uint8_t)plat_cpu_pct();
            hm.pk = (uint8_t)(blk_peak > 255u ? 255u : blk_peak);
            blk_peak = 0;
            if (plat_xruns() != xr) { xr = plat_xruns(); if (!store_recent()) xr_t = now | 1u; }   /* (a flash
                                                     * write's: no dropout) */
            hm.xr = (uint8_t)(xr_t && (int32_t)(now - xr_t) < 5000);   /* (a dropout in the last 5 s: the peak red) */
            hm.voices = (uint8_t)(set.eng ? set.eng : stat_voices);   /* (ENGINE without the synth: its mode, maxv 0) */
            hm.maxv = (uint8_t)(set.eng ? 0 : GOV_MAX);
#ifdef HYB_MULTI
            {                            /* (two parts: A's and B's of the same block, the audio interrupt's) */
                const uint32_t vab = stat_vab;
                hm.vb = (uint8_t)(set.mx_mode != SY_M_SINGLE && !set.eng ? 1u + (vab >> 8 & 0xFFu) : 0u);
                if (hm.vb && !set.eng) hm.voices = (uint8_t)(vab & 0xFFu);
            }
#endif
        }
        hm.lat = set.lat;
        hm.bpm = (uint8_t)((tp_run ? tp_run - 1u : shadow[TP_BYTE]) + 63u);   /* (the tempo running: tp_run) */
#ifdef HYB_MULTI
        hm.hold = (uint8_t)(hold_on | (set.mx_mode >= SY_M_SPLIT ? hold_o << 1 : 0u));   /* (1.15: the other part's
                                          * HOLD too, HOME's tile; 1.16: SPLIT and CHANNELS, LAYER's one HOLD) */
#else
        hm.hold = hold_on;
#endif
        hm.oct = set.oct;
        hm.db = (int8_t)master_db();
        hm.tr = set.tr;
        s0 = fnv(&hm.cpu, 4u) ^ fnv(&hm.bpm, 1u) ^ fnv(&hm.pk, 2u);   /* (the first line: CPU, voices, latency, BPM) */
#ifdef HYB_MULTI
        hm.cap = (uint8_t)voice_cap();
        s0 ^= fnv(&hm.vb, 2u);
#endif
        s1 = fnv(&hm.hold, 1u) ^ fnv(&hm.oct, 2u) ^ fnv(&hm.tr, 1u);   /* (the second: HOLD, octave, volume, transpose) */
        if (full) {
            draw_body(UI_ROW0_Y, UI_HOME_A, home_tiles);
        } else {
            if (s0 != last0) draw_body_part(UI_ROW0_Y, UI_HOME_A, 0, 36, home_tiles);
            if (s1 != last1) draw_body_part(UI_ROW0_Y, UI_HOME_A, 38, UI_HOME_A, home_tiles);
        }
        last0 = s0;
        last1 = s1;
        home_sig[0] = 1u;
    }
}

/* the list on the screen now, if any: ls (ui.c) set to it. The patch list or a parameter's values over everything
 * while they are up, else the menus' lists */
static int list_up(void)
{
    if (ui.plist_until)
        pl_set();
    else if (ui.view == V_SAVE) {
        ls_set(sv_item, UB_N, ui.scur);
        ls_foot = sv_foot;
    }
    else if (ui.vl_until && p_list(ui.vl_p)) {
        uint32_t n = p_list(ui.vl_p), v = shadow[p_byte(ui.vl_p)];
#ifdef HYB_SHIMMER
        if (P_RT(ui.vl_p)) v = rt_idx(ui.vl_p, v);
#endif
        ls_set(vl_item, n, v < n ? v : n - 1u);
    } else if (ui.view == V_PLAYED && ui.pch)
        ls_set(pc_item, pl_n + 2u, ui.psel);
    else if (ui.view == V_PLAYED)
        ls_set(pe_item, PC_N + 1u, ui.pcur);
    else if (ui.view == V_LIST && ul.choosing)
        ls_set(ch_item, U_NBLK + 1u, (uint32_t)(ul.ch + 1));
    else if (ui.view == V_LIST)
        ls_set(qa_item, QA_SLOTS + 1u, ul.cur);
    else if (ui.view == V_GLIST)
        ls_set(ui.menu == M_MTX ? mxl_item : gl_item, U_MENU[ui.menu].ng + 1u, ui.gcur);
    else if (se_on() && sqv.cp)          /* (a sequencer's EDIT > COPY TO) */
        ls_set(cp_item, cp_n(), sqv.cp - 1u);
    else if (se_on())                    /* (a sequencer's EDIT list) */
        ls_set(se_item, se_n(), sqv.menu - 1u);
    else if (ui.view == V_GLO && ui.gch == G_SEARCH) {   /* (the cursor on the device ALGO picks; within the list) */
        if (ui.gpos < gd_n() && !btdev_busy() && btdev_count()) ui.gpos = (uint8_t)btdev_pick();
        if (ui.gpos > gd_n()) ui.gpos = (uint8_t)gd_n();
        ls_set(gd_item, gd_n() + 1u, ui.gpos);
    } else if (ui.view == V_GLO && ui.gch == G_CC)
        ls_set(cc_item, cc_n() + 2u, ui.gpos);
    else if (ui.view == V_GLO && ui.gch)
        ls_set(gv_item, gv_n(ui.gch), gv_get(ui.gch));
    else if (ui.view == V_GLO)
        ls_set(glo_item, GLO_PN[ui.gpage], ui.gpos);
    else {
        ls.item = 0;
        return 0;
    }
    return 1;
}

/* the LFO lights: a filled circle by an LFO's RATE knob, lit in the first half of the LFO's cycle (the part's free
 * running LFO, S.glfo: it runs with the rate and the clock, notes or not). Only the circle is drawn, and only when
 * it changes; looked at every LED_MS at most, less often under load (gui_gap) */
#define LED_MS 30u
static void led_place(uint32_t slot, int32_t it, int on, int32_t y)   /* the row drawn at slot: its light, if any */
{
    uint32_t k;
    led[slot].lfo = -1;
    if (it < 0)
        return;
    for (k = 0; k < 4u; k++) {
        uint32_t b = U_ROW[it].p[k].byte;
        if (U_ROW[it].p[k].label[0] && (b == 67u || b == 79u || b == 128u + 7u)) {
            led[slot].lfo = (int16_t)(b == 67u ? 0 : b == 79u ? 1 : 2);
            led[slot].x = (int16_t)(4 + UI_COL_X(k) + UI_COL_C + UI_RKNOB / 2 - 2);   /* (the knob's top right) */
            led[slot].y = (int16_t)(y + 2);
            led[slot].on = (uint8_t)on;
            led[slot].st = 2u;
            return;
        }
    }
}
static void led_draw(uint32_t slot, int lit)
{
    const uint16_t bg = led[slot].on ? mix565(U_BG, u_acc, 28u) : U_ROWOFF, c = lit ? U_TH.led : U_TRACK;
    int32_t i, j;
    cv_begin(9, 9, bg);
    for (j = 0; j < 9; j++)
        for (i = 0; i < 9; i++) {
            float dx = (float)i - 4.f, dy = (float)j - 4.f;
            uint32_t t = cover(4.3f - fm_sqrtf(dx * dx + dy * dy));
            if (t) cv_blend(i, j, c, t);
        }
    cv_blit((uint32_t)led[slot].x, (uint32_t)led[slot].y);
}
static void lfo_leds(uint32_t now)
{
    static uint32_t t;
    uint32_t k;
    if (now - t < gui_gap(LED_MS))
        return;
    t = now;
    for (k = 0; k < 2u; k++) {
        uint8_t st;
        if (led[k].lfo < 0)
            continue;
        st = (uint8_t)(S.glfo[led[k].lfo].ph < 0.5f);
        if (st != led[k].st) {
            led[k].st = st;
            led_draw(k, st);
        }
    }
}

/* a row's place: a row (lit: K1..K4 on it), EDIT or BACK, or nothing */
static void draw_place(int32_t it, int on, int32_t y, uint32_t slot)
{
    led_place(slot, it, on, y);
    if (on) {
        lit.it = (int16_t)it;
        lit.hl = (int8_t)ui.last;
        if (it >= 0) lit.big = (uint8_t)row_big((uint32_t)it, shadow);
    } else
        lit.dim = (int16_t)it;
    if (it >= 0) draw_row((uint32_t)it, on, shadow, y, slot, on ? ui.last : -1);
    else if (it == RL_EDIT || it == RL_BACK) draw_entry(it == RL_EDIT ? "EDIT" : "BACK", on, y);
    else lcd_fill(4, (uint32_t)y, 232, UI_ROW_H, U_BG);
}

/* FX's blocks follow the delay / reverb mode (ui.c fx_class), whatever changed it (its knob, a patch, MIDI): the
 * rows and the strip drawn again, the cursor on the same block if the new list has it (DLY/REV, where the mode
 * turns, always), else on the place nearest to it; on EDIT or BACK it stays there */
static void fx_follow(void)
{
    static uint8_t cls = 0xFFu, nb = 0u;
    uint32_t c = fx_class(shadow), n = menu_nb(M_FX, shadow), k;
    int32_t it;
    if (c == cls) return;
    if (ui.view == V_MENU && ui.menu == M_FX && cls != 0xFFu) {
        if (ui.pos >= nb)
            ui.pos = (uint8_t)(ui.pos + n - nb);
        else {
            for (k = 0; k < n && menu_blk(M_FX, k, shadow) != ui.sec; k++)
                ;
            ui.pos = (uint8_t)(k < n ? k : ui.pos < n ? ui.pos : n - 1u);
        }
        ui.mpos[M_FX] = ui.pos;
        it = rl_item(ui.pos);
        if (it >= 0) ui.sec = (uint32_t)it;
        ui.d_rows = ui.d_lcd = 1;
    }
    cls = (uint8_t)c;
    nb = (uint8_t)n;
}

static UI_LATE void ui_theme_follow(void)        /* the colours: the part edited's; SEQ B's page part B's (green), SEQ
                                          * A's part A's; the drums' pages (DRUM SEQ, DRUM KIT) amber (UI_LATE: the
                                          * flash's code before the engine's stays short) */
{
#ifdef HYB_MULTI
    uint32_t th = ui.view == V_DRUM || ui.view == V_KIT ? U_TH_DRUM : ui.view == V_SEQ ? (sq_mi && SQ_BMO ? U_TH_GREY : sq_mi)
                                                                                   : ed_part;   /* (1.19.3: SEQ B
                                          * in SINGLE, MIDI only, the greys) */
    if (ui.view != V_SEQ && th == ed_part && u_th == &U_THEMES[!ed_part] && set.mx_mode != SY_M_SINGLE) {   /* (0.83: the arp sequencer
                                          * of the other part left for another page: that part edited, its colours
                                          * kept; from the drums' pages the part edited before, as it was) */
        part_edit(!ed_part);
        return;
    }
#else
    uint32_t th = ui.view == V_DRUM || ui.view == V_KIT ? U_TH_DRUM : 0u;
#endif
    if (fv.ed) th = U_TH_GREY;           /* (1.18C33: the favorites editor) */
    if (u_th != &U_THEMES[th]) { u_theme(th); view_reset(); }
}
static void draw(void)
{
    uint32_t now = plat_ms();
#ifdef HYB_MULTI
    if (ui.view == V_LNCH) sq_retv = 1u;           /* (the launcher was the last sequencer view, for SEQ) */
    else if (ui.view == V_DRUM || ui.view == V_SEQ) sq_retv = 0u;
#endif
    fx_follow();
    ui_theme_follow();
    {   /* PLAY mode: the battery, a thin bar under the controls (0.63: the strip's icon went in 0.57), drawn again
         * only when its level changes; gone with PLAY mode */
        static int pb = -9;
        const int b = ui.view == V_PLAY ? plat_battery() : -9;
        if (b != pb) { pv_batt(b); pb = b; }
    }
    if (ui.d_lcd) {                      /* the LCD strip: a message; a parameter turned in the last LCD_EDIT_MS
                                         * (its name and value); else the patch and the block in turn; in the lists
                                         * the line under the cursor */
        char a[24], v[24];
        uint32_t m = lcd_mode(now);
        ui.d_lcd = 0;
        a[0] = v[0] = 0;
        if (m == 0) {
            text_cat(a, ui.msg[0]);
            text_cat(v, ui.msg[1]);
        } else if (pk_lcd(a, v)) {       /* (1.18C33: a pattern left with edits that fit nowhere: asked) */
        } else if (m == 1) {
            p_name(ui.lastp, a);
            strip_text(ui.lastp, shadow[p_byte(ui.lastp)], v);
        } else if (m == 3) {             /* the block: its name, its place in the list */
            char t[6];
            uint32_t pos = ui.view == V_QUICK ? ui.qpos : ui.pos + 1u,
                     n = ui.view == V_QUICK ? qa_count() : ui.view == V_GROUP ? U_GRP[ui.grp].nr : menu_nb(ui.menu, shadow);
            text_cat(a, U_ROW[ui.sec].tag);
            put_sint(v, (int32_t)pos, 0); text_cat(v, "/"); put_sint(t, (int32_t)n, 0); text_cat(v, t);
        } else if (ui.plist_until)
            pl_lcd(a, v);                /* (1.18C33: the favorites' hints) */
        else if (play_on())
            play_lcd(a, v);
        else if (sv_on())
            sv_lcd(a, v);
        else if (ui.view == V_SEQ || ui.view == V_DRUM)
            sq_lcd(a, v);
        else if (ui.view == V_KIT)
            kit_lcd(a, v);
#ifdef HYB_MULTI
        else if (ui.view == V_LNCH)
            ln_lcd(a, v);
#endif
        else if (ui.view == V_GLO)
            glo_lcd(a, v);
        else if (ui.view == V_LIST) {    /* the list: the slot under the cursor, or BACK */
            if (ul.cur >= QA_SLOTS) text_cat(a, "BACK");
            else {
                text_cat(a, "SLOT ");
                put_sint(a + 5, ul.cur + 1, 0);
                if (ul.choosing) text_cat(v, ul.ch < 0 ? "EMPTY" : U_ROW[ul.ch].tag);
            }
        } else if (ui.view == V_QUICK && ui.qpos > qa_count())   /* the EDIT entry */
            text_cat(a, "EDIT");
        else if (ui.view == V_GLIST) {   /* a menu's groups: the menu, the line under the cursor */
            text_cat(a, U_MENU[ui.menu].name);
            text_cat(v, ui.gcur < U_MENU[ui.menu].ng ? "EDIT" : "BACK");
        } else if (ui.view == V_MENU || ui.view == V_GROUP) {   /* on EDIT or BACK */
            text_cat(a, ui.view == V_GROUP ? U_GRP[ui.grp].name : U_MENU[ui.menu].name);
            text_cat(v, rl_item(ui.pos) == RL_EDIT ? "EDIT" : "BACK");
        } else
            patch_line(a);
#ifdef HYB_MULTI
        {   /* (the launcher's volume display: in the colour of the part turned, SYNTH 1 red, SYNTH 2 green, the drums amber) */
            const u_theme_t *keep = u_th, *lt = m == 0 && ln_vq ? &U_THEMES[ln_vq == 3u ? U_TH_DRUM : ln_vq - 1u] : keep;
            if (m) ln_vq = 0;
            if (lt != lcd_th) { lcd_th = lt; lcd_la[0] = 0; }   /* (the strip: all of it again in another colour) */
            u_th = lt;
            draw_lcd(a, v, -1);
            u_th = keep;
        }
#else
        draw_lcd(a, v, -1);              /* (no battery in the strip: its room for the text; HOME has the bar) */
#endif
    }
    ui.d_tabs = 0;                       /* (no tab line: the block's name shows in the LCD strip) */
    if ((ui.plist_until && now >= ui.plist_until) || (ui.vl_until && now >= ui.vl_until)) {   /* the list's time over:
                                                     * the view again */
        ui.plist_until = ui.vl_until = 0;
        ui.d_rows = 1;
    }
    if (list_up()) {
        if (ls.item == mxl_item) {       /* (0.92) the matrix's slots: drawn again when a light or a name changes (a
                                          * patch, MIDI, SysEx, undo while the list is up) */
            static uint32_t mxs;
            const uint32_t g = mx_sig();
            if (g != mxs) { mxs = g; ui.d_rows = 1; }
        }
        if (ui.d_rows) {
            ui.d_rows = 0;
            led[0].lfo = led[1].lfo = -1;
            draw_body(UI_ROW0_Y, UI_BODY_H, ls_body);
        }
        ls_tick(now, UI_ROW0_Y);
        return;
    }
    if (ui.view == V_NAME) {             /* SAVE's name editor: all of it at each change */
        if (ui.d_rows) {
            ui.d_rows = ui.d_col = 0;
            lit.it = lit.dim = -1;
            led[0].lfo = led[1].lfo = -1;
            sv_draw();
        }
        return;
    }
    if (ui.view == V_SEQ || ui.view == V_DRUM || ui.view == V_KIT) {   /* the sequencers' pages, the drum kit's: all
                                                     * of it, or the parts changed */
        const int all = ui.d_rows;
        if (ui.view == V_SEQ || ui.view == V_DRUM) seq_frame(); else kit_frame();
        if (all) {
            ui.d_rows = ui.d_col = 0;
            lit.it = lit.dim = -1;
            led[0].lfo = led[1].lfo = -1;
        }
        if (ui.view == V_SEQ || ui.view == V_DRUM) seq_draw(all); else kit_draw(all);
        return;
    }
#ifdef HYB_MULTI
    if (ui.view == V_LNCH) {             /* (1.18C10) the launcher: all of it, or the parts changed */
        const int all = ui.d_rows;
        if (all) {
            ui.d_rows = ui.d_col = 0;
            lit.it = lit.dim = -1;
            led[0].lfo = led[1].lfo = -1;
        }
        if (lset.on) ls_draw(all); else ln_draw(all);   /* (1.18C27: the sets' page) */
        return;
    }
#endif
    if (ui.view == V_PLAY) {             /* PLAY mode: all of it, or the control turned (and the one framed before) */
        if (ui.d_rows) {
            ui.d_rows = ui.d_col = 0;
            lit.it = lit.dim = -1;
            led[0].lfo = led[1].lfo = -1;
            pv_fill();
            pv.hl = (int8_t)ui.last;
            draw_play();
        } else if (ui.d_col) {
            const int8_t old = pv.hl;
            ui.d_col = 0;
            pv.hl = (int8_t)ui.last;
            if (old >= 0 && old != ui.last) draw_play_cell((uint32_t)old);
            if (ui.last >= 0) draw_play_cell((uint32_t)ui.last);
        }
        return;
    }
    if (ui.d_col && !ui.d_rows) {        /* a knob turned: its column (and the one framed before) */
        ui.d_col = 0;
        if (lit.it < 0 || row_big((uint32_t)lit.it, shadow) != lit.big)
            ui.d_rows = 1;
        else {
            if (lit.hl >= 0 && lit.hl != ui.last)
                draw_row_col((uint32_t)lit.it, 1, shadow, UI_ROW0_Y, lit.hl, ui.last);
            if (ui.last >= 0)
                draw_row_col((uint32_t)lit.it, 1, shadow, UI_ROW0_Y, ui.last, ui.last);
            lit.hl = (int8_t)ui.last;
            led[0].st = 2u;              /* (its light drawn again: the column may hold it) */
        }
    }
    if (ui.d_rows) {
        uint32_t n = qa_count();
        ui.d_rows = 0;
        ui.d_col = 0;
        lit.it = lit.dim = -1;           /* (the places drawn below say theirs) */
        led[0].lfo = led[1].lfo = -1;    /* (the rows drawn below say theirs) */
        if (ui.view != V_QUICK || qa_on_module()) lcd_fill(4, UI_ROW0_Y + UI_ROW_H, 232, UI_ROW1_Y - UI_ROW0_Y - UI_ROW_H, U_BG);
        if (ui.view == V_MENU || ui.view == V_GROUP) {   /* the lit place, the next below it (a matrix assign: its
                                                     * source and destination in full) */
            int32_t it = rl_item(ui.pos);
            draw_place(it, 1, UI_ROW0_Y, 0);
            if (ui.view == V_GROUP && ui.menu == M_MTX && it >= 0) {
                mx_row = (uint8_t)it;
                draw_body(UI_ROW1_Y, UI_ROW_H, mx_body);
            } else
                draw_place(rl_item(ui.pos + 1u), 0, UI_ROW1_Y, 1);
        } else if (qa_on_module()) {     /* a block of the list, lit; below it the next one, dim (or EDIT) */
            draw_place((int32_t)ui.sec, 1, UI_ROW0_Y, 0);
            draw_place(ui.qpos < n ? qa_nth(ui.qpos) : RL_EDIT, 0, UI_ROW1_Y, 1);
        } else if (ui.qpos)
            draw_body(UI_ROW0_Y, UI_BODY_H, edit_entry_body);
        else
            home_sig[0] = home_sig[1] = 0;   /* HOME: both its parts below */
    }
    lfo_leds(now);
    if (ui.view == V_QUICK && !ui.qpos)
        draw_home(now);
}

/* ------------------------------------------------------ SysEx: patches in --- */
/* the development notes, tools/bank_upload (the page that sends them). USB only: fm1-base hands the app each SysEx frame
 * from USB that it does not take itself (APP_SYSEX: the main loop, never the audio interrupt; Bluetooth MIDI and the
 * TRS jack drop SysEx). Two kinds are the app's, every other frame is left to the platform:
 *
 * the single dump, as the reference sends and takes it. ASSUMPTION (nothing in this repository fixes it: the lab
 * loads singles as raw 256 bytes; this is the commonly published layout of the reference's single dump):
 *   F0 00 20 33 01 dd 10 bb pp <256 bytes: page A 0..127, page B 128..255> cs F7
 * dd the device id (any), 10 the single dump, bb the bank (any), pp the program (0..127), cs the checksum: the sum
 * of dd .. the last data byte, & 7F (a sum taken from 00 instead is accepted too). Anything else that starts
 * 00 20 33 01 xx 10 (a wrong length, a bad checksum) is answered with an error and dropped.
 *
 * the page's handshake, its own frames: F0 7D 42 55 cmd args F7 (7D: the non-commercial id; "BU": bank upload;
 * fm1-base's own 7D frames use 53 52 and 4C xx)
 *   page -> FM-1: 01 hello; 02 bank n_hi n_lo: bank C .. H (0..5; C, D, E 0..2 until 1.18C29, hello's version 2)
 *                 armed for n patches; 03 the end; 04 cancel;
 *                 05 bank slot: a patch out (1.05, patchx.c: its single, its extension frame, 53 bank slot status flags);
 *                 06 slot: armed, the slot emptied (the page's patch manager, hello version 4: patchx.c, answered 52);
 *                 07 bank: the bank's names (version 4: patchx.c, answered 54 four times)
 *   FM-1 -> page: 41 4 (hello: the version; 1 until 1.04, without 05 and the extension frame F0 7D 49 58; 2 until
 *                 1.18C29, three user banks; 3 without 06 and 07); 42 bank status (armed: 0); 43 bank saved_hi saved_lo status (the end:
 *                 written; status 0 ok, 3 the flash failed, 2 nothing came for BU_IDLE_MS; bank 7F: none was armed); 50 pp status where
 *                 (each single, once it is in: status 0 ok, 1 checksum, 2 length, 3 flash; where 0 the edit buffer,
 *                 1..6 bank C..H). The page sends the next single only after this answer (the platform holds one
 *                 SysEx frame at a time: one arriving before the last was taken would be lost)
 * Armed, a single goes into the bank's slot pp (the page numbers them: as they were, or on from a slot), staged a
 * store sector at a time (bk_stage: one flash erase per 15 patches instead of one per patch; the store stages a page
 * at a time, no RAM here); the end (or BU_IDLE_MS without one) writes the last object. Not armed, a single
 * replaces the edit buffer: it sounds at once, the strip says "SysEx NAME", SAVE stores it */
#define SX_DUMP 265u                     /* a single dump's bytes between F0 and F7 */
#define BU_IDLE_MS 10000u
static struct { uint8_t on, bank, err; uint16_t n, got, saved; uint32_t t; } bu;   /* the armed upload (not armed, n
                                          * holds the edit buffer's single's pp + 1 while it was the last frame, 0
                                          * else: patchx.c's; no RAM of its own) */
#ifdef HYB_SHIMMER
static uint8_t bu_x[UB_SLOTS / 8u] PLAT_POOL;   /* the slots the upload wrote (bit s): their extensions off at its end */
static UI_LATE void bu_ext_off(void)     /* the end: the slots written have no extension (a dump carries none; a slot
                                          * whose extension frame came has its bit cleared: patchx.c px_ext) */
{
    uint8_t x[SHX_REC];
    uint32_t s;
    int st = 0;
    for (s = 0; s < UB_SLOTS; s++)
        if (bu_x[s >> 3] >> (s & 7u) & 1u) {
            const uint32_t u = bu.bank * UB_SLOTS + s;
            if (shx_none(u, x) && ub_put(shx_obj(u), shx_off(u), x, SHX_REC) != 0) st = 1;
        }
    if (ub_flush() != 0) st = 1;         /* (all in the bank's ninth sector: one write, if any) */
    if (st) bu.err = 1;
    shx_cu = 0;
}
#endif

static UI_LATE void bu_send(uint32_t cmd, uint32_t a, uint32_t b, uint32_t c, uint32_t n)   /* F0 7D 42 55 cmd
                                                  * (n of a, b, c) F7 */
{
    uint8_t f[9];
    f[0] = 0xF0; f[1] = 0x7D; f[2] = 0x42; f[3] = 0x55; f[4] = (uint8_t)cmd;
    f[5] = (uint8_t)(a & 0x7Fu); f[6] = (uint8_t)(b & 0x7Fu); f[7] = (uint8_t)(c & 0x7Fu);
    f[5u + n] = 0xF7;
    plat_sysex_send(f, 6u + n);
}
static UI_LATE void bu_num(char *s, uint32_t v)  /* a decimal number appended */
{
    char t[8];
    int i = 0;
    do { t[i++] = (char)('0' + v % 10u); v /= 10u; } while (v);
    while (*s) s++;
    while (i) *s++ = t[--i];
    *s = 0;
}
static UI_LATE void bu_show(const char *b, uint32_t ms)   /* the strip: "Bank C" and b (or the count), not read out */
{
    ui.msg[0][0] = ui.msg[1][0] = 0;
    text_cat(ui.msg[0], "Bank C");
    ui.msg[0][5] = (char)(UB_L0 + bu.bank);
    if (b) text_cat(ui.msg[1], b);
    else {
        bu_num(ui.msg[1], bu.got);
        text_cat(ui.msg[1], "/");
        bu_num(ui.msg[1], bu.n);
    }
    ui.msg_until = plat_ms() + ms;
    ui.d_lcd = 1;
}
static UI_LATE void bu_end(uint32_t why)         /* the upload ends (why: 0 the page's end, 1 cancel, 2 nothing came):
                                                  * the last object written, answered 43 bank saved_hi saved_lo status */
{
    uint8_t f[10] = {0xF0, 0x7D, 0x42, 0x55, 0x43, 0x7F, 0, 0, 0, 0xF7};
    char a[12], v[16];
    if (bu.on) {
#ifdef HYB_SHIMMER
        bu_ext_off();                    /* (first: the ninth object, its extension frames' records staged already, is
                                          * written once) */
#endif
        if (ub_flush() != 0) bu.err = 1;
        bu.on = 0;
        bu.n = 0;                        /* (not armed: patchx.c's) */
        f[5] = bu.bank; f[6] = (uint8_t)(bu.saved >> 7 & 0x7Fu); f[7] = (uint8_t)(bu.saved & 0x7Fu);
        f[8] = (uint8_t)(bu.err ? 3u : why == 2u ? 2u : 0u);
        a[0] = v[0] = 0;
        text_cat(a, "Bank C:");
        a[5] = (char)(UB_L0 + bu.bank);
        if (bu.err) text_cat(v, "FAILED");
        else { bu_num(v, bu.saved); text_cat(v, " saved"); }
        app_say(a, v);
        ui.msg_until = plat_ms() + 3000u;
        ui.d_rows = 1;                   /* (a list of the slots up: their names) */
    }
    plat_sysex_send(f, sizeof f);        /* (not armed: bank 7F, nothing saved) */
}
static UI_LATE void sx_to_edit(const uint8_t *d)   /* a single into the edit buffer: it sounds, the strip names it */
{
    uint32_t k;
    char nm[12];
    all_notes_off();
    for (k = 0; k < 256u; k++)
        shadow[k] = d[k];
#ifdef HYB_SHIMMER
    for (k = 0; k < SHX_N; k++)
        shadow[256u + k] = SHX_DEF[k];   /* (a dump has no extension: shimmer off; an extension frame right after it
                                          * puts one: patchx.c px_ext) */
    shadow[256u + SHX_GL] = SHX_GDEF;
#endif
    sx_edit = 1;
    undo_n = 0;                          /* (steps against the patch that was up, or a single before: one step back to
                                          * set.patch, the patch up before the first single, however many came since:
                                          * UNDO returns to it, with its edits; load_patch and SAVE end the series) */
    undo_push(set.patch, UNDO_PATCH, 0);
    shadow_send();
    ui.last = -1;
    ui.edit_t = 0;
    ui.plist_until = 0;
    ui.alt_t = plat_ms();
    ui.ppatch = (uint16_t)~0u;           /* (PLAY mode: the smart choice again) */
    ui.d_lcd = ui.d_rows = 1;
    home_sig[0] = home_sig[1] = 0;
    for (k = 0; k < 10u; k++) {
        const char c = (char)d[240 + k];
        nm[k] = c >= 32 && c < 127 ? c : ' ';
    }
    while (k && nm[k - 1u] == ' ') k--;
    nm[k] = 0;
    app_say("SysEx:", nm);
}
static UI_LATE void sx_single(const uint8_t *p, uint32_t n)   /* p: 00 20 33 01 dd 10 ... (between F0 and F7) */
{
    const uint32_t pp = n > 7u ? p[7] & 0x7Fu : 0u, where = bu.on ? 1u + bu.bank : 0u;
    uint32_t k, s = 0, st = 0;
    if (n != SX_DUMP)
        st = 2;
    else {
        for (k = 4; k < SX_DUMP - 1u; k++)
            s += p[k];
        if ((s & 0x7Fu) != p[SX_DUMP - 1u] && ((s + 0x54u) & 0x7Fu) != p[SX_DUMP - 1u])
            st = 1;                      /* (0x54: 00 + 20 + 33 + 01) */
    }
    if (!st && where) {
        const uint32_t u = bu.bank * UB_SLOTS + pp;
        for (k = 0; k < 256u; k++)
            ub_cache[k] = p[8u + k];
        if (!ub_cache[240]) ub_cache[240] = ' ';   /* (a name starting with 0 would read as an empty slot) */
        if (ub_stage(u) != 0) { st = 3; bu.err = 1; }
        else bu.saved++;
#ifdef HYB_SHIMMER
        bu_x[pp >> 3] |= (uint8_t)(1u << (pp & 7u));
#endif
    } else if (!st) {
        sx_to_edit(p + 8);
        bu.n = (uint16_t)(pp + 1u);      /* (its extension frame may follow: patchx.c) */
    }
    if (where) {
        bu.got++;
        bu.t = plat_ms();
        bu_show(0, 4000u);
    }
    bu_send(0x50, pp, st, where, 3);
}
static UI_LATE void px_dump(uint32_t bank, uint32_t slot);
static UI_LATE void px_clear(uint32_t pp);
static UI_LATE void px_names(uint32_t bank);
static UI_LATE void bu_cmd(const uint8_t *a, uint32_t n)   /* the page's: a[0] the command */
{
    if (a[0] == 0x01u)
        bu_send(0x41, 4, 0, 0, 1);       /* (the version of this handshake: 2 with the dump 05 and the extension frame,
                                          * patchx.c; 3 (1.18C30) six user banks 0 .. 5, the dump's banks A, B as
                                          * PX_BANK_A, PX_BANK_B; 4 the patch manager's 06 clear and 07 names) */
    else if (a[0] == 0x02u) {
        if (n < 4u || a[1] >= UB_BANKS) { bu_send(0x42, 0x7Fu, 2, 0, 2); return; }
        if (bu.on) {                     /* (armed again: what came is written) */
            ub_flush();
#ifdef HYB_SHIMMER
            bu_ext_off();
#endif
        }
#ifdef HYB_SHIMMER
        { uint32_t k; for (k = 0; k < sizeof bu_x; k++) bu_x[k] = 0; }
#endif
        bu.on = 1; bu.bank = a[1]; bu.err = 0;
        bu.n = (uint16_t)(a[2] << 7 | a[3]);
        bu.got = bu.saved = 0;
        bu.t = plat_ms();
        bu_show("ready", 4000u);
        bu_send(0x42, bu.bank, 0, 0, 2);
    } else if (a[0] == 0x03u || a[0] == 0x04u)
        bu_end(a[0] == 0x04u);
    else if (a[0] == 0x05u && n >= 3u)
        px_dump(a[1], a[2]);             /* (a patch out: patchx.c) */
    else if (a[0] == 0x06u && n >= 2u)
        px_clear(a[1]);                  /* (armed: a slot emptied; patchx.c) */
    else if (a[0] == 0x07u && n >= 2u)
        px_names(a[1]);                  /* (a bank's names: patchx.c) */
}
static UI_LATE void bi_cmd(const uint8_t *a, uint32_t n);
#include "waves.c"                       /* the user wave bank (0.90): uw_*, its SysEx F0 7D 56 46 */
#include "splash.c"                      /* the boot image (0.99): bi_*, app_splash, its SysEx F0 7D 56 46 20-22 */
#include "remote.c"                      /* the remote editor (0.98): re_*, its SysEx F0 7D 52 45 */
#include "patchx.c"                      /* patches out and the extension frame (1.05): px_*, F0 7D 42 55 05, F0 7D 49 58 */
UI_LATE int app_sysex(const uint8_t *p, uint32_t n)
{
    const uint32_t xp = bu.on ? 0u : bu.n;   /* (not armed: the edit buffer's single right before this frame, its
                                          * pp + 1) */
    if (!bu.on) bu.n = 0;
    if (n >= 4u && p[0] == 0x7Du && p[1] == 0x49u && p[2] == 0x58u) {
        px_ext(p, n, xp);
        return 1;
    }
    if (n >= 4u && p[0] == 0x7Du && p[1] == 0x52u && p[2] == 0x45u) {
        re_cmd(p + 3, n - 3u);
        return 1;
    }
    if (n >= 4u && p[0] == 0x7Du && p[1] == 0x56u && p[2] == 0x46u) {
        uw_cmd(p + 3, n - 3u);
        return 1;
    }
    if (n >= 4u && p[0] == 0x7Du && p[1] == 0x42u && p[2] == 0x55u) {
        bu_cmd(p + 3, n - 3u);
        return 1;
    }
    if (n >= 6u && p[0] == 0x00u && p[1] == 0x20u && p[2] == 0x33u && p[3] == 0x01u && p[5] == 0x10u) {
        sx_single(p, n);
        return 1;
    }
    return 0;                            /* (not ours: the platform's, or dropped) */
}

/* ------------------------------------------------------------ the debug log's health lines --- */
#if HYB_HEALTH_LOG                       /* (0.67: built only when asked: HYB_HEALTH_LOG) */
/* the development notes: lines on the device's log (plat_log; fm1_remote.py log over USB), kept through a restart.
 * Every HL_MS an "F" line (the effects: the mode, the reverb's room, pointers' signature, its tables and canaries,
 * the ring's peak, its NaNs and denormals), every HL_SYS of them an "S" line (blocks, xruns, the stacks' depths, the
 * second core), and a "!" line at once when something is wrong (each kind once until it clears). The main loop only:
 * the ring is swept HL_SCAN words a pass (all of it in ~64 passes), the reverb's tables once a second. The state in
 * the pool (RAM is full) */
#define HL_MS 10000u                     /* an F line this often */
#define HL_SYS 3u                        /* an S line every 3rd F line */
#define HL_SCAN 512u                     /* words of the ring a main loop pass */
#define HL_MARK 0x5AC3A55Au              /* the stacks' unused words (hl_paint) */
enum { HL_SIG = 1, HL_TAB = 2, HL_CAN = 4, HL_ERR = 8, HL_NF = 16, HL_NAN = 32, HL_POS = 64, HL_C2 = 128 };
static struct {
    uint32_t t, n, chk, i, mode;         /* the last F line, the lines, the last check, the sweep's position and mode */
    uint32_t mx, nan, den;               /* the sweep so far: the peak (float bits or int16), NaNs / infinities, denormals */
    uint32_t smx, snan, sden, sint;      /* the last whole sweep's (sint: the ring held the delay's int16 then) */
    uint32_t h[8];                       /* the reverb's last health (hyb_rv_health) */
    uint32_t bad, said, err0;            /* the faults now, those said already, the reverb's re-syncs at the start */
} hl PLAT_POOL;

#ifdef __PI32V2__
extern char _ustack_lo[], _ustack_top[], _sstack_lo[], _sstack_top[];
static UI_LATE void hl_mark(char *lo, char *top, char *here)   /* lo..top marked, but 1 KB below here when inside */
{
    uint32_t *p = (uint32_t *)(void *)lo, *e = (uint32_t *)(void *)top;
    if (here >= lo && here <= top) e = (uint32_t *)(void *)(here - 1024);
    while (p < e) *p++ = HL_MARK;
}
static UI_LATE uint32_t hl_used(const char *lo, const char *top)   /* the stack's deepest use so far, bytes */
{
    const uint32_t *p = (const uint32_t *)(const void *)lo, *e = (const uint32_t *)(const void *)top;
    while (p < e && *p == HL_MARK) p++;
    return (uint32_t)(top - (const char *)p);
}
#endif
static UI_LATE void hl_paint(void)       /* app_init, interrupts off: the main and the interrupts' stacks marked */
{
#ifdef __PI32V2__
    volatile char here = 0;
    hl_mark(_ustack_lo, _ustack_top, (char *)&here);
    hl_mark(_sstack_lo, _sstack_top, (char *)&here);
#endif
}

static UI_LATE char *hl_dec(char *d, uint32_t v)
{
    char t[10];
    int i = 0;
    do { t[i++] = (char)('0' + v % 10u); v /= 10u; } while (v && i < 10);
    while (i) *d++ = t[--i];
    *d = 0;
    return d;
}
static UI_LATE char *hl_field(char *d, char c, uint32_t v)   /* " c<decimal>" */
{
    *d++ = ' ';
    *d++ = c;
    return hl_dec(d, v);
}
static UI_LATE char *hl_hex4(char *d, uint32_t v)
{
    int i;
    for (i = 3; i >= 0; i--) *d++ = "0123456789abcdef"[v >> (4 * i) & 15u];
    *d = 0;
    return d;
}
static UI_LATE uint32_t hl_peak(uint32_t mx, uint32_t isint)   /* the peak in % of full scale */
{
    float f;
    if (isint) return mx * 100u / 8192u; /* (the delay's int16: 8192 full scale) */
    { union { uint32_t u; float f; } w; w.u = mx; f = w.f; }
    f *= 100.f;
    return f >= 99999.f ? 99999u : (uint32_t)(f + 0.5f);
}

static UI_LATE void hl_sweep(void)       /* a slice of the delay / reverb memory and the chorus's lines */
{
    uint32_t k, mode = S.patch[112], rv = mode >= 2u && mode <= 4u;
    if (mode != hl.mode || S.fx_mode_last != mode) {   /* (a mode change: what the memory holds changes, start again) */
        hl.mode = mode;
        hl.i = hl.mx = hl.nan = hl.den = 0;
        return;
    }
    for (k = 0; k < HL_SCAN; k++, hl.i++) {
        uint32_t u;
        if (hl.i < 32768u) {
            if (!rv) {                   /* the delay (or nothing): its int16 line in the memory's start */
                int32_t v;
                if (hl.i >= SY_DELAY_FRAMES) { hl.i = 32767u; continue; }
                v = S.dl_buf[hl.i];
                u = (uint32_t)(v < 0 ? -v : v);
                if (u > hl.mx) hl.mx = u;
                continue;
            }
            { union { float f; uint32_t u; } w; w.f = S.rvx_ring[hl.i]; u = w.u; }
        } else if (hl.i < 32768u + 4096u) {
            union { float f; uint32_t u; } w;
            w.f = S.ch_buf[(hl.i - 32768u) >> 11][(hl.i - 32768u) & 2047u];
            if (!S.patch[105]) { hl.i = 32768u + 4095u; continue; }   /* (the chorus off: its lines not used) */
            u = w.u;
            if ((u >> 23 & 0xffu) == 0xffu) hl.nan++;
            continue;                    /* (its peak not taken: the reverb's, the delay's) */
        } else {                         /* a sweep done */
            hl.smx = hl.mx; hl.snan = hl.nan; hl.sden = hl.den; hl.sint = !rv;
            hl.i = hl.mx = hl.nan = hl.den = 0;
            if (hl.snan) hl.bad |= HL_NAN; else hl.bad &= ~(uint32_t)HL_NAN;
            return;
        }
        if ((u >> 23 & 0xffu) == 0xffu) { hl.nan++; continue; }
        if (!(u >> 23 & 0xffu) && (u & 0x7fffffu)) hl.den++;
        u &= 0x7fffffffu;
        if (u > hl.mx) hl.mx = u;
    }
}


static UI_LATE void hl_check(uint32_t now) /* once a second: the reverb's health, the second core, the positions */
{
    uint32_t h[8], k, bad = hl.bad & HL_NAN;
    char b[48], *q;
    if (hyb_rv_health(h) == 0)
        for (k = 0; k < 8u; k++) hl.h[k] = h[k];
    if (hl.n == 0 && hl.chk == 0) hl.err0 = hl.h[5];
    if (hl.h[0] >= 2u && hl.h[2] != hl.h[7]) bad |= HL_SIG;
    if (hl.h[3]) bad |= HL_TAB;
    if (hl.h[4]) bad |= HL_CAN;
    if (hl.h[5] != hl.err0) bad |= HL_ERR;
    if (hl.h[6]) bad |= HL_NF;
    if (S.dl_w > 262144u || S.ch_w > 262144u) bad |= HL_POS;   /* (the effects keep them at most 2^18) */
    if (plat_cpu2_gave_up()) bad |= HL_C2;
    hl.bad = bad;
    for (k = 1; k <= HL_C2; k <<= 1) {   /* each fault said once, when it comes */
        static const char *const W[8] = {"rv ptr", "rv tab", "rv canary", "rv resync", "rv state nan", "fx nan", "fx pos",
                                         "cpu2 off"};
        uint32_t j = 0, x = k;
        if (!(bad & k) || (hl.said & k)) continue;
        while (x >>= 1) j++;
        b[0] = '!';
        q = hl_dec(b + 1, now / 1000u);
        *q++ = ' ';
        *q = 0;
        text_cat(b, W[j]);
        q = b;
        while (*q) q++;
        if (k == HL_SIG) { *q++ = ' '; q = hl_hex4(q, hl.h[2]); *q++ = '/'; q = hl_hex4(q, hl.h[7]); }
        else if (k == HL_ERR) q = hl_field(q, 'e', hl.h[5] - hl.err0);
        else if (k == HL_NAN) q = hl_field(q, 'n', hl.snan);
        else if (k == HL_C2) q = hl_field(q, 'g', (uint32_t)plat_cpu2_gave_up());
        else if (k == HL_TAB) q = hl_field(q, 't', hl.h[3]);
        plat_log(b);
    }
    hl.said = bad;                       /* (a fault gone: said again when it comes back) */
}

static UI_LATE void hl_tick(uint32_t now)   /* app_frame */
{
    char b[48], *q;
    hl_sweep();
    if (now - hl.chk < 1000u) return;
    hl.chk = now;
    hl_check(now);
    if (hl.n && now - hl.t < HL_MS) return;
    hl.t = now;
    b[0] = 'F';                          /* F<s> m<mode> r<room> p<pointers> t<tables> k<canaries> x<peak %> n<NaN> d<denormals> */
    q = hl_dec(b + 1, now / 1000u);
    q = hl_field(q, 'm', S.patch[112]);
    q = hl_field(q, 'r', hl.h[1]);
    *q++ = ' '; *q++ = 'p'; q = hl_hex4(q, hl.h[2]);
    q = hl_field(q, 't', hl.h[3]);
    q = hl_field(q, 'k', hl.h[4]);
    q = hl_field(q, 'x', hl_peak(hl.smx, hl.sint));
    q = hl_field(q, 'n', hl.snan);
    q = hl_field(q, 'd', hl.sden);
    plat_log(b);
    if (hl.n++ % HL_SYS == 0) {          /* S<s> b<blocks k> x<xruns> u<main stack> i<interrupts' stack> c<core 2's stack> g<core 2 off> */
        b[0] = 'S';
        q = hl_dec(b + 1, now / 1000u);
        q = hl_field(q, 'b', blocks / 1000u);
        *q++ = 'k';
        q = hl_field(q, 'x', plat_xruns());
#ifdef __PI32V2__
        q = hl_field(q, 'u', hl_used(_ustack_lo, _ustack_top));
        q = hl_field(q, 'i', hl_used(_sstack_lo, _sstack_top));
#endif
        q = hl_field(q, 'c', plat_cpu2_stack());
        q = hl_field(q, 'g', (uint32_t)plat_cpu2_gave_up());
        plat_log(b);
    }
}
#endif

void app_ui_init(void)
{
    static uint8_t lb_done PLAT_POOL;    /* (1.18C43: the power-on show at the first only: not after an update that did
                                          * not take, a restore) */
    if (!lb_done) { lb_done = 1; lb_st = LED_BOOT_ANIM ? 1u : 0u; }
#ifdef HYB_TEST_NIGHT
    set.key_dark = KL_NIGHT;             /* (a test build: KEY LIGHTS NIGHT from the start, for the emulator) */
#endif
    touch_on(1);                         /* velocity: the keys' readings on */
    lcd_fill(0, 0, 240, 240, U_BG);
    ui.last = -1;
    ui.d_lcd = ui.d_tabs = ui.d_rows = 1;
    qa_go(0);                            /* (HOME) */
}

UI_SMALL void app_frame(void)
{
    static uint32_t t_status, last_ble;
    static uint32_t st_last[5];
    uint32_t now = plat_ms(), ble;
    ns_warm();
#ifdef HYB_MULTI
    ln_pre();                            /* (1.18C10: the launcher's buttons and knobs, before input's) */
#endif
    input();
#ifdef HYB_MULTI
    cm_run(now);                         /* (1.19.6: the MIDI CC map's learn and moved lines) */
    cp_run(now);                         /* (CC SHOW: a fixed map controller's parameter and value) */
    ln_idle();                           /* (the launcher's rows: the next one's patterns asked before its line) */
#endif
    hold_zone_fix();                     /* (1.16: SPLIT, a key let go latched by the part edited's HOLD, its zone's
                                          * part's off: ended) */
    key_rs_run();                        /* (HOLD's restarts, a block after their note-offs) */
    ann_tick(now);
    if (FM1B_BLE_CENTRAL) {              /* looking: a device found is named at once (REC connects to it) */
        static int found;
        int n, ev;
        if (ui.glook && (ui.view != V_GLO || ui.gch != G_SEARCH))
            ui.looking = ui.glook = 0;   /* (GLO's device list left another way: no more looking) */
        n = ui.looking ? btdev_count() : 0;
        btdev_page((ui.looking || ar.scan_t) && !set.ble_off);
        if (n != found && ui.glook)
            ui.d_rows = ui.d_lcd = 1;    /* (GLO's device list: the devices found in it) */
        else if (n > found && btdev_pick() < n)
            ble_device_say();
        found = n;
        while ((ev = btdev_event()) != BTDEV_NONE) {
            if (ev == BTDEV_CONNECTED || (ev == BTDEV_NO_ANSWER && !ar.trying) || ev == BTDEV_LOST)
                app_say("Device", btdev_word(ev));
            if (ui.view == V_GLO) ui.d_rows = 1;
        }
        auto_reconnect(now);             /* (after the events: an attempt of ours that failed is not shown) */
    }
    ble = (uint32_t)(plat_ble_connected() | plat_ble_device() << 1);
    if (ble != last_ble || (ui.msg_until && now >= ui.msg_until)) {
        if (ble != last_ble && ui.view == V_GLO) ui.d_rows = 1;   /* (the device's line) */
        last_ble = ble;
        if (now >= ui.msg_until)
            ui.msg_until = 0;
        ui.d_lcd = 1;
    }
    if (bu.on && now - bu.t >= BU_IDLE_MS)
        bu_end(2u);                      /* (an upload that stopped: what came is written) */
    re_tick(now);                        /* (the remote editor: what changed told, one there) */
    sq_idle(now);                        /* (the sequencer's pattern saved once it rests) */
    sq_busy_say(now);                    /* (a key that only fed a silent sequencer: the strip says so) */
#ifdef HYB_MULTI
    mute_hint_run();                     /* (a sequencer started by hand with its part muted: the strip says so) */
#endif
    dr_idle(now);                        /* (the drums' too, the drum kit's) */
    kit_idle(now);
    sp_idle();                           /* (the patterns asked to save at the stop; SAVE's question over) */
    fv_idle(now);                        /* (1.18C33: the favorites: the editor, a write) */
    if (ui.glo_t && !store_wait() && (int32_t)(now - ui.glo_t) >= (int32_t)GLO_SAVE_MS) {   /* GLO's settings: saved
                                          * once they rest and all is idle (store_wait; the main loop) */
        ui.glo_t = 0;
        if (glo_save() != 0) app_say("Save", "FAILED");
    }
    if (ui.view == V_GLO && ui.gpage == GP_ABOUT) {   /* (ABOUT: the battery's numbers once a second; draw_lcd
                                          * draws the strip only when they change) */
        static uint32_t t_batt;
        if (now - t_batt >= 1000u) { t_batt = now; ui.d_lcd = 1; }
    }
    {   /* the LCD strip again when what it shows changes (the alternation, a parameter's time over) */
        static uint32_t last_m = ~0u;
        uint32_t m = lcd_mode(now);
        if (m != last_m) { last_m = m; ui.d_lcd = 1; }
    }
    draw();
#ifdef HYB_BENCH
    {   /* HOME held 3 s with no key down: the chip's microbenchmark (bench.c), its numbers over the rows until they
         * are drawn again (the audio goes on: each test keeps its least time, a run not interrupted) */
        static uint32_t home_t;
        static int done;
        if (!(plat_buttons() >> B_HOME & 1u)) { home_t = now; done = 0; }
        else if (!done && now - home_t >= 3000u && !ui.keys) {
            static uint32_t res[BENCH_N], pool[1024] PLAT_POOL;
            int cyc = fm1_perf_cycles_run();
            done = 1;
            hyb_bench(cyc ? fm1_perf_cycles : plat_ticks24, cyc ? 1u : 360u / PLAT_TICKS_PER_US, pool, res);
            draw_bench(res, cyc);
        }
    }
#endif
    {   /* HOME held 0.4 s: the profile line (a press alone: nothing drawn); let go: the view drawn again */
        static uint32_t home_t;
        if (!(plat_buttons() >> B_HOME & 1u)) {
            if (prof_shown) { ui.d_rows = 1; home_sig[0] = home_sig[1] = 0; led[0].st = led[1].st = 2u; }
            home_t = now;
            prof_shown = 0;
        } else
            prof_shown = (uint8_t)(now - home_t >= 400u);
    }
#if HYB_HEALTH_LOG
    hl_tick(now);                        /* (the debug log's health lines) */
#endif
    if (prof_shown && now - t_status >= 250u) {   /* HOME held: the block's parts, % of its time */
        t_status = now;
        draw_prof(prof_avg, plat_cpu_pct(), sy_split[0], sy_split[1]);
    }                                    /* (no status line: sound before the GUI; its rows are the views') */
}
