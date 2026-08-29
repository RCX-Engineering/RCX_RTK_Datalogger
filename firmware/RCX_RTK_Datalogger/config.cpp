/*
 * config.cpp — Definitions for credential arrays declared in config.h
 * ====================================================================
 * Arrays can only be defined once across the whole project.
 * All other files include config.h (extern declarations only) and
 * link against these definitions here.
 *
 * Edit THIS file to add/remove WiFi networks or NTRIP casters.
 */

#include "config.h"

// WiFi networks are NOT defined here. The station list lives entirely in NVS and
// is managed from the web dashboard — see wifi_mgr.h for the storage layout and
// config.h for the configuration-AP settings that make a never-configured unit
// reachable in the first place.

// ── NTRIP Casters ─────────────────────────────────────────────────────────────
// These are the DEFAULTS. They cannot be deleted from the dashboard, because
// they are compiled in — but each one can be disabled there, and additional
// casters can be added at runtime without a recompile. Enable/disable state and
// added casters both live in NVS (namespace "rcx_ntrip"); see ntrip.h.
//
// Columns: host, port, username, password
//
// rtk2go: PUT YOUR OWN EMAIL ADDRESS IN THE USERNAME FIELD BELOW. The caster
// requires every rover to present an email as the NTRIP client account name, and
// documents that it will disconnect clients without one and may temporarily block
// an IP after repeated bad attempts. The password field is not used — their
// instructions are to leave it empty, or send "none" if the client insists on a
// value. The address is used for one purpose: when a connection keeps failing,
// the caster emails that account explaining what is wrong. A made-up address will
// typically still connect, but it forfeits the only feedback channel there is,
// so a blocked IP becomes a silent failure with no explanation.
//
// Centipede (open network): host=crtk.net, port=2101, user=centipede, pass=centipede.
// NOTE: crtk.net is the caster the RCX1 base PUSHES to (see RCX_RTK_Base config.h)
// — the rover must point at the same host. RCX1 is a private/undeclared mount: it
// is NEVER listed in the source table, but a direct GET /RCX1 works.
//
// 192.168.4.1 is the RCX1 base itself, serving its own NTRIP caster on its own
// access point. This is the no-internet path: join the base's AP from the WiFi
// list and corrections come straight off the base with nothing else involved —
// useful in the field, and the only setup needed for a bench test. The base
// accepts any mountpoint name with blank credentials, so the name below is just
// a label. It is reachable ONLY while associated with the base's AP; on any
// other network the address either refuses the connection or belongs to somebody
// else's router, which fails the NTRIP handshake and is skipped like any other
// unreachable caster.
const char* ntripCasters[][4] = {
    {"192.168.4.1","2101", "",                        ""},
    {"crtk.net",   "2101", "centipede",               "centipede"},
    {"rtk2go.com", "2101", "you@example.com",           "none"},
};
const int ntripCasterCount = sizeof(ntripCasters) / sizeof(ntripCasters[0]);

// Optional preferred mountpoint per caster (must have same number of entries as ntripCasters).
// Set to your known mountpoint name to skip source-table scanning and connect directly
// with top priority — useful when you operate your own base station.
// Leave "" to auto-select nearest mountpoint from the caster's source table.
const char* ntripPreferredMpt[] = {
    "Local_Wifi",  // local base over its own AP — direct connect, no source table involved
    "RCX1",  // Centipede: the base pushes here; private/undeclared mount, direct GET only
    "",      // rtk2go: auto-select nearest
};

// Compile-time guard: every caster in ntripCasters[] is indexed in parallel into
// ntripPreferredMpt[] (ntrip.cpp loadCasters / scanCaster). If you add a caster row
// above but forget to add a matching ntripPreferredMpt row, that parallel index would
// read out of bounds at runtime (garbage pointer → crash). This static_assert turns
// that mistake into a build error instead of a field crash.
static_assert(
    (sizeof(ntripPreferredMpt) / sizeof(ntripPreferredMpt[0])) >=
    (sizeof(ntripCasters)      / sizeof(ntripCasters[0])),
    "ntripPreferredMpt[] must have at least as many entries as ntripCasters[] "
    "— add a matching row (use \"\" for auto-select)");
