/*
 * Copyright (C) 2026 Dirk Nehring <dnehring@gmx.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <https://www.gnu.org/licenses/>.
 */

// --clean-psi: pmt_clean_prepare() decides per adapter buffer, and
// pmt_clean_packet() per client, as the buffer is shared by all of them.

#include "clean_psi.h"
#include "minisatip.h"
#include "opts.h"
#include "utils.h"

#include <string.h>
#include <vector>

#define DEFAULT_LOG LOG_PMT

// What a client gets on a PMT pid: the section as broadcast, nothing while
// the service is undecided, or the section without CA descriptors.
enum clean_action { CLEAN_PASS = 0, CLEAN_HOLD, CLEAN_WRITE };

typedef struct clean_pmt {
    SPMT *pmt;
    int pid;
    enum clean_action action;
} SCleanPMT;

// A packet of the adapter buffer that is not handed on as it is. Kept in
// buffer order, so each client walks the list with its own cursor.
typedef struct clean_packet {
    int idx;      // packet number in ad->buf
    uint8_t hold; // withhold it, rather than send pkt in its place
    uint8_t pkt[DVB_FRAME];
} SCleanPacket;

static std::vector<SCleanPacket> clean_list[MAX_ADAPTERS];

// Nothing is known about the service yet; a rewrite starts at the next
// section start rather than in the middle of one.
void pmt_clean_reset(SPMT *pmt) {
    pmt->in_clear = 0;
    pmt->clear_run = 0;
    pmt->clean_off = -1;
}

// Append the descriptors in b[0..n) that are not CA descriptors. False if one
// runs past n.
static bool copy_no_ca(std::vector<uint8_t> &out, const uint8_t *b, int n) {
    for (int i = 0; i < n; i += 2 + b[i + 1]) {
        if (i + 2 > n || i + 2 + b[i + 1] > n)
            return false;
        if (b[i] != 0x09)
            out.insert(out.end(), b + i, b + i + 2 + b[i + 1]);
    }
    return true;
}

// The broadcast section minus its CA descriptors, with a new version and CRC.
// Empty if there is none to drop, or unless it is one current section.
void pmt_clean_build(SPMT *pmt, uint8_t *b, int len) {
    std::vector<uint8_t> &out = pmt->clean;
    int slen, pi_len, es_len, i, at, n;
    uint8_t crc[4];

    out.clear();
    if (len < 16)
        return;
    slen = 3 + (((b[1] & 0x0F) << 8) | b[2]);
    pi_len = ((b[10] & 0x0F) << 8) | b[11];
    if (slen > len || slen < 16 || !(b[5] & 0x01) || b[6] || b[7] ||
        12 + pi_len > slen - 4)
        return;
    memcpy(pmt->clean_hdr, b, sizeof(pmt->clean_hdr));

    out.assign(b, b + 12);
    if (!copy_no_ca(out, b + 12, pi_len)) {
        out.clear();
        return;
    }
    n = out.size() - 12;
    out[10] = (b[10] & 0xF0) | (n >> 8);
    out[11] = n & 0xFF;

    for (i = 12 + pi_len; i + 5 <= slen - 4; i += 5 + es_len) {
        es_len = ((b[i + 3] & 0x0F) << 8) | b[i + 4];
        at = out.size();
        out.insert(out.end(), b + i, b + i + 5);
        if (i + 5 + es_len > slen - 4 || !copy_no_ca(out, b + i + 5, es_len)) {
            out.clear();
            return;
        }
        n = out.size() - at - 5;
        out[at + 3] = (b[i + 3] & 0xF0) | (n >> 8);
        out[at + 4] = n & 0xFF;
    }
    // Bytes left over that are no stream, or no CA descriptor to drop
    if (i != slen - 4 || (int)out.size() == slen - 4) {
        out.clear();
        return;
    }

    // The content differs from the broadcast, so the version has to as well
    out[5] = (b[5] & 0xC1) | ((((b[5] >> 1) + 1) & 0x1F) << 1);
    n = out.size() + 4 - 3;
    out[1] = (b[1] & 0xF0) | (n >> 8);
    out[2] = n & 0xFF;
    copy32(crc, 0, crc_32(out.data(), out.size()));
    out.insert(out.end(), crc, crc + 4);
}

// A service can arrive descrambled (SAT>IP server, CAM, hardware descrambler)
// with no control word to check, so a run of clear PCR stream packets counts.
void pmt_clean_count_clear(adapter *ad, uint8_t *b, SPid *p) {
    SPMT *pmt;

    if (!(b[3] & 0x10)) // no payload: clear in a scrambled service too
        return;
    pmt = get_pmt(p->pmt);
    // only the PCR stream: a service may send its teletext clear, as RTL does
    if (!pmt || p->pid != pmt->pcr_pid || pmt->in_clear)
        return;

    if (b[3] & 0xC0) {
        pmt->clear_run = 0;
        return;
    }
    if (++pmt->clear_run < CLEAN_PSI_CLEAR_PACKETS)
        return;
    pmt->in_clear = 1;
    LOG("adapter %d delivers pmt %d pid %d sid %d descrambled, the CA "
        "descriptors of its PMT are stale",
        ad->id, pmt->id, pmt->pid, pmt->sid);
}

static SCleanPMT *clean_find(SCleanPMT *cp, int n, int pid) {
    for (int i = 0; i < n; i++)
        if (cp[i].pid == pid)
            return cp + i;
    return NULL;
}

// Offset of the payload, DVB_FRAME if the packet has none.
static int ts_payload(const uint8_t *b) {
    if (!(b[3] & 0x10))
        return DVB_FRAME;
    if (!(b[3] & 0x20))
        return 4;
    return b[4] < 183 ? b[4] + 5 : DVB_FRAME;
}

// Does this packet begin a PMT section?
static int starts_pmt(const uint8_t *b) {
    int p = ts_payload(b);
    if (!(b[1] & 0x40) || p >= DVB_FRAME - 1)
        return 0;
    p += b[p] + 1; // pointer field
    return p < DVB_FRAME && b[p] == 0x02;
}

// Rewrite only a service known to be in the clear, or an unreachable card
// server would get an encrypted one announced as free to air.
static enum clean_action pmt_clean_action(SPMT *pmt) {
    if (pmt->version < 0)
        return CLEAN_HOLD; // not parsed yet
    if (pmt->clean.empty())
        return CLEAN_PASS; // free to air, or not a section we rewrite
    if (pmt->state != PMT_CACHED && pmt->in_clear)
        return CLEAN_WRITE;
    return CLEAN_HOLD;
}

// Reserve the entry for packet idx: a copy of b to patch, or a hold if b is
// NULL. The caller fills it before the next call, so the vector cannot move.
static uint8_t *clean_add(std::vector<SCleanPacket> &out, int idx, uint8_t *b) {
    out.emplace_back();
    SCleanPacket &e = out.back();
    e.idx = idx;
    e.hold = b ? 0 : 1;
    if (b)
        memcpy(e.pkt, b, DVB_FRAME);
    return e.pkt;
}

// Is a client of this adapter still inside its PMT window? Called before the
// adapter lock, as the stream locks come first.
int pmt_clean_waiting(adapter *ad, int64_t rtime) {
    if (!opts.clean_psi)
        return 0;
    for (int i = 0; i < MAX_STREAMS; i++) {
        streams *sid = __atomic_load_n(&st[i], __ATOMIC_ACQUIRE);
        if (!sid)
            continue;
        std::lock_guard<SMutex> lock(sid->mutex);
        if (sid->enabled && sid->adapter == ad->id &&
            (!sid->clean_since ||
             rtime - sid->clean_since < opts.clean_psi_grace))
            return 1;
    }
    return 0;
}

// Decide what the PMT pids of this buffer carry, after the filters and the
// descramblers. probe also holds PMTs of services not classified yet.
void pmt_clean_prepare(adapter *ad, int probe) {
    std::vector<SCleanPacket> &out = clean_list[ad->id];
    SCleanPMT cp[MAX_PMT_FOR_ADAPTER];
    int i, n = 0;

    out.clear();
    if (!opts.clean_psi)
        return;

    for (i = 0; i < ad->active_pmts && n < MAX_PMT_FOR_ADAPTER; i++) {
        SPMT *pmt = get_pmt(ad->active_pmt[i]);
        if (!pmt || !pmt->enabled || pmt->pid < 0 || pmt->pid >= 8192)
            continue;
        SCleanPMT *dup = clean_find(cp, n, pmt->pid);
        if (dup) {
            dup->action = CLEAN_PASS; // two services on one pid: leave it
            continue;
        }
        cp[n].pmt = pmt;
        cp[n].pid = pmt->pid;
        cp[n].action = pmt_clean_action(pmt);
        n++;
    }
    if (!n && !probe)
        return;

    for (i = 0; i < ad->rlen; i += DVB_FRAME) {
        uint8_t *b = ad->buf + i, *sec, *dst;
        enum clean_action action = CLEAN_PASS;
        int pid, pay, room, left, olen, slen, last = 0;
        SCleanPMT *c;
        SPMT *pmt;

        if (b[0] != 0x47)
            continue;
        pid = PID_FROM_TS(b);
        c = clean_find(cp, n, pid);
        if (c)
            action = c->action;
        else if (probe && pid && starts_pmt(b))
            action = CLEAN_HOLD; // a PMT of a service not classified yet

        if (action == CLEAN_HOLD) {
            clean_add(out, i / DVB_FRAME, NULL);
            continue;
        }
        if (action != CLEAN_WRITE) {
            // The section goes out as broadcast, so a rewrite starts again at
            // the next section start
            if (c && (b[1] & 0x40))
                c->pmt->clean_off = -1;
            continue;
        }

        pay = ts_payload(b);
        if (pay >= DVB_FRAME)
            continue;
        room = DVB_FRAME - pay;
        pmt = c->pmt;

        if ((b[1] & 0x40) && b[pay] != 0) {
            // The pointer field ends our section in the bytes before it, and
            // the next section behind them is not ours
            if (b[pay] >= room) {
                pmt->clean_off = -1;
                continue;
            }
            room = b[pay];
            pay++;
            last = 1;
        } else if (b[1] & 0x40) {
            // Replace only the section pmt->clean was built from: same table,
            // length, service, version and section number
            sec = b + pay + 1;
            if (room < 1 + (int)sizeof(pmt->clean_hdr) ||
                memcmp(sec, pmt->clean_hdr, sizeof(pmt->clean_hdr)) != 0) {
                LOGM("%s: pid %d carries a section this does not cover, "
                     "passing it on",
                     __FUNCTION__, pid);
                pmt->clean_off = -1;
                continue;
            }
            // A shorter rewrite would stuff over a section packed behind it
            olen = 3 + (((sec[1] & 0x0F) << 8) | sec[2]);
            if (sec + olen < b + DVB_FRAME && sec[olen] != 0xFF) {
                LOGM("%s: pid %d packs table %02X behind the PMT, passing it "
                     "on",
                     __FUNCTION__, pid, sec[olen]);
                pmt->clean_off = -1;
                continue;
            }
            pay++; // the pointer field stays 0
            room--;
            pmt->clean_off = 0;
        }
        if (pmt->clean_off < 0)
            continue; // a continuation of a section that was passed on

        // The rewrite is never longer than the original, so every TS header,
        // continuity counter included, reaches the client as broadcast
        slen = pmt->clean.size();
        dst = clean_add(out, i / DVB_FRAME, b);
        if (pmt->clean_off > slen)
            pmt->clean_off = slen;
        left = slen - pmt->clean_off;
        if (left > room)
            left = room;
        if (left > 0) {
            memcpy(dst + pay, pmt->clean.data() + pmt->clean_off, left);
            pmt->clean_off += left;
        }
        if (left < room) // stuffing, as the original carried past its section
            memset(dst + pay + left, 0xFF, room - left);
        if (last)
            pmt->clean_off = -1;
    }
}

// Packets of the current buffer that are not handed on as they are.
int pmt_clean_packets(adapter *ad) { return clean_list[ad->id].size(); }

// Arm the client's PMT window at the first buffer it is served; true while
// the window is open.
int pmt_clean_window(streams *sid, int64_t rtime) {
    if (!opts.clean_psi)
        return 0;
    if (!sid->clean_since)
        sid->clean_since = rtime;
    return rtime - sid->clean_since < opts.clean_psi_grace;
}

// What this client gets for packet idx: the rewrite, the original, or NULL
// while it waits for a verdict. pos walks the list along with idx.
uint8_t *pmt_clean_packet(adapter *ad, int idx, uint8_t *b, int in_grace,
                          int *pos) {
    std::vector<SCleanPacket> &l = clean_list[ad->id];
    int n = l.size();

    while (*pos < n && l[*pos].idx < idx)
        (*pos)++;
    if (*pos >= n || l[*pos].idx != idx)
        return b;
    if (!l[*pos].hold)
        return l[*pos].pkt;
    return in_grace ? NULL : b;
}
