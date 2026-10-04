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
#include "clean_psi.h"
#include "minisatip.h"
#include "utils.h"
#include "utils/testing.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define DEFAULT_LOG LOG_PMT

extern adapter *a[MAX_ADAPTERS];
extern SPMT *pmts[MAX_PMT];

// A broadcast PMT with video, audio and teletext; ca adds CA descriptors on
// the program level and on the audio stream.
static int build_broadcast_pmt(uint8_t *out, int sid, int ver, int cni,
                               int ca) {
    static const uint8_t pca[] = {0x09, 0x04, 0x09, 0x8C, 0xE0, 0xCC};
    static const uint8_t priv[] = {0x5F, 0x04, 0x00, 0x00, 0x00, 0x2A};
    static const uint8_t aca[] = {0x09, 0x04, 0x09, 0x8D, 0xE0, 0xCD};
    static const uint8_t lang[] = {0x0A, 0x04, 'd', 'e', 'u', 0x00};
    static const uint8_t ttx[] = {0x56, 0x05, 'd',  'e',  'u',
                                  0x10, 0x00, 0x52, 0x01, 0x03};
    uint8_t *b = out, *l;
    auto put = [&](const uint8_t *d, int n) {
        memcpy(b, d, n);
        b += n;
    };
    auto stream = [&](int type, int pid) {
        *b++ = type;
        copy16(b, 0, 0xE000 | pid);
        b += 2;
        l = b;
        b += 2;
    };
    auto close = [&]() { copy16(l, 0, 0xF000 | (b - l - 2)); };

    *b++ = 0x02;
    b += 2; // section_length
    copy16(b, 0, sid);
    b += 2;
    *b++ = 0xC0 | ((ver & 0x1F) << 1) | (cni ? 1 : 0);
    *b++ = 0; // section number
    *b++ = 0; // last section number
    copy16(b, 0, 0xE000 | 1279);
    b += 2;
    l = b;
    b += 2;
    if (ca)
        put(pca, sizeof(pca));
    put(priv, sizeof(priv));
    close();
    stream(27, 1279);
    close();
    stream(3, 1283);
    if (ca)
        put(aca, sizeof(aca));
    put(lang, sizeof(lang));
    close();
    stream(6, 34);
    put(ttx, sizeof(ttx));
    close();

    copy16(out, 1, 0xB000 | ((b - out) + 4 - 3));
    copy32(b, 0, crc_32(out, b - out));
    return (b - out) + 4;
}

// The rewrite is the broadcast section minus its CA descriptors: every stream
// and every other descriptor stays, byte for byte and in its order.
int test_clean_pmt() {
    SPMT pmt = {};
    uint8_t want[256], *sec;
    int slen, wlen;

    // Sized exactly, so that a read past the section is caught
    slen = build_broadcast_pmt(want, 0x0083, 5, 1, 1);
    sec = (uint8_t *)malloc(slen);

    build_broadcast_pmt(sec, 0x0083, 5, 1, 1);
    wlen = build_broadcast_pmt(want, 0x0083, 6, 1, 0);
    pmt_clean_build(&pmt, sec, slen);
    ASSERT((int)pmt.clean.size() == wlen &&
               !memcmp(pmt.clean.data(), want, wlen),
           "the rewrite is not the broadcast without its CA descriptors");
    ASSERT(!memcmp(pmt.clean_hdr, sec, sizeof(pmt.clean_hdr)),
           "the header of the broadcast section was not kept");

    // The version is five bits wide and wraps
    build_broadcast_pmt(sec, 0x0083, 31, 1, 1);
    wlen = build_broadcast_pmt(want, 0x0083, 0, 1, 0);
    pmt_clean_build(&pmt, sec, slen);
    ASSERT((int)pmt.clean.size() == wlen &&
               !memcmp(pmt.clean.data(), want, wlen),
           "the version did not wrap");

    // Nothing to strip, or a section that is not on the wire yet
    pmt_clean_build(&pmt, sec, build_broadcast_pmt(sec, 0x0083, 5, 1, 0));
    ASSERT(pmt.clean.empty(), "a free to air PMT was rewritten");
    build_broadcast_pmt(sec, 0x0083, 5, 0, 1);
    pmt_clean_build(&pmt, sec, slen);
    ASSERT(pmt.clean.empty(), "a section that is not current was rewritten");

    // A descriptor that runs past its loop leaves nothing half built
    build_broadcast_pmt(sec, 0x0083, 5, 1, 1);
    sec[13] = 0xFF;
    pmt_clean_build(&pmt, sec, slen);
    ASSERT(pmt.clean.empty(), "a broken descriptor loop was rewritten");

    free(sec);
    return 0;
}

// Wrap a section into one TS packet. pusi 0 makes it a continuation packet,
// payload_room shrinks the payload to that many bytes.
static void build_ts(uint8_t *p, int pid, int pusi, const uint8_t *payload,
                     int len, int payload_room) {
    int pay = 4;
    memset(p, 0xFF, DVB_FRAME);
    p[0] = 0x47;
    p[1] = (pusi ? 0x40 : 0) | ((pid >> 8) & 0x1F);
    p[2] = pid & 0xFF;
    p[3] = 0x10;
    if (payload_room > 0 && payload_room < DVB_FRAME - 4) {
        int af = DVB_FRAME - 4 - payload_room - 1;
        p[3] |= 0x20;
        p[4] = af;
        if (af > 0) {
            p[5] = 0;
            memset(p + 6, 0xFF, af - 1);
        }
        pay = 5 + af;
    }
    if (pusi)
        p[pay++] = 0; // pointer field
    if (len > DVB_FRAME - pay)
        len = DVB_FRAME - pay;
    if (len > 0)
        memcpy(p + pay, payload, len);
}

// The real reset plus the proof of descrambling, so the PMT is rewritten and
// whatever the rewriter leaves alone, it leaves alone on purpose.
static void arm_clean(SPMT *pmt) {
    pmt_clean_reset(pmt);
    pmt->in_clear = 1;
}

// An adapter whose buffer holds exactly npkt packets, so that a read or a
// write past the buffer is caught.
static adapter *clean_adapter(int npkt) {
    adapter *ad = adapter_alloc();
    free(ad->buf);
    ad->buf = (uint8_t *)malloc((size_t)npkt * DVB_FRAME);
    ad->rlen = npkt * DVB_FRAME;
    a[0] = ad;
    ad->id = 0;
    ad->enabled = 1;
    return ad;
}

// The service under test, descrambled, with CA descriptors on the program
// level and on the audio stream.
static void clean_pmt_setup(SPMT *pmt, adapter *ad) {
    uint8_t sec[256];
    pmts[0] = pmt;
    npmts = 1; // get_pmt() refuses anything at or above this
    pmt->id = 0;
    pmt->enabled = 1;
    pmt->adapter = 0;
    pmt->sid = 0x0083;
    pmt->pid = 100;
    pmt->pcr_pid = 1279;
    pmt->version = 5;
    pmt->state = PMT_RUNNING;
    pmt->in_clear = 1;
    pmt_clean_build(pmt, sec, build_broadcast_pmt(sec, 0x0083, 5, 1, 1));
    ad->active_pmts = 1;
    ad->active_pmt[0] = 0;
    mark_pid_add(0, ad->id, 100);
    update_pids(ad->id);
}

static void clean_teardown(adapter *ad) {
    ad->active_pmts = 0;
    pmts[0] = NULL;
    pmts[1] = NULL;
    npmts = 0;
    free(ad->buf);
    delete ad;
    a[0] = NULL;
}

// What a client is handed for packet idx; pos is the client's cursor across
// the calls for one buffer.
static uint8_t *clean_get(adapter *ad, int idx, int in_grace, int *pos) {
    return pmt_clean_packet(ad, idx, ad->buf + idx * DVB_FRAME, in_grace, pos);
}

// The single packet in the buffer, as a client with an open window sees it.
static uint8_t *clean_one(adapter *ad) {
    int pos = 0;
    pmt_clean_prepare(ad, 1);
    return clean_get(ad, 0, 1, &pos);
}

// The packet shapes pmt_clean_prepare() must not touch. The adapter buffer is
// shared with every client and the CA layer, so it always stays as read.
int test_clean_psi_packets() {
    adapter *ad = clean_adapter(1);
    SPMT pmt = {}, other = {};
    uint8_t sec[512], before[DVB_FRAME], *out;
    int slen, saved_clean = opts.clean_psi;
    int saved_grace = opts.clean_psi_grace;

    clean_pmt_setup(&pmt, ad);
    opts.clean_psi = 1;
    opts.clean_psi_grace = CLEAN_PSI_GRACE;

    slen = build_broadcast_pmt(sec, pmt.sid, pmt.version, 1, 1);

    // 1. A continuation packet arriving first is passed on: nothing may be
    //    written into the middle of a section
    arm_clean(&pmt);
    build_ts(ad->buf, 100, 0, sec, slen, 0);
    ASSERT(clean_one(ad) == ad->buf, "a continuation packet was rewritten");

    // 2. The real thing is rewritten, and the CA descriptors are gone
    arm_clean(&pmt);
    build_ts(ad->buf, 100, 1, sec, slen, 0);
    memcpy(before, ad->buf, DVB_FRAME);
    out = clean_one(ad);
    ASSERT(out && out != ad->buf, "the PMT of a descrambled service was kept");
    ASSERT(!memcmp(before, ad->buf, DVB_FRAME),
           "the rewrite changed the adapter buffer");
    ASSERT(!memcmp(before, out, 4), "the rewrite changed the TS header");
    // pointer field at 4, so the section starts at 5
    ASSERT(out[4] == 0 && out[5] == 0x02,
           "the rewritten packet lost its pointer field or table id");
    ASSERT(((out[8] << 8) | out[9]) == pmt.sid,
           "the rewritten section changed the service id");
    ASSERT(((out[10] & 0x3E) >> 1) == ((pmt.version + 1) & 0x1F),
           "the rewritten section kept the broadcast version");
    ASSERT(!memcmp(out + 5, pmt.clean.data(), pmt.clean.size()),
           "the packet does not carry the section without CA descriptors");
    for (int i = 5 + pmt.clean.size(); i < DVB_FRAME; i++)
        ASSERT(out[i] == 0xFF, "the freed bytes were not stuffed");

    // 3. Another service on the pid must not be replaced with ours
    arm_clean(&pmt);
    slen = build_broadcast_pmt(sec, 0x0084, pmt.version, 1, 1);
    build_ts(ad->buf, 100, 1, sec, slen, 0);
    ASSERT(clean_one(ad) == ad->buf, "a foreign service id was rewritten");

    // 4. A version that was never parsed, and a not-yet-current section
    arm_clean(&pmt);
    slen = build_broadcast_pmt(sec, pmt.sid, pmt.version + 1, 1, 1);
    build_ts(ad->buf, 100, 1, sec, slen, 0);
    ASSERT(clean_one(ad) == ad->buf, "an unparsed version was rewritten");

    arm_clean(&pmt);
    slen = build_broadcast_pmt(sec, pmt.sid, pmt.version, 0, 1);
    build_ts(ad->buf, 100, 1, sec, slen, 0);
    ASSERT(clean_one(ad) == ad->buf,
           "a current_next_indicator of 0 was made current");

    // 5. A second section packed behind ours would be stuffed over
    arm_clean(&pmt);
    slen = build_broadcast_pmt(sec, pmt.sid, pmt.version, 1, 1);
    sec[slen] = 0x02; // a further table starts right here
    sec[slen + 1] = 0xB0;
    build_ts(ad->buf, 100, 1, sec, slen + 2, 0);
    ASSERT(clean_one(ad) == ad->buf,
           "a section packed behind the PMT was overwritten");

    // 6. A payload too short to hold a PMT header must not be read past the
    //    end of the packet
    arm_clean(&pmt);
    build_ts(ad->buf, 100, 1, sec, 4, 6);
    ASSERT(clean_one(ad) == ad->buf, "a truncated PSI header was rewritten");

    // 7. Two services on one PMT pid: neither is ours to replace
    arm_clean(&pmt);
    slen = build_broadcast_pmt(sec, pmt.sid, pmt.version, 1, 1);
    build_ts(ad->buf, 100, 1, sec, slen, 0);
    other = pmt;
    other.id = 1;
    other.sid = 0x0084;
    pmts[1] = &other;
    npmts = 2;
    ad->active_pmts = 2;
    ad->active_pmt[1] = 1;
    ASSERT(clean_one(ad) == ad->buf,
           "a pid carrying two services was rewritten");
    ad->active_pmts = 1;
    pmts[1] = NULL;
    npmts = 1;

    // 8. With the option off there is nothing to hand a client
    arm_clean(&pmt);
    opts.clean_psi = 0;
    build_ts(ad->buf, 100, 1, sec, slen, 0);
    ASSERT(clean_one(ad) == ad->buf && pmt_clean_packets(ad) == 0,
           "--clean-psi is off but a packet was replaced");

    opts.clean_psi = saved_clean;
    opts.clean_psi_grace = saved_grace;
    clean_teardown(ad);
    return 0;
}

// A PMT that does not end on a packet boundary: the packet it ends in carries
// a pointer field and the start of the next section behind it.
int test_clean_psi_spanning_section() {
    adapter *ad = clean_adapter(3);
    SPMT pmt = {};
    uint8_t sec[512], before[3 * DVB_FRAME], asmb[512], *first, *last;
    int i, slen, head, tail, olen, pos = 0;
    int saved_clean = opts.clean_psi, saved_grace = opts.clean_psi_grace;
    uint32_t crc;

    clean_pmt_setup(&pmt, ad);
    opts.clean_psi = 1;
    opts.clean_psi_grace = CLEAN_PSI_GRACE;
    arm_clean(&pmt);

    // The first packet carries the pointer field and 19 bytes of the section,
    // so the rest of it ends inside the second one
    slen = build_broadcast_pmt(sec, pmt.sid, pmt.version, 1, 1);
    head = 19;
    tail = slen - head;
    ASSERT(tail > 0 && tail < DVB_FRAME - 6,
           "the test section does not end inside the second packet");
    build_ts(ad->buf, 100, 1, sec, slen, head + 1);
    last = ad->buf + DVB_FRAME;
    build_ts(last, 100, 1, sec + head, tail, 0);
    last[4] = (uint8_t)tail; // pointer field: the rest of the PMT comes first
    last[5 + tail] = 0x42;   // and a table of whoever sent it behind that
    last[6 + tail] = 0xF0;
    // that table continues in the packet after it, which is none of ours
    build_ts(ad->buf + 2 * DVB_FRAME, 100, 0, sec, slen, 0);
    memcpy(before, ad->buf, sizeof(before));

    pmt_clean_prepare(ad, 0);
    first = clean_get(ad, 0, 1, &pos);
    ASSERT(first && first != ad->buf, "the start of the PMT was not rewritten");
    last = clean_get(ad, 1, 1, &pos);
    ASSERT(last && last != ad->buf + DVB_FRAME,
           "the end of the PMT reached the client as broadcast");
    ASSERT(clean_get(ad, 2, 1, &pos) == ad->buf + 2 * DVB_FRAME,
           "the section behind the PMT was written into");

    // The TS header, the pointer field and the table behind it are the
    // sender's; only the bytes the PMT occupied are ours
    ASSERT(!memcmp(last, before + DVB_FRAME, 5),
           "the TS header or the pointer field of the last packet changed");
    ASSERT(!memcmp(last + 5 + tail, before + DVB_FRAME + 5 + tail,
                   DVB_FRAME - 5 - tail),
           "the section behind the pointer field was overwritten");

    // What the client reassembles has to be one PMT, not half of two
    ASSERT(first[DVB_FRAME - head - 1] == 0,
           "the rewrite did not start at a zero pointer field");
    memcpy(asmb, first + DVB_FRAME - head, head);
    memcpy(asmb + head, last + 5, tail);
    olen = 3 + (((asmb[1] & 0x0F) << 8) | asmb[2]);
    ASSERT(asmb[0] == 0x02 && olen >= 4 && olen <= head + tail,
           "the reassembled section is not a PMT");
    crc = ((uint32_t)asmb[olen - 4] << 24) | (asmb[olen - 3] << 16) |
          (asmb[olen - 2] << 8) | asmb[olen - 1];
    ASSERT(crc_32(asmb, olen - 4) == crc,
           "the reassembled section fails its CRC");
    for (i = 12; i < olen - 4; i++)
        ASSERT(!(asmb[i] == 0x09 && asmb[i + 1] == 0x04),
               "a CA descriptor survived the rewrite");
    ASSERT(!memcmp(before, ad->buf, sizeof(before)),
           "the adapter buffer was modified");

    // A pointer field that runs past the payload is not one to write into
    arm_clean(&pmt);
    ad->buf[DVB_FRAME + 4] = DVB_FRAME;
    pmt_clean_prepare(ad, 0);
    pos = 0;
    ASSERT(clean_get(ad, 1, 1, &pos) == ad->buf + DVB_FRAME,
           "a pointer field past the payload was written into");
    ad->buf[DVB_FRAME + 4] = (uint8_t)tail;

    // And neither is one that arrives with no section of ours in progress
    arm_clean(&pmt);
    ad->buf[1] = 0; // the first packet is now a pid nobody asked about
    ad->buf[2] = 200;
    pmt_clean_prepare(ad, 0);
    pos = 0;
    ASSERT(clean_get(ad, 1, 1, &pos) == ad->buf + DVB_FRAME,
           "a pointer field was followed with no section to finish");

    opts.clean_psi = saved_clean;
    opts.clean_psi_grace = saved_grace;
    clean_teardown(ad);
    return 0;
}

// The verdict is the adapter's, the waiting the client's: of two clients on
// one buffer, only the one inside its window holds the pid back.
int test_clean_psi_streams() {
    adapter *ad = clean_adapter(3);
    SPMT pmt = {};
    uint8_t sec[512], before[3 * DVB_FRAME], *out;
    int slen, pos, saved_clean = opts.clean_psi;
    int saved_grace = opts.clean_psi_grace;

    clean_pmt_setup(&pmt, ad);
    opts.clean_psi = 1;
    opts.clean_psi_grace = CLEAN_PSI_GRACE;
    slen = build_broadcast_pmt(sec, pmt.sid, pmt.version, 1, 1);

    // The PMT of the descrambled service, a video packet, and the PMT of a
    // service nothing is known about yet
    build_ts(ad->buf, 100, 1, sec, slen, 0);
    build_ts(ad->buf + DVB_FRAME, 1279, 0, sec, 0, 0);
    build_ts(ad->buf + 2 * DVB_FRAME, 200, 1, sec, slen, 0);
    memcpy(before, ad->buf, sizeof(before));

    // 1. A client inside its window: the rewrite, the video untouched, and
    //    nothing at all for the service that has no verdict
    arm_clean(&pmt);
    pmt_clean_prepare(ad, 1);
    pos = 0;
    out = clean_get(ad, 0, 1, &pos);
    ASSERT(out && out != ad->buf, "the descrambled PMT was not rewritten");
    ASSERT(clean_get(ad, 1, 1, &pos) == ad->buf + DVB_FRAME,
           "a video packet was replaced");
    ASSERT(clean_get(ad, 2, 1, &pos) == NULL,
           "an unclassified PMT was handed to a waiting client");

    // 2. A client past its window sees the same rewrite, but no longer waits
    //    for the service that has no verdict
    pos = 0;
    ASSERT(clean_get(ad, 0, 0, &pos) == out,
           "the rewrite depends on the client");
    ASSERT(clean_get(ad, 2, 0, &pos) == ad->buf + 2 * DVB_FRAME,
           "an unclassified PMT was withheld after the window closed");

    // 3. Without a client waiting, an unknown PMT pid is not looked at
    arm_clean(&pmt);
    pmt_clean_prepare(ad, 0);
    pos = 0;
    ASSERT(clean_get(ad, 2, 1, &pos) == ad->buf + 2 * DVB_FRAME,
           "an unknown PMT pid was held with no client waiting");

    // 4. An encrypted service that nothing has decrypted is held, never
    //    announced as free to air
    pmt_clean_reset(&pmt); // not in the clear this time
    pmt_clean_prepare(ad, 0);
    pos = 0;
    ASSERT(clean_get(ad, 0, 1, &pos) == NULL,
           "an undecrypted service was passed on to a waiting client");
    pos = 0;
    ASSERT(clean_get(ad, 0, 0, &pos) == ad->buf,
           "an undecrypted service was still held after the window closed");

    ASSERT(!memcmp(before, ad->buf, sizeof(before)),
           "the adapter buffer was modified");

    opts.clean_psi = saved_clean;
    opts.clean_psi_grace = saved_grace;
    clean_teardown(ad);
    return 0;
}

// A service that arrives descrambled, from a SAT>IP server or a CAM, has no
// control word to check here; the rewrite waits for a clear PCR stream.
int test_clean_psi_arrives_clear() {
    adapter *ad = clean_adapter(2);
    SPMT pmt = {};
    uint8_t sec[512], *v;
    int slen, pos, i, saved_clean = opts.clean_psi;
    int saved_grace = opts.clean_psi_grace;
    SPid *p;

    clean_pmt_setup(&pmt, ad);
    opts.clean_psi = 1;
    opts.clean_psi_grace = CLEAN_PSI_GRACE;
    slen = build_broadcast_pmt(sec, pmt.sid, pmt.version, 1, 1);
    build_ts(ad->buf, 100, 1, sec, slen, 0);
    v = ad->buf + DVB_FRAME;
    build_ts(v, 1279, 0, sec, 0, 0);

    mark_pid_add(0, ad->id, 1279);
    update_pids(ad->id);
    p = find_pid(ad->id, 1279);
    ASSERT(p != NULL, "no pid entry for the video stream");
    p->pmt = pmt.id;

    // 1. Nothing has decrypted this service and nothing has been seen of it:
    //    the pid is withheld, not announced free to air
    pmt_clean_reset(&pmt);
    pmt_clean_prepare(ad, 0);
    pos = 0;
    ASSERT(clean_get(ad, 0, 1, &pos) == NULL,
           "a service with no verdict was handed to a waiting client");

    // 2. A packet without payload is clear in a scrambled service too
    v[3] = 0x20;
    for (i = 0; i < 2 * CLEAN_PSI_CLEAR_PACKETS; i++)
        pmt_clean_count_clear(ad, v, p);
    ASSERT(!pmt.in_clear,
           "packets without payload were counted as a clear stream");

    // 3. Nor does a stream that is not the one carrying the PCR: a service
    //    may broadcast its teletext in the clear and scramble the rest
    mark_pid_add(0, ad->id, 1283);
    update_pids(ad->id);
    SPid *tt = find_pid(ad->id, 1283);
    ASSERT(tt != NULL, "no pid entry for the teletext stream");
    tt->pmt = pmt.id;
    v[3] = 0x10;
    for (i = 0; i < 2 * CLEAN_PSI_CLEAR_PACKETS; i++)
        pmt_clean_count_clear(ad, v, tt);
    ASSERT(!pmt.in_clear,
           "a clear stream that is not the PCR one decided the service");
    mark_pid_deleted(ad->id, 0, 1283, NULL);
    update_pids(ad->id);
    // update_pids() re-elects the pid owners, and this PMT lists no streams
    p->pmt = pmt.id;

    // 4. One scrambled payload packet ends the run
    v[3] = 0x10;
    for (i = 0; i < CLEAN_PSI_CLEAR_PACKETS - 1; i++)
        pmt_clean_count_clear(ad, v, p);
    v[3] = 0x90;
    pmt_clean_count_clear(ad, v, p);
    ASSERT(pmt.clear_run == 0, "a scrambled packet did not end the run");
    v[3] = 0x10;
    for (i = 0; i < CLEAN_PSI_CLEAR_PACKETS - 1; i++)
        pmt_clean_count_clear(ad, v, p);
    ASSERT(!pmt.in_clear, "the run was not counted from the start again");

    // 5. The packet that completes the run decides it, and the PMT is rebuilt
    //    without a control word ever having been validated
    pmt_clean_count_clear(ad, v, p);
    ASSERT(pmt.in_clear, "a stream that arrives in the clear was missed");
    pmt_clean_prepare(ad, 0);
    pos = 0;
    ASSERT(clean_get(ad, 0, 1, &pos) != NULL &&
               clean_get(ad, 0, 1, &pos) != ad->buf,
           "the PMT of a service that arrives in the clear was not rewritten");

    opts.clean_psi = saved_clean;
    opts.clean_psi_grace = saved_grace;
    mark_pid_deleted(ad->id, 0, 1279, NULL);
    update_pids(ad->id);
    clean_teardown(ad);
    return 0;
}

int main() {
    opts.log = 255;
    opts.debug = 255;
    strcpy(thread_info[thread_index].thread_name, "test_clean_psi");
    TEST_FUNC(test_clean_pmt(), "testing PMT rewrite without CA descriptors");
    TEST_FUNC(test_clean_psi_packets(),
              "testing which packets pmt_clean_prepare rewrites");
    TEST_FUNC(test_clean_psi_spanning_section(),
              "testing a PMT that ends inside a packet with a pointer field");
    TEST_FUNC(test_clean_psi_streams(),
              "testing the PMT each client is handed");
    TEST_FUNC(test_clean_psi_arrives_clear(),
              "testing a service that arrives already descrambled");
    fflush(stdout);
    return 0;
}
