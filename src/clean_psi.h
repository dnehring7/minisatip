#ifndef CLEAN_PSI_H
#define CLEAN_PSI_H

#include "adapter.h"
#include "pmt.h"
#include "stream.h"

#include <stdint.h>

#define CLEAN_PSI_GRACE 3000 // ms a client may wait for the PMT

// Clear PCR stream packets in a row that show a service arrives descrambled;
// a scrambled service never gets that far.
#define CLEAN_PSI_CLEAR_PACKETS 100

void pmt_clean_reset(SPMT *pmt);
void pmt_clean_build(SPMT *pmt, uint8_t *b, int len);
void pmt_clean_count_clear(adapter *ad, uint8_t *b, SPid *p);
int pmt_clean_waiting(adapter *ad, int64_t rtime);
void pmt_clean_prepare(adapter *ad, int probe);
int pmt_clean_packets(adapter *ad);
int pmt_clean_window(streams *sid, int64_t rtime);
uint8_t *pmt_clean_packet(adapter *ad, int idx, uint8_t *b, int in_grace,
                          int *pos);

#endif
