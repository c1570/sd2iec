/* Host tests for the rppico fastloader timing (src/rppico/llfl-common.c)
 *
 * The simulation keeps a cycle count that advances with every read of
 * the cycle counter and the GPIO inputs, so busy-wait loops make
 * progress. The bus inputs follow a waveform function of that time,
 * line changes are recorded with their time.
 *
 * Times in the loader definitions are in 100ns units, 15 cycles each.
 */

#include <stdio.h>
#include <stdlib.h>
#include "config.h"
#include "llfl-common.h"
#include "hardware/structs/m33.h"

#define CYCLES_PER_TICK 15

/* edges must follow the requested time within this many cycles */
#define MAX_EDGE_DELAY 40

/* ---------- simulation ---------- */

static uint32_t now;            // simulated cycle count
static uint32_t edge;           // time of the start edge of a test
static uint32_t (*waveform)(uint32_t t);

unsigned int sim_irq_disabled;

static uint32_t sim_read_cyccnt(void) {
  now += 3;
  return now;
}

sim_m33_t sim_m33 = { 0, 0, sim_read_cyccnt };

uint32_t sim_gpio_in(void) {
  now += 1;
  return waveform(now);
}

typedef struct {
  uint32_t time, mask, state;
  unsigned int irq_disabled;
} record_t;

static record_t records[64];
static int nrecords;
static int late_edges;

void iec_set_lines(uint32_t mask, unsigned int state) {
  if (!mask)
    return;
  if (nrecords < 64)
    records[nrecords] = (record_t){ now, mask, state, sim_irq_disabled };
  nrecords++;
}

void set_test_led(uint8_t state) {
  if (state)
    late_edges++;
}

/* true if t is at or after reference, handles counter wrap */
static int after(uint32_t t, uint32_t reference) {
  return (int32_t)(t - reference) >= 0;
}

static void sim_start(uint32_t start, uint32_t (*wave)(uint32_t)) {
  now        = start;
  edge       = start + 100000;
  waveform   = wave;
  nrecords   = 0;
  late_edges = 0;
}

/* ---------- checks ---------- */

static int failures;

#define CHECK(cond, ...) do {                   \
    if (!(cond)) {                              \
      failures++;                               \
      printf("FAIL %s:%d: ", __func__, __LINE__); \
      printf(__VA_ARGS__);                      \
      printf("\n");                             \
    }                                           \
  } while (0)

/* ---------- waveforms ---------- */

/* ATN and clock high, data rises at the start edge */
static uint32_t wave_data_rise(uint32_t t) {
  return IEC_BIT_ATN | IEC_BIT_CLOCK | (after(t, edge) ? IEC_BIT_DATA : 0);
}

/* clock stays low, ATN falls at the start edge */
static uint32_t wave_atn_fall(uint32_t t) {
  return after(t, edge) ? 0 : IEC_BIT_ATN;
}

/* sender for the receive test: clock rises at the start edge, then */
/* each bit pair is on the bus in a window around its sample time   */
static const generic_2bit_t jiffy_receive_def = {
  .pairtimes = {185, 315, 425, 555},
  .clockbits = {4, 6, 3, 2},
  .databits  = {5, 7, 1, 0},
  .eorvalue  = 0xff
};

static uint8_t sent_byte;

static uint32_t wave_send_byte(uint32_t t) {
  uint32_t bus = IEC_BIT_ATN;
  uint8_t value = sent_byte ^ jiffy_receive_def.eorvalue;

  if (!after(t, edge))
    return bus;

  bus |= IEC_BIT_CLOCK;
  for (int i = 0; i < 4; i++) {
    uint32_t center = edge + jiffy_receive_def.pairtimes[i] * CYCLES_PER_TICK;

    if (after(t, center - 40 * CYCLES_PER_TICK) &&
        !after(t, center + 40 * CYCLES_PER_TICK)) {
      bus &= ~(IEC_BIT_CLOCK | IEC_BIT_DATA);
      if (value & (1 << jiffy_receive_def.clockbits[i]))
        bus |= IEC_BIT_CLOCK;
      if (value & (1 << jiffy_receive_def.databits[i]))
        bus |= IEC_BIT_DATA;
    }
  }
  return bus;
}

/* ---------- tests ---------- */

static const generic_2bit_t jiffy_send_def = {
  .pairtimes = {100, 200, 310, 410},
  .clockbits = {0, 2, 4, 6},
  .databits  = {1, 3, 5, 7},
  .eorvalue  = 0
};

/* find the records for a bit pair and check value and timing */
static void check_pair(int pair, uint32_t target, unsigned int clock, unsigned int data) {
  uint32_t seen = 0, first = 0;

  for (int r = 0; r < nrecords && r < 64; r++) {
    int32_t delta = (int32_t)(records[r].time - target);

    if (delta < -1 || delta > MAX_EDGE_DELAY)
      continue;

    if (!seen)
      first = records[r].time;
    CHECK(records[r].time == first, "pair %d: lines changed %u cycles apart",
          pair, records[r].time - first);
    CHECK(records[r].irq_disabled, "pair %d: interrupts enabled", pair);

    if (records[r].mask & IEC_OBIT_CLOCK) {
      seen |= 1;
      CHECK(records[r].state == clock, "pair %d: clock %u, expected %u",
            pair, records[r].state, clock);
    }
    if (records[r].mask & IEC_OBIT_DATA) {
      seen |= 2;
      CHECK(records[r].state == data, "pair %d: data %u, expected %u",
            pair, records[r].state, data);
    }
  }

  CHECK(seen == 3, "pair %d: lines changed near %u: %u", pair, target, seen);
  CHECK(after(first, target), "pair %d: edge %d cycles early",
        pair, (int)(target - first));
}

/* generic 2-bit send with a NO_WAIT change that only teardown applies */
static void test_send(uint32_t start, uint8_t byte) {
  int expected = 1;

  sim_start(start, wave_data_rise);
  sim_irq_disabled = 0;

  llfl_setup();
  llfl_wait_data(1, NO_ATNABORT);
  CHECK(now - edge < 20, "reference captured %u cycles after the edge", now - edge);

  llfl_generic_load_2bit(&jiffy_send_def, byte);
  llfl_set_clock_at(520, 1, NO_WAIT);
  llfl_teardown();

  CHECK(late_edges == 0, "%d late edges", late_edges);
  CHECK(sim_irq_disabled == 0, "interrupts not restored");

  /* lines with the same new level are changed with one write */
  for (int i = 0; i < 4; i++) {
    unsigned int clock = (byte >> jiffy_send_def.clockbits[i]) & 1;
    unsigned int data  = (byte >> jiffy_send_def.databits[i]) & 1;

    expected += clock == data ? 1 : 2;
    check_pair(i, edge + jiffy_send_def.pairtimes[i] * CYCLES_PER_TICK, clock, data);
  }
  CHECK(nrecords == expected, "%d line changes, expected %d", nrecords, expected);

  record_t *last = &records[nrecords - 1];
  CHECK(last->mask == IEC_OBIT_CLOCK && last->state == 1 &&
        after(last->time, edge + 520 * CYCLES_PER_TICK),
        "pending change not applied by teardown");
}

/* generic 2-bit receive */
static void test_receive(uint32_t start, uint8_t byte) {
  sim_start(start, wave_send_byte);
  sent_byte = byte;

  llfl_setup();
  llfl_wait_clock(1, ATNABORT);
  uint8_t received = llfl_generic_save_2bit(&jiffy_receive_def);
  llfl_teardown();

  CHECK(received == byte, "received %02x, expected %02x", received, byte);
  CHECK(late_edges == 0, "%d late edges", late_edges);
}

/* waiting for clock high returns when ATN falls with ATNABORT */
static void test_atn_abort(void) {
  sim_start(0, wave_atn_fall);

  llfl_setup();
  llfl_wait_clock(1, ATNABORT);
  llfl_teardown();

  CHECK(after(now, edge) && now - edge < 40, "returned %d cycles after ATN fell",
        (int)(now - edge));
}

/* a change requested for a time that has passed is flagged */
static void test_late_edge(void) {
  sim_start(0, wave_data_rise);

  llfl_setup();
  llfl_wait_data(1, NO_ATNABORT);
  now += 200 * CYCLES_PER_TICK;
  llfl_set_data_at(100, 0, WAIT);
  llfl_teardown();

  CHECK(late_edges == 1, "%d late edges, expected 1", late_edges);
  CHECK(nrecords == 1, "late change not applied");
}

/* a reference time set by the caller, as the FC3 sender does */
static void test_caller_reference(uint32_t start) {
  sim_start(start, wave_data_rise);

  llfl_setup();
  llfl_reference_time = llfl_now() + 10;
  uint32_t base = now;
  llfl_set_clock_at(0, 0, WAIT);
  llfl_set_clock_at(120, 1, WAIT);
  llfl_teardown();

  CHECK(nrecords == 2, "%d line changes", nrecords);
  /* the reference is rounded to 100ns, allow one unit */
  int32_t first = (int32_t)(records[0].time - base);
  int32_t second = (int32_t)(records[1].time - records[0].time);
  CHECK(first >= 10 * CYCLES_PER_TICK - CYCLES_PER_TICK - 3 &&
        first <= 10 * CYCLES_PER_TICK + MAX_EDGE_DELAY,
        "first change after %d cycles", first);
  CHECK(second >= 120 * CYCLES_PER_TICK - 3 &&
        second <= 120 * CYCLES_PER_TICK + MAX_EDGE_DELAY,
        "second change %d cycles later", second);
}

int main(void) {
  /* start times include cycle counter wrap during the transfer */
  static const uint32_t starts[] = { 0, 50000, 0xffff0000u, 0xfffe7965u };

  for (unsigned int s = 0; s < sizeof(starts) / sizeof(starts[0]); s++) {
    for (unsigned int byte = 0; byte < 256; byte++) {
      test_send(starts[s], byte);
      test_receive(starts[s], byte);
    }
    test_caller_reference(starts[s]);
  }
  test_atn_abort();
  test_late_edge();

  if (failures) {
    printf("%d failures\n", failures);
    return 1;
  }
  printf("llfl tests passed\n");
  return 0;
}
