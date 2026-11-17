/* Host shim for the M33 registers: reading the cycle counter */
/* advances the simulated time, see llfltest.c                 */
typedef struct {
  uint32_t demcr, dwt_ctrl;
  uint32_t (*read_cyccnt)(void);
} sim_m33_t;

extern sim_m33_t sim_m33;

#define m33_hw (&sim_m33)
#define dwt_cyccnt read_cyccnt()
#define M33_DEMCR_TRCENA_BITS       (1u << 24)
#define M33_DWT_CTRL_CYCCNTENA_BITS 1u
