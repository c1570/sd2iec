/* Host shim for atomic.h: interrupt state is only tracked */
extern unsigned int sim_irq_disabled;

static inline unsigned int __get_primask(void) { return sim_irq_disabled; }
static inline unsigned int __iCliRetVal(void)  { sim_irq_disabled = 1; return 1; }
static inline void __iRestore(const unsigned int *s) { sim_irq_disabled = *s; }
