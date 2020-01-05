#ifndef _DSHOTASM_H_
#define _DSHOTASM_H_

#define DSHOT600_ASM(out_set, out_clr, data_ptr, end_byte, data_val, mask_val) \
  __asm__ __volatile__ (                                                \
                        /* Set all active bits high on port */          \
                        "L_%=: st Y, %[mask] \n\t"                      \
                        /* Load the set of bits to clear early (zeros) */ \
                        "ld %[data], X+ \n\t"                           \
                        /* wait */                                      \
                        "nop \n\t"                                      \
                        "nop \n\t"                                      \
                        "nop \n\t"                                      \
                        /* Clear zero bits... */                        \
                        "st Z, %[data] \n\t"                            \
                        /* Check if we have sent 16 bits */             \
                        "cp %[end], r26 \n\t"                           \
                        /* wait */                                      \
                        "nop \n\t"                                      \
                        "nop \n\t"                                      \
                        "nop \n\t"                                      \
                        "nop \n\t"                                      \
                        /* Clear all of the active bits now */          \
                        "st Z, %[mask] \n\t"                            \
                        /* wait */                                      \
                        "nop \n\t"                                      \
                        /* If there are more bits to send loop */       \
                        "brne L_%="                                     \
                        : /* no outputs */                              \
                        : [set]    "y"  (out_set),                      \
                          [clr]    "z"  (out_clr),                      \
                          [ptr]    "x"  (data_ptr),                     \
                          [mask]   "r"  (mask_val),                     \
                          [end]    "r"  (end_byte),                     \
                          [data]   "r"  (data_val));

#endif // _DSHOTASM_H_
