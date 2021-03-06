// Copyright 2020 Thomas DeWeese
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef _DSHOTASM_H_
#define _DSHOTASM_H_

// For DSHOT 600 there is 1.67uS between pulses (rise to rise).
// For zero pulse width is .625uS, for a one width is 1.25uS (2x zero)
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


// For DSHOT 600 there is 1.67uS between pulses (rise to rise).
// For zero pulse width is .625uS, for a one width is 1.25uS (2x zero)
#define DSHOT600_ASM_PDIV1(out_set, out_clr, data_ptr, end_byte, data_val, mask_val) \
  __asm__ __volatile__ (                                                \
                        /* Set all active bits high on port */          \
                        "L_%=: st Y, %[mask] \n\t"                      \
                        /* Load the set of bits to clear early (zeros) */ \
                        "ld %[data], X+ \n\t"                           \
                        /* wait */                                      \
                        "nop \n\t"                                      \
			"nop \n\t"                                      \
                        "nop \n\t"                                      \
                        "nop \n\t"                                      \
                        "nop \n\t"                                      \
                        "nop \n\t"                                      \
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
                        "nop \n\t"                                      \
                        "nop \n\t"                                      \
                        "nop \n\t"                                      \
                        "nop \n\t"                                      \
                        "nop \n\t"                                      \
                        "nop \n\t"                                      \
                        "nop \n\t"                                      \
                        /* Clear all of the active bits now */          \
                        "st Z, %[mask] \n\t"                            \
                        /* wait */                                      \
                        "nop \n\t"                                      \
                        "nop \n\t"                                      \
                        "nop \n\t"                                      \
                        "nop \n\t"                                      \
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

// For DSHOT 1200 (1,200,000 bps) there is 833nS between pulses (rise
// to rise).  Gor a one plus width is 625nS (75% of peak-to-peak
// time), zero pulse width is 313nS (half of one).
#define DSHOT1200_ASM_PDIV1(out_set, out_clr, data_ptr, end_byte, data_val, mask_val) \
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
