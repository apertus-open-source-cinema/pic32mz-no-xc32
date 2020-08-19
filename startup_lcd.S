#include <xc.h>

.section .devcfg0,"ax",@progbits
.word 0xfffffbff
.text

.section .devcfg1,"ax",@progbits
.word 0x7f7f7bb9
.text

.section .devcfg2,"ax",@progbits
.word 0x7ffae399
.text

.section .devcfg3,"ax",@progbits
.word 0x86ffc0de
.text



.globl __vector_dispatch_146
.section .vector_146,"awx"
.set noreorder
.align 2
.ent __vector_dispatch_146
__vector_dispatch_146:
        j uart2_isr
        nop
.set reorder
.end __vector_dispatch_146
.size __vector_dispatch_146, .-__vector_dispatch_146
