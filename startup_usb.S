#include <xc.h>

.section .devcfg0,"ax",@progbits
.word 0xfffffbff
.text

.section .devcfg1,"ax",@progbits
.word 0x7f7f7cb9
.text

.section .devcfg2,"ax",@progbits
.word 0xfff99f29
.text

.section .devcfg3,"ax",@progbits
.word 0xc6ffc0de
.text

.globl __vector_dispatch_132
.section .vector_132,"awx"
.set noreorder
.align 2
.ent __vector_dispatch_132
__vector_dispatch_132:
        j usb_isr
        nop
.set reorder
.end __vector_dispatch_132
.size __vector_dispatch_132, .-__vector_dispatch_132


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
