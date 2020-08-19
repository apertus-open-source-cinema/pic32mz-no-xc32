tmpl = """
#include <xc.h>

.section .text.init_interrupt_controller,"ax"
.globl __init_interrupt_controller
.align 2
.ent __init_interrupt_controller
__init_interrupt_controller:
{}
    jr ra
    nop
.end __init_interrupt_controller
""".strip()

contents = ""

base = 0xBF810540

for i in range(255):
    contents += "    li t0, {}\n".format(hex(base + 4 * i))
    contents += "    .extern __vector_offset_{}\n".format(i)
    contents += "    la t1, __vector_offset_{}\n".format(i)
    contents += "    sw t1, 0(t0)\n"

contents = "    " + contents.strip()

print(tmpl.format(contents))
