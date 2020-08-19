#include <xc.h>

.globl __vector_dispatch_default
.section .vector_default,"awx"
.set noreorder
.align 2
.ent __vector_dispatch_default
__vector_dispatch_default:
        // turn off backlight
        /*
        lui v0,0xBF86
        li  v1,0x000
        sw   v1,0x0330(v0)
        eret
        */
        la v0, __pic32_software_reset
        jalr v0
        nop
.set reorder
.end __vector_dispatch_default
.size __vector_dispatch_default, .-__vector_dispatch_default


.global _reset
.section .reset,"awx"
_reset:
        mfc0    k0,_CP0_STATUS
        ext     k0,k0,19,1              # Extract NMI bit
        beqz    k0,_no_nmi
        nop
        la      k0,_nmi_handler
        jr      k0
        nop

_no_nmi:
        la      sp,_stack
        la      gp,_gp
        mfc0    t1,_CP0_SRSCTL          # Read SRSCtl register
        add     t3,t1,zero              # Save off current SRSCtl

        li      t2,(8-1)

1:
        ins     t1,t2,6,4               # Put next shadow set field
        mtc0    t1,_CP0_SRSCTL          # into SRSCtl<PSS>
        ehb                             # Clear hazard before using new SRSCTL
        wrpgpr  gp,gp                   # Set global pointer in PSS

        addiu   t2,t2,-1                # Next lower shadow set
                                        # Loop for all sets
        bne     t2,$0,1b                # Down to zero (normal GPR set)
        nop

        mtc0    t3,_CP0_SRSCTL          # Restore SRSCtl
        ehb

        .extern __pic32_tlb_init_ebi_sqi
        la      t0,__pic32_tlb_init_ebi_sqi
        jalr    t0
        nop

_start_bss_init:
        la      t0,_bss_begin
        la      t1,_bss_end
        b       _bss_check
        nop

_bss_init:
        sw      zero,0x0(t0)
        addu    t0,4
_bss_check:
        bltu    t0,t1,_bss_init
        nop

        .extern   __pic32_init_cache
        la      t0,__pic32_init_cache
        jalr    t0
        nop

        la v0, __init_interrupt_controller
        jalr v0
        nop

        ##################################################################
        # Initialize CP0 registers
        ##################################################################
        # Initialize Count register
        ##################################################################
        mtc0    zero,_CP0_COUNT

        ##################################################################
        # Initialize Compare register
        ##################################################################
        li      t2,-1
        mtc0    t2,_CP0_COMPARE

        ##################################################################
        # Ensure BEV set and Initialize EBase register
        ##################################################################
        li      t0, (1<<22)
        mfc0    t2,_CP0_STATUS
        or      t2,t0,t2               # Set BEV bit 22
        mtc0    t2,_CP0_STATUS

        la      t1,_ebase_address
        ehb
        mtc0    t1,_CP0_EBASE


        la      t1,_vector_spacing
        li      t2,0                    # Clear t2 and
        ins     t2,t1,5,5               # shift value to VS field
        mtc0    t2,_CP0_INTCTL

        ##################################################################
        # Initialize CAUSE registers
        # - Enable counting of Count register <DC = 0>
        # - Use special exception vector <IV = 1>
        # - Clear pending software interrupts <IP1:IP0 = 0>
        ##################################################################
        li      t1,0x00800000
        mtc0    t1,_CP0_CAUSE

        ##################################################################
        # Initialize STATUS register
        # - Access to Coprocessor 0 not allowed in user mode <CU0 = 0>
        # - User mode uses configured endianness <RE = 0>
        # - Preserve Bootstrap Exception vectors <BEV>
        # - Preserve soft reset <SR> and non-maskable interrupt <NMI>
        # - CorExtend enabled based on whether CorExtend User Defined
        #   Instructions have been implemented <CEE = Config<UDI>>
        # - Disable any pending interrupts <IM7..IM2 = 0, IM1..IM0 = 0>
        # - Disable hardware interrupts <IPL7:IPL2 = 0>
        # - Base mode is Kernel mode <UM = 0>
        # - Error level is normal <ERL = 0>
        # - Exception level is normal <EXL = 0>
        # - Interrupts are disabled <IE = 0>
        # - DSPr2 ASE is enabled for devices that support it <MX = 1>
        # - FPU64 is enabled for devices that support it <CU1=1> & <FR=1>
        ##################################################################
        mfc0    t0,_CP0_CONFIG
        ext     t1,t0,22,1              # Extract UDI from Config register
        sll     t1,t1,17                # Move UDI to Status.CEE location
        mfc0    t0,_CP0_STATUS
        and     t0,t0,0x00580000        # Preserve SR, NMI, and BEV

        li      t2, 0x01000000          # Set the Status.MX bit to enable DSP
        or      t0,t2,t0

        ehb

        ##################################################################
        # Initialize Status<BEV> for normal exception vectors
        ##################################################################
        mfc0    t0,_CP0_STATUS
        and     t0,t0,0xffbfffff        # Clear BEV
        mtc0    t0,_CP0_STATUS

        ##################################################################
        # Call main. We do this via a thunk in the text section so that
        # a normal jump and link can be used, enabling the startup code
        # to work properly whether main is written in MIPS16 or MIPS32
        # code. I.e., the linker will correctly adjust the JAL to JALX if
        # necessary
        ##################################################################
        and     a0,a0,0
        and     a1,a1,0
        la      t0,_main_entry
        jr      t0
        nop

        ##################################################################
        # Boot Exception Vector Handler
        # Jumps to _bootstrap_exception_handler
        ##################################################################
        .section .bev_handler,"ax"
        .align 2
        .set noreorder
        .ent _bev_exception
_bev_exception:
        la        k0,_bootstrap_exception_handler
        jr        k0
        nop

        .end _bev_exception

        ##################################################################
        # General Exception Vector Handler
        # Jumps to _general_exception_context
        ##################################################################
        .section .gen_handler,"ax"
        .align 2
        .set noreorder
        .ent _gen_exception
_gen_exception:
0:      la      k0,_general_exception_context
        jr      k0
        nop

        .end _gen_exception

        ##################################################################
        # Simple TLB-Refill Exception Vector
        # Jumps to _simple_tlb_refill_exception_context
        ##################################################################
        .section .simple_tlb_refill_vector,"ax"
        .align 2
        .set noreorder
        .ent simple_tlb_refill_vector
simple_tlb_refill_vector:
        la      k0,_simple_tlb_refill_exception_context
        jr      k0
        nop

        .end simple_tlb_refill_vector


        ##################################################################
        # Cache-Error Exception Vector Handler
        # Jumps to _cache_err_exception_context
        ##################################################################
        .section .cache_err_vector,"ax"
        .align 2
        .set noreorder
        .ent _cache_err_vector
_cache_err_vector:
        la      k0,_cache_err_exception_context
        jr      k0
        nop

        .end _cache_err_vector


        .section .text.main_entry,"ax"
        .align 2
        .ent _main_entry
_main_entry:
        .weak _init
        # call .init section to run constructors etc
        lui     a0,%hi(_init)
        addiu   sp,sp,-24
        addiu   a0,a0,%lo(_init)
        beq     a0,$0,2f
        sw      $31,20(sp)	 #,
        jalr    a0
        nop
2:
        and     a0,a0,0
        and     a1,a1,0

        ##################################################################
        # Call main
        ##################################################################
        la      t0,main
        jalr    t0
        nop
        .end _main_entry
