.globl TrapInstallHandler
TrapInstallHandler:
    mov r0, #0x00000028 @ Location of software interrupt
    ldr r1, =TrapEntry @ Address of software interrupt handler
    str r1, [r0] @ Install handler to interrupt controller
    bx lr

# This function is passed 2 parameters
# R0 contains the return value - Do not touch this register
# R1 contains the user's stack
.globl TrapReturn
TrapReturn:
    # Store kernel state
    stmfd sp!, {r4-r12, lr}

        # Switch to system mode
        msr cpsr_c, #0x9F

            # Restore user's stack pointer
            mov sp, r1

            # Restore the user state
            ldmfd sp!, {r2-r12, lr}

        # Switch back to supervisor mode
        msr cpsr_c, #0x93

        # Jump back to user mode
        msr spsr, r2
        movs pc, r3

.globl TrapEntry
TrapEntry:
        # Store function parameters that we are about to clobber
        stmfd sp!, {r2, r3}

            # User CPSR is in SPSR, and User PC is in LR
            mrs r2, spsr
            mov r3, lr

            # Switch to system mode
            msr cpsr_c, #0x9F

                # Store user registers
                stmfd sp!, {r2-r12, lr}

            # Switch back to supervisor mode
            msr cpsr_c, #0x93

        # Restore clobbered function parameters
        ldmfd sp!, {r2, r3}

        # Grab the swi instruction
        ldr r4, [lr, #-4]

        # Isolate the system call number
        # Take the complement of the mask, then perform an "AND"
        bic r4, r4, #0xFF000000

        # TODO: Make the system call
        mov r0, r4

    # Restore kernel state
    # This will jump back to whoever called TrapReturn()
    ldmfd sp!, {r4-r12, pc}

.globl GetCpsr
GetCpsr:
    mrs r0, cpsr
    bx lr

.globl GetR3
GetR3:
    mov r0, r1
    bx lr

.globl GetSPSR
GetSPSR:
    mrs r0, spsr
    bx lr

.globl GetSP
GetSP:
    mov r0, sp
    bx lr

.globl GetUserSP
GetUserSP:
    msr cpsr_c, #0x9F
    mov r0, sp
    msr cpsr_c, #0x93
    bx lr
