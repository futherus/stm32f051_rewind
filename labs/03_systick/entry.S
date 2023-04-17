.syntax unified

.section .text

.align 4
.thumb_func
.global __reset_handler
__reset_handler:
	// Copy .data section to SRAM:
	ldr r0, __data_start_lma_val
	ldr r1, __data_start_vma_val
	ldr r2, __data_end_vma_val
	bl __memcpy32

	// Copy .bss section to SRAM:
	movs r0, #0
	ldr r1, __bss_start_vma_val
	ldr r2, __bss_end_vma_val
	bl __memset32

	// Execute actual software:
    blx main

__halt:
	b __halt
__exc_handler:
	b __exc_handler

// Copies memory [src:end) to dst
// Entry: r0 -- src
//		  r1 -- dst
//		  r2 -- end
// Destroys: r0 r1 r4
__memcpy32:
.Lcpyloop:
	cmp r1, r2
	beq .Lcpyend

	ldr r4, [r0, #0]
	str r4, [r1, #0]

	adds r0, r0, #4
	adds r1, r1, #4	

	b .Lcpyloop

.Lcpyend:
	bx lr

// Sets memory [src:end) with 4-byte value
// Entry: r0 -- 4-byte value
//		  r1 -- dst
//		  r2 -- end
// Destroys: r1
__memset32:
.Lloop:
	cmp r1, r2
	beq .Lend

	str r0, [r1, #0]
	adds r1, r1, #4	

	b .Lloop

.Lend:
	bx lr

__data_start_lma_val:
.word __data_start_lma
__data_start_vma_val:
.word __data_start_vma
__data_end_vma_val:
.word __data_end_vma

__bss_start_vma_val:
.word __bss_start_vma
__bss_end_vma_val:
.word __bss_end_vma

.section .vector_table
.word __stack_start         // Initial SP
.word __reset_handler       // Reset Handler
.word __exc_handler         // NMI Handler
.word __exc_handler         // Hard Fault Handler
.fill 7, 4, 0x00            // Reserved
.word __exc_handler         // SVCall
.fill 2, 4, 0x00            // Reserved
.word __exc_handler         // PendSV
.word systick_handler       // SysTick
