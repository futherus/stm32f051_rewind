.syntax unified

.section .text

.thumb_func
.global delay
delay:
    push {lr}

    cmp r0, #0
    beq end

loop:
    isb                 @ 4
    subs r0, r0, #1     @ 1
    bne loop            @ 3
 
end:
    pop {pc}

