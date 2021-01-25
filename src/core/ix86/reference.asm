; x64 dynarec generated assembly + comments reference

; $imm: 16-bit immediate field
; $simm: Sign-extended 16-bit immediate field
; $rt: Instruction's $rt register
; $rd: Instruction's $rd register
; = (const): Set a register to a value and mark as const

LUI: ; no generated assembly
    $rt = (const) $Imm << 16 

ORI:
    if const ($rs) ; no generated assembly
        $rt = (const) $rs | imm
    else
        mov $rt, $rs ; move the register that rs has been allocated to the register rt has been allocated
        or $rt, imm ; or with immediate

; TODO: Optimize like the 32-bit JIT but less horribly
SW:
    mov rax, &memWrite32Wrapper ; function pointer to write function in rax
    if const ($rs) ; address is constant
        mov rcx, ($rs + imm) ; move address to 1st param reg
    else
        mov rcx, $rs
        add rcx, imm
    
    if const ($rt) ; stored value is constant
        mov rdx, ($rt) ; move value to second param reg
    else
        mov rdx, $rt
        
    call rax ; execute the function

JR: 
    mov [registerPointer + pcIndex], newPC ; 
    