; x64 dynarec generated assembly + comments reference

; $Imm: 16-bit immediate field
; $Simm: Sign-extended 16-bit immediate field
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