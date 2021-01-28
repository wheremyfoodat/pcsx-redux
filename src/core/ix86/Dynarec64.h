#pragma once
#include "core/debug.h"
#include "core/disr3000a.h"
#include "core/gpu.h"
#include "core/gte.h"
#include "core/psxemulator.h"
#include "core/r3000a.h"
#include "core/system.h"

// Wrapper functions (these need to be global)

/// Write an 8-bit value to memory[mem]
void psxMemWrite8Wrapper(uint32_t address, uint8_t value) {
    PCSX::g_emulator->m_psxMem->psxMemWrite8(address, value);
    //printf("Wrote %02X to %08X\n", value, address);
}

/// Write a 16-bit value to memory[mem]
void psxMemWrite16Wrapper(uint32_t address, uint16_t value) {
    PCSX::g_emulator->m_psxMem->psxMemWrite8(address, value);
    //printf("Wrote %04X to %08X\n", value, address);
}

/// Write a 32-bit value to memory[mem]
void psxMemWrite32Wrapper(uint32_t address, uint32_t value) {
    PCSX::g_emulator->m_psxMem->psxMemWrite32(address, value);
    //printf("Wrote %08X to %08X\n", value, address);
}

/// Read an 8-bit value from memory[mem]
uint8_t psxMemRead8Wrapper(uint32_t address) {
    auto val = PCSX::g_emulator->m_psxMem->psxMemRead8(address);
    //printf("Read %02X from %08X\n", val, address);
    return val;
}

/// Read a 32-bit value from memory[mem]
uint32_t psxMemRead32Wrapper(uint32_t address) {
    auto val = PCSX::g_emulator->m_psxMem->psxMemRead32(address);
    printf("Read %08X from %08X\n", val, address);
    return val;
}

class X86DynaRecCPU : public PCSX::R3000Acpu {
    const int KILOYBTE = 1024; // 1 kilobyte is 1024 bytes
    const int MEGABYTE = 1024 * KILOYBTE; // 1 megabyte is 1024 kilobytes
    
    const uint32_t PC_OFFSET = (uintptr_t) &m_psxRegs.pc - (uintptr_t) &m_psxRegs; // the offset of the PC in the registers struct
    const uint32_t COP0_REGS_OFFSET = (uintptr_t) &m_psxRegs.CP0 - (uintptr_t) &m_psxRegs; // the offset of the cop0 regs in the register structs
    const uint32_t CAUSE_OFFSET = (uintptr_t) &m_psxRegs.CP0.r[13] - (uintptr_t) &m_psxRegs; // the offset of the CAUSE reg in the register struct
    const uint32_t REG_CACHE_OFFSET = (uintptr_t) &m_psxRegs.hostRegisterCache - (uintptr_t) &m_psxRegs; // the offset of the cached host registers in the register struct
    const uint32_t CYCLE_OFFSET = (uintptr_t) &m_psxRegs.cycle - (uintptr_t) &m_psxRegs; // the offset of the cycle variable in the register struct

    typedef void (X86DynaRecCPU::*FunctionPointer)(); // Define a "Function Pointer" type to make our life easier
    typedef void (*JITCallback)(); // A function pointer to JIT-emitted code

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
    const Reg64 registerPointer = rbp;  // rbp will be used as a pointer to the register array

    /* 
       The register our JIT can allocate.
       Starting by the available **non-volatile** regs, since those can be pushed and popped just once per block
       Then follow the volatile regs, which can sadly be bonked by any C-interop, which is why they're not preferred
    */
    const std::array <Reg32, 11> allocateableRegisters = { r12d, r13d, r14d, r15d, edi, esi, ebx, r8d, r9d, r10d, r11d};

    const Reg32 arg1 = ecx; // register where first arg is stored
    const Reg32 arg2 = edx; // register where second arg is stored
    const Reg32 arg3 = r8d; // register where third arg is stored
    const Reg32 arg4 = r9d; // register where fourth arg is stored
#else
#error "x64 JIT not supported outside of Windows"
#endif
    unsigned int allocatedRegisters = 0; // how many registers have been allocated in this block?

public:
    X86DynaRecCPU() : gen (REC_MEMORY_SIZE) { Init(); } // initialize JIT and emitter
// interface methods
    virtual bool isDynarec() final { return true; } // This dynarec is a dynarec, yes
    inline bool Implemented() final { return true; }  // This is implemented in 64-bit mode
    virtual void Execute() final{ while (hasToRun()) execute(); }
    virtual void Reset() final { printf("Add resetting to x64 JIT!\n"); }
    virtual void Shutdown() final { printf("Add shutdown to x64 JIT\n"); }
    virtual void SetPGXPMode(uint32_t pgxpMode) final { printf("PGXP stuff in 64-bit JIT\n"); }
   
// backend methods
  private:
    inline bool isPcValid(uint32_t addr) { return validBlockLUT[addr >> 16]; } // Check if the block is at a valid memory address by peeking at the top 16 bits
                                                                                          // This will return true if in WRAM, or BIOS
    inline bool isConst(unsigned reg) {  // for constant propagation, to check if we know a reg value at compile time
        return registers[reg].state == Constant;
    }
    
    /// invalidate block
    virtual void Clear(uint32_t addr, uint32_t size) final { 
        if ((addr >> 16) == (m_psxRegs.pc >> 16)) {  // if write is on the same page as the PC
            printf("Write to the PC page. Fuck... (SMC?)\n");
        }

        auto block = getBlockPointer (addr); // get address of block
        *block = 0; // mark block as uncompiled

        //printf("Fix page invalidation to x64 JIT\n"); 
    };  

    virtual bool Init() final {
        printf("Initializing x64 JIT...\n");
        recRAM = (uintptr_t*) calloc(0x200000, sizeof(uintptr_t*)); // initialize recompiler RAM
        recROM = (uintptr_t*) calloc(0x080000, sizeof(uintptr_t*)); // initialize recompiler ROM
        blocks = gen.getCode(); // Our code buffer

        validBlockLUT = (uintptr_t*) calloc (0x10000, sizeof(uintptr_t*));
        for (auto i = 0; i < 0x80; i++) { // map WRAM to the LUT
            auto ptr = (uintptr_t) &recRAM[(i & 0x1f) << 16];
            validBlockLUT[i] = ptr; // KUSEG
            validBlockLUT[i + 0x8000] = ptr; // KSEG0
            validBlockLUT[i + 0xA000] = ptr; // KSEG1
        }

        for (auto i = 0; i < 0x08; i++) { // map BIOS
            auto ptr = (uintptr_t) &recROM[i << 16];
            validBlockLUT[i + 0x1FC0] = ptr; // KUSEG
            validBlockLUT[i + 0x9FC0] = ptr; // KSEG0
            validBlockLUT[i + 0xBFC0] = ptr; // KSEG1
        }
        
        assert((uintptr_t)&m_psxRegs == (uintptr_t)&m_psxRegs.GPR.r[0]);
        return true; 
    }

    // for constant propagation: shows if a register is constant or not
    enum RegState { Unknown, Constant };

    // A struct that holds a register's info
    struct Register {
        uint32_t val = 0;    // the register's cached value used in const propagation (TODO: Add initial GP/FP)
        RegState state = Constant;  // is this const or not? (Assume constant, set to 0, on boot)
        Reg32 allocatedReg; // Which host reg has this guest reg been allocated to? The JIT performs register allocation, that means the MIPS reg $t0 might be cached in our x86 "edx" reg, and so on
        bool allocated = false; // Has this register been allocated to a host reg?
    };

    Register registers[32];  // the 32 guest registers

    const std::array <std::string, 11> allocateableRegNames = { "r12", "r13", "r14", "r15", "edi", "esi", "ebx", "r8", "r9", "r10", "r11" }; // x64 register strings for debugging
    const std::array <std::string, 32> guestRegNames = {"$zero", "$at", "$v0", "$v1", "$a0", "$a1", "$a2", "$a3", "$t0", "$t1", "$t2", "$t3", "$t4",
                                                        "$t5", "$t6", "$t7", "$s0", "$s1", "$s2", "$s3", "$s4", "$s5", "$s6", "$s7", "$t8", "$t9", "$k0",
                                                        "$k1", "$gp", "$sp", "$fp", "$ra" };

    uintptr_t* validBlockLUT; // shows which blocks are valid
    const uint8_t* blocks;  // contains the compiled x64 code
    uintptr_t* recROM;  // pointers to the compiled x64 BIOS code
    uintptr_t* recRAM;  // pointers to the compiled x64 WRAM code

    uint32_t recPC;                             // Points to the instruction we're compiling
    const int REC_MEMORY_SIZE = 32 * MEGABYTE;  // how big our x64 code buffer is. This is 132B vs the 32-bit JIT's 8MB,
                                                // just to be safe, and since x64 code tends to be longer + 8MB is tiny
    const int MAX_BLOCK_SIZE = 50;              // Max MIPS instructions per block. This will prolly get raised.
    CodeGenerator gen; // x64 emitter

    bool compiling = true;  // Are we compiling code right now?

    const FunctionPointer recBasic [64] = { // Function pointer table to the compilation functions for basic instructions
        &X86DynaRecCPU::recompileSpecial, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recJ, &X86DynaRecCPU::recJAL,  // 00
        &X86DynaRecCPU::recBEQ, &X86DynaRecCPU::recBNE, &X86DynaRecCPU::recBLEZ, &X86DynaRecCPU::recBGTZ,  // 04
        &X86DynaRecCPU::recADDIU, &X86DynaRecCPU::recADDIU, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 08
        &X86DynaRecCPU::recANDI, &X86DynaRecCPU::recORI,  &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recLUI,  // 0c
        &X86DynaRecCPU::recCOP0, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 10
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 14
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 18
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 1c
        &X86DynaRecCPU::recLB, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recLW,  // 20
        &X86DynaRecCPU::recLBU, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 24
        &X86DynaRecCPU::recSB, &X86DynaRecCPU::recSH, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recSW,  // 28
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 2c
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 30
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 34
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 38
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 3c
    };

    const FunctionPointer recSpecial [64] = { // Function pointer table to the compilation functions for special instructions
        &X86DynaRecCPU::recSLL, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 00
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 04
        &X86DynaRecCPU::recJR, &X86DynaRecCPU::recJALR, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 08
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 0c
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 10
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 14
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 18
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 1c
        &X86DynaRecCPU::recADDU, &X86DynaRecCPU::recADDU, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 20
        &X86DynaRecCPU::recAND, &X86DynaRecCPU::recOR, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 24
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recSLTU,  // 28
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 2c
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 30
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 34
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 38
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 3c
    };

    /// Mark registers[reg] as constant, with a value of "value"
    inline void markConst(int reg, uint32_t value) {
        registers[reg].state = Constant; // mark constant
        registers[reg].allocated = false; // unallocate register
        registers[reg].val = value;
    }

    /// Allocate a MIPS register to an x64 register
    inline void allocateReg (unsigned regNumber) {
        if (registers[regNumber].allocated) return; // if the register has been allocated, exit
        printf ("Allocated %s to %s\n", guestRegNames[allocatedRegisters].c_str(), allocateableRegNames[allocatedRegisters].c_str());

        registers[regNumber].allocated = true; // mark reg as allocated
        
        // Todo: Add support for allocating volatiles, as flushing is *really* slow
        if (allocatedRegisters >= 7) {
            printf("Ran out of non-volatiles! Flushing non-volatiles!\nTODO: Add support for allocating volatiles!\n");
            
            for (auto i = 1; i < 32; i++) {
                if (registers[i].allocated) {// back up every single allocated reg
                    gen.mov (dword [rbp + i * 4], registers[i].allocatedReg); // flush allocated reg to memory
                    registers[i].allocated = false; // this reg is no longer allocated
                }
            }

            // restore non-volatiles...
            gen.mov(rbx, qword[rbp + REG_CACHE_OFFSET + 48]);
            gen.mov(rsi, qword[rbp + REG_CACHE_OFFSET + 40]);
            gen.mov(rdi, qword[rbp + REG_CACHE_OFFSET + 32]);
            gen.mov(r15, qword[rbp + REG_CACHE_OFFSET + 24]);
            gen.mov(r14, qword[rbp + REG_CACHE_OFFSET + 16]);
            gen.mov(r13, qword[rbp + REG_CACHE_OFFSET + 8]);
            gen.mov(r12, qword[rbp + REG_CACHE_OFFSET]);
            allocatedRegisters = 0;
        }

        registers[regNumber].allocatedReg = allocateableRegisters[allocatedRegisters++]; // allocate a host reg, increment the amount of regs that's been alloc'd
        
        switch (allocatedRegisters) { // if non-volatile, preserve before allocating
            case 1: gen.mov(r12, qword[rbp + REG_CACHE_OFFSET]); break;
            case 2: gen.mov(r13, qword[rbp + REG_CACHE_OFFSET + 8]); break;
            case 3: gen.mov(r14, qword[rbp + REG_CACHE_OFFSET + 16]); break;
            case 4: gen.mov(r15, qword[rbp + REG_CACHE_OFFSET + 24]); break;
            case 5: gen.mov(rdi, qword[rbp + REG_CACHE_OFFSET + 32]); break;
            case 6: gen.mov(rsi, qword[rbp + REG_CACHE_OFFSET + 40]); break;
            case 7: gen.mov(rbx, qword[rbp + REG_CACHE_OFFSET + 48]); break;
        }

        if (isConst(regNumber)) {  // if the register is constant, load it from the constant regs and mark it as non-constant
            gen.mov(registers[regNumber].allocatedReg, registers[regNumber].val);
            registers[regNumber].state = Unknown;
        }
        
        else
            gen.mov (registers[regNumber].allocatedReg, dword [rbp + regNumber * 4]); // load the cached reg to the host reg
    }

    /// Params: A program counter value
    /// Returns: A pointer to the host x64 code that points to the block that starts from the given PC
    inline uintptr_t* getBlockPointer (uint32_t pc) {
        uintptr_t base = validBlockLUT[pc >> 16];
        uintptr_t offset = pc & 0xFFFF;
        uintptr_t* pointer = (uintptr_t*) (base + offset * sizeof(uintptr_t));

        return pointer;
    }

    /// get the size of all the current compiled instructions
    uintptr_t getBufferIndex() { 
        return (uintptr_t) gen.getCurr() - (uintptr_t) blocks;
    }

    /// Run the JIT
    void execute() {
        auto blockPointer = getBlockPointer(m_psxRegs.pc); // pointer to the current x64 block
        if ((uintptr_t*) *blockPointer == nullptr) { // if the block hasn't been compiled
            printf("Compiling block\n");
            printf("PC: %08X\n", m_psxRegs.pc);
            recompileBlock(blockPointer); // compile a block, set block pointer to the address of the block
        }
        
        auto emittedCode = (JITCallback) *blockPointer; // function pointer to the start of the block
        (*emittedCode)(); // call emitted code
        printf("PC: %08X\n", m_psxRegs.pc);
        // printRegs();
    }

    /// Compile a MIPS block
    /// Params: blockPointer -> The address to store the start of the current block
    void recompileBlock (uintptr_t*& blockPointer) {
        compiling = true; // Tell the recompiler that we are, in fact, recompiling.
        assert(getBufferIndex() < REC_MEMORY_SIZE);  // check that we haven't overflowed our code buffer
        printf("Align me!\n");                           // TODO: Alignment
        uint32_t* instructionPointer;                    // pointer to the instruction to compile

        recPC = m_psxRegs.pc;    // the PC of the recompiler
        printf ("Compiling block: Rec PC: %08X\n", recPC);
        uint32_t oldPC = recPC;  // the PC at the start of the block
        uintptr_t blockStart = (uintptr_t) gen.getCurr(); // the address the current block starts from
        
        printf("Current buffer address: %pX\n", blockStart);
        *blockPointer = (uintptr_t) blockStart; // Add the block to the block cache
     
        gen.push(rbp); // rbp is used as a pointer to the register struct, so we need to back it up
        gen.mov(rbp, (uint64_t) &m_psxRegs); // store the pointer in rbp
        gen.sub(rsp, 100); // fix up stack frame so that the return address is not overwritten
        auto compiledInstructions = 0;  // how many instructions we've compiled in this block

        while (shouldContinueCompiling()) {
            m_inDelaySlot = m_nextIsDelaySlot; // handle delay slot jazz
            m_nextIsDelaySlot = false;

            instructionPointer = (uint32_t*) PSXM(recPC);  // get pointer to instruction
            assert(instructionPointer != NULL);           // check the pointer is not null
            m_psxRegs.code = *instructionPointer;         // read the instruction
            recPC += 4;  // increment PC by sizeof(instruction)

            printf("Read instruction %08X from address %08X\n", m_psxRegs.code, recPC);
            (*this.*recBasic[m_psxRegs.code >> 26])();        // Call the function to compile the instruction (The >> 26 is to fetch the ocpode)
            compiledInstructions++;  // increment the compiled instructions counter
        }

        gen.add(dword [rbp + CYCLE_OFFSET], compiledInstructions); // add 1 cycle for each compiled instruction that was executed
        gen.add(rsp, 100); // undo the sub
        flushRegs(); // flush the cached registers
        gen.pop(rbp); // restore rbp
        gen.ret(); // emit a RET to return from the JIT 
    }

    /// Check whether to continue compiling or not
    bool shouldContinueCompiling() { 
        return m_nextIsDelaySlot || compiling;
    }

    /// Set regs to non-constant, unallocate host regs.
    void flushRegs() { 
        for (auto i = 1; i < 32; i++) 
            flushReg(i); // flush every reg except from $zero since that doesn't need flushing
        
        switch (allocatedRegisters) { // restore volatiles, abusing fallthrough between switches (Duff's device pattern)
            default:
            case 7: gen.mov(rbx, qword[rbp + REG_CACHE_OFFSET + 48]);
            case 6: gen.mov(rsi, qword[rbp + REG_CACHE_OFFSET + 40]);
            case 5: gen.mov(rdi, qword[rbp + REG_CACHE_OFFSET + 32]);
            case 4: gen.mov(r15, qword[rbp + REG_CACHE_OFFSET + 24]);
            case 3: gen.mov(r14, qword[rbp + REG_CACHE_OFFSET + 16]);
            case 2: gen.mov(r13, qword[rbp + REG_CACHE_OFFSET + 8]);
            case 1: gen.mov(r12, qword[rbp + REG_CACHE_OFFSET]);
            case 0: break;
        }

        markConst(0, 0); // some instruction might have marked $zero as non-const, fix this
        allocatedRegisters = 0; // we don't have any allocated regs anymore
    }

    void flushReg(int reg) { 
        if (isConst(reg)) {
            gen.mov(dword [rbp + (reg * 4)], registers[reg].val); // back up const regs to memory
            registers[reg].state = Unknown; // mark as non-const
        }

        else if (registers[reg].allocated) {
            gen.mov(dword[rbp + (reg * 4)], registers[reg].allocatedReg);  // store allocated regs into memory
            registers[reg].allocated = false;                              // mark them as non-allocated
        }
    }

    /// Compile the "special" (opcode == 0) instructions
    void recompileSpecial() {
        (*this.*recSpecial[m_psxRegs.code & 0x3F])();   // Call the function to compile the instruction (The >> 26 is to fetch the subfunction)
    }

    /// Compile a reserved or unimplemented instruction
    void recNULL() {
        printf("Compiling unknown instruction %08X\n", m_psxRegs.code);
        printf("Opcode: %02X\n", m_psxRegs.code >> 26);
        printf("PC: %08X\n", recPC);
        exit(1);
    }

    /// Compile a reserved or unimplemented special instruction
    void recNULLSpecial() { 
        printf("Compiling unknown special instruction %08X\n", m_psxRegs.code);
        printf("Subfunction: %02X\n", m_psxRegs.code & 0x3F);
        printf("PC: %08X\n", recPC);
        exit(1);
    }
    
    void recLUI() {
        if (!_Rt_) return; // don't compile if NOP

        auto value = _ImmU_ << 16; // fetch immediate, shift to the left by 16
        markConst (_Rt_, value); // Note: LUI *always* produces a constant result
    }

    void recORI() {
        if (!_Rt_) return; // don't compile if NOP

        if (isConst(_Rs_)) // if Rs is const, mark Rt as const too
            markConst (_Rt_, registers[_Rs_].val | _ImmU_);
        else {
            allocateReg (_Rs_);
            allocateReg (_Rt_);
            gen.mov (registers[_Rt_].allocatedReg, registers[_Rs_].allocatedReg); // mov rt, rs
            gen.or_ (registers[_Rt_].allocatedReg, _ImmU_); // or $rt, imm
        }
    }

    void recANDI() {
        if (!_Rt_) return; // don't compile if NOP

        if (isConst(_Rs_)) // if Rs is const, mark Rt as const too
            markConst (_Rt_, registers[_Rs_].val & _ImmU_);
        else {
            allocateReg (_Rs_);
            allocateReg (_Rt_);
            gen.mov (registers[_Rt_].allocatedReg, registers[_Rs_].allocatedReg); // mov rt, rs
            gen.and_ (registers[_Rt_].allocatedReg, _ImmU_); // and $rt, imm
        }
    }

    // TODO: Optimize
    void recSB() { 
        assert (8 > allocatedRegisters); // assert that we're not trampling any allocated regs
        gen.mov (rax, (uint64_t) &psxMemWrite8Wrapper); // function pointer in rax

        if (isConst(_Rs_))
            gen.mov (arg1, registers[_Rs_].val + _Imm_); // address in arg1
        else {
            allocateReg(_Rs_);
            gen.mov (arg1, registers[_Rs_].allocatedReg); // arg1 = $rs
            gen.add (arg1, _Imm_); // arg1 += imm
        }

        if (isConst(_Rt_))
            gen.mov (arg2, registers[_Rt_].val); // value in arg2
        else {
            allocateReg(_Rt_);
            gen.mov (arg2, registers[_Rt_].allocatedReg);
        }

        gen.call (rax); // call wrapper
    }

    // TODO: Optimize
    void recSH() { 
        assert (8 > allocatedRegisters); // assert that we're not trampling any allocated regs
        gen.mov (rax, (uint64_t) &psxMemWrite16Wrapper); // function pointer in rax

        if (isConst(_Rs_))
            gen.mov (arg1, registers[_Rs_].val + _Imm_); // address in arg1
        else {
            allocateReg(_Rs_);
            gen.mov (arg1, registers[_Rs_].allocatedReg); // arg1 = $rs
            gen.add (arg1, _Imm_); // arg1 += imm
        }

        if (isConst(_Rt_))
            gen.mov (arg2, registers[_Rt_].val); // value in arg2
        else {
            allocateReg(_Rt_);
            gen.mov (arg2, registers[_Rt_].allocatedReg);
        }

        gen.call (rax); // call wrapper
    }

    // TODO: Optimize
    void recSW() { 
        assert (8 > allocatedRegisters); // assert that we're not trampling any allocated regs
        gen.mov (rax, (uint64_t) &psxMemWrite32Wrapper); // function pointer in rax

        if (isConst(_Rs_))
            gen.mov (arg1, registers[_Rs_].val + _Imm_); // address in arg1
        else {
            allocateReg(_Rs_);
            gen.mov (arg1, registers[_Rs_].allocatedReg); // arg1 = $rs
            gen.add (arg1, _Imm_); // arg1 += imm
        }

        if (isConst(_Rt_))
            gen.mov (arg2, registers[_Rt_].val); // value in arg2
        else {
            allocateReg(_Rt_);
            gen.mov (arg2, registers[_Rt_].allocatedReg);
        }

        gen.call (rax); // call wrapper
    }

    // TODO: Optimize
    void recLB() { 
        if (!_Rt_) return; // don't compile if NOP
        assert (8 > allocatedRegisters); // assert that we're not trampling any allocated regs
        gen.mov (rax, (uint64_t) &psxMemRead8Wrapper); // function pointer in rax
        allocateReg(_Rt_); // allocate rt and mark as non const

        if (isConst(_Rs_))
            gen.mov (arg1, registers[_Rs_].val + _Imm_); // address in arg1
        else {
            allocateReg(_Rs_);
            gen.mov (arg1, registers[_Rs_].allocatedReg); // arg1 = $rs
            gen.add (arg1, _Imm_); // arg1 += imm
        }

        gen.call (rax); // call wrapper
        gen.movsx (registers[_Rt_].allocatedReg, al); // move sign extended result to rt
    }

    // TODO: Optimize
    void recLBU() { // Literally LB with zero extension
        if (!_Rt_) return; // don't compile if NOP
        assert (8 > allocatedRegisters); // assert that we're not trampling any allocated regs
        gen.mov (rax, (uint64_t) &psxMemRead8Wrapper); // function pointer in rax
        allocateReg(_Rt_); // allocate rt and mark as non const

        if (isConst(_Rs_))
            gen.mov (arg1, registers[_Rs_].val + _Imm_); // address in arg1
        else {
            allocateReg(_Rs_);
            gen.mov (arg1, registers[_Rs_].allocatedReg); // arg1 = $rs
            gen.add (arg1, _Imm_); // arg1 += imm
        }

        gen.call (rax); // call wrapper
        gen.movzx (registers[_Rt_].allocatedReg, al); // move sign extended result to rt
    }

    // TODO: Optimize
    void recLW() { 
        if (!_Rt_) return; // don't compile if NOP
        assert (8 > allocatedRegisters); // assert that we're not trampling any allocated regs
        gen.mov (rax, (uint64_t) &psxMemRead32Wrapper); // function pointer in rax
        allocateReg(_Rt_); // allocate rt and mark as non const

        if (isConst(_Rs_))
            gen.mov (arg1, registers[_Rs_].val + _Imm_); // address in arg1
        else {
            allocateReg(_Rs_);
            gen.mov (arg1, registers[_Rs_].allocatedReg); // arg1 = $rs
            gen.add (arg1, _Imm_); // arg1 += imm
        }

        gen.call (rax); // call wrapper
        gen.mov (registers[_Rt_].allocatedReg, eax); // move result to rt
    }

    void recSLL() {
        if (!_Rd_) return; // don't compile if NOP
        
        if (isConst(_Rt_)) // if $rt is const
            markConst (_Rd_, registers[_Rt_].val << (_Sa_ & 31)); // mask shift amount and do ($rd = $rs << sa)
    
        else { // nothing is constant
            allocateReg (_Rt_);
            allocateReg (_Rd_);
            gen.mov (registers[_Rd_].allocatedReg, registers[_Rt_].allocatedReg); // $rd = $rt
            gen.shl (registers[_Rd_].allocatedReg, _Sa_); // $rd <<= shift amount (Note: No need to mask by 31, as x86 implicitly does that)
        }
    }

    // This is temporarily used for ADDI too, since we don't account for overflows yet. TODO: Fix
    void recADDIU() { 
        if (!_Rt_) return; // don't compile if NOP
        if (isConst(_Rs_)) // If Rs is constant, mark Rt as constant too
            markConst(_Rt_, registers[_Rs_].val + _Imm_);

        else {
            printf("ADDI(U) %s, %s, %08X", guestRegNames[_Rt_].c_str(), guestRegNames[_Rs_].c_str(), _Imm_);
            allocateReg(_Rt_);
            allocateReg(_Rs_);
            gen.mov (registers[_Rt_].allocatedReg, registers[_Rs_].allocatedReg); // mov $rt, $rs
            gen.add (registers[_Rt_].allocatedReg, _Imm_); // add $rt, #signed immediate
        }
    }

    void recJ() {
        compiling = false; // mark this as the end of the block
        m_nextIsDelaySlot = true; // next instruction will be in a delay slot, as this is a branch

        const uint32_t immediate = (m_psxRegs.code & 0x3FFFFFF) << 2; // fetch the immediate (26 low bits of instruction) and multiply by 4
        const uint32_t newPC = ((recPC & 0xF0000000) | immediate); // Lower 28 bits of PC are replaced by the immediate, top 4 bits of PC are kept
        gen.mov (dword [rbp + PC_OFFSET], newPC); // set new PC
        printf("[JIT64] End of block. Jumped to %08X\n", newPC);
    }

    void recJAL() {
        markConst (31, recPC + 4); // store the return address in $ra and mark as const
        recJ(); // then do the same stuff we'd do for J
    }

    void recJALR() {
        compiling = false; // mark this as the end of the block
        m_nextIsDelaySlot = true; // next instruction will be in a delay slot, as this is a branch
        
        markConst (31, recPC + 4);
        if (isConst(_Rs_)) // if target addr is constant
            gen.mov(dword[rbp + PC_OFFSET], registers[_Rs_].val); // store $rs into pc
        else {
            allocateReg(_Rs_);
            gen.mov(dword[rbp + PC_OFFSET], registers[_Rs_].allocatedReg); // store $rs into PC
        }
    }

    void recJR() { 
        compiling = false; // mark this as the end of the block
        m_nextIsDelaySlot = true; // next instruction will be in a delay slot, as this is a branch

        if (isConst(_Rs_)) // if $rs is constant
            gen.mov (dword [rbp + PC_OFFSET], registers[_Rs_].val & ~3); // set PC to $rs and force align (TODO: Misalignment exceptions)

        else {
            allocateReg(_Rs_);
            gen.mov (eax, registers[_Rs_].allocatedReg); // $rs in eax
            gen.and_ (eax, ~3); // force align address
            gen.mov (dword[rbp + PC_OFFSET], eax); // store $rs in PC
        }
    }

    void recBNE() {
        m_nextIsDelaySlot = true; // the instruction after this will be in a delay slot, since this is a branch
        compiling = false; // stop compiling
        const uint32_t target = recPC + _Imm_ * 4; // the address we'll jump to if the branch is taken

        if (isConst(_Rs_) && isConst(_Rt_)) {  // if both operands are constant
            if (registers[_Rs_].val != registers[_Rt_].val) {  // and they're not equal
                gen.mov (dword[rbp + PC_OFFSET], target); // store new PC
                return; // dip
            }
        }

        Xbyak::Label branchSkipped, exit;

        if (isConst(_Rs_)) { // if only Rs is const
            allocateReg(_Rt_);
            gen.cmp (registers[_Rt_].allocatedReg, registers[_Rs_].val); // compare rs and rt
            gen.jz (branchSkipped, Xbyak::CodeGenerator::T_NEAR); // if equal, skip branch
        }

        else if (isConst(_Rt_)) { // else if only Rt is const
            allocateReg(_Rs_);
            gen.cmp (registers[_Rs_].allocatedReg, registers[_Rt_].val); // compare rs and rt
            gen.jz (branchSkipped, Xbyak::CodeGenerator::T_NEAR); // if equal, skip branch
        }

        else { // nothing is constant
            allocateReg(_Rs_);
            allocateReg(_Rt_);
            gen.cmp (registers[_Rs_].allocatedReg, registers[_Rt_].allocatedReg); // compare rs and rt again
            gen.jz (branchSkipped, Xbyak::CodeGenerator::T_NEAR); // if equal, skip branch
        }

        // code if branch taken
        gen.mov (dword[rbp + PC_OFFSET], target); // move new PC to pc var
        gen.jmp(exit, Xbyak::CodeGenerator::T_NEAR); // skip to the end

        gen.L(branchSkipped); // code if branch skipped
        gen.mov (dword[rbp + PC_OFFSET], recPC); // if the branch was skipped, set PC to recompiler PC
        gen.L(exit); // exit point
    }

    void recBEQ() {
        m_nextIsDelaySlot = true; // the instruction after this will be in a delay slot, since this is a branch
        compiling = false; // stop compiling
        const uint32_t target = recPC + _Imm_ * 4; // the address we'll jump to if the branch is taken

        if (isConst(_Rs_) && isConst(_Rt_)) {  // if both operands are constant
            if (registers[_Rs_].val != registers[_Rt_].val) {  // and they're not equal
                gen.mov (dword[rbp + PC_OFFSET], target); // store new PC
                return; // dip
            }
        }

        Xbyak::Label branchSkipped, exit;

        if (isConst(_Rs_)) { // if only Rs is const
            allocateReg(_Rt_);
            gen.cmp (registers[_Rt_].allocatedReg, registers[_Rs_].val); // compare rs and rt
            gen.jnz (branchSkipped, Xbyak::CodeGenerator::T_NEAR); // if equal, skip branch
        }

        else if (isConst(_Rt_)) { // else if only Rt is const
            allocateReg(_Rs_);
            gen.cmp (registers[_Rs_].allocatedReg, registers[_Rt_].val); // compare rs and rt
            gen.jnz (branchSkipped, Xbyak::CodeGenerator::T_NEAR); // if equal, skip branch
        }

        else { // nothing is constant
            allocateReg(_Rs_);
            allocateReg(_Rt_);
            gen.cmp (registers[_Rs_].allocatedReg, registers[_Rt_].allocatedReg); // compare rs and rt again
            gen.jnz (branchSkipped, Xbyak::CodeGenerator::T_NEAR); // if equal, skip branch
        }

        // code if branch taken
        gen.mov (dword[rbp + PC_OFFSET], target); // move new PC to pc var
        gen.jmp(exit, Xbyak::CodeGenerator::T_NEAR); // skip to the end

        gen.L(branchSkipped); // code if branch skipped
        gen.mov (dword[rbp + PC_OFFSET], recPC); // if the branch was skipped, set PC to recompiler PC
        gen.L(exit); // exit point
    }

    void recBGTZ() {
        m_nextIsDelaySlot = true; // the instruction after this will be in a delay slot, since this is a branch
        compiling = false; // stop compiling
        const uint32_t target = recPC + _Imm_ * 4; // the address we'll jump to if the branch is taken

        if (isConst(_Rs_)) {// if $rs is constant 
            if ((int32_t) registers[_Rs_].val > 0) { // if operand is greater than zero
                gen.mov (dword[rbp + PC_OFFSET], target); // store new PC
                return; // dip
            }
        }

        Xbyak::Label branchSkipped, exit;
        allocateReg (_Rs_);
        gen.test (registers[_Rs_].allocatedReg, registers[_Rs_].allocatedReg); // update flags based on $rs
        gen.jle (branchSkipped, Xbyak::CodeGenerator::T_NEAR); // if le => $rs <= 0, so skip the branch

        // code if branch taken
        gen.mov (dword[rbp + PC_OFFSET], target); // move new PC to pc var
        gen.jmp(exit, Xbyak::CodeGenerator::T_NEAR); // skip to the end

        gen.L(branchSkipped); // code if branch skipped
        gen.mov (dword[rbp + PC_OFFSET], recPC); // if the branch was skipped, set PC to recompiler PC
        gen.L(exit); // exit point
    }

    void recBLEZ() {
        m_nextIsDelaySlot = true; // the instruction after this will be in a delay slot, since this is a branch
        compiling = false; // stop compiling
        const uint32_t target = recPC + _Imm_ * 4; // the address we'll jump to if the branch is taken

        if (isConst(_Rs_)) {// if $rs is constant 
            if ((int32_t) registers[_Rs_].val <= 0) { // if operand is greater than zero
                gen.mov (dword[rbp + PC_OFFSET], target); // store new PC
                return; // dip
            }
        }

        Xbyak::Label branchSkipped, exit;
        allocateReg (_Rs_);
        gen.test (registers[_Rs_].allocatedReg, registers[_Rs_].allocatedReg); // update flags based on $rs
        gen.jg (branchSkipped, Xbyak::CodeGenerator::T_NEAR); // if g => $rs > 0, so skip the branch

        // code if branch taken
        gen.mov (dword[rbp + PC_OFFSET], target); // move new PC to pc var
        gen.jmp(exit, Xbyak::CodeGenerator::T_NEAR); // skip to the end

        gen.L(branchSkipped); // code if branch skipped
        gen.mov (dword[rbp + PC_OFFSET], recPC); // if the branch was skipped, set PC to recompiler PC
        gen.L(exit); // exit point
    }

    void recOR() {
        if (!_Rd_) return; // do not compile if NOP

        if (isConst(_Rs_) && isConst(_Rt_)) // if both Rs and Rt are const
            markConst (_Rd_, registers[_Rt_].val | registers[_Rs_].val);
        
        else if (isConst(_Rt_)) { // Rt is constant
            allocateReg (_Rs_);
            allocateReg (_Rd_);
            gen.mov (registers[_Rd_].allocatedReg, registers[_Rs_].allocatedReg); // $rd = $rs
            gen.or_ (registers[_Rd_].allocatedReg, registers[_Rt_].val); // $rd |= rt
        }

        else if (isConst(_Rs_)) { // Rs is constant
            allocateReg (_Rt_);
            allocateReg (_Rd_);
            gen.mov (registers[_Rd_].allocatedReg, registers[_Rt_].allocatedReg); // $rd = $rt
            gen.or_ (registers[_Rd_].allocatedReg, registers[_Rs_].val); // $rd |= rs
        }

        else { // nothing is constant
            allocateReg (_Rs_);
            allocateReg (_Rt_);
            allocateReg (_Rd_);
            gen.mov (registers[_Rd_].allocatedReg, registers[_Rs_].allocatedReg); // $rd = $rs
            gen.or_ (registers[_Rd_].allocatedReg, registers[_Rt_].allocatedReg); // $rd |= $rt
        }
    }

    void recAND() {
        if (!_Rd_) return; // do not compile if NOP

        if (isConst(_Rs_) && isConst(_Rt_)) // if both Rs and Rt are const
            markConst (_Rd_, registers[_Rt_].val & registers[_Rs_].val);
        
        else if (isConst(_Rt_)) { // Rt is constant
            allocateReg (_Rs_);
            allocateReg (_Rd_);
            gen.mov (registers[_Rd_].allocatedReg, registers[_Rs_].allocatedReg); // $rd = $rs
            gen.and_ (registers[_Rd_].allocatedReg, registers[_Rt_].val); // $rd &= rt
        }

        else if (isConst(_Rs_)) { // Rs is constant
            allocateReg (_Rt_);
            allocateReg (_Rd_);
            gen.mov (registers[_Rd_].allocatedReg, registers[_Rt_].allocatedReg); // $rd = $rt
            gen.and_ (registers[_Rd_].allocatedReg, registers[_Rs_].val); // $rd &= rs
        }

        else { // nothing is constant
            allocateReg (_Rs_);
            allocateReg (_Rt_);
            allocateReg (_Rd_);
            gen.mov (registers[_Rd_].allocatedReg, registers[_Rs_].allocatedReg); // $rd = $rs
            gen.and_ (registers[_Rd_].allocatedReg, registers[_Rt_].allocatedReg); // $rd &= $rt
        }
    }
    
    // Note: This is currently used for both ADDU and ADD
    // TODO: Split ADD and ADDU, implement overflow exceptions
    void recADDU() {
        if (!_Rd_) return; // do not compile if NOP

        if (isConst(_Rs_) && isConst(_Rt_)) // if both Rs and Rt are const
            markConst (_Rd_, registers[_Rt_].val + registers[_Rs_].val);
        
        else if (isConst(_Rt_)) { // Rt is constant
            allocateReg (_Rs_);
            allocateReg (_Rd_);
            gen.mov (registers[_Rd_].allocatedReg, registers[_Rs_].allocatedReg); // $rd = $rs
            gen.add (registers[_Rd_].allocatedReg, registers[_Rt_].val); // $rd += rt
        }

        else if (isConst(_Rs_)) { // Rs is constant
            allocateReg (_Rt_);
            allocateReg (_Rd_);
            gen.mov (registers[_Rd_].allocatedReg, registers[_Rt_].allocatedReg); // $rd = $rt
            gen.add (registers[_Rd_].allocatedReg, registers[_Rs_].val); // $rd += rs
        }

        else { // nothing is constant
            allocateReg (_Rs_);
            allocateReg (_Rt_);
            allocateReg (_Rd_);
            gen.mov (registers[_Rd_].allocatedReg, registers[_Rs_].allocatedReg); // $rd = $rs
            gen.add (registers[_Rd_].allocatedReg, registers[_Rt_].allocatedReg); // $rd += $rt
        }
    }

    void recSLTU() {
        if (!_Rd_) return; // do not compile if NOP

        if (isConst(_Rs_) && isConst(_Rt_)) // if both Rs and Rt are const
            markConst (_Rd_, registers[_Rs_].val < registers[_Rt_].val);
        
        else if (isConst(_Rt_)) { // Rt is constant
            allocateReg (_Rs_);
            allocateReg (_Rd_);
            gen.cmp (registers[_Rs_].allocatedReg, registers[_Rt_].val); // compare $rs and $rt
            gen.setb (al); // if $rs < $rt, set al to 1, else set it to 0
            gen.movzx (registers[_Rd_].allocatedReg, al); // extend al to $rd
        }

        else if (isConst(_Rs_)) { // Rs is constant
            allocateReg (_Rt_);
            allocateReg (_Rd_);
            gen.cmp (registers[_Rt_].allocatedReg, registers[_Rs_].val); // compare $rs and $rt (operands are swapped because of x64 addr mode stuff)
            gen.seta (al); // if $rt > $rs, set al to 1, else set it to 0
            gen.movzx (registers[_Rd_].allocatedReg, al); // extend al to $rd
        }

        else { // nothing is constant
            allocateReg (_Rs_);
            allocateReg (_Rt_);
            allocateReg (_Rd_);            
            gen.cmp (registers[_Rs_].allocatedReg, registers[_Rt_].allocatedReg); // compare $rs and $rt
            gen.setb (al); // if $rs < $rt, set al to 1, else set it to 0
            gen.movzx (registers[_Rd_].allocatedReg, al); // extend al to $rd
        }
    }

    void recCOP0() {
        switch (_Rs_) { // figure out the type of COP0 opcode
            case 0: recMFC0(); break; // MFC0 
            case 4: recMTC0(); break; // MTC0
            default: printf ("Unimplemented cop0 op %02X\n", _Rs_); exit (1); break; 
        }
    }

    void recMFC0() {
        if (!_Rt_) return; // don't compile if NOP
        allocateReg (_Rd_); 
        gen.mov (registers[_Rd_].allocatedReg, dword [rbp + COP0_REGS_OFFSET + _Rd_ * 4]); // load cop0 reg into 
    }

    // Todo: Handle unwriteable regs
    void recMTC0() {
        if (isConst(_Rt_)) { // if the value to store is constant 
            if (_Rd_ == 13) // if writing to CAUSE
                gen.mov (dword [rbp + CAUSE_OFFSET], registers[_Rd_].val & ~0xFC00); // apply the proper mask to the reg and store
            else // if not writing to CAUSE
                gen.mov (dword [rbp + COP0_REGS_OFFSET + _Rd_ * 4], registers[_Rd_].val); // store reg
        }

        else { // if the value to store is not constant
            allocateReg (_Rt_);
            gen.mov (dword [rbp + COP0_REGS_OFFSET + _Rd_ * 4], registers[_Rt_].allocatedReg); // store $rt in the cop 0 reg
            
            if (_Rd_ == 13) // If rd == CAUSE, apply mask
                gen.and_ (dword [rbp + CAUSE_OFFSET], ~0xFC00); 
        }

        if (_Rd_ == 12 || _Rd_ == 13) printf("Check out the testSWInt shit from the original JIT\n");
    }

    /// dump the JIT command buffer, for stuff like viewing it in a disassembler
    void dumpBuffer() {
        auto index = getBufferIndex();
        std::ofstream file ("output.bin", std::ios::binary);
        file.write ((const char*) blocks, index);
        printf ("Dumped %d bytes\n", index);
    }

    /// Call at the end of a block to print the GPR state
    void printRegs() { 
        for (auto i = 0; i < 32; i++) 
            printf("%s: %08X\n", guestRegNames[i].c_str(), m_psxRegs.GPR.r[i]);
        printf("PC: %08X\n", m_psxRegs.pc);
    }
};  
