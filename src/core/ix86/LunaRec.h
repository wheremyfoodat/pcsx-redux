#pragma once
#include "core/debug.h"
#include "core/disr3000a.h"
#include "core/gpu.h"
#include "core/gte.h"
#include "core/psxemulator.h"
#include "core/r3000a.h"
#include "core/system.h"
#include "Luna.hpp"
using namespace Luna;

// Wrapper functions (these need to be global)
    /// Write a 32-bit value to memory[mem]
void psxMemWrite32Wrapper(uint32_t address, uint32_t value) { 
    printf("Wrote %08X to %08X\n", value, address);
    PCSX::g_emulator->m_psxMem->psxMemWrite32(address, value); 
}

class X86DynaRecCPU : public PCSX::R3000Acpu {
    const int KILOYBTE = 1024; // 1 kilobyte is 1024 bytes
    const int MEGABYTE = 1024 * KILOYBTE; // 1 megabyte is 1024 kilobytes
    typedef void (X86DynaRecCPU::*FunctionPointer)(); // Define a "Function Pointer" type to make our life easier
    typedef void (*JITCallback)(); // A function pointer to JIT-emitted code

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
    const R64 registerPointer = rsi;  // RSI will be used as a pointer to the register array
    const std::array <R64, 7> allocateableRegisters = { r8, r9, r10, r11, rdx, rax, rcx }; // the registers our JIT can allocate. r8-r11 come first, as
                                                                                           // those don't have any conventional uses in x64 
    const R32 arg1 = ecx; // register where first arg is stored
    const R32 arg2 = edx; // register where second arg is stored
    const R32 arg3 = r8d; // register where third arg is stored
    const R32 arg4 = r9d; // register where fourth arg is stored
#else
#error "x64 JIT not supported outside of Windows"
#endif
    unsigned int allocatedRegisters = 0; // how many registers have been allocated in this block?

public:
    X86DynaRecCPU() { Init(); } 
// interface methods
    virtual bool isDynarec() final { return true; } // This dynarec is a dynarec, yes
    inline bool Implemented() final { return true; }  // This is implemented in 64-bit mode
    virtual void Execute() { execute(); }
    virtual void Reset() { printf("Add resetting to x64 JIT!\n"); }
    virtual void Clear(uint32_t Addr, uint32_t Size) { printf("Add clearing to x64 JIT\n");  }
    virtual void Shutdown() { printf("Add shutdown to x64 JIT\n"); }
    virtual void SetPGXPMode(uint32_t pgxpMode) final { printf("PGXP stuff in 64-bit JIT\n"); }
   
// backend methods
  private:
    inline bool isPcValid(uint32_t addr) { return validBlockLUT[addr >> 16]; } // Check if the block is at a valid memory address by peeking at the top 16 bits
                                                                                          // This will return true if in WRAM, or BIOS
    inline bool isConst(unsigned reg) {  // for constant propagation, to check if we know a reg value at compile time
        return registers[reg].state == Constant;
    }
    
    virtual bool Init() {
        printf("Initializing x64 JIT...\n");
        gen = Generator(REC_MEMORY_SIZE); // This initializes the emitter and emitter memory
        recRAM = (uintptr_t*) calloc(0x200000, sizeof(uintptr_t*)); // initialize recompiler RAM
        recROM = (uintptr_t*) calloc(0x080000, sizeof(uintptr_t*)); // initialize recompiler ROM
        blocks = gen.data(); // Our code buffer

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
        
        return true; 
    }

    // for constant propagation: shows if a register is constant or not
    enum RegState { Unknown, Constant };

    // A struct that holds a register's info
    struct Register {
        uint32_t val = 0;    // the register's cached value (TODO: Add initial GP/FP)
        RegState state = Constant;  // is this const or not? (Assume constant, set to 0, on boot)
        R32 allocatedReg; // Which host reg has this guest reg been allocated to? The JIT performs register allocation, that means the MIPS reg $t0 might be cached in our x86 "edx" reg, and so on
        bool allocated = false; // Has this register been allocated to a host reg?
    };

    Register registers[32];  // the 32 guest registers
    Generator gen;           // x64 emitter

    const std::array <std::string, 7> allocateableRegNames = { "r8", "r9", "r10", "r11", "rdx", "rax", "rcx" }; // x64 register strings for debugging
    const std::array <std::string, 32> guestRegNames = {"$zero", "$at", "$v0", "$v1", "$a0", "$a1", "$a2", "$a3", "$t0", "$t1", "$t2", "$t3", "$t4"
                                                        "$t5", "$t6", "$t7", "$s0", "$s1", "$s2", "$s3", "$s4", "$s5", "$s6", "$s7", "$t8", "$t9", "$k0"
                                                        "$k1", "$gp", "$sp", "$fp", "$ra" };

    uintptr_t* validBlockLUT; // shows which blocks are valid
    uint8_t* blocks;  // contains the compiled x64 code
    uintptr_t* recROM;  // pointers to the compiled x64 BIOS code
    uintptr_t* recRAM;  // pointers to the compiled x64 WRAM code

    uint32_t recPC;                             // Points to the instruction we're compiling
    const int REC_MEMORY_SIZE = 32 * MEGABYTE;  // how big our x64 code buffer is. This is 132B vs the 32-bit JIT's 8MB,
                                                // just to be safe, and since x64 code tends to be longer + 8MB is tiny
    const int MAX_BLOCK_SIZE = 50;              // Max MIPS instructions per block. This will prolly get raised.

    bool compiling = true;  // Are we compiling code right now?

    const FunctionPointer recBasic [64] = { // Function pointer table to the compilation functions for basic instructions
        &X86DynaRecCPU::recompileSpecial, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recJ, &X86DynaRecCPU::recNULL,  // 00
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 04
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recADDI, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 08
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recORI,  &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recLUI,  // 0c
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 10
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 14
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 18
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 1c
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 20
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 24
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recSW,  // 28
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 2c
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 30
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 34
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 38
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 3c
    };

    const FunctionPointer recSpecial [64] = { // Function pointer table to the compilation functions for special instructions
        &X86DynaRecCPU::recSLL, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 00
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 04
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 08
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 0c
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 10
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 14
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 18
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 1c
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 20
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 24
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 28
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 2c
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 30
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 34
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 38
        &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial, &X86DynaRecCPU::recNULLSpecial,  // 3c
    };

    /// Mark registers[reg] as constant, with a value of "value"
    inline void markConst(int reg, uint32_t value) {
        registers[reg].state = Constant;
        registers[reg].val = value;
    }

    /// Allocate a MIPS register to an x64 register
    void allocateRegister (unsigned regNumber) {
        if (registers[regNumber].allocated) return; // if the register has been allocated, exit
        printf ("Allocated %s to %s", guestRegNames[allocatedRegisters].c_str(), allocateableRegNames[allocatedRegisters].c_str());

        registers[regNumber].allocated = true; // mark reg as allocated
        registers[regNumber].allocatedReg = (R32) allocateableRegisters[allocatedRegisters++]; // allocate a host reg, increment the amount of regs that's been alloc'd
        // gen.mov (registers[regNumber].allocatedRegs, dword [registerPointer + regNumber * 4]); // load the cached reg to the host reg
        printf ("TODO: 32-bit addressing\n");
        assert (allocatedRegisters <= 8); // assert that we didn't overallocate
    }

    /// Params: A program counter value
    /// Returns: A pointer to the host x64 code that points to the block that starts from the given PC
    inline uintptr_t* getBlockPointer (uint32_t pc) {
        uintptr_t base = validBlockLUT[pc >> 16];
        uintptr_t offset = pc & 0xFFFF;
        uintptr_t* pointer = (uintptr_t*) (base + offset);

        return (uintptr_t*) *pointer;
    }

    /// Run the JIT
    void execute() {
        auto blockPointer = getBlockPointer(m_psxRegs.pc); // pointer to the current x64 block
        if (blockPointer == nullptr) { // if the block hasn't been compiled
            printf("Compiling block\n");
            recompileBlock(blockPointer); // compile a block, set block pointer to the address of the block
            printf("Compiled block\n"); // now
        }
        
        else
            printf ("Already compiled this block\n");
        
        auto emittedCode = (JITCallback) blockPointer; // function pointer to the start of the block
        printf ("Buffer pointer: %p\nJumping to buffer address: %p\n", blocks, blockPointer);
        (*emittedCode)(); // call emitted code
        printf("Survived executing a block\n");
        exit(1); // crash because unimplemented
    }

    /// Compile a MIPS block
    /// Params: blockPointer -> The address to store the start of the current block
    void recompileBlock (uintptr_t*& blockPointer) {
        assert(gen.getBufferIndex() < REC_MEMORY_SIZE);  // check that we haven't overflowed our code buffer
        printf("Align me!\n");                           // TODO: Alignment
        uint32_t* instructionPointer;                    // pointer to the instruction to compile

        recPC = m_psxRegs.pc;    // the PC of the recompiler
        uint32_t oldPC = recPC;  // the PC at the start of the block
            
        uintptr_t bufferIndex = gen.getBufferIndex(); // the emitter's current index
        uintptr_t blockStart = (uintptr_t) blocks + bufferIndex; // the address the current block starts from in the emitter buffer
        blockPointer = (uintptr_t*) blockStart; // Add the block to the block cache

        auto compiledInstructions = 0;  // how many instructions we've compiled in this block

        while (compiling) {
            m_inDelaySlot = m_nextIsDelaySlot; // handle delay slot jazz
            m_nextIsDelaySlot = false;

            instructionPointer = (uint32_t*) PSXM(recPC);  // get pointer to instruction
            assert(instructionPointer != NULL);           // check the pointer is not null
            m_psxRegs.code = *instructionPointer;         // read the instruction
            (*this.*recBasic[m_psxRegs.code >> 26])();        // Call the function to compile the instruction (The >> 26 is to fetch the ocpode)

            recPC += 4;              // increment PC by sizeof(instruction)
            compiledInstructions++;  // increment the compiled instructions counter
        }

        gen.ret(); // emit a RET to return from the JIT 
        dumpBuffer();
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
            allocateRegister (_Rs_);
            allocateRegister (_Rt_);
            registers[_Rt_].state = Unknown; // mark Rt as unknown
            gen.mov (registers[_Rt_].allocatedReg, registers[_Rs_].allocatedReg); // mov rt, rs
            gen.OR (registers[_Rt_].allocatedReg, (u32) _ImmU_); // or $rt, imm
        }
    }

    // TODO: Optimize
    void recSW() { 
        assert (4 > allocatedRegisters); // assert that we're not trampling any allocated regs
        gen.mov (rax, (u64) &psxMemWrite32Wrapper); // function pointer in rax

        if (isConst(_Rs_))
            gen.mov (arg1, registers[_Rs_].val + _Imm_); // address in arg1
        else {
            allocateRegister(_Rs_);
            gen.mov (arg1, registers[_Rs_].allocatedReg); // arg1 = $rs
            gen.add (arg1, _Imm_); // arg1 += imm
        }

        if (isConst(_Rt_))
            gen.mov (arg2, registers[_Rt_].val); // value in arg2
        else {
            allocateRegister(_Rt_);
            gen.mov (arg2, registers[_Rt_].allocatedReg);
        }

        gen.call (rax); // call wrapper
    }

    void recSLL() {
        if (!_Rd_) return; // don't compile if NOP
        assert(1 == 0); // Implement this later
    }

    void recADDI() { 
        if (!_Rt_) return; // don't compile if NOP
        if (isConst(_Rs_)) // If Rs is constant, mark Rt as constant too
            markConst(_Rt_, registers[_Rs_].val + _Imm_);

        else {
            printf("ADDI %s, %s, %08X", guestRegNames[_Rt_].c_str(), guestRegNames[_Rs_].c_str(), _Imm_);
            registers[_Rt_].state = Unknown;
            assert(1 == 0); // Implement this later
        }
    }

    void recJ() {
        compiling = false; // mark this as the end of the block
        const u32 immediate = (m_psxRegs.code & 0x3FFFFFF) << 2; // fetch the immediate (26 low bits of instruction) and multiply by 4
        const u32 newPC = ((recPC & 0xF0000000) | immediate); // Lower 28 bits of PC are replaced by the immediate, top 4 bits of PC are kept

        printf("[JIT64] End of block. Jumped to %08X\n", newPC);
    }

    void dumpBuffer() {
        auto index = gen.getBufferIndex();
        std::ofstream file ("output.bin", std::ios::binary);
        file.write ((const char*) blocks, index);
        printf ("Dumped %d bytes\n", index);
    }
};  
