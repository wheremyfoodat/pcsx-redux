#pragma once
#include <fmt/core.h>
#include "core/debug.h"
#include "core/disr3000a.h"
#include "core/gpu.h"
#include "core/gte.h"
#include "core/psxemulator.h"
#include "core/r3000a.h"
#include "core/system.h"
#include "Luna.hpp"
using namespace Luna;

class X86DynaRecCPU : public PCSX::R3000Acpu {
    const int KILOYBTE = 1024;
    const int MEGABYTE = 1024 * KILOYBTE;
    typedef void (X86DynaRecCPU::*FunctionPointer)();

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
    const R64 registerPointer = rbp;  // RBP is used as a pointer to the register array
    const std::array <R64, 7> allocateableRegisters = { r8, r9, r10, r11, rdx, rax, rcx }; // the registers our JIT can allocate. r8-r11 come first, as
                                                                                           // those don't have any conventional uses in x64 
#else
#error "x64 JIT not supported outside of Windows"
#endif
    unsigned int allocatedRegisters = 0; // how many registers have been allocated in this regs?

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
    // inline bool isPcValid(uint32_t addr) { return m_psxRecLUT[addr >> 16]; }
    inline bool isConst(unsigned reg) {  // for constant propagation, to check if we know a reg value at compile time
        return registers[reg].state == Constant;
    }

    
    virtual bool Init() { 
        printf("Add init method to x64 JIT!\n"); 
        gen = Generator(REC_MEMORY_SIZE); // This initializes the emitter and emitter memory
        blocks = gen.data();
        return true; 
    }

    // for constant propagation: shows if a register is constant or not
    enum RegState { Unknown, Constant };

    // A struct that holds a register's info
    struct Register {
        uint32_t val;    // the register's cached value
        RegState state;  // is this const or not?
        R32 allocatedReg; // Which host reg has this guest reg been allocated to? The JIT performs register allocation, that means the MIPS reg $t0 might be cached in our x86 "edx" reg, and so on
        bool allocated = false; // Has this register been allocated to a host reg?
    };

    Register registers[32];  // the 32 guest registers
    Generator gen;           // x64 emitter

    const std::array <std::string, 7> allocateableRegNames = { "r8", "r9", "r10", "r11", "rdx", "rax", "rcx" }; // x64 register strings for debugging
    const std::array <std::string, 32> guestRegNames = {"$zero", "$at", "$v0", "$v1", "$a0", "$a1", "$a2", "$a3", "$t0", "$t1", "$t2", "$t3", "$t4"
                                                        "$t5", "$t6", "$t7", "$s0", "$s1", "$s2", "$s3", "$s4", "$s5", "$s6", "$s7", "$t8", "$t9", "$k0"
                                                        "$k1", "$gp", "$sp", "$fp", "$ra" };

    uint8_t* blocks;  // contains the compiled x64 code
    uint8_t* recROM;  // recompiler ROM
    uint8_t* recRAM;  // recompiler RAM

    uint32_t recPC;                             // Points to the instruction we're compiling
    const int REC_MEMORY_SIZE = 16 * MEGABYTE;  // how big our x64 code buffer is. This is 16MB vs the 32-bit JIT's 8MB,
                                                // just to be safe, and since x64 code tends to be longer
    const int MAX_BLOCK_SIZE = 50;              // Max MIPS instructions per block. This will prolly get raised.

    bool compiling = true;  // Are we compiling code right now?

    const FunctionPointer recBasic [64] = { // Function pointer table to the compilation functionss
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 00
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 04
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 08
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recORI,  &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recLUI,  // 0c
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 10
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 14
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 18
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 1c
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 20
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 24
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 28
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 2c
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 30
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 34
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 38
        &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL, &X86DynaRecCPU::recNULL,  // 3c
    };

    /// Mark registers[reg] as constant, with a value of "value"
    void markConst(int reg, uint32_t value) {
        registers[reg].state = Constant;
        registers[reg].val = value;
    }

    /// Allocate a MIPS register to an x64 register
    void allocateRegister (unsigned regNumber) {
        if (registers[regNumber].allocated) return; // if the register has been allocated, exit
        //printf (fmt::format("Allocated {} to {}", guestRegNames[allocatedRegisters], allocateableRegNames[allocatedRegisters]).c_str());

        registers[regNumber].allocated = true; // mark reg as allocated
        registers[regNumber].allocatedReg = (R32) allocateableRegisters[allocatedRegisters++]; // allocate a host reg, increment the amount of regs that's been alloc'd
        assert (allocatedRegisters <= 8); // assert that we didn't overallocate
    }

    /// Run the JIT
    void execute() {
        recompileBlock(); // compile a block
        printf("Compiled block\n"); // now
        exit(1);
    }

    /// Compile a MIPS block
    void recompileBlock() {
        assert(gen.getBufferIndex() < REC_MEMORY_SIZE);  // check that we haven't overflowed our code buffer
        printf("Align me!\n");                           // TODO: Alignment
        uint32_t* instructionPointer;                    // pointer to the instruction to compile

        recPC = m_psxRegs.pc;    // the PC of the recompiler
        uint32_t oldPC = recPC;  // the PC at the start of the block
        uint32_t startingBufferIndex = gen.getBufferIndex();
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
    }

    /// Compile a reserved or unimplemented instruction
    void recNULL() {
        printf("Compiling unknown instruction %08X\n", m_psxRegs.code);
        printf("Opcode: %02X\n", m_psxRegs.code >> 26);
        exit(1);
    }
    
    void recLUI() {
        auto value = _Imm_ << 16; // fetch immediate, shift to the left by 16
        markConst (_Rt_, value); // Note: LUI *always* produces a constant result
    }

    void recORI() {
        if (isConst(_Rs_))
            markConst (_Rt_, registers[_Rs_].val | _Imm_);
        else {
            allocateRegister (_Rs_);
            allocateRegister (_Rt_);
            registers[_Rt_].state = Unknown; // mark Rt as unknown
            gen.mov (registers[_Rt_].allocatedReg, registers[_Rs_].allocatedReg); // mov rt, rs
            gen.OR (registers[_Rt_].allocatedReg, (u32) _Imm_); // or $rt, imm
        }
    }
};
