#include <cassert>
#include "core/disr3000a.h"
#include "core/psxemulator.h"
#include "core/r3000a.h"
#include "core/system.h"
#include "core/ix86/Luna.hpp"
using namespace Luna;

namespace {
const auto KILOYBTE = 1024;
const auto MEGABYTE = 1024 * KILOYBTE;
const auto registerPointer = rbp; // RBP is used as a pointer to the register array

#if defined(__x86_64) || defined (_M_AMD64) // if 64-bit
class X86DynaRecCPU : public PCSX::R3000Acpu {
    // inline bool isPcValid(uint32_t addr) { return m_psxRecLUT[addr >> 16]; }
    inline bool isConst(unsigned reg) { // for constant propagation, to check if we know a reg value at compile time
        return registers[reg].state == Constant;
    }
    constexpr bool Implemented() final { return true; }  // This is implemented in 64-bit modes

    // for constant propagation: shows if a register is constant or not
    enum RegState { Unknown, Constant };

    // A struct that holds a register's info
    struct Register {
        uint32_t val;    // the register's value
        RegState state;  // is this const or not?
    };

    Register registers[32];  // the 32 guest registers
    Generator gen;           // x64 emitter

    uint8_t* blocks;  // contains the compiled x64 code
    uint8_t* recROM;  // recompiler ROM
    uint8_t* recRAM;  // recompiler RAM

    uint32_t recPC;                             // Points to the instruction we're compiling
    const int REC_MEMORY_SIZE = 16 * MEGABYTE;  // how big our x64 code buffer is. This is 16MB vs the 32-bit JIT's 8MB,
                                                // just to be safe, and since x64 code tends to be longer
    const int MAX_BLOCK_SIZE = 50;              // Max MIPS instructions per block. This will prolly get raised.

    bool compiling = true;  // Are we compiling code right now?

    // Mark registers[reg] as constant, with a value of "value"
    void mapConst(int reg, uint32_t value) {
        registers[reg].state = Constant;
        registers[reg].val = value;
    }

    void init() {}

    void recompileBlock() {
        assert(gen.getBufferIndex() < REC_MEMORY_SIZE);  // check that we haven't overflowed our code buffer
        printf("Align me!\n");                           // TODO: Alignment
        uint32_t* instructionPointer;                    // pointer to the instruction to compile

        recPC = m_psxRegs.pc;    // the PC of the recompiler
        uint32_t oldPC = recPC;  // the PC at the start of the block
        uint32_t startingBufferIndex = gen.getBufferIndex();
        auto compiledInstructions = 0;  // how many instructions we've compiled in this block

        while (compiling) {
            m_inDelaySlot = m_nextIsDelaySlot;
            m_nextIsDelaySlot = false;

            instructionPointer = (uint32_t*)PSXM(recPC);  // get pointer to instruction
            m_psxRegs.code = *instructionPointer;         // read the instruction
            assert(instructionPointer != NULL);           // check the pointer is not null
            PCSX::g_system->message("Compiling instruction %08X", m_psxRegs.code);
            exit(1);

            recPC += 4;              // increment PC by sizeof(instruction)
            compiledInstructions++;  // increment the compiled instructions counter
        }
    }
};
#endif
} // end anonymous namespace
