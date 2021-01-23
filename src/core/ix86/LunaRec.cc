#include "core/disr3000a.h"
#include "core/psxemulator.h"
#include "core/r3000a.h"
#include "core/system.h"
#include "core/ix86/Luna.hpp"

namespace {
const auto KILOYBTE = 1024;
const auto MEGABYTE = 1024 * KILOYBTE;

#if defined(__x86_64) || defined (_M_AMD64) // if 64-bit
    class X86DynaRecCPU : public PCSX::R3000Acpu ("x64 Dynarec") {
        inline bool isPcValid(uint32_t addr) { return m_psxRecLUT[addr >> 16]; }
        inline bool isConst(unsigned reg) { return m_iRegs[reg].state == Constant; } // for constant propagation, to check if we know a reg value at compile time
        constexpr bool Implemented() final { return true; } // This is implemented in 64-bit modes

        // for constant propagation: shows if a register is constant or not
        enum RegState { Unknown, Constant };

        // A struct that holds a register's info
        struct Register {
            uint32_t val; // the register's value
            RegState state; // is this const or not?
        };

        Register registers[32]; // the 32 guest registers
        uint8_t* blocks; // contains the compiled x64 code
        uint8_t* recROM; // recompiler ROM
        uint8_t* recRAM; // recompiler RAM
        uint32_t recPC; // Points to the instruction we're compiling
        const int REC_MEMORY_SIZE = 16 * MEGABYTE; // how big our x64 code buffer is. This is 16MB vs the 32-bit JIT's 8MB, just to be safe, and since x64 code tends to be longer
        const int MAX_BLOCK_SIZE = 50; // Max MIPS instructions per block. This will prolly get raised.

        bool compiling = true; // Are we compiling code right now?

        // Mark registers[reg] as constant, with a value of "value"
        void mapConst(int reg, uint32_t value) {
            registers[reg].state = Constant;
            registers[reg].val = value;
        }

        void init() {

        }
    }

else // if not 64-bit: Mark JIT as unimplemented
    class X86DynaRecCPU : public PCSX::R3000Acpu {
        inline bool Implemented() final { return false; } // This is not implemented in 32-bit mode
        virtual bool Init() final { return false; }
        virtual void Reset() final { abort(); }
        virtual void Execute() final { abort(); }
        virtual void Clear(uint32_t Addr, uint32_t Size) final { abort(); }
        virtual void Shutdown() final { abort(); }
        virtual void SetPGXPMode(uint32_t pgxpMode) final { abort(); }
        virtual bool isDynarec() final { abort(); }
    }
#endif
} // end anonymous namespace