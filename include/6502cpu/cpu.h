#ifndef _6502cpu_CPU_H_
#define _6502cpu_CPU_H_

#include <stdbool.h>
#include <6502cpu/common/types/inttypes.h>

typedef enum {
	cpu6502_CPU_PROCESSOR_STATUS_CARRY = 1 << 0,
	cpu6502_CPU_PROCESSOR_STATUS_ZERO = 1 << 1,
	cpu6502_CPU_PROCESSOR_STATUS_NO_INT = 1 << 2,
	cpu6502_CPU_PROCESSOR_STATUS_DECIMAL = 1 << 3,
	cpu6502_CPU_PROCESSOR_STATUS_BRK = 1 << 4,
	cpu6502_CPU_PROCESSOR_STATUS_OVERFLOW = 1 << 6,
	cpu6502_CPU_PROCESSOR_STATUS_NEGATIVE = 1 << 7,
} cpu6502_CPU_ProcessorStatus;

typedef struct {
	// Registers
	cpu6502_i8 a;
	cpu6502_i8 x;
	cpu6502_i8 y;
	cpu6502_CPU_ProcessorStatus p;
	cpu6502_u8 sp;
	cpu6502_u16 pc;

	bool halted;

	bool irq_request;
	bool nmi_request;

	enum cpu6502_CPU_Stage {
		cpu6502_CPU_STATE_RESET = 0,
		cpu6502_CPU_STATE_FETCH,
		cpu6502_CPU_STATE_EXEC,

		cpu6502_CPU_STATE_INT_IRQ,
		cpu6502_CPU_STATE_INT_NMI,

		cpu6502_CPU_STATE_COUNT,
	} stage;

	cpu6502_u8 stage_cycle;

	cpu6502_u8 instruction[3];
	cpu6502_u8 work[8];

	cpu6502_u64 cycle;
} cpu6502_CPUState;

typedef struct {
	cpu6502_CPUState state;

	cpu6502_u8 lfsr;

	union {
		struct {
			cpu6502_u8 waiting_cycles;
			cpu6502_u8 instruction_cycle;
		} exec;
	};


	enum cpu6502_CPU_Flags {
		cpu6502_CPU_FLAGS_DISABLE_DECIMAL = 1 << 0,
	} flags;

	cpu6502_u8 *bus_data;
	cpu6502_u16 *bus_addr;
	bool *bus_write;
} cpu6502_CPU;

typedef enum {
	cpu6502_CPU_INTERUPT_IRQ = 0,
	cpu6502_CPU_INTERUPT_NMI,
} cpu6502_CPU_Interrupt;

typedef enum cpu6502_CPU_Flags cpu6502_CPU_Flags;

cpu6502_CPU cpu6502_CPU_create(cpu6502_CPU_Flags flags);

void cpu6502_CPU_set_state(cpu6502_CPU *cpu, cpu6502_CPUState state);
cpu6502_CPUState cpu6502_CPU_get_state(const cpu6502_CPU *cpu);

void cpu6502_CPU_set_bus(cpu6502_CPU *cpu, cpu6502_u8 *bus_data, cpu6502_u16 *bus_addr, bool *bus_write);

void cpu6502_CPU_reset(cpu6502_CPU *cpu);
void cpu6502_CPU_int(cpu6502_CPU *cpu, cpu6502_CPU_Interrupt interrupt);

void cpu6502_CPU_cycle(cpu6502_CPU *cpu);

#endif // _6502cpu_CPU_H_
