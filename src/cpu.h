#ifndef _CPU_H_
#define _CPU_H_

#include <6502cpu/cpu.h>

#define cpu6502_MAX_CPU_STATE_CYCLES 12
#define cpu6502_MAX_CPU_INSTRUCTION_CYCLES 8

#define cpu6502_CPU_INSTRUCTION_END (cpu6502_CPU_InstructionFunc)0x00
#define cpu6502_CPU_INSTRUCTION_NO_WAIT (cpu6502_CPU_InstructionFunc)0x01

typedef enum cpu6502_CPU_Stage (*cpu6502_CPU_StageFunc)(cpu6502_CPU *cpu);
typedef cpu6502_u8 (*cpu6502_CPU_InstructionFunc)(cpu6502_CPU *cpu);

// First entry per state is ran immediately after state switch but return value is discarded and the state cycle isn't incrmented
extern cpu6502_CPU_StageFunc cpu6502_CPU_states[cpu6502_CPU_STATE_COUNT][cpu6502_MAX_CPU_STATE_CYCLES];
extern cpu6502_CPU_InstructionFunc cpu6502_CPU_instructions[256][cpu6502_MAX_CPU_INSTRUCTION_CYCLES];
extern cpu6502_u8 cpu6502_CPU_instruction_operand_sizes[256];

void cpu6502_CPU_read_req(cpu6502_CPU *cpu, cpu6502_u16 addr);
void cpu6502_CPU_write_req(cpu6502_CPU *cpu, cpu6502_u16 addr, cpu6502_u8 data);
cpu6502_u8 cpu6502_CPU_get_data(cpu6502_CPU *cpu);
cpu6502_u16 cpu6502_CPU_get_addr(cpu6502_CPU *cpu);

cpu6502_u8 cpu6502_CPU_next_rnd(cpu6502_CPU *cpu);

#endif // _CPU_H_
