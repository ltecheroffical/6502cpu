#include <stddef.h>
#include <cpu.h>

static enum cpu6502_CPU_Stage nop(cpu6502_CPU *cpu) {
	return cpu->state.stage;
}

static enum cpu6502_CPU_Stage state_reset0(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, 0xFFFC);
	return cpu->state.stage;
}

static enum cpu6502_CPU_Stage state_reset1(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, 0xFFFD);
	cpu->state.pc = cpu6502_CPU_get_data(cpu);
	return cpu->state.stage;
}

static enum cpu6502_CPU_Stage state_reset2(cpu6502_CPU *cpu) {
	cpu->state.pc |= cpu6502_CPU_get_data(cpu) << 8;
	return cpu6502_CPU_STATE_FETCH;
}

static enum cpu6502_CPU_Stage state_fetch_pre(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, cpu->state.pc++);
	return cpu->state.stage;
}

static enum cpu6502_CPU_Stage state_fetch0(cpu6502_CPU *cpu) { // Opcode
	cpu->state.instruction[0] = cpu6502_CPU_get_data(cpu);
	cpu6502_CPU_read_req(cpu, cpu->state.pc); // 6502 is too dumb to skip the 2nd byte of an instruction
	return cpu->state.stage;
}

static enum cpu6502_CPU_Stage state_fetch1(cpu6502_CPU *cpu) { // Operand byte 0
	if (cpu6502_CPU_instruction_operand_sizes[cpu->state.instruction[0]] < 1) { // Discard if instruction doesn't have operands
		return cpu6502_CPU_STATE_EXEC;
	}
	// That's actually a valid fetch
	cpu->state.instruction[1] = cpu6502_CPU_get_data(cpu);
	cpu->state.pc++;

	if (cpu6502_CPU_instruction_operand_sizes[cpu->state.instruction[0]] < 2) {
		return cpu6502_CPU_STATE_EXEC;
	}
	cpu6502_CPU_read_req(cpu, cpu->state.pc++);
	return cpu->state.stage;
}

static enum cpu6502_CPU_Stage state_fetch2(cpu6502_CPU *cpu) { // Operand byte 1
	cpu->state.instruction[2] = cpu6502_CPU_get_data(cpu);
	return cpu6502_CPU_STATE_EXEC;
}

static enum cpu6502_CPU_Stage state_exec(cpu6502_CPU *cpu) {
	bool interpret = true;
	bool exit = false;
	while (interpret) {
		interpret = false;
		if (cpu->exec.waiting_cycles > 0) {
			cpu->exec.waiting_cycles--;
			return cpu->state.stage;
		}
		cpu6502_CPU_InstructionFunc cycle_func = cpu6502_CPU_instructions[cpu->state.instruction[0]][cpu->exec.instruction_cycle++];
		if (cycle_func != NULL) {
			cpu6502_u8 cycles = cycle_func(cpu);
			if (cycles == 0xFF) {
				exit = true;
			}
			cpu->exec.waiting_cycles = cycles;
		}

		cpu6502_CPU_InstructionFunc next_func = cpu6502_CPU_instructions[cpu->state.instruction[0]][cpu->exec.instruction_cycle];
		if (next_func == cpu6502_CPU_INSTRUCTION_NO_WAIT) {
			// Execute the next instruction this same cycle
			interpret = true;
			cpu->exec.instruction_cycle++;
		} else if (next_func == cpu6502_CPU_INSTRUCTION_END) {
			// End of instruction
			exit = true;
			cpu->exec.instruction_cycle = 0;
		}
	}

	if (!exit) {
		return cpu->state.stage;
	}

	// Check interrupt
	if (cpu->state.nmi_request) {
		cpu->state.nmi_request = false;
		return cpu6502_CPU_STATE_INT_NMI;
	}

	if (cpu->state.irq_request && !(cpu->state.p & cpu6502_CPU_PROCESSOR_STATUS_NO_INT)) {
		cpu->state.irq_request = false;
		return cpu6502_CPU_STATE_INT_IRQ;
	}
	return cpu6502_CPU_STATE_FETCH;
}

static enum cpu6502_CPU_Stage state_exec_pre(cpu6502_CPU *cpu) {
	cpu->exec.instruction_cycle = 0;
	cpu->exec.waiting_cycles = 0;
	return state_exec(cpu);
}

static enum cpu6502_CPU_Stage state_int_push_pc0(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, 0x100 + --cpu->state.sp, (cpu->state.pc >> 8) & 0xFF);
	return cpu->state.stage;
}

static enum cpu6502_CPU_Stage state_int_push_pc1(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, 0x100 + --cpu->state.sp, cpu->state.pc & 0xFF);
	return cpu->state.stage;
}

static enum cpu6502_CPU_Stage state_int_push_p(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, 0x100 + --cpu->state.sp, cpu->state.p);
	cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_NO_INT;
	return cpu->state.stage;
}

static enum cpu6502_CPU_Stage state_int_irq0(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, 0xFFFE);
	return cpu->state.stage;
}

static enum cpu6502_CPU_Stage state_int_irq1(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, 0xFFFF);
	cpu->state.pc = cpu6502_CPU_get_data(cpu);
	return cpu->state.stage;
}

static enum cpu6502_CPU_Stage state_int_irq2(cpu6502_CPU *cpu) {
	cpu->state.pc |= cpu6502_CPU_get_data(cpu) << 8;
	return cpu6502_CPU_STATE_FETCH;
}

static enum cpu6502_CPU_Stage state_int_nmi0(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, 0xFFFA);
	return cpu->state.stage;
}

static enum cpu6502_CPU_Stage state_int_nmi1(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, 0xFFFB);
	cpu->state.pc = cpu6502_CPU_get_data(cpu);
	return cpu->state.stage;
}

static enum cpu6502_CPU_Stage state_int_nmi2(cpu6502_CPU *cpu) {
	cpu->state.pc |= cpu6502_CPU_get_data(cpu) << 8;
	return cpu6502_CPU_STATE_FETCH;
}

cpu6502_CPU_StageFunc cpu6502_CPU_states[cpu6502_CPU_STATE_COUNT][cpu6502_MAX_CPU_STATE_CYCLES] = {
    [cpu6502_CPU_STATE_RESET] = {nop, nop, nop, nop, nop, state_reset0, state_reset1, state_reset2},
    [cpu6502_CPU_STATE_FETCH] = {state_fetch_pre, state_fetch0, state_fetch1, state_fetch2},
    [cpu6502_CPU_STATE_EXEC] = {state_exec_pre, state_exec, state_exec, state_exec, state_exec, state_exec, state_exec, state_exec, state_exec, state_exec, state_exec, state_exec},
    [cpu6502_CPU_STATE_INT_IRQ] = {state_int_push_pc0, state_int_push_pc1, state_int_push_p, state_int_irq0, state_int_irq1, state_int_irq2},
    [cpu6502_CPU_STATE_INT_NMI] = {state_int_push_pc0, state_int_push_pc1, state_int_push_p, state_int_nmi0, state_int_nmi1, state_int_nmi2},
};
