#include <stddef.h>

#include <cpu.h>

cpu6502_CPU cpu6502_CPU_create(cpu6502_CPU_Flags flags) {
	cpu6502_CPU cpu;
	cpu.state.p = 0x00;
	cpu.state.cycle = 0;
	cpu.bus_addr = NULL;
	cpu.bus_data = NULL;
	cpu.bus_write = NULL;
	cpu.flags = flags;
	// LFSR can be leaved uninitalized as some sort of randomness
	// but we don't want zero
	if (cpu.lfsr == 0) {
		cpu.lfsr++;
	}
	cpu6502_CPU_reset(&cpu);
	return cpu;
}

void cpu6502_CPU_set_state(cpu6502_CPU *cpu, cpu6502_CPUState state) {
	cpu->state = state;
}

cpu6502_CPUState cpu6502_CPU_get_state(const cpu6502_CPU *cpu) {
	return cpu->state;
}

void cpu6502_CPU_set_bus(cpu6502_CPU *cpu, cpu6502_u8 *bus_data, cpu6502_u16 *bus_addr, bool *bus_write) {
	cpu->bus_data = bus_data;
	cpu->bus_addr = bus_addr;
	cpu->bus_write = bus_write;
}

void cpu6502_CPU_reset(cpu6502_CPU *cpu) {
	cpu->state.stage = cpu6502_CPU_STATE_RESET;
	cpu->state.stage_cycle = 0;
	cpu->state.cycle = 0;
	cpu6502_CPU_states[cpu->state.stage][0](cpu); // Run the pre state

	cpu->state.halted = false;

	cpu->state.irq_request = false;
	cpu->state.nmi_request = false;
}

void cpu6502_CPU_int(cpu6502_CPU *cpu, cpu6502_CPU_Interrupt interrupt) {
	switch (interrupt) {
		case cpu6502_CPU_INTERUPT_IRQ: cpu->state.irq_request = true; break;
		case cpu6502_CPU_INTERUPT_NMI: cpu->state.nmi_request = true; break;
	}
}

void cpu6502_CPU_cycle(cpu6502_CPU *cpu) {
	cpu6502_CPU_StageFunc stage_func = cpu6502_CPU_states[cpu->state.stage][cpu->state.stage_cycle + 1];
	enum cpu6502_CPU_Stage next = stage_func(cpu);
	if (next != cpu->state.stage) {
		cpu6502_CPU_StageFunc pre_state_func = cpu6502_CPU_states[next][0];
		pre_state_func(cpu);
		cpu->state.stage_cycle = 0;
		cpu->state.stage = next;
	} else {
		cpu->state.stage_cycle++;
	}
	cpu->state.cycle++;
}

void cpu6502_CPU_read_req(cpu6502_CPU *cpu, cpu6502_u16 addr) {
	if (cpu->bus_write == NULL || cpu->bus_addr == NULL) {
		return;
	}

	*cpu->bus_write = false;
	*cpu->bus_addr = addr;
}

void cpu6502_CPU_write_req(cpu6502_CPU *cpu, cpu6502_u16 addr, cpu6502_u8 data) {
	if (cpu->bus_write == NULL || cpu->bus_addr == NULL || cpu->bus_data == NULL) {
		return;
	}

	*cpu->bus_addr = addr;
	*cpu->bus_data = data;
	*cpu->bus_write = true;
}

cpu6502_u8 cpu6502_CPU_get_data(cpu6502_CPU *cpu) {
	if (cpu->bus_write == NULL || cpu->bus_data == NULL) {
		return 0;
	}
	return *cpu->bus_data;
}

cpu6502_u16 cpu6502_CPU_get_addr(cpu6502_CPU *cpu) {
	if (cpu->bus_write == NULL || cpu->bus_addr == NULL) {
		return 0;
	}
	return *cpu->bus_addr;

}

cpu6502_u8 cpu6502_CPU_next_rnd(cpu6502_CPU *cpu) {
	cpu6502_u8 work_lfsr = cpu->lfsr >> 1;
	work_lfsr |= ((cpu->lfsr & (1 << 0)) ^ (work_lfsr & (1 << 4))) ? 0x80 : 0x00;
	cpu->lfsr = work_lfsr;
	return work_lfsr;
}
