#include <cpu.h>

static inline cpu6502_i8 set_zn_flags(cpu6502_CPU *cpu, cpu6502_i8 val) {
	cpu->state.p &= ~(cpu6502_CPU_PROCESSOR_STATUS_ZERO | cpu6502_CPU_PROCESSOR_STATUS_NEGATIVE);

	if (val < 0) {
		cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_NEGATIVE;
	}
	if (val == 0) {
		cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_ZERO;
	}
	return val;
}

static inline bool page_crossed(cpu6502_u16 addr, cpu6502_u16 offset) {
	return (addr & 0xFF00) != ((addr + offset) & 0xFF00);
}

static inline cpu6502_u8 branch(cpu6502_CPU *cpu, bool cond) {
	cpu6502_u8 cycles = 0;
	if (cond) {
		if (page_crossed(cpu->state.pc, cpu->state.instruction[1])) {
			cycles++;
		}
		cpu->state.pc += (cpu6502_i8)cpu->state.instruction[1];
		cycles++;
	}
	return cycles;
}

/* HACK: For the bus functions, if you put bus functions and don't wait for a cycle after the
 * last one, you can repurpose the calculated address into a write operation.
 */

static cpu6502_u8 bus_zp(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, cpu->state.instruction[1]);
	return 0;
}

static cpu6502_u8 bus_zp_x(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, (cpu->state.instruction[1] + cpu->state.x) & 0xFF);
	return 0;
}

static cpu6502_u8 bus_zp_y(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, (cpu->state.instruction[1] + cpu->state.y) & 0xFF);
	return 0;
}

static cpu6502_u8 bus_abs(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, cpu->state.instruction[1] | (cpu->state.instruction[2] << 8));
	return 0;
}

static cpu6502_u8 bus_abs_x(cpu6502_CPU *cpu) {
	cpu6502_u16 addr = cpu->state.instruction[1] | (cpu->state.instruction[2] << 8);
	cpu6502_CPU_read_req(cpu, addr + cpu->state.x);

	if (page_crossed(addr, cpu->state.x)) {
		return 1;
	}
	return 0;
}

static cpu6502_u8 bus_abs_y(cpu6502_CPU *cpu) {
	cpu6502_u16 addr = cpu->state.instruction[1] | (cpu->state.instruction[2] << 8);
	cpu6502_CPU_read_req(cpu, addr + cpu->state.y);

	if (page_crossed(addr, cpu->state.y)) {
		return 1;
	}
	return 0;
}

static cpu6502_u8 bus_idx_ind_x0(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, (cpu->state.instruction[1] + cpu->state.x) & 0xFF);
	return 0;
}

static cpu6502_u8 bus_idx_ind_x1(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, (cpu->state.instruction[1] + cpu->state.x + 1) & 0xFF);
	cpu->state.work[0] = cpu6502_CPU_get_data(cpu);
	return 0;
}

static cpu6502_u8 bus_idx_ind_x2(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, cpu->state.work[0] | (cpu6502_CPU_get_data(cpu) << 8));
	return 0;
}

static cpu6502_u8 bus_ind_idx_y0(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, cpu->state.instruction[1] & 0xFF);
	return 0;
}

static cpu6502_u8 bus_ind_idx_y1(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, (cpu->state.instruction[1] + 1) & 0xFF);
	cpu->state.work[0] = cpu6502_CPU_get_data(cpu);
	return 0;
}

static cpu6502_u8 bus_ind_idx_y2(cpu6502_CPU *cpu) {
	cpu6502_u16 addr = (cpu->state.work[0] | (cpu6502_CPU_get_data(cpu) << 8));
	cpu6502_CPU_read_req(cpu, addr + cpu->state.y);
	if (page_crossed(addr, cpu->state.y)) {
		return 1;
	}
	return 0;
}

static cpu6502_u8 inc_pc(cpu6502_CPU *cpu) {
	cpu->state.pc++;
	return 0;
}

static cpu6502_u8 nop(cpu6502_CPU *cpu) {
	return 0;
}

static void adc(cpu6502_CPU *cpu, cpu6502_u8 a, cpu6502_u8 b) {
	cpu6502_u8 operand = b;
	cpu6502_u16 sum = 0;

	bool carry = cpu->state.p & cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	cpu->state.p &= ~(cpu6502_CPU_PROCESSOR_STATUS_CARRY | cpu6502_CPU_PROCESSOR_STATUS_OVERFLOW);
	if (!(cpu->state.p & cpu6502_CPU_PROCESSOR_STATUS_DECIMAL) || cpu->flags & cpu6502_CPU_FLAGS_DISABLE_DECIMAL) {
		sum = a + operand + (carry ? 1 : 0);

		if (sum > 0xFF) {
			cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_CARRY;
		}
	} else {
		cpu6502_u8 lower = (a & 0xF) + (b & 0xF) + (carry ? 1 : 0);
		if (lower > 9) {
			carry = true;
			lower = 0;
		}

		cpu6502_u8 upper = (a >> 4) + (b >> 4) + (carry ? 1 : 0);
		if (upper > 9) {
			cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_CARRY;
			upper = 0;
		}

		sum = (upper << 4) | (lower & 0xF);
	}

	set_zn_flags(cpu, sum & 0xFF);

	// If the operand and A are the same sign but the result isn't equal
	if (((a ^ (sum & 0xFF)) & (operand ^ (sum & 0xFF)) & 0x80) != 0) {
		cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_OVERFLOW;
	} else {
		cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_OVERFLOW;
	}

	cpu->state.a = sum & 0xFF;
}

static void sbc(cpu6502_CPU *cpu, cpu6502_u8 a, cpu6502_u8 b) {
	cpu6502_u8 operand = b;
	cpu6502_u16 sum = 0;

	bool carry = cpu->state.p & cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	cpu->state.p &= ~(cpu6502_CPU_PROCESSOR_STATUS_CARRY | cpu6502_CPU_PROCESSOR_STATUS_OVERFLOW);
	if (!(cpu->state.p & cpu6502_CPU_PROCESSOR_STATUS_DECIMAL) || cpu->flags & cpu6502_CPU_FLAGS_DISABLE_DECIMAL) {
		sum = a - operand - (carry ? 1 : 0);
	} else {
		cpu6502_i8 lower = (a & 0xF) - (b & 0xF) - (carry ? 0 : 1);
		if (lower < 0) {
			carry = true;
			lower = 9;
		}

		cpu6502_i8 upper = (a >> 4) - (b >> 4) - (carry ? 0 : 1);
		if (upper < 0) {
			cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_CARRY;
			upper = 9;
		}

		sum = (upper << 4) | (lower & 0xF);
	}

	set_zn_flags(cpu, sum & 0xFF);

	// If the operand and A are the same sign but the result isn't equal
	if (((a ^ (sum & 0xFF)) & (operand ^ (sum & 0xFF)) & 0x80) != 0) {
		cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_OVERFLOW;
	} else {
		cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_OVERFLOW;
	}

	cpu->state.a = sum & 0xFF;
}

static cpu6502_u8 adc_imm(cpu6502_CPU *cpu) {
	adc(cpu, cpu->state.a, cpu->state.instruction[1]);
	return 0;
}

static cpu6502_u8 adc_bus(cpu6502_CPU *cpu) {
	adc(cpu, cpu->state.a, cpu6502_CPU_get_data(cpu));
	return 0;
}

static cpu6502_u8 and_imm(cpu6502_CPU *cpu) {
	cpu->state.a &= cpu->state.instruction[1];
	return 0;
}

static cpu6502_u8 and_bus(cpu6502_CPU *cpu) {
	cpu->state.a = set_zn_flags(cpu, cpu->state.a & cpu6502_CPU_get_data(cpu));
	return 0;
}

static cpu6502_u8 asl_a(cpu6502_CPU *cpu) {
	if (cpu->state.a & 0x80) {
		cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	} else {
		cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	}
	cpu->state.a = set_zn_flags(cpu, cpu->state.a << 1);
	return 0;
}

static cpu6502_u8 asl_bus(cpu6502_CPU *cpu) {
	if (cpu->state.a & 0x80) {
		cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	} else {
		cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	}
	cpu6502_CPU_write_req(cpu, cpu6502_CPU_get_addr(cpu), set_zn_flags(cpu, cpu6502_CPU_get_data(cpu) << 1));
	return 0;
}

static cpu6502_u8 bcc(cpu6502_CPU *cpu) {
	return branch(cpu, !(cpu->state.p & cpu6502_CPU_PROCESSOR_STATUS_CARRY));
}

static cpu6502_u8 bcs(cpu6502_CPU *cpu) {
	return branch(cpu, cpu->state.p & cpu6502_CPU_PROCESSOR_STATUS_CARRY);
}

static cpu6502_u8 beq(cpu6502_CPU *cpu) {
	return branch(cpu, cpu->state.p & cpu6502_CPU_PROCESSOR_STATUS_ZERO);
}

static cpu6502_u8 bit_bus(cpu6502_CPU *cpu) {
	cpu6502_i8 result = cpu->state.a & cpu6502_CPU_get_data(cpu);
	if (result == 0) {
		cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_ZERO;
	} else {
		cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_ZERO;
	}
	if (cpu6502_CPU_get_data(cpu) < 0) {
		cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_NEGATIVE;
	} else {
		cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_NEGATIVE;
	}
	if (cpu6502_CPU_get_data(cpu) & 0b01000000) {
		cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_OVERFLOW;
	} else {
		cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_OVERFLOW;
	}
	return 0;
}

static cpu6502_u8 bmi(cpu6502_CPU *cpu) {
	return branch(cpu, cpu->state.p & cpu6502_CPU_PROCESSOR_STATUS_NEGATIVE);
}

static cpu6502_u8 bne(cpu6502_CPU *cpu) {
	return branch(cpu, !(cpu->state.p & cpu6502_CPU_PROCESSOR_STATUS_ZERO));
}

static cpu6502_u8 bpl(cpu6502_CPU *cpu) {
	return branch(cpu, !(cpu->state.p & cpu6502_CPU_PROCESSOR_STATUS_NEGATIVE));
}

static cpu6502_u8 bvc(cpu6502_CPU *cpu) {
	return branch(cpu, !(cpu->state.p & cpu6502_CPU_PROCESSOR_STATUS_OVERFLOW));
}

static cpu6502_u8 brk0(cpu6502_CPU *cpu) {
	cpu->state.pc++;
	cpu6502_CPU_write_req(cpu, 0x100 + (cpu->state.sp--), (cpu->state.pc >> 8) & 0xFF);
	return 0;
}

static cpu6502_u8 brk1(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, 0x100 + (cpu->state.sp--), cpu->state.pc & 0xFF);
	return 0;
}

static cpu6502_u8 brk2(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, 0x100 + (cpu->state.sp--), cpu->state.p);
	cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_NO_INT;
	return 0;
}

static cpu6502_u8 brk3(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, 0xFFFE);
	return 0;
}

static cpu6502_u8 brk4(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, 0xFFFF);
	cpu->state.pc = cpu6502_CPU_get_data(cpu);
	return 0;
}

static cpu6502_u8 brk5(cpu6502_CPU *cpu) {
	cpu->state.pc |= cpu6502_CPU_get_data(cpu) << 8;
	cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_BRK;
	return 0;
}

static cpu6502_u8 bvs(cpu6502_CPU *cpu) {
	return branch(cpu, cpu->state.p & cpu6502_CPU_PROCESSOR_STATUS_OVERFLOW);
}

static cpu6502_u8 clc(cpu6502_CPU *cpu) {
	cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	return 0;
}

static cpu6502_u8 cld(cpu6502_CPU *cpu) {
	cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_DECIMAL;
	return 0;
}

static cpu6502_u8 cli(cpu6502_CPU *cpu) {
	cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_NO_INT;
	return 0;
}

static cpu6502_u8 clv(cpu6502_CPU *cpu) {
	cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_OVERFLOW;
	return 0;
}

static inline void cmp(cpu6502_CPU *cpu, cpu6502_i8 a, cpu6502_i8 b) {
	if (a == b) {
		cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_ZERO;
	} else {
		cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_ZERO;
	}

	if (a >= b) {
		cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	} else {
		cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	}

	if (a - b < 0) {
		cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_ZERO;
	} else {
		cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_ZERO;
	}
}

static cpu6502_u8 cmp_imm(cpu6502_CPU *cpu) {
	cmp(cpu, cpu->state.a, cpu->state.instruction[1]);
	return 0;
}

static cpu6502_u8 cmp_bus(cpu6502_CPU *cpu) {
	cmp(cpu, cpu->state.a, cpu6502_CPU_get_data(cpu));
	return 0;
}

static cpu6502_u8 cpx_imm(cpu6502_CPU *cpu) {
	cmp(cpu, cpu->state.x, cpu->state.instruction[1]);
	return 0;
}

static cpu6502_u8 cpx_bus(cpu6502_CPU *cpu) {
	cmp(cpu, cpu->state.x, cpu6502_CPU_get_data(cpu));
	return 0;
}

static cpu6502_u8 cpy_imm(cpu6502_CPU *cpu) {
	cmp(cpu, cpu->state.y, cpu->state.instruction[1]);
	return 0;
}

static cpu6502_u8 cpy_bus(cpu6502_CPU *cpu) {
	cmp(cpu, cpu->state.y, cpu6502_CPU_get_data(cpu));
	return 0;
}

static cpu6502_u8 dec_bus(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, cpu6502_CPU_get_addr(cpu), set_zn_flags(cpu, cpu6502_CPU_get_data(cpu) - 1));
	return 0;
}

static cpu6502_u8 dex(cpu6502_CPU *cpu) {
	set_zn_flags(cpu, --cpu->state.x);
	return 0;
}

static cpu6502_u8 dey(cpu6502_CPU *cpu) {
	set_zn_flags(cpu, --cpu->state.y);
	return 0;
}

static cpu6502_u8 eor_imm(cpu6502_CPU *cpu) {
	cpu->state.a = set_zn_flags(cpu, cpu->state.a ^ cpu->state.instruction[1]);
	return 0;
}

static cpu6502_u8 eor_bus(cpu6502_CPU *cpu) {
	cpu->state.a = set_zn_flags(cpu, cpu->state.a ^ cpu6502_CPU_get_data(cpu));
	return 0;
}

static cpu6502_u8 inc_bus(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, cpu6502_CPU_get_addr(cpu), set_zn_flags(cpu, cpu6502_CPU_get_data(cpu) + 1));
	return 0;
}

static cpu6502_u8 inx(cpu6502_CPU *cpu) {
	set_zn_flags(cpu, ++cpu->state.x);
	return 0;
}

static cpu6502_u8 iny(cpu6502_CPU *cpu) {
	set_zn_flags(cpu, ++cpu->state.y);
	return 0;
}

static cpu6502_u8 jmp_abs(cpu6502_CPU *cpu) {
	cpu->state.pc = cpu->state.instruction[1] | (cpu->state.instruction[2] << 8);
	return 0;
}

static cpu6502_u8 jmp_ind0(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, cpu->state.instruction[1] | (cpu->state.instruction[2] << 8));
	return 0;
}

static cpu6502_u8 jmp_ind1(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, (cpu->state.instruction[1] | (cpu->state.instruction[2] << 8)) + 1);
	cpu->state.pc = cpu6502_CPU_get_data(cpu);
	return 0;
}

static cpu6502_u8 jmp_ind2(cpu6502_CPU *cpu) {
	cpu->state.pc |= cpu6502_CPU_get_data(cpu) << 8;
	return 0;
}

static cpu6502_u8 jsr_abs0(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, 0x100 + (cpu->state.sp--), ((cpu->state.pc - 1) >> 8) & 0xFF);
	return 0;
}

static cpu6502_u8 jsr_abs1(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, 0x100 + (cpu->state.sp--), (cpu->state.pc - 1) & 0xFF);
	return 0;
}

static cpu6502_u8 jsr_abs2(cpu6502_CPU *cpu) {
	cpu->state.pc = cpu->state.instruction[1] | (cpu->state.instruction[2] << 8);
	return 0;
}

static cpu6502_u8 lda_imm(cpu6502_CPU *cpu) {
	cpu->state.a = cpu->state.instruction[1];
	return 0;
}

static cpu6502_u8 lda_bus(cpu6502_CPU *cpu) {
	cpu->state.a = cpu6502_CPU_get_data(cpu);
	return 0;
}

static cpu6502_u8 ldx_imm(cpu6502_CPU *cpu) {
	cpu->state.x = cpu->state.instruction[1];
	return 0;
}

static cpu6502_u8 ldx_bus(cpu6502_CPU *cpu) {
	cpu->state.x = cpu6502_CPU_get_data(cpu);
	return 0;
}

static cpu6502_u8 ldy_imm(cpu6502_CPU *cpu) {
	cpu->state.y = cpu->state.instruction[1];
	return 0;
}

static cpu6502_u8 ldy_bus(cpu6502_CPU *cpu) {
	cpu->state.y = cpu6502_CPU_get_data(cpu);
	return 0;
}

static cpu6502_u8 lsr_a(cpu6502_CPU *cpu) {
	if (cpu->state.a & 0x01) {
		cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	} else {
		cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	}
	cpu->state.a = set_zn_flags(cpu, cpu->state.a >> 1);
	return 0;
}

static cpu6502_u8 lsr_bus(cpu6502_CPU *cpu) {
	if (cpu6502_CPU_get_data(cpu) & 0x01) {
		cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	} else {
		cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	}
	cpu6502_CPU_write_req(cpu, cpu6502_CPU_get_addr(cpu), set_zn_flags(cpu, cpu6502_CPU_get_data(cpu) >> 1));
	return 0;
}

static cpu6502_u8 ora_imm(cpu6502_CPU *cpu) {
	cpu->state.a = set_zn_flags(cpu, cpu->state.a | cpu->state.instruction[1]);
	return 0;
}

static cpu6502_u8 ora_bus(cpu6502_CPU *cpu) {
	cpu->state.a = set_zn_flags(cpu, cpu->state.a | cpu6502_CPU_get_data(cpu));
	return 0;
}

static cpu6502_u8 pha(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, 0x100 + (cpu->state.sp--), cpu->state.a);
	return 0;
}

static cpu6502_u8 php(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, 0x100 + (cpu->state.sp--), cpu->state.p);
	return 0;
}

static cpu6502_u8 pla0(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, 0x100 + (cpu->state.sp++));
	return 0;
}

static cpu6502_u8 pla1(cpu6502_CPU *cpu) {
	cpu->state.a = cpu6502_CPU_get_data(cpu);
	return 0;
}

static cpu6502_u8 plp0(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, 0x100 + (cpu->state.sp++));
	return 0;
}

static cpu6502_u8 plp1(cpu6502_CPU *cpu) {
	cpu->state.p = cpu6502_CPU_get_data(cpu);
	return 0;
}

static cpu6502_u8 rol_a(cpu6502_CPU *cpu) {
	cpu6502_i8 prev_a = cpu->state.a;
	cpu->state.a = set_zn_flags(cpu, (cpu->state.a << 1) | ((cpu->state.p & cpu6502_CPU_PROCESSOR_STATUS_CARRY) ? 0x01 : 0x00));
	if (prev_a & 0x80) {
		cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	} else {
		cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	}
	return 0;
}

static cpu6502_u8 rol_bus(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, cpu6502_CPU_get_addr(cpu), set_zn_flags(cpu, ((cpu6502_CPU_get_data(cpu) << 1) | (cpu->state.p & cpu6502_CPU_PROCESSOR_STATUS_CARRY) ? 0x01 : 0x00)));
	if (cpu6502_CPU_get_data(cpu) & 0x80) {
		cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	} else {
		cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	}
	return 0;
}

static cpu6502_u8 ror_a(cpu6502_CPU *cpu) {
	cpu6502_i8 prev_a = cpu->state.a;
	cpu->state.a = set_zn_flags(cpu, (cpu->state.a >> 1) | ((cpu->state.p & cpu6502_CPU_PROCESSOR_STATUS_CARRY) ? 0x80 : 0x00));
	if (prev_a & 0x01) {
		cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	} else {
		cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	}
	return 0;
}

static cpu6502_u8 ror_bus(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, cpu6502_CPU_get_addr(cpu), set_zn_flags(cpu, ((cpu6502_CPU_get_data(cpu) >> 1) | (cpu->state.p & cpu6502_CPU_PROCESSOR_STATUS_CARRY) ? 0x80 : 0x00)));
	if (cpu6502_CPU_get_data(cpu) & 0x01) {
		cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	} else {
		cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	}
	return 0;
}

static cpu6502_u8 rts0(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, 0x100 + (cpu->state.sp++));
	return 0;
}

static cpu6502_u8 rts1(cpu6502_CPU *cpu) {
	cpu6502_CPU_read_req(cpu, 0x100 + (cpu->state.sp++));
	cpu->state.pc = cpu6502_CPU_get_data(cpu) << 8;
	return 0;
}

static cpu6502_u8 rts2(cpu6502_CPU *cpu) {
	cpu->state.pc |= cpu6502_CPU_get_data(cpu);
	return 0;
}

static cpu6502_u8 sbc_imm(cpu6502_CPU *cpu) {
	sbc(cpu, cpu->state.a, cpu->state.instruction[1]);
	return 0;
}

static cpu6502_u8 sbc_bus(cpu6502_CPU *cpu) {
	sbc(cpu, cpu->state.a, cpu6502_CPU_get_data(cpu));
	return 0;
}

static cpu6502_u8 sec(cpu6502_CPU *cpu) {
	cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	return 0;
}

static cpu6502_u8 sed(cpu6502_CPU *cpu) {
	cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_DECIMAL; // We don't a shit about this
	return 0;
}

static cpu6502_u8 sei(cpu6502_CPU *cpu) {
	cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_NO_INT;
	return 0;
}

static cpu6502_u8 sta_bus(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, cpu6502_CPU_get_addr(cpu), cpu->state.a);
	return 0;
}

static cpu6502_u8 stx_bus(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, cpu6502_CPU_get_addr(cpu), cpu->state.x);
	return 0;
}

static cpu6502_u8 sty_bus(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, cpu6502_CPU_get_addr(cpu), cpu->state.y);
	return 0;
}

static cpu6502_u8 tax(cpu6502_CPU *cpu) {
	cpu->state.x = cpu->state.a;
	return 0;
}

static cpu6502_u8 tay(cpu6502_CPU *cpu) {
	cpu->state.y = cpu->state.a;
	return 0;
}

static cpu6502_u8 tsx(cpu6502_CPU *cpu) {
	cpu->state.x = cpu->state.sp;
	return 0;
}

static cpu6502_u8 txa(cpu6502_CPU *cpu) {
	cpu->state.a = cpu->state.x;
	return 0;
}

static cpu6502_u8 txs(cpu6502_CPU *cpu) {
	cpu->state.sp = cpu->state.x;
	return 0;
}

static cpu6502_u8 tya(cpu6502_CPU *cpu) {
	cpu->state.a = cpu->state.y;
	return 0;
}

static cpu6502_u8 tnc(cpu6502_CPU *cpu) {
	if (cpu->state.p & cpu6502_CPU_PROCESSOR_STATUS_NEGATIVE) {
		cpu->state.p |= cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	} else {
		cpu->state.p &= ~cpu6502_CPU_PROCESSOR_STATUS_CARRY;
	}
	return 0;
}

static cpu6502_u8 jam(cpu6502_CPU *cpu) {
	cpu->state.halted = true;
	return 0;
}

static cpu6502_u8 las_bus(cpu6502_CPU *cpu) {
	cpu->state.sp &= cpu6502_CPU_get_data(cpu);
	return 0;
}

static cpu6502_u8 sax_bus(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, cpu6502_CPU_get_addr(cpu), set_zn_flags(cpu, cpu->state.a & cpu->state.x));
	return 0;
}

static cpu6502_u8 sbx_imm(cpu6502_CPU *cpu) {
	cpu->state.x = (cpu->state.a & cpu->state.x) - cpu->state.instruction[1];
	return 0;
}

static cpu6502_u8 sha_ind_y_abs(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, cpu6502_CPU_get_addr(cpu), cpu->state.a & cpu->state.x & cpu6502_CPU_get_data(cpu));
	return 0;
}

static cpu6502_u8 sha_zp_y(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, cpu6502_CPU_get_addr(cpu), cpu->state.a & cpu->state.x & ((cpu6502_CPU_get_addr(cpu) >> 8) & 0xFF));
	return 0;
}

static cpu6502_u8 shs_abs_y(cpu6502_CPU *cpu) {
	cpu->state.sp = cpu->state.a & cpu->state.x;
	cpu6502_CPU_write_req(cpu, cpu6502_CPU_get_addr(cpu), cpu->state.sp & (cpu->state.instruction[2] + 1));
	return 0;
}

static cpu6502_u8 shx_abs_y(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, cpu6502_CPU_get_addr(cpu), cpu->state.x & (cpu->state.instruction[2] + 1));
	return 0;
}

static cpu6502_u8 shy_abs_x(cpu6502_CPU *cpu) {
	cpu6502_CPU_write_req(cpu, cpu6502_CPU_get_addr(cpu), cpu->state.y & (cpu->state.instruction[2] + 1));
	return 0;
}

static cpu6502_u8 xaa_imm(cpu6502_CPU *cpu) {
	cpu6502_u8 magic = cpu6502_CPU_next_rnd(cpu);
	cpu->state.a = set_zn_flags(cpu, (cpu->state.a & magic) & cpu->state.x & cpu->state.instruction[1]);
	return 0;
}

cpu6502_CPU_InstructionFunc cpu6502_CPU_instructions[256][cpu6502_MAX_CPU_INSTRUCTION_CYCLES] = {
    [0x69] = {adc_imm, cpu6502_CPU_INSTRUCTION_END},                                                 // ADC #imm
    [0x65] = {bus_zp, adc_bus, cpu6502_CPU_INSTRUCTION_END},                                         // ADC $zp
    [0x75] = {bus_zp_x, adc_bus, cpu6502_CPU_INSTRUCTION_END},                                       // ADC $zp,X
    [0x6D] = {bus_abs, adc_bus, cpu6502_CPU_INSTRUCTION_END},                                        // ADC $abs
    [0x7D] = {bus_abs_x, adc_bus, cpu6502_CPU_INSTRUCTION_END},                                      // ADC $abs,X
    [0x79] = {bus_abs_y, adc_bus, cpu6502_CPU_INSTRUCTION_END},                                      // ADC $abs,Y
    [0x61] = {bus_idx_ind_x0, bus_idx_ind_x1, bus_idx_ind_x2, adc_bus, cpu6502_CPU_INSTRUCTION_END}, // ADC ($ind,X)
    [0x71] = {bus_ind_idx_y0, bus_ind_idx_y1, bus_ind_idx_y2, adc_bus, cpu6502_CPU_INSTRUCTION_END}, // ADC ($ind),Y

    [0x29] = {and_imm, cpu6502_CPU_INSTRUCTION_END},                                                 // AND #imm
    [0x25] = {bus_zp, and_bus, cpu6502_CPU_INSTRUCTION_END},                                         // AND $zp
    [0x35] = {bus_zp_x, and_bus, cpu6502_CPU_INSTRUCTION_END},                                       // AND $zp,X
    [0x2D] = {bus_abs, and_bus, cpu6502_CPU_INSTRUCTION_END},                                        // AND $abs
    [0x3D] = {bus_abs_x, and_bus, cpu6502_CPU_INSTRUCTION_END},                                      // AND $abs,X
    [0x39] = {bus_abs_y, and_bus, cpu6502_CPU_INSTRUCTION_END},                                      // AND $abs,Y
    [0x21] = {bus_idx_ind_x0, bus_idx_ind_x1, bus_idx_ind_x2, and_bus, cpu6502_CPU_INSTRUCTION_END}, // AND ($ind,X)
    [0x31] = {bus_ind_idx_y0, bus_ind_idx_y1, bus_ind_idx_y2, and_bus, cpu6502_CPU_INSTRUCTION_END}, // AND ($ind),Y

    [0x0A] = {asl_a, cpu6502_CPU_INSTRUCTION_END},                   // ASL A
    [0x06] = {bus_zp, asl_bus, nop, cpu6502_CPU_INSTRUCTION_END},    // ASL $zp
    [0x16] = {bus_zp_x, asl_bus, nop, cpu6502_CPU_INSTRUCTION_END},  // ASL $zp,X
    [0x0E] = {bus_abs, asl_bus, nop, cpu6502_CPU_INSTRUCTION_END},   // ASL $abs
    [0x1E] = {bus_abs_x, asl_bus, nop, cpu6502_CPU_INSTRUCTION_END}, // ASL $abs,X

    [0x90] = {bcc, cpu6502_CPU_INSTRUCTION_END}, // BCC rel
    [0xB0] = {bcs, cpu6502_CPU_INSTRUCTION_END}, // BCS rel
    [0xF0] = {beq, cpu6502_CPU_INSTRUCTION_END}, // BEQ rel

    [0x24] = {bus_zp, bit_bus, cpu6502_CPU_INSTRUCTION_END},  // BIT $zp
    [0x2C] = {bus_abs, bit_bus, cpu6502_CPU_INSTRUCTION_END}, // BIT $abs

    [0x30] = {bmi, cpu6502_CPU_INSTRUCTION_END},                                // BMI
    [0xD0] = {bne, cpu6502_CPU_INSTRUCTION_END},                                // BNE
    [0x10] = {bpl, cpu6502_CPU_INSTRUCTION_END},                                // BPL
    [0x00] = {brk0, brk1, brk2, brk3, brk4, brk5, cpu6502_CPU_INSTRUCTION_END}, // BRK
    [0x50] = {bvc, cpu6502_CPU_INSTRUCTION_END},                                // BVC
    [0x70] = {bvs, cpu6502_CPU_INSTRUCTION_END},                                // BVS

    [0x18] = {clc, cpu6502_CPU_INSTRUCTION_END}, // CLC
    [0xD8] = {cld, cpu6502_CPU_INSTRUCTION_END}, // CLD
    [0x58] = {cli, cpu6502_CPU_INSTRUCTION_END}, // CLI
    [0xB8] = {clv, cpu6502_CPU_INSTRUCTION_END}, // CLV

    [0xC9] = {cmp_imm, cpu6502_CPU_INSTRUCTION_END},                                                 // CMP #imm
    [0xC5] = {bus_zp, cmp_bus, cpu6502_CPU_INSTRUCTION_END},                                         // CMP $zp
    [0xD5] = {bus_zp_x, cmp_bus, cpu6502_CPU_INSTRUCTION_END},                                       // CMP $zp,X
    [0xCD] = {bus_abs, cmp_bus, cpu6502_CPU_INSTRUCTION_END},                                        // CMP $abs
    [0xDD] = {bus_abs_x, cmp_bus, cpu6502_CPU_INSTRUCTION_END},                                      // CMP $abs,X
    [0xD9] = {bus_abs_y, cmp_bus, cpu6502_CPU_INSTRUCTION_END},                                      // CMP $abs,Y
    [0xC1] = {bus_idx_ind_x0, bus_idx_ind_x1, bus_idx_ind_x2, cmp_bus, cpu6502_CPU_INSTRUCTION_END}, // CMP ($ind,X)
    [0xD1] = {bus_ind_idx_y0, bus_ind_idx_y1, bus_ind_idx_y2, cmp_bus, cpu6502_CPU_INSTRUCTION_END}, // CMP ($ind),Y

    [0xE0] = {cpx_imm, cpu6502_CPU_INSTRUCTION_END},          // CPX #imm
    [0xE4] = {bus_zp, cpx_bus, cpu6502_CPU_INSTRUCTION_END},  // CPX $zp
    [0xEC] = {bus_abs, cpx_bus, cpu6502_CPU_INSTRUCTION_END}, // CPX $abs

    [0xC0] = {cpy_imm, cpu6502_CPU_INSTRUCTION_END},          // CPY #imm
    [0xC4] = {bus_zp, cpy_bus, cpu6502_CPU_INSTRUCTION_END},  // CPY $zp
    [0xCC] = {bus_abs, cpy_bus, cpu6502_CPU_INSTRUCTION_END}, // CPY $abs

    [0xC6] = {bus_zp, dec_bus, nop, cpu6502_CPU_INSTRUCTION_END},    // DEC $zp
    [0xD6] = {bus_zp_x, dec_bus, nop, cpu6502_CPU_INSTRUCTION_END},  // DEC $zp,X
    [0xCE] = {bus_abs, dec_bus, nop, cpu6502_CPU_INSTRUCTION_END},   // DEC $abs
    [0xDE] = {bus_abs_x, dec_bus, nop, cpu6502_CPU_INSTRUCTION_END}, // DEC $abs,X

    [0xCA] = {dex, cpu6502_CPU_INSTRUCTION_END}, // DEX
    [0x88] = {dey, cpu6502_CPU_INSTRUCTION_END}, // DEY

    [0x49] = {eor_imm, cpu6502_CPU_INSTRUCTION_END},                                                 // EOR #imm
    [0x45] = {bus_zp, eor_bus, cpu6502_CPU_INSTRUCTION_END},                                         // EOR $zp
    [0x55] = {bus_zp_x, eor_bus, cpu6502_CPU_INSTRUCTION_END},                                       // EOR $zp,X
    [0x4D] = {bus_abs, eor_bus, cpu6502_CPU_INSTRUCTION_END},                                        // EOR $abs
    [0x5D] = {bus_abs_x, eor_bus, cpu6502_CPU_INSTRUCTION_END},                                      // EOR $abs,X
    [0x59] = {bus_abs_y, eor_bus, cpu6502_CPU_INSTRUCTION_END},                                      // EOR $abs,Y
    [0x41] = {bus_idx_ind_x0, bus_idx_ind_x1, bus_idx_ind_x2, eor_bus, cpu6502_CPU_INSTRUCTION_END}, // EOR ($ind,X)
    [0x51] = {bus_ind_idx_y0, bus_ind_idx_y1, bus_ind_idx_y2, eor_bus, cpu6502_CPU_INSTRUCTION_END}, // EOR ($ind),Y

    [0xE6] = {bus_zp, inc_bus, nop, cpu6502_CPU_INSTRUCTION_END},    // INC $zp
    [0xF6] = {bus_zp_x, inc_bus, nop, cpu6502_CPU_INSTRUCTION_END},  // INC $zp,X
    [0xEE] = {bus_abs, inc_bus, nop, cpu6502_CPU_INSTRUCTION_END},   // INC $abs
    [0xFE] = {bus_abs_x, inc_bus, nop, cpu6502_CPU_INSTRUCTION_END}, // INC $abs,X

    [0xE8] = {inx, cpu6502_CPU_INSTRUCTION_END}, // INX
    [0xC8] = {iny, cpu6502_CPU_INSTRUCTION_END}, // INY

    [0x4C] = {jmp_abs, cpu6502_CPU_INSTRUCTION_END},                      // JMP $abs
    [0x6C] = {jmp_ind0, jmp_ind1, jmp_ind2, cpu6502_CPU_INSTRUCTION_END}, // JMP ($ind)

    [0x20] = {jsr_abs0, jsr_abs1, jsr_abs2, cpu6502_CPU_INSTRUCTION_END}, // JSR $abs

    [0xA9] = {lda_imm, cpu6502_CPU_INSTRUCTION_END},                                                 // LDA #imm
    [0xA5] = {bus_zp, lda_bus, cpu6502_CPU_INSTRUCTION_END},                                         // LDA $zp
    [0xB5] = {bus_zp_x, lda_bus, cpu6502_CPU_INSTRUCTION_END},                                       // LDA $zp,X
    [0xAD] = {bus_abs, lda_bus, cpu6502_CPU_INSTRUCTION_END},                                        // LDA $abs
    [0xBD] = {bus_abs_x, lda_bus, cpu6502_CPU_INSTRUCTION_END},                                      // LDA $abs,X
    [0xB9] = {bus_abs_y, lda_bus, cpu6502_CPU_INSTRUCTION_END},                                      // LDA $abs,Y
    [0xA1] = {bus_idx_ind_x0, bus_idx_ind_x1, bus_idx_ind_x2, lda_bus, cpu6502_CPU_INSTRUCTION_END}, // LDA ($ind,X)
    [0xB1] = {bus_ind_idx_y0, bus_ind_idx_y1, bus_ind_idx_y2, lda_bus, cpu6502_CPU_INSTRUCTION_END}, // LDA ($ind),Y

    [0xA2] = {ldx_imm, cpu6502_CPU_INSTRUCTION_END},            // LDX #imm
    [0xA6] = {bus_zp, ldx_bus, cpu6502_CPU_INSTRUCTION_END},    // LDX $zp
    [0xB6] = {bus_zp_y, ldx_bus, cpu6502_CPU_INSTRUCTION_END},  // LDX $zp,X
    [0xAE] = {bus_abs, ldx_bus, cpu6502_CPU_INSTRUCTION_END},   // LDX $abs
    [0xBE] = {bus_abs_y, ldx_bus, cpu6502_CPU_INSTRUCTION_END}, // LDX $abs,Y

    [0xA0] = {ldy_imm, cpu6502_CPU_INSTRUCTION_END},            // LDY #imm
    [0xA4] = {bus_zp, ldy_bus, cpu6502_CPU_INSTRUCTION_END},    // LDY $zp
    [0xB4] = {bus_zp_y, ldy_bus, cpu6502_CPU_INSTRUCTION_END},  // LDY $zp,X
    [0xAC] = {bus_abs, ldy_bus, cpu6502_CPU_INSTRUCTION_END},   // LDY $abs
    [0xBC] = {bus_abs_y, ldy_bus, cpu6502_CPU_INSTRUCTION_END}, // LDY $abs,Y

    [0x4A] = {lsr_a, cpu6502_CPU_INSTRUCTION_END},                   // LSR A
    [0x46] = {bus_zp, lsr_bus, nop, cpu6502_CPU_INSTRUCTION_END},    // LSR $zp
    [0x56] = {bus_zp_x, lsr_bus, nop, cpu6502_CPU_INSTRUCTION_END},  // LSR $zp,X
    [0x4E] = {bus_abs, lsr_bus, nop, cpu6502_CPU_INSTRUCTION_END},   // LSR $abs
    [0x5E] = {bus_abs_x, lsr_bus, nop, cpu6502_CPU_INSTRUCTION_END}, // LSR $abs,X

    [0xEA] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP

    [0x09] = {ora_imm, cpu6502_CPU_INSTRUCTION_END},                                                 // ORA #imm
    [0x05] = {bus_zp, ora_bus, cpu6502_CPU_INSTRUCTION_END},                                         // ORA $zp
    [0x15] = {bus_zp_x, ora_bus, cpu6502_CPU_INSTRUCTION_END},                                       // ORA $zp,X
    [0x0D] = {bus_abs, ora_bus, cpu6502_CPU_INSTRUCTION_END},                                        // ORA $abs
    [0x1D] = {bus_abs_x, ora_bus, cpu6502_CPU_INSTRUCTION_END},                                      // ORA $abs,X
    [0x19] = {bus_abs_y, ora_bus, cpu6502_CPU_INSTRUCTION_END},                                      // ORA $abs,Y
    [0x01] = {bus_idx_ind_x0, bus_idx_ind_x1, bus_idx_ind_x2, ora_bus, cpu6502_CPU_INSTRUCTION_END}, // ORA ($ind,X)
    [0x11] = {bus_ind_idx_y0, bus_ind_idx_y1, bus_ind_idx_y2, ora_bus, cpu6502_CPU_INSTRUCTION_END}, // ORA ($ind),Y

    [0x48] = {pha, nop, cpu6502_CPU_INSTRUCTION_END},   // PHA
    [0x08] = {php, nop, cpu6502_CPU_INSTRUCTION_END},   // PHP
    [0x68] = {pla0, pla1, cpu6502_CPU_INSTRUCTION_END}, // PLA
    [0x28] = {plp0, plp1, cpu6502_CPU_INSTRUCTION_END}, // PLP

    [0x2A] = {rol_a, cpu6502_CPU_INSTRUCTION_END},                   // ROL A
    [0x26] = {bus_zp, rol_bus, nop, cpu6502_CPU_INSTRUCTION_END},    // ROL $zp
    [0x36] = {bus_zp_x, rol_bus, nop, cpu6502_CPU_INSTRUCTION_END},  // ROL $zp,X
    [0x2E] = {bus_abs, rol_bus, nop, cpu6502_CPU_INSTRUCTION_END},   // ROL $abs
    [0x3E] = {bus_abs_x, rol_bus, nop, cpu6502_CPU_INSTRUCTION_END}, // ROL $abs,X

    [0x6A] = {ror_a, cpu6502_CPU_INSTRUCTION_END},                   // ROR A
    [0x66] = {bus_zp, ror_bus, nop, cpu6502_CPU_INSTRUCTION_END},    // ROR $zp
    [0x76] = {bus_zp_x, ror_bus, nop, cpu6502_CPU_INSTRUCTION_END},  // ROR $zp,X
    [0x6E] = {bus_abs, ror_bus, nop, cpu6502_CPU_INSTRUCTION_END},   // ROR $abs
    [0x7E] = {bus_abs_x, ror_bus, nop, cpu6502_CPU_INSTRUCTION_END}, // ROR $abs,X

    [0x60] = {rts0, rts1, rts2, inc_pc, cpu6502_CPU_INSTRUCTION_END},                                    // RTS
    [0x40] = {plp0, plp1, cpu6502_CPU_INSTRUCTION_NO_WAIT, rts0, rts1, rts2, cpu6502_CPU_INSTRUCTION_END}, // RTI

    [0xE9] = {sbc_imm, cpu6502_CPU_INSTRUCTION_END},                                                 // SBC #imm
    [0xE5] = {bus_zp, sbc_bus, cpu6502_CPU_INSTRUCTION_END},                                         // SBC $zp
    [0xF5] = {bus_zp_x, sbc_bus, cpu6502_CPU_INSTRUCTION_END},                                       // SBC $zp,X
    [0xED] = {bus_abs, sbc_bus, cpu6502_CPU_INSTRUCTION_END},                                        // SBC $abs
    [0xFD] = {bus_abs_x, sbc_bus, cpu6502_CPU_INSTRUCTION_END},                                      // SBC $abs,X
    [0xF9] = {bus_abs_y, sbc_bus, cpu6502_CPU_INSTRUCTION_END},                                      // SBC $abs,Y
    [0xE1] = {bus_idx_ind_x0, bus_idx_ind_x1, bus_idx_ind_x2, sbc_bus, cpu6502_CPU_INSTRUCTION_END}, // SBC ($ind,X)
    [0xF1] = {bus_ind_idx_y0, bus_ind_idx_y1, bus_ind_idx_y2, sbc_bus, cpu6502_CPU_INSTRUCTION_END}, // SBC ($ind),Y

    [0x38] = {sec, cpu6502_CPU_INSTRUCTION_END}, // SEC
    [0xF8] = {sed, cpu6502_CPU_INSTRUCTION_END}, // SED
    [0x78] = {sei, cpu6502_CPU_INSTRUCTION_END}, // SEI

    [0x85] = {bus_zp, cpu6502_CPU_INSTRUCTION_NO_WAIT, sta_bus, nop, cpu6502_CPU_INSTRUCTION_END},                                         // STA $zp
    [0x95] = {bus_zp_x, cpu6502_CPU_INSTRUCTION_NO_WAIT, sta_bus, nop, cpu6502_CPU_INSTRUCTION_END},                                       // STA $zp,X
    [0x8D] = {bus_abs, cpu6502_CPU_INSTRUCTION_NO_WAIT, sta_bus, nop, cpu6502_CPU_INSTRUCTION_END},                                        // STA $abs
    [0x9D] = {bus_abs_x, cpu6502_CPU_INSTRUCTION_NO_WAIT, sta_bus, nop, cpu6502_CPU_INSTRUCTION_END},                                      // STA $abs,X
    [0x99] = {bus_abs_y, cpu6502_CPU_INSTRUCTION_NO_WAIT, sta_bus, nop, cpu6502_CPU_INSTRUCTION_END},                                      // STA $abs,Y
    [0x81] = {bus_idx_ind_x0, bus_idx_ind_x1, bus_idx_ind_x2, cpu6502_CPU_INSTRUCTION_NO_WAIT, sta_bus, nop, cpu6502_CPU_INSTRUCTION_END}, // STA ($ind,X)
    [0x91] = {bus_ind_idx_y0, bus_ind_idx_y1, bus_ind_idx_y2, cpu6502_CPU_INSTRUCTION_NO_WAIT, sta_bus, nop, cpu6502_CPU_INSTRUCTION_END}, // STA ($ind),Y

    [0x86] = {bus_zp, cpu6502_CPU_INSTRUCTION_NO_WAIT, stx_bus, nop, cpu6502_CPU_INSTRUCTION_END},   // STX $zp
    [0x96] = {bus_zp_y, cpu6502_CPU_INSTRUCTION_NO_WAIT, stx_bus, nop, cpu6502_CPU_INSTRUCTION_END}, // STX $zp,Y
    [0x8E] = {bus_abs, cpu6502_CPU_INSTRUCTION_NO_WAIT, stx_bus, nop, cpu6502_CPU_INSTRUCTION_END},  // STX $abs

    [0x84] = {bus_zp, cpu6502_CPU_INSTRUCTION_NO_WAIT, sty_bus, nop, cpu6502_CPU_INSTRUCTION_END},   // STY $zp
    [0x94] = {bus_zp_y, cpu6502_CPU_INSTRUCTION_NO_WAIT, sty_bus, nop, cpu6502_CPU_INSTRUCTION_END}, // STY $zp,Y
    [0x8C] = {bus_abs, cpu6502_CPU_INSTRUCTION_NO_WAIT, sty_bus, nop, cpu6502_CPU_INSTRUCTION_END},  // STY $abs

    [0xAA] = {tax, cpu6502_CPU_INSTRUCTION_END}, // TAX
    [0xA8] = {tay, cpu6502_CPU_INSTRUCTION_END}, // TAY
    [0xBA] = {tsx, cpu6502_CPU_INSTRUCTION_END}, // TSX
    [0x8A] = {txa, cpu6502_CPU_INSTRUCTION_END}, // TXA
    [0x9A] = {txs, cpu6502_CPU_INSTRUCTION_END}, // TXS
    [0x98] = {tya, cpu6502_CPU_INSTRUCTION_END}, // TYA

    // Undocumented
    [0x0B] = {and_imm, cpu6502_CPU_INSTRUCTION_NO_WAIT, tnc, cpu6502_CPU_INSTRUCTION_END}, // ANC #imm
    [0x2B] = {and_imm, cpu6502_CPU_INSTRUCTION_NO_WAIT, tnc, cpu6502_CPU_INSTRUCTION_END}, // ANC #imm

    [0x6B] = {and_imm, cpu6502_CPU_INSTRUCTION_NO_WAIT, ror_a, cpu6502_CPU_INSTRUCTION_END}, // ARR #imm

    [0x4B] = {and_imm, cpu6502_CPU_INSTRUCTION_NO_WAIT, lsr_a, cpu6502_CPU_INSTRUCTION_END}, // ASR #imm

    [0xCF] = {bus_abs, dec_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, cmp_bus, cpu6502_CPU_INSTRUCTION_END},                                        // DCP $abs
    [0xDF] = {bus_abs_x, dec_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, cmp_bus, cpu6502_CPU_INSTRUCTION_END},                                      // DCP $abs,X
    [0xDB] = {bus_abs_y, dec_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, cmp_bus, cpu6502_CPU_INSTRUCTION_END},                                      // DCP $abs,Y
    [0xC7] = {bus_zp, dec_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, cmp_bus, cpu6502_CPU_INSTRUCTION_END},                                         // DCP $zp,
    [0xD7] = {bus_zp_x, dec_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, cmp_bus, cpu6502_CPU_INSTRUCTION_END},                                       // DCP $zp,X
    [0xC3] = {bus_idx_ind_x0, bus_idx_ind_x1, bus_idx_ind_x2, dec_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, cmp_bus, cpu6502_CPU_INSTRUCTION_END}, // DCP ($ind,X)
    [0xD3] = {bus_ind_idx_y0, bus_ind_idx_y1, bus_ind_idx_y2, dec_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, cmp_bus, cpu6502_CPU_INSTRUCTION_END}, // DCP ($ind),Y

    [0xEF] = {bus_abs, inc_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, sbc_bus, cpu6502_CPU_INSTRUCTION_END},                                        // ISC $abs
    [0xFF] = {bus_abs_x, inc_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, sbc_bus, cpu6502_CPU_INSTRUCTION_END},                                      // ISC $abs,X
    [0xFB] = {bus_abs_y, inc_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, sbc_bus, cpu6502_CPU_INSTRUCTION_END},                                      // ISC $abs,Y
    [0xE7] = {bus_zp, inc_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, sbc_bus, cpu6502_CPU_INSTRUCTION_END},                                         // ISC $zp,
    [0xF7] = {bus_zp_x, inc_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, sbc_bus, cpu6502_CPU_INSTRUCTION_END},                                       // ISC $zp,X
    [0xE3] = {bus_idx_ind_x0, bus_idx_ind_x1, bus_idx_ind_x2, inc_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, sbc_bus, cpu6502_CPU_INSTRUCTION_END}, // ISC ($ind,X)
    [0xF3] = {bus_ind_idx_y0, bus_ind_idx_y1, bus_ind_idx_y2, inc_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, sbc_bus, cpu6502_CPU_INSTRUCTION_END}, // ISC ($ind),Y

    [0x02] = {jam, cpu6502_CPU_INSTRUCTION_END}, // JAM
    [0x12] = {jam, cpu6502_CPU_INSTRUCTION_END}, // JAM
    [0x22] = {jam, cpu6502_CPU_INSTRUCTION_END}, // JAM
    [0x32] = {jam, cpu6502_CPU_INSTRUCTION_END}, // JAM
    [0x42] = {jam, cpu6502_CPU_INSTRUCTION_END}, // JAM
    [0x52] = {jam, cpu6502_CPU_INSTRUCTION_END}, // JAM
    [0x62] = {jam, cpu6502_CPU_INSTRUCTION_END}, // JAM
    [0x72] = {jam, cpu6502_CPU_INSTRUCTION_END}, // JAM
    [0x92] = {jam, cpu6502_CPU_INSTRUCTION_END}, // JAM
    [0xB2] = {jam, cpu6502_CPU_INSTRUCTION_END}, // JAM
    [0xD2] = {jam, cpu6502_CPU_INSTRUCTION_END}, // JAM
    [0xF2] = {jam, cpu6502_CPU_INSTRUCTION_END}, // JAM

    [0xBB] = {bus_abs_y, las_bus, cpu6502_CPU_INSTRUCTION_END}, // LAS $abs,Y

    [0xAB] = {lda_imm, cpu6502_CPU_INSTRUCTION_NO_WAIT, ldx_imm, cpu6502_CPU_INSTRUCTION_END},                                                 // LAX #imm
    [0xAF] = {bus_abs, lda_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, ldx_bus, cpu6502_CPU_INSTRUCTION_END},                                        // LAX $abs
    [0xBF] = {bus_abs_y, lda_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, ldx_bus, cpu6502_CPU_INSTRUCTION_END},                                      // LAX $abs,Y
    [0xA7] = {bus_zp, lda_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, ldx_bus, cpu6502_CPU_INSTRUCTION_END},                                         // LAX $zp
    [0xB7] = {bus_zp_y, lda_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, ldx_bus, cpu6502_CPU_INSTRUCTION_END},                                       // LAX $zp,Y
    [0xA3] = {bus_idx_ind_x0, bus_idx_ind_x1, bus_idx_ind_x2, lda_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, ldx_bus, cpu6502_CPU_INSTRUCTION_END}, // LAX ($zp,X)
    [0xB3] = {bus_ind_idx_y0, bus_ind_idx_y1, bus_ind_idx_y2, lda_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, ldx_bus, cpu6502_CPU_INSTRUCTION_END}, // LAX ($zp),Y

    [0x1A] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP
    [0x3A] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP
    [0x5A] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP
    [0x7A] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP
    [0xDA] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP
    [0xFA] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP
    [0x80] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP #imm
    [0x82] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP #imm
    [0x89] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP #imm
    [0xC2] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP #imm
    [0xE2] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP #imm
    [0x0C] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP $abs
    [0x1C] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP $abs,X
    [0x3C] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP $abs,X
    [0x5C] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP $abs,X
    [0x7C] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP $abs,X
    [0xDC] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP $abs,X
    [0xFC] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP $abs,X
    [0x04] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP $zp
    [0x44] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP $zp
    [0x64] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP $zp
    [0x14] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP $zp,X
    [0x34] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP $zp,X
    [0x54] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP $zp,X
    [0x74] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP $zp,X
    [0xD4] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP $zp,X
    [0xF4] = {nop, cpu6502_CPU_INSTRUCTION_END}, // NOP $zp,X

    [0x2F] = {bus_abs, rol_a, cpu6502_CPU_INSTRUCTION_NO_WAIT, and_bus, cpu6502_CPU_INSTRUCTION_END},                                        // RLA $abs
    [0x3F] = {bus_abs_x, rol_a, cpu6502_CPU_INSTRUCTION_NO_WAIT, and_bus, cpu6502_CPU_INSTRUCTION_END},                                      // RLA $abs,X
    [0x3B] = {bus_abs_y, rol_a, cpu6502_CPU_INSTRUCTION_NO_WAIT, and_bus, cpu6502_CPU_INSTRUCTION_END},                                      // RLA $abs,Y
    [0x27] = {bus_zp, rol_a, cpu6502_CPU_INSTRUCTION_NO_WAIT, and_bus, cpu6502_CPU_INSTRUCTION_END},                                         // RLA $zp
    [0x37] = {bus_zp_x, rol_a, cpu6502_CPU_INSTRUCTION_NO_WAIT, and_bus, cpu6502_CPU_INSTRUCTION_END},                                       // RLA $zp,X
    [0x23] = {bus_idx_ind_x0, bus_idx_ind_x1, bus_idx_ind_x2, rol_a, cpu6502_CPU_INSTRUCTION_NO_WAIT, and_bus, cpu6502_CPU_INSTRUCTION_END}, // RLA ($ind,X)
    [0x33] = {bus_ind_idx_y0, bus_ind_idx_y1, bus_ind_idx_y2, rol_a, cpu6502_CPU_INSTRUCTION_NO_WAIT, and_bus, cpu6502_CPU_INSTRUCTION_END}, // RLA ($ind),Y

    [0x6F] = {bus_abs, ror_a, cpu6502_CPU_INSTRUCTION_NO_WAIT, adc_bus, cpu6502_CPU_INSTRUCTION_END},                                        // RRA $abs
    [0x7F] = {bus_abs_x, ror_a, cpu6502_CPU_INSTRUCTION_NO_WAIT, adc_bus, cpu6502_CPU_INSTRUCTION_END},                                      // RRA $abs,X
    [0x7B] = {bus_abs_y, ror_a, cpu6502_CPU_INSTRUCTION_NO_WAIT, adc_bus, cpu6502_CPU_INSTRUCTION_END},                                      // RRA $abs,Y
    [0x67] = {bus_zp, ror_a, cpu6502_CPU_INSTRUCTION_NO_WAIT, adc_bus, cpu6502_CPU_INSTRUCTION_END},                                         // RRA $zp
    [0x77] = {bus_zp_x, ror_a, cpu6502_CPU_INSTRUCTION_NO_WAIT, adc_bus, cpu6502_CPU_INSTRUCTION_END},                                       // RRA $zp,X
    [0x63] = {bus_idx_ind_x0, bus_idx_ind_x1, bus_idx_ind_x2, ror_a, cpu6502_CPU_INSTRUCTION_NO_WAIT, adc_bus, cpu6502_CPU_INSTRUCTION_END}, // RRA ($ind,X)
    [0x73] = {bus_ind_idx_y0, bus_ind_idx_y1, bus_ind_idx_y2, ror_a, cpu6502_CPU_INSTRUCTION_NO_WAIT, adc_bus, cpu6502_CPU_INSTRUCTION_END}, // RRA ($ind),Y

    [0x8F] = {bus_abs, sax_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, cpu6502_CPU_INSTRUCTION_END},                                        // SAX $abs
    [0x87] = {bus_zp, sax_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, cpu6502_CPU_INSTRUCTION_END},                                         // SAX $abs
    [0x97] = {bus_zp_y, sax_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, cpu6502_CPU_INSTRUCTION_END},                                       // SAX $abs
    [0x83] = {bus_idx_ind_x0, bus_idx_ind_x1, bus_idx_ind_x2, sax_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, cpu6502_CPU_INSTRUCTION_END}, // SAX $abs

    [0xCB] = {sbx_imm, cpu6502_CPU_INSTRUCTION_END}, // SBX #imm

    [0x9F] = {bus_abs_y, cpu6502_CPU_INSTRUCTION_NO_WAIT, sha_ind_y_abs, nop, cpu6502_CPU_INSTRUCTION_END}, // SHA $abs,Y
    [0x93] = {bus_zp_y, cpu6502_CPU_INSTRUCTION_NO_WAIT, sha_zp_y, nop, cpu6502_CPU_INSTRUCTION_END},       // SHA $zp,Y

    [0x9B] = {bus_abs_y, cpu6502_CPU_INSTRUCTION_NO_WAIT, shs_abs_y, nop, cpu6502_CPU_INSTRUCTION_END}, // SHS $abs,Y

    [0x9E] = {bus_abs_y, cpu6502_CPU_INSTRUCTION_NO_WAIT, shx_abs_y, nop, cpu6502_CPU_INSTRUCTION_END}, // SHX $abs,Y
    [0x9C] = {bus_abs_y, cpu6502_CPU_INSTRUCTION_NO_WAIT, shy_abs_x, nop, cpu6502_CPU_INSTRUCTION_END}, // SHY $abs,X

    [0x0F] = {bus_abs, asl_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, ora_bus, cpu6502_CPU_INSTRUCTION_END},                                        // SLO $abs
    [0x1F] = {bus_abs_x, asl_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, ora_bus, cpu6502_CPU_INSTRUCTION_END},                                      // SLO $abs,X
    [0x1B] = {bus_abs_y, asl_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, ora_bus, cpu6502_CPU_INSTRUCTION_END},                                      // SLO $abs,Y
    [0x07] = {bus_zp, asl_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, ora_bus, cpu6502_CPU_INSTRUCTION_END},                                         // SLO $zp
    [0x17] = {bus_zp_x, asl_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, ora_bus, cpu6502_CPU_INSTRUCTION_END},                                       // SLO $zp,X
    [0x03] = {bus_idx_ind_x0, bus_idx_ind_x1, bus_idx_ind_x2, asl_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, ora_bus, cpu6502_CPU_INSTRUCTION_END}, // SLO ($zp,X)
    [0x13] = {bus_ind_idx_y0, bus_ind_idx_y1, bus_ind_idx_y2, asl_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, ora_bus, cpu6502_CPU_INSTRUCTION_END}, // SLO $(zp),Y

    [0x4F] = {bus_abs, lsr_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, eor_bus, cpu6502_CPU_INSTRUCTION_END},                                        // SRE $abs
    [0x5F] = {bus_abs_x, lsr_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, eor_bus, cpu6502_CPU_INSTRUCTION_END},                                      // SRE $abs,X
    [0x5B] = {bus_abs_y, lsr_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, eor_bus, cpu6502_CPU_INSTRUCTION_END},                                      // SRE $abs,Y
    [0x47] = {bus_zp, lsr_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, eor_bus, cpu6502_CPU_INSTRUCTION_END},                                         // SRE $zp
    [0x57] = {bus_zp_x, lsr_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, eor_bus, cpu6502_CPU_INSTRUCTION_END},                                       // SRE $zp,X
    [0x43] = {bus_idx_ind_x0, bus_idx_ind_x1, bus_idx_ind_x2, lsr_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, eor_bus, cpu6502_CPU_INSTRUCTION_END}, // SRE ($zp,X)
    [0x53] = {bus_ind_idx_y0, bus_ind_idx_y1, bus_ind_idx_y2, lsr_bus, cpu6502_CPU_INSTRUCTION_NO_WAIT, eor_bus, cpu6502_CPU_INSTRUCTION_END}, // SRE $(zp),Y

    [0x8B] = {xaa_imm, cpu6502_CPU_INSTRUCTION_END}, // XAA #imm (Oh dear god)
};


// We can't have clang format messing array formatting up
// clang-format off
cpu6502_u8 cpu6502_CPU_instruction_operand_sizes[256] = {
    /*       -0  -1  -2  -3  -4  -5  -6  -7  -8  -9  -A  -B  -C  -D  -E  -F */
    /* 0- */  0,  1,  0,  1,  1,  1,  1,  1,  0,  1,  0,  1,  1,  2,  2,  2,
    /* 1- */  1,  1,  0,  1,  1,  1,  1,  1,  0,  2,  0,  2,  2,  2,  2,  2,
    /* 2- */  2,  1,  0,  1,  1,  1,  1,  1,  0,  1,  0,  1,  2,  2,  2,  2,
    /* 3- */  1,  1,  0,  1,  1,  1,  1,  1,  0,  2,  0,  2,  2,  2,  2,  2,
    /* 4- */  0,  1,  0,  1,  1,  1,  1,  1,  0,  1,  0,  1,  2,  2,  2,  2,
    /* 5- */  1,  1,  0,  1,  1,  1,  1,  1,  0,  2,  0,  2,  2,  2,  2,  2,
    /* 6- */  0,  1,  0,  1,  1,  1,  1,  1,  0,  1,  0,  1,  2,  2,  2,  2,
    /* 7- */  1,  1,  0,  1,  1,  1,  1,  1,  0,  2,  0,  2,  2,  2,  2,  2,
    /* 8- */  1,  1,  1,  1,  1,  1,  1,  1,  0,  1,  0,  1,  2,  2,  2,  2,
    /* 9- */  1,  1,  0,  1,  1,  1,  1,  1,  0,  2,  0,  2,  2,  2,  2,  2,
    /* A- */  1,  1,  1,  1,  1,  1,  1,  1,  0,  1,  0,  1,  2,  2,  2,  2,
    /* B- */  1,  1,  0,  1,  1,  1,  1,  1,  0,  2,  0,  2,  2,  2,  2,  2,
    /* C- */  1,  1,  1,  1,  1,  1,  1,  1,  0,  1,  0,  1,  2,  2,  2,  2,
    /* D- */  1,  1,  0,  1,  1,  1,  1,  1,  0,  2,  0,  2,  2,  2,  2,  2,
    /* E- */  1,  1,  1,  1,  1,  1,  1,  1,  0,  1,  0,  1,  2,  2,  2,  2,
    /* F- */  1,  1,  0,  1,  1,  1,  1,  1,  0,  2,  0,  2,  2,  2,  2,  2,
};
// clang-format on
