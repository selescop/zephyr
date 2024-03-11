/*
 * Copyright (c) 2023 Intel Corporation
 * Copyright (c) 2024 Schneider Electric
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/llext/elf.h>
#include <zephyr/llext/llext.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(elf, CONFIG_LLEXT_LOG_LEVEL);


#define ___opcode_identity32(x) ((uint32_t)(x))
#define ___opcode_identity16(x) ((uint16_t)(x))
#define __opcode_to_mem_arm(x) ___opcode_identity32(x)
#define __opcode_to_mem_thumb16(x) ___opcode_identity16(x)
#define __mem_to_opcode_arm(x) __opcode_to_mem_arm(x)
#define __mem_to_opcode_thumb16(x) __opcode_to_mem_thumb16(x)

static int32_t sign_extend32(uint32_t value, int index)
{
	int8_t shift = 31 - index;
	return (int32_t)(value << shift) >> shift;
}

// loc = dstsec->sh_addr + rel->r_offset;

// loc : addr of opcode
// sym_base_addr : addr of symbol if undef else addr of symbol section + sym value
// symname: symbol name

static int
apply_relocate(uint32_t rel_index, elf_word reloc_type, uint32_t loc, uint32_t sym_base_addr, const char *symname, uintptr_t load_bias)
{

	{
		uint32_t tmp;
		uint32_t upper, lower, sign, j1, j2;
		int32_t offset;

		LOG_DBG("apply_relocate:%u %d %x %x %s", rel_index, reloc_type, loc, sym_base_addr, symname);
	

		switch (reloc_type) {
			case R_ARM_NONE:
				/* ignore */
				break;

			case R_ARM_ABS32:
			case R_ARM_TARGET1:
				*(uint32_t *)loc += sym_base_addr;
				break;

			case R_ARM_PC24:
			case R_ARM_CALL:
			case R_ARM_JUMP24:
				offset = __mem_to_opcode_arm(*(uint32_t *)loc);
				offset = (offset & 0x00ffffff) << 2;
				offset = sign_extend32(offset, 25);

				offset += sym_base_addr - loc;


				if (offset <= (int32_t)0xfe000000 ||
					offset >= (int32_t)0x02000000) {
					LOG_ERR("sym '%s': relocation %u out of range (%#x -> %#x)\n",
						symname,
						rel_index,
						loc,
						sym_base_addr);
					return -ENOEXEC;
				}

				offset >>= 2;
				offset &= 0x00ffffff;

				*(uint32_t *)loc &= __opcode_to_mem_arm(0xff000000);
				*(uint32_t *)loc |= __opcode_to_mem_arm(offset);
				break;

			case R_ARM_V4BX:
#ifdef CONFIG_LLEXT_ARM_V4BX
				/* Preserve Rm and the condition code. Alter
				* other bits to re-code instruction as
				* MOV PC,Rm.
				*/
				*(uint32_t *)loc &= __opcode_to_mem_arm(0xf000000f);
				*(uint32_t *)loc |= __opcode_to_mem_arm(0x01a0f000);
#endif
				break;

			case R_ARM_PREL31:
				offset = (*(int32_t *)loc << 1) >> 1; /* sign extend */
				offset += sym_base_addr - loc;
				if (offset >= 0x40000000 || offset < -0x40000000) {
					LOG_ERR("sym '%s': relocation %u out of range (%#x -> %#x)\n",
						symname,
						rel_index,
						loc,
						sym_base_addr);
					return -ENOEXEC;
				}
				*(uint32_t *)loc &= 0x80000000;
				*(uint32_t *)loc |= offset & 0x7fffffff;
				break;

			case R_ARM_REL32:
				*(uint32_t *)loc += sym_base_addr - loc;
				break;

			case R_ARM_MOVW_ABS_NC:
			case R_ARM_MOVT_ABS:
			case R_ARM_MOVW_PREL_NC:
			case R_ARM_MOVT_PREL:
				offset = tmp = __mem_to_opcode_arm(*(uint32_t *)loc);
				offset = ((offset & 0xf0000) >> 4) | (offset & 0xfff);
				offset = sign_extend32(offset, 15);

				offset += sym_base_addr;
				if (reloc_type == R_ARM_MOVT_PREL ||
					reloc_type == R_ARM_MOVW_PREL_NC)
					offset -= loc;
				if (reloc_type == R_ARM_MOVT_ABS ||
					reloc_type == R_ARM_MOVT_PREL)
					offset >>= 16;

				tmp &= 0xfff0f000;
				tmp |= ((offset & 0xf000) << 4) |
					(offset & 0x0fff);

				*(uint32_t *)loc = __opcode_to_mem_arm(tmp);
				break;

			case R_ARM_THM_CALL:
			case R_ARM_THM_JUMP24:
				/*
				* For function symbols, only Thumb addresses are
				* allowed (no interworking).
				*
				* For non-function symbols, the destination
				* has no specific ARM/Thumb disposition, so
				* the branch is resolved under the assumption
				* that interworking is not required.
				*/

				upper = __mem_to_opcode_thumb16(*(uint16_t *)loc);
				lower = __mem_to_opcode_thumb16(*(uint16_t *)(loc + 2));

				/*
				* 25 bit signed address range (Thumb-2 BL and B.W
				* instructions):
				*   S:I1:I2:imm10:imm11:0
				* where:
				*   S     = upper[10]   = offset[24]
				*   I1    = ~(J1 ^ S)   = offset[23]
				*   I2    = ~(J2 ^ S)   = offset[22]
				*   imm10 = upper[9:0]  = offset[21:12]
				*   imm11 = lower[10:0] = offset[11:1]
				*   J1    = lower[13]
				*   J2    = lower[11]
				*/
				sign = (upper >> 10) & 1;
				j1 = (lower >> 13) & 1;
				j2 = (lower >> 11) & 1;
				offset = (sign << 24) | ((~(j1 ^ sign) & 1) << 23) |
					((~(j2 ^ sign) & 1) << 22) |
					((upper & 0x03ff) << 12) |
					((lower & 0x07ff) << 1);
				offset = sign_extend32(offset, 24);
				offset += sym_base_addr - loc;


				if (offset <= (int32_t)0xff000000 ||
					offset >= (int32_t)0x01000000) {
					LOG_ERR("sym '%s': relocation %u out of range (%#x -> %#x)\n",
						symname,
						rel_index,
						loc,
						sym_base_addr);
					return -ENOEXEC;
				}

				sign = (offset >> 24) & 1;
				j1 = sign ^ (~(offset >> 23) & 1);
				j2 = sign ^ (~(offset >> 22) & 1);
				upper = (uint16_t)((upper & 0xf800) | (sign << 10) |
							((offset >> 12) & 0x03ff));
				lower = (uint16_t)((lower & 0xd000) |
						(j1 << 13) | (j2 << 11) |
						((offset >> 1) & 0x07ff));

				*(uint16_t *)loc = __opcode_to_mem_thumb16(upper);
				*(uint16_t *)(loc + 2) = __opcode_to_mem_thumb16(lower);
				break;

			case R_ARM_THM_MOVW_ABS_NC:
			case R_ARM_THM_MOVT_ABS:
			case R_ARM_THM_MOVW_PREL_NC:
			case R_ARM_THM_MOVT_PREL:
				upper = __mem_to_opcode_thumb16(*(uint16_t *)loc);
				lower = __mem_to_opcode_thumb16(*(uint16_t *)(loc + 2));

				/*
				* MOVT/MOVW instructions encoding in Thumb-2:
				*
				* i	= upper[10]
				* imm4	= upper[3:0]
				* imm3	= lower[14:12]
				* imm8	= lower[7:0]
				*
				* imm16 = imm4:i:imm3:imm8
				*/
				offset = ((upper & 0x000f) << 12) |
					((upper & 0x0400) << 1) |
					((lower & 0x7000) >> 4) | (lower & 0x00ff);
				offset = sign_extend32(offset, 15);
				offset += sym_base_addr;

				if (reloc_type == R_ARM_THM_MOVT_PREL ||
					reloc_type == R_ARM_THM_MOVW_PREL_NC)
					offset -= loc;
				if (reloc_type == R_ARM_THM_MOVT_ABS ||
					reloc_type == R_ARM_THM_MOVT_PREL)
					offset >>= 16;

				upper = (uint16_t)((upper & 0xfbf0) |
						((offset & 0xf000) >> 12) |
						((offset & 0x0800) >> 1));
				lower = (uint16_t)((lower & 0x8f00) |
						((offset & 0x0700) << 4) |
						(offset & 0x00ff));
				*(uint16_t *)loc = __opcode_to_mem_thumb16(upper);
				*(uint16_t *)(loc + 2) = __opcode_to_mem_thumb16(lower);
				break;
			
			case R_ARM_RELATIVE:
				*(uint32_t *)loc += load_bias;
				break;

			case R_ARM_GLOB_DAT:
				*(uint32_t *)loc = sym_base_addr;
				break;

			case R_ARM_JUMP_SLOT:
				*(uint32_t *)loc = sym_base_addr;
				break;

			default:
				LOG_ERR("unknown relocation: %u\n",
					reloc_type);
				return -ENOEXEC;
		}
	}
	return 0;
}


/**
 * @brief Architecture specific function for relocating partially linked (static) elf
 *
 * Elf files contain a series of relocations described in a section. These relocation
 * instructions are architecture specific and each architecture supporting extensions
 * must implement this.
 *
 * The relocation codes for arm are well documented
 * https://github.com/ARM-software/abi-aa/blob/main/aaelf32/aaelf32.rst#relocation
 */
int32_t arch_elf_relocate(elf_rela_t *rel, uint32_t rel_index, uintptr_t loc, uintptr_t sym_base_addr, const char *symname, uintptr_t load_bias)
{
	elf_word reloc_type = ELF32_R_TYPE(rel->r_info);

	return apply_relocate(rel_index, reloc_type, (uint32_t)loc, (uint32_t)sym_base_addr, symname, load_bias);
}
