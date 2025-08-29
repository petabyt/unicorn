extern GHashTable *global_helper_table;
typedef struct TCGHelperInfo {
    void *func;
    const char *name;
    unsigned flags;
    unsigned sizemask;
    unsigned n_args;
} TCGHelperInfo;

static uint64_t tci_read_reg_ext(const tcg_target_ulong *regs, TCGReg index)
{
    return (uint64_t)tci_read_reg(regs, index) | ((uint64_t)tci_read_reg(regs, index + 1) << 32);
}


static int debug_info(TCGHelperInfo *info) {
	printf("name: %s\n", info->name);
	printf("flags: 0x%x\n", info->flags);
	printf("sizemask: 0x%x\n", info->sizemask);
	printf("n_args: %d\n", info->n_args);
	printf("t0: %lu\n", (uintptr_t)info->func);
}

static uint64_t do_op_call(tcg_target_ulong *regs, tcg_target_ulong t0) {
	TCGHelperInfo *info;

	info = g_hash_table_lookup(global_helper_table, (gpointer)t0);
	if (info == NULL) {
	    fprintf(stderr, "tci.c: g_hash_table_lookup\n");
	    abort();
	}

	// Manual ABI interventions (wasm32 requires very specific conventions for uint64_t)
#if TCG_TARGET_REG_BITS == 32
	if (info->flags & dh_callflag_void && info->sizemask == 0x10 && info->n_args == 4) {
	    ((void (*)(uint32_t, uint64_t, uint32_t, uint32_t))t0)(tci_read_reg(regs, TCG_REG_R0), tci_read_reg_ext(regs, TCG_REG_R1), tci_read_reg(regs, TCG_REG_R3), tci_read_reg(regs, TCG_REG_R4));
	    return 0;
	} else if (info->sizemask == 0x255 && info->n_args == 4) {
	    return ((uint64_t (*)(uint64_t, uint64_t, uint64_t, uint32_t))t0)(tci_read_reg_ext(regs, TCG_REG_R0), tci_read_reg_ext(regs, TCG_REG_R2), tci_read_reg_ext(regs, TCG_REG_R4), tci_read_reg(regs, TCG_REG_R7));
	} else if (info->sizemask == 4 && info->n_args == 3) {
	    return ((uint32_t (*)(uint64_t, uint32_t, uint32_t))t0)(tci_read_reg_ext(regs, TCG_REG_R0), tci_read_reg(regs, TCG_REG_R2), tci_read_reg(regs, TCG_REG_R3));
	}

	for (int i = 1; i < 15; i++) {
	    if (info->sizemask & (1 << (i * 2))) {
	        printf("passed u32 to u64 func\n");
	        abort();
	    }
	}
#endif

    if (info->flags & dh_callflag_void) {
        // return type void
        switch (info->n_args) {
        case 0: ((void (*)(void))t0)(); break;
        case 1: ((void (*)(tcg_target_ulong))t0)(tci_read_reg(regs, TCG_REG_R0)); break;
        case 2: ((void (*)(tcg_target_ulong, tcg_target_ulong))t0)(tci_read_reg(regs, TCG_REG_R0), tci_read_reg(regs, TCG_REG_R1)); break;
        case 3: ((void (*)(tcg_target_ulong, tcg_target_ulong, tcg_target_ulong))t0)(tci_read_reg(regs, TCG_REG_R0), tci_read_reg(regs, TCG_REG_R1), tci_read_reg(regs, TCG_REG_R2)); break;
        case 4: ((void (*)(tcg_target_ulong, tcg_target_ulong, tcg_target_ulong, tcg_target_ulong))t0)(tci_read_reg(regs, TCG_REG_R0), tci_read_reg(regs, TCG_REG_R1), tci_read_reg(regs, TCG_REG_R2), tci_read_reg(regs, TCG_REG_R3)); break;
        case 5: ((void (*)(tcg_target_ulong, tcg_target_ulong, tcg_target_ulong, tcg_target_ulong, tcg_target_ulong))t0)(tci_read_reg(regs, TCG_REG_R0), tci_read_reg(regs, TCG_REG_R1), tci_read_reg(regs, TCG_REG_R2), tci_read_reg(regs, TCG_REG_R3), tci_read_reg(regs, TCG_REG_R4)); break;
        default: abort();
        }
        return 0;
    } else if (info->sizemask & 1) {
        // 64 bit return type
        switch (info->n_args) {
        case 0: return ((uint64_t (*)(void))t0)(); break;
        case 1: return ((uint64_t (*)(tcg_target_ulong))t0)(tci_read_reg(regs, TCG_REG_R0)); break;
        case 2: return ((uint64_t (*)(tcg_target_ulong, tcg_target_ulong))t0)(tci_read_reg(regs, TCG_REG_R0), tci_read_reg(regs, TCG_REG_R1)); break;
        case 3: return ((uint64_t (*)(tcg_target_ulong, tcg_target_ulong, tcg_target_ulong))t0)(tci_read_reg(regs, TCG_REG_R0), tci_read_reg(regs, TCG_REG_R1), tci_read_reg(regs, TCG_REG_R2)); break;
        case 4: return ((uint64_t (*)(tcg_target_ulong, tcg_target_ulong, tcg_target_ulong, tcg_target_ulong))t0)(tci_read_reg(regs, TCG_REG_R0), tci_read_reg(regs, TCG_REG_R1), tci_read_reg(regs, TCG_REG_R2), tci_read_reg(regs, TCG_REG_R3)); break;
        case 5: return ((uint64_t (*)(tcg_target_ulong, tcg_target_ulong, tcg_target_ulong, tcg_target_ulong, tcg_target_ulong))t0)(tci_read_reg(regs, TCG_REG_R0), tci_read_reg(regs, TCG_REG_R1), tci_read_reg(regs, TCG_REG_R2), tci_read_reg(regs, TCG_REG_R3), tci_read_reg(regs, TCG_REG_R4)); break;
        default: abort();
        }
    } else {
        // 32 bit return type
        switch (info->n_args) {
        case 0: return ((uint32_t (*)(void))t0)(); break;
        case 1: return ((uint32_t (*)(tcg_target_ulong))t0)(tci_read_reg(regs, TCG_REG_R0)); break;
        case 2: return ((uint32_t (*)(tcg_target_ulong, tcg_target_ulong))t0)(tci_read_reg(regs, TCG_REG_R0), tci_read_reg(regs, TCG_REG_R1)); break;
        case 3: return ((uint32_t (*)(tcg_target_ulong, tcg_target_ulong, tcg_target_ulong))t0)(tci_read_reg(regs, TCG_REG_R0), tci_read_reg(regs, TCG_REG_R1), tci_read_reg(regs, TCG_REG_R2)); break;
        case 4: return ((uint32_t (*)(tcg_target_ulong, tcg_target_ulong, tcg_target_ulong, tcg_target_ulong))t0)(tci_read_reg(regs, TCG_REG_R0), tci_read_reg(regs, TCG_REG_R1), tci_read_reg(regs, TCG_REG_R2), tci_read_reg(regs, TCG_REG_R3)); break;
        case 5: return ((uint32_t (*)(tcg_target_ulong, tcg_target_ulong, tcg_target_ulong, tcg_target_ulong, tcg_target_ulong))t0)(tci_read_reg(regs, TCG_REG_R0), tci_read_reg(regs, TCG_REG_R1), tci_read_reg(regs, TCG_REG_R2), tci_read_reg(regs, TCG_REG_R3), tci_read_reg(regs, TCG_REG_R4)); break;
        default: abort();
        }
    }
}
