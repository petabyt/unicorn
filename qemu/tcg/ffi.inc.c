/*
 * Typed helper-call dispatch for the TCI interpreter.
 *
 * tcg_reg_alloc_call (tcg.c) looks up each helper's signature once at
 * translation time and streams it with the call as a 32-bit immediate:
 *
 *     meta = sizemask | (n_args << 16) | (void_return << 20)
 *
 * where sizemask follows helper-head.h dh_sizemask(): bit 0 = 64-bit
 * return, bit 2i = 64-bit i-th argument (i counted from 1).
 *
 * do_op_call decodes meta and builds an exactly-typed C call.  This is
 * required on wasm32 (call_indirect traps on signature mismatch), works
 * on native hosts, and costs no hash lookup and no per-helper adapter
 * code at execution time.
 *
 * Argument slots follow tcg_target_call_iarg_regs, which skips
 * TCG_REG_CALL_STACK (R4) on every host width.  On 32-bit hosts an
 * i64 argument occupies two consecutive slots (low, high); a pair may
 * therefore straddle R3/R5.
 *
 * Dispatch is a preprocessor binary tree: each arity N has a seed and
 * N-1 append stages; parameter types travel as individual macro
 * arguments (never comma-joined lists), argument expressions travel
 * via __VA_ARGS__.  Each leaf is a complete, exactly-typed call for
 * all three return kinds (void / 64-bit / 32-bit).
 */

extern GHashTable *global_helper_table;

typedef struct TCGHelperInfo {
    void *func;
    const char *name;
    unsigned flags;
    unsigned sizemask;
    unsigned n_args;
} TCGHelperInfo;

#define CALL_VOID_BIT  (1u << 20)

/* sizemask bit for the 64-bitness of argument i (i counted from 1). */
#define CALL_BIT64(m, i)  (((m) >> ((i) + 1) * 2) & 1)

/* Slot -> host register. */
#define SLOT_REG(s)  ((s) < 4 ? (s) : (s) + 1)

#define ARG_W(s)     ((uint32_t)tci_read_reg(regs, SLOT_REG(s)))

#if TCG_TARGET_REG_BITS == 32
#define CALL_STRIDE_64  2
#define CALL_STRIDE_W   1
#define ARG_64(s)  tci_uint64(tci_read_reg(regs, SLOT_REG((s) + 1)), \
                              tci_read_reg(regs, SLOT_REG(s)))
#else
#define CALL_STRIDE_64  1
#define CALL_STRIDE_W   1
#define ARG_64(s)  tci_read_reg(regs, SLOT_REG(s))
#endif

/* Close a call: types T0.. are fixed macro args, values via __VA_ARGS__. */
#define CALL_LEAF1(m, i, s, T0, ...)                                    \
    (((m) & CALL_VOID_BIT)                                              \
     ? (((void (*)(T0))t0)(__VA_ARGS__), 0)                             \
     : ((m) & 1                                                         \
        ? ((uint64_t (*)(T0))t0)(__VA_ARGS__)                           \
        : (uint64_t)((uint32_t (*)(T0))t0)(__VA_ARGS__)))

#define CALL_LEAF2(m, i, s, T0, T1, ...)                                \
    (((m) & CALL_VOID_BIT)                                              \
     ? (((void (*)(T0, T1))t0)(__VA_ARGS__), 0)                         \
     : ((m) & 1                                                         \
        ? ((uint64_t (*)(T0, T1))t0)(__VA_ARGS__)                       \
        : (uint64_t)((uint32_t (*)(T0, T1))t0)(__VA_ARGS__)))

#define CALL_LEAF3(m, i, s, T0, T1, T2, ...)                            \
    (((m) & CALL_VOID_BIT)                                              \
     ? (((void (*)(T0, T1, T2))t0)(__VA_ARGS__), 0)                     \
     : ((m) & 1                                                         \
        ? ((uint64_t (*)(T0, T1, T2))t0)(__VA_ARGS__)                   \
        : (uint64_t)((uint32_t (*)(T0, T1, T2))t0)(__VA_ARGS__)))

#define CALL_LEAF4(m, i, s, T0, T1, T2, T3, ...)                        \
    (((m) & CALL_VOID_BIT)                                              \
     ? (((void (*)(T0, T1, T2, T3))t0)(__VA_ARGS__), 0)                 \
     : ((m) & 1                                                         \
        ? ((uint64_t (*)(T0, T1, T2, T3))t0)(__VA_ARGS__)               \
        : (uint64_t)((uint32_t (*)(T0, T1, T2, T3))t0)(__VA_ARGS__)))

#define CALL_LEAF5(m, i, s, T0, T1, T2, T3, T4, ...)                    \
    (((m) & CALL_VOID_BIT)                                              \
     ? (((void (*)(T0, T1, T2, T3, T4))t0)(__VA_ARGS__), 0)             \
     : ((m) & 1                                                         \
        ? ((uint64_t (*)(T0, T1, T2, T3, T4))t0)(__VA_ARGS__)           \
        : (uint64_t)((uint32_t (*)(T0, T1, T2, T3, T4))t0)(__VA_ARGS__)))

#define CALL_LEAF6(m, i, s, T0, T1, T2, T3, T4, T5, ...)                \
    (((m) & CALL_VOID_BIT)                                              \
     ? (((void (*)(T0, T1, T2, T3, T4, T5))t0)(__VA_ARGS__), 0)         \
     : ((m) & 1                                                         \
        ? ((uint64_t (*)(T0, T1, T2, T3, T4, T5))t0)(__VA_ARGS__)       \
        : (uint64_t)((uint32_t (*)(T0, T1, T2, T3, T4, T5))t0)(__VA_ARGS__)))

#define CALL_LEAF7(m, i, s, T0, T1, T2, T3, T4, T5, T6, ...)            \
    (((m) & CALL_VOID_BIT)                                              \
     ? (((void (*)(T0, T1, T2, T3, T4, T5, T6))t0)(__VA_ARGS__), 0)     \
     : ((m) & 1                                                         \
        ? ((uint64_t (*)(T0, T1, T2, T3, T4, T5, T6))t0)(__VA_ARGS__)   \
        : (uint64_t)((uint32_t (*)(T0, T1, T2, T3, T4, T5, T6))t0)(__VA_ARGS__)))

/*
 * Append stages: Pk has already-fixed type parameters T0..T(k-1),
 * chooses the type of argument k, then invokes NEXT with one more
 * fixed type and one more value appended.
 */
#define CALL_P1(m, i, s, T0, NEXT, ...)                                 \
    (CALL_BIT64(m, i)                                                   \
     ? NEXT(m, (i) + 1, (s) + CALL_STRIDE_64,                           \
            T0, uint64_t, __VA_ARGS__, ARG_64(s))                       \
     : NEXT(m, (i) + 1, (s) + CALL_STRIDE_W,                            \
            T0, uint32_t, __VA_ARGS__, ARG_W(s)))

#define CALL_P2(m, i, s, T0, T1, NEXT, ...)                             \
    (CALL_BIT64(m, i)                                                   \
     ? NEXT(m, (i) + 1, (s) + CALL_STRIDE_64,                           \
            T0, T1, uint64_t, __VA_ARGS__, ARG_64(s))                   \
     : NEXT(m, (i) + 1, (s) + CALL_STRIDE_W,                            \
            T0, T1, uint32_t, __VA_ARGS__, ARG_W(s)))

#define CALL_P3(m, i, s, T0, T1, T2, NEXT, ...)                         \
    (CALL_BIT64(m, i)                                                   \
     ? NEXT(m, (i) + 1, (s) + CALL_STRIDE_64,                           \
            T0, T1, T2, uint64_t, __VA_ARGS__, ARG_64(s))               \
     : NEXT(m, (i) + 1, (s) + CALL_STRIDE_W,                            \
            T0, T1, T2, uint32_t, __VA_ARGS__, ARG_W(s)))

#define CALL_P4(m, i, s, T0, T1, T2, T3, NEXT, ...)                     \
    (CALL_BIT64(m, i)                                                   \
     ? NEXT(m, (i) + 1, (s) + CALL_STRIDE_64,                           \
            T0, T1, T2, T3, uint64_t, __VA_ARGS__, ARG_64(s))           \
     : NEXT(m, (i) + 1, (s) + CALL_STRIDE_W,                            \
            T0, T1, T2, T3, uint32_t, __VA_ARGS__, ARG_W(s)))

#define CALL_P5(m, i, s, T0, T1, T2, T3, T4, NEXT, ...)                 \
    (CALL_BIT64(m, i)                                                   \
     ? NEXT(m, (i) + 1, (s) + CALL_STRIDE_64,                           \
            T0, T1, T2, T3, T4, uint64_t, __VA_ARGS__, ARG_64(s))       \
     : NEXT(m, (i) + 1, (s) + CALL_STRIDE_W,                            \
            T0, T1, T2, T3, T4, uint32_t, __VA_ARGS__, ARG_W(s)))

#define CALL_P6(m, i, s, T0, T1, T2, T3, T4, T5, NEXT, ...)             \
    (CALL_BIT64(m, i)                                                   \
     ? NEXT(m, (i) + 1, (s) + CALL_STRIDE_64,                           \
            T0, T1, T2, T3, T4, T5, uint64_t, __VA_ARGS__, ARG_64(s))   \
     : NEXT(m, (i) + 1, (s) + CALL_STRIDE_W,                            \
            T0, T1, T2, T3, T4, T5, uint32_t, __VA_ARGS__, ARG_W(s)))

/* Seed: choose the type of argument 0, then invoke NEXT. */
#define CALL_SEED(m, s, NEXT)                                           \
    (CALL_BIT64(m, 0)                                                   \
     ? NEXT(m, 1, (s) + CALL_STRIDE_64, uint64_t, ARG_64(s))            \
     : NEXT(m, 1, (s) + CALL_STRIDE_W,  uint32_t, ARG_W(s)))

/* Arity wrappers: bind the next stage of the chain for arity N. */
#define CALL_A2(m, s)  CALL_SEED(m, s, CALL_P1_A2)
#define CALL_A3(m, s)  CALL_SEED(m, s, CALL_P1_A3)
#define CALL_A4(m, s)  CALL_SEED(m, s, CALL_P1_A4)
#define CALL_A5(m, s)  CALL_SEED(m, s, CALL_P1_A5)
#define CALL_A6(m, s)  CALL_SEED(m, s, CALL_P1_A6)
#define CALL_A7(m, s)  CALL_SEED(m, s, CALL_P1_A7)

#define CALL_P1_A2(m, i, s, T0, ...)  CALL_P1(m, i, s, T0, CALL_LEAF2, __VA_ARGS__)

#define CALL_P1_A3(m, i, s, T0, ...)  CALL_P1(m, i, s, T0, CALL_P2_A3, __VA_ARGS__)
#define CALL_P2_A3(m, i, s, T0, T1, ...)  CALL_P2(m, i, s, T0, T1, CALL_LEAF3, __VA_ARGS__)

#define CALL_P1_A4(m, i, s, T0, ...)  CALL_P1(m, i, s, T0, CALL_P2_A4, __VA_ARGS__)
#define CALL_P2_A4(m, i, s, T0, T1, ...)  CALL_P2(m, i, s, T0, T1, CALL_P3_A4, __VA_ARGS__)
#define CALL_P3_A4(m, i, s, T0, T1, T2, ...)  CALL_P3(m, i, s, T0, T1, T2, CALL_LEAF4, __VA_ARGS__)

#define CALL_P1_A5(m, i, s, T0, ...)  CALL_P1(m, i, s, T0, CALL_P2_A5, __VA_ARGS__)
#define CALL_P2_A5(m, i, s, T0, T1, ...)  CALL_P2(m, i, s, T0, T1, CALL_P3_A5, __VA_ARGS__)
#define CALL_P3_A5(m, i, s, T0, T1, T2, ...)  CALL_P3(m, i, s, T0, T1, T2, CALL_P4_A5, __VA_ARGS__)
#define CALL_P4_A5(m, i, s, T0, T1, T2, T3, ...)  CALL_P4(m, i, s, T0, T1, T2, T3, CALL_LEAF5, __VA_ARGS__)

#define CALL_P1_A6(m, i, s, T0, ...)  CALL_P1(m, i, s, T0, CALL_P2_A6, __VA_ARGS__)
#define CALL_P2_A6(m, i, s, T0, T1, ...)  CALL_P2(m, i, s, T0, T1, CALL_P3_A6, __VA_ARGS__)
#define CALL_P3_A6(m, i, s, T0, T1, T2, ...)  CALL_P3(m, i, s, T0, T1, T2, CALL_P4_A6, __VA_ARGS__)
#define CALL_P4_A6(m, i, s, T0, T1, T2, T3, ...)  CALL_P4(m, i, s, T0, T1, T2, T3, CALL_P5_A6, __VA_ARGS__)
#define CALL_P5_A6(m, i, s, T0, T1, T2, T3, T4, ...)  CALL_P5(m, i, s, T0, T1, T2, T3, T4, CALL_LEAF6, __VA_ARGS__)

#define CALL_P1_A7(m, i, s, T0, ...)  CALL_P1(m, i, s, T0, CALL_P2_A7, __VA_ARGS__)
#define CALL_P2_A7(m, i, s, T0, T1, ...)  CALL_P2(m, i, s, T0, T1, CALL_P3_A7, __VA_ARGS__)
#define CALL_P3_A7(m, i, s, T0, T1, T2, ...)  CALL_P3(m, i, s, T0, T1, T2, CALL_P4_A7, __VA_ARGS__)
#define CALL_P4_A7(m, i, s, T0, T1, T2, T3, ...)  CALL_P4(m, i, s, T0, T1, T2, T3, CALL_P5_A7, __VA_ARGS__)
#define CALL_P5_A7(m, i, s, T0, T1, T2, T3, T4, ...)  CALL_P5(m, i, s, T0, T1, T2, T3, T4, CALL_P6_A7, __VA_ARGS__)
#define CALL_P6_A7(m, i, s, T0, T1, T2, T3, T4, T5, ...)  CALL_P6(m, i, s, T0, T1, T2, T3, T4, T5, CALL_LEAF7, __VA_ARGS__)

/* Cold path: diagnostics only, never reached in valid bytecode. */
static void helper_call_fail(uintptr_t t0, uint32_t meta)
{
    TCGHelperInfo *info = g_hash_table_lookup(global_helper_table,
                                              (gpointer)t0);

    fprintf(stderr,
            "tci: unsupported helper call %p meta=0x%x "
            "(name=%s flags=0x%x sizemask=0x%x n_args=%d)\n",
            (void *)t0, meta,
            info ? info->name : "?", info ? info->flags : 0,
            info ? info->sizemask : 0, info ? (int)info->n_args : -1);
    abort();
}

static uint64_t do_op_call(tcg_target_ulong *regs, tcg_target_ulong t0,
                           uint32_t meta)
{
    unsigned n_args = (meta >> 16) & 0xF;

    switch (n_args) {
    case 0:
        if (meta & CALL_VOID_BIT) {
            ((void (*)(void))t0)();
            return 0;
        }
        if (meta & 1) {
            return ((uint64_t (*)(void))t0)();
        }
        return ((uint32_t (*)(void))t0)();
    case 1:
        return CALL_SEED(meta, 0, CALL_LEAF1);
    case 2:
        return CALL_A2(meta, 0);
    case 3:
        return CALL_A3(meta, 0);
    case 4:
        return CALL_A4(meta, 0);
    case 5:
        return CALL_A5(meta, 0);
    case 6:
        return CALL_A6(meta, 0);
    case 7:
        return CALL_A7(meta, 0);
    default:
        helper_call_fail((uintptr_t)t0, meta);
        return 0;
    }
}
