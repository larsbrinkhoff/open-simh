/* bcc500_cpu.c: BCC 500 microprocessor simulator

   Copyright (c) 2024, Lars Brinkhoff

   Permission is hereby granted, free of charge, to any person obtaining a
   copy of this software and associated documentation files (the "Software"),
   to deal in the Software without restriction, including without limitation
   the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
   LARS BRINKHOFF BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
   IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

   Except as contained in this notice, the name of Lars Brinkhoff shall not be
   used in advertising or otherwise to promote the sale, use or other dealings
   in this Software without prior written authorization from Lars Brinkhoff.
*/

#include "bcc500_defs.h"


/* Microcode fields. */
//  0- 5 MC       Branch condition.
//                00 - Never branch
//                01 - Always branch
//                02 - X = 0
//                03 - X # 0
//                04 - X < 0
//                05 - X >= 0
//                06 - X > 0
//                07 - Y >= 0
//                10 - Y < 0
//                11 - R0 < 0
//                12 - R0 >= 0
//                13 - X <= 0
//                14 - Not X AND 777777 = 0
//                15 - Not X AND 777777 # 0
//                16 - Z >= 0
//                17 - Z < 0
//                20 - Always branch
//                21 - Y AND 7 # 0
//                22 - BL = 0
//                23 - BL # 0
//                24 - Y even
//                25 - Y odd
//                26 - Attention latch 1 not set, reset
//                27 - Both request strobe latches = 0
//                30 - Protect # X
//                31 - Request strobe latch 2 = 0
//                32 - Special flag A not set
//                33 - Special flag A set
//                34 - Attention latch 2 not set, reset
//                35 - Attention latch 3 not set, reset
//                36 - Attention latch 1 set, reset
//                42 - Local memory parity error = 0, reset
//                43 - M940
//                44 - Central memory parity error = 0, reset
//                45 - Breakpoint = 1
//  6- 7 MCONT    Sequence control.
//                0 - Jump
//                1 - Call
//                2 - Return
//                3 - Indirect jump
//  8-17 B        Branch address.
// 18-41 C        Constant.
// 42    IHR      Increment holding register.
// 43    TCX      Transfer constant to X bus.
// 44    TCY      Transfer constant to Y bus.
// 45    TSPY     Transfer scratch pad to Y bus.
// 46    THY      Transfer holding register to Y bus.
// 47    TXW      Transfer X bus to holding register.
// 48    TYW      Transfer Y bus to holding register.
// 49    TAX      Transfer adder to X bus.
// 50    LOC      Adder low order carry.
// 51-56 SSP      Select scratch pad address.
// 57    TOSY     Transfer OS register to Y bus.
// 58    LR0      Load holding R0 from X or Y bus.
// 59    LSPX     Load scratch pad from X bus.
// 60-65 MS       Special condition.
//                00 - No action
//                01 - LCY 1
//                02 - LCY 2
//                03 - LCY 3
//                04 - LCY 4
//                05 - LCY 8
//                06 - LCY 12
//                07 - LCY 16
//                10 - LCY 20
//                11 - LCL Z
//                12 - LCH Z
//                13 - SKZ
//                14 - ALERT
//                15 - POT
//                16 - PIN
//                17 - Request strobe 1
//                20 - Unprotect
//                22 - Load memory request priority
//                23 - Reset request strobe latch 1
//                24 - Reset central memory request
//                25 - Set protect mask from X bus
//                26 - Reset device attached to I/O connector
//                30 - Set special flag A
//                31 - Reset special flag A
//                32 - Reset request strobe latch 2
//                33 - Request strobe 2
//                40 - Release
//                41 - Prestore
//                42 - Store
//                43 - Store and hold
//                44 - Fetch
//                45 - Fetch and hold
//                47 - Prefetch
//                50 - "TO"
//                60 - Set bank B
//                61 - Set bank A
//                62 - Clear map
//                64 - Oddword fetch
//                65 - Oddword fetch and hold
//                72 - "CLM940"
// 66-68 RRN      Read holding register into incrementer.
// 69-71 LRN      Load holding register from X or Y bus.
// 72    LMX      Load M from X bus.
// 73    LMY      Load M from Y bus.
// 74    LQX      Load Q from X bus.
// 75    LQY      Load Q from Y bus.
// 76    LZX      Load Z from X bus.
// 77    LZY      Load Z from Y bus.
// 78-81 BL       Left boolean box.
//                00 - M AND Q
//                01 - M EQV Q
//                02 - Q
//                03 - NOT M OR Q
//                04 - M
//                05 - M OR NOT Q
//                06 - M OR Q
//                07 - -1
//                10 - 0
//                11 - NOT M AND NOT Q
//                12 - NOT M AND Q
//                13 - NOT M
//                14 - M AND NOT Q
//                15 - NOT Q
//                16 - M EOR Q
//                17 - NOT M OR NOTQ
// 82-85 BR       Right boolean box.
//                     Z instead of M.
// 86    VCY      Force 200 nanosecond cycle.
// 87    DGO      Deferred jump.
// 88    TE1Y     Transfer E1 bus to Y bus.
// 89    TE2Y     Transfer E2 bus to Y bus.

#define U_MC     ((I[0] & 07700000000) >> 24)
#define U_MCONT  ((I[0] & 00060000000) >> 22)
#define U_B      ((I[0] & 00017770000) >> 12)
#define U_C     (((I[0] & 00000007777) << 12) | \
                 ((I[1] & 07777000000) >> 18))
#define U_IHR     (I[1] & 00000400000)
#define U_TCX     (I[1] & 00000200000)
#define U_TCY     (I[1] & 00000100000)
#define U_TSPY    (I[1] & 00000040000)
#define U_THY     (I[1] & 00000020000)
#define U_TXW     (I[1] & 00000010000)
#define U_TYW     (I[1] & 00000004000)
#define U_TAX     (I[1] & 00000002000)
#define U_LOC     (I[1] & 00000001000)
#define U_SSP    ((I[1] & 00000000770) >>  3)
#define U_TOSY    (I[1] & 00000000004)
#define U_LR0     (I[1] & 00000000002)
#define U_LSPX    (I[1] & 00000000001)
#define U_MS     ((I[2] & 07700000000) >> 24)
#define U_RRN    ((I[2] & 00070000000) >> 21)
#define U_LRN    ((I[2] & 00007000000) >> 18)
#define U_LMX     (I[2] & 00000400000)
#define U_LMY     (I[2] & 00000200000)
#define U_LQX     (I[2] & 00000100000)
#define U_LQY     (I[2] & 00000040000)
#define U_LZX     (I[2] & 00000020000)
#define U_LZY     (I[2] & 00000010000)
#define U_BL     ((I[2] & 00000007400) >> 8)
#define U_BR     ((I[2] & 00000000360) >> 4)
#define U_VCY     (I[2] & 00000000010)
#define U_DGO     (I[2] & 00000000004)
#define U_TE1Y    (I[2] & 00000000002)
#define U_TE2Y    (I[2] & 00000000001)

/* Debug */
#define DBG_TRACE         0001
#define DBG_STATE         0002

/* Microprocessor state. */
typedef struct {
  int XXC_XXB;
  microword_t I;
  uint16 O, OS;
  uint32 M, Q, Z;
  uint32 R[7];
  uint32 SP[64];
  microword_t ROM[2048];
} u_state;

static u_state cpu0_state, cpu1_state, msch_state, chio_state, amc_state, amtu_state;

/* Function declaration. */
static t_stat cpu_ex(t_value *vptr, t_addr ea, UNIT *uptr, int32 sw);
static t_stat cpu_dep(t_value val, t_addr ea, UNIT *uptr, int32 sw);
static t_stat cpu_reset(DEVICE *dptr);

static UNIT cpu_unit = { UDATA(NULL, UNIT_FIX + UNIT_BINK, 020000) };

static REG cpu0_reg[] = {
  { ORDATAD(O,  cpu0_state.O,     11,     "Microcode address") },
  { ORDATAD(OS, cpu0_state.OS,    11,     "Saved microcode address") },
  { ORDATAD(M,  cpu0_state.M,     24,     "M register") },
  { ORDATAD(Q,  cpu0_state.Q,     24,     "Q register") },
  { ORDATAD(Z,  cpu0_state.Z,     24,     "Z register") },
  { BRDATAD(R,  cpu0_state.R,  8, 24,  7, "Holding register") },
  { BRDATAD(SP, cpu0_state.SP, 8, 24, 64, "Scratchpad") },
  { NULL }
};

static REG cpu1_reg[] = {
  { ORDATAD(O,  cpu1_state.O,     11,     "Microcode address") },
  { ORDATAD(OS, cpu1_state.OS,    11,     "Microcode saved address") },
  { ORDATAD(M,  cpu1_state.M,     24,     "M register") },
  { ORDATAD(Q,  cpu1_state.Q,     24,     "Q register") },
  { ORDATAD(Z,  cpu1_state.Z,     24,     "Z register") },
  { BRDATAD(R,  cpu1_state.R,  8, 24,  7, "Holding register") },
  { BRDATAD(SP, cpu1_state.SP, 8, 24, 64, "Scratchpad") },
  { NULL }
};

static REG msch_reg[] = {
  { ORDATAD(O,  msch_state.O,     11,     "Microcode address") },
  { ORDATAD(OS, msch_state.OS,    11,     "Microcode saved address") },
  { ORDATAD(M,  msch_state.M,     24,     "M register") },
  { ORDATAD(Q,  msch_state.Q,     24,     "Q register") },
  { ORDATAD(Z,  msch_state.Z,     24,     "Z register") },
  { BRDATAD(R,  msch_state.R,  8, 24,  7, "Holding register") },
  { BRDATAD(SP, msch_state.SP, 8, 24, 64, "Scratchpad") },
  { NULL }
};

static REG chio_reg[] = {
  { ORDATAD(O,  chio_state.O,     11,     "Microcode address") },
  { ORDATAD(OS, chio_state.OS,    11,     "Microcode saved address") },
  { ORDATAD(M,  chio_state.M,     24,     "M register") },
  { ORDATAD(Q,  chio_state.Q,     24,     "Q register") },
  { ORDATAD(Z,  chio_state.Z,     24,     "Z register") },
  { BRDATAD(R,  chio_state.R,  8, 24,  7, "Holding register") },
  { BRDATAD(SP, chio_state.SP, 8, 24, 64, "Scratchpad") },
  { NULL }
};

static REG amc_reg[] = {
  { ORDATAD(O,  amc_state.O,     11,     "Microcode address") },
  { ORDATAD(OS, amc_state.OS,    11,     "Microcode saved address") },
  { ORDATAD(M,  amc_state.M,     24,     "M register") },
  { ORDATAD(Q,  amc_state.Q,     24,     "Q register") },
  { ORDATAD(Z,  amc_state.Z,     24,     "Z register") },
  { BRDATAD(R,  amc_state.R,  8, 24,  7, "Holding register") },
  { BRDATAD(SP, amc_state.SP, 8, 24, 64, "Scratchpad") },
  { NULL }
};

static REG amtu_reg[] = {
  { ORDATAD(O,  amtu_state.O,     11,     "Microcode address") },
  { ORDATAD(OS, amtu_state.OS,    11,     "Microcode saved address") },
  { ORDATAD(M,  amtu_state.M,     24,     "M register") },
  { ORDATAD(Q,  amtu_state.Q,     24,     "Q register") },
  { ORDATAD(Z,  amtu_state.Z,     24,     "Z register") },
  { BRDATAD(R,  amtu_state.R,  8, 24,  7, "Holding register") },
  { BRDATAD(SP, amtu_state.SP, 8, 24, 64, "Scratchpad") },
  { NULL }
};

REG *sim_PC = &cpu0_reg[0];

static MTAB cpu_mod[] = {
  { 0 }
};

static DEBTAB cpu_deb[] = {
  { "TRACE", DBG_TRACE },
  { "STATE", DBG_STATE },
  { NULL, 0 }
};

DEVICE cpu0_dev = {
  "CPU0", &cpu_unit, cpu0_reg, cpu_mod,
  0, 8, 16, 1, 8, 16,
  &cpu_ex, &cpu_dep, &cpu_reset,
  NULL, NULL, NULL, NULL, DEV_DEBUG, 0, cpu_deb,
  NULL, NULL, NULL, NULL, NULL, NULL
};

DEVICE cpu1_dev = {
  "CPU1", &cpu_unit, cpu1_reg, cpu_mod,
  0, 8, 16, 1, 8, 16,
  &cpu_ex, &cpu_dep, &cpu_reset,
  NULL, NULL, NULL, NULL, DEV_DEBUG, 0, cpu_deb,
  NULL, NULL, NULL, NULL, NULL, NULL
};

DEVICE msch_dev = {
  "MSCH", &cpu_unit, msch_reg, cpu_mod,
  0, 8, 16, 1, 8, 16,
  &cpu_ex, &cpu_dep, &cpu_reset,
  NULL, NULL, NULL, NULL, DEV_DEBUG, 0, cpu_deb,
  NULL, NULL, NULL, NULL, NULL, NULL
};

DEVICE chio_dev = {
  "CHIO", &cpu_unit, chio_reg, cpu_mod,
  0, 8, 16, 1, 8, 16,
  &cpu_ex, &cpu_dep, &cpu_reset,
  NULL, NULL, NULL, NULL, DEV_DEBUG, 0, cpu_deb,
  NULL, NULL, NULL, NULL, NULL, NULL
};

DEVICE amc_dev = {
  "AMC", &cpu_unit, amc_reg, cpu_mod,
  0, 8, 16, 1, 8, 16,
  &cpu_ex, &cpu_dep, &cpu_reset,
  NULL, NULL, NULL, NULL, DEV_DEBUG, 0, cpu_deb,
  NULL, NULL, NULL, NULL, NULL, NULL
};

DEVICE amtu_dev = {
  "AMTU", &cpu_unit, amtu_reg, cpu_mod,
  0, 8, 16, 1, 8, 16,
  &cpu_ex, &cpu_dep, &cpu_reset,
  NULL, NULL, NULL, NULL, DEV_DEBUG, 0, cpu_deb,
  NULL, NULL, NULL, NULL, NULL, NULL
};

static int branch(int code, uint32 X, uint32 Y, uint32 BL, u_state *state)
{
  switch (code) {
  case 000: return 0;
  case 001: return 1;
  case 002: return X == 0;
  case 003: return X != 0;
  case 004: return X & 040000000;
  case 005: return !(X & 040000000);
  case 006: return X != 0 && !(X & 040000000);
  case 007: return !(Y & 040000000);
  case 010: return Y & 040000000;
  case 011: return state->R[0] & 040000000;
  case 012: return !(state->R[0] & 040000000);
  case 013: return X & 040000000 || X == 0;
  case 014: return (X & 000777777) == 000777777;
  case 015: return (X & 000777777) != 000777777;
  case 016: return !(state->Z & 040000000);
  case 017: return state->Z & 040000000;
  case 020: return 1;
  case 021: return (Y & 7) != 0;
  case 022: return BL == 0;
  case 023: return BL != 0;
  case 024: return !(Y & 1);
  case 025: return Y & 1;
  case 026:
    fprintf(stderr, "Branch if attention latch 1 set, reset\n");
    return 0;
  case 034:
    fprintf(stderr, "Branch if attention latch 2 set, reset\n");
    return 0;
  case 045:
    fprintf(stderr, "Branch if breakpoint it set\n");
    return 0;
  default:
    fprintf(stderr, "Unimplemented branch %02o\n", code);
    exit(1);
  }
}

static void special(int code, uint32 BL, uint32 *X, u_state *state)
{
  switch (code) {
  case 000:
    break;
  case 001:
    *X |= (BL << 1) & 077777777;
    break;
  case 002:
    *X |= (BL << 2) & 077777777;
    break;
  case 003:
    *X |= (BL << 3) & 077777777;
    break;
  case 004:
    *X |= (BL << 4) & 077777777;
    break;
  case 005:
    *X |= (BL << 8) & 077777777;
    break;
  case 006:
    *X |= (BL << 12) & 077777777;
    break;
  case 007:
    *X |= (BL << 16) & 077777777;
    break;
  case 010:
    *X |= (BL << 20) & 077777777;
    break;
  case 011:
    *X |= (BL << (state->Z & 3)) & 077777777;
    break;
  case 012:
    *X |= (BL << ((state->Z >> 2) & 7)) & 077777777;
    break;
  case 014:
    fprintf(stderr, "Alert\n");
    break;
  case 013:
    // SKZ, select scratch pad address from Z.
    fprintf(stderr, "SKZ\n");
    break;
  case 016:
    fprintf(stderr, "Peripheral input\n");
    break;
  case 020:
    fprintf(stderr, "Unprotect\n");
    break;
  case 023:
    fprintf(stderr, "Reset strobe 1\n");
    break;
  case 032:
    fprintf(stderr, "Reset strobe 2\n");
    break;
  case 033:
    fprintf(stderr, "Set strobe 2\n");
    break;
  case 060:
    state->O &= 01777;  // Select ROM bank A.
    break;
  case 061:
    state->O |= 02000;  // Select ROM bank B.
    break;
  default:
    fprintf(stderr, "Unimplemented special function %02o\n", code);
    exit(1);
  }
}

static uint32 boolean(int code, uint32 A, uint32 B)
{
  uint32 notA = ~A & 077777777;
  uint32 notB = ~B & 077777777;
  switch (code) {
  case 000: return A & B;
  case 001: return ~(A ^ B) & 077777777;
  case 002: return B;
  case 003: return notA | B;
  case 004: return A;
  case 005: return A | notB;
  case 006: return A | B;
  case 007: return 077777777;
  case 010: return 0;
  case 011: return notA & notB;
  case 012: return notA & B;
  case 013: return notB;
  case 014: return A & notB;
  case 015: return notB;
  case 016: return A & B;
  case 017: return notA | notB;
  default:
    fprintf(stderr, "Unimplemented boolean %02o\n", code);
    exit(1);
  }
}

static void oload(uint16 b, u_state *state)
{
  state->O = (state->O & 02000) | (b & 01777);
}

static void jump(int code, uint16 B, uint32 X, u_state *state)
{
  switch (code) {
  case 0:
    oload(B, state);
    break;
  case 1:
    state->OS = state->O;
    oload(B, state);
    break;
  case 2:
    state->O = state->OS;
    break;
  case 3:
    state->O = X;
    break;
  }
}

static void cpu_cycle(DEVICE *dev, u_state *state)
{
  int BRANCH, RCE;
  uint32 Y, X, BL;
  microword_t I;
  memcpy(I, state->I, sizeof I);

  X = Y = 0;
  if (U_TCX)
    X |= U_C;
  if (U_TCY)
    Y |= U_C;
  if (U_TSPY)
    Y |= state->SP[U_SSP | (U_MS == 013 ? state->Z & 077 : 0)];
  if (U_THY)
    Y |= state->R[U_RRN] + (U_IHR ? 1 : 0);
  if (U_TOSY)
    Y |= state->OS;
  if (U_TE1Y)
    Y |= 0;
  if (U_TE2Y)
    Y |= 0;

  BL = boolean(U_BL, state->M, state->Q);
  special(U_MS, BL, &X, state);

  //The adder output takes another cycle to settle.
  if (U_TAX)
    X |= (BL + boolean(U_BR, state->Z, state->Q) + (U_LOC ? 1 : 0)) & 077777777;

  BRANCH = branch(U_MC, X, Y, BL, state);

  RCE =
    (state->XXC_XXB == 0 && !U_VCY && (U_DGO || !BRANCH)) ||
    (state->XXC_XXB == 1 && !(U_VCY && BRANCH)) ||
    state->XXC_XXB == 2;

  // Registers are loaded last in the cycle.
  if (RCE) {
    if (U_LR0)
      state->R[0] = (U_TXW ? X : 0) | (U_TYW ? Y : 0);
    if (U_TXW || U_TYW)
      state->R[U_LRN] = (U_TXW ? X : 0) | (U_TYW ? Y : 0);
    if (U_LSPX)
      state->SP[U_SSP | (U_MS == 013 ? state->Z & 077 : 0)] = X;
    if (U_LMX || U_LMY)
      state->M = (U_LMX ? X : 0) | (U_LMY ? Y : 0);
    if (U_LQX || U_LQY)
      state->Q = (U_LQX ? X : 0) | (U_LQY ? Y : 0);
    if (U_LZX || U_LZY)
      state->Z = (U_LZX ? X : 0) | (U_LZY ? Y : 0);

    sim_debug(DBG_STATE, dev, "X %08o Y %08o", X, Y);
    if (U_LMX || U_LMY)
      sim_debug(DBG_STATE, dev, " M %08o", state->M);
    if (U_LQX || U_LQY)
      sim_debug(DBG_STATE, dev, " Q %08o", state->Q);
    if (U_LZX || U_LZY)
      sim_debug(DBG_STATE, dev, " Z %08o", state->Z);
    if (U_LR0)
      sim_debug(DBG_STATE, dev, " R0 %08o", state->R[0]);
    if (U_TXW || U_TYW)
      sim_debug(DBG_STATE, dev, " R%o %08o", U_LRN, state->R[U_LRN]);
    if (U_LSPX)
      sim_debug(DBG_STATE, dev, " SP%o %08o",
                U_SSP | (U_MS == 013 ? state->Z & 077 : 0),
                state->SP[U_SSP | (U_MS == 013 ? state->Z & 077 : 0)]);
    sim_debug(DBG_STATE, dev, "\n");
  }

  // Compute next state, load I, and compute next microcode address.
  switch(state->XXC_XXB) {
  case 0:
    if ((BRANCH && !U_DGO) || U_VCY)
      state->XXC_XXB = 1;
    if (U_VCY)
      break;
    if (!BRANCH || U_DGO)
      memcpy(state->I, state->ROM[state->O], sizeof state->I);
    if (BRANCH)
      jump(U_MCONT, U_B, X, state);
    else
      oload(state->O + 1, state);
    break;
  case 1:
    if (BRANCH && !U_DGO && U_VCY)
      state->XXC_XXB = 2;
    else
      state->XXC_XXB = 0;
    memcpy(state->I, state->ROM[state->O], sizeof state->I);
    if (U_VCY && BRANCH)
      jump(U_MCONT, U_B, X, state);
    else
      oload(state->O + 1, state);
    break;
  case 2:
    state->XXC_XXB = 0;
    memcpy(state->I, state->ROM[state->O], sizeof state->I);
    oload(state->O + 1, state);
    break;
  }
}

t_stat sim_instr(void)
{
  t_stat reason;

  if ((reason = build_dev_tab ()) != SCPE_OK)
    return reason;

  for (;;) {
    AIO_CHECK_EVENT;
    if (sim_interval <= 0) {
      if ((reason = sim_process_event()) != SCPE_OK)
        return reason;
    }

#if 0
    if (sim_brk_summ && sim_brk_test(PC, SWMASK('E')))
      return STOP_IBKPT;
#endif

    if (sim_step != 0) {
      if (--sim_step == 0)
        return SCPE_STEP;
    }

    cpu_cycle(&cpu0_dev, &cpu0_state);
    sim_debug(DBG_TRACE, &cpu0_dev, "%c %04o %08o%08o%08o\n",
              'A' + cpu0_state.XXC_XXB, cpu0_state.O,
              cpu0_state.I[0], cpu0_state.I[1], cpu0_state.I[2]);

    cpu_cycle(&msch_dev, &msch_state);

#if 0
    cpu_cycle(&cpu1_dev, &cpu1_state);
    cpu_cycle(&chio_dev, &chio_state);
    cpu_cycle(&amc_dev, &amc_state);
    cpu_cycle(&amtu_dev, &amtu_state);
#endif
  }

  return SCPE_OK;
}

static t_stat cpu_ex(t_value *vptr, t_addr ea, UNIT *uptr, int32 sw)
{
  if (vptr == NULL)
    return SCPE_ARG;
  return SCPE_OK;
}

static t_stat cpu_dep(t_value val, t_addr ea, UNIT *uptr, int32 sw)
{
  return SCPE_OK;
}

static t_bool pc_is_a_subroutine_call(t_addr **ret_addrs)
{
  return FALSE;
}

static t_stat
cpu_reset(DEVICE *dptr)
{
  sim_brk_types = SWMASK('D') | SWMASK('E');
  sim_brk_dflt = SWMASK('E');
  sim_vm_is_subroutine_call = &pc_is_a_subroutine_call;
  if (sim_switches & SWMASK('P')) {
    memset(&cpu0_state, 0, sizeof cpu0_state);
    /*
      START:  ;
              GOTO *+1;
              CALL SUBR;
              DGOTO START;
              R0←R0+1;
      SUBR:   Z←-1, Q←SK0;
              SK0←Z+Q;
              RETURN;
    */
    cpu0_state.ROM[0000][0] = 00000000000;  // .MC = 0
    cpu0_state.ROM[0001][0] = 02000020000;  // .MC = 20, .B = 2
    cpu0_state.ROM[0002][0] = 02021000000;  // .MC = 20, .MCONT = 1, .B = 100
    cpu0_state.ROM[0003][0] = 02000000000;  // .MC = 20, .B = 0, .DGO
    cpu0_state.ROM[0003][2] = 00000000004;
    cpu0_state.ROM[0004][0] = 00000000000;  // .IHR, .THY, .LR0, .TYW
    cpu0_state.ROM[0004][1] = 00000424000;
    cpu0_state.ROM[0100][0] = 00000007777;  // .TSPY, .LQY, .TCX, .C = -1, .LZX
    cpu0_state.ROM[0100][1] = 07777240000;
    cpu0_state.ROM[0100][2] = 00000060000;
    cpu0_state.ROM[0101][1] = 00000002001;  // .BL = Q, .BR = Z, .TAX, .LSPX, .VCY
    cpu0_state.ROM[0101][2] = 00000001110;
    cpu0_state.ROM[0102][0] = 02040000000;  // .MC = 20, .MCONT = 2
    memcpy(cpu0_state.I, cpu0_state.ROM[cpu0_state.O], sizeof cpu0_state.I);
    oload(cpu0_state.O + 1, &cpu0_state);

    memset(&msch_state, 0, sizeof msch_state);
    microword_t *ROM = msch_state.ROM;
    #include "bcc500_msch_microcode.c"
    memcpy(msch_state.I, msch_state.ROM[msch_state.O], sizeof msch_state.I);
    oload(msch_state.O + 1, &msch_state);
  }
  return SCPE_OK;
}
