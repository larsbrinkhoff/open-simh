/* pdp11_nm.c: Newman Multiprocessor.
  ------------------------------------------------------------------------------

   Copyright (c) 2025 Lars Brinkhoff.

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
   THE AUTHOR BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
   IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

   Except as contained in this notice, the name of the author shall not be
   used in advertising or otherwise to promote the sale, use or other dealings
   in this Software without prior written authorization from the author.

  ------------------------------------------------------------------------------

*/

#include "pdp11_defs.h"

//#include "sim_tmxr.h"

#define IOLN_NM  070
#define DBG_TRC  0x0001
#define DBG_REG  0x0002
#define DBG_PKT  0x0004
#define DBG_DAT  0x0008
#define DBG_INT  0x0010
#define DBG_ERR  0x0020

t_stat nm_svc(UNIT *);
t_stat nm_reset (DEVICE *);
t_stat nm_attach (UNIT *, CONST char *);
t_stat nm_detach (UNIT *);
t_stat nm_rd(int32 *, int32, int32);
t_stat nm_wr(int32, int32, int32);
t_stat nm_help (FILE *, DEVICE *, UNIT *, int32, const char *);
t_stat nm_help_attach (FILE *, DEVICE *, UNIT *, int32, const char *);
const char *nm_description (DEVICE *);

static uint16 nm_enb;        // Enable.
static uint16 nm_pid;        // Processor ID.
static uint16 nm_mbx[16];    // Mail boxes.
static uint8 nm_map[4];      // Mappings.
static uint16 nm_mbm;        // Interrupt enable mask.
static uint16 nm_mbt;        // Mailbox transmit status.
static uint16 nm_mbr;        // Mailbox receive status.

UNIT nm_unit[] = {
  { UDATA (&nm_svc, UNIT_IDLE|UNIT_ATTABLE, 0) },
};

REG nm_reg[] = {
  { HRDATAD(ENB, nm_enb, 16, "Enable"), 0 },
  { HRDATAD(PID, nm_pid, 16, "Processor ID"), 0 },
  { BRDATAD(MAP, nm_map, 16,  4,  4, "Map"), 0 },
  { HRDATAD(MBM, nm_mbm, 16, "Interrupt enable mask"), 0 },
  { BRDATAD(MBX, nm_mbx, 16, 16, 32, "Mail boxes"), 0 },
  { HRDATAD(MBM, nm_mbt, 16, "Mailbox transmit status"), 0 },
  { HRDATAD(MBM, nm_mbr, 16, "Mailbot receive status"), 0 },
  { NULL }  };

MTAB nm_mod[] = {
  { MTAB_XTD|MTAB_VDV|MTAB_VALR, 010, "ADDRESS", "ADDRESS",
    &set_addr, &show_addr, NULL, "Unibus address" },
  { MTAB_XTD|MTAB_VDV|MTAB_VALR, 0, "VECTOR", "VECTOR",
    &set_vec, &show_vec, NULL, "Interrupt vector" },
  { 0 },
};

DIB nm_dib = {
  IOBA_AUTO, IOLN_NM, &nm_rd, &nm_wr,
  1, IVCL (NM), VEC_AUTO
};

DEBTAB nm_debug[] = {
    { "TRC",       DBG_TRC,   "Detailed trace" },
    { "REG",       DBG_REG,   "Hardware registers" },
    { "PKT",       DBG_PKT,   "Packets" },
    { "DAT",       DBG_DAT,   "Packet data" },
    { "INT",       DBG_INT,   "Interrupts" },
    { "ERR",       DBG_ERR,   "Error conditions" },
    { 0 }
};

DEVICE nm_dev = {
    "NM", nm_unit, nm_reg, nm_mod,
    1, 8, 16, 1, 8, 16,
    NULL, NULL, &nm_reset,
    NULL, &nm_attach, &nm_detach,
    &nm_dib, DEV_DISABLE | DEV_DIS | DEV_QBUS | DEV_DEBUG | DEV_MUX,
    0, nm_debug, NULL, NULL, &nm_help, &nm_help_attach, NULL,
    &nm_description
  };

int nm_test_int (void)
{
  if (nm_mbt & nm_mbm) {
    unsigned i;
    sim_debug (DBG_INT, &nm_dev, "Interrupt\n");
    for (i = 1; i < 0200000; i <<= 1) {
      SET_INT(NM);
    }
    return 1;
  } else {
    CLR_INT(NM);
    return 0;
  }
}

t_stat nm_rd (int32 *data, int32 PA, int32 access)
{
  PA &= 077;
  switch (PA) {
  case 000: case 002: case 004: case 006: // NM.MBX
  case 010: case 012: case 014: case 016:
  case 020: case 022: case 024: case 026:
  case 030: case 032: case 034: case 036:
    *data = nm_mbx[PA >> 1];
    nm_mbt &= ~(1 << (PA >> 1));
    break;
  case 040: // NM.MAP
  case 042:
    *data = nm_map[PA & 2] + (nm_map[(PA & 2) + 1] << 8);
    break;
  case 044: // NM.MBM
    *data = nm_mbm;
    break;
  case 046: // NM.KFR
    // touching this word resets pending interrupts
    break;
  case 050: // NM.ENB
    *data = nm_enb;
    break;
  case 054: // NM.PID
    *data = nm_pid;
    break;
  case 062: // NM.MBT
    *data = nm_mbt;
    break;
  case 064: // NM.MBR
    *data = nm_mbr;
    break;
  default:
    *data = 0;
    break;
  }
  return SCPE_OK;
}

t_stat nm_wr (int32 data, int32 PA, int32 access)
{
  PA &= 077;
  switch (PA) {
  case 000: case 002: case 004: case 006: // NM.MBX
  case 010: case 012: case 014: case 016:
  case 020: case 022: case 024: case 026:
  case 030: case 032: case 034: case 036:
    nm_mbx[PA >> 1] = data;
    nm_mbt |= 1 << (PA >> 1);
    break;
  case 040: // NM.MAP
  case 042:
    if (access == WRITEB)
      nm_map[PA & 3] = data;
    else {
      nm_map[(PA & 2) + 0] = data & 0xFF;
      nm_map[(PA & 2) + 1] = data >> 8;
    }
    break;
  case 044: // NM.MBM
    nm_mbm = data;
    break;
  case 046: // NM.KFR
    break;
  case 050: // NM.ENB
    nm_enb = data;
    break;
  case 054: // NM.PID
    nm_pid = data;
    break;
  case 062: // NM.MBT
    nm_mbt = data;
    break;
  case 064: // NM.MBR
    nm_mbr = data;
    break;
  default:
    break;
  }
  return SCPE_OK;
}

t_stat nm_svc(UNIT *uptr)
{
  return SCPE_OK;
}

t_stat nm_attach (UNIT *uptr, CONST char *cptr)
{
  return SCPE_OK;
}

t_stat nm_detach (UNIT *uptr)
{
  sim_cancel (uptr);
  return SCPE_OK;
}

t_stat nm_reset (DEVICE *dptr)
{
  return auto_config (dptr->name, (dptr->flags & DEV_DIS)? 0 : 1);  /* auto config */
}

const char *nm_description (DEVICE *dptr)
{
  return "NM Newman multiprocessor";
}

t_stat nm_help (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, const char *cptr)
{
  fprintf (st, "NM Newman multiprocessor\n\n");
  fprint_show_help (st, dptr);
  nm_help_attach (st, dptr, uptr, flag, cptr);
  return SCPE_OK;
}

t_stat nm_help_attach (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, const char *cptr)
{
  fprintf (st, "To configure NM...\n");
  return SCPE_OK;
}
