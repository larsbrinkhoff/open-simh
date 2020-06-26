/* imlac_lp.c: Light pen device

   Copyright (c) 2020, Lars Brinkhoff

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

#include "imlac_defs.h"
#include "sim_video.h"

/* Debug */
#define DBG             0001

/* Function declaration. */
static t_stat lp_svc (UNIT *uptr);
static t_stat lp_reset (DEVICE *dptr);
static uint16 lp_iot (uint16, uint16);

/* State. */
static uint16 LBR; /* Lightpen buffer register. */

static UNIT lp_unit = {
  UDATA (&lp_svc, UNIT_IDLE, 0)
};

static REG lp_reg[] = {
  { ORDATAD (LBR, LBR, 16, "Lightpen buffer register") },
  { NULL }
};

static IMDEV lp_imdev = {
  1,
  { { 0013, lp_iot, { NULL } } }
};

static DEBTAB lp_deb[] = {
  { "DBG", DBG },
  { NULL, 0 }
};

DEVICE lp_dev = {
  "LP", &lp_unit, NULL, NULL,
  1, 8, 16, 1, 8, 16,
  NULL, NULL, &lp_reset,
  NULL, NULL, NULL,
  &lp_imdev, DEV_DISABLE | DEV_DEBUG | DEV_DIS, 0, lp_deb,
  NULL, NULL, NULL, NULL, NULL, NULL
};

static t_stat
lp_svc (UNIT *uptr)
{
#ifdef HAVE_LIBSDL
  // SIM_MOUSE_EVENT ev;
  // vid_poll_mouse (&ev);
  // x/y_rel
  // x/y_pos
  // b1/2/3_state
  sim_activate_after (&lp_unit, 10000);
#endif
  return SCPE_OK;
}

static t_stat
lp_reset (DEVICE *dptr)
{
  if (dptr->flags & DEV_DIS) {
    sim_cancel (&lp_unit);
  } else {
    sim_activate_abs (&lp_unit, 0);
  }
  return SCPE_OK;
}

static uint16
lp_iot (uint16 insn, uint16 AC)
{
  if ((insn & 0771) == 0131) {
    sim_debug (DBG, &lp_dev, "IOT %06o\n", insn);
    AC |= LBR;
  }
  if ((insn & 0772) == 0132) {
    sim_debug (DBG, &lp_dev, "IOT %06o\n", insn);
    flag_off (FLAG_LP);
  }
  if ((insn & 0774) == 0134) {
    sim_debug (DBG, &lp_dev, "IOT %06o\n", insn);
    pcinc (flag_check (FLAG_LP));
  }
  return AC;
}

void lp_hit (uint16 pc)
{
  LBR = pc;
  flag_on (FLAG_LP);
}
