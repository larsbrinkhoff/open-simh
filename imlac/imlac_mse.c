/* imlac_mse.c: MIT mouse and keyset device

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
static t_stat mse_svc (UNIT *uptr);
static t_stat mse_reset (DEVICE *dptr);
static uint16 mse_iot (uint16, uint16);

/* State. */
static uint16 MRB; /* Mouse position. */
static uint16 MRC; /* Mouse coordinates. */
static uint16 MSW = 0177777; /* Mouse and keyset switches. */

static UNIT mse_unit = {
  UDATA (&mse_svc, UNIT_IDLE, 0)
};

static REG mse_reg[] = {
  { ORDATAD (MRB, MRB, 16, "Mouse position") },
  { ORDATAD (MRC, MRC, 16, "Mouse coordinates") },
  { ORDATAD (MSW, MSW, 16, "Mouse and keyset switches") },
  { NULL }
};

static IMDEV mse_imdev = {
  2,
  { { 0070, mse_iot, { "MRB", "MCF", "MRC" } },
    { 0073, mse_iot, { "MSW", NULL, NULL } } }
};

static DEBTAB mse_deb[] = {
  { "DBG", DBG },
  { NULL, 0 }
};

DEVICE mse_dev = {
  "MSE", &mse_unit, NULL, NULL,
  0, 8, 16, 1, 8, 16,
  NULL, NULL, &mse_reset,
  NULL, NULL, NULL,
  &mse_imdev, DEV_DISABLE | DEV_DEBUG | DEV_DIS, 0, mse_deb,
  NULL, NULL, NULL, NULL, NULL, NULL
};

static t_stat
mse_svc (UNIT *uptr)
{
#ifdef HAVE_LIBSDL
  // SIM_MOUSE_EVENT ev;
  // vid_poll_mouse (&ev);
  // x/y_rel
  // x/y_pos
  // b1/2/3_state
  sim_activate_after (&mse_unit, 10000);
#endif
  return SCPE_OK;
}

#define KEYSET_1    0000001   /* Maze: go back. */
#define KEYSET_2    0000002   /* Maze: turn right. */
#define KEYSET_3    0000004   /* Maze: go forward. */
#define KEYSET_4    0000010   /* Maze: turn left. */
#define KEYSET_5    0000020   /* Maze: turn around. */
#define MOUSE_1     0000400   /* Maze: peek right. */
#define MOUSE_2     0001000   /* Maze: fire. */
#define MOUSE_3     0002000   /* Maze: peek left. */

static void
mse_motion (int device, int axis, int value)
{
  sim_debug (DBG, &mse_dev, "Game motion %d/%d/%d\n", device, axis, value);
  switch (axis) {
  case 0:
    MSW |= KEYSET_2|KEYSET_4;
    if (value < -10000)
      MSW &= ~KEYSET_4;
    else if (value > 1000)
      MSW &= ~KEYSET_2;
    break;
  case 1:
    MSW |= KEYSET_1|KEYSET_3;
    if (value < -10000)
      MSW &= ~KEYSET_3;
    else if (value > 1000)
      MSW &= ~KEYSET_1;
    break;
  case 2: /* Some gamepads have these mixed up. */
  case 3:
    MSW |= MOUSE_1|MOUSE_3;
    if (value < -10000)
      MSW &= ~MOUSE_3;
    else if (value > 1000)
      MSW &= ~MOUSE_1;
    break;
  }
}

static void
mse_button (int device, int button, int state)
{
  sim_debug (DBG, &mse_dev, "Game button %d/%d/%d\n", device, button, state);
  if (state)
    MSW |= MOUSE_2;
  else
    MSW &= ~MOUSE_2;
}

static t_stat
mse_reset (DEVICE *dptr)
{
  vid_register_gamepad_motion_callback (mse_motion);
  vid_register_gamepad_button_callback (mse_button);
  if (dptr->flags & DEV_DIS) {
    sim_cancel (&mse_unit);
  } else {
    sim_activate_abs (&mse_unit, 0);
  }
  return SCPE_OK;
}

static uint16
mse_iot (uint16 insn, uint16 AC)
{
  if ((insn & 0773) == 0701) { /* MRB */
    sim_debug (DBG, &mse_dev, "IOT MRB %06o\n", MRB);
    AC |= MRB;
  }
  if ((insn & 0773) == 0702) { /* MCF */
    sim_debug (DBG, &mse_dev, "IOT MCF %06o\n", 0);
    AC = 0;
  }
  if ((insn & 0773) == 0703) { /* MRC */
    sim_debug (DBG, &mse_dev, "IOT MRC %06o\n", MRC);
    AC |= MRC;
  }
  if ((insn & 0771) == 0731) { /* MSW */
    sim_debug (DBG, &mse_dev, "IOT MSW %06o\n", MSW);
    AC |= MSW;
  }
  return AC;
}
