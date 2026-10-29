/* linc_sam.c: LINC sampled analog inputs.

   Copyright (c) 2025, Lars Brinkhoff

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

#include "linc_defs.h"
#include "sim_video.h"
#include "linc_subprocess.h"

/* Function declaration. */
static t_stat sam_svc(UNIT *uptr);
static t_stat sam_reset(DEVICE *dptr);
static t_stat sam_attach(UNIT *uptr, CONST char *cptr);
static t_stat sam_detach(UNIT *uptr);
static uint16 ones_complement(int value);

/* Debug */
#define DBG             0001

#define SSW (*(uint16 *)cpu_reg[11].loc)
#define SAM ((uint16 *)cpu_reg[19].loc)

static SIM_SUBPROCESS *sam_proc = NULL;

static UNIT sam_unit = {
  UDATA(&sam_svc, UNIT_IDLE, 0)
};

static DEBTAB sam_deb[] = {
  { "DBG",  DBG },
  { NULL, 0 }
};

DEVICE sam_dev = {
  "SAM", &sam_unit, NULL, NULL,
  1, 8, 12, 1, 8, 12,
  NULL, NULL, &sam_reset,
  NULL, &sam_attach, &sam_detach,
  NULL, DEV_DISABLE | DEV_DEBUG, 0, sam_deb,
  NULL, NULL, NULL, NULL, NULL, NULL
};

static void mixer_controller(uint8 port, int value)
{
  value &= ~0x7F;
  value >>= 6;
  value -= 127;
  SAM[port] = ones_complement(value);
  sim_debug(DBG, &sam_dev, "SAM %d set to %04o (%d)\n", port, SAM[port], value);
}

static t_stat
sam_svc(UNIT *uptr)
{
  uint8 data[3];
  uint8 *p;
  uint16 value;
  size_t n;
  t_stat stat;
  int m;

  if (sam_proc == NULL)
    return SCPE_OK;

  sim_activate_after(uptr, 10000);

  do {
    n = 1;
    stat = sim_subprocess_read(sam_proc, data, &n);
    if (stat != SCPE_OK) {
      sim_messagef(stat, "Error reading from SAM subprocess");
      return SCPE_OK;
    }
    if (n == 0)
      return SCPE_OK;
  } while ((data[0] & 0x80) == 0);

  for (p = data + 1, m = 2; m > 0; p += n, m -= n) {
    n = m;
    stat = sim_subprocess_read(sam_proc, p, &n);
    if (stat != SCPE_OK) {
      sim_messagef(stat, "Error reading from SAM subprocess");
      return SCPE_OK;
    }
  }

  if ((data[1] & 0x80) != 0 || (data[2] & 0x80) != 0) {
    sim_messagef(stat, "Error reading from SAM subprocess");
    return SCPE_OK;
  }

  data[0] &= 0x7F;
  value = (data[1] << 7) | data[2];
  sim_debug(DBG, &sam_dev, "Mixer controller %u, value %u\n", data[0], value);
  mixer_controller(data[0], value);

  return SCPE_OK;
}

static uint16 ones_complement(int value)
{
  if (value < 0)
    return (uint16)(-value & 07777) ^ 07777;
  else
    return (uint16)(value & 07777);
}

static void sam_motion(int which, int axis, int value)
{
  sim_debug(DBG, &sam_dev, "Motion %d, axis %d, value %d\n",
            which, axis, value);
  value /= 258;
  switch (which) {
  case 0:
    switch (axis) {
    case 0:
      SAM[0] = ones_complement(value);
      break;
    case 1:
      SAM[1] = ones_complement(-value);
      break;
    }
    break;
  case 1:
    switch (axis) {
    case 0:
      SAM[6] = ones_complement(value);
      break;
    case 1:
      SAM[7] = ones_complement(-value);
      break;
    }
    break;
  }
}

static void sam_button(int which, int button, int state)
{
  switch (which) {
  case 0:
    switch (button) {
    case 0:
      SSW = (SSW & 037) | (state << 5);
      break;
    }
    break;
  case 1:
    switch (button) {
    case 0:
      SSW = (SSW & 076) | state;
      break;
    }
    break;
  }
}

static t_stat
sam_reset(DEVICE *dptr)
{
  t_stat stat;

  if ((dptr->flags & DEV_DIS) != 0 || (sim_switches & SWMASK('P')) != 0) {
    sim_cancel(&sam_unit);
    return SCPE_OK;
  }

  if (crt_dev.flags & DEV_DIS)
    return SCPE_OK;

  crt_dev.reset(&crt_dev);
  stat = vid_register_gamepad_motion_callback(sam_motion);
  if (stat == SCPE_NOATT)
    return SCPE_OK; /* Gamepad not available. */
  if (stat != SCPE_OK && stat != SCPE_ALATT)
    return stat;
  stat = vid_register_gamepad_button_callback(sam_button);
  if (stat != SCPE_OK && stat != SCPE_ALATT)
    return stat;
  sim_activate(&sam_unit, 1);
  return SCPE_OK;
}

static t_stat sam_attach(UNIT *uptr, CONST char *cptr)
{
  t_stat stat;

  if ((uptr->flags & UNIT_ATT) != 0)
    return SCPE_ALATT;

  sam_detach(uptr);

  uptr->filename = (char *)calloc(CBUFSIZE, sizeof(char));
  if (uptr->filename == NULL)
    return SCPE_MEM;

  stat = sim_subprocess_start(&sam_proc, cptr);
  if (stat != SCPE_OK) {
    free(uptr->filename);
    return stat;
  }

  strlcpy(uptr->filename, cptr, CBUFSIZE);
  uptr->flags |= UNIT_ATT;
  return SCPE_OK;
}

static t_stat sam_detach(UNIT *uptr)
{
  if ((uptr->flags & UNIT_ATT) == 0)
    return SCPE_NOATT;

  free(uptr->filename);
  uptr->flags &= ~UNIT_ATT;
  return sim_subprocess_stop(sam_proc);
}
