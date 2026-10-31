/* linc_panel.c: LINC control console panel.

   Copyright (c) 2026, Lars Brinkhoff

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

#define P (*(uint16 *)cpu_reg[0].loc)
#define S (*(uint16 *)cpu_reg[6].loc)
#define B (*(uint16 *)cpu_reg[7].loc)
#define LSW (*(uint16 *)cpu_reg[9].loc)
#define RSW (*(uint16 *)cpu_reg[10].loc)
#define SSW (*(uint16 *)cpu_reg[11].loc)
#define paused (*(int *)cpu_reg[12].loc)
#define SAM ((uint16 *)cpu_reg[19].loc)

/*

             RUN       CYCLE            INSTRUCTION                   RELAYS
OFF          x x      x x x x         xxx xxx xxx xxx   CLEAR MARK    xxx xxx
             R P      I X O E

ON           AUTO     I    XOE      INSTRUCTION LOCATION           SENSE SWITCHES
            RESTART  STOP STOP          x xxx xxx xxx                 yyy yyy

AUDIO       DELAY        START
1234        1234          400
 O           O                                             L    ACCUMULATOR
                         START                             x  xxx xxx xxx xxx
                           20

                         START         MEMORY ADDRESS         MEMORY CONTENTS
                           RS          xx xxx xxx xxx         xxx xxx xxx xxx

FILL  FILL  RESUME CBC  DO TOG
STEP                     INSTR         LEFT SWITCHES           RIGHT SWITCHES
 y            y           y           yyy yyy yyy yyy         yyy yyy yyy yyy
STEP  EXAM  STEP   IBI  STOP
EXAM

small indicator lights = 71
lit buttons = 9
unlit buttons = 5
small toggle switches = 30
large momentary switches = 3
knobs = 2

lit buttons:
AUTO RESTART
ISTOP
XOESTOP
FILL
EXAM
CYCLE BY CYCLE
INSTR BY INSTR
CLEAR
MARK


mixer banks

RUN/CYCLE
.xx.xxxx
INSTRUCTION
..xxxxxx
..xxxxxx
RELAYS
..xxxxxx
INSTRUCTION LOCATION
....xxxx
..xxxxxx
SENSE SWITCHES
..xxxxxx
ACCUMULATOR
.xxxxxxx
..xxxxxx
MEMORY ADDRESS
...xxxxx
..xxxxxx
MEMORY CONTENTS
..xxxxxx
..xxxxxx
LEFT SWITCHES
..xxxxxx
..xxxxxx
RIGHT SWITCHES
..xxxxxx
..xxxxxx

START 400
START 20

FILL
EXAM
FILL STEP
EXAM STEP
RESUME
STEP
DO TOG INSTR
STOP

Item Bits Direction
  1.    1 <--> ON/OFF
  2.    2 ---> RUN, PAUSE
  3.    2 ---> CYCLE
  4.   12 ---> INSTRUCTION
  5.    1 <--> CLEAR
  6.    1 <--> MARK
  7.    6 ---> RELAYS
  8.    1 <--> AUTO RESTART
  9.    1 <--> ISTOP
 10.    1 <--> XOESTOP
 11.   10 ---> INSTRUCTION LOCATION
 12.    6 <--> SENSE SWITCHES
 13.    2 <--> AUDIO
 14.    2 <--> DELAY
 15.    1 <--- START 400
 16.    1 <--- START 20
 17.    1 <--- START RS
 18.   13 ---> ACCUMULATOR (also l bit)
 19.   12 ---> MEMORY ADDRESS (also h bit!)
 20.   12 ---> MEMORY CONTENTS
 21.    1 <--- FILL STEP
 22.    1 <--- EXAM STEP
 23.    1 <--> FILL
 24.    1 <--> EXAM
 25.    1 <--> CYCLE BY CYCLE
 26.    1 <--> INSTR BY INSTR
 27.    1 <--- RESUME
 28.    1 <--- STEP
 29.    1 <--- DO TOG INSTR
 30.    1 <--- STOP
 31.   12 <--> LEFT SWITCHES
 32.   12 <--> RIGHT SWITCHES
 33.    1 <--> Gong on/off
 34.    1 <--> Speaker output select
 35-42. 8 <--- Sampled analog knobs
 */

/*
  They took away the buttons and switches.
  They took away the blinking lights.
  They took away the source code.
  They took away the chunky CRT.
  They took away the compilers.
  They took away physical media.
  They took away the keyboard.
  They took away the mouse.
  They took away creativity.
  They took away privacy.
  They took away the joy.

  They put in touch screens.
  They put in locked down appstores.
  They put in consumerism.
  They put in advertising.
  They put in surveillance.
*/

/* Function declaration. */
static t_stat panel_svc(UNIT *uptr);
static t_stat panel_reset(DEVICE *dptr);
static t_stat panel_attach(UNIT *uptr, CONST char *cptr);
static t_stat panel_detach(UNIT *uptr);

/* Debug */
#define DBG             0001

static SIM_SUBPROCESS *panel_proc = NULL;

static UNIT panel_unit = {
  UDATA(&panel_svc, UNIT_IDLE, 0)
};

static DEBTAB panel_deb[] = {
  { "DBG",  DBG },
  { NULL, 0 }
};

DEVICE panel_dev = {
  "PANEL", &panel_unit, NULL, NULL,
  1, 8, 12, 1, 8, 12,
  NULL, NULL, &panel_reset,
  NULL, &panel_attach, &panel_detach,
  NULL, DEV_DISABLE | DEV_DIS | DEV_DEBUG, 0, panel_deb,
  NULL, NULL, NULL, NULL, NULL, NULL
};

void
panel_send(uint8 item, uint16 value)
{
  t_stat stat;
  uint8 data[3];

  if (panel_proc == NULL)
    return;

  item &= 0x7F;
  data[0] = 0x80 | item;
  data[1] = (value >> 7) & 0x7F;
  data[2] = value & 0x7F;
  stat = sim_subprocess_write(panel_proc, data, sizeof data);
  if (stat != SCPE_OK)
    sim_messagef(stat, "Error writing to subprocess.");
}

static void
panel_receive(uint8 item, uint16 value)
{
  sim_debug(DBG, &panel_dev, "Item %u value %04o (%u)\n", item, value, value);
  switch (item) {
  case 0:
    // on/off
    break;
  case 5:
    // clear
    break;
  case 6:
    // mark
    break;
  case 8:
    // auto restart
    break;
  case 9:
    // ibreak
    break;
  case 10:
    // rwbreak
    break;
  case 12:
    SSW = value & 077;
    break;
  case 13:
    // audio
    break;
  case 14:
    // delay
    break;
  case 15:
    P = 0400;
    break;
  case 16:
    P = 020;
    break;
  case 17:
    P = RSW;
    break;
  case 21:
    B = RSW;
    M[S] = B;
    panel_send(23, 0);
    // Fall through.
  case 22:
    S = (S + 1) & AMASK;
    B = M[S];
    panel_send(19, S);
    panel_send(20, B);
    panel_send(24, 1);
    break;
  case 23:
    S = LSW;
    B = RSW;
    M[S] = B;
    panel_send(23, 0);
    goto exam;
  case 24:
    S = LSW;
    B = M[S];
  exam:
    panel_send(19, S);
    panel_send(20, B);
    panel_send(24, 1);
    break;
  case 29:
    cpu_do();
    break;
  case 30:
    panel_send(2, 0);
    break;
  case 31:
    LSW = value;
    break;
  case 32:
    RSW = value;
    break;
  case 33:
    if (value)
      set_cmd(0, "GONG ENABLED");
    else
      set_cmd(0, "GONG DISABLED");
    break;
  case 34:
    switch(value) {
    case 1: set_cmd(0, "MUS S1"); break;
    case 2: set_cmd(0, "MUS Z11"); break;
    case 3: set_cmd(0, "MUS A11"); break;
    }
    break;
  case 35:
  case 36:
  case 37:
  case 38:
  case 39:
  case 40:
  case 41:
  case 42:
    if (value & 0400)
      value |= 07000;
    else
      value &= ~07000;
    SAM[item - 35] = value;
    break;
  default:
    sim_debug(DBG, &panel_dev, "Unknown item for input\n");
    break;
  }
}

static t_stat
panel_svc(UNIT *uptr)
{
  uint8 data[3];
  uint8 *p;
  uint16 value;
  size_t n;
  t_stat stat;
  int m;

  if (panel_proc == NULL)
    return SCPE_OK;

  sim_activate_after(uptr, 10000);

  do {
    n = 1;
    stat = sim_subprocess_read(panel_proc, data, &n);
    if (stat != SCPE_OK) {
      sim_messagef(stat, "Error reading from PANEL subprocess");
      return SCPE_OK;
    }
    if (n == 0)
      return SCPE_OK;
  } while ((data[0] & 0x80) == 0);

  for (p = data + 1, m = 2; m > 0; p += n, m -= n) {
    n = m;
    stat = sim_subprocess_read(panel_proc, p, &n);
    if (stat != SCPE_OK) {
      sim_messagef(stat, "Error reading from PANEL subprocess");
      return SCPE_OK;
    }
  }

  if ((data[1] & 0x80) != 0 || (data[2] & 0x80) != 0) {
    sim_messagef(stat, "Error reading from PANEL subprocess");
    return SCPE_OK;
  }

  data[0] &= 0x7F;
  value = (data[1] << 7) | data[2];
  panel_receive(data[0], value);
  sim_debug(DBG, &panel_dev, "Panel item %u, value %u\n", data[0], value);

  return SCPE_OK;
}

static t_stat
panel_reset(DEVICE *dptr)
{
  t_stat stat;

  if ((dptr->flags & DEV_DIS) != 0 || (sim_switches & SWMASK('P')) != 0) {
    sim_cancel(&panel_unit);
    return SCPE_OK;
  }

  if (crt_dev.flags & DEV_DIS)
    return SCPE_OK;

  sim_activate(&panel_unit, 1);
  return SCPE_OK;
}

static t_stat panel_attach(UNIT *uptr, CONST char *cptr)
{
  t_stat stat;

  if ((uptr->flags & UNIT_ATT) != 0)
    return SCPE_ALATT;

  panel_detach(uptr);

  uptr->filename = (char *)calloc(CBUFSIZE, sizeof(char));
  if (uptr->filename == NULL)
    return SCPE_MEM;

  stat = sim_subprocess_start(&panel_proc, cptr);
  if (stat != SCPE_OK) {
    free(uptr->filename);
    return stat;
  }

  strlcpy(uptr->filename, cptr, CBUFSIZE);
  uptr->flags |= UNIT_ATT;
  return SCPE_OK;
}

static t_stat panel_detach(UNIT *uptr)
{
  if ((uptr->flags & UNIT_ATT) == 0)
    return SCPE_NOATT;

  free(uptr->filename);
  uptr->flags &= ~UNIT_ATT;
  return sim_subprocess_stop(panel_proc);
}
