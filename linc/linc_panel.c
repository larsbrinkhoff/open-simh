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
#include "linc_subprocess.h"
#include "linc_panel.h"

/* If the result from sim_poll_kbd() has SCPE_KFLAG set, these are the
   character data bits.  Unfortunately The SIMH framework doesn't have
   a symbolic definition for this. */
#define SCPE_KMASK  0xFF

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

#define BUFFER_SIZE 250

static uint16 panel_state[128];
static SIM_SUBPROCESS *panel_proc = NULL;
static uint8 panel_buffer[BUFFER_SIZE + 3];
static uint8 panel_length = 0;
static char panel_cmd[100];

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
panel_flush(void)
{
  t_stat stat = sim_subprocess_write(panel_proc, panel_buffer, panel_length);
  if (stat != SCPE_OK)
    sim_messagef(stat, "Error writing to subprocess.");
  panel_length = 0;
}

void
panel_send(uint8 item, uint16 value)
{
  t_stat stat;
  uint8 data[3];

  if (panel_proc == NULL)
    return;

  item &= 0x7F;
  if (panel_state[item] == value)
    return;
  panel_state[item] = value;

  panel_buffer[panel_length++] = 0x80 | item;
  panel_buffer[panel_length++] = (value >> 7) & 0x7F;
  panel_buffer[panel_length++] = value & 0x7F;
  if (panel_length >= BUFFER_SIZE)
    panel_flush();
}

static void
panel_start(uint16 address)
{
  P = address;
  strncpy(panel_cmd, "continue", sizeof panel_cmd);
}

static void
panel_receive(uint8 item, uint16 value)
{
  switch (item) {
  case PANEL_ON_OFF:
    if (value == 0)
      exit_cmd(0, "0");
    break;
  case PANEL_CLEAR:
    // light when pressed
    // memory clear and test mode
    // clear all other functions
    // memory zeroed when leaving clear mode
    panel_send(PANEL_ISTOP, 0);
    panel_send(PANEL_XOESTOP, 0);
    panel_send(PANEL_EXAM, 0);
    panel_send(PANEL_AUTO_RESTART, 0);
    panel_send(PANEL_CYCLE_BY_CYCLE, 0);
    panel_send(PANEL_INSTR_BY_INSTR, 0);
    panel_send(PANEL_CYCLE, PANEL_CYCLE_OFF);
    panel_send(PANEL_RUN_PAUSE, PANEL_RUNNING);
    break;
  case PANEL_MARK:
    // todo
    // light when pressed
    // clear all other functions except istop and xoestop
    // turned off by other function, or HLT or MTP
    panel_send(PANEL_EXAM, 0);
    panel_send(PANEL_AUTO_RESTART, 0);
    panel_send(PANEL_CYCLE_BY_CYCLE, 0);
    panel_send(PANEL_INSTR_BY_INSTR, 0);
    panel_start(040);
    break;
  case PANEL_AUTO_RESTART:
    // todo
    // light when pressed
     // repeat exam, step, 
    break;
  case PANEL_ISTOP:
    // todo
    // Instruction fetch breakpoint at the momentary value of LSW.
    if (value && !panel_state[PANEL_CLEAR]) {
      panel_send(PANEL_XOESTOP, 0);
      panel_send(PANEL_AUTO_RESTART, 0);
    }
    break;
  case PANEL_XOESTOP:
    // todo
    // Data read/write breakpoint at the momentary value of LSW.
    if (value && !panel_state[PANEL_CLEAR]) {
      panel_send(PANEL_ISTOP, 0);
      panel_send(PANEL_AUTO_RESTART, 0);
    }
    break;
  case PANEL_SENSE_SWITCHES:
    SSW = value & RMASK;
    break;
  case PANEL_AUDIO:
    // todo
    break;
  case PANEL_DELAY:
    // todo
    break;
  case PANEL_START_400:
    // clear all other functions except istop and xoestop
    if (value) {
      panel_send(PANEL_EXAM, 0);
      panel_send(PANEL_AUTO_RESTART, 0);
      panel_send(PANEL_CYCLE_BY_CYCLE, 0);
      panel_send(PANEL_INSTR_BY_INSTR, 0);
      panel_start(0400);
    }
    return;
  case PANEL_START_20:
    // clear all other functions except istop and xoestop
    if (value) {
      panel_send(PANEL_EXAM, 0);
      panel_send(PANEL_AUTO_RESTART, 0);
      panel_send(PANEL_CYCLE_BY_CYCLE, 0);
      panel_send(PANEL_INSTR_BY_INSTR, 0);
      panel_start(020);
    }
    return;
  case PANEL_START_RS:
    // clear all other functions except istop and xoestop
    if (value) {
      panel_send(PANEL_EXAM, 0);
      panel_send(PANEL_AUTO_RESTART, 0);
      panel_send(PANEL_CYCLE_BY_CYCLE, 0);
      panel_send(PANEL_INSTR_BY_INSTR, 0);
      panel_start(RSW);
    }
    return;
  case PANEL_FILL_STEP:
    // clear all other functions except istop and xoestop
    if (value) {
      panel_send(PANEL_EXAM, 0);
      panel_send(PANEL_AUTO_RESTART, 0);
      panel_send(PANEL_CYCLE_BY_CYCLE, 0);
      panel_send(PANEL_INSTR_BY_INSTR, 0);
      panel_send(PANEL_MEMORY_CONTENTS, M[S] = B = RSW);
      return;
    } else {
      panel_state[item] = value;
      panel_send(item, 0);
      item = PANEL_EXAM_STEP;
      value = 1;
      // Fall through to PANEL_EXAM_STEP.
    }
  case PANEL_EXAM_STEP:
    // clear all other functions except istop and xoestop
    if (value) {
      panel_send(PANEL_AUTO_RESTART, 0);
      panel_send(PANEL_CYCLE_BY_CYCLE, 0);
      panel_send(PANEL_INSTR_BY_INSTR, 0);
      S = (S & HMASK) | ((S + 1) & AMASK);
      panel_send(PANEL_MEMORY_ADDRESS, S);
      panel_send(PANEL_MEMORY_CONTENTS, B = M[S]);
      panel_send(PANEL_EXAM, 1);
      stop_reason = STOP_EXAM;
    }
    break;
  case PANEL_FILL:
    if (value) {
      // clear all other functions except istop and xoestop
      panel_send(PANEL_EXAM, 0);
      panel_send(PANEL_AUTO_RESTART, 0);
      panel_send(PANEL_CYCLE_BY_CYCLE, 0);
      panel_send(PANEL_INSTR_BY_INSTR, 0);
      panel_send(PANEL_MEMORY_ADDRESS, S = LSW);
      panel_send(PANEL_MEMORY_CONTENTS, M[S] = B = RSW);
      return;
    }
    // Fall through to PANEL_EXAM_STEP.
  case PANEL_EXAM:
    // clear all other functions except istop and xoestop
    if (value) {
      panel_send(PANEL_AUTO_RESTART, 0);
      panel_send(PANEL_CYCLE_BY_CYCLE, 0);
      panel_send(PANEL_INSTR_BY_INSTR, 0);
      panel_send(PANEL_MEMORY_ADDRESS, S = LSW);
      panel_send(PANEL_MEMORY_CONTENTS, B = M[S]);
      stop_reason = STOP_EXAM;
    }
    break;
  case PANEL_CYCLE_BY_CYCLE:
    panel_state[item] = value;
    if (value) {
      panel_send(PANEL_INSTR_BY_INSTR, 0);
      goto step;
    }
    return;
  case PANEL_INSTR_BY_INSTR:
    panel_state[item] = value;
    if (value) {
      panel_send(PANEL_CYCLE_BY_CYCLE, 0);
      goto step;
    }
    return;
  case PANEL_STEP:
    if (value) {
    step:
      panel_send(PANEL_EXAM, 0);
      if (!panel_state[PANEL_CYCLE_BY_CYCLE])
        panel_send(PANEL_INSTR_BY_INSTR, 1);
      stop_reason = STOP_STEP;
      strncpy(panel_cmd, "step", sizeof panel_cmd);
    }
    return;
  case PANEL_RESUME:
    if (value) {
      panel_send(PANEL_EXAM, 0);
      panel_send(PANEL_CYCLE_BY_CYCLE, 0);
      panel_send(PANEL_INSTR_BY_INSTR, 0);
      panel_start(P);
    }
    return;
  case PANEL_DO_TOG_INSTR:
    if (value) {
      panel_send(PANEL_EXAM, 0);
      panel_send(PANEL_AUTO_RESTART, 0);
      panel_send(PANEL_CYCLE_BY_CYCLE, 0);
      panel_send(PANEL_INSTR_BY_INSTR, 0);
      cpu_do();
      panel_send(PANEL_MEMORY_ADDRESS, S = 0);
      stop_reason = STOP_STOP;
    }
    return;
  case PANEL_STOP:
    if (value) {
      panel_send(PANEL_ISTOP, 0);
      panel_send(PANEL_XOESTOP, 0);
      panel_send(PANEL_EXAM, 0);
      panel_send(PANEL_AUTO_RESTART, 0);
      panel_send(PANEL_CYCLE, PANEL_CYCLE_OFF);
      panel_send(PANEL_MEMORY_CONTENTS, B = 0);
      paused = 0;
      stop_reason = STOP_STOP;
    }
    return;
  case PANEL_LEFT_SWITCHES:
    LSW = value;
    break;
  case PANEL_RIGHT_SWITCHES:
    RSW = value;
    break;
  case PANEL_GONG_ON_OFF:
    if (value)
      set_cmd(0, "GONG ENABLED");
    else
      set_cmd(0, "GONG DISABLED");
    break;
  case PANEL_SPEAKER_SELECT:
    switch(value) {
    case 0: set_cmd(0, "MUS S1"); break;
    case 1: set_cmd(0, "MUS Z11"); break;
    case 2: set_cmd(0, "MUS A11"); break;
    }
    break;
  case PANEL_SCOPE_KNOB_0:
  case PANEL_SCOPE_KNOB_1:
  case PANEL_SCOPE_KNOB_2:
  case PANEL_SCOPE_KNOB_3:
  case PANEL_SCOPE_KNOB_4:
  case PANEL_SCOPE_KNOB_5:
  case PANEL_SCOPE_KNOB_6:
  case PANEL_SCOPE_KNOB_7:
    if (value & 0200)
      value |= 07400;
    else
      value &= ~07400;
    SAM[item - PANEL_SCOPE_KNOB_0] = value;
    break;
  default:
    sim_debug(DBG, &panel_dev, "Unknown panel item for input.\n");
    return;
  }
  panel_state[item] = value;
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

static char *
panel_getchar(FILE *f, char *cptr)
{
  t_stat ch, stat;

  ch = sim_poll_kbd();

  if ((ch & SCPE_KFLAG) == 0 || (ch & SCPE_BREAK) != 0) {
    sim_os_ms_sleep(10);
    return cptr;
  }

  ch &= SCPE_KMASK;
  switch (ch) {
  case '\b':
  case 0177:
     stat = sim_putchar('\b');
     stat = sim_putchar(' ');
     stat = sim_putchar('\b');
     return --cptr;
  case '\r':
  case '\n':
    stat = sim_putchar('\r');
    stat = sim_putchar('\n');
    *cptr = 0;
    return NULL;
  }

  stat = sim_putchar(ch);
  if (stat != SCPE_OK)
    sim_messagef(stat, "Error echoing character.");

  *cptr++ = ch;
  return cptr;
}

static char *
panel_fgets(char *cptr, int32 n, FILE *f)
{
  char *data = cptr;

  fflush(stdout);
  cpu_update_panel();
  panel_flush();

  /* Non blocking read from SIMH console. */
  sim_ttrun();

  while (n > 0) {
    panel_svc(&panel_unit);
    if (*panel_cmd != 0) {
       strncpy(data, panel_cmd, n);
       *panel_cmd = 0;
       break;
    }
    cptr = panel_getchar(f, cptr);
    if (cptr == NULL)
      break;
  }

  sim_ttcmd();

  return data;
}

static t_stat
panel_reset(DEVICE *dptr)
{
  t_stat stat;

  if ((dptr->flags & DEV_DIS) != 0 || (sim_switches & SWMASK('P')) != 0) {
    sim_vm_read = NULL;
    sim_cancel(&panel_unit);
    return SCPE_OK;
  }

  memset(panel_state, 0, sizeof panel_state);
  *panel_cmd = 0;
  sim_vm_read = &panel_fgets;
  sim_activate(&panel_unit, 1);
  return SCPE_OK;
}

static t_stat
panel_attach(UNIT *uptr, CONST char *cptr)
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

static t_stat
panel_detach(UNIT *uptr)
{
  if ((uptr->flags & UNIT_ATT) == 0)
    return SCPE_NOATT;

  panel_length = 0;
  free(uptr->filename);
  uptr->flags &= ~UNIT_ATT;
  return sim_subprocess_stop(panel_proc);
}
