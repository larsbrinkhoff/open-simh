/* linc_gong.c: Gong for sounding a chime.

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

static t_stat gong_svc(UNIT *uptr);
static t_stat gong_reset(DEVICE *dptr);
static t_stat gong_attach(UNIT *uptr, CONST char *cptr);
static t_stat gong_detach(UNIT *uptr);

/* Debug */
#define DBG_CHIME       0001

static UNIT gong_unit = { UDATA(NULL, 0, 0) };

static SIM_SUBPROCESS *gong_proc = NULL;

static MTAB gong_mod[] = {
  { 0 }
};

static DEBTAB gong_deb[] = {
  { "CHIME",  DBG_CHIME },
  { NULL, 0 }
};

DEVICE gong_dev = {
  "GONG", &gong_unit, NULL, gong_mod,
  1, 8, 12, 1, 8, 12,
  NULL, NULL, &gong_reset,
  NULL, &gong_attach, &gong_detach,
  NULL, DEV_DISABLE | DEV_DIS | DEV_DEBUG, 0, gong_deb,
  NULL, NULL, NULL, NULL, NULL, NULL
};

static t_stat gong_reset(DEVICE *dptr)
{
  t_stat stat;

  if (dptr->flags & DEV_DIS) {
    gong_detach(&gong_unit);
  } else {
    ;
  }

  return SCPE_OK;
}

static t_stat gong_attach(UNIT *uptr, CONST char *cptr)
{
  t_stat stat;

  if ((uptr->flags & UNIT_ATT) != 0)
    return SCPE_ALATT;

  gong_detach(uptr);

  uptr->filename = (char *)calloc(CBUFSIZE, sizeof(char));
  if (uptr->filename == NULL)
    return SCPE_MEM;

  stat = sim_subprocess_start(&gong_proc, cptr);
  if (stat != SCPE_OK) {
    free(uptr->filename);
    return stat;
  }

  strlcpy(uptr->filename, cptr, CBUFSIZE);
  uptr->flags |= UNIT_ATT;
  return SCPE_OK;
}

static t_stat gong_detach(UNIT *uptr)
{
  if ((uptr->flags & UNIT_ATT) == 0)
    return SCPE_NOATT;

  free(uptr->filename);
  uptr->flags &= ~UNIT_ATT;
  return sim_subprocess_stop(gong_proc);
}

void gong_chime(void)
{
  sim_debug(DBG_CHIME, &gong_dev, "Gong chime.\n");

  if (gong_dev.flags & DEV_DIS)
    ;
  else if (gong_unit.flags & UNIT_ATT)
    sim_subprocess_write(gong_proc, "!", 1);
#ifdef LINC_USE_AUDIO
  else
    mus_chime();
#endif
}
