/* linc_subprocess.c: Run a subprocess.

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

#include "sim_defs.h"

typedef struct subprocess SIM_SUBPROCESS;

/* Start a subprocess running the command.  On success, *pptr will be
   a malloced memory area. */
extern t_stat sim_subprocess_start(SIM_SUBPROCESS **pptr, CONST char *command);

/* Stop a subprocess and free the malloced memory. */
extern t_stat sim_subprocess_stop(SIM_SUBPROCESS *pptr);

/* Receive data from a subprocess.  Try to receive *n bytes from the
   subprocess.  The actual amount read will be returned in *n. */
extern t_stat sim_subprocess_read(SIM_SUBPROCESS *pptr, void *cptr, size_t *n);

/* Send n bytes to a subprocess. */
extern t_stat sim_subprocess_write(SIM_SUBPROCESS *pptr, CONST void *cptr, size_t n);
