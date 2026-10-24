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


#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include "linc_subprocess.h"

#ifndef _WIN32
struct subprocess {
  pid_t pid;
  int wfd, rfd;
};

t_stat sim_subprocess_start(SIM_SUBPROCESS **pptr, CONST char *command)
{
  int pipe1[2], pipe2[2];
  int flags;
  SIM_SUBPROCESS *ptr = malloc(sizeof(SIM_SUBPROCESS));

  *pptr = NULL;

  if (ptr == NULL)
    return SCPE_MEM;

  pipe(pipe1);
  pipe(pipe2);

  ptr->pid = fork();
  if (ptr->pid == -1) {
    free(ptr);
    return sim_messagef(SCPE_OPENERR, "Error starting process: %s\n", strerror(errno));
  }
  
  if (ptr->pid == 0) {
    dup2(pipe1[0], 0);
    dup2(pipe2[1], 1);
    dup2(pipe2[1], 2);
    close(pipe1[1]);
    close(pipe2[0]);
    system(command);
    exit(0);
  }

  flags = fcntl(pipe2[0], F_GETFL);
#ifdef O_NONBLOCK
  flags |= O_NONBLOCK;
#endif
#ifdef O_NDELAY
  flags |= O_NDELAY;
#endif
  fcntl(pipe2[0], F_SETFL, flags);

  ptr->wfd = pipe1[1];
  ptr->rfd = pipe2[0];
  close(pipe1[0]);
  close(pipe2[1]);

  *pptr = ptr;
  return SCPE_OK;
}

t_stat sim_subprocess_stop(SIM_SUBPROCESS *pptr)
{
  kill(pptr->pid, SIGTERM);
  free(pptr);
  return SCPE_OK;
}

t_stat sim_subprocess_read(SIM_SUBPROCESS *pptr, void *cptr, size_t *n)
{
  ssize_t x = read(pptr->rfd, cptr, *n);
  *n = 0;
  if (x == -1)
    return SCPE_IOERR;
  *n = x;
  return SCPE_OK;
}

t_stat sim_subprocess_write(SIM_SUBPROCESS *pptr, CONST void *cptr, size_t n)
{
  ssize_t x;

  while (n > 0) {
    x = write(pptr->wfd, cptr, n);
    if (x <= 0)
      return SCPE_IOERR;
    n -= x;
  }

  return SCPE_OK;
}

#else /* _WIN32 */

t_stat sim_subprocess_start(SIM_SUBPROCESS **pptr, CONST char *command)
{
  *pptr = NULL;
  return sim_messagef(SCPE_NOFNC, "Subprocess not implemented.\n");
}

t_stat sim_subprocess_stop(SIM_SUBPROCESS *pptr)
{
  return SCPE_NOFNC;
}

t_stat sim_subprocess_read(SIM_SUBPROCESS *pptr, void *cptr, size_t *n)
{
  *n = 0;
  return SCPE_NOFNC;
}

t_stat sim_subprocess_write(SIM_SUBPROCESS *pptr, CONST void *cptr, size_t n)
{
  return SCPE_NOFNC;
}

#endif /* _WIN32 */
