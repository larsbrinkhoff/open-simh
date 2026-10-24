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


#include "linc_subprocess.h"

#ifndef _WIN32

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>

struct SUBPROCESS {
  pid_t      pid;  /* The process ID. */
  int        rfd;  /* File descriptor for reading from the subprocess. */
  int        wfd;  /* File descriptor for writing to the subprocess. */
};

t_stat sim_subprocess_start(SIM_SUBPROCESS **pptr, CONST char *command)
{
  int pipe1[2], pipe2[2];
  int i, flags;
  SIM_SUBPROCESS *ptr = malloc(sizeof(SIM_SUBPROCESS));

  *pptr = NULL;

  if (ptr == NULL)
    return SCPE_MEM;

  if (pipe(pipe1) == -1 || pipe(pipe2) == -1) {
    free(ptr);
    return sim_messagef(SCPE_OPENERR, "Error creating pipe: %s\n", strerror(errno));
  }

  ptr->pid = fork();
  if (ptr->pid == -1) {
    free(ptr);
    return sim_messagef(SCPE_OPENERR, "Error starting process: %s\n", strerror(errno));
  }
  
  if (ptr->pid == 0) {
    dup2(pipe1[0], 0);
    dup2(pipe2[1], 1);
    dup2(open("/dev/null", O_RDONLY), 2);
    for (i = 3; i < 100; i++)
      close(i);
    i = system(command);
    (void)i;
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
  if (x == -1) {
    *n = 0;
    switch (errno) {
    case EAGAIN:
#if EAGAIN != EWOULDBLOCK
    case EWOULDBLOCK:
#endif
      return SCPE_OK;
    default:
      return SCPE_IOERR;
    }
  }
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

#include <process.h>

struct SUBPROCESS {
  HANDLE     hProcess;  /* The process handle. */
  HANDLE     hRead;     /* Handle for reading from the subprocess. */
  HANDLE     hWrite;    /* Handle for writing to the subprocess. */
};

t_stat sim_subprocess_start(SIM_SUBPROCESS **pptr, CONST char *command)
{
  SIM_SUBPROCESS *ptr = malloc(sizeof(SIM_SUBPROCESS));
  SECURITY_ATTRIBUTES saAttr; 
  HANDLE hRead, hWrite;

  *pptr = NULL;
  if (ptr == NULL)
    return SCPE_MEM;

  saAttr.nLength = sizeof saAttr; 
  saAttr.bInheritHandle = TRUE; 
  saAttr.lpSecurityDescriptor = NULL; 

  if (CreatePipe(&hRead, &ptr->hWrite, &saAttr, 0) == 0) {
    free(ptr);
    return SCPE_OPENERR;
  }

  if (CreatePipe(&ptr->hRead, &hWrite, &saAttr, 0) == 0) {
    CloseHandle(hRead);
    CloseHandle(ptr->hWrite);
    free(ptr);
    return SCPE_OPENERR;
  }

  SetHandleInformation(ptr->hRead, HANDLE_FLAG_INHERIT, 0);
  SetHandleInformation(ptr->hWrite, HANDLE_FLAG_INHERIT, 0);

  memset(&ProcessInfo, 0, sizeof ProcessInfo);
  memset(&StartupInfo, 0, sizeof StartupInfo);
  StartupInfo.cb = sizeof StartupInfo;
  StartupInfo.dwFlags = STARTF_USESTDHANDLES;
  StartupInfo.hStdInput = hRead;
  StartupInfo.hStdOutput = hWrite;
  StartupInfo.hStdError = INVALID_HANDLE_VALUE;

  if (CreateProcessA(NULL, command, NULL, NULL, FALSE, CREATE_NO_WINDOW,
                     NULL, NULL, &StartupInfo, &ProcessInfo) == 0) {
    CloseHandle(hRead);
    CloseHandle(hWrite);
    CloseHandle(ptr->hRead);
    CloseHandle(ptr->hWrite);
    free(ptr);
    return sim_messagef(SCPE_OPENERR, "Error starting process\n");
  }

  ptr->hProcess = ProcessInfo.hProcess;

  return SCPE_OK;
}

t_stat sim_subprocess_stop(SIM_SUBPROCESS *pptr)
{
  TerminateProcess(pptr->hProcess, 0);
  WaitForSingleObject(pptr->hProcess, INFINITE);
  CloseHandle(pptr->hProcess);
  CloseHandle(pptr->hRead);
  CloseHandle(pptr->hWrite);
  free(pptr);
  return SCPE_OK;
}

t_stat sim_subprocess_read(SIM_SUBPROCESS *pptr, void *cptr, size_t *n)
{
  DWORD m;
  *n = 0;
  if (ReadFile(pptr->hRead, cptr, n, &m, NULL) == 0)
    SCPE_IOERR;
  *n = n - m;
  return SCPE_OK;
}

t_stat sim_subprocess_write(SIM_SUBPROCESS *pptr, CONST void *cptr, size_t n)
{
  DWORD m;
  while (n > 0) {
    if (WriteFile(pptr->hWrite, cptr, n, &m, NULL) == 0)
      SCPE_IOERR;
    n -= m;
  }
  return SCPE_OK;
}

#endif /* _WIN32 */
