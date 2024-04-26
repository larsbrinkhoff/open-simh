/* bcc500_defs.h: BCC 500 simulator definitions

   Copyright (c) 2024, Lars Brinkhoff

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

#ifndef BCC500_DEFS_H_
#define BCC500_DEFS_H_  0

#include "sim_defs.h"

#define STOP_HALT       1
#define STOP_IBKPT      2
#define STOP_ACCESS     3

typedef uint32 microword_t[3];

extern t_bool build_dev_tab (void);

extern REG cpu_reg[];
extern DEVICE cpu0_dev;
extern DEVICE cpu1_dev;
extern DEVICE msch_dev;
extern DEVICE chio_dev;
extern DEVICE amc_dev;
extern DEVICE amtu_dev;

#endif /* BCC500_DEFS_H_ */
