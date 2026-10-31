/* linc_panel.h: LINC control console panel.

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

#ifndef LINC_PANEL_H_
#define LINC_PANEL_H_  0

#define PANEL_ON_OFF                 1  //    1 <--> 
#define PANEL_RUN_PAUSE              2  //    2 ---> 
#define   PANEL_STOPPED              0
#define   PANEL_RUNNING              1
#define   PANEL_PAUSED               2
#define PANEL_CYCLE                  3  //    3 ---> 
#define   PANEL_CYCLE_OFF            0
#define   PANEL_CYCLE_I              1
#define   PANEL_CYCLE_X              2
#define   PANEL_CYCLE_O              3
#define   PANEL_CYCLE_E              4
#define PANEL_INSTRUCTION            4  //   12 ---> 
#define PANEL_CLEAR                  5  //    1 <--> 
#define PANEL_MARK                   6  //    1 <--> 
#define PANEL_RELAYS                 7  //    6 ---> 
#define PANEL_AUTO_RESTART           8  //    1 <--> 
#define PANEL_ISTOP                  9  //    1 <--> 
#define PANEL_XOESTOP               10  //    1 <--> 
#define PANEL_INSTRUCTION_LOCATION  11  //   10 ---> 
#define PANEL_SENSE_SWITCHES        12  //    6 <--> 
#define PANEL_AUDIO                 13  //   14 <-->
//        Four fixed positions, plus continuous.
#define PANEL_DELAY                 14  //   14 <--> 
//        Two fixed positions, plus continuous.
#define PANEL_START_400             15  //    1 <--- 
#define PANEL_START_20              16  //    1 <--- 
#define PANEL_START_RS              17  //    1 <--- 
#define PANEL_LINK                  18  //   13 ---> 
#define PANEL_ACCUMULATOR           19  //   13 ---> 
#define PANEL_MEMORY_ADDRESS        20  //   12 ---> 
#define PANEL_MEMORY_CONTENTS       21  //   12 ---> 
#define PANEL_FILL_STEP             22  //    1 <--- 
#define PANEL_STEP_EXAM             23  //    1 <--- 
#define PANEL_FILL                  24  //    1 <--> 
#define PANEL_EXAM                  25  //    1 <--> 
#define PANEL_CYCLE_BY_CYCLE        26  //    1 <--> 
#define PANEL_INSTR_BY_INSTR        27  //    1 <--> 
#define PANEL_RESUME                28  //    1 <--- 
#define PANEL_STEP                  29  //    1 <--- 
#define PANEL_DO_TOG_INSTR          30  //    1 <--- 
#define PANEL_STOP                  31  //    1 <--- 
#define PANEL_LEFT_SWITCHES         32  //   12 <--> 
#define PANEL_RIGHT_SWITCHES        33  //   12 <--> 
#define PANEL_GONG_ON_OFF           34  //    1 <--> 
#define PANEL_SPEAKER_SELECT        35  //    2 <--> 
#define PANEL_SCOPE_KNOB_0          36  //    8 <--- 
#define PANEL_SCOPE_KNOB_1          37  //    8 <--- 
#define PANEL_SCOPE_KNOB_2          38  //    8 <--- 
#define PANEL_SCOPE_KNOB_3          39  //    8 <--- 
#define PANEL_SCOPE_KNOB_4          40  //    8 <--- 
#define PANEL_SCOPE_KNOB_5          41  //    8 <--- 
#define PANEL_SCOPE_KNOB_6          42  //    8 <--- 
#define PANEL_SCOPE_KNOB_7          43  //    8 <--- 
#define PANEL_ITEMS                 44

#endif /* LINC_PANEL_H_ */
