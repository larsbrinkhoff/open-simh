/* imlac_xy.c: Imlac interface to XY display.

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

#include <math.h>
#include "imlac_defs.h"
#include "sim_video.h"

#define CHANNELS 4

#ifdef HAVE_LIBSDL
static SDL_AudioDeviceID dev = -1;
#endif
static int px = -1;
static int py = -1;
static int beam = 0;

void xy_init (void)
{
#ifdef HAVE_LIBSDL
  SDL_AudioSpec want, have;
  int i;

  if (dev != -1)
    return;

  for (i = 0; i < SDL_GetNumAudioDevices (0); i++)
    printf ("Device %d: %s\n", i, SDL_GetAudioDeviceName (i, 0));

  SDL_memset(&want, 0, sizeof want);
  want.freq = 44100;
  want.format = AUDIO_S16SYS;
  want.channels = CHANNELS;
  want.samples = 4096;
  want.callback = NULL;
  want.userdata = NULL;

  SDL_Init (SDL_INIT_AUDIO);

  dev = SDL_OpenAudioDevice(NULL, 0, &want, &have, SDL_AUDIO_ALLOW_FREQUENCY_CHANGE);
  if (dev == 0) {
    SDL_Log("Failed to open audio: %s", SDL_GetError());
    exit (1);
  } else {
    if (have.format != want.format)
      SDL_Log("We didn't get requested audio format.");
    if (have.freq != want.freq)
      SDL_Log("We didn't get requested audio frequency.");
    if (have.channels != want.channels)
      SDL_Log("We didn't get requested audio channels.");
    fprintf(stderr, "Format: %x\n", have.format);
    fprintf(stderr, "Frequency: %d\n", have.freq);
    fprintf(stderr, "Channels: %d\n", have.channels);
    fprintf(stderr, "Samples: %d\n", have.samples);
    fprintf(stderr, "Size: %d\n", have.size);
    SDL_PauseAudioDevice(dev, 0);
  }
#endif
}

void xy_xyz (int x, int y, int z)
{
#ifdef HAVE_LIBSDL
  int16_t data[CHANNELS];

  if (x < 0 || x > 65535 || y < 0 || x > 65535)
    return;

  while (SDL_GetQueuedAudioSize (dev) > 10*CHANNELS*2*44100)
    SDL_Delay (10);

  memset (data, 0, sizeof data);
  data[0] = x - 32768;
  data[1] = y - 32768;
  data[2] = z - 32768;

  SDL_QueueAudio (dev, data, sizeof data);
#endif
}

#define ABS(_X) ((_X) >= 0 ? (_X) : -(_X))

void xy_to (int x, int y)
{
  if (x == px && y == py && beam)
    return;
  xy_xyz (x, y, 65535);
  px = x;
  py = y;
  beam = 1;
}

void xy_blank (int x, int y)
{
  int i, n = 2;
  double dx, dy;

  if (x == px && y == py)
    goto done;
  if (!beam)
    goto done;

  dx = (double)(x - px) / n;
  dy = (double)(y - py) / n;
  for (i = 0; i <= n; i++)
    xy_xyz ((int)(px + i * dx + .499),
            (int)(py + i * dy + .499), 0);

 done:
  px = x;
  py = y;
  beam = 0;
}

void xy_point (int x, int y)
{
  xy_blank (x, y);
  xy_xyz (x, y, 65535);
  xy_xyz (x, y, 65535);
  px = x;
  py = y;
  beam = 1;
}

void xy_line(int x1, int y1, int x2, int y2)
{
  double x, y, dx, dy, r;
  int i, n = 3000;
  dx = x2 - x1;
  dy = y2 - y1;
  r = sqrt (dx * dx + dy * dy);
  dx /= r;
  dy /= r;

  xy_blank (x1, y1);

  if (r <= n) {
    xy_to (x1, y1);
    xy_to (x2, y2);
    x = x2;
    y = y2;
  } else {
    for (i = 0; i <= (int)(r + .499); i += n) {
      x = x1 + i * dx + .499;
      y = y1 + i * dy + .499;
      xy_to ((int)x, (int)y);
    }

    if (i - (int)(r + .499) > n / 4) {
      x = x2 + .499;
      y = y2 + .499;
      xy_to ((int)x, (int)y);
    }
  }

  px = (int)x;
  py = (int)y;
  beam = 1;
}

static void
xy_frame (void)
{
  xy_xyz (0, 0, 0);
  xy_xyz (0, 0, 0);
  xy_xyz (0, 65535, 0);
  xy_xyz (0, 65535, 0);
  xy_xyz (65535, 65535, 0);
  xy_xyz (65535, 65535, 0);
  xy_xyz (65535, 0, 0);
  xy_xyz (65535, 0, 0);
}

static int clear = 0;

void xy_idle (void)
{
#ifdef HAVE_LIBSDL
  Uint32 n = SDL_GetQueuedAudioSize (dev);
  int i;
  clear = n < CHANNELS*2*44100 / 30;
  if (clear) {
    n = CHANNELS*2*44100 / 20 - n;
    for (i = 0; i < 1000; i++)
      xy_frame ();
  }
#endif
}

void xy_clear (void)
{
#ifdef HAVE_LIBSDL
  if (clear)
    SDL_ClearQueuedAudio (dev);
  xy_frame ();
#endif
}
