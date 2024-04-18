#include <time.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <pigpio.h>

#define DA_X 0xB0
#define DA_Y 0x30

#define PIN_MOVE  17
#define PIN_DRAW  22
#define PIN_Z1    18
#define PIN_Z2    23

static int current_x = -1;
static int current_y = -1;
static int spi_fd;

static void fatal(const char *message)
{
  fprintf(stderr, "ABORT: %s\n", message);
  fprintf(stderr, "Error: %s\n", strerror(errno));
  exit(1);
}

static void delay_us(int us)
{
  struct timespec now, then;

  clock_gettime(CLOCK_MONOTONIC_RAW, &now);
  then = now;
  then.tv_nsec += 1000 * us;
  if (then.tv_nsec >= 1000000000) {
    then.tv_nsec -= 1000000000;
    then.tv_sec++;
  }

  do {
    clock_gettime(CLOCK_MONOTONIC_RAW, &now);
  } while (now.tv_sec <= then.tv_sec && now.tv_nsec < then.tv_nsec);
}

static void set_da(int which, int value)
{
  uint8_t tx[2];
  uint8_t rx[2];

  tx[0] = which;
  tx[0] |= (value >> 8) & 0xF;
  tx[1] = value & 0xFF;

  struct spi_ioc_transfer tr;
  memset(&tr, 0, sizeof tr);
  tr.tx_buf = (__u64)tx;
  tr.rx_buf = (__u64)rx;
  tr.len = sizeof tx;
  tr.delay_usecs = 20;

  if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) == -1)
    fatal("sending spi message");
}

static void move_to_point(int x, int y)
{
  if (current_x == x && current_y == y)
    return;

  set_da(DA_X, x);
  set_da(DA_Y, y);
  gpioWrite(PIN_MOVE, 1);
  delay_us(15);
  gpioWrite(PIN_MOVE, 0);

  current_x = x;
  current_y = y;
}

static int to_int(float x)
{
  int i = (4095.0/2) * (x + 1.0) + .499999;
  if (i < 0)
    return 0;
  if (i > 4095)
    return 4095;
  return i;
}

static void draw_segment(float sx, float sy)
{
  int dx = to_int(sx);
  int dy = to_int(sy);

  set_da(DA_X, dx);
  set_da(DA_Y, dy);
  gpioWrite(PIN_DRAW, 1);
  delay_us(50);
  gpioWrite(PIN_DRAW, 0);
}

void wwi_dot(int x, int y)
{
  move_to_point(x, y);
  draw_segment(0, 0);
}

void wwi_line(int x1, int y1, int x2, int y2)
{
  float dx = x2 - x1;
  float dy = y2 - y1;
  float sx, sy, ax, ay, mx;
  int i, n;

  sx = 8.0/4095.0 * dx;
  sy = 8.0/4095.0 * dy;
  ax = fabs(sx);
  ay = fabs(sy);

  if (ax <= 1.0 && ay <= 1.0) {
    move_to_point(x1, y1);
    draw_segment(sx, sy);
    current_x = x2;
    current_y = y2;
    return;
  }

  mx = ax > ay ? ax : ay;
  n = mx + .999999;
  sx /= n;
  sy /= n;

  move_to_point(x1, y1);
  for (i = 0; i < n; i++)
    draw_segment(sx, sy);

  current_x = x2;
  current_y = y2;
}

static void init_spi(void)
{
  uint8_t mode = SPI_MODE_0;
  uint8_t bits = 8;
  uint32_t speed = 4000000;

  spi_fd = open("/dev/spidev0.0", O_RDWR);
  if (spi_fd == -1)
    fatal("opening spi");

  if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) == -1)
    fatal("setting spi write mode");
  if (ioctl(spi_fd, SPI_IOC_RD_MODE, &mode) == -1)
    fatal("setting spi write mode");
  if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1)
    fatal("setting spi write word size");
  if (ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits) == -1)
    fatal("setting spi write word size");
  if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1)
    fatal("setting spi write speed");
  if (ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) == -1)
    fatal("setting spi write speed");
}

static void init_gpio(void)
{
  gpioInitialise();
  gpioSetMode(PIN_MOVE, PI_OUTPUT);
  gpioSetMode(PIN_DRAW, PI_OUTPUT);
  gpioSetMode(PIN_Z1, PI_OUTPUT);
  gpioSetMode(PIN_Z2, PI_OUTPUT);

  gpioWrite(PIN_Z1, 0);
  gpioWrite(PIN_Z2, 0);
}

static void die(void)
{
  fprintf(stderr, "Shutting down gracefully.\n");
  move_to_point(2048, 2048);
  close(spi_fd);
  gpioTerminate();
}

static void terminate(int sig)
{
  die();
}

void wwi_init(void)
{
  init_spi();
  init_gpio();
}
