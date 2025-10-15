#include <stdio.h>
unsigned a, z, v;
static void munch(void)
{
  a = 0112;
  a += v;
  v = a;
  unsigned long long tmp = (a << 12) | z;
  tmp = ((tmp << 18) | (tmp >> 6)) & 077777777;
  a = tmp >> 12;
  z = tmp & 07777;
  a ^= v;
  printf("A=%04o, Z=%04o, V=%04o, X=%03o, Y=%03o\n",
         a, z, v, (a >> 3), (z >> 3));
}
int main(void)
{
  int i;
  a = z = v = 0;
  for(i = 0; i < 30; i++)
    munch();
  return 0;
}
