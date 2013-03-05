#include <stdarg.h>

#include "ch.h"

#define MAX_FILLER 11
#define FLOAT_PRECISION 100000

char *long_to_string_with_divisor(char *p, long num, unsigned radix, long divisor) {
  int i;
  char *q;
  long l, ll;

  l = num;
  if (divisor == 0) {
    ll = num;
  } else {
    ll = divisor;
  }

  q = p + MAX_FILLER;
  do {
    i = (int)(l % radix);
    i += '0';
    if (i > '9')
      i += 'A' - '0' - 10;
    *--q = i;
    l /= radix;
  } while ((ll /= radix) != 0);

  i = (int)(p + MAX_FILLER - q);
  do
    *p++ = *q++;
  while (--i);

  return p;
}

char *ltoa(char *p, long num, unsigned radix) {
  return long_to_string_with_divisor(p, num, radix, 0);
}

char *ftoa(char *p, double num) {
  long l;
  unsigned long precision = FLOAT_PRECISION;

  l = num;
  p = long_to_string_with_divisor(p, l, 10, 0);
  *p++ = '.';

  l = (num - l) * precision;
  return long_to_string_with_divisor(p, l, 10, precision / 10);
}

