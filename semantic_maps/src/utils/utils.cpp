#include "utils.h"

double getTime() {
  struct timeval tv;
  gettimeofday(&tv, 0);
  return tv2sec(tv);
}
