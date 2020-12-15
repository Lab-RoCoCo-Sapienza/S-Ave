#pragma once

#include <sys/time.h>

double getTime();

inline double tv2sec(const struct timeval& tv_) {return (static_cast<double>(tv_.tv_sec)+1e-6*static_cast<double>(tv_.tv_usec));}
