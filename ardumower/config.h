// ------ choose ONLY ONE robot configuration ---------------------------------------------
#define USE_MOWER
//#define USE_SIM

// ----------------------------------------------------------------------------------------

#ifdef USE_MOWER
#include "mower.h"  // Ardumower Chassis Kit 1.0  (see mower.cpp for actual code)
#endif

#ifdef USE_SIM
#include "sim.h"  // simbad simulator - maybe future :)
#endif
