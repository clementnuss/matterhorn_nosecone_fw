#ifndef AIRBRAKES_LOOKUP_TABLE_H
#define AIRBRAKES_LOOKUP_TABLE_H

#include <Misc/Common.h>

//SHURIKEN LOOKUP TABLE
#ifdef SHURIKEN
#include <Airbrakes/shuriken/lookup_table_shuriken.h>
#endif

#ifdef FLAPPYFLAPPY
#include <Airbrakes/flappyflappy/lookup_table_flappy.h>
#endif /* AIRBRAKES_LOOKUP_TABLE_H */

#endif
