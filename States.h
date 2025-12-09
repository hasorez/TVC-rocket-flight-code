// States.h
#ifndef STATES_H
#define STATES_H

#include <Arduino.h>
#include "Globals.h"

// Main flight functions
void poweredAscentState(unsigned long currentMicros, float dt, Vector3 cachedGyro, Vector3 cachedAccel, float cachedPressure);
void descentState(unsigned long currentMicros, float dt, Vector3 cachedGyro, Vector3 cachedAccel, float cachedPressure);

// Transition checks
bool checkPrelaunchToPowered(Vector3 cachedAccel);
bool checkPoweredToApogee(float cachedPressure);
bool checkApogeeToDescent();

void chutesOut();

#endif
