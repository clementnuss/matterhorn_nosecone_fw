/*
 * physical_iface.h
 *
 *  Created on: 25 Apr 2018
 *      Author: Clément Nussbaumer
 */

#ifndef MISC_PHYSICAL_IFACE_H_
#define MISC_PHYSICAL_IFACE_H_

#include <Misc/Common.h>

#ifdef CENTRALBODY

#define IGNITION_DURATION 1000

void TK_physical_iface(void const * argument);

void shortBip();

void longBip();

void triggerFirstEvent();

void triggerSecondEvent();

#endif

#endif /* MISC_PHYSICAL_IFACE_H_ */
