/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*-  *
 *
 * Copyright (c) 1997 Regents of the University of California.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by the Computer Systems
 *	Engineering Group at Lawrence Berkeley Laboratory.
 * 4. Neither the name of the University nor of the Laboratory may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $Header: /cvsroot/nsnam/ns-2/mac/wireless-phy.h,v 1.15 2007/01/30 05:00:50 tom_henderson Exp $
 *
 * Ported from CMU/Monarch's code, nov'98 -Padma Haldar.
 *
 * wireless-phy.h
 * -- a SharedMedia network interface
 */

#ifndef ns_WirelessChargingPhy_h
#define ns_WirelessChargingPhy_h

#include "propagation.h"
#include "modulation.h"
#include "omni-antenna.h"
#include "phy.h"
#include "mobilenode.h"
#include "timer-handler.h"

class Phy;
class Propagation;
class WirelessChargingPhy;

class WirelessChargingPhy : public WirelessPhy {
public:
	WirelessChargingPhy();

	int 	sendUp(Packet *p);
	void 	sendDown(Packet *p);
	void 	turnOnRadio();
	void 	turnOffRadio();
	int 	getRadioStatus();
	double	timeToFullCharge() { return (em()->maxenergy() - em()->energy()) / P_charge;}		// return power needed for TX/RX
	void 	chargeNode(double chargeTime);
	double	timeToCauseLead(double shiftTime) { return (Pt_consume_ /(Pt_consume_ + P_charge))*shiftTime;}
	double	timeToCauseLag(double shiftTime) { return (P_charge /(Pt_consume_ + P_charge))*shiftTime;}
	bool 	isNodeCharged() { return em()->energy() >= em()->maxenergy();}
	double	getNodeEnergy() { return em()->energy();}
	double 	getResidualEnergy()	{ return em()->maxenergy() - em()->energy(); }
	double 	getmaxRadioOnTime() {return em()->maxenergy() / Pt_consume_;}
	double	getSuperFrameTime(double frameTime) {return frameTime * (1 + Pt_consume_/P_charge);}
	double 	getP_charge() {	return P_charge;}
	void	setP_charge(double newPcharge) { P_charge = newPcharge;}
	void 	printvalues() { 	printf("node:%d, WirelessChargingPhy::WirelessChargingPhy: P_charge:%f, Pt_consume_:%f\n",index_,P_charge,Pt_consume_);
}
	EnergyModel* em() { return node()->energy_model(); }
protected:
	enum 	RadioStatus { RADIOON, RADIOOFF };	
private:

	int 	radioStatus_;
	double 	last_radioOn_time_;
	double	last_charged_time_;
};

#endif /* !ns_WirelessPhy_h */





















