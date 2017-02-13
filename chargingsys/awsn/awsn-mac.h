/*
 * AWSNMac.h
 *
 *  Created on: Feb 1, 2017
 *      Author: navalkashyap
 */

#ifndef CHARGINGSYS_MAC_AWSN_MAC_H_
#define CHARGINGSYS_MAC_AWSN_MAC_H_

#define frameLen 					10
#define slot_Beacon 				0
#define slot_Tx						1
#define slot_Rx						2


#include <vector>
#include <list>
#include "mac.h"
#include <delay.h>
#include <connector.h>
#include <packet.h>
#include "god.h"
#include "wireless-chargingphy.h"

class MacAwsn;
enum MacFrameSubType {
  MAC_RIL_RRTS = 1,
  MAC_RIL_ACK = 2,
  MAC_RIL_RESPONSE = 3,
  MAC_RIL_DATA = 4,
  MAC_RIL_ONDEMAND = 5,
};

struct neighborEntry {
	int id;
	neighborEntry(int i):id(i){}
};

class MacAwsnTimer: public Handler {
public:
	MacAwsnTimer(MacAwsn* m) : mac(m) {
	  busy_ = 0;
	}
	virtual void handle(Event *e) = 0;
	virtual void restart(double time);
	virtual void start(double time);
	virtual void stop(void);
	virtual void forcestop(void);
	inline int busy(void) { return busy_; }
	inline double expire(void) {
		return ((stime + rtime) - Scheduler::instance().clock());
	}
protected:
	MacAwsn*	mac;
	int		    busy_;
	Event		intr;
	double		stime;
	double		rtime;
	double		slottime;
};

/* Timers to schedule transmitting and receiving. */
class StartAwsnTimer : public MacAwsnTimer {
public:
	StartAwsnTimer(MacAwsn *m) : MacAwsnTimer(m) {}
	void	handle(Event *e);
};

class SuperFrameAwsnTimer : public MacAwsnTimer {
public:
	SuperFrameAwsnTimer(MacAwsn *m) : MacAwsnTimer(m) {}
	void	handle(Event *e);
};

class FrameAwsnTimer : public MacAwsnTimer {
public:
	FrameAwsnTimer(MacAwsn *m) : MacAwsnTimer(m) {}
	void	handle(Event *e);
};

class SlotAwsnTimer : public MacAwsnTimer {
public:
	SlotAwsnTimer(MacAwsn *m) : MacAwsnTimer(m) {}
	void	handle(Event *e);
};

class ChargeAwsnTimer : public MacAwsnTimer {
public:
	ChargeAwsnTimer(MacAwsn *m) : MacAwsnTimer(m) {}
	void	handle(Event *e);
};

/* Timers to control packet sending and receiving time. */
class RxPktAwsnTimer : public MacAwsnTimer {
public:
	RxPktAwsnTimer(MacAwsn *m) : MacAwsnTimer(m) {}

	void	handle(Event *e);
};

class TxPktAwsnTimer : public MacAwsnTimer {
public:
	TxPktAwsnTimer(MacAwsn *m) : MacAwsnTimer(m) {}

	void	handle(Event *e);
};

class RadioOffAwsnTimer : public MacAwsnTimer {
public:
	RadioOffAwsnTimer(MacAwsn *m) : MacAwsnTimer(m) {}

	void	handle(Event *e);
};

class MacAwsn_config {
public:
	MacAwsn_config() {
		/* Configuration for Standard 802.15.4 MAC */
		data_rate = 250000; 			//	Bits/sec
		backoff_slot = 0.00032;
		CCA = 0.000128;
		cw_min = 7;
		cw_max = 127;
		header_len = 16;		//bytes
		ack_len = 16;
		slotTime = header_len*2*8/data_rate; //	0.001024 = ((16+16)*8/250000) One slot can carry 1 Beacon + 1 beaconAck
		payload_len = slotTime * data_rate;		// 16 bytes
	}
	double data_rate;
	double backoff_slot;
	double CCA;
	unsigned int cw_min;
	unsigned int cw_max;
	int header_len;                 // The length (in bytes) of MAC header
	int ack_len;                    // The length (in bytes) of ACK
	int payload_len;				// The length (in bytes) of MAC payload
	int superFrameLen;
	double slotTime;
	double P_slot;
	double P_frame;
	double P_superframe;

};


class MacAwsn : public Mac{
	friend class SuperFrameAwsnTimer;
public:
	MacAwsn();
	virtual ~MacAwsn();
	void recv(Packet *p, Handler *h);

	inline u_int16_t hdr_submactype(char *hdr, u_int16_t type);
	inline u_int16_t hdr_mactype(char *hdr, u_int16_t type);
	inline u_int16_t hdr_backoff(char *hdr, int16_t value);

	void runAtstartUp();
	//Handlers
	void SlotHandler(void);
	void FrameHandler();
	void SuperFrameHandler();
	void ChargeHandler();
	void recvHandler();
	void RadioOFFHandler();
	void recvBeacon(Packet *p);
	void sendBeaconAck(int src);

	void findNeighbour();
	bool updateNeighborInfo(int src);
	void sendBeacon(int dst);
	double Txtime(Packet *p);

private:
	MacAwsn_config 		config_;
	Packet*	        	pktRx_;         // recv packet
	Packet*	        	pktTx_;         // send packet (not from mac)
	Packet*         	pktRRTS_;       // mac beacon packet
	Packet*         	pktCTRL_;       // mac ack packet
	MacState        	rx_state_;      // incoming state (MAC_RECV or MAC_IDLE)
	MacState        	tx_state_;      // outgoing state
	int             	tx_active_;     // transmission active flag
	unsigned int    	cw_;			      // contention window

	std::vector<neighborEntry> 	neighborList;	// store neighbor'sinformation
	std::vector<neighborEntry> 	txTable;  // store sender's information
	std::vector<neighborEntry> 	rxTable;  // store receiver's information
	std::vector< Packet* > 		macqueue;  // mac buffer
	u_int16_t retryCount_;          // retry count for one data packet

	int 				alloc_slot[frameLen];
	int 				maxSlots;
	int					slotNum;
	int					currentFrameSlots;
	int 				shiftFrameSlot;
	bool 				myRole;
	StartAwsnTimer		startTimer;			// To start a node
	SuperFrameAwsnTimer SFAwsnTimer; 		// Charging + discharging cycle
	ChargeAwsnTimer		chargeAwsnTimer;	// Charge Timer
	FrameAwsnTimer 		FAwsnTimer; 		// Discharging cycle
	SlotAwsnTimer 		slotAwsnTimer;		// Activity mode Rx/Tx/Idle
	RxPktAwsnTimer		recvTimer;			// pkt receiption timer
	TxPktAwsnTimer 		sendTimer;      	// pkt transmission timer
	RadioOffAwsnTimer	RadioOFFTimer;		// timer to turn Radio OFF


	unsigned int sender_cw_;
	double chargetime;
	double dischargetime;
	double randomTime;
	int		superFrameNum;
	inline WirelessChargingPhy* newnetif(){return dynamic_cast<WirelessChargingPhy*>(netif_);}
};

#endif /* CHARGINGSYS_MAC_AWSN_MAC_H_ */