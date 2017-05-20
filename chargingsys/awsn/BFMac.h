/*
 * AWSNMac.h
 *
 *  Created on: Feb 1, 2017
 *      Author: navalkashyap
 */

#ifndef CHARGINGSYS_MAC_AWSN_MAC_H_
#define CHARGINGSYS_MAC_AWSN_MAC_H_

//#define MAC_BufferSize				10
#define pktSendLatency				10
#define pktDropLatency				30
#define frameLen 					10
#define slot_Beacon 				0
#define slot_Tx						1
#define slot_Rx						2
#define slot_Ack					3
#define slot_RxIdle					4
#define slot_RTS					5


#define EPSILON    (1.0E-8)
#define UNKNOWN						-2
#include <vector>
#include <list>
#include "mac.h"
#include <delay.h>
#include <connector.h>
#include <packet.h>
#include "god.h"
#include "wireless-chargingphy.h"

class MacBF;
enum MacFrameSubType {
  MAC_AWSN_Beacon = 1,
  MAC_AWSN_ACK = 2,
  MAC_AWSN_RTS = 3,
  MAC_AWSN_DATA = 4,
  MAC_AWSN_ONDEMAND = 5,
};

enum myRoleType {
	PARENT = 0,
	CHILD = 1,
};

struct neighborNode {
	int id;
	int role;
	int minHops;
	int	distSlots;
	int SFslots;
	double SFtime;
	neighborNode(int i):id(i),role(0),minHops(0),distSlots(-1),SFslots(0),SFtime(0){}
};

class MacBFTimer: public Handler {
public:
	MacBFTimer(MacBF* m) : mac(m) {
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
	MacBF*	mac;
	int		    busy_;
	Event		intr;
	double		stime;
	double		rtime;
	double		slottime;
};

/* Timers to schedule transmitting and receiving. */
class StartBFTimer : public MacBFTimer {
public:
	StartBFTimer(MacBF *m) : MacBFTimer(m) {}
	void	handle(Event *e);
};

class SuperFrameBFTimer : public MacBFTimer {
public:
	SuperFrameBFTimer(MacBF *m) : MacBFTimer(m) {}
	void	handle(Event *e);
};

class FrameBFTimer : public MacBFTimer {
public:
	FrameBFTimer(MacBF *m) : MacBFTimer(m) {}
	void	handle(Event *e);
};

class SlotBFTimer : public MacBFTimer {
public:
	SlotBFTimer(MacBF *m) : MacBFTimer(m) {}
	void	handle(Event *e);
};

class ChargeBFTimer : public MacBFTimer {
public:
	ChargeBFTimer(MacBF *m) : MacBFTimer(m) {}
	void	handle(Event *e);
};

/* Timers to control packet sending and receiving time. */
class RxPktBFTimer : public MacBFTimer {
public:
	RxPktBFTimer(MacBF *m) : MacBFTimer(m) {}

	void	handle(Event *e);
};

class TxPktBFTimer : public MacBFTimer {
public:
	TxPktBFTimer(MacBF *m) : MacBFTimer(m) {}

	void	handle(Event *e);
};

class BeaconRxBFTimer : public MacBFTimer {
public:
	BeaconRxBFTimer(MacBF *m) : MacBFTimer(m) {}

	void	handle(Event *e);
};

class BeaconTxBFTimer : public MacBFTimer {
public:
	BeaconTxBFTimer(MacBF *m) : MacBFTimer(m) {}
	void	handle(Event *e);
};

class sendBeaconTxBFTimer : public MacBFTimer {
public:
	sendBeaconTxBFTimer(MacBF *m) : MacBFTimer(m) {}
	void	handle(Event *e);
};

class sendRTSBFTimer : public MacBFTimer {
public:
	sendRTSBFTimer(MacBF *m) : MacBFTimer(m) {}
	void	handle(Event *e);
};

class RadioOffBFTimer : public MacBFTimer {
public:
	RadioOffBFTimer(MacBF *m) : MacBFTimer(m) {}
	void	handle(Event *e);
};

class postFrameBFTimer : public MacBFTimer {
public:
	postFrameBFTimer(MacBF *m) : MacBFTimer(m) {}
	void	handle(Event *e);
};


class MacBF_config {
public:
	MacBF_config() {
		/* Configuration for Standard 802.15.4 MAC */
		data_rate = 250000; 			//	Bits/sec
		backoff_slot = 0.00032;
		CCA = 0.000128;
		cw_min = 7;
		cw_max = 127;
		header_len = 16;		//bytes
		ack_len = 16;
		slotTime = 0.002400;//header_len*2*8/data_rate; //	0.001024 = ((16+16)*8/250000) One slot can carry 1 Beacon + 1 beaconAck
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


class MacBF : public Mac{
	friend class SuperFrameBFTimer;
public:
	MacBF();
	virtual ~MacBF();
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
	void postFrameHandler();
	void recvData(Packet *p);
	void recvAck(Packet *p);
	void recvBeacon(Packet *p);
	void recvRTS(Packet *p);

	void setAllSlots();
	void scheduleChildren();
	bool updateNeighborInfo(int src);
	bool updateParentInfo(int src);
	neighborNode getParentInfo();
	bool updateChildInfo(int src);
	void sendBeacon(int dst);
	void BeaconSent();
	void sendData();
	void sendAck();
	void sendRTS();
	void calcNextSFslots();
	double 	Txtime(Packet *p);

private:
	MacBF_config 		config_;
	Packet*	        	pktRx_;         // recv packet
	Packet*	        	pktTx_;         // send packet (not from mac)
	Packet*         	pktRRTS_;       // mac beacon packet
	Packet*         	pktCTRL_;       // mac ack packet
	MacState        	rx_state_;      // incoming state (MAC_RECV or MAC_IDLE)
	MacState        	tx_state_;      // outgoing state
	int             	tx_active_;     // transmission active flag
	unsigned int    	cw_;			      // contention window
	char 	TxRx[6];
	char	Role[2];
	char	dir[3];
	std::vector<neighborNode> 	neighborList;	// store neighbor'sinformation
	std::vector<neighborNode> 	ParentTable;  	// store sender's information
	std::vector<neighborNode> 	ChildTable;  	// store receiver's information
	std::list< Packet* > 		macQueue;  		// mac buffer
	std::list<double>			pktTimeStamp;	// Maintain pkt arrival time
	std::vector<Packet*>        macQueueTx;		// Temporary buffer during Tx, waiting for Ack
	u_int16_t retryCount_;          // retry count for one data packet
	double last_alive_;             // last time a data message is received

	int 				MAC_BufferSize;
	int 				TotalDataSent;
	int					ofPackets;
	int 				alloc_slot[frameLen];
	int 				alloc_slot_prev[frameLen];
	int 				maxFrameSlots;
	int					slotNum;
	int					maxParentCycle;
	int					parentCycleNum;
	int					currentFrameSlots;
	double				runTimeShift;
	double				mySFTime;
	bool				runTimeShiftReq;
	int 				shiftFrameSlot;
	int					totalSlotsShift;
	int					myTotalSFslots;
	bool 				myRole;				// 0: Parent, 1: Child
	int 				parentID;
	bool				adjustSFslots;		// true: lag according to other node
	StartBFTimer		startTimer;			// To start a node
	SuperFrameBFTimer 	SFBFTimer; 		// Charging + discharging cycle
	ChargeBFTimer		chargeBFTimer;	// Charge Timer
	FrameBFTimer 		FBFTimer; 		// Discharging cycle
	SlotBFTimer 		slotBFTimer;		// Activity mode Rx/Tx/Idle
	RxPktBFTimer		recvTimer;			// pkt receiption timer
	TxPktBFTimer 		sendTimer;      	// pkt transmission timer
	RadioOffBFTimer		RadioOFFTimer;		// timer to turn Radio OFF
	BeaconTxBFTimer		BeaconTxTimer;		// Trigger the Timer once Beacon is tramitted
	BeaconRxBFTimer		BeaconRxTimer;		// Trigger when beacon reciever time is over
	sendBeaconTxBFTimer sendBeaconTimer;
	postFrameBFTimer	postFrameTimer;
	sendRTSBFTimer 		sendRTSTimer;
	unsigned int sender_cw_;
	double 	chargeTime;
	double 	dischargeTime;
	double 	adjustmentTime;
	double 	randomTime;
	int		superFrameCount;
	bool	adjustSF;
	bool 	adjustF;
	bool 	checkForAck;
	bool 	sendRTSFlag;
	inline WirelessChargingPhy* newnetif(){return dynamic_cast<WirelessChargingPhy*>(netif_);}
};

class BFNode {
protected:
	int Id;

public:
	BFNode() {}
	int getID() {return Id;}
};
#endif /* CHARGINGSYS_MAC_AWSN_MAC_H_ */
