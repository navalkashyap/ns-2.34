/*
 * AWSNMac.h
 *
 *  Created on: Feb 1, 2017
 *      Author: navalkashyap
 */

#ifndef CHARGINGSYS_MAC_AWSN_MAC_H_
#define CHARGINGSYS_MAC_AWSN_MAC_H_

#define MAC_BufferSize				20
#define pktSendLatency				10
#define pktDropLatency				30
#define frameLen 					10
#define slot_Beacon 				0
#define slot_Tx						1
#define slot_Rx						2
#define slot_Ack					3
#define slot_RxIdle					4
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

class MacAwsn;
enum MacFrameSubType {
  MAC_RIL_RRTS = 1,
  MAC_RIL_ACK = 2,
  MAC_RIL_RESPONSE = 3,
  MAC_RIL_DATA = 4,
  MAC_RIL_ONDEMAND = 5,
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

class BeaconRxAwsnTimer : public MacAwsnTimer {
public:
	BeaconRxAwsnTimer(MacAwsn *m) : MacAwsnTimer(m) {}

	void	handle(Event *e);
};

class BeaconTxAwsnTimer : public MacAwsnTimer {
public:
	BeaconTxAwsnTimer(MacAwsn *m) : MacAwsnTimer(m) {}
	void	handle(Event *e);
};

class sendBeaconTxAwsnTimer : public MacAwsnTimer {
public:
	sendBeaconTxAwsnTimer(MacAwsn *m) : MacAwsnTimer(m) {}
	void	handle(Event *e);
};

class RadioOffAwsnTimer : public MacAwsnTimer {
public:
	RadioOffAwsnTimer(MacAwsn *m) : MacAwsnTimer(m) {}
	void	handle(Event *e);
};

class postFrameAwsnTimer : public MacAwsnTimer {
public:
	postFrameAwsnTimer(MacAwsn *m) : MacAwsnTimer(m) {}
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
	void postFrameHandler();
	void recvData(Packet *p);
	void recvAck(Packet *p);
	void recvBeacon(Packet *p);
	void sendBeaconAck(int src);

	void setAllSlotsRx();
	void scheduleChildren();
	bool updateNeighborInfo(int src);
	bool updateParentInfo(int src);
	neighborNode getParentInfo();
	bool updateChildInfo(int src);
	void sendBeacon(int dst);
	void BeaconSent();
	void sendData();
	void sendAck();
	void calcNextSFslots();
	double 	Txtime(Packet *p);

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
	char 	TxRx[5];
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


	int 				TotalDataSent;
	int					ofPackets;
	int 				alloc_slot[frameLen];
	int 				maxFrameSlots;
	int					slotNum;
	int					currentFrameSlots;
	double				runTimeShiftTime;
	double				mytotalSFTime;
	bool				runTimeShiftReq;
	int 				shiftFrameSlot;
	int					totalSlotsShift;
	int					myTotalSFslots;
	bool 				myRole;				// 0: Parent, 1: Child
	int 				parentID;
	bool				adjustSFslots;		// true: lag according to other node
	StartAwsnTimer		startTimer;			// To start a node
	SuperFrameAwsnTimer SFAwsnTimer; 		// Charging + discharging cycle
	ChargeAwsnTimer		chargeAwsnTimer;	// Charge Timer
	FrameAwsnTimer 		FAwsnTimer; 		// Discharging cycle
	SlotAwsnTimer 		slotAwsnTimer;		// Activity mode Rx/Tx/Idle
	RxPktAwsnTimer		recvTimer;			// pkt receiption timer
	TxPktAwsnTimer 		sendTimer;      	// pkt transmission timer
	RadioOffAwsnTimer	RadioOFFTimer;		// timer to turn Radio OFF
	BeaconTxAwsnTimer	BeaconTxTimer;		// Trigger the Timer once Beacon is tramitted
	BeaconRxAwsnTimer	BeaconRxTimer;		// Trigger when beacon reciever time is over
	sendBeaconTxAwsnTimer sendBeaconTimer;
	postFrameAwsnTimer	postFrameTimer;

	unsigned int sender_cw_;
	double chargeTime;
	double dischargeTime;
	double adjustmentTime;
	double randomTime;
	int		superFrameCount;
	inline WirelessChargingPhy* newnetif(){return dynamic_cast<WirelessChargingPhy*>(netif_);}
};

class AwsnNode {
protected:
	int Id;

public:
	AwsnNode() {}
	int getID() {return Id;}
};
#endif /* CHARGINGSYS_MAC_AWSN_MAC_H_ */
