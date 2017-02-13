/*
 * AWSNMac.cc
 *
 *  Created on: Feb 1, 2017
 *      Author: navalkashyap
 */

#include "awsn-mac.h"
#include "mac.h"
#include "random.h"
#include "math.h"

#define log_normal

static class MacAWSNClass : public TclClass {
public:
	MacAWSNClass() : TclClass("Mac/Awsn") {}
	TclObject* create(int, const char*const*) {
		return (new MacAwsn());
	}
} class_mac_awsn;

MacAwsn::MacAwsn():Mac(),startTimer(this),SFAwsnTimer(this), chargeAwsnTimer(this), FAwsnTimer(this), slotAwsnTimer(this),
		recvTimer(this),sendTimer(this),RadioOFFTimer(this),superFrameNum(0)
{
	tx_state_ = rx_state_ = MAC_IDLE;
	tx_active_ = 0;
	sender_cw_ = cw_ = config_.cw_min;
	shiftFrameSlot = 0;
	dischargetime = config_.slotTime * frameLen;
	chargetime = 0;
	alloc_slot[0] = slot_Beacon;
	currentFrameSlots = maxSlots = frameLen;
	slotNum = 0;
	startTimer.start(0);
}

MacAwsn::~MacAwsn() {
	// TODO Auto-generated destructor stub
}

// set or get mac type, used for all messages
inline u_int16_t MacAwsn::hdr_mactype(char *hdr, u_int16_t type) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (type) dh->ftype_ = (MacFrameType)type;
  return dh->ftype_;
}

// set or get submac type, used for all messages
inline u_int16_t MacAwsn::hdr_submactype(char *hdr, u_int16_t type ) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (type) {dh->hdr_type_ &= 0x8000; dh->hdr_type_ |= (0x7fff&type);}
  return dh->hdr_type_&0x7fff;
}

// set or get backoff count, used only in ack messages
inline u_int16_t MacAwsn::hdr_backoff(char *hdr, int16_t value) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (value > -1) {
    dh->padding_ &= 0x0000ffff;
    u_int16_t tempvalue = (u_int16_t)value;
    dh->padding_ |= (tempvalue<<16UL);
  }
  return (dh->padding_&0xffff0000)>>16UL;
}

void MacAwsn::sendBeacon(int dst) {
	Packet *p = Packet::alloc();
	hdr_cmn* ch = HDR_CMN(p);
	ch->direction() = hdr_cmn::DOWN;
	ch->size() = config_.header_len;
	ch->uid() = 0;
	ch->ptype() = PT_MAC;
	ch->iface() = -2;
	ch->error() = 0;
	ch->txtime() = (8.0 * ch->size()) / config_.data_rate;


	char *mh = (char*)HDR_MAC(p);
	hdr_mac::access(p)->sstime() = 0;
	hdr_mactype(mh, MF_CONTROL);
	hdr_submactype(mh, MAC_RIL_RRTS);
	hdr_backoff(mh, sender_cw_);
	hdr_dst(mh, dst);
	hdr_src(mh, index_);

	pktRRTS_ = p;
	if (tx_state_ == MAC_IDLE && rx_state_ == MAC_IDLE && tx_active_ == 0) {
	//tx_state_ = MAC_RIMACBEACON;
	//tx_active_ = 1;
		printf("node:%d, MacAwsn::sendBeacon: ch->txtime@%f,  time@%f\n",index_, ch->txtime(), NOW);
		downtarget_->recv(pktRRTS_->copy(), this);
	} else {
		printf("node:%d, MacAwsn::sendBeacon: Radio Busy:tx_state_:%d,rx_state_:%d,  time@%f\n",index_,tx_state_, rx_state_, NOW);
}

}

// get data tx time
double MacAwsn::Txtime(Packet *p) {
  struct hdr_cmn *ch = HDR_CMN(p);
  double t = ch->txtime();
  if (t < 0.0)
  t = 0.0;
  return t;
}

void MacAwsn::recv(Packet *p, Handler *h) {
	struct hdr_cmn *hdr = HDR_CMN(p);
	char* mh = (char*)HDR_MAC(p);
	int src = hdr_src(mh);
	// If we are transmitting, then set the error bit in the packet
	if(tx_active_ && hdr->error() == 0) {
	hdr->error() = 1;
	}
	//printf("MacAwsn::recv: node:%d\n",index_);
	if(alloc_slot[slotNum] != slot_Tx && !newnetif()->getRadioStatus()) {
		printf("node:%d, MacAwsn::recv: Msg in direction %d, alloc_slot:%d, slotNum:%d, txtime@%f, source_node: %d, msg received, time@%f\n",
				index_,hdr->direction(),alloc_slot[slotNum],slotNum,txtime(p),src,NOW);
	} else {
		printf("node:%d, MacAwsn::recv: Msg in direction %d, alloc_slot:%d, slotNum:%d, source_node: %d, msg Dropped, time@%f\n",
				index_,hdr->direction(),alloc_slot[slotNum],slotNum,src,NOW);
		return;
	}
	if (rx_state_ == MAC_IDLE) {
	    rx_state_ = MAC_RECV;
	    pktRx_ = p;
		recvTimer.start(config_.slotTime/2);
	} else {
		// decide whether this packet's power is high enough to cause collision
	    if (pktRx_->txinfo_.RxPr / p->txinfo_.RxPr >= p->txinfo_.CPThresh) {
	    	// power too low, ignore the incoming packet
	    	Packet::free(p);
	    } else {
			// power is high enough to cause collision
			rx_state_ = MAC_COLL;
			// decide which packets to be left
			if (txtime(p) > recvTimer.expire()) {
				recvTimer.stop();
				Packet::free(pktRx_);
				pktRx_ = p;
				recvTimer.start(txtime(pktRx_)); // start recv timer to process
			} else {
				Packet::free(p);
			}
	    }
	}
}

// This method handles the packet received and change the state
void MacAwsn::recvHandler() {
	if (pktRx_==NULL || (rx_state_ != MAC_RECV && rx_state_ != MAC_COLL)) {
		printf("error_y!!! node %d wrong state in recvHandle\n",index_);
		exit(-1);
	}
	printf("node:%d, MacAwsn::recvHandler: time@%f\n",index_,NOW);
	Packet* p = pktRx_;
	hdr_cmn *ch = HDR_CMN(p);
	char* mh = (char*)HDR_MAC(p);
	int dst = hdr_dst(mh);
	u_int16_t  type = hdr_mactype(mh,0);
	u_int16_t  subtype = hdr_submactype(mh,0);
	if (tx_active_) {
	    Packet::free(p);
	    pktRx_ = 0;
	    rx_state_ = MAC_IDLE;
	    return;
	}
	if (rx_state_ == MAC_COLL) {
		Packet::free(p);
		pktRx_ = 0;
		rx_state_ = MAC_IDLE;
		// two successive receiption cause collision
//		double boundtime = collisionDetection(true);
		//checktoSendBeacon(boundtime);
		return;
	}
	if (ch->error()) {
		Packet::free(p);
		pktRx_ = 0;
		rx_state_ = MAC_IDLE;
		// error is caused by pre collision with tx or passed csthreshold
		// it should not affect tx or rx
//		double boundtime = collisionDetection(false);
		//checktoSendBeacon(boundtime);
		return;
	}

	if (dst != index_ && (MacFrameType)type == MF_CONTROL && subtype == MAC_RIL_ACK){
		subtype = MAC_RIL_RRTS;
		hdr_submactype((char*)HDR_MAC(p), subtype);
		dst = MAC_BROADCAST;
		hdr_dst((char*)HDR_MAC(p), dst);
	}
	if (dst != index_ && dst != BCAST_ADDR) {
		Packet::free(p);
		pktRx_ = 0;
		rx_state_ = MAC_IDLE;
//		double boundtime = collisionDetection(true);
		// checktoSendBeacon(boundtime);
		return;
	}

	switch(type) {
		case MF_CONTROL:
			switch(subtype) {
				case MAC_RIL_ACK:
				//	recvACK(pktRx_);
				break;
				case MAC_RIL_RRTS:
					recvBeacon(pktRx_);
				break;
				case MAC_RIL_ONDEMAND:
				//	recvOndemand(pktRx_);
				break;
				default:
					fprintf(stderr,"Invalid MAC Control Subtype %x\n", subtype);
					exit(1);
			}
		break;
		case MF_DATA:
			switch(subtype) {
				case MAC_RIL_DATA:
					//recvDATA(pktRx_);
					break;
				default:
					fprintf(stderr, "Invalid MAC Data Subtype %x\n", subtype);
					exit(1);
			}
			break;
			default:
				fprintf(stderr, "Invalid MAC Type %x\n", type);
				exit(1);
		}
	return;

}

// receive a receivers beacon (an ack to other nodes or a bcast rrts)
void MacAwsn::recvBeacon(Packet *p) {
	int src = hdr_src((char*)HDR_MAC(p));
	int dst = hdr_dst((char*)HDR_MAC(p));
	unsigned int sendercw = hdr_backoff((char*)HDR_MAC(p),-1)/2;
//	double backofftime = (Random::random()%sendercw+1)*config_.backoff_slot;
	Packet::free(p); pktRx_ = 0; rx_state_ = MAC_IDLE;

#ifdef log_normal
	printf("node:%d, MacAwsn::recvBeacon: src:%d, src_slotNum:%d, on slotNum:%d, time@%f\n",
			index_,src,God::instance()->getNodeSlot(src),slotNum, NOW);
#endif
	updateNeighborInfo(src);
	if(God::instance()->getMyParent(index_) == -2) {
		God::instance()->setMyParent(index_,src);
		int src_slot = God::instance()->getNodeSlot(src) - 1;
		if(src_slot>slotNum) {
			shiftFrameSlot = src_slot - 1;
			currentFrameSlots = maxSlots - shiftFrameSlot;
			printf("node:%d, shiftFrameSlot:%d, currentFrameSlots:%d, maxSlots:%d\n",index_,shiftFrameSlot,currentFrameSlots,maxSlots);
		} else if(src_slot < slotNum) {
			shiftFrameSlot = -slotNum + 1;
			printf("node:%d, shiftFrameSlot:%d, currentFrameSlots:%d, maxSlots:%d\n",index_,shiftFrameSlot,currentFrameSlots,maxSlots);
		} else {
			//start sending data Frames are aligned

		}
	}
	//send Beacon Ack
	if(dst == MAC_BROADCAST)
		sendBeaconAck(src);
	return;
}

void MacAwsn::sendBeaconAck(int src) {
	Packet *p = Packet::alloc();
	hdr_cmn* ch = HDR_CMN(p);
	ch->direction() = hdr_cmn::DOWN;
	ch->size() = config_.header_len;
	ch->uid() = 0;
	ch->ptype() = PT_MAC;
	ch->iface() = -2;
	ch->error() = 0;
	ch->txtime() = (8.0 * ch->size()) / config_.data_rate;


	char *mh = (char*)HDR_MAC(p);
	hdr_mac::access(p)->sstime() = 0;
	hdr_mactype(mh, MF_CONTROL);
	hdr_submactype(mh, MAC_RIL_RRTS);
	hdr_backoff(mh, sender_cw_);
	hdr_dst(mh, src);
	hdr_src(mh, index_);

	pktRRTS_ = p;
	if (tx_state_ == MAC_IDLE && rx_state_ == MAC_IDLE && tx_active_ == 0) {
	//tx_state_ = MAC_RIMACBEACON;
	//tx_active_ = 1;
		printf("node:%d, MacAwsn::sendBeaconAck: ch->txtime@%f,  time@%f\n",index_, ch->txtime(), NOW);
		downtarget_->recv(pktRRTS_->copy(), this);
	} else {
		printf("node:%d, MacAwsn::sendBeaconAck: Radio Busy:tx_state_:%d,rx_state_:%d,  time@%f\n",index_,tx_state_, rx_state_, NOW);
	}
}

void MacAwsn::findNeighbour() {
	for (int i = 1; i < currentFrameSlots; i++)
		alloc_slot[i] = slot_Rx;
	return;
}

bool MacAwsn::updateNeighborInfo(int src) {
vector<neighborEntry>::iterator it = neighborList.begin();
	while (it!=rxTable.end()) {
		if (it->id == src) {
			return true;
		}
		it++;
	}
	neighborEntry *r = new neighborEntry(src);
	neighborList.push_back(*r);
	return true;
}
void MacAwsn::runAtstartUp() {
	int ranNum = Random::random();
	randomTime = fmod(double(ranNum)/1000000, dischargetime);
	randomTime = 0;
	if(!God::instance()->isSink(index_)) {
		myRole = 1;
		God::instance()->setMyParent(index_,-2);

	} else {
		God::instance()->setMyParent(index_,index_); 		// Sink node to consider parent of itself
		randomTime = dischargetime * 2.6;
	}
	printf("node:%d, AWSNMac::AWSNMac: ranNum:%d, randomtime@%f, dischargetime:%f,chargetime:%f, myParent:%d\n",
			index_,ranNum,randomTime, dischargetime,chargetime,God::instance()->getMyParent(index_));
	SFAwsnTimer.start(randomTime);

}
void MacAwsn::SlotHandler() {
	printf("node:%d, MacAwsn::SlotHandler: slotNum:%d, slotType:%d, currentFrameSlots:%d, time@%f\n",
			index_,slotNum,alloc_slot[slotNum],currentFrameSlots,NOW);
	switch(alloc_slot[slotNum]) {
	case slot_Beacon:
		sendBeacon(MAC_BROADCAST);
		break;
	case slot_Tx:
		break;
	case slot_Rx:
		break;
	}

	slotNum++;
	slotNum = slotNum%currentFrameSlots;
	God::instance()->setMySlot(slotNum,index_);
	if(slotNum != 0)
		slotAwsnTimer.restart(config_.slotTime);
	else { // After all slots over
		if(God::instance()->getMyParent(index_) == -2) {
			shiftFrameSlot = - maxSlots + 1;		// Linear search with minimum overlap slots
			printf("node:%d, MacAwsn:: No parent found after super frame, shifting SuperFrame by %d slots\n",index_,shiftFrameSlot);
		}
		SFAwsnTimer.start(config_.slotTime);
	}
	return;
}

void MacAwsn::ChargeHandler() {
	printf("node:%d, MacAwsn::ChargeHandler: chargetime:%f, time@%f\n",index_,chargetime,NOW);
	newnetif()->chargeNode(chargetime);
	if(shiftFrameSlot > 0) {
		dischargetime = newnetif()->timeToCauseLead(shiftFrameSlot * config_.slotTime);
		shiftFrameSlot = 0;
		newnetif()->turnOnRadio();
		RadioOFFTimer.start(dischargetime);
	} else if(shiftFrameSlot < 0) {
		dischargetime = newnetif()->timeToCauseLag(-shiftFrameSlot * config_.slotTime);
		shiftFrameSlot = 0;
		newnetif()->turnOnRadio();
		RadioOFFTimer.start(dischargetime);
	} else {
		FAwsnTimer.start(0);
	}
}

void MacAwsn::RadioOFFHandler() {
	newnetif()->turnOffRadio();
	printf("node:%d, MacAwsn::RadioOFFHandler: time@%f\n",index_,NOW);
	shiftFrameSlot = 0;
	SFAwsnTimer.start(0);
	return;
}

void MacAwsn::FrameHandler() {
	printf("node:%d, MacAwsn::FrameHandler: nodeEnergy:%f, time@%f\n",index_,newnetif()->getNodeEnergy(),NOW);
	currentFrameSlots = maxSlots;
	newnetif()->turnOnRadio();
	slotAwsnTimer.start(0);
}

// Decide next SuperFrame here
void MacAwsn::SuperFrameHandler() {
	newnetif()->turnOffRadio();
	superFrameNum++;
	findNeighbour();
	chargetime = newnetif()->timeToFullCharge();
	chargeAwsnTimer.restart(chargetime);
	printf("node:%d, MacAwsn::SuperFrameHandler:%d energy:%f, isNodecharged:%d, chargetime:%f, shiftFrameSlot:%d time@%f\n",
			index_,superFrameNum,newnetif()->getNodeEnergy(),newnetif()->isNodeCharged(),chargetime,shiftFrameSlot,NOW);
}


/* Timers */
void MacAwsnTimer::start(double time)
{
	Scheduler &s = Scheduler::instance();
	assert(busy_ == 0);
	busy_ = 1;
	//paused_ = 0;
	stime = s.clock();
	rtime = time;
	assert(rtime >= 0.0);
	s.schedule(this, &intr, rtime);
}

void MacAwsnTimer::stop()
{
	Scheduler &s = Scheduler::instance();
	assert(busy_);
	s.cancel(&intr);
	busy_ = 0;
	stime = rtime = 0.0;
}

void MacAwsnTimer::restart(double time) {
  if (busy_)
    stop();
  start(time);
}

void MacAwsnTimer::forcestop() {
    if (busy_) {
        stop();
    }
}

/* Slot timer for TDMA scheduling. */
void SuperFrameAwsnTimer::handle(Event *e)
{
	busy_ = 0;
	stime = 0.0;
	rtime = 0.0;
	mac->SuperFrameHandler();
}

void FrameAwsnTimer::handle(Event *e)
{
	busy_ = 0;
	stime = 0.0;
	rtime = 0.0;
	mac->FrameHandler();
}

void ChargeAwsnTimer::handle(Event *e)
{
	busy_ = 0;
	stime = 0.0;
	rtime = 0.0;
	mac->ChargeHandler();
}

void SlotAwsnTimer::handle(Event *e)
{
	busy_ = 0;
	stime = 0.0;
	rtime = 0.0;
	mac->SlotHandler();
}

void RadioOffAwsnTimer::handle(Event *e)
{
	busy_ = 0;
	stime = 0.0;
	rtime = 0.0;
	mac->RadioOFFHandler();
}

void TxPktAwsnTimer::handle(Event *e)
{
	busy_ = 0;
	stime = 0.0;
	rtime = 0.0;

}

void RxPktAwsnTimer::handle(Event *e)
{
	busy_ = 0;
	stime = 0.0;
	rtime = 0.0;
	mac->recvHandler();
}

void StartAwsnTimer::handle(Event *e)
{
	busy_ = 0;
	stime = 0.0;
	rtime = 0.0;
	mac->runAtstartUp();
}
