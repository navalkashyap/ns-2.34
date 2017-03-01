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
	totalSlotsShift = 0;
	dischargetime = config_.slotTime * frameLen;
	chargetime = 0;
	alloc_slot[0] = slot_Beacon;
	currentFrameSlots = maxSlots = frameLen;
	slotNum = 0;
	parentID = -2;
	startTimer.start(0);
}

MacAwsn::~MacAwsn() {
	// TODO Auto-generated destructor stub
}


void MacAwsn::runAtstartUp() {
	int ranNum = Random::random();
	randomTime = fmod(double(ranNum)/1000000, dischargetime);
	randomTime = 0;
	maxSlotShift = newnetif()->getSuperFrameTime(maxSlots);
	if(God::instance()->isSink(index_)) {
		myRole = PARENT;											// Parent node
		randomTime = dischargetime * 10;
		God::instance()->setMyParent(index_,index_); 		// Sink node to consider parent of itself
	} else {
		myRole = PARENT;
		randomTime = dischargetime * 1 * index_;
		God::instance()->setMyParent(index_,-2);
	}
	God::instance()->setRole(index_,0);						// Every node starts with empty MacBuffer and act as PARENT
	printf("node:%d, AWSNMac::AWSNMac: ranNum:%d, randomtime@%f, dischargetime:%f,chargetime:%f, myParent:%d, maxSlotShift:%d\n",
			index_,ranNum,randomTime, dischargetime,chargetime,God::instance()->getMyParent(index_),maxSlotShift);
	SFAwsnTimer.start(randomTime);

}

// Decide next SuperFrame here
void MacAwsn::SuperFrameHandler() {
	newnetif()->turnOffRadio();
	superFrameNum++;
	parentID = God::instance()->getMyParent(index_);
	setAllSlotsRx();		// Same slots in case of receiving
	if(myRole == PARENT) {
		scheduleChildren();
		alloc_slot[0] = slot_Beacon;
	} else if( parentID == -2) {
		alloc_slot[0] = slot_Beacon;
	}					// else listen to Parent in first slot
	chargetime = newnetif()->timeToFullCharge();
	chargeAwsnTimer.restart(chargetime);
	printf("node:%d, MacAwsn::SuperFrameHandler:%d, myRole:%d, energy:%f, isNodecharged:%d, chargetime:%f, parentID:%d, shiftFrameSlot:%d, totalSlotsShift:%d time@%f\n",
			index_,superFrameNum,myRole,newnetif()->getNodeEnergy(),newnetif()->isNodeCharged(),chargetime,parentID,shiftFrameSlot,totalSlotsShift,NOW);
}

void MacAwsn::scheduleChildren() {
	vector<int> newSchedule;
	int slot_per_child = 0;
	newSchedule.resize(maxSlots,-2);
	if(ChildTable.size() != 0) {
		vector<neighborNode>::iterator it = ChildTable.begin();
		slot_per_child = (maxSlots - 1)/ChildTable.size();
		int child_num = -1;
		for (int i = 1; i< maxSlots;) {
			if(child_num<ChildTable.size()-1) {
				child_num++;
			} else {
				child_num = 0;
			}
			for (int j = 0; (j< slot_per_child) & (i < maxSlots);j++,i++) {
				newSchedule[i] = ChildTable[child_num].id;
			}
		}
	}
	printf("node:%d, slot_per_child:%d, ChildTableSize:%d, My schedule as PARENT:- ",index_,slot_per_child,ChildTable.size());
	for (int i =0;i<newSchedule.size();i++)
		printf("%d:%d, ",i,newSchedule[i]);
	printf("\n");
	God::instance()->setSchedule(index_,newSchedule);
	return;
}

void MacAwsn::setAllSlotsRx() {
	for (int i = 1; i < currentFrameSlots; i++)
		alloc_slot[i] = slot_Rx;
	return;
}

void MacAwsn::ChargeHandler() {
	printf("node:%d, MacAwsn::ChargeHandler: chargetime:%f, time@%f\n",index_,chargetime,NOW);
	newnetif()->chargeNode(chargetime);
	if(myRole == CHILD)
		totalSlotsShift = totalSlotsShift + shiftFrameSlot;
	if(shiftFrameSlot > 0) {
		dischargetime = newnetif()->timeToCauseLead(shiftFrameSlot * config_.slotTime);
		shiftFrameSlot = 0;
		newnetif()->turnOnRadio();
		tx_active_ = 1;
		RadioOFFTimer.start(dischargetime);
	} else if(shiftFrameSlot < 0) {
		dischargetime = newnetif()->timeToCauseLag(-shiftFrameSlot * config_.slotTime);
		shiftFrameSlot = 0;
		newnetif()->turnOnRadio();
		tx_active_ = 1;
		RadioOFFTimer.start(dischargetime);
	} else {					// shiftFrameSlot == 0
		FAwsnTimer.start(0);
	}
}

void MacAwsn::RadioOFFHandler() {
	newnetif()->turnOffRadio();
	printf("node:%d, MacAwsn::RadioOFFHandler: dischargetime:%f, time@%f\n",index_,dischargetime,NOW);
	shiftFrameSlot = 0;
	tx_active_ = 0;
	SFAwsnTimer.start(0);
	return;
}

void MacAwsn::FrameHandler() {
	printf("node:%d, MacAwsn::FrameHandler: nodeEnergy:%f, time@%f\n",index_,newnetif()->getNodeEnergy(),NOW);
	currentFrameSlots = maxSlots;
	newnetif()->turnOnRadio();
	slotAwsnTimer.start(0);
}

void MacAwsn::SlotHandler() {
	printf("node:%d, MacAwsn::SlotHandler: slotNum:%d, slotType:%d, time@%f\n",
			index_,slotNum,alloc_slot[slotNum],NOW);
	switch(alloc_slot[slotNum]) {
	case slot_Beacon:
		sendBeacon(MAC_BROADCAST);
		break;
	case slot_Tx:
		sendData();
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
		postFrameHandler();
	}
	return;
}

void MacAwsn::postFrameHandler() {
	SFAwsnTimer.start(config_.slotTime);
	if(God::instance()->isSink(index_))
		return;
	if(myRole == PARENT && macQueue.size() > MAC_BufferSize * 0.8) {
		printf("node:%d, MacAwsn::postFrameHandler new Role as CHILD, macQueue.size:%d @time:%f\n",index_,macQueue.size(),NOW);
		myRole = CHILD;
		if(ParentTable.size()>0) {
			neighborNode myParentNode = getParentInfo();
			shiftFrameSlot = myParentNode.distSlots;
			totalSlotsShift = 0;
			God::instance()->setMyParent(index_,myParentNode.id);;
		}
	} else if(myRole == CHILD ) {
		if(macQueue.size() < MAC_BufferSize * 0.3) {
			printf("node:%d, MacAwsn::postFrameHandler new Role as Parent, macQueue.size:%d @time:%f\n",index_,macQueue.size(),NOW);
			myRole = PARENT;
			shiftFrameSlot = -(maxSlotShift + totalSlotsShift);
			totalSlotsShift = 0;
			God::instance()->setMyParent(index_,-2);;
		}
	}
	if(God::instance()->getMyParent(index_) == -2 && myRole == CHILD) {
		shiftFrameSlot = - maxSlots + 1;		// Linear search with minimum overlap slots
		printf("node:%d, MacAwsn:: No parent found after super frame, shifting SuperFrame by %d slots @time:%f\n",
				index_,shiftFrameSlot,NOW);
	}
	God::instance()->setRole(index_,myRole);
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
		printf("node:%d, MacAwsn::sendBeacon: ch->txtime@%f, time@%f\n",index_, ch->txtime(), NOW);
		downtarget_->recv(pktRRTS_->copy(), this);
	} else {
		printf("node:%d, MacAwsn::sendBeacon: Radio Busy:tx_state_:%d,rx_state_:%d, tx_active_:%d, time@%f\n",
				index_,tx_state_, rx_state_,tx_active_, NOW);
	}
}

void MacAwsn::sendData() {
	if (macQueue.empty()) {
		printf("node:%d, MacAwsn::sendData: no data to send when tempto send\n",index_);
		return;
	}
	else {
	  pktTx_ = macQueue.front();
	  macQueue.pop_front();
	}

	hdr_cmn* ch = HDR_CMN(pktTx_);
	ch->txtime() = (8.0 * (ch->size() + config_.header_len)) / config_.data_rate;

	char* mh = (char*)HDR_MAC(pktTx_);
	hdr_mactype(mh, MF_DATA);
	hdr_submactype(mh, MAC_RIL_DATA);
	printf("node:%d, MacAwsn::sendData: ch->txtime@%f,  time@%f\n",index_, ch->txtime(), NOW);
	downtarget_->recv(pktTx_->copy(), this);
}

void MacAwsn::recv(Packet *p, Handler *h) {
	struct hdr_cmn *hdr = HDR_CMN(p);
	char* mh = (char*)HDR_MAC(p);
	int src = hdr_src(mh);
	int dst = hdr_dst(mh);
	// If we are transmitting, then set the error bit in the packet
	if(tx_active_ && hdr->error() == 0) {
	hdr->error() = 1;
	}
	//printf("MacAwsn::recv: node:%d\n",index_);
	if(hdr->direction() == hdr->UP) {
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
		    	printf("node:%d, MacAwsn::recv: Successful, call handler in %f\n",index_,config_.slotTime/2);
			} else {
				// decide whether this packet's power is high enough to cause collision
			    if (pktRx_->txinfo_.RxPr / p->txinfo_.RxPr >= p->txinfo_.CPThresh) {
			    	// power too low, ignore the incoming packet
			    	printf("node:%d, MacAwsn::recv: power too low, ignore the incoming packet\n",index_);
			    	Packet::free(p);
			    } else {
					// power is high enough to cause collision
					rx_state_ = MAC_COLL;
			    	printf("node:%d, MacAwsn::recv: power is high enough to cause collision\n",index_);
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
	} else {    // hdr->DOWN
	    if (macQueue.size() < MAC_BufferSize) {
	    	macQueue.push_back(p);
	    	printf("node:%d, MacAwsn::recv: Msg in direction %d, dst node: %d, source_node: %d, adding to macBuffer:%d, time@%f\n",
	    							index_,hdr->direction(),dst,src,macQueue.size(),NOW);

	    } else {
	    	printf("node:%d, MacAwsn::recv: Msg in direction %d, dst node: %d, source_node: %d, Overflow as macBuffer:%d is full, time@%f\n",
	    		    							index_,hdr->direction(),dst,src,macQueue.size(),NOW);
	    	Packet::free(p);
	    }
	    h->handle((Event*) 0);
		rx_state_ = MAC_IDLE;
	}

}

// This method handles the packet received and change the state
void MacAwsn::recvHandler() {
	if (pktRx_==NULL || (rx_state_ != MAC_RECV && rx_state_ != MAC_COLL)) {
		printf("error_y!!! node %d wrong state in recvHandle\n",index_);
		exit(-1);
	}
	Packet* p = pktRx_;
	hdr_cmn *ch = HDR_CMN(p);
	char* mh = (char*)HDR_MAC(p);
	int dst = hdr_dst(mh);
	int src = hdr_src((char*)HDR_MAC(p));
	u_int16_t  type = hdr_mactype(mh,0);
	u_int16_t  subtype = hdr_submactype(mh,0);
	printf("node:%d, MacAwsn::recvHandler: src:%d, role:%d, tx_active_:%d, time@%f\n",index_,src,God::instance()->getRole(src),tx_active_,NOW);
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
		return;
	}
	if (ch->error()) {
		Packet::free(p);
		pktRx_ = 0;
		rx_state_ = MAC_IDLE;
	    printf("node:%d, MacAwsn::recvHandler--------: src:%d, time@%f\n",index_,src,NOW);
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
		return;
	}

	switch(type) {
		case MF_CONTROL:
			switch(subtype) {
				case MAC_RIL_ACK:
				//	recvACK(pktRx_);
				break;
				case MAC_RIL_RRTS:
					if(God::instance()->getRole(index_))
						if(God::instance()->getRole(src))
							break;		// neighbor node is acting as child so Beacon from other children will be dropped
					recvBeacon(pktRx_);
				break;
				case MAC_RIL_ONDEMAND:
				//	recvOndemand(pktRx_);
				break;
				default:
					fprintf(stderr,"Invalid MAC Control SubType %x\n", subtype);
					exit(1);
			}
		break;
		case MF_DATA:
			switch(subtype) {
				case MAC_RIL_DATA:
					recvData(pktRx_);
					break;
				default:
					fprintf(stderr, "Invalid MAC Data SubType %x\n", subtype);
					exit(1);
			}
			break;
			default:
				fprintf(stderr, "Invalid MAC Type %x\n", type);
				exit(1);
		}
	return;

}

void MacAwsn::recvData(Packet *p) {
	char* mh = (char*)HDR_MAC(p);
	int dst = hdr_dst(mh);
	int src = hdr_src(mh);
	last_alive_ = NOW;
	if (tx_state_ != MAC_IDLE || rx_state_ != MAC_RECV){
		printf("error_z!!! node %d wrong state in recvDATA \n", index_);
		exit(-1);
	}
	if (God::instance()->isSink(index_))
		macQueue.push_back(p);
	else if(macQueue.size() < MAC_BufferSize)
		macQueue.push_back(p);

	rx_state_ = MAC_IDLE;
	printf("node:%d, MacAwsn::recvData src:%d, dst:%d, macQueue_size:%d\n",index_,src,dst,macQueue.size());
}

// receive a receivers beacon (an ack to other nodes or a bcast rrts)
void MacAwsn::recvBeacon(Packet *p) {
	int src = hdr_src((char*)HDR_MAC(p));
	unsigned int dst = hdr_dst((char*)HDR_MAC(p));
//	unsigned int sendercw = hdr_backoff((char*)HDR_MAC(p),-1)/2;
	Packet::free(p); pktRx_ = 0; rx_state_ = MAC_IDLE;

#ifdef log_normal
	printf("node:%d, MacAwsn::recvBeacon: src:%d, src_slotNum:%d, on slotNum:%d, time@%f\n",
			index_,src,God::instance()->getNodeSlot(src),slotNum, NOW);
#endif

	updateNeighborInfo(src);
	if(God::instance()->getRole(src) == 0) {
		updateParentInfo(src);
	} else {
		updateChildInfo(src);
	}
	if(myRole == PARENT) {
		if(dst == MAC_BROADCAST)
			sendBeaconAck(src);
	} else {   // myRole == CHILD
		if(src == parentID) {  // Get the scheduling info
			vector<int> schedule = God::instance()->getSchedule(src);
			int myTxSlotsNum = 0;
			printf("node:%d, My schedule as CHILD:: ",index_);
			for (unsigned int i = 1; i < schedule.size(); i++) {
				if(index_ == schedule[i]) {
					alloc_slot[i] = slot_Tx;
					myTxSlotsNum++;
				} else {
					alloc_slot[i] = slot_Rx;
				}
				printf("Slot:%d:%d, ",i,alloc_slot[i]);
			}
			printf("\n");
			if(myTxSlotsNum < 1) {
				God::instance()->setMyParent(index_,-2);
				printf("node:%d, Reseting my parent:%d as it is not serving me",index_,parentID);
			}
		}
		if(God::instance()->getMyParent(index_) == -2) {	// Still looking for parent
			God::instance()->setMyParent(index_,src);
			int src_slot = God::instance()->getNodeSlot(src) - 1;
			if(src_slot>slotNum) {
				shiftFrameSlot = src_slot - 1;
				currentFrameSlots = maxSlots - shiftFrameSlot;
				printf("node:%d, shiftFrameSlot:%d, currentFrameSlots:%d, maxSlots:%d, totalSlotsShift:%d \n",index_,shiftFrameSlot,currentFrameSlots,maxSlots,totalSlotsShift);
			} else if(src_slot < slotNum) {
				shiftFrameSlot = -slotNum + 1;
				printf("node:%d, shiftFrameSlot:%d, currentFrameSlots:%d, maxSlots:%d, totalSlotsShift:%d \n",index_,shiftFrameSlot,currentFrameSlots,maxSlots,totalSlotsShift);
			} else {
				//start sending data Frames are aligned

			}
			//send Beacon Ack
			if(dst == MAC_BROADCAST)
				sendBeaconAck(src);
		}
	}
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

bool MacAwsn::updateNeighborInfo(int src) {
	vector<neighborNode>::iterator it = neighborList.begin();
	while (it!=neighborList.end()) {
		if (it->id == src) {
			return true;
		}
		it++;
	}
	neighborNode *r = new neighborNode(src);
	r->role = God::instance()->getRole(src);
	neighborList.push_back(*r);
	printf("node:%d, MacAwsn::updateNeighborInfo: tableSize:%d, src:%d\n",index_,neighborList.size(),src);
	return true;
}

bool MacAwsn::updateParentInfo(int src) {
	vector<neighborNode>::iterator it = ParentTable.begin();
	while (it!=ParentTable.end()) {
		if (it->id == src) {
			it->role = God::instance()->getRole(src);
			it->distSlots = totalSlotsShift;
			return true;
		}
		it++;
	}
	neighborNode *r = new neighborNode(src);
	r->role = God::instance()->getRole(src);
	r->distSlots = totalSlotsShift;
	ParentTable.push_back(*r);
	printf("node:%d, MacAwsn::updateParentInfo: tableSize:%d\n",index_,ParentTable.size());
	return true;
}

neighborNode MacAwsn::getParentInfo() {
	return ParentTable.front();
}

bool MacAwsn::updateChildInfo(int src) {
	vector<neighborNode>::iterator it = ChildTable.begin();
	while (it!=ChildTable.end()) {
		if (it->id == src) {
			return true;
		}
		it++;
	}
	neighborNode *r = new neighborNode(src);
	r->role = God::instance()->getRole(src);
	ChildTable.push_back(*r);
	printf("node:%d, MacAwsn::updateChildInfo: tableSize:%d\n",index_,ChildTable.size());
	return true;
}


// get data tx time
double MacAwsn::Txtime(Packet *p) {
  struct hdr_cmn *ch = HDR_CMN(p);
  double t = ch->txtime();
  if (t < 0.0)
  t = 0.0;
  return t;
}

/* Timers */
void MacAwsnTimer::start(double time) {
	Scheduler &s = Scheduler::instance();
	assert(busy_ == 0);
	busy_ = 1;
	//paused_ = 0;
	stime = s.clock();
	rtime = time;
	assert(rtime >= 0.0);
	s.schedule(this, &intr, rtime);
}

void MacAwsnTimer::stop() {
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
