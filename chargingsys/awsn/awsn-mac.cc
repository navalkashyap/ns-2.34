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
		recvTimer(this),sendTimer(this),RadioOFFTimer(this),BeaconTxTimer(this),BeaconRxTimer(this),sendBeaconTimer(this),
		postFrameTimer(this),superFrameCount(0)
{
	tx_state_ = rx_state_ = MAC_IDLE;
	tx_active_ = 0;
	sender_cw_ = cw_ = config_.cw_min;
	shiftFrameSlot = 0;
	totalSlotsShift = 0;
	dischargeTime = config_.slotTime * frameLen;
	chargeTime = 0;
	alloc_slot[0] = slot_Beacon;
	currentFrameSlots = maxFrameSlots = frameLen;
	slotNum = 0;
	parentID = -2;
	adjustmentTime = 0;
	startTimer.start(0);
	TotalDataSent = 0;
	runTimeShiftReq = false;
	TxRx[0] = 'B'; TxRx[1] = 'T'; TxRx[2] = 'R'; TxRx[3] = 'A';
}

MacAwsn::~MacAwsn() {
	// TODO Auto-generated destructor stub
}


void MacAwsn::runAtstartUp() {
	int ranNum = Random::random();
	randomTime = fmod(double(ranNum)/1000000, dischargeTime);
	randomTime = 0;
	if(God::instance()->isSink(index_)) {
		myRole = PARENT;											// Parent node
		randomTime = dischargeTime * 10;
		parentID = index_; 		// Sink node to consider parent of itself
	} else {
		myRole = PARENT;
		randomTime = dischargeTime * 1 * index_;
		parentID = -2;
	}
	newnetif()->setP_charge(God::instance()->getP_charge(index_));
	mytotalSFTime = newnetif()->getSuperFrameTime(maxFrameSlots * config_.slotTime);
	myTotalSFslots = mytotalSFTime / config_.slotTime;
	God::instance()->setMyParent(index_,parentID);
	God::instance()->setTotalSFslots(index_,myTotalSFslots);
	God::instance()->setTotalSFTime(index_,mytotalSFTime);
	God::instance()->setRole(index_,0);						// Every node starts with empty MacBuffer and act as PARENT
	printf("node:%d, AWSNMac::AWSNMac: ranNum:%d, randomtime@%f, dischargeTime:%f,chargeTime:%f, myParent:%d, myTotalSFslots:%d, mytotalSFTime:%f\n",
			index_,ranNum,randomTime, dischargeTime,chargeTime,God::instance()->getMyParent(index_),
			myTotalSFslots,mytotalSFTime);
	SFAwsnTimer.start(randomTime);
	newnetif()->printvalues();

}

// Decide next SuperFrame here
void MacAwsn::SuperFrameHandler() {
	superFrameCount++;
	parentID = God::instance()->getMyParent(index_);
	setAllSlotsRx();		// Same slots in case of receiving
	if(myRole == PARENT) {
		scheduleChildren();
		alloc_slot[0] = slot_Beacon;
		alloc_slot[maxFrameSlots -1] = slot_Ack;
	} else if(parentID == -2) {
		alloc_slot[0] = slot_Beacon;
	}					// else listen to Parent in first slot
	chargeTime = newnetif()->timeToFullCharge();
	chargeAwsnTimer.restart(chargeTime);
	printf("node:%d, MacAwsn::SuperFrameHandler:%d, myRole:%d, energy:%f, chargeTime:%f, parentID:%d, shiftFrameSlot:%d, totalSlotsShift:%d time@%f\n",
			index_,superFrameCount,myRole,newnetif()->getNodeEnergy(),chargeTime,parentID,shiftFrameSlot,totalSlotsShift,NOW);
}

void MacAwsn::scheduleChildren() {
	vector<int> newSchedule;
	vector<neighborNode>::iterator it = ChildTable.begin();
	std::vector<neighborNode> 	ActiveChildTable;
	int slot_per_child = 0;
	newSchedule.resize(maxFrameSlots,-2);
	newSchedule[0] = newSchedule[maxFrameSlots-1] = index_;
	while (it!=ChildTable.end()) {
		if (God::instance()->getRole(it->id) == CHILD) {
			ActiveChildTable.push_back(*it);
		}
		it++;
	}
	if(ActiveChildTable.size() != 0) {
		slot_per_child = (maxFrameSlots - 2)/ActiveChildTable.size();
		int child_num = -1;
		for (int i = 1; i< maxFrameSlots-1;) {
			if(child_num<ActiveChildTable.size()-1) {
				child_num++;
			} else {
				child_num = 0;
			}
			for (int j = 0; (j< slot_per_child) & (i < maxFrameSlots-1);j++,i++) {
				newSchedule[i] = ActiveChildTable[child_num].id;
			}
		}
	}
	printf("node:%d, slot_per_child:%d, ChildTableSize:%d, ActiveChildTable:%d\n"
		   "node:%d, My schedule as PARENT:- ",index_,slot_per_child,ChildTable.size(),ActiveChildTable.size(),index_);
	for (int i =0;i<newSchedule.size();i++)
		printf("%d:%d, ",i,newSchedule[i]);
	printf("\n");
	for (int i=0;i<frameLen;i++)
		God::instance()->setSchedule(index_,i,newSchedule[i]);
	return;
}

void MacAwsn::setAllSlotsRx() {
	for (int i = 1; i < currentFrameSlots; i++)
		alloc_slot[i] = slot_Rx;
	return;
}

void MacAwsn::ChargeHandler() {
	printf("node:%d, MacAwsn::ChargeHandler: chargeTime:%f, time@%f\n",index_,chargeTime,NOW);
	newnetif()->chargeNode(chargeTime);
	if(runTimeShiftReq) {
		dischargeTime = newnetif()->timeToCauseLag(runTimeShiftTime);
		newnetif()->turnOnRadio();
		printf("node:%d, MacAwsn::ChargeHandler: runTimeShiftTime:%f, dischargeTime:%f, time@%f\n",
				index_,runTimeShiftTime,dischargeTime,NOW);
		tx_active_ = 1;
		rx_state_ = MAC_MGMT;
		runTimeShiftReq = false;
		RadioOFFTimer.start(dischargeTime);
		return;
	}
	if(myRole == CHILD) {
		totalSlotsShift = (totalSlotsShift + shiftFrameSlot) % myTotalSFslots;
	}
	if(shiftFrameSlot > 0) {
		dischargeTime = newnetif()->timeToCauseLead(shiftFrameSlot * config_.slotTime);
		shiftFrameSlot = 0;
		newnetif()->turnOnRadio();
		tx_active_ = 1;
		rx_state_ = MAC_MGMT;
		RadioOFFTimer.start(dischargeTime);
	} else if(shiftFrameSlot < 0) {
		dischargeTime = newnetif()->timeToCauseLag(-shiftFrameSlot * config_.slotTime);
		shiftFrameSlot = 0;
		newnetif()->turnOnRadio();
		tx_active_ = 1;
		rx_state_ = MAC_MGMT;
		RadioOFFTimer.start(dischargeTime);
	} else {					// shiftFrameSlot == 0
		FAwsnTimer.start(0);
	}
}

void MacAwsn::RadioOFFHandler() {
	newnetif()->turnOffRadio();
	rx_state_ = MAC_IDLE;
	shiftFrameSlot = 0;
	tx_active_ = 0;
	chargeTime = newnetif()->timeToFullCharge();
	chargeAwsnTimer.start(chargeTime);
	printf("node:%d, MacAwsn::RadioOFFHandler: dischargeTime:%f, chargeTime:%f, time@%f\n",index_,dischargeTime,chargeTime,NOW);
	return;
}

void MacAwsn::FrameHandler() {
	printf("node:%d, MacAwsn::FrameHandler: time@%f\n",index_,NOW);
	currentFrameSlots = maxFrameSlots;
	newnetif()->turnOnRadio();
	slotAwsnTimer.start(0);
}

void MacAwsn::SlotHandler() {
	printf("node:%d, MacAwsn::SlotHandler: slotNum:%d, slotType:%c, time@%f\n",
			index_,slotNum,TxRx[alloc_slot[slotNum]],NOW);
	switch(alloc_slot[slotNum]) {
	case slot_Beacon:
		if(myRole == PARENT) {
			sendBeaconTimer.start(0.00001);
		}
		break;
	case slot_Tx:
		sendData();
		break;
	case slot_Rx:
		break;
	case slot_Ack:
		if(myRole == PARENT) {
			sendAck();
		}
		break;
	}

	slotNum++;
//	slotNum = slotNum%currentFrameSlots;
//	God::instance()->setMySlot(slotNum,index_);
	if(slotNum != currentFrameSlots) {
		slotAwsnTimer.restart(config_.slotTime);
	} else { // After all slots over
		slotNum = 0;
		postFrameTimer.restart(config_.slotTime);
//		postFrameHandler();
	}
	God::instance()->setMySlot(slotNum,index_);
	return;
}

void MacAwsn::postFrameHandler() {
	SFAwsnTimer.restart(0);
	newnetif()->turnOffRadio();
	printf("node:%d, MacAwsn::postFrameHandler: macQueue.size:%d @time:%f\n",index_,macQueue.size(),NOW);
	if(fabs(runTimeShiftTime - 0.00000) > EPSILON) {
		runTimeShiftReq = true;
	}
	if(God::instance()->isSink(index_))
		return;
	if(myRole == PARENT) {
		if(macQueue.size() > MAC_BufferSize * 0.8) {
			myRole = CHILD;
			if(ParentTable.size()>0) {
				neighborNode myParentNode = getParentInfo();
				shiftFrameSlot = myParentNode.distSlots;
				totalSlotsShift = 0;
				God::instance()->setMyParent(index_,myParentNode.id);
			}
			printf("node:%d, MacAwsn::postFrameHandler new Role as CHILD, macQueue.size:%d @time:%f\n",index_,macQueue.size(),NOW);
		}
	} else if(myRole == CHILD ) {
		if(macQueue.size() < MAC_BufferSize * 0.3) {
			myRole = PARENT;
			shiftFrameSlot = -(myTotalSFslots + totalSlotsShift + 1);
			totalSlotsShift = 0;
			God::instance()->setMyParent(index_,-2);
			printf("node:%d, MacAwsn::postFrameHandler new Role as Parent, macQueue.size:%d, shiftFrameSlot:%d @time:%f\n",
					index_,macQueue.size(),shiftFrameSlot,NOW);
		}
	}
	if(God::instance()->getMyParent(index_) == -2 && myRole == CHILD) {
		shiftFrameSlot = - maxFrameSlots + 1;		// Linear search with minimum overlap slots
		printf("node:%d, MacAwsn:: No parent found after super frame, shifting SuperFrame by %d slots @time:%f\n",
				index_,shiftFrameSlot,NOW);
	}
	God::instance()->setRole(index_,myRole);
}

inline u_int16_t MacAwsn::hdr_mactype(char *hdr, u_int16_t type) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (type) dh->ftype_ = (MacFrameType)type;
  return dh->ftype_;
}

// set or get mac type, used for all messages
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
	tx_state_ = MAC_RIMACBEACON;
	tx_active_ = 1;
		printf("node:%d, MacAwsn::sendBeacon: ch->txtime@%f, time@%f\n",index_, ch->txtime(), NOW);
		downtarget_->recv(pktRRTS_->copy(), this);
	} else {
		printf("node:%d, MacAwsn::sendBeacon: Radio Busy:tx_state_:%d,rx_state_:%d, tx_active_:%d, time@%f\n",
				index_,tx_state_, rx_state_,tx_active_, NOW);
	}
	BeaconTxTimer.restart(ch->txtime());
	calcNextSFslots();

}

void MacAwsn::BeaconSent() {
	tx_state_ = MAC_IDLE;
	tx_active_ = 0;
	printf("node:%d, MacAwsn::BeaconSent: tx_active_:%d @%f\n",index_,tx_active_,NOW);
//	rx_state_ = MAC_IDLE;
}

void MacAwsn::sendData() {
	if (macQueue.empty()) {
		printf("node:%d, MacAwsn::sendData: no data to send when tempto send\n",index_);
		return;
	} else {
		macQueueTx.push_back(macQueue.front());
		macQueue.pop_front();
		pktTx_ = macQueueTx.back();
	}

	hdr_cmn* ch = HDR_CMN(pktTx_);
	ch->txtime() = (8.0 * (ch->size() + config_.header_len)) / config_.data_rate;
//	ch->direction() = hdr_cmn::DOWN;
	char* mh = (char*)HDR_MAC(pktTx_);
	hdr_mactype(mh, MF_DATA);
	hdr_submactype(mh, MAC_RIL_DATA);
//	hdr_src(mh, index_);
//	hdr_dst(mh, parentID);

	printf("node:%d, MacAwsn::sendData: ch->txtime@%f, time@%f\n",index_, ch->txtime(), NOW);
	downtarget_->recv(pktTx_->copy(), this);
}

/*
 * 	1. Send Ack for successful data Rx
 * 	2. send vital information about next SuperFrame like number of slots of next SF so that children can adjust themselves
 * 	3. Availability for next SF in case Parent is switching to child mode
 */

void MacAwsn::sendAck() {
	Packet *p = Packet::alloc();
	hdr_cmn* ch = HDR_CMN(p);
	int dst = MAC_BROADCAST;
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
	hdr_submactype(mh, MAC_RIL_ACK);
	hdr_backoff(mh, sender_cw_);
	hdr_dst(mh, dst);
	hdr_src(mh, index_);
	if (tx_state_ == MAC_IDLE && rx_state_ == MAC_IDLE && tx_active_ == 0) {
		printf("node:%d, MacAwsn::sendAck: ch->txtime@%f, time@%f\n",index_, ch->txtime(), NOW);
		downtarget_->recv(p->copy(), this);
	} else {
		printf("node:%d, MacAwsn::sendAck: Radio Busy:tx_state_:%d,rx_state_:%d, tx_active_:%d, time@%f\n",
				index_,tx_state_, rx_state_,tx_active_, NOW);
	}
	calcNextSFslots();
}

void MacAwsn::calcNextSFslots() {
	vector<neighborNode>::iterator it = ChildTable.begin();
	int maxSFslots = myTotalSFslots; 			// Assuming no all nodes have same P_charge
	double maxSFTime = mytotalSFTime;
	while (it!=ChildTable.end()) {
		if(it->SFslots > maxSFslots) {
			maxSFslots = it->SFslots;
			maxSFTime = it->SFtime;
		}
		it++;
	}
	God::instance()->setmaxSFslots(index_,maxSFslots);
	God::instance()->setmaxSFTime(index_,maxSFTime);
}

void MacAwsn::recv(Packet *p, Handler *h) {
	struct hdr_cmn *hdr = HDR_CMN(p);
	char* mh = (char*)HDR_MAC(p);
	int src = hdr_src(mh);
	int dst = hdr_dst(mh);
	// If we are transmitting, then set the error bit in the packet
	if(tx_active_ && hdr->error() == 0) {
		printf("node:%d, MacAwsn::recv, tx_active_:%d, hdr->error:%d\n",index_,tx_active_, hdr->error());
		hdr->error() = 1;
	}
	if(hdr->direction() == hdr->UP) {
		if(alloc_slot[slotNum] != slot_Tx && !newnetif()->getRadioStatus()) {
			printf("node:%d, MacAwsn::recv: Msg in direction %d, alloc_slot:%d, slotNum:%d, txtime@%f, source_node: %d, msg received, time@%f\n",
					index_,hdr->direction(),alloc_slot[slotNum],slotNum,hdr->txtime(),src,NOW);
		} else {
			printf("node:%d, MacAwsn::recv: Msg in direction %d, alloc_slot:%d, slotNum:%d, source_node: %d, msg Dropped, time@%f\n",
					index_,hdr->direction(),alloc_slot[slotNum],slotNum,src,NOW);
			return;
		}
		printf("node:%d, MacAwsn::recv, tx_active_:%d, hdr->error:%d, hdr->txtime():%f,rx_state_:%d\n",index_,tx_active_, hdr->error(),hdr->txtime(),rx_state_);
		if (rx_state_ == MAC_IDLE) {
			rx_state_ = MAC_RECV;
			    pktRx_ = p;
				recvTimer.start(hdr->txtime());
		    	printf("node:%d, MacAwsn::recv: Successful, call handler in %f(%f)\n",index_,hdr->txtime(),hdr->txtime()+NOW);
			} else if(rx_state_ == MAC_MGMT) {
				printf("node:%d, MacAwsn::recv: rx_state_ == MAC_MGMT ignore the msg, time@%f\n",index_,NOW);
				Packet::free(p);
			} else {
				if(pktRx_ == NULL) {
					return;
				}
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
	return;
}

// This method handles the packet received and change the state
void MacAwsn::recvHandler() {
	if (pktRx_==NULL || (rx_state_ != MAC_RECV && rx_state_ != MAC_COLL)) {
		printf("MacAwsn::recvHandler: error_y!!! node %d, rx_state_:%d, wrong state in recvHandle\n",index_,rx_state_);
		exit(-1);
	}
	Packet* p = pktRx_;
	hdr_cmn *ch = HDR_CMN(p);
	char* mh = (char*)HDR_MAC(p);
	int dst = hdr_dst(mh);
	int src = hdr_src((char*)HDR_MAC(p));
	u_int16_t  type = hdr_mactype(mh,0);
	u_int16_t  subtype = hdr_submactype(mh,0);
	printf("node:%d, MacAwsn::recvHandler: src:%d, role:%d, dst:%d, tx_active_:%d, time@%f\n",index_,src,God::instance()->getRole(src),dst,tx_active_,NOW);
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
	    printf("node:%d, MacAwsn::recvHandler: src:%d,ch->error(), time@%f\n",index_,src,NOW);
		return;
	}

//	if (dst != index_ && (MacFrameType)type == MF_CONTROL && subtype == MAC_RIL_ACK){
//		printf("node:%d, MacAwsn::recvHandler--------: Changing MAC header, time@%f\n",index_,NOW);
//		subtype = MAC_RIL_RRTS;
//		hdr_submactype((char*)HDR_MAC(p), subtype);
//		dst = MAC_BROADCAST;
//		hdr_dst((char*)HDR_MAC(p), dst);
//	}
//	if (dst != index_ && dst != BCAST_ADDR) {
//		Packet::free(p);
//		pktRx_ = 0;
//		rx_state_ = MAC_IDLE;
//		return;
//	}

	switch(type) {
		case MF_CONTROL:
			switch(subtype) {
				case MAC_RIL_ACK:
					printf("node:%d, MacAwsn::recvHandler: MAC_RIL_ACK\n",index_);
					recvAck(pktRx_);
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
	if(God::instance()->isSink(index_)) {
//		macQueue.push_back(p);
		uptarget_->recv(p->copy(), (Handler*) 0);
	} else if(macQueue.size() < MAC_BufferSize) {
		macQueue.push_back(p);
	}
	God::instance()->setslotDataAck(index_,slotNum-1,1);
	rx_state_ = MAC_IDLE;
	printf("node:%d, MacAwsn::recvData: src:%d, dst:%d, slotNum:%d, macQueue_size:%d time@%f\n",index_,src,dst,slotNum-1,macQueue.size(),NOW);
}

void MacAwsn::recvAck(Packet *p) {
	char* mh = (char*)HDR_MAC(p);
	int dst = hdr_dst(mh);
	int src = hdr_src(mh);
	last_alive_ = NOW;
	if (tx_state_ != MAC_IDLE || rx_state_ != MAC_RECV){
		printf("error_z!!! node %d wrong state in recvAck \n", index_);
		exit(-1);
	}
	printf("node:%d, MacAwsn::recvAck src:%d, dst:%d, parentID:%d, slotNum:%d, macQueue_size:%d, macQueueTx_size:%d\n",
			index_,src,dst,parentID,slotNum-1,macQueue.size(),macQueueTx.size());
	if(src == parentID) {
		for(int i = frameLen; i > 0;i--) {
			if(alloc_slot[i] == slot_Tx ) {
				if(God::instance()->getslotDataAck(parentID,i) == 1) {
					macQueueTx.pop_back();
				} else {
					macQueue.push_front(macQueueTx.back());
					macQueueTx.pop_back();
				}
//				printf("node:%d data delivered at macQueueSize:%d macQueueTxSize:%d\n",index_,macQueue.size(),macQueueTx.size());
			}
		}
//		int maxSFslots = God::instance()->getmaxSFslots(parentID);
//		if(( maxSFslots - myTotalSFslots) > 0) {
//			shiftFrameSlot = shiftFrameSlot - maxSFslots + myTotalSFslots;
//			printf("node:%d, MacAwsn::recvAck: shiftFrameSlot:%d at start up due to charging rate difference\n",index_,shiftFrameSlot);
//		}
	}
	rx_state_ = MAC_IDLE;
	return;
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
	if(God::instance()->getRole(src) == PARENT) {
		updateParentInfo(src);
	} else {
		updateChildInfo(src);
	}
	if(myRole == PARENT) {
		if(dst == MAC_BROADCAST)
			sendBeaconAck(src);
	} else {   // myRole == CHILD
		if(src == parentID) {  // Get the scheduling info
			int myTxSlotsNum = 0;
			printf("node:%d, My schedule as CHILD::- ",index_);
			for (unsigned int i = 0; i < frameLen; i++) {
				if(index_ == God::instance()->getSchedule(src,i)) {
					alloc_slot[i] = slot_Tx;
					myTxSlotsNum++;
				} else {
					alloc_slot[i] = slot_Rx;
				}
				printf("%d:%c, ",i,TxRx[alloc_slot[i]]);
			}
			printf("\n");
			if(myTxSlotsNum < 1) {
//				God::instance()->setMyParent(index_,-2);
				sendBeaconAck(src);
				printf("node:%d, Send Beacon Ack to my parent:%d as it is not serving me\n",index_,parentID);
			}
		}
		if(parentID == -2) {	// Still looking for parent
			parentID = src;
			God::instance()->setMyParent(index_,parentID);
			int src_slot = God::instance()->getNodeSlot(src) - 1;
			if(src_slot>slotNum) {
				shiftFrameSlot = src_slot - 1;
				currentFrameSlots = maxFrameSlots - shiftFrameSlot;
				printf("node:%d, shiftFrameSlot:%d, currentFrameSlots:%d, maxFrameSlots:%d, totalSlotsShift:%d \n",index_,shiftFrameSlot,currentFrameSlots,maxFrameSlots,totalSlotsShift);
			} else if(src_slot < slotNum) {
				shiftFrameSlot = -slotNum + 1;
				printf("node:%d, shiftFrameSlot:%d, currentFrameSlots:%d, maxFrameSlots:%d, totalSlotsShift:%d \n",index_,shiftFrameSlot,currentFrameSlots,maxFrameSlots,totalSlotsShift);
			} else {
				//start sending data Frames are aligned

			}
			//send Beacon Ack
			if(dst == MAC_BROADCAST)
				sendBeaconAck(src);
		}
//		int maxSFslots = God::instance()->getmaxSFslots(parentID);
		double maxSFTime = God::instance()->getmaxSFTime(parentID);;
		if(( maxSFTime - mytotalSFTime) > 0) {
			runTimeShiftTime = maxSFTime - mytotalSFTime;
			printf("node:%d, MacAwsn::recvBeacon: runTimeShift:%f at start up due to charging rate difference\n",index_,runTimeShiftTime);
		}
	}
	return;
}
/*
 * This msg is send from child to parent to tell its availability
 * Send node ID, node's role, total super frame slots per cycle,
 */
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
		printf("node:%d, MacAwsn::sendBeaconAck: Radio Busy:tx_state_:%d,rx_state_:%d, time@%f\n",index_,tx_state_, rx_state_, NOW);
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
			it->SFslots = God::instance()->getTotalSFslots(src);
			return true;
		}
		it++;
	}
	neighborNode *r = new neighborNode(src);
	r->role = God::instance()->getRole(src);
	r->distSlots = totalSlotsShift;
	r->SFslots = God::instance()->getTotalSFslots(src);
	ParentTable.push_back(*r);
	printf("node:%d, MacAwsn::updateParentInfo: tableSize:%d\n",index_,ParentTable.size());
	return true;
}

// A method can be used to get most suitable Parent as well
//future work
neighborNode MacAwsn::getParentInfo() {
	return ParentTable.front();
}

bool MacAwsn::updateChildInfo(int src) {
	vector<neighborNode>::iterator it = ChildTable.begin();
	while (it!=ChildTable.end()) {
		if (it->id == src) {
			it->SFslots = God::instance()->getTotalSFslots(src);
			it->SFtime = God::instance()->getTotalSFTime(src);
			return true;
		}
		it++;
	}
	neighborNode *r = new neighborNode(src);
	r->role = God::instance()->getRole(src);
	r->SFslots = God::instance()->getTotalSFslots(src);
	r->SFtime = God::instance()->getTotalSFTime(src);
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

void BeaconTxAwsnTimer::handle(Event *e)
{
	busy_ = 0;
	stime = 0.0;
	rtime = 0.0;
	mac->BeaconSent();
}

void BeaconRxAwsnTimer::handle(Event *e)
{
	busy_ = 0;
	stime = 0.0;
	rtime = 0.0;
	mac->runAtstartUp();
}

void sendBeaconTxAwsnTimer::handle(Event *e)
{
	busy_ = 0;
	stime = 0.0;
	rtime = 0.0;
	mac->sendBeacon(MAC_BROADCAST);
}

void postFrameAwsnTimer::handle(Event *e)
{
	busy_ = 0;
	stime = 0.0;
	rtime = 0.0;
	mac->postFrameHandler();
}
