/*
 * nrrapplication.cc
 *
 *  Created on: Aug 19, 2009
 *      Author: yang
 */

#include "rilapp.h"
#include "god.h"


static class RILAppClass : public TclClass {
public:
	RILAppClass() : TclClass("Application/RILApp") {}
	TclObject* create(int argc, const char*const* argv) {
		assert(argc==5);
		return (new RILApp(atoi(argv[4])));
	}
}class_RILApp;


int RILApp::command(int argc, const char*const* argv) {
	//Tcl& tcl = Tcl::instance();
	if (argc == 2) {
		if (strcmp(argv[1], "start") == 0) {
			start();
			return (TCL_OK);
		}
		else if (strcmp(argv[1], "stop") == 0) {
			stop();
			return (TCL_OK);
		}
	}
	return (Application::command(argc, argv));
}

double RILApp::next_interval() {
#if 0
    double U;
    U = Random::exponential(dataInterval_);
    double L = exp(-dataInterval_);
    int k = 0; double p = 1;
    do {
        k = k+1;
        p= p*Random::uniform();
    }while (p>L);
    U = k-1;
    return U+0.1;
#endif

        return Random::uniform(dataInterval_*0.9,dataInterval_*1.1);

}


void RILApp::timeout(int type) {
    if (type == DTIMER) {
	    RILAppMessageT_ msg;
		if (myID != 0 && God::instance()->getparentDiscovered(myID)) {
	        produceMsg(&msg, DATA_SENSING);
            sendMsg(&msg);
//            printf("RILApp::timeout: node:%d send app, msg seq:%d, msg size:%d @ %f\n", myID, msg.seq,msg.len, NOW);
        }
        dataT_->resched(next_interval());
    }

}

// process received nrrapp packet
void RILApp::process_data(int size, AppData* data) {
	PacketData* p = (PacketData*)data;
	RILAppMessageT_* cappmsgp = (RILAppMessageT_*)p->data();
    recvMsg(cappmsgp);
	return;
}

void RILApp::start() {
	//God::instance()->ctrace()->log("node %d start charging app @ %f\n", myID, NOW);
	if (myID != 0 && God::instance()->isEdgeNode(myID)) {
	    dataT_ = new RILAppTimer(this, DTIMER);
	    averageDelay = 0.0;
	    totalDelay = 0;
	    totalReceivedpkt = 0;
        dataT_->sched(next_interval());
        //printf("node %d data rate %f\n",myID, dataInterval_);
        printf("node:%d, RILApp::start: isEdgeNode:%d\n",myID,God::instance()->isEdgeNode(myID));
    } else {
    	printf("node:%d, RILApp::start: Does not started, isEdgeNode:%d\n",myID,God::instance()->isEdgeNode(myID));
    }
}

void RILApp::stop() {
	//God::instance()->ctrace()->log("node %d stop charging app @ %f\n", myID, NOW);
	if (dataT_!=0) {
		dataT_->stoptimer();
	}
}

void RILApp::produceMsg(RILAppMessageT_* msg, int msgtype) {
	msg->source = myID;
	msg->msgType = msgtype;
	msg->timestamp = NOW;
	msg->seq = packetseq++;
	return;
}

// send msg based on destination
void RILApp::sendMsg(RILAppMessageT_* msg) {
	msg->dest = 0;//God::instance()->sink_num();
    msg->netType = UCAST;
	PacketData* data = new PacketData(sizeof(RILAppMessageT_));
	memcpy(data->data(), msg, sizeof(RILAppMessageT_));
    printf("node:%d, RILApp::sendMsg: data arrival msg->seq:%d, @%f\n", myID, msg->seq,NOW);
	agent_->sendmsg(sizeof(RILAppMessageT_), data);
}

void RILApp::recvMsg(RILAppMessageT_* msg) {
    static CTRACE* trace_ = God::instance()->ctrace();
    if (msg == 0) return;
    if (msg->msgType == DATA_SENSING) {
        int src = msg->source;
        double delay = NOW-msg->timestamp;
        totalReceivedpkt += 1.0f;
        totalDelay += delay;
        averageDelay = totalDelay / totalReceivedpkt;
        printf("node:%d, RILApp::recvMsg: receive a data message from node-%d msg->seq:%d delay:%f, averageDelay:%f, totalReceivedpkt:%f @%f\n",
        		myID,src, msg->seq,delay,averageDelay,totalReceivedpkt,NOW);
        trace_->log("node %d receive a data message from %d seq %d delay %f @ %f\n", myID,src, msg->seq,delay,NOW);
        //printf("node %d received %f\n", src, NOW);
    }
}

void RILApp::dumpMsg(RILAppMessageT_* msg, bool yes) {

}

/*------------------- NRRApp class stop ----------------------*/

void RILAppTimer::expire(Event *) {
	if (continueRunning) {
		appinstance_->timeout(this->type);
	}
}







