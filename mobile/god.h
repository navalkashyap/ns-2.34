
/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*- */
/*
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
 */
/* Ported from CMU/Monarch's code, nov'98 -Padma.*/
/* -*- c++ -*-
   god.h

   General Operations Director

   perform operations requiring omnipotence in the simulation
   */

#ifndef __god_h
#define __god_h
#include <vector>  // vector use causes overriding problem with in some methods

#include <stdarg.h>
#include "bi-connector.h"
#include "object.h"
#include "packet.h"
#include "trace.h"
#include "node.h"
#include "diffusion/hash_table.h"

#include "ctrace.h"
#include "time.h"
//#include "charger.h"
#include "charging-energy-model.h"
// Added by Chalermek  12/1/99

#define MIN_HOPS(i,j)    min_hops[i*num_nodes+j]
#define NEXT_HOP(i,j)    next_hop[i*num_nodes+j]
#define SRC_TAB(i,j)     source_table[i*num_nodes+j]
#define SK_TAB(i,j)      sink_table[i*num_nodes+j]
#define	UNREACHABLE	 0x00ffffff
#define RANGE            250.0                 // trasmitter range in meters


class NodeStatus {
public:
  bool is_source_;
  bool is_sink_;
  bool is_on_trees_;

  NodeStatus() { is_source_ = is_sink_ = is_on_trees_ = false; }
};


// ------------------------


class God : public BiConnector {
public:
        God();

        int             command(int argc, const char* const* argv);

        void            recv(Packet *p, Handler *h);
        void            stampPacket(Packet *p);

        int initialized() {
                return num_nodes && min_hops && uptarget_;
        }

        int             hops(int i, int j);
        static God*     instance() { assert(instance_); return instance_; }
        int nodes() { return num_nodes; }

        inline void getGrid(double *x, double *y, double *z) {
		*x = maxX; *y = maxY; *z = gridsize_;
	}


  // Added by Chalermek 12/1/99

        int  data_pkt_size;        // in bytes. 
        int  num_alive_node;
        int  num_connect;
        int  num_recv;
        int  num_compute;          // number of route-computation times
        double prev_time;          // the previous time it computes the route
        int  num_data_types;      
        int  **source_table;
        int  *sink_table;
        int  *num_send;            // for each data type
        Data_Hash_Table dtab;

        void DumpNodeStatus();
        void DumpNumSend();
        void CountNewData(int *attr);
        void IncrRecv();
        bool ExistSource();
        bool ExistSink();
        bool IsPartition();
        void StopSimulation();
        void CountConnect();
        void CountAliveNode();
        void ComputeRoute();      
        int  NextHop(int from, int to);
        void ComputeNextHop();     // Look at min_hops to fill in next_hop
        void Dump();               // Dump all internal data
        bool IsReachable(int i, int j);  // Is node i reachable to node j ?
        bool IsNeighbor(int i, int j);   // Is node i a neighbor of node j ?
        void ComputeW();           // Initialize the connectivity metrix
        void floyd_warshall();     // Calculate the shortest path

        void AddSink(int dt, int skid);
        void AddSource(int dt, int srcid);
        void Fill_for_Sink(int dt, int srcid);
        void Fill_for_Source(int dt, int skid);
        void Rewrite_OIF_Map();
        void UpdateNodeStatus();
        
        // Return number of next oifs in ret_num_oif.
        // Return array of next oifs as return value of the function.

        int *NextOIFs(int dt, int srcid, int curid, int *ret_num_oif);
  
        // serve for GAF algorithm
  
        int load_grid(int,int,int);

        int getMyGrid(double x, double y);
        int getMyLeftGrid(double x, double y);
        int getMyRightGrid(double x, double y);
        int getMyTopGrid(double x, double y);
        int getMyBottomGrid(double x, double y);
        
        inline int getMyGridSize() {
		return gridsize_;
	}

  // -----------------------
        // Yang- 04/25/2010
        CTRACE*	ctrace() { assert(ctrace_); return ctrace_; }
        void addDutyOnTime(int nodeid, double time);
        void logDutyOntime();
        MobileNode* getNode(int targetID){return mb_node[targetID];}
	inline bool isSink(int nodeid){return sink_id[nodeid];}
	inline int sink_num(){return sink_num_;}
	inline bool enableCharge(){return enableCharge_;}
	double* lastUpdateTime;
	double* lastUpdateEng;
	double moveEng;
	double* lastWorkload;
        // -Yang
	// Naval
	inline void setMyParent(int nodeId, int parentId) { myParent[nodeId] = parentId;}
	inline int getMyParent(int nodeId) {return myParent[nodeId]; }
	inline void setRole(int nodeId, int role) {Role[nodeId] = role; }
	inline bool getRole(int nodeId) {return Role[nodeId]; }
	inline void setMySlot(int slotNum,int nodeID) {mySlot[nodeID] = slotNum;}
	inline int getNodeSlot(int nodeID) {return mySlot[nodeID]; }
	inline void setSchedule(int nodeID, std::vector<int> myschedule) {NodeSchedule[nodeID] = myschedule;}
	inline std::vector<int> getSchedule(int nodeID) { return NodeSchedule[nodeID]; }
	inline double getP_charge(int nodeID) {return P_charge[nodeID];}
	inline void setTotalSFslots(int nodeID,int totalSlots) {TotalSFslots[nodeID] = totalSlots;}
	inline int getTotalSFslots(int nodeID) {return TotalSFslots[nodeID];}
	inline void setmaxSFslots(int nodeID,int totalSlots) {maxSFslots[nodeID] = totalSlots;}
	inline int getmaxSFslots(int nodeID) {return maxSFslots[nodeID];}
	inline void setslotDataAck(int nodeID, int slotNum, int val) {slotDataAck[nodeID][slotNum] = val;}
	inline int getslotDataAck(int nodeID, int slotNum) {return slotDataAck[nodeID][slotNum];}

//-Naval
private:
        int num_nodes;
        int* min_hops;   // square array of num_nodesXnum_nodes
                         // min_hops[i * num_nodes + j] giving 
			 // minhops between i and j
        static God*     instance_;


        // Added by Chalermek    12/1/99

        bool active;
        bool allowTostop;
        MobileNode **mb_node; // mb_node[i] giving pointer to object 
                              // mobile node i
        NodeStatus *node_status;
        int *next_hop;        // next_hop[i * num_nodes + j] giving
                              //   the next hop of i where i wants to send
                              //	 a packet to j.

        int maxX;          // keeping grid demension info: max X, max Y and 
        int maxY;          // grid size
        int gridsize_;
        int gridX;
        int gridY;
        //Naval
        int* myParent;
        bool* Role;			// Parent :0, Child: 1
        int* mySlot;
        vector<int> sche;
        vector<vector<int> > NodeSchedule;
        int** slotDataAck;		//dataAck[i] gives pointer to array of int
        int chargeid;
        double* P_charge;
        int* TotalSFslots;
        int* maxSFslots;
        //~Naval
        // added by Yang 04/25/2010
        CTRACE* ctrace_;
        double* dutyontime_;
	bool* sink_id;
	int sink_num_;
	bool enableCharge_;
};

#endif

