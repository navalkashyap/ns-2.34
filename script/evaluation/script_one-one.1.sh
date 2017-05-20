#for interval in {0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1,1.2,1.3,1.4,1.5,1.6,1.7,1.8,1.9,2.0,2.1,2.2,2.3,2.4,2.5,2.6,2.7,2.8,2.9,3.0,3.1,3.2,3.3,3.4,3.5,3.6,3.7,3.8,3.9,4.0,4.1,4.2,4.3,4.4,4.5,4.6,4.7,4.8,4.9,5.0} 

for interval in {0.1,0.2,0.3,0.4,0.5,0.7,1,2,3,4,5,6,7,8,9,10}
do
for buffer in {2,3,4,5,6,7,8,9,10,13,15,17,20} 
do
	outputfile=one-one.log
	#interval=$(($i/10))
	../../ns one-on-one.tcl 2 $interval 1 $buffer> $outputfile
	Delay=0
	Delay=`grep  "node:0, RILApp::recvMsg.*node-$nn" $outputfile | tail -1 | awk '$0=$2' FS=Delay: RS=,`
	echo "$interval $buffer $Delay" >>  oneOnly_latency_buffer_interval.log
	pktSent=`grep -c "node:1, RILApp::sendMsg" $outputfile`
	pktRecvd=`grep -c "RILApp::recvMsg: receive a data message from node-1" $outputfile`
	drop=$(($pktRecvd*100/$pktSent))
	drop=$((100-$drop))
	echo "Interval $interval, Buffer $buffer, Drop Ratio $drop Latency $Delay"
	echo "$interval $buffer $drop" >>  oneOnly_drop_buffer_interval.log
done
done
exit

for interval in {0.2,0.5,1,2,3,4,5} 
do
	outputfile=one-one.log
	#interval=$(($i/10))
	../../ns one-on-one.tcl 2 $interval 1 > $outputfile
	Delay=0
	Delay=`grep  "node:0, RILApp::recvMsg.*node-$nn" $outputfile | tail -1 | awk '$0=$2' FS=Delay: RS=,`
	echo "$interval $Delay" >>  oneOnly_latency.log
	pktSent=`grep -c "node:1, RILApp::sendMsg" $outputfile`
	pktRecvd=`grep -c "RILApp::recvMsg: receive a data message from node-1" $outputfile`
	drop=$(($pktRecvd*100/$pktSent))
	drop=$((100-$drop))
	echo "Interval $interval, Drop Ratio $drop Latency $Delay"
	echo "$interval $drop" >>  oneOnly_drop.log
done

for interval in {6,7,8,9,10,11,12,13,14,15,16,17,18,19,20} 
do
	outputfile=one-one.log
	#interval=$(($i/10))
	../../ns one-on-one.tcl 2 $interval 1 > $outputfile
	Delay=0
	Delay=`grep  "node:0, RILApp::recvMsg.*node-$nn" $outputfile | tail -1 | awk '$0=$2' FS=Delay: RS=,`
	echo "$interval $Delay" >>  oneOnly_latency.log
	pktSent=`grep -c "node:1, RILApp::sendMsg" $outputfile`
	pktRecvd=`grep -c "RILApp::recvMsg: receive a data message from node-1" $outputfile`
	drop=$(($pktRecvd*100/$pktSent))
	drop=$((100-$drop))
	echo "Interval $interval, Drop Ratio $drop Latency $Delay"
	echo "$interval $drop" >>  oneOnly_drop.log
done