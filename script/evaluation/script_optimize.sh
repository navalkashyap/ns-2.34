interval=7
buffer=9
node_num=9
for maxParentcycle in {3}
do
	outputfile=output.log
	../../ns optimized.tcl $node_num $interval 1 > $outputfile
	Delay=0
	Delay=`grep  "node:0, RILApp::recvMsg.*node-" $outputfile | tail -1 | awk '$0=$2' FS=Delay: RS=,`
#	echo "$interval $buffer $Delay" >>  oneOnly_latency_buffer_interval.log
	pktSent=`grep -c "RILApp::sendMsg" $outputfile`
	pktRecvd=`grep -c "RILApp::recvMsg" $outputfile`
	drop=$(($pktRecvd*100/$pktSent))
	drop=$((100-$drop))
	echo "Interval $interval, Buffer $buffer, Drop Ratio $drop Latency $Delay"
#	echo "$interval $buffer $drop" >>  oneOnly_drop_buffer_interval.log

done

exit

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
