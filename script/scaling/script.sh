node=16
buffersize=9
for node in {15,31,47,63,79,95,111}
do 
	for interval in {0.5,1,2,3,4,5,6,7,8,9,10}
	do
		for buffersize in {5,10,15,20,25,30}
		do
			outputfile=output.log
			../../ns amensticmac.tcl $node $interval 1 $buffersize > $outputfile
			Delay=0
			Delay=`grep  "RILApp::recvMsg" $outputfile | tail -1 | awk '$0=$2' FS=Delay: RS=,`
			pktSent=`grep -c "node:.*, RILApp::sendMsg" $outputfile`
			pktRecvd=`grep -c "RILApp::recvMsg: receive a data message from node-" $outputfile`
			drop=$(($pktRecvd*100/$pktSent))
			drop=$((100-$drop))
			echo "Node $node, Interval $interval, buffersize $buffersize, Drop Ratio $drop, Latency $Delay"
			echo "$node, $interval, $buffersize, $drop, $Delay" >>  node_interval_buffer.csv
			echo "$node  $interval  $buffersize  $drop  $Delay" >>  node_interval_buffer.log
		done
	done
done


exit

for node in {15,31,47,63,79,95,111}
do
	outputfile=output.log
	../../ns amensticmac.tcl $node 5 1 > $outputfile
	Delay=0
	Delay=`grep  "RILApp::recvMsg" $outputfile | tail -1 | awk '$0=$2' FS=Delay: RS=,`
	pktSent=`grep -c "node:.*, RILApp::sendMsg" $outputfile`
	pktRecvd=`grep -c "RILApp::recvMsg: receive a data message from node-" $outputfile`
	drop=$(($pktRecvd*100/$pktSent))
	drop=$((100-$drop))
	echo "Node $node, Drop Ratio $drop Latency $Delay"
	echo "$node 40" >>  node_vs_buffer.log
done
