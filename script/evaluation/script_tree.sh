for interval in `seq 1 20`;
	do
	outputfile=output.log
	../../ns tree.tcl 8 $interval 1 > $outputfile
	Delay=`grep "RILApp::recvMsg" $outputfile | tail -1 | awk '$0=$2' FS=Delay: RS=,`
	pktSent=`grep -c "RILApp::sendMsg" $outputfile`
	pktRecvd=`grep -c "RILApp::recvMsg: receive a data message from node" $outputfile`
	drop=$(($pktRecvd*100/$pktSent))
	drop=$((100-$drop))
	echo "$interval $Delay" >> tree_latency_node_7.log 
	echo "$interval $drop" >> tree_drop_node_7.log
done
