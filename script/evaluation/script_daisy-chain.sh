nn=5
	for interval in {1,2,5,10};
        do
		outputfile=output.log
		../../ns daisy_chain.tcl $nn $interval 1 9 > $outputfile
		Delay=0
		Delay=`grep  "node:0, RILApp::recvMsg.*node-" $outputfile | tail -1 | awk '$0=$2' FS=Delay: RS=,`
		echo "$interval $Delay" >>  daisy_latency_node_$nn.log
		pktSent=`grep -c "node:.*, RILApp::sendMsg" $outputfile`
		pktRecvd=`grep -c "RILApp::recvMsg: receive a data message from node-" $outputfile`
		drop=$(($pktRecvd*100/$pktSent))
		drop=$((100-$drop))
		echo "Node $nn, Drop Ratio $drop Latency $Delay"
		echo "$interval $drop" >>  daisy_drop_node_$nn.log
	done

	
exit	
	nn=4
	for interval in {1,2,5,10};
        do
		outputfile=output.log
		../../ns daisy_chain.tcl $nn $interval 1 > $outputfile
		Delay=0
		Delay=`grep  "node:0, RILApp::recvMsg.*node-" $outputfile | tail -1 | awk '$0=$2' FS=Delay: RS=,`
		echo "$interval $Delay" >>  daisy_latency_node_$nn.log
		pktSent=`grep -c "node:.*, RILApp::sendMsg" $outputfile`
		pktRecvd=`grep -c "RILApp::recvMsg: receive a data message from node-" $outputfile`
		drop=$(($pktRecvd*100/$pktSent))
		drop=$((100-$drop))
		echo "Node $nn, Drop Ratio $drop Latency $Delay"
		echo "$interval $drop" >>  daisy_drop_node_$nn.log
	done
