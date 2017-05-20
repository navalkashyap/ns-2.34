
interval=25
for interval in {0.1,0.2,0.5,0.7,0.9,1,5,10,15,20,25,30,35,40}
do
outputfile=output.log
../ns amensticmac.tcl 3 $interval 1 > $outputfile
averageDelay=0
TotalpktSent=0
TotalpktRecvd=0
for node in {1,2}
do
	Delay=`grep  "node:0, RILApp::recvMsg.*node-$node" $outputfile | tail -1 | awk '$0=$2' FS=Delay: RS=,`
	pktSent=`grep -c "node:$node, RILApp::sendMsg" $outputfile`
	pktRecvd=`grep -c "RILApp::recvMsg: receive a data message from node-$node" $outputfile`
	averageDelay=$averageDelay+$Delay
	TotalpktSent=$TotalpktSent+$pktSent
	TotalpktRecvd=$TotalpktRecvd+$pktRecvd
#	echo Node=$node, Pkts Sent=$pktSent, PktsRecvd=$pktRecvd, AvgDelay=$Delay
done
echo Nodes,3, Interval,$interval, EverageDelay,$averageDelay, TotalpktRecvd,$TotalpktRecvd, TotalpktSent,$TotalpktSent


averageDelay=0
TotalpktSent=0
TotalpktRecvd=0
outputfile=simulation_logs/edgenode7_$interval.log
../ns amensticmac.tcl 7 $interval 3 > $outputfile
for node in {3,4,5,6}
do
	Delay=`grep  "node:0, RILApp::recvMsg.*node-$node" $outputfile | tail -1 | awk '$0=$2' FS=Delay: RS=,`
	pktSent=`grep -c "node:$node, RILApp::sendMsg" $outputfile`
	pktRecvd=`grep -c "RILApp::recvMsg: receive a data message from node-$node" $outputfile`
	averageDelay=$averageDelay+$Delay
	TotalpktSent=$TotalpktSent+$pktSent
	TotalpktRecvd=$TotalpktRecvd+$pktRecvd
#	echo Node=$node, Pkts Sent=$pktSent, PktsRecvd=$pktRecvd, AvgDelay=$Delay
done
echo Nodes,7, Interval,$interval, EverageDelay,$averageDelay, TotalpktRecvd,$TotalpktRecvd, TotalpktSent,$TotalpktSent


averageDelay=0
TotalpktSent=0
TotalpktRecvd=0
outputfile=simulation_logs/edgenode15_$interval.log
../ns amensticmac.tcl 15 $interval 7 > $outputfile
for node in {7,8,9,10,11,12,13,14}
do
	Delay=`grep  "node:0, RILApp::recvMsg.*node-$node" $outputfile | tail -1 | awk '$0=$2' FS=Delay: RS=,`
	pktSent=`grep -c "node:$node, RILApp::sendMsg" $outputfile`
	pktRecvd=`grep -c "RILApp::recvMsg: receive a data message from node-$node" $outputfile`
	averageDelay=$averageDelay+$Delay
	TotalpktSent=$TotalpktSent+$pktSent
	TotalpktRecvd=$TotalpktRecvd+$pktRecvd
#	echo Node=$node, Pkts Sent=$pktSent, PktsRecvd=$pktRecvd, AvgDelay=$Delay
done
echo Nodes,15, Interval,$interval, EverageDelay,$averageDelay, TotalpktRecvd,$TotalpktRecvd, TotalpktSent,$TotalpktSent



averageDelay=0
TotalpktSent=0
TotalpktRecvd=0
outputfile=simulation_logs/edgenode31_$interval.log
../ns amensticmac.tcl 31 $interval 15 > $outputfile
for node in {15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31}
do
        Delay=`grep  "node:0, RILApp::recvMsg.*node-$node" $outputfile | tail -1 | awk '$0=$2' FS=Delay: RS=,`
        pktSent=`grep -c "node:$node, RILApp::sendMsg" $outputfile`
        pktRecvd=`grep -c "RILApp::recvMsg: receive a data message from node-$node" $outputfile`
		averageDelay=$averageDelay+$Delay
        TotalpktSent=$TotalpktSent+$pktSent
        TotalpktRecvd=$TotalpktRecvd+$pktRecvd
#       echo Node=$node, Pkts Sent=$pktSent, PktsRecvd=$pktRecvd, AvgDelay=$Delay
done
echo Nodes,31, Interval,$interval, EverageDelay,$averageDelay, TotalpktRecvd,$TotalpktRecvd, TotalpktSent,$TotalpktSent

done










