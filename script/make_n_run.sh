#cd ..
#make
#cd -

#../ns amensticmac.tcl 7 25 3 > output
#cat output | grep "node:[0,1,3,4]" > output_1
#cat output | grep "node:[0,2,5,6]" > output_2

interval=25
for interval in {1,5,10,15,20,25,30,35,40}
do

../ns amensticmac.tcl 3 $interval 1 > output
averageDelay=0
TotalpktSent=0
TotalpktRecvd=0
for node in {1,2}
do
	Delay=`grep  "node:0, RILApp::recvMsg.*node-$node" output | tail -1 | awk '$0=$2' FS=Delay: RS=,`
	pktSent=`grep -c "node:$node, RILApp::sendMsg" output`
	pktRecvd=`grep -c "RILApp::recvMsg: receive a data message from node-$node" output`
	averageDelay=$(python -c "print $averageDelay+$Delay")
	TotalpktSent=$TotalpktSent+$pktSent
	TotalpktRecvd=$TotalpktRecvd+$pktRecvd
#	echo Node=$node, Pkts Sent=$pktSent, PktsRecvd=$pktRecvd, AvgDelay=$Delay
done
echo Nodes,3, Interval,$interval, EverageDelay,$averageDelay, TotalpktRecvd,$TotalpktRecvd, TotalpktSent,$TotalpktSent


averageDelay=0
TotalpktSent=0
TotalpktRecvd=0
../ns amensticmac.tcl 7 $interval 3 > output
for node in {3,4,5,6}
do
	Delay=`grep  "node:0, RILApp::recvMsg.*node-$node" output | tail -1 | awk '$0=$2' FS=Delay: RS=,`
	pktSent=`grep -c "node:$node, RILApp::sendMsg" output`
	pktRecvd=`grep -c "RILApp::recvMsg: receive a data message from node-$node" output`
	averageDelay=$(python -c "print $averageDelay+$Delay")
	TotalpktSent=$TotalpktSent+$pktSent
	TotalpktRecvd=$TotalpktRecvd+$pktRecvd
#	echo Node=$node, Pkts Sent=$pktSent, PktsRecvd=$pktRecvd, AvgDelay=$Delay
done
echo Nodes,7, Interval,$interval, EverageDelay,$averageDelay, TotalpktRecvd,$TotalpktRecvd, TotalpktSent,$TotalpktSent


averageDelay=0
TotalpktSent=0
TotalpktRecvd=0
../ns amensticmac.tcl 15 $interval 7 > output
for node in {7,8,9,10,11,12,13,14}
do
	Delay=`grep  "node:0, RILApp::recvMsg.*node-$node" output | tail -1 | awk '$0=$2' FS=Delay: RS=,`
	pktSent=`grep -c "node:$node, RILApp::sendMsg" output`
	pktRecvd=`grep -c "RILApp::recvMsg: receive a data message from node-$node" output`
	averageDelay=$(python -c "print $averageDelay+$Delay")
	TotalpktSent=$TotalpktSent+$pktSent
	TotalpktRecvd=$TotalpktRecvd+$pktRecvd
#	echo Node=$node, Pkts Sent=$pktSent, PktsRecvd=$pktRecvd, AvgDelay=$Delay
done
echo Nodes,15, Interval,$interval, EverageDelay,$averageDelay, TotalpktRecvd,$TotalpktRecvd, TotalpktSent,$TotalpktSent

done












