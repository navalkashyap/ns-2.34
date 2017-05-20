nn=1
interval=3
outputfile=one-one.log
../../ns one-on-one.tcl 2 $interval 1 > $outputfile
Delay=0
Delay=`grep  "node:0, RILApp::recvMsg.*node-$nn" $outputfile | tail -1 | awk '$0=$2' FS=Delay: RS=,`
pktSent=`grep -c "node:1, RILApp::sendMsg" $outputfile`
pktRecvd=`grep -c "RILApp::recvMsg: receive a data message from node-1" $outputfile`
drop=$(($pktRecvd*100/$pktSent))
drop=$((100-$drop))
echo "Interval $interval, Drop Ratio $drop Latency $Delay"
