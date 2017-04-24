for node in {3,7,15}
do
        for interval in {1,5,10,15,20,25,30,35,40}
        do
                ../ns amensticmac.tcl $node $interval 1 > output
                avgDelay=0
                Delay=0
                for nn in `seq 1 $(($node-1))`;
                do
                        Delay=`grep  "node:0, RILApp::recvMsg.*node-$nn" output | tail -1 | awk '$0=$2' FS=Delay: RS=,`
                        if [ "$Delay" != "" ]
			then averageDelay=$(python -c "print $averageDelay+$Delay")
			fi
                done
                echo Nodes,$node, Interval,$interval, EverageDelay,$averageDelay
        done

done

