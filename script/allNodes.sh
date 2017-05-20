for node in {3,7,15,31}
do
        for interval in `seq 1 20`;
        do
				outputfile=simulation_logs/allnode_$node.$interval.log
                ../ns amensticmac.tcl $node $interval 1 > $outputfile
                avgDelay=0
                Delay=0
				count=0
                for nn in `seq 1 $(($node-1))`;
                do
                        Delay=`grep  "node:0, RILApp::recvMsg.*node-$nn" $outputfile | tail -1 | awk '$0=$2' FS=Delay: RS=,`
                        if [ "$Delay" != "" ]
							then averageDelay=$(python -c "print $averageDelay+$Delay")
						fi
						count=$(($count+1)) 
                done
                echo Nodes,$node, Interval,$interval, AverageDelay,$averageDelay
        done

done

