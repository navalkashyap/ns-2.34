# ======================================================================
# Define options
# ======================================================================
set val(chan)           Channel/WirelessChannel    ;# channel type
set val(prop)           Propagation/TwoRayGround   ;# radio-propagation model
set val(netif)          Phy/WirelessCPhy           ;# network interface type
#set val(mac)            Mac/Awsn		           ;# user our own mac
set val(mac)            Mac/BF		           ;# user our own mac
set val(ifq)            Queue/DropTail             ;# interface queue type
set val(ll)             LL                         ;# link layer type
set val(ant)            Antenna/OmniAntenna        ;# antenna model
set val(ifqlen)         1000                       ;# max packet in ifq
set val(rp)             ChargingRT                 ;# use our own routing

#set val(et)             [lindex $argv 2]
#set val(nsseed)         [lindex $argv 3]
#set val(rtype)          [lindex $argv 4]
#set val(random)         [lindex $argv 5]
#set val(runningmac)     [lindex $argv 6]
#set val(logfile)        [lindex $argv 7]
#set val(routeinterval)  [lindex $argv 8]
#set val(trvalue)  		 [lindex $argv 9]
#set val(fulleng)        20000.0


set val(nn)             [lindex $argv 0]
set val(datainterval)   [lindex $argv 1]
set val(firstLeafNode)  [lindex $argv 2]
set val(buffersize)		[lindex $argv 3]
set val(maxParentCycle)	[lindex $argv 4]

#set val(nn)             7
#set val(datainterval)  25
#set val(firstLeafNode)  3
set val(et)             100000
set val(nsseed)         100
set val(rtype)          1
set val(random)         10
set val(runningmac)     1
set val(logfile)        log
set val(routeinterval)  3
set val(trvalue)  		1
set val(fulleng)        0.0015
puts $val(nn)
puts $val(datainterval)
puts $val(firstLeafNode)



# some ns2 default settings
ns-random 		$val(nsseed)
set ns_         [new Simulator]
set tracefd     [open /dev/null w]
$ns_ trace-all  $tracefd
set topo        [new Topography]
$topo           load_flatgrid 1500 1500

source ns2parameters.tcl
Mac/BF				set MaxParentCycle 	$val(maxParentCycle)
Mac/BF				set Buffersize  	$val(buffersize)

Agent/ChargingRT    set RTYPE $val(rtype)
Agent/ChargingRT    set routinginterval $val(routeinterval)
Agent/ChargingRT    set twtf [expr $val(trvalue)/10]
#Mac/RILMac          set runningmac $val(runningmac)
#Mac/RILMac          set defaulttr $val(trvalue)
Phy/WirelessCPhy    set routingscheme $val(rtype)
Phy/WirelessCPhy    set randomenergy $val(random)

remove-all-packet-headers
add-packet-header LL MAC IP RILAGENT CHARGINGRT

$ns_ node-config    -adhocRouting $val(rp) \
                    -llType $val(ll) \
                    -macType $val(mac) \
                    -ifqType $val(ifq) \
                    -ifqLen $val(ifqlen) \
                    -antType $val(ant) \
                    -propType $val(prop) \
                    -phyType $val(netif) \
                    -channelType $val(chan) \
                    -topoInstance $topo \
                    -agentTrace OFF \
                    -routerTrace OFF \
                    -macTrace OFF \
                    -movementTrace OFF \
                    -phyTrace_ OFF \
                    -eotTrace_ OFF \
                    -toraDebug OFF

set val(total) [expr {$val(nn)}]; #extra one is for sink
set val(mylog) [new CTRACE $val(logfile)]

set god_ [create-god $val(total)]
$god_ add-logfile $val(mylog)

source treeStructure.tcl

for {set i 0} {$i <  $val(total)} {incr i} {
    $god_ setnodechargeID $i
	if {$i==0} {
        $god_ setnodePcharge 0.00010
    } elseif {$i < 5} {
        $god_ setnodePcharge 0.00010
    } elseif {$i < 15} {
        $god_ setnodePcharge 0.00010
    } elseif {$i < 31} {
        $god_ setnodePcharge 0.00010
    } else {
        $god_ setnodePcharge 0.00010
    } 
    set node_($i) [$ns_ node]
	$node_($i) random-motion 0
    set rtagent [$node_($i) set ragent_]
    $rtagent add-node $node_($i)
    set val(myenergy) [expr $val(fulleng)*$i/$val(total)]
    set val(maxenergy) $val(fulleng)
    set engmodel_($i) [new EnergyModel $node_($i) $val(fulleng) 0 0]
    $node_($i) addenergymodel $engmodel_($i)
    $god_ new_node $node_($i)

	Application/RILApp 		set DataRate 	$val(datainterval)
    set RILapp_($i) 		[new Application/RILApp $i]
    set RILagent_($i) 		[new Agent/RILAgent]
    $ns_ attach-agent $node_($i) $RILagent_($i)
    $RILapp_($i) 			attach-agent $RILagent_($i)

	if {$val(rtype) == 1} {
        if {$i==0} {
            $god_ setassink $i
        } else {
            $god_ setassource $i
        }
	} else {
        puts "wrong input"
    }
	
	if {$i < 1} {
	    $node_($i) set X_ 0.0
	    $node_($i) set Y_ 0.0
    } elseif {$i < 3} {
	    $node_($i) set X_ [expr 60.0 + $i]
	    $node_($i) set Y_ 0.0
    } elseif {$i < 8} {
	    $node_($i) set X_ [expr 2*60.0 + $i]
	    $node_($i) set Y_ 0.0
    } elseif {$i < 16} {
	    $node_($i) set X_ [expr 3*60.0 + $i]
	    $node_($i) set Y_ 0.0
    } elseif {$i < 32} {
	    $node_($i) set X_ [expr 4*60.0 + $i]
	    $node_($i) set Y_ 0.0
    } elseif {$i >=32} {
	    $node_($i) set X_ [expr 5*60.0 + $i]
	    $node_($i) set Y_ 0.0
    }
    
     
    if {$i >= $val(firstLeafNode)} {
    	$god_ setasEdgeNode $i
    } else {
    	$god_ setasNotEdgeNode $i
    }
    $node_($i) set Z_ 0.0      

    $ns_ at 5.0 "$RILapp_($i) start"
}

for {set i 0} {$i < $val(total) } {incr i} {
    $ns_ at [expr {$val(et)+0.5}] "$RILapp_($i) stop";
    $ns_ at [expr {$val(et)+1.5}] "$node_($i) reset";
}

$ns_ at [expr {$val(et)+2.0}] "stop"
$ns_ at [expr {$val(et)+2.1}] "puts \"NS EXITING...\"; $ns_ halt;"
#$ns_ at 20000.0 "stop"
#$ns_ at 20000.0 "puts \"NS EXITING...\"; $ns_ halt;"

proc stop {} {
    puts "calling stop function"
    global ns_ tracefd
    $ns_ flush-trace
    close $tracefd
}

puts "Starting Simulation..."
$ns_ run

