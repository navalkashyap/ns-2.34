
set grid
set term png
set title 'One-on-One Topology - Latency'  font "Helvetica,24"
set output "one_to_one_latency.png"
set xlabel "data generation interval (sec)" font "Helvetica,20"
set ylabel "latency (sec)" font "Helvetica,20"
plot [0.5:] [:250] 'oneOnly_latency.log' title 'Number of nodes: 2' with lines

set grid
set term png
set title 'One-on-One Topology - Drop ratio'  font "Helvetica,24"
set output "one_to_one_drop.png"
set xlabel "data generation interval (sec)" font "Helvetica,20"
set ylabel "drop ratio (%)" font "Helvetica,20"
plot [0.5:] [:100] 'oneOnly_drop.log' title 'Number of nodes: 2' with lines

