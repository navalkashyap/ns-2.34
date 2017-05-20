#!/usr/bin/env perl

set grid
set term png
set title 'Daisy Chain Topology - Latency' font "Helvetica,20"
set output "daisychain_latency.png"
set xlabel "data generation interval (sec)" font "Helvetica,20"
set ylabel "latency (sec)" font "Helvetica,20"
plot [0.5:] [:1200] 'daisy_latency_node_3.log' title 'Number of nodes: 3' with lines, 'daisy_latency_node_7.log' title 'Number of nodes: 7' with lines

set grid
set term png
set title 'Daisy Chain Topology - Drop Ratio' font "Helvetica,24"
set output "daisychain_drop.png"
set xlabel "data generation interval (sec)" font "Helvetica,20"
set ylabel "drop ratio (%)" font "Helvetica,20"
plot [0.9:] [:100] 'daisy_drop_node_3.log' title 'Number of nodes: 3' with lines, 'daisy_drop_node_7.log' title 'Number of nodes: 7' with lines


