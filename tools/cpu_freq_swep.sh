#!/bin/bash

if [ $# -eq 0 ]; then
	echo "sweeping all freqs"
	FREQS=(346000 499000 653000 806000 960000 1110000 1270000 1420000
	1570000 1730000 1880000 2040000)
else
	echo "sweeping freqs: " $@
	FREQS=$@
fi

NUM_CORES=(0 1 2 3 4 5)

cpufreq-set -g userspace

trap "exit" INT TERM ERR
trap "kill 0" EXIT

for i in ${FREQS[@]}; do
	echo "setting cpu_freq to $i"
	for j in ${NUM_CORES[@]}; do
		cpufreq-set -c $j -d $(( i - 1000 )) -u $i
	done
	
	echo "running retinanetTestCPU.py > time.log"
	python3 jetsrc/retinanetTestCPU.py > tools/cpufreq_logs/cpufreq_${i}_time.log &
	thing=$!
	
	echo "running tegrastats > stats.log"
	~/tegrastats --interval 100 --logfile tools/cpufreq_logs/cpufreq_${i}_stats.log &
	wait $thing
	~/tegrastats --stop

	echo "parsing tegrastats for" $i
	python3 tools/statParser.py --time tools/cpufreq_logs/cpufreq_${i}_time.log --stats tools/cpufreq_logs/cpufreq_${i}_stats.log

done
