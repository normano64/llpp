#!/bin/bash
rm -f ./testdata.txt
RUNS=2
THREADS=4
if [[ $1 =~ ^-?[0-9]+$ ]] && [ $1 > 0 ]; then
    RUNS=$1
    if [[ $2 =~ ^-?[0-9]+$ ]] && [ $2 > 0 ]; then
        THREADS=$2
    fi
fi
printf "Test of %d runs with 1-%d threads.\n" $RUNS $THREADS 
for i in $(seq 1 $THREADS);
do
    if [ "$i" == 1 ]; then
        printf "Running with %d thread" $i
        for n in $(seq 1 $RUNS);
        do
            printf .
            LD_LIBRARY_PATH=. ./demo --silent --timing-mode --plot --threads 1
        done
    else
        printf "Running with %d threads" $i
        for n in $(seq 1 $RUNS);
        do
            printf .
            LD_LIBRARY_PATH=. ./demo --silent --timing-mode --pthread --plot --threads "$i"
        done
        for n in $(seq 1 $RUNS);
        do
            printf .
            LD_LIBRARY_PATH=. ./demo --silent --timing-mode --omp --plot --threads "$i"
        done
    fi
    echo ""
done
echo ""
echo "Tests complete!"
