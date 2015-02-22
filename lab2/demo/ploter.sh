#!/bin/bash
rm -f ./testdata.txt ./data.txt
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
    printf "Running with %d threads" $i
    if [ "$i" == 1 ]; then
        rm -f ./testdata.txt
        for n in $(seq 1 $RUNS);
        do
            printf .
            LD_LIBRARY_PATH=. ./demo scenario_small.xml --silent --timing-mode --plot --threads "$i"
        done
        FRACTION="$(awk '{total += $6} END {total=total/'$RUNS';print total}' testdata.txt)"
        awk '{total += $6} END {total=(total/'$RUNS');print "seq     \t\t('$i',"total")"}' testdata.txt >> data.txt
        rm -f ./testdata.txt
        for n in $(seq 1 $RUNS);
        do
            printf .
            LD_LIBRARY_PATH=. ./demo scenario_small.xml --silent --timing-mode --plot --threads "$i" --simd
        done
        awk '{total += $6} END {total=(total/'$RUNS')/'$FRACTION';print "seq     \tsimd\t('$i',"total")"}' testdata.txt >> data.txt
        rm -f ./testdata.txt
        for n in $(seq 1 $RUNS);
        do
            printf .
            LD_LIBRARY_PATH=. ./demo scenario_small.xml --silent --timing-mode --opencl --plot
        done
        awk '{total += $6} END {total=(total/'$RUNS')/'$FRACTION';print "opencl  \t\t('$i',"total")"}' testdata.txt >> data.txt
    fi
    rm -f ./testdata.txt
    for n in $(seq 1 $RUNS);
    do
        printf .
        LD_LIBRARY_PATH=. ./demo scenario_small.xml --silent --timing-mode --pthreads --plot --threads "$i"
    done
    awk '{total += $6} END {total=(total/'$RUNS')/'$FRACTION';print "pthreads\t\t('$i',"total")"}' testdata.txt >> data.txt
    rm -f ./testdata.txt
    for n in $(seq 1 $RUNS);
    do
        printf .
        LD_LIBRARY_PATH=. ./demo scenario_small.xml --silent --timing-mode --pthreads --plot --threads "$i" --simd
    done
    awk '{total += $6} END {total=(total/'$RUNS')/'$FRACTION';print "pthreads\tsimd\t('$i',"total")"}' testdata.txt >> data.txt
    rm -f ./testdata.txt
    for n in $(seq 1 $RUNS);
    do
        printf .
        LD_LIBRARY_PATH=. ./demo scenario_small.xml --silent --timing-mode --omp --plot --threads "$i"
    done
    awk '{total += $6} END {total=(total/'$RUNS')/'$FRACTION';print "omp     \t\t('$i',"total")"}' testdata.txt >> data.txt
    rm -f ./testdata.txt
    for n in $(seq 1 $RUNS);
    do
        printf .
        LD_LIBRARY_PATH=. ./demo scenario_small.xml --silent --timing-mode --omp --plot --threads "$i" --simd
    done
    awk '{total += $6} END {total=(total/'$RUNS')/'$FRACTION';print "omp     \tsimd\t('$i',"total")"}' testdata.txt >> data.txt
    rm -f ./testdata.txt
    echo ""
done
echo "Tests complete!"
