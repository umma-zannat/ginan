#!/bin/bash

set -euo pipefail

# activate conda environment
eval "$(/root/.miniconda3/bin/conda shell.bash hook)"
conda activate gn37

# download example tests
cd /ginan
python scripts/download_examples.py

# run example tests
TEST_NUM=$1

# start mongo DB
#/bin/systemctl start /usr/bin/mongod
#@/bin/systemctl status /usr/bin/mongod
mkdir /ginan/db
/usr/bin/mongod --dbpath /ginan/db --bind_ip 127.0.0.1 &
sleep 5

cd /ginan/examples

# in all cases there is a pea.out file but no sensible way to compare it
case $TEST_NUM in
  1)
    pea --config ex11_pea_pp_user_gps.yaml | tee pea11.out
    tar cvfz run-results-ex11.tar.gz ex11 
    aws s3 cp run-results-ex11.tar.gz s3://ginan-pipeline-results/$TAG/
    python ../scripts/diffsnx.py   -i ex11/ex1120624.snx            -o solutions/ex11/ex1120624.snx
    sed -i 's/REC_SYS_BIAS/   REC_CLOCK/g' solutions/ex11/*.TRACE
    python ../scripts/difftrace.py -i ex11/ex11-ALIC201919900.TRACE -o solutions/ex11/ex11-ALIC201919900.TRACE
    ;;
  2)
    pea --config ex12_pea_pp_user_gnss.yaml | tee pea12.out
    tar cvfz run-results-ex12.tar.gz ex12 
    aws s3 cp run-results-ex12.tar.gz s3://ginan-pipeline-results/$TAG/
    python ../scripts/diffsnx.py   -i ex12/ex1220624.snx            -o solutions/ex12/ex1220624.snx
    sed -i 's/REC_SYS_BIAS/   REC_CLOCK/g' solutions/ex12/*.TRACE
    python ../scripts/difftrace.py -i ex12/ex12-ALIC201919900.TRACE -o solutions/ex12/ex12-ALIC201919900.TRACE
    ;;
  3)
    pea --config ex13_pea_pp_user_gps_sf.yaml | tee pea13.out
    tar cvfz run-results-ex13.tar.gz ex13 
    aws s3 cp run-results-ex13.tar.gz s3://ginan-pipeline-results/$TAG/
    python ../scripts/diffsnx.py   -i ex13/ex1320624.snx            -o solutions/ex13/ex1320624.snx
    sed -i 's/REC_SYS_BIAS/   REC_CLOCK/g' solutions/ex13/*.TRACE

    for trace in `ls ex13/*.TRACE`; do
    tracebase="$(basename $trace)";
    python ../scripts/difftrace.py -i $trace -o "solutions/ex13/$tracebase";
    done
    ;;
  4)
    pea --config ex14_pea_pp_user_gnss_ar.yaml | tee pea14.out
    tar cvfz run-results-ex14.tar.gz ex14 
    aws s3 cp run-results-ex14.tar.gz s3://ginan-pipeline-results/$TAG/
    python ../scripts/diffsnx.py   -i ex14/ex1420624.snx            -o solutions/ex14/ex1420624.snx
    sed -i 's/REC_SYS_BIAS/   REC_CLOCK/g' solutions/ex14/*.TRACE
    python ../scripts/difftrace.py -i ex14/TUG/ex14-ALIC201919900.TRACE -o solutions/ex14/TUG/ex14-ALIC201919900.TRACE
    ;;
  5)
    # realtime: needs auscors user/pass in config: skipped
    ## pea --config ex15_pea_rt_user_gnss_ar.yaml
    ;;
  6)
    pea --config ex16_pea_pp_ionosphere.yaml | tee pea16.out
    tar cvfz run-results-ex16.tar.gz ex16 
    aws s3 cp run-results-ex16.tar.gz s3://ginan-pipeline-results/$TAG/
    python ../scripts/diffionex.py  -i ex16/AUSG1990.19I            -o solutions/ex16/AUSG1990.19I
    # compare .SUM files 
    # python ../scripts/diffsnx.py -i ex16/ex1620624.snx -o solutions/ex16/ex1620624.snx #should be removed from yaml (?) as ESTIMATE blk empty
    # diff ex16/ex16_201919900.stec solutions/ex16/ex16_201919900.stec # Need update stec files format and push the stec utility
    #loop over 87 TRACE files. space in name so need to find trick to get it to work
    ;;
  7)
    pea --config ex17_pea_pp_netw_gnss_ar.yaml | tee pea17.out
    tar cvfz run-results-ex17.tar.gz ex17 
    aws s3 cp run-results-ex17.tar.gz s3://ginan-pipeline-results/$TAG/
    python ../scripts/diffsnx.py   -i ex17/ex1720624.snx            -o solutions/ex17/ex1720624.snx

    # for trace in `ls ex17/*.TRACE`; do
    # tracebase="$(basename $trace)"; #need to check what blocks to compare. $ or res are not persent
    # python ../scripts/difftrace.py -i $trace -o "solutions/ex1.7_/$tracebase";
    # done
    ;;
  8)
    # TODO not working:
    # pea --config ex18_pea_rt_netw_gnss_ar.yaml
    ;;
esac

