#!/bin/bash

set -euo pipefail

# activate conda environment
eval "$(/root/.miniconda3/bin/conda shell.bash hook)"
conda activate gn37

# download example tests
cd /ginan
python scripts/download_examples.py

# run example tests
cd /ginan/examples

TEST_NUM=$1

case $TEST_NUM in
  1)
    pod -y ex21_pod_fit_gps.yaml | tee pod21.out
    mv pod21.out ex21/pod.out
    cd ex21
    python ../../scripts/rms_bar_plot.py -i gag19424_igs19424_orbdiff_rtn.out -d . -c G >pod.rms
    python ../../scripts/compare_pod_rms.py -ro pod.out -rr pod.rms -so ../solutions/ex21/pod.out -sr ../solutions/ex21/pod.rms -em 0.0002
    ;;
  2g) # ex22 GPS
    pod -y ex22_pod_fit_gps.yaml | tee pod22g.out
    mv pod22g.out ex22/gps/pod.out
    cd ex22/gps
    python ../../../scripts/rms_bar_plot.py -i gag20624__orbdiff_rtn.out -d . -c G >pod_G.rms
    python ../../../scripts/compare_pod_rms.py -ro pod.out -rr pod_G.rms -so ../../solutions/ex22/gps/pod.out -sr ../../solutions/ex22/gps/pod_G.rms -em 0.0002
    ;;
  2r) # ex22 GLONASS
    pod -y ex22_pod_fit_glo.yaml | tee pod22r.out
    mv pod22r.out ex22/glo/pod.out
    cd ex22/glo
    python ../../../scripts/rms_bar_plot.py -i gag20624__orbdiff_rtn.out -d . -c R >pod_R.rms
    python ../../../scripts/compare_pod_rms.py -ro pod.out -rr pod_R.rms -so ../../solutions/ex22/glo/pod.out -sr ../../solutions/ex22/glo/pod_R.rms -em 0.0002
    ;;
  2e) # ex22 GALILEO
    pod -y ex22_pod_fit_gal.yaml | tee pod22e.out
    mv pod22e.out ex22/gal/pod.out
    cd ex22/gal
    python ../../../scripts/rms_bar_plot.py -i gag20624__orbdiff_rtn.out -d . -c E >pod_E.rms
    python ../../../scripts/compare_pod_rms.py -ro pod.out -rr pod_E.rms -so ../../solutions/ex22/gal/pod.out -sr ../../solutions/ex22/gal/pod_E.rms -em 0.0002
    ;;
  2c) # ex22 BEIDOU
    pod -y ex22_pod_fit_bds.yaml | tee pod22c.out
    mv pod22c.out ex22/bds/pod.out
    cd ex22/bds
    python ../../../scripts/rms_bar_plot.py -i gag20624__orbdiff_rtn.out -d . -c C >pod_C.rms
    python ../../../scripts/compare_pod_rms.py -ro pod.out -rr pod_C.rms -so ../../solutions/ex22/bds/pod.out -sr ../../solutions/ex22/bds/pod_C.rms -em 0.0002
    ;;
  2j) # ex22 QZSS
    pod -y ex22_pod_fit_qzss.yaml | tee pod22j.out
    mv pod22j.out ex22/qzss/pod.out
    cd ex22/qzss
    python ../../../scripts/rms_bar_plot.py -i gag20624__orbdiff_rtn.out -d . -c J >pod_J.rms
    python ../../../scripts/compare_pod_rms.py -ro pod.out -rr pod_J.rms -so ../../solutions/ex22/qzss/pod.out -sr ../../solutions/ex22/qzss/pod_J.rms -em 0.0002
    ;;
  3)
    pod -y ex23_pod_prd_gps.yaml | tee pod23.out
    mv pod23.out ex23/pod.out
    cd ex23
    python ../../scripts/rms_bar_plot.py -i gag20010_igs20010_orbdiff_rtn.out -d . -c G >pod.rms
    python ../../scripts/compare_pod_rms.py -ro pod.out -rr pod.rms -so ../solutions/ex23/pod.out -sr ../solutions/ex23/pod.rms -em 0.0002
    ;;
  4)
    pod -y ex24_pod_ic_gps.yaml | tee pod24.out
    mv pod24.out ex24/pod.out
    cd ex24
    python ../../scripts/rms_bar_plot.py -i gag20624_igs20624_orbdiff_rtn.out -d . -c G >pod.rms
    python ../../scripts/compare_pod_rms.py -ro pod.out -rr pod.rms -so ../solutions/ex24/pod.out -sr ../solutions/ex24/pod.rms -em 0.0002
    ;;
  5)
    pod -y ex25_pod_fit_gps.yaml | tee pod25.out
    mv pod25.out ex25/pod.out
    cd ex25
    python ../../scripts/rms_bar_plot.py -i gag19424_igs19424_orbdiff_rtn.out -d . -c G >pod.rms
    python ../../scripts/compare_pod_rms.py -ro pod.out -rr pod.rms -so ../solutions/ex25/pod.out -sr ../solutions/ex25/pod.rms -em 0.0002
    ;;
  6)
    pod -y ex26_pod_fit_meo.yaml | tee pod26.out
    mv pod26.out ex26/pod.out
    cd ex26
    python ../../scripts/rms_bar_plot.py -i gag21330_ilrsa.orb.lageos1.201128.v70_orbdiff_rtn.out -d . -c G >pod.rms
    python ../../scripts/compare_pod_rms.py -ro pod.out -rr pod.rms -so ../solutions/ex26/pod.out -sr ../solutions/ex26/pod.rms -em 0.002
    ;;
esac

