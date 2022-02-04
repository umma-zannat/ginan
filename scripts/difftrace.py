#!/usr/bin/env python

'''Utility for testing trace files equivalence. Checks estimate blocks (marked with $) and SV residuals (+-Residuals)
The functionality is based on assert_frame_equal method (https://pandas.pydata.org/docs/reference/api/pandas.testing.assert_frame_equal.html)'''
import argparse
import os as _os
import sys as _sys
import logging

from gn_lib.gn_io.trace import _read_trace_states, _read_trace_residuals,_compare_gnss_states,_compare_postrop_states,_compare_recsysbias_states,_compare_postfit_residuals, _valvar2diffstd
from gn_lib.gn_io.common import path2bytes

def parse_arguments():
    parser = argparse.ArgumentParser(description='Compares the content of two sinex files, specifically the SOLUTION/ESTIMATE blocks and returns errors on difference.')
    parser.add_argument('-i', '--trace1', type=file_path,help='path to trace file (.trace). Can be compressed with LZW (.Z) or gzip (.gz)')
    parser.add_argument('-o', '--trace2', type=file_path,help='path to another trace file')
    parser.add_argument('-a', '--atol',   type=float,help='absolute tolerance',default=1E-4)
    parser.add_argument('-c', '--coef',   type=float,help='std coefficient',default=1)
    parser.add_argument('-p', '--passthrough', action='store_true',help='passthrough or return 0 even if failed')
    return parser.parse_args()

def file_path(path):
    if _os.path.isfile(path):
        return path
    else:
        raise argparse.ArgumentTypeError(f"{path} is not a valid path")

def difftrace(trace1_path,trace2_path,atol,passthrough,std_coeff):
    '''Creates sinex station map html'''
    logging.getLogger().setLevel(logging.INFO)
    logging.info(f':difftrace testing of {_os.path.basename(trace1_path)}')
    
    trace1 = path2bytes(trace1_path)
    trace2 = path2bytes(trace2_path)

    # rtol, default 1e-5. Relative tolerance.
    # atol, default 1e-8. Absolute tolerance.
    
    states1 = _read_trace_states(trace1)
    states2 = _read_trace_states(trace2)

    resids1 = _read_trace_residuals(trace1)
    resids2 = _read_trace_residuals(trace2)

    diffstd_states    = _valvar2diffstd(states1.iloc[:,:2],states2.iloc[:,:2],std_coeff=std_coeff)
    diffstd_residuals = _valvar2diffstd(resids1.iloc[:,2:],resids2.iloc[:,2:],std_coeff=std_coeff)


    status = 0
    

    if diffstd_states.attrs['EXTRA_SATS'] != []:
        logging.warning(msg=f':difftrace found no counterpart for: {sorted(diffstd_states.attrs["EXTRA_SATS"])}')

    bad_sv_states = _compare_gnss_states(diffstd_states,tol=atol)
    if bad_sv_states is not None:
        logging.warning(msg=f':difftrace found SV states diffs above {atol:.1E} tolerance (OPTIONAL):\n{bad_sv_states.to_string(justify="center")}\n')
        # status = -1
    else: logging.info(f':difftrace [OK] SVs states diffs within {atol:.1E} tolerance (OPTIONAL)')

    bad_sv_states = _compare_gnss_states(diffstd_states,tol=None)
    if bad_sv_states is not None:
        logging.warning(msg=f':difftrace found SV states diffs above the extracted STDs (REQUIRED):\n{bad_sv_states.to_string(justify="center")}\n')
        status = -1
    else: logging.info(f':difftrace [OK] SVs states diffs within the extracted STDs (REQUIRED)')

    bad_postrop_states = _compare_postrop_states(diffstd_states,tol=atol)
    if bad_postrop_states is not None:
        logging.warning(msg=f':difftrace found POS and TROP diffs above {atol:.1E} tolerance (OPTIONAL):\n{bad_postrop_states.to_string(justify="center")}\n')
        # status = -1
    else: logging.info(f':difftrace [OK] SVs POS and TROP states diffs within {atol:.1E} tolerance (OPTIONAL)')

    bad_postrop_states = _compare_postrop_states(diffstd_states,tol=None)
    if bad_postrop_states is not None:
        logging.warning(msg=f':difftrace found POS and TROP diffs above the extracted STDs (REQUIRED):\n{bad_postrop_states.to_string(justify="center")}\n')
        status = -1
    else: logging.info(f':difftrace [OK] SVs POS and TROP states diffs within the extracted STDs (REQUIRED)')

    bad_recsysbias_states = _compare_recsysbias_states(diffstd_states,tol=atol)
    if bad_recsysbias_states is not None:
        logging.warning(msg=f':difftrace found REC_SYS_BIAS diffs above {atol:.1E} tolerance (OPTIONAL):\n{bad_recsysbias_states.to_string(justify="center")}\n')
        # status = -1
    else: logging.info(f':difftrace [OK] REC_SYS_BIAS states diffs within {atol:.1E} tolerance (OPTIONAL)')

    bad_recsysbias_states = _compare_recsysbias_states(diffstd_states,tol=None)
    if bad_recsysbias_states is not None:
        logging.warning(msg=f':difftrace found REC_SYS_BIAS diffs above the extracted STDs (REQUIRED):\n{bad_recsysbias_states.to_string(justify="center")}\n')
        status = -1
    else: logging.info(f':difftrace [OK] REC_SYS_BIAS states diffs within the extracted STDs (REQUIRED)')

    bad_sv_residuals = _compare_postfit_residuals(diffstd_residuals,tol=atol)
    if bad_sv_residuals is not None:
        logging.warning(msg=f':difftrace found SV residuals diffs above {atol:.1E} tolerance (OPTIONAL):\n{bad_sv_residuals.to_string(justify="center")}\n')
        # status = -1
    else: logging.info(f':difftrace [OK] SVs residuals diffs within {atol:.1E} tolerance (OPTIONAL)')

    bad_sv_residuals = _compare_postfit_residuals(diffstd_residuals,tol=None)
    if bad_sv_residuals is not None:
        logging.warning(msg=f':difftrace found SV residuals diffs above the extracted STDs (REQUIRED):\n{bad_sv_residuals.to_string(justify="center")}\n')
        status = -1
    else: logging.info(f':difftrace [OK] SVs residuals diffs within the extracted STDs (REQUIRED)')

    if status:
        logging.error(msg = f':difftrace failed [{_os.path.abspath(trace1_path)}]\n')
        if not passthrough:
            _sys.exit(status)
        else:
            logging.error(msg = ':difftrace returning 0 as passthrough enabled\n')
            return 0

    logging.info(':difftrace [ALL OK]')

if __name__ == "__main__":
    parsed_args = parse_arguments()
    difftrace(trace1_path=parsed_args.trace1,trace2_path=parsed_args.trace2,atol=parsed_args.atol,passthrough=parsed_args.passthrough,std_coeff=parsed_args.coef)
