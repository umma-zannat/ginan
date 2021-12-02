#!/usr/bin/env python

'''Utility for testing trace files equivalence. Checks estimate blocks (marked with $) and SV residuals (+-Residuals)
The functionality is based on assert_frame_equal method (https://pandas.pydata.org/docs/reference/api/pandas.testing.assert_frame_equal.html)'''
import argparse
import os as _os
import sys as _sys
import numpy as _np
import logging

from gn_lib.gn_io.trace import _read_trace_states, _read_trace_residuals,_compare_gnss_states,_compare_postrop_states,_compare_recsysbias_states,_compare_postfit_residuals
from gn_lib.gn_io.common import path2bytes

def parse_arguments():
    parser = argparse.ArgumentParser(description='Compares the content of two sinex files, specifically the SOLUTION/ESTIMATE blocks and returns errors on difference.')
    parser.add_argument('-i', '--trace1', type=file_path,help='path to trace file (.trace). Can be compressed with LZW (.Z) or gzip (.gz)')
    parser.add_argument('-o', '--trace2', type=file_path,help='path to another trace file')
    parser.add_argument('-a', '--atol',   type=file_path,help='absolute tolerance',default=1E-4)
    parser.add_argument('-p', '--passthrough', action='store_true',help='passthrough or return 0 even if failed')
    return parser.parse_args()

def file_path(path):
    if _os.path.isfile(path):
        return path
    else:
        raise argparse.ArgumentTypeError(f"{path} is not a valid path")

def difftrace(trace1_path,trace2_path,atol,passthrough):
    '''Creates sinex station map html'''
    logging.getLogger().setLevel(logging.INFO)
    logging.info(f':difftrace testing of {_os.path.basename(trace1_path)}')
    
    trace1 = path2bytes(trace1_path)
    trace2 = path2bytes(trace2_path)

    # rtol, default 1e-5. Relative tolerance.
    # atol, default 1e-8. Absolute tolerance.
    
    states1 = _read_trace_states(trace1)
    states2 = _read_trace_states(trace2)

    residuals1 = _read_trace_residuals(trace1)
    residuals2 = _read_trace_residuals(trace2)
    status = 0
    
    extra_sv, bad_sv_states = _compare_gnss_states(states1,states2,tol=atol)
    if extra_sv != []:
        logging.warning(msg=f':difftrace found no counterpart for: {sorted(extra_sv)}')
        status = -1
    else: logging.info(msg=f':difftrace [OK] SVs consistency within {atol:.1E} tolerance')

    if bad_sv_states is not None:
        logging.warning(msg=f':difftrace found SV states diffs above the tolerance ({atol:.1E}):\n{bad_sv_states.to_string()}\n')
        status = -1
    else: logging.info(f':difftrace [OK] SVs states diffs within {atol:.1E} tolerance')

    bad_postrop_states = _compare_postrop_states(states1,states2,tol=atol)
    if bad_postrop_states is not None:
        logging.warning(msg=f':difftrace found POS and TROP diffs above the tolerance ({atol:.1E}):\n{bad_postrop_states.to_string()}\n')
        status = -1
    else: logging.info(f':difftrace [OK] SVs POS and TROP states diffs within {atol:.1E} tolerance')

    bad_recsysbias_states = _compare_recsysbias_states(states1,states2,tol=atol)
    if bad_recsysbias_states is not None:
        logging.warning(msg=f':difftrace found REC_SYS_BIAS diffs above the tolerance ({atol:.1E}):\n{bad_recsysbias_states.to_string()}\n')
        status = -1
    else: logging.info(f':difftrace [OK] REC_SYS_BIAS states diffs within {atol:.1E} tolerance')

    bad_sv_residuals = _compare_postfit_residuals(residuals1,residuals2,tol=atol)
    if bad_sv_residuals is not None:
        logging.warning(msg=f':difftrace found SV residuals diffs above the tolerance ({atol:.1E}):\n{bad_sv_residuals.to_string()}\n')
        status = -1
    else: logging.info(f':difftrace [OK] SVs residuals diffs within {atol:.1E} tolerance')

    if status:
        logging.error(msg = ':difftrace test failed\n')
        if not passthrough:
            _sys.exit(status)
        else:
            logging.error(msg = ':difftrace returning 0 as passthrough enabled\n')
            return 0

    logging.info(':difftrace [ALL OK]')

if __name__ == "__main__":
    parsed_args = parse_arguments()
    difftrace(trace1_path=parsed_args.trace1,trace2_path=parsed_args.trace2,atol=parsed_args.atol,passthrough=parsed_args.passthrough)
