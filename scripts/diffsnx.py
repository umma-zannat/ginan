#!/usr/bin/env python

'''Utility for testing sinex files equivalence based on comparison of values in the SOLUTION/ESTIMATE blocks. Ignores other blocks and header info.
The functionality is based on assert_frame_equal method (https://pandas.pydata.org/docs/reference/api/pandas.testing.assert_frame_equal.html)'''
import argparse
import os as _os
import numpy as _np
import logging
import sys as _sys

from gn_lib.gn_io.sinex import _read_snx_solution
from gn_lib.gn_io.trace import diff2msg

def parse_arguments():
    parser = argparse.ArgumentParser(description='Compares the content of two sinex files, specifically the SOLUTION/ESTIMATE blocks and returns errors on difference.')
    parser.add_argument('-i', '--snx1', type=file_path,help='path to sinex file (.snx/.ssc). Can be compressed with LZW (.Z)')
    parser.add_argument('-o', '--snx2', type=file_path,help='path to another sinex file')
    parser.add_argument('-a', '--atol', type=float,help='absolute tolerance',default=1E-4)
    parser.add_argument('-p', '--passthrough', action='store_true',help='passthrough or return 0 even if failed')
    return parser.parse_args()

def file_path(path):
    if _os.path.isfile(path):
        return path
    else:
        raise argparse.ArgumentTypeError(f"{path} is not a valid path")

def diffsnx(snx1_path,snx2_path,atol,passthrough):
    '''Compares two sinex files. '''
    logging.getLogger().setLevel(logging.INFO)
    logging.info(f':diffsnx testing of {_os.path.basename(snx1_path)}')

    snx1_df = _read_snx_solution(path_or_bytes=snx1_path).unstack(0)
    snx2_df = _read_snx_solution(path_or_bytes=snx2_path).unstack(0)

    bad_snx_vals = diff2msg(snx1_df - snx2_df,tol=atol)
    if bad_snx_vals is not None:
        logging.warning(msg=f':diffsnx found estimates diffs above the tolerance ({atol:.1E}):\n{bad_snx_vals.to_string()}\n')
        logging.error(msg = ':difftrace test failed\n')
        if not passthrough:
            _sys.exit(-1)
        else:
            logging.error(msg = ':diffsnx returning 0 as passthrough enabled\n')
            return 0
        
    else: logging.info(f':diffsnx [OK] estimates diffs within {atol:.1E} tolerance')
    logging.info(':diffsnx [ALL OK]')

if __name__ == "__main__":
    parsed_args = parse_arguments()
    diffsnx(snx1_path=parsed_args.snx1,snx2_path=parsed_args.snx2,atol=parsed_args.atol,passthrough=parsed_args.passthrough)
