#!/usr/bin/env python

'''Utility for testing sinex files equivalence based on comparison of values in the SOLUTION/ESTIMATE blocks. Ignores other blocks and header info.
The functionality is based on assert_frame_equal method (https://pandas.pydata.org/docs/reference/api/pandas.testing.assert_frame_equal.html)'''
import argparse
import os as _os
import logging
import sys as _sys

from gn_lib.gn_io.ionex import read_ionex
from gn_lib.gn_io.trace import diff2msg

def parse_arguments():
    parser = argparse.ArgumentParser(description='Compares the content of two sinex files, specifically the SOLUTION/ESTIMATE blocks and returns errors on difference.')
    parser.add_argument('-i', '--ionex1', type=file_path,help='path to ionex file. Can be compressed with LZW (.Z) or gzip')
    parser.add_argument('-o', '--ionex2', type=file_path,help='path to another ionex file')
    parser.add_argument('-a', '--atol', type=float,help='absolute tolerance',default=None)
    parser.add_argument('-p', '--passthrough', action='store_true',help='passthrough or return 0 even if failed')
    return parser.parse_args()

def file_path(path):
    if _os.path.isfile(path):
        return path
    else:
        raise argparse.ArgumentTypeError(f"{path} is not a valid path")

def diffionex(ionex1_path,ionex2_path,atol,passthrough):
    '''Compares two sinex files. '''
    logging.getLogger().setLevel(logging.INFO)
    logging.info(f':diffsnx testing of {_os.path.basename(ionex1_path)}')

    ionex1_df = read_ionex(path_or_bytes=ionex1_path)          
    ionex2_df = read_ionex(path_or_bytes=ionex2_path)

    atol = 10**min(ionex1_df.attrs['EXPONENT'],ionex2_df.attrs['EXPONENT']) if atol is None else atol

    status = 0

    diff_ionex = (ionex1_df.unstack(['Type','Lat']) - ionex2_df.unstack(['Type','Lat'])).swaplevel('Lon','Type',axis=1) #output looks cleaner then

    bad_ionex_vals = diff2msg(diff_ionex,tol=atol,from_valvar=False)
    if bad_ionex_vals is not None:
        logging.warning(msg=f':diffionex found estimates diffs above 10^min(exp1,exp2) = {atol:.1E} tolerance (REQUIRED):\n{bad_ionex_vals.to_string(justify="center")}\n')
        status = -1
    else:
        logging.info(f':diffionex [OK] estimates diffs within 10^min(exp1,exp2) = {atol:.1E} tolerance (REQUIRED)')

    if status:
        logging.error(msg = f':diffionex failed [{_os.path.abspath(ionex1_path)}]\n')
        if not passthrough:
            _sys.exit(status)
        else:
            logging.error(msg = ':diffionex returning 0 as passthrough enabled\n')
            return 0

    logging.info(':diffionex [ALL OK]')

if __name__ == "__main__":
    parsed_args = parse_arguments()
    diffionex(ionex1_path=parsed_args.ionex1,ionex2_path=parsed_args.ionex2,atol=parsed_args.atol,passthrough=parsed_args.passthrough)
