#!/usr/bin/env python

'''Utility for testing sinex files equivalence based on comparison of values in the SOLUTION/ESTIMATE blocks. Ignores other blocks and header info.
The functionality is based on assert_frame_equal method (https://pandas.pydata.org/docs/reference/api/pandas.testing.assert_frame_equal.html)'''
import argparse
import os as _os
import logging
import sys as _sys

from gn_lib.gn_io.sinex import _get_snx_vector
from gn_lib.gn_io.trace import _valvar2diffstd, diff2msg

def parse_arguments():
    parser = argparse.ArgumentParser(description='Compares the content of two sinex files, specifically the SOLUTION/ESTIMATE blocks and returns errors on difference.')
    parser.add_argument('-i', '--snx1', type=file_path,help='path to sinex file (.snx/.ssc). Can be compressed with LZW (.Z)')
    parser.add_argument('-o', '--snx2', type=file_path,help='path to another sinex file')
    parser.add_argument('-a', '--atol', type=float,help='absolute tolerance',default=1E-4)
    parser.add_argument('-c', '--coef',   type=float,help='std coefficient',default=1)
    parser.add_argument('-p', '--passthrough', action='store_true',help='passthrough or return 0 even if failed')
    return parser.parse_args()

def file_path(path):
    if _os.path.isfile(path):
        return path
    else:
        raise argparse.ArgumentTypeError(f"{path} is not a valid path")

def diffsnx(snx1_path,snx2_path,atol,std_coeff,passthrough):
    '''Compares two sinex files. '''
    logging.getLogger().setLevel(logging.INFO)
    logging.info(f':diffsnx testing of {_os.path.basename(snx1_path)}')

    snx1_df = _get_snx_vector(path_or_bytes=snx1_path,stypes=('EST',),snx_format=True,verbose=False)             
    snx2_df = _get_snx_vector(path_or_bytes=snx2_path,stypes=('EST',),snx_format=True,verbose=False)
    status = 0

    diff_snx = _valvar2diffstd(snx1_df,snx2_df,trace=False,std_coeff=std_coeff).unstack(['CODE_PT','TYPE'])
    bad_snx_vals = diff2msg(diff_snx,tol=atol)
    if bad_snx_vals is not None:
        logging.warning(msg=f':diffsnx found estimates diffs above {atol:.1E} tolerance (OPTIONAL):\n{bad_snx_vals.to_string(justify="center")}\n')
        # status = -1
    else:
        logging.info(f':diffsnx [OK] estimates diffs within {atol:.1E} tolerance (OPTIONAL)')
    bad_snx_vals = diff2msg(diff_snx,tol=None)
    if bad_snx_vals is not None:
        logging.warning(msg=f':diffsnx found estimates diffs above the extracted STDs (REQUIRED):\n{bad_snx_vals.to_string(justify="center")}\n')
        status = -1
    else:
        logging.info(f':diffsnx [OK] estimates diffs within the extracted STDs (REQUIRED)')

    if status:
        logging.error(msg = f':diffsnx failed [{_os.path.abspath(snx1_path)}]\n')
        if not passthrough:
            _sys.exit(status)
        else:
            logging.error(msg = ':diffsnx returning 0 as passthrough enabled\n')
            return 0

    logging.info(':diffsnx [ALL OK]')

if __name__ == "__main__":
    parsed_args = parse_arguments()
    diffsnx(snx1_path=parsed_args.snx1,snx2_path=parsed_args.snx2,atol=parsed_args.atol,std_coeff=parsed_args.coef,passthrough=parsed_args.passthrough)
