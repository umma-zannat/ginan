'''TRACE file parser. Note the separate functions for values and residuals'''
import os as _os
import re as _re
from io import BytesIO as _BytesIO
from itertools import product
from multiprocessing import Pool as _Pool

import numpy as _np
import pandas as _pd

from ..gn_datetime import gpsweeksec2datetime as _gpsweeksec2datetime, _J2000_ORIGIN
from ..gn_const import PL_TYPE,PRN_CATEGORY, STATE_TYPES_CATEGORY
from .common import path2bytes

def _trace_extract(path_or_bytes,blk_name):
    # 'States', 'Residuals'
    blks_supported = ['States','Residuals']
    assert blk_name in blks_supported, f'"{blk_name}" blk not supported. Select one of {blks_supported}'

    trace_bytes = path2bytes(path_or_bytes) #path2bytes passes through bytes 

    begin = end = 0
    buf=[]

    blk_begin = (f'+ {blk_name}\n').encode()
    blk_end   = (f'- {blk_name}'  ).encode()

    while True:
        begin = trace_bytes.find(blk_begin,end)
        if begin==-1:
            break
        end = trace_bytes.find(blk_end,begin)
        buf.append(trace_bytes[begin+len(blk_begin):end])
    return b''.join(buf)

def _read_trace_states(path_or_bytes):
    states = _trace_extract(path_or_bytes,blk_name='States')
    df = _pd.read_csv(_BytesIO(states),delimiter='\t',usecols=[1,2,3,4,5,6,7,8],skipinitialspace=True,dtype={'SAT':PRN_CATEGORY,'TYPE':STATE_TYPES_CATEGORY},keep_default_na=False,
                    comment='#',header=None,names = ['TIME','TYPE','SITE','SAT','NUM','EST','VAR','ADJ'],parse_dates=['TIME']) # type:ignore
    df.TIME = (df.TIME.values - _J2000_ORIGIN).astype('timedelta64[s]').astype(int)

    return df.set_index(['TIME','SITE','TYPE','SAT','NUM'])

def _read_trace_residuals(path_or_bytes,it_max_only=True):
    residuals = _trace_extract(path_or_bytes,blk_name='Residuals')
    df = _pd.read_csv(_BytesIO(residuals),delimiter='\t',comment='#',header=None,usecols=[1,2,3,4,5,6,7,8],skipinitialspace=True,keep_default_na=False,
            names = ['It','TIME','SITE','SAT','TYPE','PREFIT','POSTFIT','STD'],parse_dates=['TIME'],dtype={'It':int,'SAT':PRN_CATEGORY,'TYPE':PL_TYPE}) # type:ignore

    df.TIME = (df.TIME.values - _J2000_ORIGIN).astype('timedelta64[s]').astype(int)
    if not it_max_only:
        return df.set_index(['TIME','SITE','TYPE','SAT'])
    # to get max_ind values pandas >= 1.1 is required
    it_max_ind=df[['TIME','It']].groupby(['TIME']).max().reset_index().values.tolist()
    return df.set_index(['TIME','It']).loc[it_max_ind].reset_index().set_index(['TIME','SITE','TYPE','SAT'])


def _valvar2diffstd(valvar1,valvar2,trace=True,std_coeff=1):
    df = _pd.concat([valvar1,valvar2],axis=0,keys=['valvar1','valvar2']).unstack(0) #fastest\
    df_nd = df.values
    diff = df_nd[:,0] - df_nd[:,1]
    nan_mask = ~_np.isnan(diff)

    diff = diff[nan_mask]
    std = std_coeff*_np.sqrt((df_nd[:,3] + df_nd[:,2])[nan_mask])
    df_combo = _pd.DataFrame(_np.vstack([diff,std]).T,columns=['DIFF','STD'],index=df.index[nan_mask])

    if trace:
        sats = df.index.get_level_values(3)
        sats_mask = ~sats.isna()
        sats_df = sats[sats_mask].unique()
        
        df_combo.attrs['SAT_MASK'] = sats_mask[nan_mask]
        sats_common = sats[sats_mask & nan_mask]#.unique()
        df_combo.attrs['EXTRA_SATS'] = sats_df[~sats_df.isin(sats_common)].to_list() # is [] if none
    return df_combo

def diff2msg(diff, tol = None,from_valvar=True):
    _pd.set_option("display.max_colwidth", 10000)

    if from_valvar: #if from_valvar else diff.values
        diff_df = diff.DIFF
        std_df  = diff.STD
        std_vals = std_df.values
    else:
        diff_df = diff
        assert tol is not None

    count_total = (~_np.isnan(diff_df.values)).sum(axis=0)
    mask2d_over_threshold = (_np.abs(diff_df) > (std_vals if tol is None else tol))

    diff_count = mask2d_over_threshold.sum(axis=0)
    
    mask = diff_count.astype(bool)
    if mask.sum() == 0:
        return None
    mask_some_vals = mask[mask.values].index
    # print(diff_count[mask])

    diff_over = diff_df[mask2d_over_threshold][mask_some_vals]
    idx_max = diff_over.idxmax()
    diff_max = _pd.Series(_np.diag(diff_over.loc[idx_max.values].values),index=idx_max.index)
    idx_min = diff_over.idxmin()
    diff_min = _pd.Series(_np.diag(diff_over.loc[idx_min.values].values),index=idx_min.index)

    if from_valvar:
        std_over  =  std_df[mask2d_over_threshold][mask_some_vals]
        std_max = _pd.Series(_np.diag(std_over.loc[idx_max.values].values),index=idx_max.index)
        std_min = _pd.Series(_np.diag(std_over.loc[idx_min.values].values),index=idx_min.index)
    # return idx_min
    msg = _pd.DataFrame()
    msg['RATIO'] =  (diff_count[mask].astype(str).astype(object) + '/' + count_total[mask].astype(str) 
    + ('(' + (diff_count[mask]/count_total[mask] * 100).round(2).astype(str)).str.ljust(5,fillchar='0') +'%)')

    msg['DIFF/MIN_DIFF'] = (diff_min.round(4).astype(str)
    +('±' + std_min.round(4).astype(str).str.ljust(6,fillchar='0') if from_valvar else '') + ' @' + (idx_min.values + _J2000_ORIGIN).astype(str))

    if (diff_count[mask]>1).sum()>0:
        msg['MAX_DIFF'] = (diff_max.round(4).astype(str).str.rjust(7) 
        +('±' + std_max.round(4).astype(str).str.ljust(6,fillchar='0') if from_valvar else '') + ' @' + (idx_max.values + _J2000_ORIGIN).astype(str)) * (diff_count[mask]>1)
        
        msg['MEAN_DIFF'] = (diff_over.mean(axis=0).round(4).astype(str)
        +'±' + diff_over.std(axis=0).round(4).astype(str).str.ljust(6,fillchar='0'))* (diff_count[mask]>1)

    return msg

def _compare_gnss_states(diffstd,tol=None):
    p_bias_diff = diffstd[diffstd.attrs['SAT_MASK']].droplevel([1,2,4]).unstack(1)
    return diff2msg(p_bias_diff,tol)

def _compare_postrop_states(diffstd,tol=None):
    pos_trop_diff = diffstd.droplevel(level = 'SAT',axis=0)[~diffstd.attrs['SAT_MASK']].loc[:,:,['REC_POS','TROP']].unstack(['SITE','TYPE','NUM'])
    return diff2msg(pos_trop_diff,tol=tol)

def _compare_recsysbias_states(diffstd,tol=None):
    recsysbias_diff = diffstd[~diffstd.attrs['SAT_MASK']].droplevel('SAT').loc[:,:,'REC_CLOCK'].unstack(['SITE','NUM'])
    return diff2msg(recsysbias_diff,tol=tol)

def _compare_postfit_residuals(diffstd,tol=None):
    diff_count =  diffstd.droplevel('SITE').unstack(['TYPE','SAT'])
    return diff2msg(diff_count,tol)


_RE_TRACE_HEAD = _re.compile(
    rb'station\s*\:\s*(.{4})\n\w+\s*\:\s*(.+|)\n\w+\s*\:\s*(.+|)\n\w+\s*\:\s*(\d)\n\w+\s*\:\s*(.+)')
_RE_TRACE_LC = _re.compile(rb'PDE\sform\sLC.+((?:\n.+)+)')
_RE_EL = _re.compile(rb'\*2 PDE-CS GPST\s+\w+\s+(\d+)\s+(\d+).0\s+(\w\d\d)\s+(\d+.\d+)')

def _find_trace(output_path: str) -> tuple:
    '''Scans output path for TRACE files'''
    station_names = set()
    trace_paths = []
    _re_station_name = _re.compile(r'\-(.{4})\d+.TRACE')

    for file in _os.scandir(path=output_path):
        if file.path.endswith('TRACE'):
            station_names.add(_re_station_name.findall(file.path)[0])
            trace_paths.append(file.path)

    station_names = sorted(station_names)
    trace_paths = sorted(trace_paths)
    return station_names, trace_paths

def _read_trace_LC(path_or_bytes):
    '''Parses the LC combo block of the trace files producing
     a single dataframe. WORK-IN-PROGRESS'''    
    # regex search string
    if isinstance(path_or_bytes, str):
        trace_content = path2bytes(path_or_bytes) # will accept .trace.Z also
    else:
        trace_content = path_or_bytes
    trace_LC_list = _RE_TRACE_LC.findall(string=trace_content)
    LC_bytes = b''.join(trace_LC_list)
    LC_bytes = LC_bytes.replace(b'=',b'') #getting rif of '='
    
    df_LC = _pd.read_csv(_BytesIO(LC_bytes),delim_whitespace=True,header=None,usecols=[1,2,4,6,8,9,10,11,12,13]).astype(
        {
            1: _np.int16, 2:_np.int32, 4: '<U3',
            6: '<U1', 8: '<U4',
            9: _np.float_, 10: '<U4', 11: _np.float_,
            12: '<U4', 13: _np.float_
        })
    
    df_LC.columns = ['W','S','PRN','LP',8,9,10,11,12,13]
    df_LC['time'] = _gpsweeksec2datetime(gps_week = df_LC.W, 
                                                tow = df_LC.S, 
                                                as_j2000=True)
    df_LC.drop(columns=['W','S'],inplace=True)

    df1 = df_LC[['time','PRN','LP',8,9]]
    df1.columns = ['time','PRN','LP','combo','value']

    df2 = df_LC[['time','PRN','LP',10,11]]
    df2.columns = ['time','PRN','LP','combo','value']

    df3 = df_LC[['time','PRN','LP',12,13]]
    df3.columns = ['time','PRN','LP','combo','value']

    df_LC = _pd.concat([df1,df2,df3],axis=0)
    
    
    return df_LC.set_index(['time'])

def _read_trace_el(path_or_bytes):
    "Get elevation angles for satellites from trace file"
    if isinstance(path_or_bytes, str):
        trace_content = path2bytes(path_or_bytes) # will accept .trace.Z also
    else:
        trace_content = path_or_bytes
    trace_EL_list = _RE_EL.findall(string=trace_content)
    
    el_df = _pd.DataFrame(trace_EL_list).astype({0:_np.int16, 1:_np.int32, 2:bytes, 3:_np.float})
    el_df[2] = el_df[2].str.decode("utf-8")
    el_df['time'] = _gpsweeksec2datetime(gps_week=el_df[0], tow=el_df[1], as_j2000=True)
    el_df.drop(columns=[0,1],inplace=True)
    el_df.columns = ['PRN','el','time']
    
    return el_df.set_index(['time'])
