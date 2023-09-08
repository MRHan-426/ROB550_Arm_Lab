import sys
import numpy as np

def parse_dh_param_file(dh_config_file):
    assert(dh_config_file is not None)
    f_line_contents = None
    with open(dh_config_file, "r") as f:
        f_line_contents = f.readlines()

    assert(f.closed)
    assert(f_line_contents is not None)
    # maybe not the most efficient/clean/etc. way to do this, but should only have to be done once so NBD
    dh_params = np.asarray([line.rstrip().split(',') for line in f_line_contents[1:]])
    dh_params = dh_params.astype(float)
    return dh_params


### TODO(DONE): parse a pox parameter file
def parse_pox_param_file(pox_config_file):
    assert(pox_config_file is not None)
    f_line_contents = None
    with open(pox_config_file, "r") as f:
        f_line_contents = f.readlines()

    assert(f.closed)
    assert(f_line_contents is not None)
    # maybe not the most efficient/clean/etc. way to do this, but should only have to be done once so NBD

    m_mat = np.asarray([line.rstrip().split(' ') for line in f_line_contents[1:5]])
    s_lst = np.asarray([line.rstrip().split(' ') for line in f_line_contents[6:]])
    
    m_mat = m_mat.astype(float)
    s_lst = s_lst.astype(float)
    return m_mat, s_lst