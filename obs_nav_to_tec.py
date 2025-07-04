import numpy as np
import georinex as gr
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from math import floor

# Frequencies for GPS L1 and L2
f1 = 1575.42e6  # Hz
f2 = 1227.60e6  # Hz
c = 299_792_458  # Speed of light in m/s
k = 40.3
l1_lambda = c / f1
l2_lambda = c / f2

denom = k * (1/f1**2 - 1/f2**2)

PLOTS_PER_ROW = 6

def converter(fname):
    ds = gr.load(fname)
    num_figures = len(ds.sv.values)
    print(num_figures)
    #fig, axes = plt.subplots(PLOTS_PER_ROW, num_figures // PLOTS_PER_ROW)
    for idx, prn in enumerate(ds.sv.values):
        try:
            C1 = ds['C1'].sel(sv=prn)
            C2 = ds['C2'].sel(sv=prn)
            stec = (C2-C1) / denom
            
            stec_clean = stec.dropna(dim="time")
            print(stec_clean)
        except KeyError:
            continue
    plt.legend()
    plt.ylabel("STEC (TECU)")
    plt.xlabel("Time")
    plt.title("Slant TEC from RINEX")
    plt.grid(True)
    ax = plt.gca()
    ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
    fname_no_folder = fname.split('/')[1]
    fname_no_folder = "images/" + fname_no_folder
    plt.savefig(f"{fname_no_folder}_slant_tec.png", dpi=500)
   
import os

files = os.listdir()
for f in files:
    if f.endswith(".ubx"):
        f_name = f.split('.')[0]
        os.system(f'./convbin -od -os -oi -ot -n nav_files/{f_name}.nav -o observations/{f_name}.obs {f}')
        converter(f'observations/{f_name}.obs')
     