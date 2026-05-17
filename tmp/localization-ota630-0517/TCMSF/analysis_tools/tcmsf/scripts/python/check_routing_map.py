import matplotlib.pyplot as plt
import numpy as np
import subprocess
import sys
import argparse
import os

def check_routing_map(file_in):
    #print(file_in)
    pub_time = np.loadtxt(file_in, usecols=(0), dtype=float, delimiter=',', skiprows=1) 
    meas_time = np.loadtxt(file_in, usecols=(1), dtype=float, delimiter=',', skiprows=1) 
    ld_link_id = np.loadtxt(file_in, usecols=(2), dtype=int, delimiter=',', skiprows=1) 
    ld_offset = np.loadtxt(file_in, usecols=(3), dtype=float, delimiter=',', skiprows=1) 
    sd_link_id = np.loadtxt(file_in, usecols=(4), dtype=int, delimiter=',', skiprows=1) 
    sd_offset = np.loadtxt(file_in, usecols=(5), dtype=float, delimiter=',', skiprows=1)  

    prev_ld_linkid = 0
    prev_sd_linkid = 0
    print('begin time:', meas_time[0])
    for i in range(len(meas_time)):
        cur_time = meas_time[i]

        cur_ld_linkid = ld_link_id[i]
        if prev_ld_linkid == 0 and cur_ld_linkid != 0:
            print('ld link id become none zero:', cur_time)
        if prev_ld_linkid != 0 and cur_ld_linkid == 0:
            print('**** ld link id become zero:', cur_time)
        prev_ld_linkid = cur_ld_linkid

        cur_sd_linkid = sd_link_id[i]
        if prev_sd_linkid == 0 and cur_sd_linkid != 0:
            print('sd link id become none zero:', cur_time)
        if prev_sd_linkid != 0 and cur_sd_linkid == 0:
            print('**** sd link id become zero:', cur_time)
        prev_sd_linkid = cur_sd_linkid

    meas_time_gap = [meas_time[i+1] - meas_time[i] for i in range(len(meas_time) - 1)]
    meas_time_gap_time = [meas_time[i] for i in range(len(meas_time) - 1)]
    pub_mea_time_gap = [pub_time[i] - meas_time[i] for i in range(len(meas_time))]

    plt.figure(), plt.title('meas_time_gap')
    plt.plot(meas_time_gap_time, meas_time_gap), plt.grid()

    plt.figure(), plt.title('pub_mea_time_gap')
    plt.plot(meas_time, pub_mea_time_gap), plt.grid()

    plt.figure(), plt.title('ld_link_id')
    plt.plot(meas_time, ld_link_id), plt.grid()    

    plt.figure(), plt.title('ld_offset')
    plt.plot(meas_time, ld_offset), plt.grid()        

    plt.figure(), plt.title('sd_link_id')
    plt.plot(meas_time, sd_link_id), plt.grid()    

    plt.figure(), plt.title('sd_offset')
    plt.plot(meas_time, sd_offset), plt.grid()       

    plt.show()

def main():
    parser = argparse.ArgumentParser(
        description="Check ld and sd linkid in routing map", formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("--input", "-i", help="input file", required=True)
    args = parser.parse_args()
    try:
        check_routing_map(args.input)

    except Exception as e:
        print(e)

if __name__ == "__main__":
    main()