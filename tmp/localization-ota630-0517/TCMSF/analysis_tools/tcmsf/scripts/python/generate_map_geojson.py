import matplotlib.pyplot as plt
import numpy as np
import subprocess
import sys
import argparse
import os

def install(package):
    subprocess.check_call([sys.executable, "-m", "pip", "install", package])

try:
    import geojson
except ImportError:
    install('geojson')
    import geojson

def list_files_sorted(directory):
    if not os.path.isdir(directory):
        raise ValueError(f"'{directory}' 不是有效目录")
    
    files = []
    for entry in os.listdir(directory):
        full_path = os.path.join(directory, entry)
        if os.path.isfile(full_path):
            files.append(entry)
    
    # 按文件名排序(不区分大小写)
    files.sort(key=lambda x: x.lower())    
    return files

def generate_lane_geojson(dir_in, dir_out):
    #print(dir_in)
    #print(dir_out)
    if not os.path.exists(dir_out):
        os.makedirs(dir_out)

    sorted_files = list_files_sorted(dir_in)

    for filename in sorted_files:        
        #print(filename)        
        file_in = os.path.join(dir_in, filename)
        file_out = os.path.join(dir_out, filename.replace("txt", "geojson"))        
        #print(file_in)
        #print(file_out)    

        # gather map information
        boundary_ids = []
        lons = []
        lats = []
        link_ids = []
        lane_ids = [] # need to search and bind
        lane_boundary_dict = {}

        with open(file_in, 'r', encoding='utf-8') as file:
            lines = file.readlines()
            for line in lines:
                #print(line)
                if 'boundaryid:' in line and 'lon' in line:
                    link_ids.append(int(line.split('linkid:')[1].split(',')[0]))
                    boundary_ids.append(int(line.split('boundaryid:')[1].split(',')[0]))
                    lons.append(float(line.split('lon:')[1].split(',')[0]))
                    lats.append(float(line.split('lat:')[1]))            
                elif 'laneid:' in line:
                    lane_id = int(line.split('laneid:')[1].split(',')[0])
                    boundary_id = int(line.split('boundaryid:')[1])
                    lane_boundary_dict[boundary_id] = lane_id
            # for i in range(len(boundary_ids)): # bind lane id and boundary id
            #     lane_id = lane_boundary_dict[boundary_ids[i]]
            #     lane_ids.append(lane_id)

        # dump to geojson file
        seg_x = []
        seg_y = []
        features = []
        cur_boundary_id = 0
        cur_link_id = 0
        cur_lane_id = 0

        for i in range(len(boundary_ids)):
            boundary_id = boundary_ids[i]
            if boundary_id != cur_boundary_id and cur_boundary_id != 0:
                line = geojson.LineString((seg_x[j], seg_y[j]) for j in range(len(seg_x)))
                cur_lane_id = lane_boundary_dict[boundary_id]
                feature = geojson.Feature(geometry=line, properties={"link_id": int(cur_link_id), "lane_id": int(cur_lane_id), "boundary_id": int(cur_boundary_id), "linear_object_type": int(1), "marking_type": int(3)})
                features.append(feature)
                seg_x = []
                seg_y = []
            cur_boundary_id = boundary_id
            cur_link_id = link_ids[i]
            #cur_lane_id = lane_ids[i]
            seg_x.append(lons[i])
            seg_y.append(lats[i])

        feature_collection = geojson.FeatureCollection(features)
        feature_json = geojson.dumps(feature_collection)

        with open(file_out, 'w') as f:
            f.write(feature_json)

def main():
    parser = argparse.ArgumentParser(
        description="Convert local map info into geojson", formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("--input", "-i", help="input dir", required=True)
    parser.add_argument("--output", "-o", help="output_dir", required=True)

    args = parser.parse_args()
    try:
        generate_lane_geojson(args.input, args.output)

    except Exception as e:
        print(e)

if __name__ == "__main__":
    main()