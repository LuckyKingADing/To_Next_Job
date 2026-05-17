#!/bin/bash 

search_dir=data/Record_data/SPP_LC/2025-10-09_11-41-26
for entry in "$search_dir"/*
do
  cyber_recorder split -f ${entry} -o data/Record_data/SPP_LC/mini_lc/2025-10-09_11-41-26/${entry##*/} \
  -c /drivers/navi_traffic/raw /perception/detection/bev/lane_track /drivers/gnss/raw /localization/dr /drivers/imu/raw /drivers/navi_sd/raw /drivers/canbus/vehicle_info /drivers/navi_status/raw /localization/tcmsf 
  # -c /drivers/imu/raw /drivers/gnss/raw /drivers/rover_rtcm/raw /drivers/base_rtcm/raw /drivers/canbus/vehicle_info /localization/dr /perception/detection/bev/lane_track /drivers/navi_sd/raw /drivers/camera/front_narrow/compressed/image /drivers/camera/front_wide/compressed/image /cem/ehr/traffic_road /cem/ehr/map_scene /cem/ehr/position
done

