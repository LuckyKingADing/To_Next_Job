#!/bin/bash

BASEFOLD=/mnt/d/dockers/rt/rtk_pvt/2026/ 
RESULT_DIR=/mnt/d/dockers/rt/rtk_pvt/2026/results

if [ ! -d "$BASEFOLD" ]; then
    echo "错误: 基础目录 '$BASEFOLD' 不存在"
    exit 1
fi

echo "基础目录: $BASEFOLD"
echo "结果目录: $RESULT_DIR"
echo "================================"

# 遍历 basefold 下的所有目录 (datasets)
for dataset in "$BASEFOLD"/*/; do
    # 检查是否为目录
    if [ -d "$dataset" ]; then
        # 获取数据集名称（去掉路径末尾的斜杠和前缀路径）
        dataset_name=$(basename "$dataset")

        # 遍历该数据集目录下的所有子目录
        for subdir in "$dataset"*/; do
            if [ -d "$subdir" ]; then
                subdir_name=$(basename "$subdir")

                # 构建完整的数据路径
                data_path="$subdir"

                # 构建结果目录路径
                result_path="$RESULT_DIR/$dataset_name/$subdir_name"
                
                cd /apollo/modules/localization/src/TCMSF/tcmsf/util/init_evaluator
                python3 tcmsf_init_evaluator_dispatcher.py \
                    $data_path \
                    $result_path \
                    0 40 -1 300

                echo "数据路径: $data_path"
                echo "结果路径: $result_path"
                echo "---"
            fi
        done
    fi
done