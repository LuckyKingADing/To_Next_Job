#!/bin/bash

# =====================================================
# 云端批量初始化评测脚本 - 切片测试模式
# =====================================================
# 功能：
# 1. 使用 bin/tcmsf_init_evaluator 进行切片测试
# 2. 支持不同日期使用不同的 GNSS 数据源
# 3. 生成汇总报告（开头表格 + 统计指标 + 详细报告）
# 4. 自动跳过时间不足的最后切片
# =====================================================

# 数据基础路径
BASEFOLD=/mnt/data/oss-byd-wl-roadtest/users/localization/split_data/split_data_dir/pvtrtk_nocamera/

# 结果输出路径
RESULT_DIR=/mnt/data/oss-byd-wl-roadtest/users/localization/init_eval_results/

# tcmsf_offline 目录路径
TCMSF_OFFLINE_DIR=/mnt/workspace/user/wufengbo/offline_program/tcmsf/ota630_2e72e88/tcmsf_offline

# =====================================================
# GNSS 数据源配置（根据日期配置）
# =====================================================
declare -A DATE_SOURCE_MAP
DATE_SOURCE_MAP["20260507"]="pvt"
DATE_SOURCE_MAP["20260511"]="gnss"

# 要处理的日期列表
TARGET_DATES=("20260507" "20260511")

# =====================================================
# 切片测试参数
# =====================================================
SLICE_INTERVAL=40      # 切片间隔（秒）
SLICE_COUNT=-1         # -1 表示自动运行到数据结束
INIT_TIMEOUT=300       # 单切片超时（秒）

# =====================================================
# 检查环境
# =====================================================

if [ ! -d "$BASEFOLD" ]; then
    echo "错误: 基础目录 '$BASEFOLD' 不存在"
    exit 1
fi

if [ ! -d "$TCMSF_OFFLINE_DIR" ]; then
    echo "错误: tcmsf_offline 目录 '$TCMSF_OFFLINE_DIR' 不存在"
    exit 1
fi

# 检查 Python 脚本
DISPATCHER_SCRIPT="$TCMSF_OFFLINE_DIR/util/init_evaluator/tcmsf_init_evaluator_dispatcher_cloud.py"
if [ ! -f "$DISPATCHER_SCRIPT" ]; then
    echo "错误: 调度脚本不存在: $DISPATCHER_SCRIPT"
    exit 1
fi

# source 环境变量
if [ -f "$TCMSF_OFFLINE_DIR/source_env.sh" ]; then
    source $TCMSF_OFFLINE_DIR/source_env.sh
    echo "[成功] 已加载环境变量"
else
    echo "[警告] source_env.sh 不存在"
fi

echo "================================"
echo "基础目录: $BASEFOLD"
echo "结果目录: $RESULT_DIR"
echo "TCMSF目录: $TCMSF_OFFLINE_DIR"
echo "切片间隔: $SLICE_INTERVAL s"
echo "切片超时: $INIT_TIMEOUT s"
echo "目标日期: ${TARGET_DATES[*]}"
echo "================================"
echo ""
echo "GNSS 数据源配置:"
for date in "${TARGET_DATES[@]}"; do
    source_type=${DATE_SOURCE_MAP[$date]}
    echo "  $date -> ${source_type} (/drivers/gnss/${source_type})"
done
echo "================================"

# =====================================================
# 遍历指定的日期目录
# =====================================================

for target_date in "${TARGET_DATES[@]}"; do
    dataset="$BASEFOLD$target_date/"

    pvtlc_source=${DATE_SOURCE_MAP[$target_date]}
    if [ -z "$pvtlc_source" ]; then
        pvtlc_source="pvt"
    fi

    if [ ! -d "$dataset" ]; then
        echo "[警告] 日期目录不存在: $dataset"
        continue
    fi

    dataset_name=$(basename "$dataset")
    echo ""
    echo "========================================"
    echo "[处理日期] $dataset_name"
    echo "[数据源] $pvtlc_source"
    echo "========================================"

    # 遍历子数据集
    for subdir in "$dataset"*/; do
        if [ ! -d "$subdir" ]; then
            continue
        fi

        subdir_name=$(basename "$subdir")
        if [[ ! "$subdir_name" =~ [0-9]{4}-[0-9]{2}-[0-9]{2} ]]; then
            echo "[跳过] 非数据目录: $subdir_name"
            continue
        fi

        data_path="$subdir"
        result_path="$RESULT_DIR/$dataset_name/$subdir_name"

        echo ""
        echo "  [子数据集] $subdir_name"
        echo "  数据路径: $data_path"
        echo "  结果路径: $result_path"
        echo "  ---"

        cd $TCMSF_OFFLINE_DIR/util/init_evaluator

        python3 tcmsf_init_evaluator_dispatcher_cloud.py \
            "$data_path" \
            "$result_path" \
            0 $SLICE_INTERVAL $SLICE_COUNT $INIT_TIMEOUT \
            "$pvtlc_source"

        echo "  [完成] $subdir_name"
    done

    echo "[完成日期] $dataset_name"
done

# =====================================================
# 汇总所有结果到外层目录
# =====================================================

echo ""
echo "========================================"
echo "[汇总] 开始整理所有评测结果"
echo "========================================"

SUMMARY_FILE="$RESULT_DIR/init_eval_summary_all.txt"
CSV_FILE="$RESULT_DIR/init_eval_summary.csv"

# 临时文件用于收集所有结果
RESULTS_LIST_FILE="/tmp/results_list.txt"
> "$RESULTS_LIST_FILE"

# 遍历所有日期目录，收集结果
for date_dir in "$RESULT_DIR"/*/; do
    if [ ! -d "$date_dir" ]; then
        continue
    fi

    date_name=$(basename "$date_dir")
    if [[ ! "$date_name" =~ ^[0-9]{8}$ ]]; then
        continue
    fi

    echo ""
    echo "[整理日期] $date_name"

    for subdir in "$date_dir"*/; do
        if [ ! -d "$subdir" ]; then
            continue
        fi

        subdir_name=$(basename "$subdir")
        summary_file="$subdir/init_eval_summary.txt"

        if [ -f "$summary_file" ]; then
            # 解析新的汇总文件格式
            # 格式:
            #   Total slices: X (valid: Y, skipped: Z)
            #   Success: S
            #   Failed: F
            #   Success rate: R%
            #   Average: A.s
            #   Min: M.s
            #   Max: X.s
            #   Median: Med.s

            total_line=$(grep "Total slices:" "$summary_file")
            if [[ "$total_line" =~ \(valid:\ ([0-9]+) ]]; then
                total_slices=${BASH_REMATCH[1]}
            else
                total_slices=$(echo "$total_line" | awk '{print $NF}')
            fi

            # 获取跳过数量
            if [[ "$total_line" =~ skipped:\ ([0-9]+) ]]; then
                skipped_count=${BASH_REMATCH[1]}
            else
                skipped_count=0
            fi

            success_count=$(grep "Success:" "$summary_file" | awk '{print $NF}')
            fail_count=$(grep "Failed:" "$summary_file" | awk '{print $NF}')
            rate=$(grep "Success rate:" "$summary_file" | awk '{print $NF}' | sed 's/%//')

            # 初始化时间统计
            avg_time=$(grep "Average:" "$summary_file" | awk '{print $NF}' | sed 's/s//')
            min_time=$(grep "Min:" "$summary_file" | awk '{print $NF}' | sed 's/s//')
            max_time=$(grep "Max:" "$summary_file" | awk '{print $NF}' | sed 's/s//')
            median_time=$(grep "Median:" "$summary_file" | awk '{print $NF}' | sed 's/s//')

            # 如果没有数据，设为默认值
            if [ -z "$avg_time" ]; then avg_time="-"; fi
            if [ -z "$max_time" ]; then max_time="-"; fi
            if [ -z "$min_time" ]; then min_time="-"; fi
            if [ -z "$median_time" ]; then median_time="-"; fi

            # 判断整体状态
            if [ "$success_count" -gt 0 ] && [ "$fail_count" -eq 0 ]; then
                overall_status="PASS"
            elif [ "$success_count" -gt "$fail_count" ]; then
                overall_status="PARTIAL"
            else
                overall_status="FAIL"
            fi

            # 写入临时文件
            echo "$date_name|$subdir_name|$total_slices|$success_count|$fail_count|$rate|$avg_time|$max_time|$min_time|$median_time|$skipped_count|$overall_status" >> "$RESULTS_LIST_FILE"

            echo "  [已添加] $subdir_name - ${rate}% (${success_count}/${total_slices})"
        fi
    done
done

# =====================================================
# 创建汇总文件（开头表格 + 统计 + 详细报告）
# =====================================================

# 统计全局数据
total_datasets=0
pass_datasets=0
fail_datasets=0
total_slices_all=0
total_success_all=0
total_fail_all=0
total_skipped_all=0
all_avg_times=""
all_max_times=""

while IFS='|' read -r date name slices success fail rate avg max min median skipped status; do
    total_datasets=$((total_datasets + 1))
    total_slices_all=$((total_slices_all + slices))
    total_success_all=$((total_success_all + success))
    total_fail_all=$((total_fail_all + fail))
    total_skipped_all=$((total_skipped_all + skipped))

    if [ "$status" = "PASS" ]; then
        pass_datasets=$((pass_datasets + 1))
    elif [ "$status" = "PARTIAL" ]; then
        pass_datasets=$((pass_datasets + 1))
    else
        fail_datasets=$((fail_datasets + 1))
    fi

    # 收集有效的时间数据
    if [ "$avg" != "-" ] && [ -n "$avg" ]; then
        all_avg_times="$all_avg_times $avg"
    fi
    if [ "$max" != "-" ] && [ -n "$max" ]; then
        all_max_times="$all_max_times $max"
    fi
done < "$RESULTS_LIST_FILE"

# 计算全局平均和最大
if [ -n "$all_avg_times" ]; then
    global_avg=$(echo $all_avg_times | tr ' ' '\n' | awk '{sum+=$1; count++} END {if(count>0) printf "%.2f", sum/count; else print "-"}')
else
    global_avg="-"
fi

if [ -n "$all_max_times" ]; then
    global_max=$(echo $all_max_times | tr ' ' '\n' | sort -n | tail -1)
else
    global_max="-"
fi

# 写入汇总文件
{
    echo "# TCMSF Initialization Evaluation Summary"
    echo "# Generated: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "# Slice interval: ${SLICE_INTERVAL}s, Timeout: ${INIT_TIMEOUT}s"
    echo "#============================================================"
    echo ""
    echo "# Summary Table"
    echo "#------------------------------------------------------------"
    printf "%-10s %-42s %-8s %-8s %-8s %-10s %-10s %-10s %-10s\n" \
        "Date" "Dataset" "Slices" "Success" "Fail" "Rate" "AvgTime" "MaxTime" "Status"
    echo "---------- ------------------------------------------ -------- -------- -------- ---------- ---------- ---------- ----------"

    while IFS='|' read -r date name slices success fail rate avg max min median skipped status; do
        # 格式化时间
        if [ "$avg" != "-" ] && [ -n "$avg" ]; then
            avg_fmt="${avg}s"
        else
            avg_fmt="-"
        fi
        if [ "$max" != "-" ] && [ -n "$max" ]; then
            max_fmt="${max}s"
        else
            max_fmt="-"
        fi

        printf "%-10s %-42s %-8s %-8s %-8s %-9s %-10s %-10s %-10s\n" \
            "$date" "$name" "$slices" "$success" "$fail" "${rate}%" "$avg_fmt" "$max_fmt" "$status"
    done < "$RESULTS_LIST_FILE"

    echo ""
    echo "#------------------------------------------------------------"
    echo "# Overall Statistics"
    echo "#------------------------------------------------------------"
    echo ""
    echo "Total datasets:     $total_datasets"
    echo "Passed datasets:    $pass_datasets"
    echo "Failed datasets:    $fail_datasets"
    if [ $total_datasets -gt 0 ]; then
        overall_pass_rate=$(awk "BEGIN {printf \"%.2f\", $pass_datasets * 100 / $total_datasets}")
        echo "Dataset pass rate:  ${overall_pass_rate}%"
    fi
    echo ""
    echo "Total slices:       $total_slices_all"
    echo "Successful slices:  $total_success_all"
    echo "Failed slices:      $total_fail_all"
    echo "Skipped slices:     $total_skipped_all"
    if [ $total_slices_all -gt 0 ]; then
        slice_success_rate=$(awk "BEGIN {printf \"%.2f\", $total_success_all * 100 / $total_slices_all}")
        echo "Slice success rate: ${slice_success_rate}%"
    fi
    echo ""
    echo "Global Avg Init Time: ${global_avg}s"
    echo "Global Max Init Time: ${global_max}s"

} > "$SUMMARY_FILE"

# 写入 CSV
{
    echo "# TCMSF Init Evaluation Summary CSV"
    echo "# Generated: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "Date,Dataset,Slices,Success,Fail,Skipped,Rate,AvgTime,MaxTime,MinTime,MedianTime,Status"
    while IFS='|' read -r date name slices success fail rate avg max min median skipped status; do
        echo "$date,$name,$slices,$success,$fail,$skipped,${rate}%,${avg},${max},${min},${median},$status"
    done < "$RESULTS_LIST_FILE"
} > "$CSV_FILE"

# 添加详细报告部分
{
    echo ""
    echo "#============================================================"
    echo "# Detailed Reports"
    echo "#============================================================"

    for date_dir in "$RESULT_DIR"/*/; do
        if [ ! -d "$date_dir" ]; then
            continue
        fi
        date_name=$(basename "$date_dir")
        if [[ ! "$date_name" =~ ^[0-9]{8}$ ]]; then
            continue
        fi

        for subdir in "$date_dir"*/; do
            if [ ! -d "$subdir" ]; then
                continue
            fi
            subdir_name=$(basename "$subdir")
            summary_file="$subdir/init_eval_summary.txt"

            if [ -f "$summary_file" ]; then
                echo ""
                echo "## Dataset: $date_name / $subdir_name"
                echo "#------------------------------------------------------------"
                cat "$summary_file"
                echo "#------------------------------------------------------------"
            fi
        done
    done
} >> "$SUMMARY_FILE"

# 清理临时文件
rm -f "$RESULTS_LIST_FILE"

echo ""
echo "========================================"
echo "[完成] 批量处理和汇总完成"
echo "========================================"
echo "结果目录: $RESULT_DIR"
echo "汇总报告: $SUMMARY_FILE"
echo "CSV表格: $CSV_FILE"
echo ""
echo "统计汇总:"
echo "  数据集: $total_datasets 个 (通过 $pass_datasets, 失败 $fail_datasets)"
echo "  切片:   $total_slices_all 个 (成功 $total_success_all, 失败 $total_fail_all, 跳过 $total_skipped_all)"
echo "  全局平均初始化时间: ${global_avg}s"
echo "  全局最大初始化时间: ${global_max}s"
echo "========================================"