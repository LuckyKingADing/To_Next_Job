#!/bin/bash

# =====================================================
# 云端批量初始化评测脚本 - 通过 TCMSF 可执行程序调用
# =====================================================
# 特点：
# 1. 直接调用 bin/TCMSF 可执行程序，无需编译动态库链接
# 2. 不支持切片测试，运行完整数据
# 3. 支持不同日期使用不同的 GNSS 数据源 (pvt 或 gnss)
# =====================================================

# 数据基础路径（pvtrtk_nocamera 目录）
BASEFOLD=/mnt/data/oss-byd-wl-roadtest/users/localization/split_data/split_data_dir/pvtrtk_nocamera/

# 结果输出路径
RESULT_DIR=/mnt/data/oss-byd-wl-roadtest/users/localization/init_eval_results_exec/

# tcmsf_offline 目录路径（云端路径）
TCMSF_OFFLINE_DIR=/mnt/workspace/user/wufengbo/offline_program/tcmsf/ota630_2e72e88/tcmsf_offline

# =====================================================
# GNSS 数据源配置（根据日期配置）
# =====================================================
# 格式：日期:数据源类型
# "pvt" -> 使用 /drivers/gnss/pvt topic
# "gnss" -> 使用 /drivers/gnss/raw topic
#
# 配置说明（参考 batch_run_tcmsf_config_non_apollo.toml）：
# - 20260507: lc_source = "pvt" (存在 gnss/pvt topic)
# - 20260511: lc_source = "gnss" (不存在 gnss/pvt，使用 gnss/raw)
# =====================================================

# 日期与数据源对应关系
declare -A DATE_SOURCE_MAP
DATE_SOURCE_MAP["20260507"]="pvt"
DATE_SOURCE_MAP["20260511"]="gnss"

# 要处理的日期列表
TARGET_DATES=("20260507" "20260511")

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

# 检查 TCMSF 可执行程序
TCMSF_EXEC="$TCMSF_OFFLINE_DIR/bin/TCMSF"
if [ ! -f "$TCMSF_EXEC" ]; then
    echo "错误: TCMSF 可执行程序不存在: $TCMSF_EXEC"
    exit 1
fi

# source 环境变量
if [ -f "$TCMSF_OFFLINE_DIR/source_env.sh" ]; then
    source $TCMSF_OFFLINE_DIR/source_env.sh
    echo "[成功] 已加载环境变量: $TCMSF_OFFLINE_DIR/source_env.sh"
else
    echo "[警告] source_env.sh 不存在，将使用默认环境"
fi

echo "================================"
echo "基础目录: $BASEFOLD"
echo "结果目录: $RESULT_DIR"
echo "TCMSF目录: $TCMSF_OFFLINE_DIR"
echo "TCMSF可执行程序: $TCMSF_EXEC"
echo "目标日期: ${TARGET_DATES[*]}"
echo "================================"
echo ""
echo "GNSS 数据源配置:"
for date in "${TARGET_DATES[@]}"; do
    source_type=${DATE_SOURCE_MAP[$date]}
    if [ "$source_type" = "pvt" ]; then
        echo "  $date -> /drivers/gnss/pvt"
    else
        echo "  $date -> /drivers/gnss/raw"
    fi
done
echo "================================"

# =====================================================
# 遍历指定的日期目录
# =====================================================

for target_date in "${TARGET_DATES[@]}"; do
    dataset="$BASEFOLD$target_date/"

    # 获取该日期的数据源配置
    pvtlc_source=${DATE_SOURCE_MAP[$target_date]}
    if [ -z "$pvtlc_source" ]; then
        pvtlc_source="pvt"  # 默认使用 pvt
    fi

    if [ ! -d "$dataset" ]; then
        echo "[警告] 日期目录不存在: $dataset"
        continue
    fi

    dataset_name=$(basename "$dataset")
    echo ""
    echo "========================================"
    echo "[处理日期] $dataset_name"
    echo "[数据源] $pvtlc_source ($( [ "$pvtlc_source" = "pvt" ] && echo '/drivers/gnss/pvt' || echo '/drivers/gnss/raw' ))"
    echo "========================================"

    # 遍历该日期目录下的所有子数据集
    for subdir in "$dataset"*/; do
        if [ -d "$subdir" ]; then
            subdir_name=$(basename "$subdir")

            # 跳过非数据目录（子数据集目录名通常包含日期格式）
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
            echo "  数据源: $pvtlc_source"
            echo "  ---"

            # 运行初始化评测（通过 Python 脚本调用 TCMSF 可执行程序）
            cd $TCMSF_OFFLINE_DIR/util/init_evaluator

            python3 tcmsf_init_evaluator_via_exec.py \
                "$data_path" \
                "$result_path" \
                --pvtlc-source "$pvtlc_source"

            echo "  [完成] $subdir_name"
        fi
    done

    echo "[完成日期] $dataset_name"
done

# =====================================================
# 汇总所有 init_report.txt 到外层目录
# =====================================================

echo ""
echo "========================================"
echo "[汇总] 开始整理所有 init_report.txt"
echo "========================================"

# 汇总文件路径
SUMMARY_FILE="$RESULT_DIR/init_eval_summary_all.txt"
CSV_FILE="$RESULT_DIR/init_eval_summary.csv"
DETAIL_FILE="$RESULT_DIR/init_eval_detail.tmp"

# 统计变量
total_count=0
success_count=0
fail_count=0

# 临时存储结果列表（用于生成开头表格）
RESULTS_LIST=""

# 先收集所有结果，写入临时详情文件
> "$DETAIL_FILE"

# 遍历所有日期目录
for date_dir in "$RESULT_DIR"/*/; do
    if [ ! -d "$date_dir" ]; then
        continue
    fi

    date_name=$(basename "$date_dir")

    # 跳过非日期目录
    if [[ ! "$date_name" =~ ^[0-9]{8}$ ]]; then
        continue
    fi

    echo ""
    echo "[整理日期] $date_name"

    # 遍历该日期下的所有子数据集
    for subdir in "$date_dir"*/; do
        if [ ! -d "$subdir" ]; then
            continue
        fi

        subdir_name=$(basename "$subdir")
        report_file="$subdir/init_report.txt"

        if [ -f "$report_file" ]; then
            total_count=$((total_count + 1))

            # 解析结果
            init_status=$(grep "Initialization:" "$report_file" | awk '{print $2}')
            init_time=$(grep "Total init time:" "$report_file" | grep -oP '[\d.]+')
            reinit_count=$(grep "Reinit count:" "$report_file" | awk '{print $3}')

            if [ "$init_status" = "SUCCESS" ]; then
                success_count=$((success_count + 1))
            else
                fail_count=$((fail_count + 1))
            fi

            # 存储到结果列表（用于开头表格）
            RESULTS_LIST="${RESULTS_LIST}${date_name}|${subdir_name}|${init_status}|${init_time}|${reinit_count}\n"

            # 写入详情到临时文件
            {
                echo ""
                echo "## 子数据集: $subdir_name"
                echo "## 路径: $subdir"
                echo "#------------------------------------------------------------"
                cat "$report_file"
                echo "#------------------------------------------------------------"
            } >> "$DETAIL_FILE"

            # 写入 CSV
            echo "$date_name,$subdir_name,$init_status,$init_time,$reinit_count" >> "$CSV_FILE"

            echo "  [已添加] $subdir_name - $init_status"
        else
            echo "  [警告] 未找到报告文件: $report_file"
        fi
    done
done

# =====================================================
# 创建汇总文件（开头先显示表格）
# =====================================================

{
    echo "# TCMSF 初始化评测汇总报告"
    echo "# 生成的日期: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "# 数据来源: $BASEFOLD"
    echo "# 结果目录: $RESULT_DIR"
    echo "#============================================================"
    echo ""
    echo "#============================================================"
    echo "# 概览表格"
    echo "#============================================================"
    echo ""
    printf "%-10s %-42s %-10s %-12s %-6s\n" "Date" "Dataset" "Status" "InitTime(s)" "Reinit"
    echo "---------- ------------------------------------------ ---------- ---------- ------"

    # 输出结果表格
    echo "$RESULTS_LIST" | while IFS='|' read -r d name status time reinit; do
        if [ -n "$d" ]; then
            printf "%-10s %-42s %-10s %-12s %-6s\n" "$d" "$name" "$status" "$time" "$reinit"
        fi
    done

    echo ""
    echo "#============================================================"
    echo "# 统计汇总"
    echo "#============================================================"
    echo ""
    echo "总数据集数量: $total_count"
    echo "初始化成功: $success_count"
    echo "初始化失败: $fail_count"
    if [ $total_count -gt 0 ]; then
        success_rate=$(awk "BEGIN {printf \"%.2f\", $success_count * 100 / $total_count}")
        echo "成功率: ${success_rate}%"
    fi
    echo ""
    echo "#============================================================"
    echo ""
    echo "#============================================================"
    echo "# 详细报告"
    echo "#============================================================"
    echo ""
    cat "$DETAIL_FILE"
} > "$SUMMARY_FILE"

# 删除临时文件
rm -f "$DETAIL_FILE"

echo ""
echo "========================================"
echo "[完成] 批量处理和汇总完成"
echo "========================================"
echo "结果目录: $RESULT_DIR"
echo "汇总报告: $SUMMARY_FILE"
echo "汇总CSV表: $CSV_FILE"
echo "统计: 总数=$total_count, 成功=$success_count, 失败=$fail_count"
echo "========================================"