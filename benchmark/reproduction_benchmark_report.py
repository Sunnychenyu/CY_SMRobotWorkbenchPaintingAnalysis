import argparse
import csv
from pathlib import Path

from openpyxl import Workbook, load_workbook
from openpyxl.styles import Alignment, Font, PatternFill
from openpyxl.utils import get_column_letter


METHODS = [
    ("tzinava_2020", "Tzinava 2020"),
    ("wu_2020", "Wu 2020"),
    ("fuke_2005", "Fuke 2005"),
    ("vanerio_2021", "Vanerio 2021"),
    ("current_gpu", "本文方法"),
]
NUMERIC = {
    "repeat", "scene_side_mm", "plate_count", "plate_spacing_mm",
    "cell_mm", "spray_distance_mm", "incidence_deg", "scan_speed_mm_s",
    "point_interval_s", "input_vertices", "input_triangles",
    "trajectory_points", "output_vertices", "output_triangles",
    "preparation_ms", "core_ms", "conversion_ms", "display_ms",
    "first_frame_wait_ms", "total_ms", "nonzero_vertices",
}
HEADERS = [
    "算法 ID", "运行类型", "序号", "状态", "平板边长 (mm)", "平板数",
    "间距 (mm)", "网格单元 (mm)", "喷涂距离 (mm)", "入射角 (°)",
    "扫描速度 (mm/s)", "点间隔 (s)", "输入顶点", "输入三角面片",
    "轨迹点", "输出顶点", "输出三角面片", "准备 (ms)", "核心 (ms)",
    "转换 (ms)", "显示上传 (ms)", "首帧等待 (ms)", "总耗时 (ms)",
    "非零顶点", "失败原因", "其他调度 (ms)",
]


def style_sheet(sheet):
    sheet.freeze_panes = "A2"
    sheet.auto_filter.ref = sheet.dimensions
    for cell in sheet[1]:
        cell.font = Font(name="Arial", bold=True, color="FFFFFF")
        cell.fill = PatternFill("solid", fgColor="24566A")
        cell.alignment = Alignment(horizontal="center")
    for row in sheet.iter_rows(min_row=2):
        for cell in row:
            cell.font = Font(name="Arial", size=10)
            cell.alignment = Alignment(vertical="center")
    for column in sheet.columns:
        letter = get_column_letter(column[0].column)
        width = max(len(str(cell.value or "")) for cell in column[:100]) + 2
        sheet.column_dimensions[letter].width = min(max(width, 13), 42)


def build_report(source, destination):
    with source.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    workbook = Workbook()
    conditions = workbook.active
    conditions.title = "实验条件"
    conditions.append(["项目", "设定或说明"])
    conditions.append(["主指标", "从点击运行复现到厚度结果首次进入视口的总耗时"])
    conditions.append(["计时边界", "场景和轨迹预先生成；生成过程不纳入主指标"])
    conditions.append(["预热与重复", "每个算法预热 1 次，正式运行 5 次；仅成功且厚度非零的正式运行参与统计"])
    conditions.append(["场景", "100 × 100 mm 双平板；间距 20 mm；网格单元 1 mm"])
    conditions.append(["轨迹", "线扫一个来回；入射角 90°；速度 50 mm/s；点间隔 0.01 s"])
    conditions.append(["喷涂距离", "Wu 30 mm；其余算法 120 mm"])
    conditions.append(["独立变量", "Tzinava、Wu、Fuke、Vanerio 与本文方法的计算与显示流程"])
    conditions.append(["原始记录", str(source.resolve())])

    runs = workbook.create_sheet("逐次运行")
    runs.append(HEADERS)
    keys = list(rows[0]) if rows else HEADERS[:-1]
    for row in rows:
        values = []
        for key in keys:
            value = row[key]
            values.append(float(value) if value and key in NUMERIC else value or None)
        runs.append(values)
        line = runs.max_row
        runs.cell(line, 26, f'=IF(W{line}="","",W{line}-SUM(R{line}:V{line}))')

    summary = workbook.create_sheet("统计汇总")
    summary.append(["算法 ID", "方法", "有效次数", "均值 (ms)", "中位数 (ms)",
                    "标准差 (ms)", "最短 (ms)", "最长 (ms)", "排名", "备注",
                    "第1次", "第2次", "第3次", "第4次", "第5次"])
    for method_id, label in METHODS:
        summary.append([method_id, label])
        line = summary.max_row
        measured = [(index + 2, row) for index, row in enumerate(rows)
                    if row.get("algorithm_id") == method_id
                    and row.get("run_type") == "measured"]
        for repeat in range(1, 6):
            matching = [row_number for row_number, row in measured
                        if row.get("repeat") == str(repeat)]
            if matching:
                raw = matching[0]
                summary.cell(line, repeat + 10,
                             f'=IF(AND(\'逐次运行\'!D{raw}="success",'
                             f'ISNUMBER(\'逐次运行\'!W{raw})),'
                             f'\'逐次运行\'!W{raw},"")')
        sample_range = f"K{line}:O{line}"
        summary.cell(line, 3, f"=COUNT({sample_range})")
        summary.cell(line, 4, f'=IF(C{line}=0,"",AVERAGE({sample_range}))')
        summary.cell(line, 5, f'=IF(C{line}=0,"",MEDIAN({sample_range}))')
        summary.cell(line, 6, f'=IF(C{line}<2,"",STDEV.S({sample_range}))')
        summary.cell(line, 7, f'=IF(C{line}=0,"",MIN({sample_range}))')
        summary.cell(line, 8, f'=IF(C{line}=0,"",MAX({sample_range}))')
        summary.cell(line, 9,
                     f'=IF(C{line}<5,"",RANK.EQ(D{line},$D$2:$D$6,1))')

    failures = workbook.create_sheet("失败记录")
    failures.append(["算法 ID", "运行类型", "序号", "状态", "失败原因", "原始行号"])
    for line, row in enumerate(rows, start=2):
        if row.get("status") != "success":
            failures.append([row.get("algorithm_id"), row.get("run_type"),
                             int(row["repeat"]), row.get("status"),
                             row.get("error"), line])

    for sheet in workbook:
        style_sheet(sheet)
    workbook.calculation.fullCalcOnLoad = True
    workbook.calculation.forceFullCalc = True
    workbook.save(destination)
    checked = load_workbook(destination, read_only=True)
    assert checked.sheetnames == ["实验条件", "逐次运行", "统计汇总", "失败记录"]
    assert checked["逐次运行"].max_row == len(rows) + 1
    checked.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input_csv", type=Path)
    parser.add_argument("output_xlsx", type=Path)
    arguments = parser.parse_args()
    build_report(arguments.input_csv, arguments.output_xlsx)
