import pandas as pd
from openpyxl.styles import Alignment

CSV_FILE = "/home/hzq/桌面/测试结果/reckless_driving_identification_test_log_2025-12-26_09-30-30.csv"
EXCEL_FILE = CSV_FILE.replace(".csv", ".xlsx")


def visual_length(value):
    """
    估算 Excel 中的显示宽度：
    - 中文字符 ≈ 2
    - 英文 / 数字 / 符号 ≈ 1
    """
    if value is None:
        return 0
    text = str(value)
    length = 0
    for ch in text:
        if '\u4e00' <= ch <= '\u9fff':
            length += 2
        else:
            length += 1
    return length


def main():
    # 读取 CSV
    df = pd.read_csv(CSV_FILE)

    # 统一 none / 空值
    df.replace(["none", "None", ""], pd.NA, inplace=True)

    with pd.ExcelWriter(EXCEL_FILE, engine="openpyxl") as writer:
        df.to_excel(writer, index=False, sheet_name="Results")
        worksheet = writer.book["Results"]

        # ===== 1️⃣ 中文友好的自动列宽 =====
        for col in worksheet.columns:
            max_len = max(
                visual_length(cell.value) for cell in col
            )
            col_letter = col[0].column_letter
            worksheet.column_dimensions[col_letter].width = max_len + 2  # padding

        # ===== 2️⃣ 表格内容居中 + 自动换行 =====
        center_align = Alignment(
            horizontal="center",
            vertical="center",
            wrap_text=True
        )

        for row in worksheet.iter_rows():
            for cell in row:
                cell.alignment = center_align

        # （刻意不设置行高，交给 Excel 自动计算）

    print(f"✅ Excel 已生成（列宽完整显示）：{EXCEL_FILE}")


if __name__ == "__main__":
    main()
