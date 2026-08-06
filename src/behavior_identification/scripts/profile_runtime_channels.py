from pathlib import Path

import pandas as pd


ROOT = Path("src/behavior_identification/runtime_results")
TARGET_IDS = {
    "normal": 32,
    "red_light": 47,
    "overspeed": 60,
    "slow": 70,
    "aggressive_accel": 85,
    "aggressive_decel": 106,
    "lane_deviation": 116,
}
COLS = [
    "speed_mps",
    "accel_mps2",
    "lateral_deviation_m",
    "st_constraint",
    "sl_ratio",
    "accel_ratio",
    "overspeed_ratio",
]

for scenario, vehicle_id in TARGET_IDS.items():
    data = pd.read_csv(ROOT / f"{scenario}.csv")
    data = data.loc[data["vehicle_id"] == vehicle_id].copy()
    duration = data["ros_time"].max() - data["ros_time"].min()
    print(f"\n{scenario}: id={vehicle_id}, n={len(data)}, duration={duration:.2f}s")
    for column in COLS:
        values = data[column].dropna()
        print(
            f"  {column:24s} min={values.min():8.3f} "
            f"p50={values.quantile(.50):8.3f} "
            f"p95={values.quantile(.95):8.3f} max={values.max():8.3f}"
        )
