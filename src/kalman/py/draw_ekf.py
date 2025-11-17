import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import mplcursors # 建议安装，用于交互式放大

# 读取 CSV 文件
try:
    # 确保文件路径正确
    df = pd.read_csv('/mnt/planB/planning_explore/config/ekf.csv')
except FileNotFoundError:
    print("Error: CSV file not found. Check file path. Did you run the C++ code first?")
    exit()

# 提取数据列
time_series = df['Timestamp'].values
# --- 归一化时间轴，从 t=0 开始 ---
time = time_series - time_series[0]

# 位置数据
px_raw, py_raw = df['px_raw'].values, df['py_raw'].values
px_est, py_est = df['px_est'].values, df['py_est'].values

# 姿态数据 (弧度转角度，更直观)
r_raw, p_raw, y_raw = np.rad2deg(df['r_raw'].values), np.rad2deg(df['p_raw'].values), np.rad2deg(df['y_raw'].values)
r_est, p_est, y_est = np.rad2deg(df['r_est'].values), np.rad2deg(df['p_est'].values), np.rad2deg(df['y_est'].values)

# --- 绘图 ---

fig, axes = plt.subplots(3, 2, figsize=(14, 18))
plt.suptitle('EKF Filtering Results (Quaternion State) - Time Normalized', fontsize=16)

# 1. X Position
axes[0, 0].plot(time, px_raw, label='Raw Odometry X', alpha=0.6, marker='.')
axes[0, 0].plot(time, px_est, label='EKF Estimated X', color='red', linewidth=2)
axes[0, 0].set_title('X Position Filtering')
axes[0, 0].set_xlabel('Time (s) [Normalized]')
axes[0, 0].set_ylabel('Position X (m)')
axes[0, 0].grid(True)
axes[0, 0].legend()

# 2. Y Position
axes[0, 1].plot(time, py_raw, label='Raw Odometry Y', alpha=0.6, marker='.')
axes[0, 1].plot(time, py_est, label='EKF Estimated Y', color='red', linewidth=2)
axes[0, 1].set_title('Y Position Filtering')
axes[0, 1].set_xlabel('Time (s) [Normalized]')
axes[0, 1].set_ylabel('Position Y (m)')
axes[0, 1].grid(True)
axes[0, 1].legend()

# 3. Roll Angle
axes[1, 0].plot(time, r_raw, label='Raw Odometry Roll', alpha=0.6, marker='.')
axes[1, 0].plot(time, r_est, label='EKF Estimated Roll', color='red', linewidth=2)
axes[1, 0].set_title('Roll Angle Filtering')
axes[1, 0].set_xlabel('Time (s) [Normalized]')
axes[1, 0].set_ylabel('Roll (degrees)')
axes[1, 0].grid(True)
axes[1, 0].legend()

# 4. Pitch Angle
axes[1, 1].plot(time, p_raw, label='Raw Odometry Pitch', alpha=0.6, marker='.')
axes[1, 1].plot(time, p_est, label='EKF Estimated Pitch', color='red', linewidth=2)
axes[1, 1].set_title('Pitch Angle Filtering')
axes[1, 1].set_xlabel('Time (s) [Normalized]')
axes[1, 1].set_ylabel('Pitch (degrees)')
axes[1, 1].grid(True)
axes[1, 1].legend()

# 5. Yaw Angle
axes[2, 0].plot(time, y_raw, label='Raw Odometry Yaw', alpha=0.6, marker='.')
axes[2, 0].plot(time, y_est, label='EKF Estimated Yaw', color='red', linewidth=2)
axes[2, 0].set_title('Yaw Angle Filtering')
axes[2, 0].set_xlabel('Time (s) [Normalized]')
axes[2, 0].set_ylabel('Yaw (degrees)')
axes[2, 0].grid(True)
axes[2, 0].legend()

# 6. XY 轨迹 (附加)
axes[2, 1].plot(px_raw, py_raw, label='Raw Odometry Trajectory', alpha=0.6, linestyle='--')
axes[2, 1].plot(px_est, py_est, label='EKF Estimated Trajectory', color='red', linewidth=2)
axes[2, 1].set_title('XY Trajectory')
axes[2, 1].set_xlabel('X Position (m)')
axes[2, 1].set_ylabel('Y Position (m)')
axes[2, 1].axis('equal') # 确保X Y轴比例相同
axes[2, 1].grid(True)
axes[2, 1].legend()

# 启用交互式功能
try:
    mplcursors.cursor(hover=True)
except NameError:
    pass

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()