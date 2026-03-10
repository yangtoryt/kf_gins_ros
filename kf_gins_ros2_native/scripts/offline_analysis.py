#!/usr/bin/env python3
"""
离线分析脚本 - 从 ROS bag 中提取数据并进行深度分析

功能:
- 从 rosbag2 数据库读取数据
- 导出为 CSV 和 TUM 格式
- 计算完整的评估指标
- 生成对比图表
- 输出详细报告
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import Optional, List, Tuple, Dict
import math
import json
from datetime import datetime

import numpy as np
import pandas as pd
from scipy import interpolate, signal
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')  # 使用非交互后端

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    import rclpy
    import rclpy.serialization
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    HAS_ROSBAG = True
except ImportError:
    HAS_ROSBAG = False
    print("⚠️  rosbag2 不可用，仅支持 CSV 输入")


class TrajectoryData:
    """轨迹数据容器"""
    
    def __init__(self, name: str):
        self.name = name
        self.timestamps = []  # 秒
        self.positions = []   # (x, y, z) 列表
        self.attitudes = []   # (roll, pitch, yaw) 列表，弧度
        self.velocities = []  # (vx, vy, vz) 列表
    
    def to_dataframe(self) -> pd.DataFrame:
        """转换为 pandas DataFrame"""
        df = pd.DataFrame({
            'timestamp': self.timestamps,
            'x': [p[0] for p in self.positions],
            'y': [p[1] for p in self.positions],
            'z': [p[2] for p in self.positions],
            'roll': [a[0] for a in self.attitudes],
            'pitch': [a[1] for a in self.attitudes],
            'yaw': [a[2] for a in self.attitudes],
            'vx': [v[0] if len(self.velocities) > i else 0.0 for i, v in enumerate(self.velocities)],
            'vy': [v[1] if len(self.velocities) > i else 0.0 for i, v in enumerate(self.velocities)],
            'vz': [v[2] if len(self.velocities) > i else 0.0 for i, v in enumerate(self.velocities)],
        })
        return df
    
    def to_tum_format(self) -> str:
        """导出为 TUM 格式 (evo 工具兼容)
        
        格式: timestamp tx ty tz qx qy qz qw
        """
        lines = []
        for ts, pos, att in zip(self.timestamps, self.positions, self.attitudes):
            # 欧拉角转四元数
            qx, qy, qz, qw = self.rpy_to_quat(*att)
            line = f"{ts:.6f} {pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}"
            lines.append(line)
        return "\n".join(lines)
    
    @staticmethod
    def rpy_to_quat(roll, pitch, yaw) -> Tuple[float, float, float, float]:
        """欧拉角 (RPY) 转四元数 (qx, qy, qz, qw)"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw


class OfflineAnalyzer:
    """离线分析器"""
    
    def __init__(self, output_dir: Path):
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.ekf2_data: Optional[TrajectoryData] = None
        self.iekf_data: Optional[TrajectoryData] = None
        
        self.metrics: Dict[str, float] = {}
    
    def load_from_rosbag(self, bag_path: str):
        """从 rosbag2 文件加载数据"""
        if not HAS_ROSBAG:
            raise RuntimeError("rosbag2 模块不可用")
        
        storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr',
                                            output_serialization_format='cdr')
        
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        
        self.ekf2_data = TrajectoryData('EKF2')
        self.iekf_data = TrajectoryData('IEKF')
        
        topic_types = reader.get_all_topics_and_types()
        
        while reader.has_next():
            topic, data, ts = reader.read_next()
            
            # 跳过不需要的话题
            if topic not in ['/ekf2/pose', '/kf_gins/odom', '/ekf2/pose_odom']:
                continue
            
            try:
                if topic == '/ekf2/pose':
                    msg = rclpy.serialization.deserialize_message(data, PoseStamped)
                    self._process_ekf2_msg(msg)
                
                elif topic in ['/kf_gins/odom', '/ekf2/pose_odom']:
                    msg = rclpy.serialization.deserialize_message(data, Odometry)
                    if topic == '/kf_gins/odom':
                        self._process_iekf_msg(msg)
                    else:
                        self._process_ekf2_odom_msg(msg)
            
            except Exception as e:
                print(f"⚠️  处理消息时出错: {e}")
                continue
        
        print(f"✓ 从 {bag_path} 加载完成")
        print(f"  EKF2: {len(self.ekf2_data.timestamps)} 条消息")
        print(f"  IEKF: {len(self.iekf_data.timestamps)} 条消息")
    
    def load_from_csv(self, ekf2_csv: str, iekf_csv: str):
        """从 CSV 文件加载数据"""
        self.ekf2_data = self._csv_to_trajectory(ekf2_csv, 'EKF2')
        self.iekf_data = self._csv_to_trajectory(iekf_csv, 'IEKF')
    
    def _csv_to_trajectory(self, csv_path: str, name: str) -> TrajectoryData:
        """从 CSV 文件创建轨迹数据"""
        df = pd.read_csv(csv_path)
        traj = TrajectoryData(name)
        
        traj.timestamps = df['timestamp'].tolist()
        traj.positions = list(zip(df['x'], df['y'], df['z']))
        traj.attitudes = list(zip(df.get('roll', [0]*len(df)), 
                                   df.get('pitch', [0]*len(df)), 
                                   df.get('yaw', [0]*len(df))))
        if 'vx' in df:
            traj.velocities = list(zip(df['vx'], df['vy'], df['vz']))
        
        return traj
    
    def _process_ekf2_msg(self, msg: PoseStamped):
        """处理 EKF2 PoseStamped 消息"""
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        
        quat = msg.pose.orientation
        roll, pitch, yaw = self._quat_to_rpy(quat.x, quat.y, quat.z, quat.w)
        
        self.ekf2_data.timestamps.append(ts)
        self.ekf2_data.positions.append(pos)
        self.ekf2_data.attitudes.append((roll, pitch, yaw))
    
    def _process_ekf2_odom_msg(self, msg: Odometry):
        """处理 EKF2 Odometry 消息"""
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        
        quat = msg.pose.pose.orientation
        roll, pitch, yaw = self._quat_to_rpy(quat.x, quat.y, quat.z, quat.w)
        
        vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
        
        self.ekf2_data.timestamps.append(ts)
        self.ekf2_data.positions.append(pos)
        self.ekf2_data.attitudes.append((roll, pitch, yaw))
        self.ekf2_data.velocities.append(vel)
    
    def _process_iekf_msg(self, msg: Odometry):
        """处理 IEKF Odometry 消息"""
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        
        quat = msg.pose.pose.orientation
        roll, pitch, yaw = self._quat_to_rpy(quat.x, quat.y, quat.z, quat.w)
        
        vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
        
        self.iekf_data.timestamps.append(ts)
        self.iekf_data.positions.append(pos)
        self.iekf_data.attitudes.append((roll, pitch, yaw))
        self.iekf_data.velocities.append(vel)
    
    @staticmethod
    def _quat_to_rpy(x, y, z, w) -> Tuple[float, float, float]:
        """四元数转欧拉角"""
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
        sinp = 2*(w*y - z*x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        return roll, pitch, yaw
    
    def align_trajectories(self):
        """对齐两个轨迹 (使用时间戳)"""
        if self.ekf2_data is None or self.iekf_data is None:
            raise ValueError("数据未加载")
        
        # 获取共同的时间范围
        t_start = max(self.ekf2_data.timestamps[0], self.iekf_data.timestamps[0])
        t_end = min(self.ekf2_data.timestamps[-1], self.iekf_data.timestamps[-1])
        
        # 过滤数据
        ekf2_mask = np.logical_and(
            np.array(self.ekf2_data.timestamps) >= t_start,
            np.array(self.ekf2_data.timestamps) <= t_end
        )
        iekf_mask = np.logical_and(
            np.array(self.iekf_data.timestamps) >= t_start,
            np.array(self.iekf_data.timestamps) <= t_end
        )
        
        # 应用过滤
        self.ekf2_data.timestamps = np.array(self.ekf2_data.timestamps)[ekf2_mask].tolist()
        self.ekf2_data.positions = [p for i, p in enumerate(self.ekf2_data.positions) if ekf2_mask[i]]
        self.ekf2_data.attitudes = [a for i, a in enumerate(self.ekf2_data.attitudes) if ekf2_mask[i]]
        
        self.iekf_data.timestamps = np.array(self.iekf_data.timestamps)[iekf_mask].tolist()
        self.iekf_data.positions = [p for i, p in enumerate(self.iekf_data.positions) if iekf_mask[i]]
        self.iekf_data.attitudes = [a for i, a in enumerate(self.iekf_data.attitudes) if iekf_mask[i]]
        
        print(f"✓ 轨迹已对齐: {t_start:.2f}s - {t_end:.2f}s")
    
    def compute_metrics(self):
        """计算所有评估指标"""
        if self.ekf2_data is None or self.iekf_data is None:
            raise ValueError("数据未加载")
        
        # 插值到共同的采样率
        common_ts, ekf2_pos, iekf_pos = self._interpolate_to_common_rate()
        
        # 位置误差
        pos_errors = np.array([
            np.linalg.norm(np.array(e) - np.array(i))
            for e, i in zip(ekf2_pos, iekf_pos)
        ])
        
        self.metrics['position_rmse'] = np.sqrt(np.mean(pos_errors**2))
        self.metrics['position_mae'] = np.mean(np.abs(pos_errors))
        self.metrics['position_max_error'] = np.max(pos_errors)
        self.metrics['position_mean_error'] = np.mean(pos_errors)
        
        # 轨迹长度
        ekf2_lengths = np.linalg.norm(np.diff(ekf2_pos, axis=0), axis=1)
        iekf_lengths = np.linalg.norm(np.diff(iekf_pos, axis=0), axis=1)
        
        self.metrics['ekf2_trajectory_length'] = np.sum(ekf2_lengths)
        self.metrics['iekf_trajectory_length'] = np.sum(iekf_lengths)
        
        # 初始化时间
        self.metrics['ekf2_first_timestamp'] = self.ekf2_data.timestamps[0]
        self.metrics['iekf_first_timestamp'] = self.iekf_data.timestamps[0]
        self.metrics['initialization_delay'] = abs(
            self.ekf2_data.timestamps[0] - self.iekf_data.timestamps[0]
        )
        
        # 测试时长
        self.metrics['test_duration'] = common_ts[-1] - common_ts[0]
        
        return self.metrics
    
    def _interpolate_to_common_rate(self, rate_hz: float = 50.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """插值到共同采样率"""
        # 创建共同时间轴
        t_start = max(self.ekf2_data.timestamps[0], self.iekf_data.timestamps[0])
        t_end = min(self.ekf2_data.timestamps[-1], self.iekf_data.timestamps[-1])
        common_ts = np.arange(t_start, t_end, 1.0/rate_hz)
        
        # 插值 EKF2
        ekf2_pos = np.array(self.ekf2_data.positions)
        f_ekf2_x = interpolate.interp1d(self.ekf2_data.timestamps, ekf2_pos[:, 0], kind='linear', fill_value='extrapolate')
        f_ekf2_y = interpolate.interp1d(self.ekf2_data.timestamps, ekf2_pos[:, 1], kind='linear', fill_value='extrapolate')
        f_ekf2_z = interpolate.interp1d(self.ekf2_data.timestamps, ekf2_pos[:, 2], kind='linear', fill_value='extrapolate')
        
        ekf2_interp = np.column_stack([
            f_ekf2_x(common_ts),
            f_ekf2_y(common_ts),
            f_ekf2_z(common_ts)
        ])
        
        # 插值 IEKF
        iekf_pos = np.array(self.iekf_data.positions)
        f_iekf_x = interpolate.interp1d(self.iekf_data.timestamps, iekf_pos[:, 0], kind='linear', fill_value='extrapolate')
        f_iekf_y = interpolate.interp1d(self.iekf_data.timestamps, iekf_pos[:, 1], kind='linear', fill_value='extrapolate')
        f_iekf_z = interpolate.interp1d(self.iekf_data.timestamps, iekf_pos[:, 2], kind='linear', fill_value='extrapolate')
        
        iekf_interp = np.column_stack([
            f_iekf_x(common_ts),
            f_iekf_y(common_ts),
            f_iekf_z(common_ts)
        ])
        
        return common_ts, ekf2_interp, iekf_interp
    
    def export_trajectories(self):
        """导出轨迹为多种格式"""
        # TUM 格式 (evo 工具兼容)
        tum_ekf2 = self.output_dir / 'ekf2_trajectory.tum'
        with open(tum_ekf2, 'w') as f:
            f.write(self.ekf2_data.to_tum_format())
        print(f"✓ 导出 EKF2: {tum_ekf2}")
        
        tum_iekf = self.output_dir / 'iekf_trajectory.tum'
        with open(tum_iekf, 'w') as f:
            f.write(self.iekf_data.to_tum_format())
        print(f"✓ 导出 IEKF: {tum_iekf}")
        
        # CSV 格式
        csv_ekf2 = self.output_dir / 'ekf2_trajectory.csv'
        self.ekf2_data.to_dataframe().to_csv(csv_ekf2, index=False)
        print(f"✓ 导出 EKF2 CSV: {csv_ekf2}")
        
        csv_iekf = self.output_dir / 'iekf_trajectory.csv'
        self.iekf_data.to_dataframe().to_csv(csv_iekf, index=False)
        print(f"✓ 导出 IEKF CSV: {csv_iekf}")
    
    def plot_comparison(self):
        """绘制对比图表"""
        if self.ekf2_data is None or self.iekf_data is None:
            return
        
        # 创建图表
        fig, axes = plt.subplots(3, 2, figsize=(16, 12))
        fig.suptitle('EKF2 vs IEKF 对比分析', fontsize=16)
        
        # 位置轨迹 (XY 平面)
        ax = axes[0, 0]
        ekf2_pos = np.array(self.ekf2_data.positions)
        iekf_pos = np.array(self.iekf_data.positions)
        
        ax.plot(ekf2_pos[:, 0], ekf2_pos[:, 1], 'b-', label='EKF2', linewidth=2)
        ax.plot(iekf_pos[:, 0], iekf_pos[:, 1], 'r-', label='IEKF', linewidth=2)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('平面轨迹')
        ax.legend()
        ax.grid(True)
        ax.axis('equal')
        
        # 位置误差时间序列
        ax = axes[0, 1]
        common_ts, ekf2_pos_interp, iekf_pos_interp = self._interpolate_to_common_rate()
        pos_errors = np.linalg.norm(ekf2_pos_interp - iekf_pos_interp, axis=1)
        
        ax.plot(common_ts - common_ts[0], pos_errors, 'g-', linewidth=2)
        ax.set_xlabel('时间 (s)')
        ax.set_ylabel('位置误差 (m)')
        ax.set_title('位置误差时间序列')
        ax.grid(True)
        
        # 高度对比
        ax = axes[1, 0]
        ax.plot(np.array(self.ekf2_data.timestamps) - self.ekf2_data.timestamps[0], 
               [p[2] for p in self.ekf2_data.positions], 'b-', label='EKF2')
        ax.plot(np.array(self.iekf_data.timestamps) - self.iekf_data.timestamps[0], 
               [p[2] for p in self.iekf_data.positions], 'r-', label='IEKF')
        ax.set_xlabel('时间 (s)')
        ax.set_ylabel('高度 Z (m)')
        ax.set_title('高度对比')
        ax.legend()
        ax.grid(True)
        
        # 3D 轨迹
        ax = axes[1, 1]
        ax.remove()
        ax = fig.add_subplot(3, 2, 4, projection='3d')
        
        ax.plot(ekf2_pos[:, 0], ekf2_pos[:, 1], ekf2_pos[:, 2], 'b-', label='EKF2')
        ax.plot(iekf_pos[:, 0], iekf_pos[:, 1], iekf_pos[:, 2], 'r-', label='IEKF')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('3D 轨迹')
        ax.legend()
        
        # 姿态对比 (Yaw)
        ax = axes[2, 0]
        ekf2_yaw = [a[2] for a in self.ekf2_data.attitudes]
        iekf_yaw = [a[2] for a in self.iekf_data.attitudes]
        
        ax.plot(np.array(self.ekf2_data.timestamps) - self.ekf2_data.timestamps[0], 
               np.degrees(ekf2_yaw), 'b-', label='EKF2')
        ax.plot(np.array(self.iekf_data.timestamps) - self.iekf_data.timestamps[0], 
               np.degrees(iekf_yaw), 'r-', label='IEKF')
        ax.set_xlabel('时间 (s)')
        ax.set_ylabel('Yaw (°)')
        ax.set_title('Yaw 对比')
        ax.legend()
        ax.grid(True)
        
        # 误差统计
        ax = axes[2, 1]
        ax.axis('off')
        
        metrics_text = "评估指标:\n"
        metrics_text += f"位置 RMSE: {self.metrics.get('position_rmse', 0):.4f} m\n"
        metrics_text += f"位置 MAE: {self.metrics.get('position_mae', 0):.4f} m\n"
        metrics_text += f"位置最大误差: {self.metrics.get('position_max_error', 0):.4f} m\n"
        metrics_text += f"初始化延迟: {self.metrics.get('initialization_delay', 0):.4f} s\n"
        metrics_text += f"测试时长: {self.metrics.get('test_duration', 0):.2f} s\n"
        
        ax.text(0.05, 0.95, metrics_text, transform=ax.transAxes, 
               fontsize=10, verticalalignment='top', fontfamily='monospace',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # 保存图表
        plot_file = self.output_dir / 'ekf_iekf_comparison.png'
        plt.tight_layout()
        plt.savefig(plot_file, dpi=150)
        print(f"✓ 图表已保存: {plot_file}")
        plt.close()
    
    def generate_report(self):
        """生成对比报告"""
        report_file = self.output_dir / 'analysis_report.md'
        
        with open(report_file, 'w') as f:
            f.write("# EKF2 vs IEKF 对比分析报告\n\n")
            f.write(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            f.write("## 基本信息\n\n")
            f.write(f"| 指标 | 数值 |\n")
            f.write(f"|------|------|\n")
            f.write(f"| 测试时长 | {self.metrics.get('test_duration', 0):.2f} s |\n")
            f.write(f"| EKF2 消息数 | {len(self.ekf2_data.timestamps)} |\n")
            f.write(f"| IEKF 消息数 | {len(self.iekf_data.timestamps)} |\n")
            f.write(f"| EKF2 轨迹长度 | {self.metrics.get('ekf2_trajectory_length', 0):.2f} m |\n")
            f.write(f"| IEKF 轨迹长度 | {self.metrics.get('iekf_trajectory_length', 0):.2f} m |\n\n")
            
            f.write("## 定位精度评估\n\n")
            f.write(f"| 指标 | 数值 | 说明 |\n")
            f.write(f"|------|------|------|\n")
            f.write(f"| 位置 RMSE | {self.metrics.get('position_rmse', 0):.4f} m | 均方根误差 |\n")
            f.write(f"| 位置 MAE | {self.metrics.get('position_mae', 0):.4f} m | 平均绝对误差 |\n")
            f.write(f"| 位置最大误差 | {self.metrics.get('position_max_error', 0):.4f} m | 峰值误差 |\n")
            f.write(f"| 位置平均误差 | {self.metrics.get('position_mean_error', 0):.4f} m | 平均值 |\n\n")
            
            f.write("## 初始化性能\n\n")
            f.write(f"| 指标 | 数值 |\n")
            f.write(f"|------|------|\n")
            f.write(f"| EKF2 首次输出时间 | {self.metrics.get('ekf2_first_timestamp', 0):.2f} s |\n")
            f.write(f"| IEKF 首次输出时间 | {self.metrics.get('iekf_first_timestamp', 0):.2f} s |\n")
            f.write(f"| 初始化延迟 | {self.metrics.get('initialization_delay', 0):.4f} s |\n\n")
            
            f.write("## 输出文件\n\n")
            f.write("- `ekf2_trajectory.tum` - EKF2 轨迹 (TUM 格式，evo 兼容)\n")
            f.write("- `iekf_trajectory.tum` - IEKF 轨迹 (TUM 格式，evo 兼容)\n")
            f.write("- `ekf2_trajectory.csv` - EKF2 轨迹 (CSV 格式)\n")
            f.write("- `iekf_trajectory.csv` - IEKF 轨迹 (CSV 格式)\n")
            f.write("- `ekf_iekf_comparison.png` - 对比分析图表\n\n")
            
            f.write("## 分析建议\n\n")
            if self.metrics.get('position_rmse', 0) < 0.5:
                f.write("✓ **定位精度优秀** - 两种算法都有很好的定位精度\n\n")
            elif self.metrics.get('position_rmse', 0) < 2.0:
                f.write("○ **定位精度良好** - 符合一般定位需求\n\n")
            else:
                f.write("✗ **定位精度需改进** - 检查 GPS/IMU 配置\n\n")
            
            f.write("使用 evo 工具进行更深入的分析:\n\n")
            f.write("```bash\n")
            f.write("# 轨迹对齐与评估\n")
            f.write(f"evo_traj tum ekf2_trajectory.tum iekf_trajectory.tum --align --plot\n\n")
            f.write("# 绝对轨迹误差\n")
            f.write(f"evo_ape tum ekf2_trajectory.tum iekf_trajectory.tum -p --plot_mode=xyz\n\n")
            f.write("# 相对位置误差\n")
            f.write(f"evo_rpe tum ekf2_trajectory.tum iekf_trajectory.tum -p\n")
            f.write("```\n")
        
        print(f"✓ 报告已生成: {report_file}")


def main():
    parser = argparse.ArgumentParser(
        description='EKF2 vs IEKF 离线分析工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 从 rosbag2 分析
  python3 offline_analysis.py --rosbag /path/to/comparison_open_field.db3 --scenario open_field
  
  # 从 CSV 文件分析
  python3 offline_analysis.py --ekf2-csv ekf2.csv --iekf-csv iekf.csv --output results/
        """
    )
    
    parser.add_argument('--rosbag', type=str, help='ROS bag 文件路径')
    parser.add_argument('--ekf2-csv', type=str, help='EKF2 CSV 文件路径')
    parser.add_argument('--iekf-csv', type=str, help='IEKF CSV 文件路径')
    parser.add_argument('--scenario', type=str, default='test', help='测试场景名称')
    parser.add_argument('--output', type=str, default='./results', help='输出目录')
    
    args = parser.parse_args()
    
    # 验证输入
    if not args.rosbag and not (args.ekf2_csv and args.iekf_csv):
        parser.print_help()
        sys.exit(1)
    
    # 创建分析器
    output_dir = Path(args.output) / args.scenario
    analyzer = OfflineAnalyzer(output_dir)
    
    # 加载数据
    try:
        if args.rosbag:
            if not HAS_ROSBAG:
                print("❌ rosbag2 模块不可用，请安装: apt install ros-humble-rosbag2")
                sys.exit(1)
            analyzer.load_from_rosbag(args.rosbag)
        else:
            analyzer.load_from_csv(args.ekf2_csv, args.iekf_csv)
    except Exception as e:
        print(f"❌ 加载数据失败: {e}")
        sys.exit(1)
    
    # 处理数据
    try:
        analyzer.align_trajectories()
        analyzer.compute_metrics()
        analyzer.export_trajectories()
        analyzer.plot_comparison()
        analyzer.generate_report()
        
        print("\n✅ 分析完成！")
        print(f"📊 结果目录: {output_dir}")
        
    except Exception as e:
        print(f"❌ 分析失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
