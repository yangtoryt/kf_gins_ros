#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
KF-GINS 导航结果对比绘制脚本
支持 EKF 和 IEKF 结果的并行对比

用法:
  python3 plot_navresult_comparison.py \
    --ekf ../dataset/EKF_Navresult.nav \
    --iekf ../dataset/IEKF_Navresult.nav \
    --truth ../dataset/truth.nav
"""

import numpy as np
import math as m
import matplotlib.pyplot as plt
import os
import argparse
from pathlib import Path

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# WGS84参数
WGS84_RA = 6378137.0
WGS84_E1 = 0.00669437999013
WGS84_WIE = 7.2921151467e-5

D2R = np.pi / 180.0
R2D = 180.0 / np.pi

# 颜色配置
COLORS = {
    'ekf': '#1f77b4',   # 蓝色
    'iekf': '#ff7f0e'   # 橙色
}

STYLES = {
    'ekf': '-',         # 实线
    'iekf': '--'        # 虚线
}

LABELS = {
    'ekf': 'EKF',
    'iekf': 'IEKF (迭代)'
}


def radiusmn(lat):
    """计算子午圈半径和卯酉圈半径"""
    tmp = np.square(m.sin(lat))
    tmp = 1 - WGS84_E1 * tmp
    sqrttmp = np.sqrt(tmp)

    radm = WGS84_RA * (1 - WGS84_E1) / (sqrttmp * tmp)
    radn = WGS84_RA / sqrttmp
    return radm, radn


def drad2dm(rm, rn, pos, drad):
    """地理坐标系增量转成n系下坐标增量"""
    dm = np.zeros([3, 1])
    dm[0] = drad[0] * (rm + pos[2])
    dm[1] = drad[1] * (rn + pos[2]) * m.cos(pos[0])
    dm[2] = -drad[2]
    return dm


def load_navresult(navresult_filepath, method_type='ekf'):
    """
    加载导航结果文件
    @return: dict with pos, navresult, label, color, linestyle
    """
    
    if not os.path.exists(navresult_filepath):
        print(f"❌ 文件不存在: {navresult_filepath}")
        return None
    
    print(f"✓ 加载 {LABELS[method_type]}: {navresult_filepath}")
    navresult = np.loadtxt(navresult_filepath)

    # 小范围内将位置转到第一个位置确定的n系
    pos = np.zeros([len(navresult), 4])
    navresult_copy = navresult.copy()
    navresult_copy[:, 2:4] = navresult_copy[:, 2:4] * D2R
    pos[:, 0] = navresult_copy[:, 1]

    blh_station = navresult_copy[0, 2:5]
    rm, rn = radiusmn(blh_station[0])

    for i in range(len(pos)):
        delta_blh = navresult_copy[i, 2:5] - navresult_copy[0, 2:5]
        pos[i, 1:4] = drad2dm(rm, rn, blh_station, delta_blh).reshape(1, 3)

    return {
        'pos': pos,
        'navresult': navresult_copy,
        'label': LABELS[method_type],
        'color': COLORS[method_type],
        'linestyle': STYLES[method_type],
        'type': method_type
    }


def plot_horizontal_position(results_list):
    """绘制水平位置对比"""
    plt.figure(figsize=(12, 6))
    
    for result in results_list:
        plt.plot(result['pos'][:, 2], result['pos'][:, 1], 
                label=result['label'],
                color=result['color'],
                linestyle=result['linestyle'],
                linewidth=2,
                alpha=0.8)
    
    plt.axis('equal')
    plt.xlabel('East [m]', fontsize=12)
    plt.ylabel('North [m]', fontsize=12)
    plt.title('水平位置对比 / Horizontal Position Comparison', fontsize=14, fontweight='bold')
    plt.legend(fontsize=11)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()


def plot_height(results_list):
    """绘制高度对比"""
    plt.figure(figsize=(12, 6))
    
    for result in results_list:
        plt.plot(result['navresult'][:, 1], result['navresult'][:, 4],
                label=result['label'],
                color=result['color'],
                linestyle=result['linestyle'],
                linewidth=2,
                alpha=0.8)
    
    plt.xlabel('Time [s]', fontsize=12)
    plt.ylabel('Height [m]', fontsize=12)
    plt.title('高度对比 / Height Comparison', fontsize=14, fontweight='bold')
    plt.legend(fontsize=11)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()


def plot_velocity(results_list):
    """绘制速度对比"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    titles = ['North Velocity', 'East Velocity', 'Down Velocity']
    col_indices = [5, 6, 7]
    
    for idx, (ax, title, col_idx) in enumerate(zip(axes, titles, col_indices)):
        for result in results_list:
            ax.plot(result['navresult'][:, 1], result['navresult'][:, col_idx],
                   label=result['label'],
                   color=result['color'],
                   linestyle=result['linestyle'],
                   linewidth=2,
                   alpha=0.8)
        
        ax.set_xlabel('Time [s]', fontsize=11)
        ax.set_ylabel('Velocity [m/s]', fontsize=11)
        ax.set_title(title, fontsize=12)
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()


def plot_attitude(results_list):
    """绘制姿态对比"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    titles = ['Roll', 'Pitch', 'Yaw']
    col_indices = [8, 9, 10]
    
    for idx, (ax, title, col_idx) in enumerate(zip(axes, titles, col_indices)):
        for result in results_list:
            ax.plot(result['navresult'][:, 1], result['navresult'][:, col_idx],
                   label=result['label'],
                   color=result['color'],
                   linestyle=result['linestyle'],
                   linewidth=2,
                   alpha=0.8)
        
        ax.set_xlabel('Time [s]', fontsize=11)
        ax.set_ylabel('Angle [deg]', fontsize=11)
        ax.set_title(title, fontsize=12)
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()


def plot_position_error_difference(ekf_result, iekf_result):
    """
    绘制 EKF 和 IEKF 的位置误差差异
    显示 IEKF 相比 EKF 的改进
    """
    if ekf_result is None or iekf_result is None:
        print("⚠️  无法计算误差差异，需要 EKF 和 IEKF 两个结果")
        return
    
    # 计算位置差异（相对差异）
    min_len = min(len(ekf_result['pos']), len(iekf_result['pos']))
    
    # 插值到相同时间轴
    time_ekf = ekf_result['navresult'][:, 1]
    time_iekf = iekf_result['navresult'][:, 1]
    
    common_start = max(time_ekf[0], time_iekf[0])
    common_end = min(time_ekf[-1], time_iekf[-1])
    
    common_time = np.linspace(common_start, common_end, 1000)
    
    # 东北天坐标位置差异
    east_ekf = np.interp(common_time, time_ekf, ekf_result['pos'][:, 2])
    north_ekf = np.interp(common_time, time_ekf, ekf_result['pos'][:, 1])
    
    east_iekf = np.interp(common_time, time_iekf, iekf_result['pos'][:, 2])
    north_iekf = np.interp(common_time, time_iekf, iekf_result['pos'][:, 1])
    
    # 计算位置差异
    east_diff = east_iekf - east_ekf
    north_diff = north_iekf - north_ekf
    position_diff = np.sqrt(east_diff**2 + north_diff**2)
    
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    # 绘制东北天差异
    axes[0].plot(common_time, east_diff, label='East Difference', color='red', linewidth=2)
    axes[0].plot(common_time, north_diff, label='North Difference', color='green', linewidth=2)
    axes[0].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    axes[0].set_xlabel('Time [s]', fontsize=11)
    axes[0].set_ylabel('Position Difference [m]', fontsize=11)
    axes[0].set_title('IEKF 相对 EKF 的位置差异 / Position Difference (IEKF - EKF)', fontsize=12, fontweight='bold')
    axes[0].legend(fontsize=10)
    axes[0].grid(True, alpha=0.3)
    
    # 绘制总体位置差异
    axes[1].plot(common_time, position_diff, color='purple', linewidth=2)
    axes[1].fill_between(common_time, 0, position_diff, alpha=0.3, color='purple')
    axes[1].set_xlabel('Time [s]', fontsize=11)
    axes[1].set_ylabel('Horizontal Distance [m]', fontsize=11)
    axes[1].set_title('水平位置总体差异 / Horizontal Position Difference', fontsize=12, fontweight='bold')
    axes[1].grid(True, alpha=0.3)
    
    # 添加统计信息
    axes[1].text(0.02, 0.95, f'Mean Diff: {np.mean(position_diff):.3f} m\nMax Diff: {np.max(position_diff):.3f} m\nStd Dev: {np.std(position_diff):.3f} m',
                transform=axes[1].transAxes,
                verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
                fontsize=10)
    
    plt.tight_layout()


def plot_comparison_statistics(ekf_result, iekf_result):
    """绘制对比统计信息"""
    if ekf_result is None or iekf_result is None:
        print("⚠️  无法显示统计信息，需要 EKF 和 IEKF 两个结果")
        return
    
    fig = plt.figure(figsize=(14, 8))
    
    # 创建文本信息
    info_text = "EKF 与 IEKF 对比统计\n\n"
    
    # 数据长度
    ekf_len = len(ekf_result['navresult'])
    iekf_len = len(iekf_result['navresult'])
    info_text += f"EKF 数据点数: {ekf_len}\n"
    info_text += f"IEKF 数据点数: {iekf_len}\n"
    info_text += f"时间长度: {ekf_result['navresult'][-1, 1]:.1f} s\n\n"
    
    # 位置统计
    ekf_pos_north = ekf_result['pos'][:, 1]
    ekf_pos_east = ekf_result['pos'][:, 2]
    ekf_pos_height = ekf_result['navresult'][:, 4]
    
    iekf_pos_north = iekf_result['pos'][:, 1]
    iekf_pos_east = iekf_result['pos'][:, 2]
    iekf_pos_height = iekf_result['navresult'][:, 4]
    
    info_text += "=== 北向位置 ===\n"
    info_text += f"EKF:  Mean={np.mean(ekf_pos_north):.2f}, Std={np.std(ekf_pos_north):.3f}\n"
    info_text += f"IEKF: Mean={np.mean(iekf_pos_north):.2f}, Std={np.std(iekf_pos_north):.3f}\n\n"
    
    info_text += "=== 东向位置 ===\n"
    info_text += f"EKF:  Mean={np.mean(ekf_pos_east):.2f}, Std={np.std(ekf_pos_east):.3f}\n"
    info_text += f"IEKF: Mean={np.mean(iekf_pos_east):.2f}, Std={np.std(iekf_pos_east):.3f}\n\n"
    
    info_text += "=== 高度 ===\n"
    info_text += f"EKF:  Mean={np.mean(ekf_pos_height):.2f}, Std={np.std(ekf_pos_height):.3f}\n"
    info_text += f"IEKF: Mean={np.mean(iekf_pos_height):.2f}, Std={np.std(iekf_pos_height):.3f}\n\n"
    
    # 速度统计
    ekf_vel_norm = np.sqrt(ekf_result['navresult'][:, 5]**2 + 
                           ekf_result['navresult'][:, 6]**2 + 
                           ekf_result['navresult'][:, 7]**2)
    iekf_vel_norm = np.sqrt(iekf_result['navresult'][:, 5]**2 + 
                            iekf_result['navresult'][:, 6]**2 + 
                            iekf_result['navresult'][:, 7]**2)
    
    info_text += "=== 速度（合成） ===\n"
    info_text += f"EKF:  Mean={np.mean(ekf_vel_norm):.3f}, Std={np.std(ekf_vel_norm):.4f}\n"
    info_text += f"IEKF: Mean={np.mean(iekf_vel_norm):.3f}, Std={np.std(iekf_vel_norm):.4f}\n\n"
    
    # 姿态统计
    info_text += "=== 姿态（Roll, Pitch, Yaw） ===\n"
    info_text += f"EKF:  Roll={np.mean(ekf_result['navresult'][:, 8]):.3f}, Pitch={np.mean(ekf_result['navresult'][:, 9]):.3f}, Yaw={np.mean(ekf_result['navresult'][:, 10]):.3f}\n"
    info_text += f"IEKF: Roll={np.mean(iekf_result['navresult'][:, 8]):.3f}, Pitch={np.mean(iekf_result['navresult'][:, 9]):.3f}, Yaw={np.mean(iekf_result['navresult'][:, 10]):.3f}\n"
    
    ax = fig.add_subplot(111)
    ax.text(0.05, 0.95, info_text, transform=ax.transAxes,
           fontsize=11, verticalalignment='top', family='monospace',
           bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    ax.axis('off')
    
    plt.tight_layout()


def main():
    parser = argparse.ArgumentParser(description='KF-GINS 导航结果对比绘制')
    parser.add_argument('--ekf', type=str, help='EKF 导航结果文件')
    parser.add_argument('--iekf', type=str, help='IEKF 导航结果文件')
    parser.add_argument('--truth', type=str, help='真值文件（可选）')
    parser.add_argument('--output', type=str, default=None, help='保存图表到文件夹（可选）')
    
    args = parser.parse_args()
    
    results_list = []
    ekf_result = None
    iekf_result = None
    
    # 加载 EKF 结果
    if args.ekf:
        ekf_result = load_navresult(args.ekf, 'ekf')
        if ekf_result:
            results_list.append(ekf_result)
    
    # 加载 IEKF 结果
    if args.iekf:
        iekf_result = load_navresult(args.iekf, 'iekf')
        if iekf_result:
            results_list.append(iekf_result)
    
    if not results_list:
        print("❌ 没有可用的导航结果数据！")
        return
    
    print(f"\n✓ 成功加载 {len(results_list)} 个数据文件")
    print(f"✓ 开始绘制对比图表...\n")
    
    # 绘制对比图表
    plot_horizontal_position(results_list)
    plot_height(results_list)
    plot_velocity(results_list)
    plot_attitude(results_list)
    
    # 如果有两个结果，绘制差异分析
    if len(results_list) == 2:
        print("✓ 绘制差异分析...")
        plot_position_error_difference(ekf_result, iekf_result)
        plot_comparison_statistics(ekf_result, iekf_result)
    
    print("✓ 所有图表已准备完毕！")
    
    # 保存图表（可选）
    if args.output:
        output_dir = Path(args.output)
        output_dir.mkdir(parents=True, exist_ok=True)
        
        for i, fig_num in enumerate(plt.get_fignums()):
            fig = plt.figure(fig_num)
            filename = output_dir / f"comparison_{i:02d}.png"
            fig.savefig(filename, dpi=150, bbox_inches='tight')
            print(f"✓ 已保存: {filename}")
        
        print(f"✓ 所有图表已保存到: {output_dir}")
    
    plt.show()


if __name__ == '__main__':
    main()
