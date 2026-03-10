#!/usr/bin/env python3
"""
PX4 vs KF-GINS 导航性能对比分析

功能：
- 加载 CSV 数据
- 计算性能指标（RMSE, MAE, 稳定性等）
- 生成对比图表
- 输出性能报告

使用：
    python3 compare_algorithms.py \
      --px4_results px4_baseline_results.csv \
      --gins_results gins_results.csv \
      --output_dir results/comparison
"""

import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from pathlib import Path


class NavigationAnalyzer:
    """导航算法性能分析工具"""
    
    def __init__(self, output_dir='results/comparison'):
        """初始化分析器"""
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # 配置 matplotlib 中文支持
        plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Liberation Sans']
        plt.rcParams['axes.unicode_minus'] = False
        
        self.results = {}
    
    def load_data(self, csv_file, algorithm_name):
        """加载 CSV 数据"""
        print(f'加载 {algorithm_name} 数据: {csv_file}')
        df = pd.read_csv(csv_file)
        self.results[algorithm_name] = df
        return df
    
    def calculate_metrics(self, df, algorithm_name):
        """计算性能指标"""
        metrics = {}
        
        # 位置误差
        pos_error = np.sqrt(
            df['error_pos_x']**2 + 
            df['error_pos_y']**2 + 
            df['error_pos_z']**2
        )
        
        # 速度误差
        vel_error = np.sqrt(
            df['error_vel_x']**2 + 
            df['error_vel_y']**2 + 
            df['error_vel_z']**2
        )
        
        # 姿态误差（转换为度）
        attitude_error = np.sqrt(
            df['error_roll']**2 + 
            df['error_pitch']**2 + 
            df['error_yaw']**2
        )
        
        # 计算各项指标
        metrics['position'] = {
            'rmse': float(np.sqrt(np.mean(pos_error**2))),
            'mae': float(np.mean(np.abs(pos_error))),
            'std': float(np.std(pos_error)),
            'max': float(np.max(pos_error)),
            'min': float(np.min(pos_error))
        }
        
        metrics['velocity'] = {
            'rmse': float(np.sqrt(np.mean(vel_error**2))),
            'mae': float(np.mean(np.abs(vel_error))),
            'std': float(np.std(vel_error)),
            'max': float(np.max(vel_error)),
            'min': float(np.min(vel_error))
        }
        
        metrics['attitude'] = {
            'rmse': float(np.sqrt(np.mean(attitude_error**2))),
            'mae': float(np.mean(np.abs(attitude_error))),
            'std': float(np.std(attitude_error)),
            'max': float(np.max(attitude_error)),
            'min': float(np.min(attitude_error))
        }
        
        # 稳定性指标（连续帧间变化的标准差）
        metrics['stability'] = {
            'position': float(np.std(np.diff(pos_error))),
            'velocity': float(np.std(np.diff(vel_error))),
            'attitude': float(np.std(np.diff(attitude_error)))
        }
        
        return metrics, {
            'position_error': pos_error,
            'velocity_error': vel_error,
            'attitude_error': attitude_error
        }
    
    def print_report(self, metrics_dict):
        """打印性能报告"""
        report = []
        report.append('\n' + '='*80)
        report.append('PX4 vs KF-GINS 导航性能对比分析报告')
        report.append('='*80 + '\n')
        
        # 位置精度对比
        report.append('【位置精度对比 (米)】')
        report.append('-' * 80)
        report.append(f"{'指标':<15} {'PX4':<15} {'KF-GINS':<15} {'改进百分比':<15}")
        report.append('-' * 80)
        
        for metric in ['rmse', 'mae', 'std', 'max']:
            px4_val = metrics_dict['PX4']['position'].get(metric, 0)
            gins_val = metrics_dict['KF-GINS']['position'].get(metric, 0)
            if px4_val > 0:
                improvement = ((px4_val - gins_val) / px4_val) * 100
            else:
                improvement = 0
            report.append(
                f'{metric.upper():<15} {px4_val:<15.4f} {gins_val:<15.4f} '
                f'{improvement:>+.1f}%')
        
        # 速度精度对比
        report.append('\n【速度精度对比 (m/s)】')
        report.append('-' * 80)
        report.append(f"{'指标':<15} {'PX4':<15} {'KF-GINS':<15} {'改进百分比':<15}")
        report.append('-' * 80)
        
        for metric in ['rmse', 'mae', 'std', 'max']:
            px4_val = metrics_dict['PX4']['velocity'].get(metric, 0)
            gins_val = metrics_dict['KF-GINS']['velocity'].get(metric, 0)
            if px4_val > 0:
                improvement = ((px4_val - gins_val) / px4_val) * 100
            else:
                improvement = 0
            report.append(
                f'{metric.upper():<15} {px4_val:<15.4f} {gins_val:<15.4f} '
                f'{improvement:>+.1f}%')
        
        # 姿态精度对比
        report.append('\n【姿态精度对比 (度)】')
        report.append('-' * 80)
        report.append(f"{'指标':<15} {'PX4':<15} {'KF-GINS':<15} {'改进百分比':<15}")
        report.append('-' * 80)
        
        for metric in ['rmse', 'mae', 'std', 'max']:
            px4_val = metrics_dict['PX4']['attitude'].get(metric, 0)
            gins_val = metrics_dict['KF-GINS']['attitude'].get(metric, 0)
            if px4_val > 0:
                improvement = ((px4_val - gins_val) / px4_val) * 100
            else:
                improvement = 0
            report.append(
                f'{metric.upper():<15} {px4_val:<15.4f} {gins_val:<15.4f} '
                f'{improvement:>+.1f}%')
        
        # 稳定性对比
        report.append('\n【稳定性对比 (方差)】')
        report.append('-' * 80)
        report.append(f"{'类型':<15} {'PX4':<15} {'KF-GINS':<15} {'改进百分比':<15}")
        report.append('-' * 80)
        
        for stability_type in ['position', 'velocity', 'attitude']:
            px4_val = metrics_dict['PX4']['stability'].get(stability_type, 0)
            gins_val = metrics_dict['KF-GINS']['stability'].get(stability_type, 0)
            if px4_val > 0:
                improvement = ((px4_val - gins_val) / px4_val) * 100
            else:
                improvement = 0
            report.append(
                f'{stability_type.upper():<15} {px4_val:<15.6f} {gins_val:<15.6f} '
                f'{improvement:>+.1f}%')
        
        # 总结
        report.append('\n' + '='*80)
        report.append('【总体结论】')
        report.append('='*80)
        
        px4_pos_rmse = metrics_dict['PX4']['position']['rmse']
        gins_pos_rmse = metrics_dict['KF-GINS']['position']['rmse']
        
        if gins_pos_rmse < px4_pos_rmse:
            improvement_pct = ((px4_pos_rmse - gins_pos_rmse) / px4_pos_rmse) * 100
            report.append(
                f'✅ KF-GINS 算法在位置精度上优于 PX4 原生算法 {improvement_pct:.1f}%')
        else:
            report.append(f'ℹ️  两种算法位置精度相当')
        
        report.append('\n' + '='*80 + '\n')
        
        report_text = '\n'.join(report)
        print(report_text)
        
        return report_text
    
    def plot_comparison(self, metrics_dict, errors_dict):
        """生成对比图表"""
        
        # 创建子图
        fig, axes = plt.subplots(3, 2, figsize=(14, 12))
        fig.suptitle('PX4 vs KF-GINS Navigation Performance Comparison', 
                     fontsize=14, fontweight='bold')
        
        algorithms = ['PX4', 'KF-GINS']
        colors = ['#FF6B6B', '#4ECDC4']
        
        # 1. 位置误差时间序列
        ax = axes[0, 0]
        time = self.results['PX4']['timestamp'].values
        ax.plot(time, errors_dict['PX4']['position_error'], 
                label='PX4', color=colors[0], alpha=0.7)
        ax.plot(time, errors_dict['KF-GINS']['position_error'], 
                label='KF-GINS', color=colors[1], alpha=0.7)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position Error (m)')
        ax.set_title('Position Error Over Time')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # 2. 速度误差时间序列
        ax = axes[0, 1]
        ax.plot(time, errors_dict['PX4']['velocity_error'], 
                label='PX4', color=colors[0], alpha=0.7)
        ax.plot(time, errors_dict['KF-GINS']['velocity_error'], 
                label='KF-GINS', color=colors[1], alpha=0.7)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity Error (m/s)')
        ax.set_title('Velocity Error Over Time')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # 3. 位置 RMSE 对比
        ax = axes[1, 0]
        pos_rmse = [metrics_dict['PX4']['position']['rmse'],
                    metrics_dict['KF-GINS']['position']['rmse']]
        bars = ax.bar(algorithms, pos_rmse, color=colors, alpha=0.7)
        ax.set_ylabel('RMSE (m)')
        ax.set_title('Position RMSE Comparison')
        ax.set_ylim(0, max(pos_rmse) * 1.2)
        for bar, val in zip(bars, pos_rmse):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{val:.4f}', ha='center', va='bottom')
        
        # 4. 速度 RMSE 对比
        ax = axes[1, 1]
        vel_rmse = [metrics_dict['PX4']['velocity']['rmse'],
                    metrics_dict['KF-GINS']['velocity']['rmse']]
        bars = ax.bar(algorithms, vel_rmse, color=colors, alpha=0.7)
        ax.set_ylabel('RMSE (m/s)')
        ax.set_title('Velocity RMSE Comparison')
        ax.set_ylim(0, max(vel_rmse) * 1.2)
        for bar, val in zip(bars, vel_rmse):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{val:.4f}', ha='center', va='bottom')
        
        # 5. 姿态误差时间序列
        ax = axes[2, 0]
        ax.plot(time, errors_dict['PX4']['attitude_error'], 
                label='PX4', color=colors[0], alpha=0.7)
        ax.plot(time, errors_dict['KF-GINS']['attitude_error'], 
                label='KF-GINS', color=colors[1], alpha=0.7)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Attitude Error (deg)')
        ax.set_title('Attitude Error Over Time')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # 6. 误差分布直方图
        ax = axes[2, 1]
        ax.hist(errors_dict['PX4']['position_error'], bins=30, 
                label='PX4', color=colors[0], alpha=0.6)
        ax.hist(errors_dict['KF-GINS']['position_error'], bins=30, 
                label='KF-GINS', color=colors[1], alpha=0.6)
        ax.set_xlabel('Position Error (m)')
        ax.set_ylabel('Frequency')
        ax.set_title('Error Distribution')
        ax.legend()
        ax.grid(True, alpha=0.3, axis='y')
        
        plt.tight_layout()
        
        # 保存图表
        output_path = self.output_dir / 'comparison_chart.png'
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f'图表已保存: {output_path}')
        
        return fig
    
    def run_analysis(self, px4_file, gins_file):
        """执行完整分析流程"""
        print('\n' + '='*80)
        print('PX4 vs KF-GINS Navigation Performance Analysis')
        print('='*80)
        
        # 1. 加载数据
        print('\n[1/4] 加载数据...')
        self.load_data(px4_file, 'PX4')
        self.load_data(gins_file, 'KF-GINS')
        
        # 2. 计算指标
        print('\n[2/4] 计算性能指标...')
        metrics_dict = {}
        errors_dict = {}
        
        for algo_name in ['PX4', 'KF-GINS']:
            metrics, errors = self.calculate_metrics(self.results[algo_name], algo_name)
            metrics_dict[algo_name] = metrics
            errors_dict[algo_name] = errors
        
        # 3. 生成报告
        print('\n[3/4] 生成分析报告...')
        report = self.print_report(metrics_dict)
        
        # 保存报告文本
        report_path = self.output_dir / 'performance_report.txt'
        with open(report_path, 'w') as f:
            f.write(report)
        print(f'报告已保存: {report_path}')
        
        # 4. 生成图表
        print('\n[4/4] 生成对比图表...')
        self.plot_comparison(metrics_dict, errors_dict)
        
        print('\n' + '='*80)
        print('分析完成！')
        print('='*80 + '\n')
        
        return metrics_dict, report


def main():
    """命令行入口"""
    parser = argparse.ArgumentParser(
        description='PX4 vs KF-GINS Navigation Performance Analysis')
    
    parser.add_argument('--px4_results', required=True,
                       help='PX4 baseline results CSV file')
    parser.add_argument('--gins_results', required=True,
                       help='KF-GINS results CSV file')
    parser.add_argument('--output_dir', default='results/comparison',
                       help='Output directory for reports and plots')
    
    args = parser.parse_args()
    
    # 检查文件存在性
    if not os.path.exists(args.px4_results):
        print(f'错误: PX4 结果文件不存在: {args.px4_results}')
        return
    
    if not os.path.exists(args.gins_results):
        print(f'错误: KF-GINS 结果文件不存在: {args.gins_results}')
        return
    
    # 执行分析
    analyzer = NavigationAnalyzer(args.output_dir)
    analyzer.run_analysis(args.px4_results, args.gins_results)


if __name__ == '__main__':
    main()
