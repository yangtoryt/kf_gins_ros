#!/usr/bin/env python3
"""
Auto Compare Results - Automated Analysis of PX4 vs KF-GINS Performance
=========================================================================

This script automatically:
1. Finds PX4 baseline and KF-GINS result CSV files
2. Loads and validates the data
3. Calculates comprehensive performance metrics
4. Generates comparison visualizations
5. Creates detailed performance report

Usage:
    python3 auto_compare_results.py [--results_dir <path>] [--output_dir <path>]

Default Paths:
    Results: ~/kf_gins_ws/results/
    Output: ~/kf_gins_ws/results/analysis/

Example:
    python3 auto_compare_results.py --results_dir ~/kf_gins_ws/results
"""

import os
import sys
import glob
import argparse
from pathlib import Path
from datetime import datetime
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-GUI backend
import matplotlib.pyplot as plt
from matplotlib import rcParams
import warnings
warnings.filterwarnings('ignore')

# Set up matplotlib for consistent rendering
rcParams['figure.dpi'] = 100
rcParams['savefig.dpi'] = 150
rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial']
rcParams['axes.unicode_minus'] = False


class AutoAnalyzer:
    """Automatically analyze and compare navigation algorithm results."""
    
    def __init__(self, results_dir, output_dir):
        self.results_dir = Path(results_dir).expanduser()
        self.output_dir = Path(output_dir).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Data containers
        self.px4_data = None
        self.gins_data = None
        self.metrics = {}
        self.errors = {}
        
        print(f"AutoAnalyzer initialized")
        print(f"  Results directory: {self.results_dir}")
        print(f"  Output directory: {self.output_dir}")
    
    def find_csv_files(self):
        """Find PX4 and KF-GINS CSV files in results directory."""
        print("\n[1/5] Finding CSV files...")
        
        if not self.results_dir.exists():
            print(f"✗ Results directory not found: {self.results_dir}")
            return False
        
        # Find PX4 baseline
        px4_files = list(self.results_dir.glob('*px4*baseline*.csv'))
        if not px4_files:
            px4_files = list(self.results_dir.glob('*px4*.csv'))
        
        # Find KF-GINS results
        gins_files = list(self.results_dir.glob('*gins*.csv'))
        gins_files += list(self.results_dir.glob('*kf_gins*.csv'))
        gins_files += list(self.results_dir.glob('*iekf*.csv'))
        
        if not px4_files:
            print(f"✗ No PX4 baseline CSV files found")
            print(f"  Looking in: {self.results_dir}")
            print(f"  Expected file pattern: *px4*baseline*.csv or *px4*.csv")
            return False
        
        if not gins_files:
            print(f"✗ No KF-GINS CSV files found")
            print(f"  Looking in: {self.results_dir}")
            print(f"  Expected file pattern: *gins*.csv or *kf_gins*.csv")
            return False
        
        self.px4_file = px4_files[0]
        self.gins_file = gins_files[0]
        
        print(f"✓ Found PX4 file: {self.px4_file.name}")
        print(f"✓ Found KF-GINS file: {self.gins_file.name}")
        
        return True
    
    def load_data(self):
        """Load CSV data files."""
        print("\n[2/5] Loading data...")
        
        try:
            self.px4_data = pd.read_csv(self.px4_file)
            print(f"✓ PX4 data loaded: {len(self.px4_data)} rows, {len(self.px4_data.columns)} columns")
            print(f"  Columns: {', '.join(self.px4_data.columns[:5])}...")
            
            self.gins_data = pd.read_csv(self.gins_file)
            print(f"✓ KF-GINS data loaded: {len(self.gins_data)} rows, {len(self.gins_data.columns)} columns")
            print(f"  Columns: {', '.join(self.gins_data.columns[:5])}...")
            
            return True
        except Exception as e:
            print(f"✗ Error loading data: {e}")
            return False
    
    def calculate_metrics(self):
        """Calculate performance metrics for both algorithms."""
        print("\n[3/5] Calculating metrics...")
        
        try:
            # PX4 metrics
            self.metrics['PX4'] = self._compute_metrics(self.px4_data, 'PX4')
            print(f"✓ PX4 metrics calculated")
            
            # KF-GINS metrics
            self.metrics['KF-GINS'] = self._compute_metrics(self.gins_data, 'KF-GINS')
            print(f"✓ KF-GINS metrics calculated")
            
            return True
        except Exception as e:
            print(f"✗ Error calculating metrics: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _compute_metrics(self, df, name):
        """Compute metrics from a dataframe."""
        metrics = {'algorithm': name, 'records': len(df)}
        
        # Position error columns (try multiple naming conventions)
        pos_error_cols = [col for col in df.columns 
                         if 'position_error' in col.lower() or 'pos_error' in col.lower()]
        vel_error_cols = [col for col in df.columns 
                         if 'velocity_error' in col.lower() or 'vel_error' in col.lower()]
        att_error_cols = [col for col in df.columns 
                         if 'attitude_error' in col.lower() or 'att_error' in col.lower()]
        
        # Calculate position metrics
        if pos_error_cols:
            pos_errors = df[pos_error_cols].abs().values.flatten()
            metrics['position_rmse'] = float(np.sqrt(np.mean(pos_errors**2)))
            metrics['position_mae'] = float(np.mean(pos_errors))
            metrics['position_std'] = float(np.std(pos_errors))
            metrics['position_max'] = float(np.max(pos_errors))
            metrics['position_min'] = float(np.min(pos_errors))
        
        # Calculate velocity metrics
        if vel_error_cols:
            vel_errors = df[vel_error_cols].abs().values.flatten()
            metrics['velocity_rmse'] = float(np.sqrt(np.mean(vel_errors**2)))
            metrics['velocity_mae'] = float(np.mean(vel_errors))
            metrics['velocity_std'] = float(np.std(vel_errors))
            metrics['velocity_max'] = float(np.max(vel_errors))
            metrics['velocity_min'] = float(np.min(vel_errors))
        
        # Calculate attitude metrics
        if att_error_cols:
            att_errors = df[att_error_cols].abs().values.flatten()
            metrics['attitude_rmse'] = float(np.sqrt(np.mean(att_errors**2)))
            metrics['attitude_mae'] = float(np.mean(att_errors))
            metrics['attitude_std'] = float(np.std(att_errors))
            metrics['attitude_max'] = float(np.max(att_errors))
            metrics['attitude_min'] = float(np.min(att_errors))
        
        # Store raw errors for plotting
        if name not in self.errors:
            self.errors[name] = {}
        if pos_error_cols:
            self.errors[name]['position'] = df[pos_error_cols].values
        if vel_error_cols:
            self.errors[name]['velocity'] = df[vel_error_cols].values
        if att_error_cols:
            self.errors[name]['attitude'] = df[att_error_cols].values
        
        return metrics
    
    def generate_report(self):
        """Generate text report of metrics."""
        print("\n[4/5] Generating report...")
        
        report_file = self.output_dir / 'performance_report.txt'
        
        with open(report_file, 'w') as f:
            f.write("="*70 + "\n")
            f.write("PX4 vs KF-GINS Navigation Performance Comparison Report\n")
            f.write("="*70 + "\n\n")
            
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"PX4 Data: {self.px4_file.name}\n")
            f.write(f"KF-GINS Data: {self.gins_file.name}\n\n")
            
            # Position metrics
            f.write("-"*70 + "\n")
            f.write("POSITION ACCURACY (meters)\n")
            f.write("-"*70 + "\n")
            f.write(f"{'Metric':<15} {'PX4':>15} {'KF-GINS':>15} {'Improvement':>15}\n")
            f.write("-"*70 + "\n")
            
            for metric in ['rmse', 'mae', 'std', 'max', 'min']:
                px4_val = self.metrics['PX4'].get(f'position_{metric}', 0)
                gins_val = self.metrics['KF-GINS'].get(f'position_{metric}', 0)
                if px4_val > 0:
                    improve = ((px4_val - gins_val) / px4_val) * 100
                else:
                    improve = 0
                f.write(f"{metric.upper():<15} {px4_val:>15.6f} {gins_val:>15.6f} {improve:>14.1f}%\n")
            
            # Velocity metrics
            f.write("\n" + "-"*70 + "\n")
            f.write("VELOCITY ACCURACY (m/s)\n")
            f.write("-"*70 + "\n")
            f.write(f"{'Metric':<15} {'PX4':>15} {'KF-GINS':>15} {'Improvement':>15}\n")
            f.write("-"*70 + "\n")
            
            for metric in ['rmse', 'mae', 'std', 'max', 'min']:
                px4_val = self.metrics['PX4'].get(f'velocity_{metric}', 0)
                gins_val = self.metrics['KF-GINS'].get(f'velocity_{metric}', 0)
                if px4_val > 0:
                    improve = ((px4_val - gins_val) / px4_val) * 100
                else:
                    improve = 0
                f.write(f"{metric.upper():<15} {px4_val:>15.6f} {gins_val:>15.6f} {improve:>14.1f}%\n")
            
            # Attitude metrics
            f.write("\n" + "-"*70 + "\n")
            f.write("ATTITUDE ACCURACY (degrees)\n")
            f.write("-"*70 + "\n")
            f.write(f"{'Metric':<15} {'PX4':>15} {'KF-GINS':>15} {'Improvement':>15}\n")
            f.write("-"*70 + "\n")
            
            for metric in ['rmse', 'mae', 'std', 'max', 'min']:
                px4_val = self.metrics['PX4'].get(f'attitude_{metric}', 0)
                gins_val = self.metrics['KF-GINS'].get(f'attitude_{metric}', 0)
                if px4_val > 0:
                    improve = ((px4_val - gins_val) / px4_val) * 100
                else:
                    improve = 0
                f.write(f"{metric.upper():<15} {px4_val:>15.6f} {gins_val:>15.6f} {improve:>14.1f}%\n")
            
            f.write("\n" + "="*70 + "\n")
            f.write("SUMMARY\n")
            f.write("="*70 + "\n")
            
            # Overall summary
            px4_pos = self.metrics['PX4'].get('position_rmse', 0)
            gins_pos = self.metrics['KF-GINS'].get('position_rmse', 0)
            if px4_pos > 0:
                pos_improve = ((px4_pos - gins_pos) / px4_pos) * 100
                f.write(f"Position RMSE Improvement: {pos_improve:.1f}%\n")
            
            px4_vel = self.metrics['PX4'].get('velocity_rmse', 0)
            gins_vel = self.metrics['KF-GINS'].get('velocity_rmse', 0)
            if px4_vel > 0:
                vel_improve = ((px4_vel - gins_vel) / px4_vel) * 100
                f.write(f"Velocity RMSE Improvement: {vel_improve:.1f}%\n")
            
            px4_att = self.metrics['PX4'].get('attitude_rmse', 0)
            gins_att = self.metrics['KF-GINS'].get('attitude_rmse', 0)
            if px4_att > 0:
                att_improve = ((px4_att - gins_att) / px4_att) * 100
                f.write(f"Attitude RMSE Improvement: {att_improve:.1f}%\n")
            
            f.write("\n" + "="*70 + "\n")
        
        print(f"✓ Report saved to: {report_file}")
        return report_file
    
    def generate_plots(self):
        """Generate comparison plots."""
        print("\n[5/5] Generating plots...")
        
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        fig.suptitle('PX4 vs KF-GINS Navigation Performance Comparison', 
                     fontsize=16, fontweight='bold')
        
        colors = {'PX4': '#FF6B6B', 'KF-GINS': '#4ECDC4'}
        
        # Plot 1: Position RMSE
        ax = axes[0, 0]
        algorithms = list(self.metrics.keys())
        pos_rmse = [self.metrics[alg].get('position_rmse', 0) for alg in algorithms]
        bars = ax.bar(algorithms, pos_rmse, color=[colors[alg] for alg in algorithms])
        ax.set_ylabel('RMSE (meters)')
        ax.set_title('Position RMSE')
        ax.grid(axis='y', alpha=0.3)
        for bar, val in zip(bars, pos_rmse):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{val:.4f}', ha='center', va='bottom')
        
        # Plot 2: Velocity RMSE
        ax = axes[0, 1]
        vel_rmse = [self.metrics[alg].get('velocity_rmse', 0) for alg in algorithms]
        bars = ax.bar(algorithms, vel_rmse, color=[colors[alg] for alg in algorithms])
        ax.set_ylabel('RMSE (m/s)')
        ax.set_title('Velocity RMSE')
        ax.grid(axis='y', alpha=0.3)
        for bar, val in zip(bars, vel_rmse):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{val:.4f}', ha='center', va='bottom')
        
        # Plot 3: Attitude RMSE
        ax = axes[0, 2]
        att_rmse = [self.metrics[alg].get('attitude_rmse', 0) for alg in algorithms]
        bars = ax.bar(algorithms, att_rmse, color=[colors[alg] for alg in algorithms])
        ax.set_ylabel('RMSE (degrees)')
        ax.set_title('Attitude RMSE')
        ax.grid(axis='y', alpha=0.3)
        for bar, val in zip(bars, att_rmse):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{val:.4f}', ha='center', va='bottom')
        
        # Plot 4: Position MAE
        ax = axes[1, 0]
        pos_mae = [self.metrics[alg].get('position_mae', 0) for alg in algorithms]
        bars = ax.bar(algorithms, pos_mae, color=[colors[alg] for alg in algorithms])
        ax.set_ylabel('MAE (meters)')
        ax.set_title('Position MAE')
        ax.grid(axis='y', alpha=0.3)
        for bar, val in zip(bars, pos_mae):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{val:.4f}', ha='center', va='bottom')
        
        # Plot 5: Velocity MAE
        ax = axes[1, 1]
        vel_mae = [self.metrics[alg].get('velocity_mae', 0) for alg in algorithms]
        bars = ax.bar(algorithms, vel_mae, color=[colors[alg] for alg in algorithms])
        ax.set_ylabel('MAE (m/s)')
        ax.set_title('Velocity MAE')
        ax.grid(axis='y', alpha=0.3)
        for bar, val in zip(bars, vel_mae):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{val:.4f}', ha='center', va='bottom')
        
        # Plot 6: Attitude MAE
        ax = axes[1, 2]
        att_mae = [self.metrics[alg].get('attitude_mae', 0) for alg in algorithms]
        bars = ax.bar(algorithms, att_mae, color=[colors[alg] for alg in algorithms])
        ax.set_ylabel('MAE (degrees)')
        ax.set_title('Attitude MAE')
        ax.grid(axis='y', alpha=0.3)
        for bar, val in zip(bars, att_mae):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{val:.4f}', ha='center', va='bottom')
        
        plt.tight_layout()
        
        plot_file = self.output_dir / 'comparison_chart.png'
        plt.savefig(plot_file, dpi=150, bbox_inches='tight')
        print(f"✓ Plot saved to: {plot_file}")
        
        plt.close()
        return plot_file
    
    def run(self):
        """Run complete analysis pipeline."""
        print("\n" + "="*70)
        print("AUTO ANALYSIS: PX4 vs KF-GINS Performance Comparison")
        print("="*70)
        
        # Pipeline
        if not self.find_csv_files():
            return False
        
        if not self.load_data():
            return False
        
        if not self.calculate_metrics():
            return False
        
        self.generate_report()
        self.generate_plots()
        
        print("\n" + "="*70)
        print("ANALYSIS COMPLETE!")
        print("="*70)
        print(f"\nOutput files:")
        print(f"  Report: {self.output_dir}/performance_report.txt")
        print(f"  Charts: {self.output_dir}/comparison_chart.png")
        print(f"\nView results with:")
        print(f"  cat {self.output_dir}/performance_report.txt")
        print(f"  display {self.output_dir}/comparison_chart.png")
        
        return True


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Automatically analyze and compare PX4 vs KF-GINS results',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 auto_compare_results.py
  python3 auto_compare_results.py --results_dir ~/kf_gins_ws/results
  python3 auto_compare_results.py --results_dir ~/kf_gins_ws/results --output_dir ~/kf_gins_ws/results/analysis
        """
    )
    
    parser.add_argument('--results_dir', 
                       default='~/kf_gins_ws/results',
                       help='Directory containing CSV result files (default: ~/kf_gins_ws/results)')
    parser.add_argument('--output_dir',
                       default='~/kf_gins_ws/results/analysis',
                       help='Directory for output files (default: ~/kf_gins_ws/results/analysis)')
    
    args = parser.parse_args()
    
    analyzer = AutoAnalyzer(args.results_dir, args.output_dir)
    success = analyzer.run()
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
