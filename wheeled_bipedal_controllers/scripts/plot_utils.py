#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import glob
import argparse

import pandas as pd
import matplotlib.pyplot as plt


DEFAULT_LOG_DIR = (
    '/home/handshow/wheeled_bipedal_ws/src/'
    'wheeled_bipedal_simulator/'
    'wheeled_bipedal_controllers/logs'
)

# 让 SVG 中的文字尽量保持为可编辑文本
plt.rcParams['svg.fonttype'] = 'none'

# 全局字体大小设置
plt.rcParams['font.size'] = 16          # 默认字体
plt.rcParams['axes.titlesize'] = 20     # 标题字体
plt.rcParams['axes.labelsize'] = 18     # 坐标轴标签字体
plt.rcParams['xtick.labelsize'] = 15    # x轴刻度字体
plt.rcParams['ytick.labelsize'] = 15    # y轴刻度字体
plt.rcParams['legend.fontsize'] = 20    # 图例字体

plot_linewidth = 2.0

PLOT_CONFIGS = {
    'leg_length': {
        'title': 'Target and Actual Leg Length',
        'ylabel': 'Leg Length / m',
        'ylim': (0.05, 0.4),
        'items': [
            ('left_leg_length_target', 'Left Target Length', '--'),
            ('right_leg_length_target', 'Right Target Length', '--'),
            ('left_leg_length_actual', 'Left Actual Length', '-'),
            ('right_leg_length_actual', 'Right Actual Length', '-'),
        ],
    },

    'leg_phi0': {
        'title': 'Left and Right Leg Swing Angle',
        'ylabel': 'Swing Angle / deg',
        'ylim': (40, 140),
        # 'ylim': (60, 120),
        'items': [
            ('left_leg_phi0', 'Left Leg Swing Angle', '-'),
            ('right_leg_phi0', 'Right Leg Swing Angle', '-'),
        ],
    },

    'pitch': {
        'title': 'Body Pitch Angle',
        'ylabel': 'Pitch / deg',
        'ylim': (-30, 30),
        # 'ylim': (-10, 10),
        'items': [
            ('pitch', 'Pitch', '-'),
        ],
    },

    'roll': {
        'title': 'Body Roll Angle',
        'ylabel': 'Roll / deg',
        'ylim': (-10, 10),
        'items': [
            ('roll', 'Roll', '-'),
        ],
    },

    'linear_velocity': {
        'title': 'Target and Actual Linear Velocity',
        'ylabel': 'Linear Velocity / (m/s)',
        'ylim': (-1.5, 1.5),
        'items': [
            ('target_linear_vel', 'Target Linear Velocity', '--'),
            ('actual_linear_vel', 'Actual Linear Velocity', '-'),
        ],
    },

    'angular_velocity': {
        'title': 'Target and Actual Angular Velocity',
        'ylabel': 'Angular Velocity / (rad/s)',
        'ylim': (-7.0, 7.0),
        'items': [
            ('target_angular_vel', 'Target Angular Velocity', '--'),
            ('actual_angular_vel', 'Actual Angular Velocity', '-'),
        ],
    },

    'torque': {
        'title': 'Torque Output',
        'ylabel': 'Torque / (N·m)',
        'ylim': (-11, 11),
        'items': [
            ('left_front_joint_torque', 'Left Front Joint Torque', '-'),
            ('left_rear_joint_torque', 'Left Rear Joint Torque', '-'),
            ('right_front_joint_torque', 'Right Front Joint Torque', '-'),
            ('right_rear_joint_torque', 'Right Rear Joint Torque', '-'),
            ('left_wheel_torque', 'Left Wheel Torque', '--'),
            ('right_wheel_torque', 'Right Wheel Torque', '--'),
        ],
    },
}


def get_latest_csv(log_dir):
    # 递归搜索 logs 目录及其子文件夹中的 CSV 文件
    csv_files = glob.glob(os.path.join(log_dir, '**', '*.csv'), recursive=True)

    if len(csv_files) == 0:
        raise FileNotFoundError(f'日志文件夹及其子文件夹中没有找到 CSV 文件: {log_dir}')

    return max(csv_files, key=os.path.getmtime)


def resolve_csv_path(csv_path):
    if csv_path is None:
        csv_path = get_latest_csv(DEFAULT_LOG_DIR)

    csv_path = os.path.abspath(csv_path)

    if not os.path.exists(csv_path):
        raise FileNotFoundError(f'CSV 文件不存在: {csv_path}')

    return csv_path


def check_columns(data, required_columns):
    for col in required_columns:
        if col not in data.columns:
            raise ValueError(f'CSV 文件中缺少列: {col}')


def select_time_range(data, start_time=None, end_time=None):
    raw_time = data['time'].to_numpy(dtype=float)
    rel_time = raw_time - raw_time[0]

    if start_time is None:
        start_time = rel_time[0]

    if end_time is None:
        end_time = rel_time[-1]

    if end_time < start_time:
        raise ValueError(
            f'结束时间不能小于起始时间: start={start_time}, end={end_time}'
        )

    mask = (rel_time >= start_time) & (rel_time <= end_time)

    if mask.sum() == 0:
        raise ValueError(
            f'指定时间范围内没有数据: start={start_time}, end={end_time}'
        )

    selected_data = data.loc[mask].reset_index(drop=True)

    # 截取后，绘图时间轴重新从 0 开始
    selected_time = rel_time[mask]
    selected_time = selected_time - selected_time[0]

    return selected_data, selected_time


def format_time_value(value):
    text = f'{value:.3f}'
    text = text.rstrip('0').rstrip('.')
    return text


def make_time_suffix(start_time=None, end_time=None):
    if start_time is None and end_time is None:
        return ''

    if start_time is not None and end_time is not None:
        return f'_from_{format_time_value(start_time)}s_to_{format_time_value(end_time)}s'

    if start_time is not None:
        return f'_from_{format_time_value(start_time)}s'

    return f'_to_{format_time_value(end_time)}s'


def get_save_path(csv_path, plot_name, start_time=None, end_time=None):
    save_dir = os.path.dirname(csv_path)
    csv_name = os.path.splitext(os.path.basename(csv_path))[0]
    time_suffix = make_time_suffix(start_time, end_time)

    file_name = f'{csv_name}_{plot_name}_curve{time_suffix}.svg'
    return os.path.join(save_dir, file_name)


def plot_one_figure(data, time, csv_path, plot_name, start_time=None, end_time=None):
    config = PLOT_CONFIGS[plot_name]

    required_columns = ['time'] + [item[0] for item in config['items']]
    check_columns(data, required_columns)

    plt.figure(figsize=(12, 7))

    for column_name, label, linestyle in config['items']:
        y = data[column_name].to_numpy(dtype=float)
        plt.plot(time, y, label=label, linestyle=linestyle, linewidth=plot_linewidth)

    plt.xlabel('Time / s')
    plt.ylabel(config['ylabel'])
    plt.title(config['title'])

    # 固定横坐标起点为 0
    if len(time) > 1:
        plt.xlim(0, time[-1])
    else:
        plt.xlim(0, 1)
    # 固定纵坐标范围
    plt.ylim(config['ylim'])

    plt.grid(True)
    plt.legend(loc='upper right', framealpha=0.8)
    plt.tight_layout()

    save_path = get_save_path(csv_path, plot_name, start_time, end_time)
    plt.savefig(save_path, format='svg')

    print(f'Figure saved to: {save_path}')


def parse_common_args():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        'csv_path',
        type=str,
        nargs='?',
        default=None,
        help='CSV file path. If not specified, the latest CSV in logs folder will be used.'
    )

    parser.add_argument(
        '--start',
        type=float,
        default=None,
        help='Start time in seconds, relative to the beginning of the CSV data.'
    )

    parser.add_argument(
        '--end',
        type=float,
        default=None,
        help='End time in seconds, relative to the beginning of the CSV data.'
    )

    parser.add_argument(
        '--no-show',
        action='store_true',
        help='Only save SVG files without showing figures.'
    )

    return parser.parse_args()


def run_single_plot(plot_name):
    args = parse_common_args()

    csv_path = resolve_csv_path(args.csv_path)

    data = pd.read_csv(csv_path)
    check_columns(data, ['time'])

    selected_data, selected_time = select_time_range(
        data,
        start_time=args.start,
        end_time=args.end
    )

    print(f'Read CSV: {csv_path}')

    if args.start is not None or args.end is not None:
        print(f'Time range: start={args.start}, end={args.end}')

    plot_one_figure(
        selected_data,
        selected_time,
        csv_path,
        plot_name,
        start_time=args.start,
        end_time=args.end
    )

    if args.no_show:
        plt.close('all')
    else:
        plt.show()


def run_all_plots():
    args = parse_common_args()

    csv_path = resolve_csv_path(args.csv_path)

    data = pd.read_csv(csv_path)
    check_columns(data, ['time'])

    selected_data, selected_time = select_time_range(
        data,
        start_time=args.start,
        end_time=args.end
    )

    print(f'Read CSV: {csv_path}')

    if args.start is not None or args.end is not None:
        print(f'Time range: start={args.start}, end={args.end}')

    for plot_name in PLOT_CONFIGS.keys():
        plot_one_figure(
            selected_data,
            selected_time,
            csv_path,
            plot_name,
            start_time=args.start,
            end_time=args.end
        )

    if args.no_show:
        plt.close('all')
    else:
        plt.show()