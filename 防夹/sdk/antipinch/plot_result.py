"""
座椅防夹检测算法 — 结果绘图脚本
===============================
读取输入 CSV + runner 输出 CSV，绘制时序图

用法:
    python plot_result.py
    python plot_result.py --input ../data/seat_data_13.5V.csv --output eval_output.csv
"""

from pathlib import Path
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# --- 路径默认 ---
DEFAULT_INPUT = "../data/test.csv"
DEFAULT_OUTPUT = "eval_output.csv"


def load_csv(path, **kwargs):
    """用 numpy 读取 CSV"""
    data = np.genfromtxt(str(path), delimiter=',', skip_header=1, **kwargs)
    with open(str(path), 'r', encoding='utf-8') as f:
        header = f.readline().strip().split(',')
    return {col: data[:, i] for i, col in enumerate(header)}


def plot_result(input_csv, output_csv):
    """绘图"""
    outputs = load_csv(output_csv)

    t = outputs['time']
    mot_curr = outputs['mot_curr']
    hall_width = outputs['hall_width']
    axis_posn = outputs['axis_posn']
    oper_st = outputs['oper_st']
    batt_vol = outputs['batt_vol']
    antipinch_st = outputs['antipinch_st']

    # 防夹参考时刻 (pos=1, motor=forward)
    ref_ts = None
    first_flag_ts = None
    for i in range(len(t)):
        if oper_st[i] == 1 and axis_posn[i] == 1 and ref_ts is None:
            ref_ts = t[i]
        if antipinch_st[i] == 1 and first_flag_ts is None:
            first_flag_ts = t[i]

    # 中文字体
    plt.rcParams['font.sans-serif'] = ['SimHei',
                                       'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    fig = plt.figure(figsize=(16, 12))
    gs = gridspec.GridSpec(5, 1, hspace=0.35, figure=fig)

    def add_vlines(ax):
        if ref_ts is not None:
            ax.axvline(ref_ts, color='orange', linestyle='--',
                       linewidth=1, alpha=0.7, label='防夹参考(pos=1)')
        if first_flag_ts is not None:
            ax.axvline(first_flag_ts, color='red', linestyle='--',
                       linewidth=1, alpha=0.7, label='首次检出')

    # ① 电机电流
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, mot_curr, color='#2196F3', linewidth=0.8, label='MotCurr (原始)')
    ax1.set_ylabel('电机电流')
    ax1.set_xlim(t[0], t[-1])
    ax1.grid(True, alpha=0.3)
    add_vlines(ax1)
    ax1.legend(loc='upper right', fontsize=8)

    # ② 霍尔脉宽 + 位置
    ax2 = fig.add_subplot(gs[1])
    ax2.plot(t, hall_width, color='#4CAF50',
             linewidth=0.8, label='HallWidth (脉宽)')
    ax2b = ax2.twinx()
    ax2b.plot(t, axis_posn, color='#9C27B0', linewidth=0.8,
              label='AxisPosn (位置)', alpha=0.6)
    ax2b.set_ylabel('位置 (0~100)')
    ax2.set_ylabel('脉宽')
    ax2.legend(loc='upper left', fontsize=8)
    ax2b.legend(loc='upper right', fontsize=8)
    ax2.set_xlim(t[0], t[-1])
    ax2.grid(True, alpha=0.3)
    add_vlines(ax2)

    # ③ 电机运行状态
    ax3 = fig.add_subplot(gs[2])
    ax3.plot(t, oper_st, color='#607D8B',
             linewidth=0.8, drawstyle='steps-post')
    ax3.set_yticks([0, 1, 2])
    ax3.set_yticklabels(['停止', '正转', '反转'])
    ax3.set_ylabel('电机状态')
    ax3.set_xlim(t[0], t[-1])
    ax3.grid(True, alpha=0.3)
    add_vlines(ax3)

    # ④ 供电电压
    ax4 = fig.add_subplot(gs[3])
    ax4.plot(t, batt_vol, color='#FF9800',
             linewidth=0.8, label='BattVol (ADC)')
    ax4.set_ylabel('电池电压 (ADC)')
    ax4.set_xlim(t[0], t[-1])
    ax4.grid(True, alpha=0.3)
    add_vlines(ax4)
    ax4.legend(loc='upper right', fontsize=8)

    # ⑤ 防夹输出
    ax5 = fig.add_subplot(gs[4])
    ax5.fill_between(t, antipinch_st, color='#F44336', alpha=0.3, step='post')
    ax5.plot(t, antipinch_st, color='#F44336', linewidth=1.2,
             drawstyle='steps-post', label='AntiPinchSt')
    ax5.set_yticks([0, 1])
    ax5.set_yticklabels(['Normal', 'Occurred'])
    ax5.set_ylabel('防夹输出')
    ax5.set_xlabel('时间 (s)')
    ax5.legend(loc='upper right', fontsize=8)
    ax5.set_xlim(t[0], t[-1])
    ax5.grid(True, alpha=0.3)
    add_vlines(ax5)

    # 标题
    resp_ms = (first_flag_ts - ref_ts) * \
        1000 if ref_ts and first_flag_ts else None
    title = f'座椅防夹检测算法 — 全信号时序图 [{Path(input_csv).stem}]'
    if resp_ms is not None:
        title += f'\n防夹参考(pos=1): {ref_ts:.3f}s  检出: {first_flag_ts:.3f}s  响应: {resp_ms:.0f}ms'
    fig.suptitle(title, fontsize=14, fontweight='bold')

    png_path = Path(output_csv).parent / f'防夹检测_{Path(input_csv).stem}.png'
    plt.savefig(str(png_path), dpi=150, bbox_inches='tight')
    print(f"Plot saved: {png_path}")
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Plot anti-pinch detection results')
    parser.add_argument('--input', default=DEFAULT_INPUT,
                        help='Input CSV path')
    parser.add_argument('--output', default=DEFAULT_OUTPUT,
                        help='Output CSV path from runner')
    args = parser.parse_args()

    if not Path(args.output).exists():
        print(
            f"Error: {args.output} not found. Run 'seat_runner.exe --output {args.output}' first.")
        exit(1)

    plot_result(args.input, args.output)
