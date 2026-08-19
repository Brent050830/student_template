# SDK 使用示例手册


> **说明**：本手册描述的是 **示例 SDK（座椅防夹检测）**，用于帮助参赛者理解
> SDK 的接口风格、构建方式和评测机制，对应目录 `sdk/antipinch/`。
>
> **正式赛题为"基于导航信息的电池充电温度预调"**，工作目录为
> `sdk/student_template/`，完整赛题说明请见
> [`student_template/2026USP创新训练营参赛开发指南.md`](./student_template/2026USP创新训练营参赛开发指南.md)。
> 本手册中的算法细节（§3、§4）仅适用于座椅防夹示例

---

## 1. 环境搭建

> **终端选择**：推荐使用 **cmd**（或 Windows Terminal 的 cmd 标签页）。
> 若必须用 PowerShell，建议升级到 PowerShell 7（`pwsh`）以获得可靠的 UTF-8 支持（PS 5.1 对中文文件名可能乱码）。

### 1.1 安装 MinGW-w64（Windows）

1. 下载指定版本：
   https://static-1314150920.cos.ap-shanghai.myqcloud.com/download/MinGW/x86_64-13.1.0-release-posix-seh-ucrt-rt_v11-rev1.7z
2. 解压到指定目录（ `C:\mingw64`）
3. 将 `C:\mingw64\bin\` 目录添加到系统 PATH
4. 验证安装：

```cmd
gcc --version
```

你应该能看到类似 `gcc.exe (x86_64-posix-seh-rev0, ...)` 的输出。

### 1.2 配置 `make`

MinGW 自带的 `make` 工具实际文件名为 `mingw32-make.exe`。为了能在命令行中直接使用 `make`，需要将其重命名：

1. 进入 `mingw64\bin\` 目录
2. 将 `mingw32-make.exe` 复制一份，重命名为 `make.exe`

验证：

```cmd
make --version
```

如果显示 `GNU Make x.x` 版本信息，则配置成功。

### 1.3 SDK 包目录结构

解压 SDK 包后，你的目录应该如下所示：

```
sdk/
├── include/
│   ├── seat_sdk.h              <-- 座椅防夹的头文件
│   └── usp_api.h               <-- 赛题"基于导航信息的电池充电温度预调"API 声明
├── data/
│   ├── test.csv                <-- 座椅防夹测试数据集（`make run` 默认）
│   └── validate.csv            <-- 座椅防夹验证数据集
├── lib/
├── student_template/               <-- "基于导航信息的电池充电温度预调"的工作目录
│   ├── student_solution.c          <-- ★★★ 唯一需要编辑的文件 ★★★
│   ├── Makefile                    <-- 构建脚本（请勿修改）
│   ├── build_student.bat           <-- Windows 一键构建脚本
└── antipinch/                      <-- 座椅防夹参考实现示例（可阅读，勿提交）
    ├── student_solution.c          <-- 空模板
    ├── student_solution_ref.c      <-- 参考实现
    ├── Makefile
    ├── build.bat
    ├── plot_result.py              <-- 绘图脚本（可选，需 Python + matplotlib）
    └── 座椅防夹考题.md              <-- 考题说明
```

**重要规则**：
- 你只能修改 `antipinch/student_solution.c`
- 不要修改 `seat_sdk.h`、`Makefile` 或 `lib/` 中的任何文件
- 不要添加新的 `.c` 文件（所有代码都放在 `student_solution.c` 中）

---

## 2. 快速开始

### 2.1 在 `antipinch/` 中打开终端

```cmd
cd sdk\antipinch
```

### 2.2 实现算法

打开 `student_solution.c` 并实现两个函数：

```c
void anti_pinch_detector_init(void);
void anti_pinch_detector_step(void);
```

### 2.3 构建和运行

推荐使用"antipinch/build.bat", 依次完成以下步骤；

```cmd
# 使用公开数据集运行数据评估
make run

# 运行评估并生成结果时序图（需要 Python + matplotlib）
make plot
```

### 2.4 切换数据集

数据集位于 `../data/`（如 `test.csv`、`validate.csv`）。切换数据集有以下方式：

**方式一：build.bat 传参（推荐）**
```bat
build.bat ..\data\validate.csv     REM 指定数据集
build.bat                          REM 不带参数 = 默认 test.csv
```

**方式二：直接调用 runner 与绘图脚本**
```bat
seat_runner.exe --data ..\data\validate.csv --output eval_output.csv
python plot_result.py --input ..\data\validate.csv --output eval_output.csv
```

> 切换数据集后，PNG 会按数据集命名（如 `防夹检测_validate.png`、`防夹检测_test.png`），互不覆盖。

### 2.5 预期输出

`make run` / `make plot` 会输出加载与处理的样本数，并生成 `eval_output.csv` 与 `防夹检测_<数据集名>.png` 信号图。算法效果通过查看该 PNG 判断。

---

## 3. API 参考

### 3.1 输入服务接口（Getter）

在 `anti_pinch_detector_step()` 中调用这些函数来读取传感器数据：

```c
MotCurr_u16    mot_curr;
HallPosn_u16   hall_posn;
HallPlsWidth_u32 hall_width;
OperMotSt_u8   oper_st;
MotPwrVolt_u16 batt_vol;
Posn_u8        axis_posn;

SeatBackRclnMotDD_u16MotCur(&mot_curr);
BO_Atm_SeatBackRcln_ntfHallPosn(&hall_posn);
SeatBackRclnHallDD_u32CurrHallPlsWidth(&hall_width);
BO_Atm_SeatBackRcln_ntfOperSt(&oper_st);
SeatBackRclnMotDD_u16MotPwrVolt(&batt_vol);
BO_Atm_SeatBackRcln_ntfPosn(&axis_posn);
```

| 信号 | 类型 | 范围 | 说明 |
|------|------|------|------|
| 电机电流 | `MotCurr_u16` | `uint16` 0~10636 | 电机电流 |
| 霍尔位置 | `HallPosn_u16` | `uint16` 0~65535 | 霍尔传感器位置 |
| 霍尔脉冲宽度 | `HallPlsWidth_u32` | `uint32` | 霍尔脉冲宽度 |
| 电机状态 | `OperMotSt_u8` | `uint8` 0/1/2 | 0=停止, 1=正转, 2=反转 |
| 供电电压 | `MotPwrVolt_u16` | `uint16` | 电池电压 ADC 值 |
| 靠背角度 | `Posn_u8` | `uint8` 0~100, 0xFF=无效 | 位置（已映射：Excel-1） |

### 3.2 输出服务接口（Setter）

调用此函数输出防夹检测结果：

```c
AntiPinchSt_u8 result = ANTIPINCHST_NORMAL;  /* 或 ANTIPINCHST_OCCURRED */
BO_Atm_SeatBackRcln_ntfAntiPinchSt(result);
```

| 值 | 含义 |
|----|------|
| `ANTIPINCHST_NORMAL` (0) | 未检测到夹伤 |
| `ANTIPINCHST_OCCURRED` (1) | 检测到夹伤事件 |

### 3.3 可调参数

这些宏定义在 `seat_sdk.h` 中。你可以使用它们，但不应修改：

```c
#define V_REF               13.5f
#define V_ADC_FACTOR        (13.5f / 13650.0f)
#define PINCH_ZONE_MIN   1
#define PINCH_ZONE_MAX   99
#define BASELINE_WINDOW     200
#define BASELINE_MIN_FILL   50
#define CURR_SIGMA_FACTOR   3
#define HW_SIGMA_FACTOR     3
#define CURR_DEV_MIN        800.0f
#define HW_DEV_MIN          800.0f
#define STARTUP_DELAY_MS    500
#define STARTUP_DELAY_SAMPLES  (STARTUP_DELAY_MS / 2)  /* = 250 */
#define MIN_DURATION_MS     20
#define MIN_DURATION_SAMPLES   (MIN_DURATION_MS / 2)   /* = 10 */
#define SAMPLE_PERIOD_MS    2
```

---

## 4. 算法指导

### 4.1 核心概念

算法每 2ms（500Hz）通过 `anti_pinch_detector_step()` 调用一次。你必须：

1. 通过 getter 接口读取全部 6 个输入信号
2. 处理信号以检测夹伤事件
3. 通过 setter 接口输出结果

### 4.2 关键设计要点

| 机制 | 为什么重要 | 缺少的后果 |
|------|-----------|-----------|
| 电压归一化 | 电池电压在 11V~15V 之间变化 | 低电压导致漏检 |
| 方向感知基线 | 正转/反转的电流/脉冲模式不同 | 方向切换导致误触发 |
| 启动延时 | 电机启动电流不稳定 | 启动时必定误触发 |
| 持续时间滤波 / CUSUM | 消除瞬态噪声 | 单脉冲误触发 |
| 双信号融合 | 单靠电流不可靠 | 单信号误触发或漏检 |
| 夹伤时基线冻结 | 防止异常值污染基线 | 基线漂移，导致漏检 |

### 4.3 状态持久化

由于 `anti_pinch_detector_init()` 和 `anti_pinch_detector_step()` 没有参数，你必须在 `student_solution.c` 中使用 `static` 变量来在调用之间保持状态：

```c
static struct {
    float curr_baseline;
    float hw_baseline;
    float curr_sigma;
    float hw_sigma;
    /* ... 其他状态变量 ... */
} g_det;
```

### 4.4 位置映射（已完成）

SDK 已经将 Excel 位置映射为接口值：
- Excel 0 -> `0xFF`（无效）
- Excel 1 -> `0`（0%）
- Excel 101 -> `100`（100%）

你**不需要**做任何位置映射。直接使用 `axis_posn`。

### 4.5 夹伤检测区域

使用 `seat_sdk.h` 中的宏：
- 检测区域（正转/反转统一）：`PINCH_ZONE_MIN` (1) 到 `PINCH_ZONE_MAX` (99)

即检测区域为 **1%~99%**：行程两端（位置 0 与 100，启动涌流区与机械限位堵转区）不参与防夹检测。

**重要**：位置 `0xFF` 表示无效。使用前务必检查。

---

## 5. 提交

只提交你的 `student_solution.c` 文件。

提交前验证：
```bat
make plot DATA_CSV=../data/test.csv       REM 生成 防夹检测_test.png
make plot DATA_CSV=../data/validate.csv   REM 生成 防夹检测_validate.png
```

---

## 6. 故障排除

### "undefined reference to anti_pinch_detector_init"
你忘记在 `student_solution.c` 中实现 `anti_pinch_detector_init()`。

### "undefined reference to anti_pinch_detector_step"
你忘记在 `student_solution.c` 中实现 `anti_pinch_detector_step()`。

### 正常区域误触发
- 检查仅在电流和霍尔宽度都异常时才触发
- 检查位置是否在夹伤区域内
- 检查启动延时是否正确实现

### 夹伤时无检测
- 检查电压归一化（低电压应产生更高的归一化电流）
- 检查基线是否正确更新
- 检查持续时间滤波 / CUSUM 是否允许足够时间（需 >=20ms）
- 检查夹伤区域边界是否正确

### 所有输出都是 0
确保你在**每次** `step()` 调用中都调用了 `BO_Atm_SeatBackRcln_ntfAntiPinchSt()`。

---

*祝你好运！专注于算法；SDK 会处理其他一切。*
