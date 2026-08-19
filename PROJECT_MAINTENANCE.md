# 项目维护文档

## 选题/项目目标

本工作区包含两个 SDK 示例/任务：

- `sdk/student_template/`：基于导航信息的电池充电温度预调算法。目标是在车辆到达充电站前决定最优预热开启距离，并输出到达充电站时的电池温度、SOC、预热能耗和预计充电时间。
- `sdk/antipinch/`：座椅防夹检测示例。目标是根据电机电流、霍尔脉宽、位置、电机状态、电压等信号判断是否发生防夹。

当前重点维护对象是 `sdk/student_template/student_solution.c` 中的电池预热寻优算法。以后每次修改算法代码后，应同步更新本文档中的建模、搜索策略、运行方式、验证结果和风险说明。

## 程序整体框架

电池预热程序的主流程集中在 `sdk/student_template/student_solution.c`：

1. 通过 `usp_api.h` 中的接口读取电池初始状态和导航功率预测。
2. 将路线预测保存为内部 `NavSeg` 数组。
3. 使用物理模型仿真不同预热开启距离下的到达状态。
4. 使用搜索算法选择综合代价最低的预热开启距离。
5. 通过 `BattChrgPreHeatg_ntf*()` 接口通知系统结果。
6. 在本地 runner 中打印约束检查结果。

接口边界：

- 输入接口定义在 `sdk/student_template/usp_api.h`。
- 算法实现位于 `sdk/student_template/student_solution.c` 的 `Your algorithm code starts here` 到 `Your algorithm code ends here` 区域。
- 主函数只负责读取输入、调用算法、通知结果和打印验证信息。

## 主要模块职责

### `load_nav_segments()`

调用 `VehPwrPred_getPwrPred()` 读取路线预测。每段路线包含：

- `length`：路段距离，单位 km。
- `avgSpd`：平均车速，单位 km/h。
- `drvPwr`：预测驱动功率，单位 kW。
- `ambTemp`：环境温度，单位摄氏度。

### `interp_map()` 与三个 MAP 语义包装函数

对二维 MAP 做双线性插值。当前用于：

- 电池内阻 `R_INT_MAP(T, SOC)`。
- 开路电压 `UOC_MAP(T, SOC)`。
- 快充功率上限 `P_CHARGE_MAP(T, SOC)`。

三个二维表的自变量都是温度 `T` 和 SOC，只是行轴温度点、列轴 SOC 点以及表格物理含义不同：

| MAP | 输入 | 输出 | 用途 |
|---|---|---|---|
| `R_INT_MAP` | 电池温度、SOC | 内阻，单位 Ohm | 估算行驶/预热时的电池电流与内阻发热 |
| `UOC_MAP` | 电池温度、SOC | 开路电压，单位 V | 估算等效电路电流 |
| `P_CHARGE_MAP` | 电池温度、SOC | 充电功率上限，单位 kW | 估算到站后快充到 80% SOC 的时间 |

当前插值方法是双线性插值，步骤如下：

1. 先把输入温度和 SOC 截断到 MAP 轴范围内，避免越界查表。
2. 在温度轴上找到相邻两个节点 `temp_axis[ti]` 和 `temp_axis[ti + 1]`。
3. 在 SOC 轴上找到相邻两个节点 `soc_axis[si]` 和 `soc_axis[si + 1]`。
4. 计算温度方向比例 `tf` 和 SOC 方向比例 `sf`。
5. 先沿 SOC 方向分别插出下温度行 `lower` 和上温度行 `upper`。
6. 再沿温度方向在 `lower` 和 `upper` 之间插值得到最终结果。

当前仅在通用插值外增加三个薄包装函数：

```c
battery_resistance(T, SOC)
open_circuit_voltage(T, SOC)
charge_power(T, SOC)
```

包装函数不增加插值点、不修改表格，也不进行新的函数拟合；内部 SOC 仍使用 `0~1` 比例，调用通用插值前转换为百分数。

核心代码格式：

```c
const float temp = fminf(temp_axis[6], fmaxf(temp_axis[0], temp_C));
const float soc = fminf(soc_axis[7], fmaxf(soc_axis[0], soc_pct));

while (ti < 5 && temp > temp_axis[ti + 1]) ++ti;
while (si < 6 && soc > soc_axis[si + 1]) ++si;

const float tf = (temp - temp_axis[ti]) /
                 (temp_axis[ti + 1] - temp_axis[ti]);
const float sf = (soc - soc_axis[si]) /
                 (soc_axis[si + 1] - soc_axis[si]);

const float lower = map[ti][si] + sf * (map[ti][si + 1] - map[ti][si]);
const float upper = map[ti + 1][si] +
                    sf * (map[ti + 1][si + 1] - map[ti + 1][si]);

return lower + tf * (upper - lower);
```

当前实现每次插值都用 `while` 顺序查找轴区间。由于温度轴只有 7 个点、SOC 轴只有 8 个点，这个成本很小。后续如果要优化，可以考虑：

- 对固定轴长度展开区间判断，减少循环。
- 如果搜索过程中相邻仿真步的温度/SOC 变化很小，可缓存上一次的 `ti/si`。
- 如果改为大量网格搜索，可把常用温度/SOC 区域预采样，换取更少的运行时插值计算。

### `build_route_context()` / `simulate_heating_suffix()`

先分别建立 1 s 和 0.2 s 的无加热路线基线。给定整数启动时间步后，从对应基线状态开始只仿真加热后缀，避免每个候选重复计算启动前完全相同的状态。

0.2 s 模型使用固定全局内部时间步；每个外部 1 s 周期开始时采样一次速度、驱动功率和环境温度，随后 5 个内部子步保持这些导航量不变。

```c
typedef struct {
    float temp_C;
    float soc;
    float heat_kWh;
} TimeRouteResult;
```

### `estimate_charge_time()`

根据到达充电站时的温度和 SOC，使用快充功率 MAP 估计从当前 SOC 充到 `SOC_TARGET = 80%` 所需时间。

### `optimize_preheat()` / `find_optimal_start_distance()`

`optimize_preheat()` 是当前完整寻优入口，同时返回最佳结果和四个标签极值。模板主函数继续通过 `find_optimal_start_distance()` 薄包装获得：

- `T_end_opt`
- `SOC_end_opt`
- `E_heat_opt`
- `chrg_time_s`

## 核心物理建模

### 热模型

电池包温度更新基于 lumped thermal model：

```text
dT/dt = (Q_gen + Q_heat - Q_loss) / (M_BAT * CP)
```

代码形式：

```c
result.temp_C += (generated_heat +
                  (heating ? ETA_HEAT * P_HEAT_MAX : 0.0f) - heat_loss) *
                 dt / (M_BAT * CP);
```

其中：

- `M_BAT = 400 kg`：电池包质量。
- `CP = 1000 J/(kg*C)`：比热容。
- `generated_heat = I^2 * R`：电池内阻发热。
- `ETA_HEAT * P_HEAT_MAX`：PTC 有效加热功率。
- `heat_loss = (H0 + KV * speed) * (T_bat - T_env)`：与环境的散热，车速越高散热越强。

### 电模型

行驶和预热总功率：

```c
const float power_w = g_segs[seg].P_drive_kW * 1000.0f +
                      (heating ? P_HEAT_MAX : 0.0f);
```

根据等效电路：

```text
P = U_oc * I - I^2 * R
```

解二次方程得到电流：

```c
const float current = (fabsf(resistance) > 1.0e-8f)
                    ? (uoc - sqrtf(discriminant)) / (2.0f * resistance)
                    : power_w / fmaxf(uoc, 1.0f);
```

SOC 更新：

```c
result.soc -= current * dt / (C_NOM_AH * 3600.0f);
```

### 充电时间模型

充电时间按 PDF 3.4 的恒功率近似口径计算。先用到站终点状态查表：

```c
P_cha = f(SOC_end, T_end);
```

然后认为从 `SOC_end` 充到 `SOC_TARGET = 80%` 的过程都使用该终点充电功率：

```c
const float charge_kw = fmaxf(0.1f,
    interp_map(P_CHARGE_TEMPS, P_CHARGE_SOCS, P_CHARGE_MAP,
               temp_C, start_soc * 100.0f));

return (SOC_TARGET - start_soc) * C_NOM_AH * U_NOM * 3.6f / charge_kw;
```

这里 `C_NOM_AH * U_NOM * 3.6` 用于把 SOC 差值、电池标称能量和 kW 功率换算成秒。旧版本曾采用“充电过程每秒更新 SOC 并重新查表”的积分口径；当前版本为了贴近 PDF 原公式，改为终点功率恒定近似。

## 使用的搜索算法

当前算法采用“时间边界确定可行区间 + 可行区间内分层搜索”。控制量的主键不再是浮点距离，而是从路线起点开始计数的 0.2 s 启动时间步 `start_step`；最终再把启动时刻转换成到充电桩的剩余距离。

### 1. 建立时间与距离映射

导航段给出距离和平均速度，因此每段持续时间为：

```text
segment_time_s = 3600 * segment_distance_km / speed_kmh
```

`remaining_distance_at_time()` 使用各段的精确时间和距离计算启动时刻对应的剩余距离。候选通过整数 `start_step` 去重，路线起点使用 `start_step = 0`，路线终点使用最终步编号；因此真实总里程不会再和邻近浮点网格点合并。

### 2. 两级积分模型

- `FAST_DT_S = 1.0 s`：只用于快速定位三个边界和粗搜初筛。
- `FINAL_DT_S = 0.2 s`：用于边界精修、所有正式候选完整仿真和最终评分。
- `OUTER_UPDATE_S = 1.0 s`：外部导航量更新周期。每个周期中的 5 个 0.2 s 子步保持速度、驱动功率和环境温度不变。

1 s 结果不会进入最终候选池，因而不会和 0.2 s 结果共同计算同一组 score。

### 3. 确定可行时间边界

先用 1 s 模型二分定位，再以该结果为初值，在 0.2 s 整数步上扩展括号并精确二分：

```text
t20  = 到站温度达到 20 C 的最短加热时间
t25  = 到站温度不超过 25 C 的最长加热时间
tSOC = 到站 SOC 不低于 10% 的最长加热时间

t_min = t20
t_max = min(t25, tSOC, route_total_time)
```

若 1 s 模型在极窄区间上没有找到边界，它只能失去“初值”作用，不能否决工况；算法会从对应路线端点继续做 0.2 s 精确搜索。

在二分前，算法还会用 60 s 间隔的 1 s 模型探针检查 `T20`、`T25` 和 `SOC` 条件是否发生趋势反转。若发现非单调，二分结果不再作为正式边界，而是仅对该异常工况枚举全部 0.2 s 启动步，直接从硬约束可行候选中确定时间和能耗端点。

最终边界会再次经过全部硬约束检查。若精确启动步区间为空，即 `t_min > t_max`，工况无可行解。

### 4. 直接计算能耗边界

能耗边界不再来自候选池 min/max：

```c
min_heat_kWh = (P_HEAT_MAX / 1000.0f) * t_min / 3600.0f;
max_heat_kWh = (P_HEAT_MAX / 1000.0f) * t_max / 3600.0f;
```

当前 `P_HEAT_MAX = 6000 W`，因此与 `6.0f * time_s / 3600.0f` 完全等价。两个能耗边界都对应通过全部硬约束的 0.2 s 候选。

### 5. 可行区间内分层搜索

搜索只覆盖精确可行启动步区间：

| 层级 | 积分模型 | 启动时间采样 | 下一层搜索范围 |
|---|---:|---:|---:|
| 粗搜 | 1 s 初筛、0.2 s 正式锚点 | 60 s | 候选附近约 40 s |
| 细搜 | 0.2 s | 5 s | 候选附近 5 s |
| 精搜 | 0.2 s | 0.2 s | 不再细化 |

1 s 粗搜结果只决定精确锚点的加入顺序。全部粗搜锚点随后都用 0.2 s 模型重算并进入统一候选池，因此最初未入选的粗点仍可在后续重评分后晋级。粗搜候选必须先生成 5 s 细搜候选；只有 `TIME_LEVEL_FINE` 候选可以继续生成 0.2 s 精搜候选，不能跳级。

### 6. 动态重评分和精筛条件

每展开一个粗搜或细搜中心后，都会基于全部已知且通过硬约束的 0.2 s 候选重新计算：

```text
min_charge_s
max_charge_s
全部候选 score
全部保护标志
```

以下候选会继续精筛：

- 当前最佳 score 附近候选，阈值为 `best_score + 0.01`。
- 最小充电时间候选。
- 最大充电时间候选。
- 充电时间局部最大值和局部最小值。
- 能耗—充电时间 Pareto 非支配前沿的 3 个均匀代表候选；端点、全局极值和局部极值由其他标志独立保护。
- score 局部最小值。
- 可行区间左右端点。

候选按真实整数启动步排序后判断局部极值。Pareto 前沿从最少能耗端反向扫描充电时间下界得到，不依赖候选插入顺序；平滑权衡段上可能几乎每个点都非支配，因此只均匀保留 3 个前沿代表继续展开，避免搜索退化成全可行区间穷举。

### 7. 评分规则

能耗边界来自精确时间边界，充电时间边界来自全部已评估的可行 0.2 s 候选：

```c
energy_cost = (E - min_heat) / (max_heat - min_heat);
charge_cost = (charge_s - min_charge) / (max_charge - min_charge);
score = 0.4f * energy_cost + 0.6f * charge_cost;
```

任一分母不大于 `1.0e-6f` 时，对应归一化项取 0，避免产生 NaN。未通过 `20 C <= T_end <= 25 C` 或 `SOC_end >= 10%` 的候选不参与任何边界统计、score 或最佳结果选择。

### 8. 无可行解

`optimize_preheat()` 默认把结果初始化为 NaN。边界不存在、可行区间为空或内存分配失败时，不使用 fallback 填充正式结果。标签生成器输出：

```text
best_start_distance_km = nan
min_charge_s = nan
max_charge_s = nan
min_heat_kWh = nan
max_heat_kWh = nan
```

### 当前特点和风险

优点：

- 先裁剪可行时间区间，避免在全路线距离上盲目搜索。
- 0.2 s 整数步键从结构上消除浮点距离去重问题。
- 无加热前缀状态只计算一次，每个候选只仿真加热后缀。
- 每个积分步对应的导航段和实际步长在路线基线中预计算，候选仿真不重复定位导航段。
- 1 s 模型只做加速，不拥有最终否决权；趋势探针发现非单调时改走 0.2 s 全步兜底。
- 粗搜锚点、细搜候选和精搜候选在同一个 0.2 s 候选池中重评分，旧候选可以重新晋级。
- 最终边界、极值、score 和最佳距离统一来自 0.2 s 模型。

仍需关注：

- 常规边界搜索仍利用“加热越久，温度总体上升、SOC 总体下降”的物理单调性；60 s 趋势探针能识别宏观反转并触发精确兜底，但比探针间隔更窄的孤立非单调小岛仍需专门构造数据验证。
- 分层搜索不是对所有正式工况做连续数学穷举；通过固定随机种子的 0.2 s 全时间步穷举进行回归核对。
## 主流程/运行流程

电池预热主函数运行流程：

```text
main()
  -> EMS_HVBatt_getTempAvg()
  -> EMS_HVBatt_getTargetSOC()
  -> EMS_HVBatt_getCurrent()
  -> EMS_HVBatt_getVolt()
  -> load_nav_segments()
  -> find_optimal_start_distance()
       -> optimize_preheat()
            -> build_route_context()
            -> find_feasible_time_bounds()
            -> 1 s coarse screening
            -> 0.2 s fine/final search + refresh_time_flags()
  -> BattChrgPreHeatg_ntfPreHeatgStartDist()
  -> BattChrgPreHeatg_ntfPreHeatgEndTemp()
  -> BattChrgPreHeatg_ntfPreHeatgEndSOC()
  -> BattChrgPreHeatg_ntfPreHeatgEnergy()
  -> print constraint checks
```

通知接口代码格式：

```c
BattChrgPreHeatg_ntfPreHeatgStartDist(start_distance);
BattChrgPreHeatg_ntfPreHeatgEndTemp(T_end_opt);
BattChrgPreHeatg_ntfPreHeatgEndSOC(SOC_end_opt);
BattChrgPreHeatg_ntfPreHeatgEnergy(E_heat_opt);
```

## 运行方式

### 电池预热 `student_template`

目录：

```powershell
cd C:\Users\17871\Desktop\sdk\student_template
```

推荐强制重新编译并运行当前源码：

```powershell
mingw32-make -B
```

也可以使用项目自带批处理脚本：

```powershell
.\build_student.bat
```

当前实测输出：

```text
start_distance = 29.64 km
T_end_opt      = 20.00 C
SOC_end_opt    = 12.20 %
E_heat_opt     = 3.1772 kWh
chrg_time_s    = 1030.53 s
constraint     = ALL CONSTRAINTS SATISFIED
```

### 导航训练集标签生成

`sdk/data/nav_train_100000.txt` 是 100000 条固定总里程 39 km 的导航工况数据。当前批量标签生成器位于：

```text
sdk/student_template/generate_nav_labels.c
```

该工具直接 `include "student_solution.c"` 复用当前预热搜索算法，并提供本地 stub 接口，不修改正式提交入口。CSV 表头、每条标签和关闭文件时都会检查写入错误；磁盘写满时返回失败，不再静默打印 `done`。默认初始电池状态沿用当前 mock 工况：

```text
T_init = 0 C
SOC_init = 65 %
```

编译和生成完整标签：

```powershell
cd C:\Users\17871\Desktop\sdk\student_template
gcc -Wall -Wextra -O2 -std=c11 -I../include -o generate_nav_labels.exe generate_nav_labels.c -lm
.\generate_nav_labels.exe nav_train_100000.txt nav_train_100000_labels.csv
.\generate_nav_labels.exe --verify-random nav_train_100000.txt 100 20260819
```

输出文件：

```text
sdk/student_template/nav_train_100000_labels.csv
```

输出字段：

```text
case_id,best_start_distance_km,min_charge_s,max_charge_s,min_heat_kWh,max_heat_kWh
```

其中 `best_start_distance_km` 来自当前搜索算法在可行候选中的最终选择；四个 min/max 只统计通过硬约束筛选的可行候选，即满足 `T_end_opt in [20, 25] C` 且 `SOC_end_opt >= 10%` 的候选。若某个工况没有任何可行候选，5 个输出字段均为 `nan`，不使用未过硬筛的 fallback 或零能耗候选填充标签。

2026-08-19 全量验证结果：

- 新版 100000 条中，有效标签 `52425`，无效标签 `47575`。
- 相对旧版 `52378/47622`，有 61 条由旧无效变为 0.2 s 可行，有 14 条由旧有效变为 0.2 s 无解；这 75 条变化工况逐条与 0.2 s 全步穷举核对，`mismatches = 0`。
- 固定随机种子 `20260819` 抽取 100 条与 0.2 s 全步穷举核对，`mismatches = 0`。
- 旧版单进程全量 wall time 为 `2656.807 s`；新版四分片全量 wall time 为 `1344.457 s`，四进程 CPU 合计 `5195.062 s`。因此新版实际计算量约为旧版的 1.96 倍，并行后墙钟时间约 22 分 24 秒。
- 完整合并标签已复制到 `sdk/student_template/nav_train_100000_labels.csv`，共 `100001` 行；四个分片均为 `25001` 行且首尾 case_id 连续。D 盘基准副本与项目文件的 SHA-256 均为 `1D4DBA444BF6B0F41DF736A91D1B66EA8DE602C7A7632EFFEECA4E29E61E90B4`。

### 座椅防夹 `antipinch`

目录：

```powershell
cd C:\Users\17871\Desktop\防夹\sdk\antipinch
```

默认运行 `student_solution.c`：

```powershell
mingw32-make.exe plot DATA_CSV=../data/test.csv
mingw32-make.exe plot DATA_CSV=../data/validate.csv
```

运行参考实现可覆盖 Makefile 变量：

```powershell
mingw32-make.exe plot SRC=student_solution_ref.c OBJ=student_solution_ref.o DATA_CSV=../data/test.csv OUTPUT_CSV=ref_outputs/eval_ref_test.csv
mingw32-make.exe plot SRC=student_solution_ref.c OBJ=student_solution_ref.o DATA_CSV=../data/validate.csv OUTPUT_CSV=ref_outputs/eval_ref_validate.csv
```

## 常见运行问题

### `student_template` 找不到 `student_solution.exe`

当前生成的可执行文件名是 `run.exe`，不是 `student_solution.exe`。应运行：

```powershell
.\run.exe
```

### `student_template` 的 Makefile 找不到 `../lib/competition_mock.dll`

新项目结构中依赖 DLL 位于 `sdk/lib/competition_mock.dll`，当前 Makefile 的 `../lib/competition_mock.dll` 路径已经可以正常工作。如果以后移动目录后出现：

```text
No rule to make target '../lib/competition_mock.dll'
```

应先确认当前目录是 `sdk/student_template`，并确认 `..\lib\competition_mock.dll` 存在，再强制重新构建：

```powershell
mingw32-make -B
```

### `antipinch` 图名在终端显示乱码

`plot_result.py` 生成的 PNG 文件名包含中文，例如 `防夹检测_test.png`。Windows 控制台编码可能显示为乱码，但文件本身正常。

### `antipinch` 参考算法验证结果

当前改进后的参考策略为：

```text
电流异常 && (脉宽异常 || 位置停滞)
```

并且异常帧不进入基线缓冲区。

最近一次验证：

- `test.csv`：首次触发 `22.1562s`，共 `197` 帧。
- `validate.csv`：首次触发 `24.5612s`，最后 `24.7012s`，共 `15` 帧。

## 维护约定

每次修改算法代码后，应同步维护本文档：

- 如果物理模型公式、常量、积分步长或 MAP 使用方式变化，更新“核心物理建模”。
- 如果搜索策略、候选数量、代价函数或约束处理变化，更新“使用的搜索算法”。
- 如果构建命令、输出文件名或 runner 依赖变化，更新“运行方式”和“常见运行问题”。
- 如果运行了验证，记录实际命令和关键输出。
- 如果只是实验性修改，还未确认效果，应明确标注“未验证”或“实验中”。

## PR 提交记录

### 2026-08-18 - 新增项目维护文档

- 本次目标：创建唯一维护文档，记录电池预热算法的物理建模、搜索算法、核心代码格式、运行方式和常见问题。
- 主要改动：新增 `PROJECT_MAINTENANCE.md`，整理 `student_template` 的预热算法框架，并记录 `antipinch` 当前参考验证结果。
- 为什么这样改：方便后续修改代码时同步维护算法说明，避免实现、运行方式和口头说明不一致。
- 如何验证：只读检查当前代码和接口；运行 `sdk/student_template/run.exe`，确认电池预热约束检查通过。
- 未覆盖风险：未重新生成 `run.exe`；`student_template` Makefile 的 DLL 路径问题仍存在。
- 需要 reviewer 重点看的文件：`PROJECT_MAINTENANCE.md`、`sdk/student_template/student_solution.c`。
- 提交代号/Commit ID：待提交。
- PR/分支信息：尚未创建 PR，尚未推送。

### 2026-08-18 - 补充二维 MAP 插值说明

- 本次目标：补充三个二维表格插值方法，方便后续评估是否需要优化查表逻辑。
- 主要改动：在 `interp_map()` 小节写明 `R_INT_MAP`、`UOC_MAP`、`P_CHARGE_MAP` 的输入输出、物理用途、双线性插值步骤和潜在优化方向。
- 为什么这样改：插值函数同时影响行驶热仿真、电流估算和充电时间估算，是后续算法优化的重要基础。
- 如何验证：只读检查 `PROJECT_MAINTENANCE.md` 与 `sdk/student_template/student_solution.c` 中 `interp_map()` 的代码一致性；未运行自动化测试。
- 未覆盖风险：本次未修改代码，因此未重新验证数值结果。
- 需要 reviewer 重点看的文件：`PROJECT_MAINTENANCE.md`。
- 提交代号/Commit ID：待提交。
- PR/分支信息：尚未创建 PR，尚未推送。

### 2026-08-18 - 预热开启距离改为粗搜加细搜

- 本次目标：将电池预热寻优策略从目标温度枚举加二分搜索，改为直接对 `start_distance` 做粗网格搜索、局部细搜和最终精修。
- 主要改动：新增 `SearchCandidate`、`evaluate_start_distance()`、`candidate_is_better()`、`remember_refine_center()`；粗搜参数为 `COARSE_STEP_KM = 1.0 km`，保留 `REFINE_CANDIDATE_COUNT = 3` 个可行粗搜候选，细搜参数为 `FINE_RADIUS_KM = 1.0 km`、`FINE_STEP_KM = 0.05 km`，最终精修参数为 `FINAL_RADIUS_KM = 0.05 km`、`FINAL_STEP_KM = 0.005 km`；代价函数沿用归一化 `0.4f * energy_cost + 0.6f * charge_cost`。
- 为什么这样改：直接搜索最终控制量，覆盖完整路线距离区间，并对前 3 个粗搜候选做 50 m 局部细化，最后用 5 m 精修避免 50 m 网格在温度可行边界处过冲。
- 如何验证：运行 `gcc -Wall -Wextra -O2 -std=c11 -I../include -o run.exe student_solution.c -L. -l:competition_mock.dll -lm` 编译通过；运行 `.\run.exe`，输出 `start_distance = 35.32 km`、`T_end_opt = 20.00 C`、`SOC_end_opt = 15.95 %`、`E_heat_opt = 3.1017 kWh`、`chrg_time_s = 915.81 s`，约束检查通过。
- 未覆盖风险：未使用更多隐藏路线或极端温度/SOC 场景验证；归一化权重仍需结合评分规则继续调参。
- 需要 reviewer 重点看的文件：`sdk/student_template/student_solution.c`、`PROJECT_MAINTENANCE.md`。
- 提交代号/Commit ID：待提交。
- PR/分支信息：尚未创建 PR，尚未推送。

### 2026-08-18 - 恢复归一化代价函数

- 本次目标：保留粗搜、细搜和最终精修的搜索覆盖度，同时恢复原有归一化多目标评分口径。
- 主要改动：新增 `SearchScale`、`search_scale_add()`、`normalized_candidate_cost()`；恢复归一化 `0.4f * energy_cost + 0.6f * charge_cost` 评分。
- 为什么这样改：搜索策略可以更精细，但评价标准应与原算法保持一致，避免因为代价函数变化导致结果口径不一致。
- 如何验证：运行 `gcc -Wall -Wextra -O2 -std=c11 -I../include -o run.exe student_solution.c -L. -l:competition_mock.dll -lm` 编译通过；运行 `.\run.exe`，输出 `start_distance = 35.32 km`、`T_end_opt = 20.00 C`、`SOC_end_opt = 15.95 %`、`E_heat_opt = 3.1017 kWh`、`chrg_time_s = 915.81 s`，约束检查通过。
- 未覆盖风险：只验证了当前 mock 路线，未覆盖隐藏路线和极端初始状态。
- 需要 reviewer 重点看的文件：`sdk/student_template/student_solution.c`、`PROJECT_MAINTENANCE.md`。
- 提交代号/Commit ID：待提交。
- PR/分支信息：尚未创建 PR，尚未推送。

### 2026-08-18 - 合并候选后统一归一化评分

- 本次目标：按“粗搜候选、局部评分定范围、细搜和精修候选、合并去重、统一 min/max、重新评分、选最终最优”的流程重构搜索评分。
- 主要改动：新增 `SearchPool` 候选池和 `search_pool_add()`、`search_scale_from_pool()`、`best_from_pool()`；粗搜和细搜的局部评分只用于确定后续搜索范围，最终结果由 `all_pool` 中全部可行候选统一归一化评分选出。
- 为什么这样改：避免粗搜、细搜、精修各自评分导致 cost 尺度不一致，同时保留局部细化搜索的覆盖度和精度。
- 如何验证：运行 `gcc -Wall -Wextra -O2 -std=c11 -I../include -o run.exe student_solution.c -L. -l:competition_mock.dll -lm` 编译通过；运行 `.\run.exe`，输出 `start_distance = 35.32 km`、`T_end_opt = 20.00 C`、`SOC_end_opt = 15.95 %`、`E_heat_opt = 3.1017 kWh`、`chrg_time_s = 915.81 s`，约束检查通过。
- 未覆盖风险：只验证了当前 mock 路线，未覆盖隐藏路线和极端初始状态；`MAX_SEARCH_CANDIDATES = 512` 对当前参数足够，若未来把路线距离或细搜候选数量显著放大，需要同步检查容量。
- 需要 reviewer 重点看的文件：`sdk/student_template/student_solution.c`、`PROJECT_MAINTENANCE.md`。
- 提交代号/Commit ID：待提交。
- PR/分支信息：尚未创建 PR，尚未推送。

### 2026-08-18 - 粗搜细化范围改为自适应区间

- 本次目标：将固定“粗搜前 3 名候选分别细搜”改成基于代价余量的自适应细搜区间。
- 主要改动：新增 `REFINE_COST_MARGIN = 0.03f`、`MAX_REFINE_INTERVALS = 8`、`SearchInterval`、`add_refine_interval()`、`build_refine_intervals()`；粗搜局部评分后保留所有 `candidate.cost <= best_cost + 0.03` 的近优候选，并按距离合并相邻区间；`MAX_SEARCH_CANDIDATES` 从 512 提升到 2048，降低自适应细搜范围扩大时的截断风险。
- 为什么这样改：固定前 3 名可能漏掉第 4、5 名附近的近优区域；直接对所有近优候选单独细搜又会在平坦代价曲线下重复仿真。区间合并可以兼顾覆盖度和速度。
- 如何验证：运行 `gcc -Wall -Wextra -O2 -std=c11 -I../include -o run.exe student_solution.c -L. -l:competition_mock.dll -lm` 编译通过；运行 `.\run.exe`，输出 `start_distance = 35.32 km`、`T_end_opt = 20.00 C`、`SOC_end_opt = 15.95 %`、`E_heat_opt = 3.1017 kWh`、`chrg_time_s = 915.81 s`，约束检查通过。
- 未覆盖风险：只验证了当前 mock 路线，未覆盖隐藏路线和极端初始状态；如果隐藏场景出现超过 8 个互不相邻的近优区间，可能需要提高 `MAX_REFINE_INTERVALS`。
- 需要 reviewer 重点看的文件：`sdk/student_template/student_solution.c`、`PROJECT_MAINTENANCE.md`。
- 提交代号/Commit ID：待提交。
- PR/分支信息：尚未创建 PR，尚未推送。

### 2026-08-18 - 搜索筛选改为 Pareto 保护闭环

- 本次目标：降低动态 Min-Max 归一化在多层搜索中造成排序反转和过早漏筛的风险。
- 主要改动：新增 `score_pool_with_scale()`、`score_pool_against()`、`candidate_dominates()`、`candidate_is_pareto()`、`curve_local_minimum()`、`store_candidate()`、`sample_intervals()`；所有候选进入 `all_pool`，每轮新增候选后对全池统一归一化重评分；细搜区间由 Pareto 非支配、最小能耗、最小充电时间、近优综合 cost 和当前采样曲线局部最优共同决定；精修层级为 `REFINE_STEPS_KM = {0.05, 0.005}`，`MAX_REFINE_INTERVALS = 16`，`MAX_SEARCH_CANDIDATES = 20000`。
- 为什么这样改：动态 Min-Max 归一化适合最终统一排序，但不适合单独承担候选淘汰；Pareto 和极值保护可以保留两目标权衡上的关键点，局部最优保护可以覆盖粗搜曲线中的多峰结构，区间合并可以减少重复仿真。
- 如何验证：运行 `gcc -Wall -Wextra -O2 -std=c11 -I../include -o run.exe student_solution.c -L. -l:competition_mock.dll -lm` 编译通过；运行 `.\run.exe`，输出 `start_distance = 35.32 km`、`T_end_opt = 20.00 C`、`SOC_end_opt = 15.95 %`、`E_heat_opt = 3.1017 kWh`、`chrg_time_s = 915.81 s`，约束检查通过。
- 未覆盖风险：只验证了当前 mock 路线，未覆盖隐藏路线和极端初始状态；若未来路线显著变长且 Pareto 前沿覆盖全程，可能需要继续提高候选池容量或增加候选池满时的降级策略。
- 需要 reviewer 重点看的文件：`sdk/student_template/student_solution.c`、`PROJECT_MAINTENANCE.md`。
- 提交代号/Commit ID：待提交。
- PR/分支信息：尚未创建 PR，尚未推送。

### 2026-08-18 - 充电时间改为 PDF 恒功率口径

- 本次目标：将 `chrg_time_s` 计算从逐秒积分查表改为 PDF 3.4 中的终点功率恒定近似。
- 主要改动：`estimate_charge_time()` 现在只用到站终点 `(T_end, SOC_end)` 查询一次 `P_CHARGE_MAP`，并用 `(SOC_TARGET - SOC_end) * C_NOM_AH * U_NOM * 3.6f / P_cha` 直接计算充电秒数；同步更新主函数注释和本文档中的充电时间模型说明。
- 为什么这样改：PDF 原模型给出 `P_cha = f(SOC_end, T_end)`，再使用终点充电功率进行全程恒功率近似；该口径更贴近赛题公式，也减少了充电时间估算中的循环计算。
- 如何验证：运行 `gcc -Wall -Wextra -O2 -std=c11 -I../include -o run.exe student_solution.c -L. -l:competition_mock.dll -lm` 编译通过；运行 `.\run.exe`，输出 `start_distance = 35.32 km`、`T_end_opt = 20.00 C`、`SOC_end_opt = 15.95 %`、`E_heat_opt = 3.1017 kWh`、`chrg_time_s = 947.16 s`，约束检查通过。
- 未覆盖风险：只验证了当前 mock 路线，未覆盖隐藏路线和极端初始状态；恒功率口径与逐秒积分口径数值不同，后续应以 PDF/官方评分口径为准。
- 需要 reviewer 重点看的文件：`sdk/student_template/student_solution.c`、`PROJECT_MAINTENANCE.md`。
- 提交代号/Commit ID：待提交。
- PR/分支信息：尚未创建 PR，尚未推送。

### 2026-08-18 - 按候选来源层级补做细化搜索

- 本次目标：修复重新晋级的粗搜候选可能直接跳到精修尺度、漏掉应有 0.05 km 细搜范围的问题。
- 主要改动：新增 `SearchLevel`、`SearchCandidate.level`、`SearchCandidate.refined_to_next`；保留长期存在的 `coarse_pool`、`fine_pool`、`final_pool`、`all_pool`；粗搜候选晋级后只补做 `±1.0 km / 0.05 km` 细搜，细搜候选晋级后只补做 `±0.05 km / 0.005 km` 精修；新增 `fine_covered` 和 `final_covered` 覆盖区间，减少重复仿真；移除会丢失层级语义的 `layer_a/layer_b` 交换式搜索。
- 为什么这样改：候选是否应该细化取决于它自己的采样分辨率，而不是当前循环处于哪一层；这样即使粗搜候选在后续全局重评分后才重新变优，也会先补做完整细搜区间，不会跳过中间尺度。
- 如何验证：运行 `gcc -Wall -Wextra -O2 -std=c11 -I../include -o run.exe student_solution.c -L. -l:competition_mock.dll -lm` 编译通过；运行 `.\run.exe`，输出 `start_distance = 35.32 km`、`T_end_opt = 20.00 C`、`SOC_end_opt = 15.95 %`、`E_heat_opt = 3.1017 kWh`、`chrg_time_s = 947.16 s`，约束检查通过。
- 未覆盖风险：只验证了当前 mock 路线，未覆盖隐藏路线和极端初始状态；覆盖区间目前以“完全覆盖则跳过、部分重叠则采样并由候选池去重”的方式减少重复，若未来性能压力很大，可以进一步切分只采样未覆盖子区间。
- 需要 reviewer 重点看的文件：`sdk/student_template/student_solution.c`、`PROJECT_MAINTENANCE.md`。
- 提交代号/Commit ID：待提交。
- PR/分支信息：尚未创建 PR，尚未推送。

### 2026-08-18 - 区间动态分配和 Pareto 排序扫描

- 本次目标：取消每轮 16 个细化区间的逻辑上限，并把 Pareto 判断从两两比较优化为二维排序扫描。
- 主要改动：删除 `MAX_REFINE_INTERVALS`，新增 `SearchCandidate.pareto`、`ParetoItem`、`CandidateRef`、`compute_pareto_flags()`、`compare_interval_low()`、`compare_pareto_item()`、`compare_candidate_ref()`、`find_candidate_ref()`；`build_level_refine_intervals()` 改为按 `level_pool->count` 动态分配 `raw_intervals` 和 `merged_intervals`，先收集全部原始区间，再 `qsort` 排序并线性合并；`candidate_is_promising()` 直接读取候选的 `pareto` 标志。
- 为什么这样改：每个候选最多产生一个原始区间，因此按当前层候选数量分配就足够，不会丢弃第 17 个之后的区间；二维 Pareto 可通过按充电时间和能耗排序后扫描完成，避免候选晋级判断中反复执行最坏 `O(N^2)` 的两两支配检查。
- 如何验证：运行 `gcc -Wall -Wextra -O2 -std=c11 -I../include -o run.exe student_solution.c -L. -l:competition_mock.dll -lm` 编译通过；运行 `.\run.exe`，输出 `start_distance = 35.32 km`、`T_end_opt = 20.00 C`、`SOC_end_opt = 15.95 %`、`E_heat_opt = 3.1017 kWh`、`chrg_time_s = 947.16 s`，约束检查通过。
- 未覆盖风险：只验证了当前 mock 路线，未覆盖隐藏路线和极端初始状态；动态分配失败时会打印错误并保守跳过本轮细化，覆盖区间仍为固定数组但只影响减少重复搜索，不影响候选是否可被保存。
- 需要 reviewer 重点看的文件：`sdk/student_template/student_solution.c`、`PROJECT_MAINTENANCE.md`。
- 提交代号/Commit ID：待提交。
- PR/分支信息：尚未创建 PR，尚未推送。

### 2026-08-18 - 完善搜索闭环边界处理

- 本次目标：只完善电池预热寻优的搜索闭环可靠性，不改变物理模型、充电时间公式或归一化代价函数。
- 主要改动：新增 `AddResult` 区分 `ADD_NEW`、`ADD_DUPLICATE`、`ADD_FULL`；无可行粗搜点时使用 SOC 安全且温度最接近目标下限的 fallback 补搜 `±1 km / 0.05 km`；`refined_to_next` 改为在采样成功、覆盖记录成功、完整请求区间回查已覆盖后再设置；覆盖区间容量溢出会打印 `ERROR: covered interval capacity exceeded` 并停止继续扩展；局部最小值改为用 `CandidateRef` 按真实 `dist_km` 排序后比较左右邻居。
- 为什么这样改：避免无可行粗搜点时直接退出、候选生成区间后被误认为已细化、重复候选和满池错误混淆、覆盖记录失败后反复搜索、以及插入顺序影响局部最优判断。
- 如何验证：运行 `gcc -Wall -Wextra -O2 -std=c11 -I../include -o run.exe student_solution.c .\competition_mock.dll -lm` 编译通过；运行 `.\run.exe`，输出 `start_distance = 35.32 km`、`T_end_opt = 20.00 C`、`SOC_end_opt = 15.95 %`、`E_heat_opt = 3.1017 kWh`、`chrg_time_s = 947.16 s`，约束检查通过。`mingw32-make clean; mingw32-make` 仍会失败，因为 Makefile 依赖 `../lib/competition_mock.dll`，而当前 DLL 位于 `sdk/student_template/competition_mock.dll`。
- 未覆盖风险：fallback 和覆盖区间溢出属于边界逻辑，未用专门构造数据触发验证；Makefile 的 DLL 路径尚未修改。
- 需要 reviewer 重点看的文件：`sdk/student_template/student_solution.c`、`PROJECT_MAINTENANCE.md`。
- 提交代号/Commit ID：待提交。
- PR/分支信息：尚未创建 PR，尚未推送。

### 2026-08-18 - 生成导航训练集预热标签

- 本次目标：基于 `sdk/data/nav_train_100000.txt` 的 100000 条导航工况，用当前 `student_solution.c` 搜索算法生成强化学习训练标签。
- 主要改动：新增 `sdk/student_template/generate_nav_labels.c` 批量标签生成器；该工具直接 include 当前 `student_solution.c` 以复用搜索算法，并为接口函数提供本地 stub；生成 `sdk/data/nav_train_100000_labels.csv`，字段为 `case_id,best_start_distance_km,min_charge_s,max_charge_s,min_heat_kWh,max_heat_kWh`。
- 为什么这样改：逐条启动 exe 成本过高；批量工具在同一进程内反复填充 `g_segs` 并调用搜索逻辑，能稳定处理 100000 个工况，同时不影响正式提交用的 `student_solution.c` 主入口。
- 如何验证：运行 `gcc -Wall -Wextra -O2 -std=c11 -I../include -o generate_nav_labels.exe generate_nav_labels.c -lm` 编译通过；先用 `.\generate_nav_labels.exe ..\data\nav_train_100000.txt ..\data\nav_train_100000_labels_sample10.csv 10` 抽测，确认无可行候选工况输出全 `nan`；随后 4 分片并行跑满 100000 条并合并，最终 `nav_train_100000_labels.csv` 共 100001 行，含表头和 100000 条工况标签。统计结果：`zero_min_heat=0`，`positive_min_heat=52378`，`nan_min_heat=47622`，`nan_best=47622`。
- 未覆盖风险：标签生成使用固定初始状态 `T_init = 0 C`、`SOC_init = 65 %`；min/max 只统计当前搜索闭环实际评估过的可行候选，不等于对连续动作空间做数学全局穷举。
- 需要 reviewer 重点看的文件：`sdk/student_template/generate_nav_labels.c`、`sdk/data/nav_train_100000_labels.csv`、`PROJECT_MAINTENANCE.md`。
- 提交代号/Commit ID：待提交。
- PR/分支信息：尚未创建 PR，尚未推送。

### 2026-08-18 - 修正候选距离去重边界

- 本次目标：修复总里程浮点误差导致全程预热候选可能被相邻粗搜网格点误去重的问题。
- 主要改动：在 `student_solution.c` 中新增 `DIST_KEY_EPS_KM = 1.0e-6f`，并用于 `search_pool_find_index()`、`search_pool_add()` 和 `find_candidate_ref()` 的候选距离键匹配；覆盖区间合并容差保持不变。
- 为什么这样改：例如某工况总里程为 `39.000003815 km`，粗搜点 `39.000000 km` 与真实 `route_km` 只差几毫米。旧的 `1e-4 km` 去重会把二者当成同一候选，导致真实全程预热点没有被评估；收紧到 `1e-6 km` 后可保留真实端点候选。
- 如何验证：`case 40195` 从 `nan,nan,nan,nan,nan` 变为 `39.000,857.525452,857.525452,3.199827,3.199827`；重新生成全量 `nav_train_100000_labels.csv` 后，统计结果为 `zero_min_heat=0`、`positive_min_heat=52378`、`nan_min_heat=47622`、`nan_best=47622`。运行 `gcc -Wall -Wextra -O2 -std=c11 -I../include -fsyntax-only student_solution.c` 和 `gcc -Wall -Wextra -O2 -std=c11 -I../include -fsyntax-only generate_nav_labels.c` 均通过。
- 未覆盖风险：未对官方隐藏工况重新跑评分；该修复只影响候选去重精度，不改变物理模型、硬约束或代价函数。
- 需要 reviewer 重点看的文件：`sdk/student_template/student_solution.c`、`sdk/student_template/generate_nav_labels.c`、`sdk/data/nav_train_100000_labels.csv`、`PROJECT_MAINTENANCE.md`。
- 提交代号/Commit ID：待提交。
- PR/分支信息：尚未创建 PR，尚未推送。

### 2026-08-19 - 将预热寻优算法适配到新模板

- 本次目标：把 `student_solution(me).c` 中的电池预热最优开启距离算法适配到新项目结构的 `student_solution.c`。
- 主要改动：仅在模板算法区域补入路线仿真、二维 MAP 插值、PDF 3.4 终点恒功率充电时间估算和 `1.0/0.05/0.005 km` 分层搜索；主函数改为调用 `find_optimal_start_distance()`；保留 `DIST_KEY_EPS_KM = 1.0e-6f`；最终无可行候选时统一输出 `NaN`，fallback 只用于补搜。
- 为什么这样改：复用已经验证过的搜索框架，同时避免未通过温度和 SOC 硬约束的候选被误报为最终最优结果。
- 如何验证：在 `C:\Users\17871\Desktop\sdk\student_template` 运行 `mingw32-make -B` 和 `.\build_student.bat`，两次均编译运行成功且无编译警告；实测输出 `start_distance = 29.65 km`、`T_end_opt = 20.01 C`、`SOC_end_opt = 12.20 %`、`E_heat_opt = 3.1778 kWh`、`chrg_time_s = 1030.53 s`，硬约束检查全部通过。
- 未覆盖风险：只验证了当前 mock 路线，尚未覆盖官方隐藏路线和专门构造的最终无可行候选工况。
- 需要 reviewer 重点看的文件：`sdk/student_template/student_solution.c`、`sdk/student_template/PROJECT_MAINTENANCE.md`。
- 提交代号/Commit ID：待提交。
- PR/分支信息：尚未创建 PR，尚未推送。

### 2026-08-19 - 采用时间边界和 0.2 s 分层搜索

- 本次目标：以“时间边界确定可行区间 + 可行区间内分层搜索”替代全路线浮点距离搜索，同时保持物理参数、二维 MAP 数值、硬约束、充电时间公式和 `0.4/0.6` score 权重不变。
- 主要改动：保留原 `interp_map()` 双线性插值并增加 `battery_resistance()`、`open_circuit_voltage()`、`charge_power()` 三个薄包装函数，不增加插值点或函数拟合；建立 1 s/0.2 s 路线基线，以 0.2 s 整数启动步去重；精修 `t20/t25/tSOC` 后直接计算能耗端点；粗、细、精候选按层级晋升并在统一 0.2 s 候选池中反复重评分；1 s 趋势探针发现非单调时改用 0.2 s 全步兜底；正式无解统一输出五个 `nan`。
- 生成器改动：`generate_nav_labels.c` 直接复用 `optimize_preheat()`，新增固定随机种子的 0.2 s 穷举对照入口，并检查 CSV 表头、标签行和 `fclose()` 写入错误，磁盘写满时返回失败。
- 为什么这样改：整数时间步消除浮点路线端点误去重；时间边界把正式搜索裁剪到硬约束可行区间；统一 0.2 s 仿真避免混用 1 s/0.2 s 结果；全池重评分允许旧粗搜锚点重新晋级；写错误检查避免磁盘满时产生看似完成的截断标签。
- 如何验证：`gcc -Wall -Wextra -O2 -std=c11 -I../include -fsyntax-only student_solution.c`、同参数检查 `generate_nav_labels.c`、`mingw32-make -B` 和 `build_student.bat` 均通过且无新增警告；mock 输出 `29.64 km / 20.00 C / 12.20 % / 3.1772 kWh / 1030.53 s`，硬约束全部通过；case 40195 输出精确端点 `39.000 km`，case 9470 五个标签均为 `nan`；随机 100 条及全部 75 条新旧可行性变化工况分别与 0.2 s 全步穷举比较，均为 `mismatches = 0`。
- 全量结果：旧版有效/无效为 `52378/47622`；新版为 `52425/47575`。旧无效中有 61 条在 0.2 s 模型下可行，旧有效中有 14 条在 0.2 s 模型下无解；75 条均由穷举复核。旧版单进程 wall time `2656.807 s`；新版四分片 wall time `1344.457 s`、CPU 合计 `5195.062 s`。
- 未覆盖风险：60 s 趋势探针无法从理论上发现比探针间隔更窄的孤立非单调小岛；随机和变化工况穷举不等于穷举全部 100000 工况；未运行官方隐藏工况。C 盘曾写满并导致第一次临时 CSV 截断，生成器现已增加写错误检查，最终完整标签已通过 D/C 两份文件的 SHA-256 一致性核对。
- 需要 reviewer 重点看的文件：`sdk/student_template/student_solution.c`、`sdk/student_template/generate_nav_labels.c`、`sdk/student_template/PROJECT_MAINTENANCE.md`。
- 提交代号/Commit ID：待提交。
- PR/分支信息：尚未创建 PR，尚未推送。
