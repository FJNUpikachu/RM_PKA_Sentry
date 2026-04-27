# 前哨站（Outpost）解算说明

所有前哨站相关逻辑集中在 **`outpost_solver.hpp` / `outpost_solver.cpp`**。  
`armor_tracker`、`armor_solver_node`、`armor_solver` 仅调用这里的接口，不含任何前哨站实现细节。

---

## 1. 物理模型

2026 赛季前哨站物理特征：

| 特征 | 说明 |
|------|------|
| 装甲板数量 | 3 块小装甲板 |
| 水平分布 | 均匀分布在半径 `R` 的圆上，相邻夹角 120° |
| **高度差异** | 三块板**高度各不相同**（本赛季新变化），相邻板高度差为 `d_za`（EKF 在线估计）|
| 旋转方式 | 绕竖直轴匀速旋转，角速度 `v_yaw` 由 EKF 估计 |

**坐标约定**（与 `Solver::getArmorPositions` / `outpostThreePlateSlotWorldPosition` 一致）：

设旋转中心为 `(xc, yc, zc)`，`yaw` 为第 0 块板（slot-0）的朝向角，则第 `i` 块板（`i = 0, 1, 2`）的世界坐标为：

```
水平角:  θ_i = yaw + i × (2π/3)
位置 X:  xc - R · cos(θ_i)
位置 Y:  yc - R · sin(θ_i)
位置 Z:  zc + d_zc + i × d_za
```

其中 `d_zc` 是 slot-0 相对于中心高度 `zc` 的偏移量（EKF state(9)），`d_za` 是相邻板高度步长（EKF state(10)）。

---

## 2. EKF 状态与观测

### 状态向量（11 维，与其他兵种共用）

```
x = [xc, vx, yc, vy, zc, vz, yaw, v_yaw, r, d_zc, d_za]
```

| 索引 | 含义 | 前哨处理 |
|------|------|---------|
| 0, 1 | 中心 x 坐标及其速度 | EKF 估计 |
| 2, 3 | 中心 y 坐标及其速度 | EKF 估计 |
| 4, 5 | 中心 z 坐标及其速度 | EKF 估计 |
| 6, 7 | slot-0 朝向角 `yaw` 及角速度 `v_yaw` | EKF 估计 |
| 8    | 旋转半径 `r`（初始为 `radius_m` yaml 值，EKF 在线收敛）| EKF 估计 |
| 9    | slot-0 高度偏移 `d_zc`（初始 0）| EKF 估计 |
| 10   | 相邻板高度步长 `d_za`（初始为 `dz_m` yaml 值，EKF 在线收敛）| EKF 估计 |

> **非前哨机器人**：state(10) 始终钳制为 0，`Measure.slot_k = 0` 恒成立，因此 d_za 对非前哨 EKF 无任何影响。

### 可观性分析

`Measure` 函数（slot-k 感知版本）：

```
z[0] = xc - r·cos(yaw)              (XY 已旋转到 slot-0 等效)
z[1] = yc - r·sin(yaw)
z[2] = zc + d_zc + slot_k · d_za    ← H[2,10] = slot_k
z[3] = yaw                          (yaw 已减去 k·2π/3)
```

- `slot_k = 0`：d_za 不出现在 H，**不可观**（slot-0 板或非前哨目标）
- `slot_k = 1`：H[2,10] = 1，**d_za 可观**
- `slot_k = 2`：H[2,10] = 2，**d_za 可观**

实现路径：`outpostPrepareEKFMeasurement()` 在每次 EKF::update() 前注入 `Measure.slot_k = k`，使 EKF 雅可比矩阵中 d_za 的梯度正确传播。

---

## 3. 两种瞄准模式与单板回退

### 模式 0：单板（SINGLE_PLATE）

每帧直接用当前检测到的装甲板推算虚拟中心，发布 `Target.armors_num = 1`。  
Solver 直接瞄准该装甲板，**不依赖旋转预测**，作为保底兜底方案。  
开火条件受 `fire_yaw_max_deg` 限制（|yaw_diff| ≤ 门限才允许开火）。

### 模式 1：融合（FUSION）

EKF 持续跟踪三板的旋转状态，发布 `Target.armors_num = 3`。  
Solver 根据三块板的预测位置选择最佳打击目标，**具备超前预测能力**。

**DETECTING 阶段自动回退**：  
融合模式下，EKF 收敛前（`tracker_state == DETECTING`），节点自动调用
`outpostFillDetectingTarget()` 发布 `armors_num = 1` 的单板目标，使 Solver 可以
立即输出有效 `cmd_gimbal`；EKF 进入 `TRACKING` 后无缝切换到三板模型。

`Target` 字段含义（FUSION/TRACKING 状态）：

| 字段 | 含义 |
|------|------|
| `position` | 旋转中心 `(xc, yc, zc)` |
| `velocity` | 中心速度（EKF 输出；前哨站固定，通常接近 0）|
| `yaw` | slot-0 当前朝向角 |
| `v_yaw` | 角速度（EKF 在线估计）|
| `radius_1 = radius_2` | EKF 估计的旋转半径（state(8)，不再使用固定 yaml 值）|
| `d_zc` | EKF 估计的 slot-0 高度偏移（state(9)）|
| `d_za` | EKF 估计的相邻板高度步长（state(10)，不再使用固定 yaml 值）|

---

## 4. 单板火控限制（fire_yaw_max_deg）

当 `armors_num == 1`（单板模式或 DETECTING 回退）时，Solver 在完成 yaw/pitch 解算后
调用 `outpostApplySinglePlateFireConstraint()`：

```cpp
if (|yaw_diff| > p.fire_yaw_max_deg) { fire_advice = false; }
```

参数 `outpost.fire_yaw_max_deg`（单位：度）在 yaml 中配置，默认 8°。

---

## 5. 整体数据流（每帧）

```
检测节点 → Armors 消息
    ↓
armor_solver_node::armorsCallback()
    ↓ LOST 状态
    outpostInitState()                 ← outpost_solver（11-D 状态初始化）
    tracker_->init()

    ↓ DETECTING 状态（前哨融合模式回退）
    EKF::predict() + EKF::update()
    outpostFindBestSlotMatch()         ← outpost_solver（读 ekf_pred(10) 为 d_za）
    outpostPrepareEKFMeasurement()     ← outpost_solver（注入 slot_k，XY 映射，Z 保持原始）
    outpostFillDetectingTarget()       ← outpost_solver（armors_num=1 单板回退）

    ↓ TRACKING / TEMP_LOST 状态
    outpostFindBestSlotMatch()
    outpostPrepareEKFMeasurement()
    EKF::predict() + EKF::update()
    outpostApplyTargetFields()         ← outpost_solver（按模式填充 Target，使用 EKF state(8/10)）
    ↓
target_pub_ 发布 Target
    ↓
armor_solver_node::timerCallback()
    ↓
Solver::solve()
    getArmorPositions()                ← 调用 outpostThreePlateSlotWorldPosition()
    selectBestArmor()
    calcYawAndPitch()
    outpostApplySinglePlateFireConstraint() ← outpost_solver（单板 yaw 限制）
    ↓
gimbal_pub_ 发布 GimbalCmd
```

---

## 6. 接口速查

| 接口 | 调用位置 | 作用 |
|------|----------|------|
| `declareOutpostParameters` | node 构造函数 | 声明所有 `outpost.*` 参数（含新增参数）|
| `loadOutpostParams` | node 构造函数 | 读取几何参数与火控限制 |
| `loadOutpostMode` | node 构造函数 | 读取瞄准模式（0/1）|
| `filterOutpostArmors` | armor_solver_node | 从 Armors 消息筛出前哨装甲板 |
| `isOutpostId` | 多处 | 判断 id 是否为前哨 |
| `outpostInitState` | armor_tracker::initEKF | 单板初始化 11-D EKF 状态 |
| `outpostThreePlateSlotWorldPosition` | armor_solver::getArmorPositions | 计算第 i 块板的世界坐标 |
| `outpostFindBestSlotMatch` | armor_tracker::update | 匹配观测与三块预测 slot（d_za 从 ekf_pred(10) 读）|
| `outpostPrepareEKFMeasurement` | armor_tracker::update | 注入 slot_k，XY/yaw 映射 slot-0，Z 保持原始 |
| `outpostApplyTargetFields` | armor_solver_node::armorsCallback | 按模式填充 Target（FUSION 用 EKF state(8/10)）|
| `outpostFillDetectingTarget` | armor_solver_node::armorsCallback | DETECTING 阶段单板回退目标 |
| `outpostApplySinglePlateFireConstraint` | armor_solver::solve | 单板火控 yaw 限制 |
| `appendOutpostFilteredArmorsStripMarkers` | armor_solver_node::publishMarkers | RViz 三板可视化 |

---

## 7. 参数配置（YAML）

推荐在 `rm_bringup/config/node_params/armor_solver_params.yaml` 中配置：

```yaml
outpost:
  id: "outpost"                  # 与检测器贴纸字段一致

  # 物理参数（作为 EKF 初始值，EKF 会在线收敛到真实值）
  radius_m: 0.275                # 旋转半径 (m) → EKF state(8)
  dz_m: 0.102                    # 相邻板高度差 (m) → EKF state(10)
  plate_pitch_rad: -0.2618       # 装甲板俯仰角（仅影响 RViz 显示）

  # EKF 过程噪声（前哨专用，与地面兵种分离）
  sigma2_q_d_za: 50.0            # d_za 过程噪声（slot_k≠0 时可观）

  # 瞄准模式: 0=单板 1=融合（推荐）
  mode: 1

  # 单板火控限制（DETECTING 阶段回退 + SINGLE_PLATE 模式均生效）
  fire_yaw_max_deg: 8.0          # |yaw_diff| 超过此值时 fire_advice=false

  # 前哨专用 tracker 参数（与地面兵种分开调参）
  tracker:
    max_match_distance: 0.40
    max_match_yaw_diff: 1.8
    tracking_thres: 1
    lost_time_thres: 3.0
```

调试：

```yaml
debug_outpost: true   # 开启后输出 [Outpost::...] 级别日志
```

---

## 8. 第一块板非 slot-0 时的收敛行为（常见问题）

**问：若第一次观测到的不是最下面的板（slot-0），系统会重置吗？**

不会重置。`outpostInitState` 将观测板当作 slot-0 初始化（`d_zc=0`），后续板被预测在更高位置。实际的 slot-0 与模型最近 slot 位置差约 `dz`（0.102m），远小于匹配门限 0.40m，所以**匹配不会失败，tracker 不会进入 LOST**。EKF 通过 `d_zc` 和 `d_za` 的在线更新逐渐收敛到真实几何。

**问：若最初连续看到的两块板高度差为 2×dz，后续 EKF 能正确收敛吗？**

可以收敛。高度差 2×dz 对应看到 slot-i 和 slot-i+2（跳过中间一块）。
- slot 匹配基于 3D 距离，水平方向（120° 间隔）提供额外约束；
- 当第三块板（高度差 dz）被观测到后，EKF 的 `d_za`（state(10)）估计值会受到
  slot_k≠0 的 z 方向残差驱动，在若干帧内收敛到真实步长；
- `d_zc` 同时自适应调整，确保整体高度模型的一致性。

---

## 9. 已知限制

- **d_za 可观性依赖 slot 多样性**：若长时间只看到同一块板（slot_k 恒为 0），d_za 无法更新，只随过程噪声缓慢漂移。正常旋转时三板交替出现，d_za 可以快速收敛。
- **收敛需要若干帧**：d_za 和 r 从 yaml 初始值收敛到真实值通常需要 10～50 帧（约 0.05～0.25 s），期间使用单板回退保证火控连续性。
