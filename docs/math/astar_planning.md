# 栅格上的 A\* 全局规划:地图、膨胀、启发式与最优性

> 本文是 MiniNav V3 全局规划的数学说明:占据栅格与坐标变换、障碍物膨胀
> (配置空间)、A\* 的最优性证明、三种启发式在 4 / 8 连通下的可采纳性与
> 一致性,以及防穿角、tie-break、代价梯度这几条实现规则的依据。对应代码在
> `src/planning/`;工程总结见 [`docs/v3_summary.md`](../v3_summary.md),
> 定量实验见 [`docs/experiments/v3_planning.md`](../experiments/v3_planning.md)。

## 目录

1. [问题陈述](#1-问题陈述)
2. [占据栅格与坐标变换](#2-占据栅格与坐标变换)
3. [配置空间与障碍物膨胀](#3-配置空间与障碍物膨胀)
4. [A\* 搜索与最优性](#4-a-搜索与最优性)
5. [启发式:可采纳性与一致性](#5-启发式可采纳性与一致性)
6. [8 连通的两条附加规则](#6-8-连通的两条附加规则)
7. [代价梯度](#7-代价梯度)
8. [复杂度](#8-复杂度)
9. [栅格路径的几何局限](#9-栅格路径的几何局限)
10. [与实现的对应](#10-与实现的对应)

---

## 1. 问题陈述

占据栅格把平面离散成 $W \times H$ 个方格(cell)。把每个可通行 cell 看作图的
顶点、相邻关系看作边,全局规划就化为图上的单源单汇最短路问题:

$$
G = (V, E, c), \qquad V = \{\text{可通行 cell}\}, \qquad c : E \to \mathbb{R}_{>0}.
$$

- **4 连通**:$E$ 只含上下左右四个邻居,每步代价 $1$(以 cell 为单位)。
- **8 连通**:再加四个对角邻居,代价 $\sqrt2$,并受防穿角规则约束(§6.1)。

给定起点 $s$、目标 $t$,求路径 $\pi = (s = n_0, n_1, \dots, n_k = t)$,使

$$
C(\pi) = \sum_{i=1}^{k} c(n_{i-1}, n_i)
$$

最小;world 长度等于 $C(\pi)$ 乘以分辨率 $\rho$(米/cell)。记号:

- $g^*(n)$:$s \to n$ 的最短代价;
- $h^*(n)$:$n \to t$ 的最短代价;
- $C^* = g^*(t) = h^*(s)$:最优解代价。

建模假设(与实现一致):

1. 地图静态、完全已知;
2. 机器人经膨胀"缩成一个点"(§3);
3. 连续的起止点先量化到所在 cell 的中心(§2.3)。

---

## 2. 占据栅格与坐标变换

### 2.1 从 PGM 灰度到三值占据

沿用 ROS `map_server` 约定。像素值 $p \in [0, p_{\max}]$ 先转成"占据程度":

$$
\text{occ} =
\begin{cases}
1 - p / p_{\max}, & \texttt{negate} = 0 \ (\text{暗 = 占据}) \\
p / p_{\max},     & \texttt{negate} = 1
\end{cases}
$$

再按两个阈值三值化:$\text{occ} > \tau_{\text{occ}}$(默认 0.65)为占据(100),
$\text{occ} < \tau_{\text{free}}$(默认 0.25)为空闲(0),其余为未知(−1)。

图像行自上而下存储,world 的 $y$ 轴向上,所以加载时做 $y$ 翻转:图像第 $r$ 行
对应 grid 行 $y = H - 1 - r$。

### 2.2 world ↔ grid

$\mathbf{o}$ 是栅格**左下角**的 world 坐标,$\rho$ 是分辨率:

$$
\text{world\_to\_grid}(\mathbf{p}) =
\left( \left\lfloor \frac{p_x - o_x}{\rho} \right\rfloor,\;
       \left\lfloor \frac{p_y - o_y}{\rho} \right\rfloor \right),
\qquad
\text{grid\_to\_world}(c) = \mathbf{o} + \rho \left( c + \tfrac12 \right).
$$

**往返一致**:

$$
\text{world\_to\_grid}\big(\text{grid\_to\_world}(c)\big)
= \left\lfloor c + \tfrac12 \right\rfloor = c.
$$

格心到两侧 cell 边界各有 $\rho/2$ 的余量,浮点舍入不会把它推过边界
(测试 `RoundTripCellCenterIsStable`)。

**越界约定**:`at(c)` 对越界 cell 返回占据,等价于在地图外围加了一圈墙
(封闭世界假设)。A\* 的邻居生成因此不必单独做边界判断。

### 2.3 量化误差

连续点 $\mathbf{p}$ 被映射到它所在 cell 的中心,两者在每个轴上相差不超过
$\rho/2$,欧氏距离不超过

$$
\|\Delta\| \le \frac{\sqrt2}{2}\,\rho .
$$

$\rho = 0.05\ \text{m}$ 时约 3.5 cm。所以规划路径的首尾 waypoint 是格心,而非
真实起止点;这是一个有界的量化零头(v3_summary §8.5)。

---

## 3. 配置空间与障碍物膨胀

### 3.1 为什么膨胀

把机器人近似为半径 $r$ 的圆盘。它位于 $\mathbf{q}$ 时与障碍集 $O$ 碰撞,当且仅当

$$
\exists\, \mathbf{o} \in O : \|\mathbf{q} - \mathbf{o}\| \le r
\quad\Longleftrightarrow\quad
d(\mathbf{q}, O) \le r,
\qquad d(\mathbf{q}, O) = \min_{\mathbf{o} \in O} \|\mathbf{q} - \mathbf{o}\| .
$$

所以"圆盘在 $O$ 中找路"等价于"质点在膨胀后的障碍集 $O \oplus B_r$
(与半径 $r$ 圆盘的 Minkowski 和)中找路"——这就是配置空间(C-space)的做法。
圆盘对朝向不敏感,C-space 仍是二维,规划器只需处理一个点。

### 3.2 距离变换

在栅格上计算每个 cell 中心到最近占据 cell 中心的欧氏距离(单位 cell):

$$
D(c) = \min_{o \in O} \|c - o\|_2 .
$$

`obstacle_distance_cells` 用多源传播计算它:全部占据 cell 作为源、以距离 0
入最小堆;出队的 cell 把**自己记住的最近源**传给 8 个邻居,邻居的候选距离是
它到**该源**的直线距离,更小则更新并入堆。

它与"每步 +1 / +√2"的 BFS / Dijkstra 的区别在于:这里比较的不是累加步长,而是
到源的真实几何距离,所以结果是(近似)精确的欧氏距离,而不是 Manhattan /
Chebyshev 的阶梯。"近似"是因为这类沿 8 邻域传播最近源的向量传播
(Danielsson 1980 即属此类)在极少数构型下会得到亚 cell 级的误差;ROS
`costmap_2d` 的膨胀层采用同样的做法。需要严格精确时可换用 Felzenszwalb–Huttenlocher
的精确 EDT。每个 cell 只在距离严格减小时重新入堆,实际开销接近 $O(N \log N)$。

### 3.3 膨胀规则

$$
\text{inflate}(c) =
\begin{cases}
\text{占据}, & D(c) \le r / \rho \\
\text{原值}, & \text{否则}
\end{cases}
$$

- 半径内的 unknown 同样变为占据(保守处理);
- $r \le 0$ 时 `inflate` 是恒等映射;
- $D$ 是**格心到格心**的距离,cell 是否被膨胀由它的中心决定。真实障碍边界与
  格心之间最多差 $\frac{\sqrt2}{2}\rho$,所以半径应至少留出这个量级的裕度。

### 3.4 半径怎么选

$r$ 应不小于机器人外接圆半径加安全裕度。当前 `config/planner.yaml` 的
$r = 0.05\ \text{m}$ 小于半轮距 $0.075\ \text{m}$(`kWheelBase = 0.150 m`):
用于验证算法足够,但并不代表真实车体。V4 做路径跟踪前,应改为由底盘外形推导
(v3_summary §8.6)。

---

## 4. A\* 搜索与最优性

### 4.1 算法

A\* 给每个节点一个评估值

$$
f(n) = g(n) + h(n),
$$

$g(n)$ 是目前已知的 $s \to n$ 最小代价,$h(n)$ 是对 $h^*(n)$ 的估计。open 集是
按 $f$ 排序的最小堆,每次取出 $f$ 最小的节点扩展。`astar.cpp` 的结构:

```
g[s] = 0;  push(s, f = h(s))
while open 非空:
    n = pop_min(open)                  # 按 (f, h) 字典序出队,见 §6.2
    if closed[n]: continue             # 惰性删除:跳过过期的堆条目
    closed[n] = true;  expanded += 1
    if n == t: return backtrack(n)     # 首次出队即最优(§4.3)
    for m in neighbors(n):             # 4/8 连通 + 防穿角
        if closed[m]: continue
        tentative = g[n] + c(n, m)
        if tentative < g[m]:
            g[m] = tentative;  parent[m] = n
            push(m, f = tentative + h(m))
return 失败                              # open 耗尽:s 与 t 不连通
```

两处实现细节:

- **惰性删除**:`std::priority_queue` 不支持 decrease-key,$g$ 变小时直接压入
  新条目,出队时用 closed 表跳过过期的条目。
- **closed 节点永不重开**:这一点只在 $h$ **一致**时正确(§4.3)。

取 $h \equiv 0$ 时 $f = g$,A\* 退化为 Dijkstra。实验里的 ground-truth 就是这样
得到的(`scripts/v3/optimality_check.py`)。好的 $h$ 只改变"先扩展谁",不改变
答案——前提是它满足下面的条件。

### 4.2 可采纳性与一致性

- $h$ **可采纳**(admissible):$\forall n,\ h(n) \le h^*(n)$。
- $h$ **一致**(consistent):$h(t) = 0$,且对每条边 $(n, m)$ 有
  $h(n) \le c(n, m) + h(m)$。

**引理 1(一致 ⇒ 可采纳)。** 取 $n = n_0, n_1, \dots, n_k = t$ 为 $n$ 到目标的一条
最短路,对每条边用一致性并做望远镜求和:

$$
h(n_0) \le c(n_0, n_1) + h(n_1) \le \cdots
\le \sum_{i=1}^{k} c(n_{i-1}, n_i) + h(t) = h^*(n).
$$

**引理 2(一致 ⇒ $f$ 沿路径不减)。** 若 $m$ 经边 $(n, m)$ 得到
$g(m) = g(n) + c(n, m)$,则

$$
f(m) = g(n) + c(n, m) + h(m) \ge g(n) + h(n) = f(n).
$$

### 4.3 最优性定理

**定理。** 若 $h$ 一致,则任意节点**首次出队**时,$g(n) = g^*(n)$。

**证明(反证)。** 设 $n$ 是**第一个**在出队时 $g(n) > g^*(n)$ 的节点,于是在它
之前 closed 的节点 $g$ 都已最优。取 $s \to n$ 的一条最短路 $P^*$,令 $n'$ 为
$P^*$ 上第一个尚未 closed 的节点($s$ 最先出队且 $g(s) = 0$ 最优,而 $n$ 自己
尚未 closed,所以 $n' \ne s$ 且存在)。$n'$ 在 $P^*$ 上的前驱已经 closed,$g$
最优,并在扩展时松弛过 $n'$,所以 $g(n') = g^*(n')$ 且 $n'$ 在 open 中。

- 若 $n' = n$,则 $g(n) = g^*(n)$,与假设直接矛盾;
- 否则沿 $P^*$ 从 $n'$ 到 $n$ 对一致性做望远镜求和:

$$
f(n') = g^*(n') + h(n')
\le g^*(n') + c_{P^*}(n' \to n) + h(n)
= g^*(n) + h(n)
< g(n) + h(n) = f(n).
$$

$f(n') < f(n)$,$n'$ 应先于 $n$ 出队,矛盾。∎

**推论。** 目标 $t$ 首次出队时 $g(t) = C^*$,可以立即返回。这就是 `astar.cpp` 中
注释"一致启发式下,首次出队目标即最优"的依据,也是 closed 节点可以永不重开
的依据。

**只可采纳、不一致会怎样。** 可采纳性只保证**允许重开** closed 节点的 A\* 最优。
像本实现这样不重开的 A\*(图搜索版本),还需要一致性,否则可能返回次优路径。
本项目允许的三种组合都一致(§5.2),这个细节不会造成问题,但它说明了"组合
合法"的真正门槛是一致性。

### 4.4 终止与不可达

有限图上每个节点至多 closed 一次,所以有效扩展至多 $|V|$ 次;open 耗尽仍未见到
目标,说明 $s$ 与 $t$ 不连通——此时返回失败,而不是陷入死循环(测试
`UnreachableGoalReturnsNoPath`)。起止 cell 在界外或不可通行时直接判失败,
不做就近吸附。

---

## 5. 启发式:可采纳性与一致性

记 $\Delta x = |x_n - x_t|$、$\Delta y = |y_n - y_t|$(cell 单位),
$m = \min(\Delta x, \Delta y)$,$M = \max(\Delta x, \Delta y)$。

### 5.1 无障碍栅格上的精确代价

**4 连通。** 每步只把一个坐标改变 1,至少需要 $\Delta x + \Delta y$ 步,并且可以
恰好做到:

$$
h^*_{4,\text{free}} = \Delta x + \Delta y \qquad (\text{Manhattan}).
$$

**8 连通。** 设路径含 $a$ 个对角步、$b = b_x + b_y$ 个直走步($b_x$ 沿 $x$,$b_y$
沿 $y$)。对角步同时改变两个坐标各 1,直走步只改变一个,所以必须

$$
a + b_x \ge \Delta x, \qquad a + b_y \ge \Delta y .
$$

代价 $J = \sqrt2\,a + b \ge \sqrt2\,a + (\Delta x - a)^+ + (\Delta y - a)^+ =: \varphi(a)$。
分段看:

- $0 \le a \le m$:$\varphi(a) = \Delta x + \Delta y - (2 - \sqrt2)\,a$,随 $a$ 递减;
- $m \le a \le M$:$\varphi(a) = M + (\sqrt2 - 1)\,a$,随 $a$ 递增。

最小值在 $a = m$ 处取得,而且可以达到(先走 $m$ 个对角步,再直走 $M - m$ 步):

$$
h^*_{8,\text{free}} = \sqrt2\, m + (M - m) = (\Delta x + \Delta y) + (\sqrt2 - 2)\, m
\qquad (\text{Octile}).
$$

第二种写法就是 `astar.cpp` 里的实现形式。

### 5.2 为什么在有障碍时仍可采纳、且一致

**可采纳。** 障碍物以及防穿角规则(§6.1)只会**删边**,不会造出更短的边,所以
真实最短代价只增不减:$h^*(n) \ge h^*_{\text{free}}(n)$。于是 $h^*_{\text{free}}$ 是
$h^*$ 的下界。

**一致。** $h^*_{\text{free}}$ 是无障碍图上的最短路距离,本身满足三角不等式:对无
障碍图的每条边 $h^*_{\text{free}}(n) \le c(n, m) + h^*_{\text{free}}(m)$。有障碍图的
边集是它的子集、代价相同,不等式依旧成立。

**Euclidean。** $h_E = \sqrt{\Delta x^2 + \Delta y^2}$。平面上两点之间直线最短,任何
栅格路径都不会更短,所以 $h_E \le h^*_{8,\text{free}} \le h^*_{4,\text{free}}$,在 4、8
连通下都可采纳。一致性:欧氏距离满足三角不等式,而每条边的代价(1 或 $\sqrt2$)
恰好等于这条边的欧氏长度,所以
$h_E(n) \le \|n - m\| + h_E(m) = c(n, m) + h_E(m)$。

**Octile 用于 4 连通。** 4 连通能走的路 8 连通都能走,所以
$h^*_{8,\text{free}} \le h^*_{4,\text{free}}$,Octile 在 4 连通下同样可采纳。
一致性:Octile 距离 $\max(|x|,|y|) + (\sqrt2 - 1)\min(|x|,|y|)$ 是一个范数
(单位球是正八边形),满足三角不等式;对直走边它等于 1,对对角边它等于
$\sqrt2$,都恰好等于边代价。

| 启发式       | 4 连通               | 8 连通                   |
|-----------|--------------------|------------------------|
| Manhattan | 无障碍时精确;可采纳、一致      | **高估,不可采纳**            |
| Euclidean | 可采纳、一致(偏松)         | 可采纳、一致(偏松)             |
| Octile    | 可采纳、一致             | 无障碍时精确;可采纳、一致          |

### 5.3 Manhattan 在 8 连通下为什么不行

看一个对角步就够了:真实代价 $\sqrt2$,Manhattan 给出 2,

$$
h_M = 2 > \sqrt2 = h^* ,
$$

它高估了。更精确地说,$h_M / \sqrt2$ 在 8 连通下是一致的:对角边上它的变化量
最多为 $2/\sqrt2 = \sqrt2$,恰好等于边代价;直走边上最多为 $1/\sqrt2 < 1$。因此

$$
h_M = \sqrt2 \cdot \underbrace{\left( h_M / \sqrt2 \right)}_{\text{一致}},
$$

即 Manhattan 等价于膨胀系数 $\varepsilon = \sqrt2$ 的**加权 A\***。加权 A\* 的经典
结论是:即使不重开节点,解的代价也不超过 $\varepsilon\, C^*$(Likhachev et al. 2003)。
所以它仍有一个宽松的次优界 $C \le \sqrt2\, C^*$,但**不再保证最优**。

高估的 $h$ 放大了"离目标近"的吸引力,搜索变得贪心。office500 上实测:扩展
节点从 40 945 骤降到 976,路径却比最优长 1.13 cell,超出"≤ 1 cell"的指标,而且
没有任何报错。所以 `is_admissible` 把"Manhattan + 8 连通"判为非法组合,配置
解析与 `AStarPlanner` 构造都会直接拒绝。

### 5.4 更紧的启发式扩展更少

一致启发式下,A\* 必然扩展所有 $f(n) = g^*(n) + h(n) < C^*$ 的节点,且不会扩展
$f(n) > C^*$ 的节点(Hart, Nilsson & Raphael 1968)。若 $h_1 \le h_2 \le h^*$,则

$$
\{\, n : g^*(n) + h_2(n) < C^* \,\} \subseteq \{\, n : g^*(n) + h_1(n) < C^* \,\},
$$

更紧的 $h_2$ 扩展的节点集(不计 $f = C^*$ 的平局)是 $h_1$ 的子集。8 连通下
$h_E \le h_{\text{oct}} = h^*_{8,\text{free}}$,Octile 更紧:office500 上两者给出同样的
最优路径,Octile 少扩展约 26%(40 945 对 55 398)。这是 `planner.yaml` 默认使用
Octile 的原因。

---

## 6. 8 连通的两条附加规则

### 6.1 防穿角

对角移动 $(x, y) \to (x + d_x,\, y + d_y)$ 只在两个正交邻居 $(x + d_x,\, y)$ 与
$(x,\, y + d_y)$ 都可通行时才允许。几何上,对角线段恰好穿过四个 cell 的公共
顶点;若其中一个正交 cell 是障碍,质点就会从障碍的角上"擦"过去。在膨胀后的
地图上,这意味着车体侵入安全裕度。

这条规则只**删边**,§5.2 的论证不受影响,可采纳性与一致性都保持。它会让某些
位置的真实代价严格变大:蛇形迷宫(实验报告 §3)的每个门洞都只能正交进出,
单测 `FindsExactShortestPathThroughSerpentine200x200` 的解析长度
$67\,(198 + \sqrt2) + 132$ 中的 $132 = 66 \times 2$ 就来自这里。

### 6.2 Tie-break

open 堆按 $(f, h)$ 的字典序出队:$f$ 相同时先出 $h$ 小(也就是 $g$ 大、离目标近)
的节点。§4.3 的证明只用到 $f(n') < f(n)$ 的严格情形,所以一致 $h$ 下 $f$ 相等的
节点以任何顺序扩展都不影响最优性。tie-break 是纯粹的效率手段:在 $f$ 相等的
"平台"上优先向深处推进,而不是横向铺开。

空地图对角线是最干净的例子。从 $(0, 0)$ 到 $(n-1, n-1)$,对角线上的节点
$(k, k)$ 满足

$$
f = k\sqrt2 + (n - 1 - k)\sqrt2 = (n - 1)\sqrt2 = C^* .
$$

偏离对角线一步,例如 $(k+1, k)$,$g = k\sqrt2 + 1$,剩余位移为
$(n-2-k,\ n-1-k)$:

$$
f = k\sqrt2 + 1 + \big[\sqrt2\,(n - 2 - k) + 1\big] = (n-2)\sqrt2 + 2
= C^* + (2 - \sqrt2).
$$

偏离对角线的节点 $f$ 严格大于 $C^*$(差 $2 - \sqrt2 \approx 0.586$),永远不会先于
目标出队。所以 A\* 恰好扩展 $n$ 个节点:$n = 200$ 时实测 200 个。$h \equiv 0$ 的
Dijkstra 则要扩展几乎全部 $n^2$ 个。单测 `HeuristicFocusesSearchOnOpenMap` 用
"展开数 ≤ 2n"守住这一点,这条断言不依赖构建类型,在 CI 里同样生效。

**确定性。** 出队顺序完全由输入决定(没有 RNG,也不遍历哈希表),所以同输入
得到同一条路径,`path.csv` 逐字节相同。

---

## 7. 代价梯度

`cost_weight` $= w > 0$ 时,边代价变为

$$
c'(n, m) = c(n, m) + w\, \phi\big(D(m)\big),
\qquad
\phi(d) = \min\!\big(1,\ e^{-(d - 1)}\big) \in [0, 1],
$$

$D$ 是 §3.2 的障碍距离(cell)。紧贴障碍($d = 1$)时附加代价为 $w$,每远离一个
cell 衰减到 $1/e$。

- **最优性仍然成立。** $c' \ge c$,§5 中对 $c$ 一致的 $h$ 对 $c'$ 更是一致:
  $h(n) \le c + h(m) \le c' + h(m)$。A\* 找到的是 $c'$ 意义下的最优路径。
- **"最优"的含义变了。** 被最小化的不再是长度,而是"长度 + $w$ × 贴近障碍的程度"。
  $w$ 小时主要在等长路径中挑离障碍远的一条(单测
  `CostGradientPrefersClearanceAtEqualLength`);$w$ 大时会为了远离障碍绕远路。
- `planner.yaml` 默认 $w = 0$,保持纯最短路,让最优性指标可以直接和 Dijkstra 比对。

---

## 8. 复杂度

设 $N = W \cdot H$。

- 每个 cell 至多 closed 一次;每次扩展至多压入 8 个条目,堆中条目总数为 $O(N)$;
  每次堆操作 $O(\log N)$。
- 时间 $O(N \log N)$;空间 $O(N)$:$g$ / parent / closed 三张扁平表,按行主序
  $y \cdot W + x$ 索引。

实际耗时由"展开数 × 每次堆操作的代价"共同决定(200×200,8 连通,Octile,
Release):

| 场景          | 展开节点          | 单次耗时      |
|-------------|---------------|-----------|
| 空图对角线       | 200           | ~0.26 ms  |
| 20% 随机障碍    | ~13 100(典型)   | 3.26 ms(中位) |
| 蛇形迷宫(近似最坏)  | 26 666        | ~2.7 ms(中位) |

空图对角线只展开 200 个节点,耗时主要花在初始化三张 40 000 格的表上。蛇形迷宫
展开的节点更多,耗时反而更短:走廊里的 open 集始终只有几个节点,堆操作几乎是
常数时间;随机图的搜索波前宽,堆也更大。扁平表按行主序连续存放,邻居访问的
局部性好,也省去了哈希表的哈希计算与指针追逐。

---

## 9. 栅格路径的几何局限

8 连通只允许 8 个方向,所以栅格最短路一般比平面上的真实最短路更长。沿方向角
$\theta \in [0^\circ, 45^\circ]$ 移动单位欧氏距离时,Octile 代价为
$\cos\theta + (\sqrt2 - 1)\sin\theta$,二者之比

$$
\frac{h_{\text{oct}}}{h_E} = \cos\theta + (\sqrt2 - 1)\sin\theta
$$

在 $\tan\theta = \sqrt2 - 1$(即 $\theta = 22.5^\circ$)处取最大值

$$
\sqrt{1 + (\sqrt2 - 1)^2} = \sqrt{4 - 2\sqrt2} \approx 1.082 .
$$

也就是说,8 连通路径最多比真实几何最短路长约 8.2%,并且呈"台阶"形。4 连通更差,
比值最大为 $\sqrt2$(在 45° 方向)。这是 V4 引入路径平滑或 any-angle 规划
(如 Theta\*)要解决的问题(v3_summary §8.1)。

---

## 10. 与实现的对应

| 概念                | 代码                                                  | 守护它的测试                                                                                                                          |
|-------------------|-----------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------|
| 三值占据、PGM、$y$ 翻转   | `map_io.cpp`:`classify` / `load_occupancy_grid`      | `MapIo.CorridorYAxisIsFlippedOnLoad`、`MapIo.NegateInvertsOccupancy`                                                              |
| world↔grid、越界视为占据 | `occupancy_grid.cpp`                                | `OccupancyGrid.RoundTripCellCenterIsStable`、`OccupancyGrid.OutOfBoundsReadsAsOccupied`                                         |
| 欧氏距离变换、膨胀         | `inflation.cpp`:`obstacle_distance_cells` / `inflate` | `Inflation.ObstacleDistanceIsEuclideanToNearest`、`Inflation.MarksDiagonalNeighborsAtRadiusOnePointFive`                         |
| A\*、惰性删除、首次出队即最优  | `astar.cpp`:`AStarPlanner::plan`                     | `AStar.FindsKnownShortestPathAroundWall`、`AStar.FindsExactShortestPathThroughSerpentine200x200`                                  |
| 启发式与可采纳性规则        | `astar.cpp`:`heuristic`;`grid_types.ixx`:`is_admissible` | `AStar.AdmissibleHeuristicsAgreeOnOptimalLength(EightConnected)`、`AStar.RejectsInadmissibleManhattanUnderEightConnectivity`、`PlannerConfig.RejectsManhattanUnlessFourConnected` |
| 防穿角               | `astar.cpp` 邻居生成                                    | `AStar.DiagonalBlockedAtObstacleCorner`、`AStar.DiagonalAllowedWhenBothCornersFree`                                               |
| tie-break、展开数、确定性  | `astar.cpp`:`NodeWorse`                              | `AStar.HeuristicFocusesSearchOnOpenMap`、`AStar.DeterministicAcrossRuns`                                                          |
| 代价梯度              | `astar.cpp`:`compute_gradient_cost`                  | `AStar.CostGradientPrefersClearanceAtEqualLength`                                                                                 |

---

## 参考文献

1. P. E. Hart, N. J. Nilsson, B. Raphael. *A Formal Basis for the Heuristic
   Determination of Minimum Cost Paths.* IEEE Transactions on Systems Science and
   Cybernetics, 1968.
2. J. Pearl. *Heuristics: Intelligent Search Strategies for Computer Problem
   Solving.* Addison-Wesley, 1984.
3. M. Likhachev, G. Gordon, S. Thrun. *ARA\*: Anytime A\* with Provable Bounds on
   Sub-Optimality.* NIPS, 2003.
4. S. Russell, P. Norvig. *Artificial Intelligence: A Modern Approach*, Ch. 3
   (图搜索版 A\* 的最优性需要一致性).
5. S. M. LaValle. *Planning Algorithms*, Ch. 4(配置空间). Cambridge University
   Press, 2006.
6. P.-E. Danielsson. *Euclidean Distance Mapping.* Computer Graphics and Image
   Processing, 1980.
7. P. F. Felzenszwalb, D. P. Huttenlocher. *Distance Transforms of Sampled
   Functions.* Theory of Computing, 2012.
8. A. Nash, K. Daniel, S. Koenig, A. Felner. *Theta\*: Any-Angle Path Planning on
   Grids.* AAAI, 2007.
9. ROS 2 Navigation:`costmap_2d` 膨胀层、`nav2_core::GlobalPlanner` 接口。
