# 卡尔曼滤波（KF）与扩展卡尔曼滤波（EKF）算法


> **适用版本**：MiniNav V2  
> **参考文献**：Thrun, Burgard, Fox. *Probabilistic Robotics*, Ch. 1.1 — 1.3

---

## 目录

1. [概率论基础](#1-概率论基础)
2. [高斯分布](#2-高斯分布)
3. [矩阵微积分预备](#3-矩阵微积分预备)
4. [机器人状态估计基础](#4-机器人状态估计基础)
5. [贝叶斯滤波器（Bayes Filter）](#5-贝叶斯滤波器bayes-filter)
6. [卡尔曼滤波器（Kalman Filter）](#6-卡尔曼滤波器kalman-filter)
7. [卡尔曼滤波器的数学推导](#7-卡尔曼滤波器的数学推导)
8. [扩展卡尔曼滤波器（EKF）](#8-扩展卡尔曼滤波器ekf)
9. [卡方分布与一致性检验](#9-卡方分布与一致性检验)

---

## 1. 概率论基础

### 1.1 条件概率与贝叶斯定理

**条件概率公式：**

$$
p(x \mid y) = \frac{p(x, y)}{p(y)}
$$

若 $x, y$ 相互独立，则 $p(x \mid y) = p(x)$。

**全概率公式：**

$$
p(x) = \sum_y p(x \mid y) p(y), \qquad p(x) = \int p(x \mid y) p(y) \, dy
$$

**贝叶斯定理：**

$$
p(x \mid y) = \frac{p(y \mid x)\, p(x)}{p(y)} = \frac{p(y \mid x)\, p(x)}{\sum_{x'} p(y \mid x')\, p(x')}
$$

连续形式：

$$
p(x \mid y) = \frac{p(y \mid x)\, p(x)}{\int p(y \mid x')\, p(x')\, dx'}
$$

**先验与后验的含义：**
若 $x$ 是希望从 $y$ 推断出的量，则 $p(x)$ 称为**先验概率分布**，$y$ 称为数据。该分布刻画了引入 $y$ 数据之前，我们关于 $x$ 的知识。条件概率 $p(x \mid y)$ 被称为 **$x$ 的后验分布**。Bayes 定理给出了先验分布与后验分布的转换关系。

**归一化因子 $\eta$：**

由于 $p(y)^{-1}$ 不依赖于 $x$，在贝叶斯公式中通常将其记作归一化常数 $\eta$：

$$
p(x \mid y) = \eta\, p(y \mid x)\, p(x)
$$

> 笔记中 $\eta$、$\eta'$、$\eta''$ 等均用于表示某种归一化常数（其具体数值由要求 $\int p(x \mid y) dx = 1$ 隐式确定）。

**条件化的 Bayes 公式**（在某个额外变量 $z$ 已知的条件下）：

$$
p(x \mid y, z) = \frac{p(y \mid x, z)\, p(x \mid z)}{p(y \mid z)}
$$

每一项都加上了 "$|z$"。

**证明：** 由条件概率公式

$$
p(x \mid y, z) = \frac{p(x, y, z)}{p(y, z)}
$$

而

$$
\begin{aligned}
p(x, y, z) &= p(y \mid x, z) \cdot p(x, z) = p(y \mid x, z)\, p(x \mid z)\, p(z) \\
p(y, z) &= p(y \mid z)\, p(z)
\end{aligned}
$$

代入即得证。

### 1.2 条件独立

**两个随机变量独立：**

$$
p(x, y) = p(x)\, p(y)
$$

**在 $z$ 已知的条件下 $x, y$ 条件独立：**

$$
p(x, y \mid z) = p(x \mid z)\, p(y \mid z)
$$

即一旦 $z$ 已知，$x$ 和 $y$ 之间便没有任何相互信息了，而 $x, y$ 本身不必绝对独立。

等价形式：

$$
p(x \mid z) = p(x \mid z, y), \qquad p(y \mid z) = p(y \mid z, x)
$$

> **注意：** 反过来，如果 $x, y$ 独立，在 $z$ 条件已知下也不一定独立：
> $$ p(x, y) = p(x)\,p(y) \;\not\Rightarrow\; p(x, y \mid z) = p(x \mid z)\, p(y \mid z) $$

### 1.3 期望、方差与协方差

**期望：**

$$
E[X] = \sum_x x\, p(x), \qquad E[X] = \int x\, p(x)\, dx
$$

**期望的线性性：**

$$
E[aX + b] = a\, E[X] + b
$$

**方差（一维）：**

$$
\mathrm{Cov}[X] = E\!\left[(X - E[X])^2\right] = E[X^2] - E[X]^2 = \mathrm{Var}[X]
$$

**两变量协方差：**

$$
\mathrm{Cov}[X, Y] = E\!\left[(X - \mu_X)(Y - \mu_Y)\right] = E[XY] - E[X]\, E[Y]
$$

**多维随机变量（随机向量）的协方差矩阵：**

$$
\Sigma = \mathrm{Cov}[\mathbf{X}] = E\!\left[(\mathbf{X} - \mathbf{\mu})(\mathbf{X} - \mathbf{\mu})^\top\right] = E[\mathbf{X}\mathbf{X}^\top] - \mathbf{\mu}\mathbf{\mu}^\top
$$

### 1.4 熵

$$
H_p(x) = E[-\log_2 p(x)]
$$

离散与连续形式：

$$
H_p(x) = -\sum_x p(x)\log_2 p(x), \qquad H_p(x) = -\int p(x)\log_2 p(x)\, dx
$$

---

## 2. 高斯分布

### 2.1 高斯分布概率密度函数

**一维高斯分布 PDF（概率密度函数）：**

$$
p(x) = \frac{1}{\sigma\sqrt{2\pi}} \, \exp\!\left(-\frac{(x-\mu)^2}{2\sigma^2}\right)
$$

通常简写为 $\mathcal{N}(x;\,\mu,\,\sigma^2)$。

**多元高斯分布 PDF：**

$$
p(\mathbf{x}) = \frac{1}{\sqrt{(2\pi)^n \det(\Sigma)}} \, \exp\!\left(-\tfrac{1}{2}(\mathbf{x}-\mathbf{\mu})^\top \Sigma^{-1} (\mathbf{x}-\mathbf{\mu})\right)
$$

其中 $\mathbf{\mu}$ 为期望向量，$\Sigma$ 为协方差矩阵。

### 2.2 多元高斯分布的马氏距离

在概率分布的语境下,"距离"不是几何上的远近,而是某一个结果出现的**惊讶程度**，即这个点在这个分布下出现得有多罕见。

朴素欧氏距离 $\|x - \mu\|^2 = (x - \mu)^T (x - \mu)$ 有两个致命问题:

1. **量纲不同**。EKF 状态向量里有 $p_x$ (米)、$\theta$ (弧度)、$v$ (米每秒)——直接平方相加在数学和物理上都没有意义。
2. **维度间相关**。$\Sigma$ 的非对角元素描述维度之间的相关性,欧氏距离完全忽略这种结构——它隐含假设 $\Sigma = I$,而这几乎从来都不成立。

因此在概率分布上引入了马氏距离（Mahalanobis distance）：

$$\boxed{d_M^2(x) = (x - \mu)^T \, \Sigma^{-1} \, (x - \mu)}$$

马氏距离根据 $\Sigma$ 的结构对不同维度的距离进行**加权**，使得在高方差的方向上距离被压缩，在低方差的方向上距离被放大。
使用$\Sigma^{-1}$ 某个方向方差大,意味着分布在那个方向"延伸得远",所以在那个方向上走远一点相对来讲就并不奇怪—，在该方向的距离应该被压缩。
反之，方差小的方向上走远一点就很奇怪了，所以距离应该被放大。

### 2.3 白化（Whitening）

**白化 (whitening)** 即通过线性变换把有相关性的随机向量变成"白噪声式"向量(各分量不相关、方差为 1)。
该名字的来自"白噪声"，白噪声是各时刻不相关、方差恒定的随机过程，它的功率谱密度像白光一样在所有频率上平坦,所以叫"白"。

给定 $x \sim \mathcal{N}(\mu, \Sigma)$,寻找线性变换 $T$ 使得
$$y = T (x - \mu), \quad y \sim \mathcal{N}(0, I)$$

#### 2.3.1 Cholesky 白化

对 $\Sigma$ 做 Cholesky 分解 $\Sigma = L L^T$(其中 $L$ 是下三角),定义:
$$y = L^{-1} (x - \mu)$$
则 $y$ 的协方差为:
$$\Sigma_y = L^{-1} (L L^T) (L^{-1})^T = (L^{-1} L)(L^T L^{-T}) = I \cdot I = I$$
因此 $y$ 是一个白化后的随机向量。

实际上 $\Sigma$ 的平方根不唯一,还有:

- 对称平方根 $\Sigma^{1/2} = V \Lambda^{1/2} V^T$
- ZCA 白化、PCA 白化

都能把 $\Sigma$ 变成 $I$,只是结果 $y$ 在 $\mathbb{R}^n$ 里被旋转到不同方向。Cholesky 的优势是**数值计算快**(三角分解 $O(n^3/3)$)且**天然支持顺序求解**——square-root filter 利用这点保持 $P$ 始终以 Cholesky 因子形式存储。

#### 2.3.2 白化的几何意义

白化变换 $y = L^{-1}(x - \mu)$ 是三件事的复合:

1. **平移**:$x - \mu$ 把椭球中心搬到原点
2. **旋转**:$L^{-1}$ 隐含旋转,把椭球主轴对齐坐标轴
3. **缩放**:沿每个对齐后的主轴,把半轴 $\sqrt{\lambda_i}$ 压缩到 1

合起来:**椭球被搬到原点、旋转到主轴对齐、压缩成单位球**。
即任何 $\mathcal{N}(\mu, \Sigma)$ 都可看成从 $\mathcal{N}(0, I)$ 出发,沿主轴拉伸($L$)、加上偏移($\mu$)得到。白化和这个生成过程互为逆变换。

#### 2.3.3 白化与马氏距离

白化后的 $y$ 的欧氏距离 $\|y\|^2$ 就是原始 $x$ 的马氏距离:
$$\|y\|^2 = y^T y = (x - \mu)^T L^{-T} L^{-1} (x - \mu) = (x - \mu)^T \Sigma^{-1} (x - \mu) = d_M^2(x)$$
因此白化可以看成把马氏距离转化为欧氏距离的过程。

## 3. 矩阵微积分预备

### 3.1 求导约定：分母布局

在涉及向量/矩阵求导时，存在两套主流约定：**分子布局**（numerator layout）与**分母布局**（denominator layout），二者在所得矩阵的形状上互为转置。
本笔记采用 **分母布局**（与 Thrun 等《Probabilistic Robotics》一致）：

- 标量对列向量求导得到 **列向量**
- 列向量对列向量求导得到一个矩阵，其行数与分母维度对应

### 3.2 标量对向量的梯度

设 $f: \mathbb{R}^n \to \mathbb{R}$，$\mathbf{x} \in \mathbb{R}^n$，则梯度为列向量：

$$
\frac{\partial f}{\partial \mathbf{x}} = \begin{bmatrix} \dfrac{\partial f}{\partial x_1} \\[4pt] \dfrac{\partial f}{\partial x_2} \\[4pt] \vdots \\[4pt] \dfrac{\partial f}{\partial x_n} \end{bmatrix} \in \mathbb{R}^n
$$

### 2.3 雅可比矩阵：向量对向量的导数

**定义：** 设 $\mathbf{y} = f(\mathbf{x})$，其中 $f: \mathbb{R}^n \to \mathbb{R}^m$，$\mathbf{x} \in \mathbb{R}^n$，$\mathbf{y} \in \mathbb{R}^m$。则 $f$ 在 $\mathbf{x}$ 处的**雅可比矩阵**为：

$$
J = \frac{\partial \mathbf{y}}{\partial \mathbf{x}} = \begin{bmatrix}
\dfrac{\partial y_1}{\partial x_1} & \dfrac{\partial y_1}{\partial x_2} & \cdots & \dfrac{\partial y_1}{\partial x_n} \\[6pt]
\dfrac{\partial y_2}{\partial x_1} & \dfrac{\partial y_2}{\partial x_2} & \cdots & \dfrac{\partial y_2}{\partial x_n} \\[6pt]
\vdots & \vdots & \ddots & \vdots \\[6pt]
\dfrac{\partial y_m}{\partial x_1} & \dfrac{\partial y_m}{\partial x_2} & \cdots & \dfrac{\partial y_m}{\partial x_n}
\end{bmatrix} \in \mathbb{R}^{m \times n}
$$

即 $J_{ij} = \dfrac{\partial y_i}{\partial x_j}$。

Bayes Filter 的递推中需要把高斯分布通过运动 / 观测模型传递。**线性变换作用在高斯分布上仍然是高斯分布，非线性变换则不是**。EKF 用雅可比矩阵把非线性变换近似成线性变换，从而能继续用高斯参数化的方式做滤波。

### 3.4 常用求导规则

#### 3.4.1 仿射函数的雅可比

若 $\mathbf{y} = C\mathbf{x} + \mathbf{d}$，则

$$
\frac{\partial \mathbf{y}}{\partial \mathbf{x}} = C
$$

这是 KF 中 $A_t, B_t, C_t$ 对应的形式——线性系统的雅可比就是矩阵本身，因此 KF 不需要 Taylor 展开。

#### 3.4.2 链式法则

若 $\mathbf{z} = g(\mathbf{y})$，$\mathbf{y} = f(\mathbf{x})$，则在分母布局下：

$$
\frac{\partial \mathbf{z}}{\partial \mathbf{x}} = \frac{\partial \mathbf{y}}{\partial \mathbf{x}} \cdot \frac{\partial \mathbf{z}}{\partial \mathbf{y}}
$$

针对**标量** $z$ 的情形，可写作 $\dfrac{\partial z}{\partial \mathbf{x}} = \left(\dfrac{\partial \mathbf{y}}{\partial \mathbf{x}}\right)^\top \dfrac{\partial z}{\partial \mathbf{y}}$。

#### 3.4.3 二次型的梯度

设 $M \in \mathbb{R}^{n\times n}$，$\mathbf{x} \in \mathbb{R}^n$：

$$
\frac{\partial}{\partial \mathbf{x}}(\mathbf{x}^\top M \mathbf{x}) = (M + M^\top)\mathbf{x}
$$

若 $M$ 对称（协方差矩阵的逆即满足这一条件），则：

$$
\frac{\partial}{\partial \mathbf{x}}\!\left(\tfrac{1}{2}\mathbf{x}^\top M \mathbf{x}\right) = M\mathbf{x}
$$

**证明：** $\mathbf{x}^\top M \mathbf{x} = \sum_i \sum_j M_{ij} x_i x_j$。对 $x_k$ 求偏导：

$$
\frac{\partial}{\partial x_k}\mathbf{x}^\top M \mathbf{x} = \sum_j M_{kj} x_j + \sum_i M_{ik} x_i = [(M + M^\top)\mathbf{x}]_k
$$

#### 3.4.4 双线性型对其中一个向量的导数

若 $f = \mathbf{x}^\top A \mathbf{y}$，则

$$
\frac{\partial f}{\partial \mathbf{x}} = A\mathbf{y}, \qquad \frac{\partial f}{\partial \mathbf{y}} = A^\top \mathbf{x}
$$

#### 3.4.5 中心化二次型的梯度

若 $f = \tfrac{1}{2}(\mathbf{x} - \mathbf{a})^\top M (\mathbf{x} - \mathbf{a})$，$M$ 对称，则：

$$
\frac{\partial f}{\partial \mathbf{x}} = M(\mathbf{x} - \mathbf{a})
$$

### 3.5 一阶 Taylor 展开

对于光滑函数 $f: \mathbb{R}^n \to \mathbb{R}^m$，在 $\mathbf{x}_0$ 处的一阶 Taylor 展开为：

$$
f(\mathbf{x}) \approx f(\mathbf{x}_0) + J(\mathbf{x}_0)\,(\mathbf{x} - \mathbf{x}_0)
$$

其中 $J(\mathbf{x}_0)$ 是 $f$ 在 $\mathbf{x}_0$ 处的雅可比矩阵。通过雅可比矩阵把非线性映射 $f$ 在 $\mathbf{x}$ 处**局部线性化**——它是 $f$ 在该点的**最佳一阶线性近似**。

---

## 4. 机器人状态估计基础

### 4.1 机器人的两个重要概率

- **状态转移模型概率：** $p(x_t \mid x_{t-1}, u_t)$
- **观测概率（测量模型）：** $p(z_t \mid x_t)$

### 4.2 信念（Belief）

信念反映的是机器人对环境状态的**内部认识**：

$$
\mathrm{bel}(x_t) = p(x_t \mid z_{1:t}, u_{1:t})
$$

在纳入观测 $z_t$ 之前（有时这也会发生），我们也需要计算刚执行完控制 $u_t$ 后的**先验信念**：

$$
\overline{\mathrm{bel}}(x_t) = p(x_t \mid z_{1:t-1}, u_{1:t})
$$

在滤波器语境中：

- $\overline{\mathrm{bel}}(x_t)$ 通常被称为 **预测（prediction）**；
- 由 $\overline{\mathrm{bel}}(x_t)$ 计算 $\mathrm{bel}(x_t)$ 的过程称为 **校正（correction）** 或 **观测更新（measurement update）**。

---

## 5. 贝叶斯滤波器（Bayes Filter）

### 5.1 算法描述

Bayes Filter 通过控制 $u_t$ 和测量 $z_t$ 来计算环境状态的信念 $\mathrm{bel}(x_t)$。

**Algorithm Bayes_Filter($\mathrm{bel}(x_{t-1}), u_t, z_t$):**

```
1: for all x_t do
2:    bel_bar(x_t) = ∫ p(x_t | u_t, x_{t-1}) bel(x_{t-1}) dx_{t-1}    # 全概率公式（预测）
3:    bel(x_t)     = η p(z_t | x_t) bel_bar(x_t)                       # 贝叶斯公式（测量更新）
4: endfor
5: return bel(x_t)
```

数学表达：

$$
\overline{\mathrm{bel}}(x_t) = \int p(x_t \mid u_t, x_{t-1})\, \mathrm{bel}(x_{t-1})\, dx_{t-1} \quad \text{（预测）}
$$

$$
\mathrm{bel}(x_t) = \eta\, p(z_t \mid x_t)\, \overline{\mathrm{bel}}(x_t) \quad \text{（测量更新）}
$$

> Bayes Filter 是一个抽象框架，KF/EKF/PF 等都是它在不同分布假设下的具体实现。Bayes Filter 与 HMM（隐马尔可夫模型）在结构上是对应的，KF/EKF 是其在线性/弱非线性高斯情形下的解析解。

---

## 6. 卡尔曼滤波器（Kalman Filter）

### 6.1 概述

Kalman Filter 是 Bayes Filter 的一种**实现方式**，用于在**线性高斯系统**中运行滤波和预测。Kalman Filter 用矩参数化表示信念：后验是高斯分布，用 $\mu$ 和 $\Sigma$ 就可以描述。

### 6.2 三个核心假设

Kalman Filter 要保持后验 $\mathrm{bel}(x_t)$ 始终为高斯分布，需要满足以下三个假设：

**假设 1：状态转移概率是线性的并加入高斯噪声**

$$
x_t = A_t x_{t-1} + B_t u_t + \varepsilon_t
$$

其中：
- $x_t$ 为状态向量，$u_t$ 为控制向量；
- $A_t$ 为 $n \times n$ 矩阵，$B_t$ 为 $n \times m$ 矩阵（$m$ 为控制向量维度）；
- $\varepsilon_t$ 是随机高斯向量，刻画状态转移概率中的不确定性，期望为 $0$，协方差记为 $R_t$，即 $p(\varepsilon_t) = \mathcal{N}(0, R_t)$。

后验状态依然是一个高斯分布，其均值由 $A_t x_{t-1} + B_t u_t$ 给出，协方差由 $R_t$ 给出：

$$
p(x_t \mid x_{t-1}, u_t) = \det(2\pi R_t)^{-\frac{1}{2}} \exp\!\left\{-\tfrac{1}{2}(x_t - A_t x_{t-1} - B_t u_t)^\top R_t^{-1} (x_t - A_t x_{t-1} - B_t u_t)\right\}
$$

**假设 2：测量也是线性的并加入高斯噪声**

$$
z_t = C_t x_t + \delta_t
$$

其中 $C_t$ 为 $k \times n$ 矩阵（$k$ 为 $z_t$ 向量维度），$\delta_t$ 描述测量噪声，符合多元高斯分布，期望为 $0$，协方差为 $Q_t$。

测量的概率分布符合如下形式：

$$
p(z_t \mid x_t) = \det(2\pi Q_t)^{-\frac{1}{2}} \exp\!\left\{-\tfrac{1}{2}(z_t - C_t x_t)^\top Q_t^{-1} (z_t - C_t x_t)\right\}
$$

**假设 3：初始信念也必须是高斯分布**

$$
\mathrm{bel}(x_0) = p(x_0) = \det(2\pi \Sigma_0)^{-\frac{1}{2}} \exp\!\left\{-\tfrac{1}{2}(x_0 - \mu_0)^\top \Sigma_0^{-1} (x_0 - \mu_0)\right\}
$$

> 上面这三条假设条件足以确保在任何时间 $t$，后验 $\mathrm{bel}(x_t)$ 始终为高斯分布。

### 6.3 卡尔曼滤波器算法（线性高斯情形）

**Algorithm Kalman_filter($\mu_{t-1}, \Sigma_{t-1}, u_t, z_t$):**

```
1: μ_bar_t   = A_t μ_{t-1} + B_t u_t                              # 预测均值
2: Σ_bar_t   = A_t Σ_{t-1} A_t^T + R_t                             # 预测协方差
3: K_t       = Σ_bar_t C_t^T (C_t Σ_bar_t C_t^T + Q_t)^{-1}        # 卡尔曼增益
4: μ_t       = μ_bar_t + K_t (z_t - C_t μ_bar_t)                   # 更新均值
5: Σ_t       = (I - K_t C_t) Σ_bar_t                               # 更新协方差
6: return μ_t, Σ_t
```

数学表达：

$$
\begin{aligned}
\bar\mu_t &= A_t \mu_{t-1} + B_t u_t \\
\bar\Sigma_t &= A_t \Sigma_{t-1} A_t^\top + R_t \\
K_t &= \bar\Sigma_t C_t^\top (C_t \bar\Sigma_t C_t^\top + Q_t)^{-1} \quad \text{（卡尔曼增益）}\\
\mu_t &= \bar\mu_t + K_t (z_t - C_t \bar\mu_t) \\
\Sigma_t &= (I - K_t C_t) \bar\Sigma_t
\end{aligned}
$$

> 第 5 步的协方差更新使用了标准形式，浮点累积会破坏对称性；工程实现中应改用 **Joseph 形式** $\Sigma_t = (I - K_t C_t) \bar\Sigma_t (I - K_t C_t)^\top + K_t Q_t K_t^\top$，从结构上保证对称正半定。

---

## 7. 卡尔曼滤波器的数学推导

> 目标：从 Bayes Filter 的两步公式出发，推导得出第 4.3 节的 5 个递推式。

### 7.1 预测步推导

**起点：** Bayes Filter 的预测步

$$
\overline{\mathrm{bel}}(x_t) = \int p(x_t \mid x_{t-1}, u_t)\, p(x_{t-1} \mid z_{1:t-1}, u_{1:t-1})\, dx_{t-1} = \int p(x_t \mid x_{t-1}, u_t)\, \mathrm{bel}(x_{t-1})\, dx_{t-1}
$$

其中 $\mathrm{bel}(x_{t-1})$ 是以 $\mu_{t-1}$ 为期望、协方差为 $\Sigma_{t-1}$ 的高斯分布，转移概率由前文给出：

$$
p(x_t \mid x_{t-1}, u_t) \sim \mathcal{N}(x_t;\, A_t x_{t-1} + B_t u_t,\, R_t)
$$

$$
= \det(2\pi R_t)^{-\frac{1}{2}} \exp\!\left\{-\tfrac{1}{2}(x_t - A_t x_{t-1} - B_t u_t)^\top R_t^{-1} (x_t - A_t x_{t-1} - B_t u_t)\right\}
$$

而

$$
\mathrm{bel}(x_{t-1}) \sim \mathcal{N}(x_{t-1};\, \mu_{t-1},\, \Sigma_{t-1}) = \det(2\pi\Sigma_{t-1})^{-\frac{1}{2}} \exp\!\left\{-\tfrac{1}{2}(x_{t-1} - \mu_{t-1})^\top \Sigma_{t-1}^{-1} (x_{t-1} - \mu_{t-1})\right\}
$$

将两者乘起来：

$$
\overline{\mathrm{bel}}(x_t) = \eta \int \exp\!\left\{-\tfrac{1}{2}(x_t - A_t x_{t-1} - B_t u_t)^\top R_t^{-1} (x_t - A_t x_{t-1} - B_t u_t) - \tfrac{1}{2}(x_{t-1} - \mu_{t-1})^\top \Sigma_{t-1}^{-1} (x_{t-1} - \mu_{t-1})\right\} dx_{t-1}
$$

### 7.2 二次型分解

设：

$$
L_t = \tfrac{1}{2}(x_t - A_t x_{t-1} - B_t u_t)^\top R_t^{-1} (x_t - A_t x_{t-1} - B_t u_t) + \tfrac{1}{2}(x_{t-1} - \mu_{t-1})^\top \Sigma_{t-1}^{-1} (x_{t-1} - \mu_{t-1})
$$

则

$$
\overline{\mathrm{bel}}(x_t) = \eta \int \exp\{-L_t\}\, dx_{t-1}
$$

由于 $L_t$ 是 $x_t$ 和 $x_{t-1}$ 的二次型，积分变量为 $x_{t-1}$，我们可以把 $L_t$ 看作关于 $x_{t-1}$ 的二次型，把 $x_t$ 看作系数里的参数。展开 $L_t$ 中的两个二次型，并按 $x_{t-1}$ 的幂次归类：

$$
L_t = \tfrac{1}{2} x_{t-1}^\top M\, x_{t-1} - x_{t-1}^\top b(x_t) + C(x_t)
$$

**目标：** 把 $L_t$ 分解为两部分：

$$
L_t = L_t(x_{t-1}, x_t) + L_t(x_t)
$$

这一过程可以通过**配方**完成。然后：

$$
\overline{\mathrm{bel}}(x_t) = \eta \int \exp(-L_t)\, dx_{t-1} = \eta \exp\{-L_t(x_t)\} \int \exp\{-L_t(x_{t-1}, x_t)\}\, dx_{t-1}
$$

将 $\exp\{-L_t(x_t)\}$ 拆出来后，剩下的积分会被吸收进归一化常数 $\eta$。


### 7.3 寻找关于 $x_{t-1}$ 的二次函数

计算 $L_t$ 关于 $x_{t-1}$ 的前两个导数：

$$
\frac{\partial L_t}{\partial x_{t-1}} = -A_t^\top R_t^{-1}(x_t - A_t x_{t-1} - B_t u_t) + \Sigma_{t-1}^{-1}(x_{t-1} - \mu_{t-1})
$$

$$
\frac{\partial^2 L_t}{\partial x_{t-1}^2} = A_t^\top R_t^{-1} A_t + \Sigma_{t-1}^{-1} =: \Psi_t^{-1}
$$

$\Psi_t$ 定义了 $L_t(x_{t-1}, x_t)$ 部分的曲率。通过将一次导数置 0，可以得到中心点（即抛物面最低点）。

> **扩展：** 任何二次型（半正定）都可以写成标准形式：
> $$L_t(x_{t-1}, x_t) = \tfrac{1}{2}(x_{t-1} - c)^\top M (x_{t-1} - c) + k$$
> 完全由 $M$（曲率矩阵）、$c$（中心点）和 $k$（常数）所确定。

**求解中心点（令一次导数为 0）：**

$$
A_t^\top R_t^{-1}(x_t - B_t u_t) - A_t^\top R_t^{-1} A_t x_{t-1} = -\Sigma_{t-1}^{-1}(x_{t-1} - \mu_{t-1})
$$

$$
(A_t^\top R_t^{-1} A_t + \Sigma_{t-1}^{-1}) x_{t-1} = A_t^\top R_t^{-1}(x_t - B_t u_t) + \Sigma_{t-1}^{-1} \mu_{t-1} = \Psi_t^{-1} x_{t-1}
$$

$$
\boxed{\;x_{t-1}^* = \Psi_t\!\left[A_t^\top R_t^{-1}(x_t - B_t u_t) + \Sigma_{t-1}^{-1} \mu_{t-1}\right]\;}
$$

因此

$$
L_t(x_{t-1}, x_t) = \tfrac{1}{2}\!\left(x_{t-1} - \Psi_t\!\left[A_t^\top R_t^{-1}(x_t - B_t u_t) + \Sigma_{t-1}^{-1} \mu_{t-1}\right]\right)^\top \Psi_t^{-1}\!\left(x_{t-1} - \Psi_t\!\left[A_t^\top R_t^{-1}(x_t - B_t u_t) + \Sigma_{t-1}^{-1} \mu_{t-1}\right]\right)
$$

### 7.4 回到原积分式

$$
\overline{\mathrm{bel}}(x_t) = \eta \exp\{-L_t(x_t)\} \int \det(2\pi\Psi_t)^{\frac{1}{2}} \det(2\pi\Psi_t)^{-\frac{1}{2}} \exp\{-L_t(x_{t-1}, x_t)\}\, dx_{t-1}
$$

积分项内的部分是一个完整的高斯密度，对 $x_{t-1}$ 积分为 1，因此

$$
\overline{\mathrm{bel}}(x_t) = \eta \exp\{-L_t(x_t)\} \cdot \det(2\pi\Psi_t)^{\frac{1}{2}} = \eta'\exp\{-L_t(x_t)\}
$$

### 7.5 计算 $L_t(x_t)$

由 $L_t(x_t) = L_t - L_t(x_{t-1}, x_t)$，展开两侧并整理得：

$$
\begin{aligned}
L_t(x_t) &= \tfrac{1}{2} x_{t-1}^\top A_t^\top R_t^{-1} A_t\, x_{t-1} - x_{t-1}^\top A_t^\top R_t^{-1}(x_t - B_t u_t) + \tfrac{1}{2}(x_t - B_t u_t)^\top R_t^{-1}(x_t - B_t u_t) \\
&\quad + \tfrac{1}{2} x_{t-1}^\top \Sigma_{t-1}^{-1} x_{t-1} - x_{t-1}^\top \Sigma_{t-1}^{-1} \mu_{t-1} + \tfrac{1}{2} \mu_{t-1}^\top \Sigma_{t-1}^{-1} \mu_{t-1} \\
&\quad - \tfrac{1}{2} x_{t-1}^\top (A_t^\top R_t^{-1} A_t + \Sigma_{t-1}^{-1}) x_{t-1} + x_{t-1}^\top \!\left[A_t^\top R_t^{-1}(x_t - B_t u_t) + \Sigma_{t-1}^{-1}\mu_{t-1}\right] \\
&\quad - \tfrac{1}{2}\!\left[A_t^\top R_t^{-1}(x_t - B_t u_t) + \Sigma_{t-1}^{-1}\mu_{t-1}\right]^\top (A_t^\top R_t^{-1} A_t + \Sigma_{t-1}^{-1})^{-1}\!\left[A_t^\top R_t^{-1}(x_t - B_t u_t) + \Sigma_{t-1}^{-1}\mu_{t-1}\right]
\end{aligned}
$$

下划线项相互抵消，最终化简为：

$$
L_t(x_t) = \tfrac{1}{2}(x_t - B_t u_t)^\top R_t^{-1}(x_t - B_t u_t) + \tfrac{1}{2}\mu_{t-1}^\top \Sigma_{t-1}^{-1}\mu_{t-1} - \tfrac{1}{2}\!\left[A_t^\top R_t^{-1}(x_t - B_t u_t) + \Sigma_{t-1}^{-1}\mu_{t-1}\right]^\top (A_t^\top R_t^{-1} A_t + \Sigma_{t-1}^{-1})^{-1}\!\left[A_t^\top R_t^{-1}(x_t - B_t u_t) + \Sigma_{t-1}^{-1}\mu_{t-1}\right]
$$

### 7.6 对 $L_t(x_t)$ 求 $x_t$ 的偏导

$$
\frac{\partial L_t(x_t)}{\partial x_t} = R_t^{-1}(x_t - B_t u_t) - R_t^{-1} A_t (A_t^\top R_t^{-1} A_t + \Sigma_{t-1}^{-1})^{-1}\!\left[A_t^\top R_t^{-1}(x_t - B_t u_t) + \Sigma_{t-1}^{-1}\mu_{t-1}\right]
$$

$$
= \!\left[R_t^{-1} - R_t^{-1} A_t (A_t^\top R_t^{-1} A_t + \Sigma_{t-1}^{-1})^{-1} A_t^\top R_t^{-1}\right](x_t - B_t u_t) - R_t^{-1} A_t (A_t^\top R_t^{-1} A_t + \Sigma_{t-1}^{-1})^{-1} \Sigma_{t-1}^{-1} \mu_{t-1}
$$

> **引理：** 对于可逆矩阵 $R, Q$ 与任意矩阵 $P$，以下关系成立：
> $$(R + P Q P^\top)^{-1} = R^{-1} - R^{-1} P (Q^{-1} + P^\top R^{-1} P)^{-1} P^\top R^{-1}$$
> 
> **证明：** 记 $\Psi = (Q^{-1} + P^\top R^{-1} P)^{-1}$，验证：$$
\begin{aligned}
(R + P Q P^\top)\left[R^{-1} - R^{-1} P \Psi P^\top R^{-1}\right]
&= I - P\Psi P^\top R^{-1} + PQP^\top R^{-1} - PQP^\top R^{-1} P \Psi P^\top R^{-1}\\
&= I + (PQ - P\Psi + PQP^\top R^{-1}\Psi - PQ \cdot Q^{-1}\Psi + PQ \cdot Q^{-1}\Psi) P^\top R^{-1}\\
&= I + P\!\left[Q - QQ^{-1}\Psi + QP^\top R^{-1}\Psi - \Psi + Q\cdot Q^{-1}\Psi\right] P^\top R^{-1}\\
&= I + PQ\!\left(I - [Q^{-1} - P^\top R^{-1} P]\Psi\right) Q^{-1}\Psi P^\top R^{-1}\\
&\quad\text{(注意 }Q^{-1} + P^\top R^{-1}P = \Psi^{-1}\text{，故 }Q^{-1} - P^\top R^{-1}P\text{ 替换……整理得 )}\\
&= I + PQ(I - \Psi^{-1}\Psi) P^\top R^{-1} = I
\end{aligned}
$$

对 $\dfrac{\partial L_t(x_t)}{\partial x_t}$ 中的方括号项应用引理（取 $R \to R_t,\, P \to A_t,\, Q \to \Sigma_{t-1}$）：

$$
R_t^{-1} - R_t^{-1} A_t (A_t^\top R_t^{-1} A_t + \Sigma_{t-1}^{-1})^{-1} A_t^\top R_t^{-1} = (R_t + A_t \Sigma_{t-1} A_t^\top)^{-1}
$$

于是

$$
\frac{\partial L_t(x_t)}{\partial x_t} = (R_t + A_t \Sigma_{t-1} A_t^\top)^{-1}(x_t - B_t u_t) - R_t^{-1} A_t (A_t^\top R_t^{-1} A_t + \Sigma_{t-1}^{-1})^{-1} \Sigma_{t-1}^{-1} \mu_{t-1}
$$

### 7.7 得到预测均值与协方差

令 $\dfrac{\partial L_t(x_t)}{\partial x_t} = 0$，整理：

$$
\begin{aligned}
x_t &= B_t u_t + (R_t + A_t \Sigma_{t-1} A_t^\top) R_t^{-1} A_t (A_t^\top R_t^{-1} A_t + \Sigma_{t-1}^{-1})^{-1} \Sigma_{t-1}^{-1} \mu_{t-1} \\
&= B_t u_t + (A_t + A_t \Sigma_{t-1} A_t^\top R_t^{-1} A_t)(\Sigma_{t-1}^{-1} + A_t^\top R_t^{-1} A_t)^{-1} \Sigma_{t-1}^{-1} \mu_{t-1} \\
&= B_t u_t + A_t (I + \Sigma_{t-1} A_t^\top R_t^{-1} A_t)(I + \Sigma_{t-1} A_t^\top R_t^{-1} A_t)^{-1} \mu_{t-1} \\
&= B_t u_t + A_t \mu_{t-1}
\end{aligned}
$$

因此 $\overline{\mathrm{bel}}(x_t)$ 的均值为：

$$
\boxed{\;\bar\mu_t = A_t \mu_{t-1} + B_t u_t\;}
$$

**对均值再次求偏导** 得到 $\overline{\mathrm{bel}}(x_t)$ 的协方差的逆：

$$
\frac{\partial^2 L_t(x_t)}{\partial x_t^2} = (A_t \Sigma_{t-1} A_t^\top + R_t)^{-1}
$$

因此

$$
\boxed{\;\bar\Sigma_t = A_t \Sigma_{t-1} A_t^\top + R_t\;}
$$

> 至此推出了 Kalman Filter 算法的前两步。

---

### 7.8 测量更新步推导

现在我们根据测量对信念进行更新：

$$
\mathrm{bel}(x_t) = \eta\, p(z_t \mid x_t)\, \overline{\mathrm{bel}}(x_t)
$$

其中

$$
z_t = C_t x_t + \delta_t,\quad p(z_t \mid x_t) \sim \mathcal{N}(z_t;\, C_t x_t,\, Q_t),\quad \overline{\mathrm{bel}}(x_t) \sim \mathcal{N}(x_t;\, \bar\mu_t,\, \bar\Sigma_t)
$$

可写为：

$$
\mathrm{bel}(x_t) = \eta \exp\{-J_t\}
$$

其中

$$
J_t = \tfrac{1}{2}(z_t - C_t x_t)^\top Q_t^{-1}(z_t - C_t x_t) + \tfrac{1}{2}(x_t - \bar\mu_t)^\top \bar\Sigma_t^{-1}(x_t - \bar\mu_t)
$$

**计算 $J_t$ 对 $x_t$ 的前两个导数：**

$$
\frac{\partial J_t}{\partial x_t} = -C_t^\top Q_t^{-1}(z_t - C_t x_t) + \bar\Sigma_t^{-1}(x_t - \bar\mu_t)
$$

$$
\frac{\partial^2 J_t}{\partial x_t^2} = C_t^\top Q_t^{-1} C_t + \bar\Sigma_t^{-1}
$$

因此 $\mathrm{bel}(x_t)$ 的协方差矩阵为：

$$
\Sigma_t = (C_t^\top Q_t^{-1} C_t + \bar\Sigma_t^{-1})^{-1}
$$

**计算 $\mathrm{bel}(x_t)$ 的期望**，使 $\dfrac{\partial J_t}{\partial x_t} = 0$（将 $x_t$ 替换为 $\mu_t$）：

$$
C_t^\top Q_t^{-1}(z_t - C_t \mu_t) = \bar\Sigma_t^{-1}(\mu_t - \bar\mu_t)
$$

$$
C_t^\top Q_t^{-1} z_t = C_t^\top Q_t^{-1} C_t \mu_t + \bar\Sigma_t^{-1}(\mu_t - \bar\mu_t)
$$

$$
C_t^\top Q_t^{-1} z_t - C_t^\top Q_t^{-1} C_t \bar\mu_t = (C_t^\top Q_t^{-1} C_t + \bar\Sigma_t^{-1})(\mu_t - \bar\mu_t)
$$

$$
C_t^\top Q_t^{-1}(z_t - C_t \bar\mu_t) = (C_t^\top Q_t^{-1} C_t + \bar\Sigma_t^{-1})(\mu_t - \bar\mu_t)
$$

$$
C_t^\top Q_t^{-1}(z_t - C_t \bar\mu_t) = \Sigma_t^{-1}(\mu_t - \bar\mu_t)
$$

$$
\mu_t - \bar\mu_t = \Sigma_t C_t^\top Q_t^{-1}(z_t - C_t \bar\mu_t)
$$

设 $K_t = \Sigma_t C_t^\top Q_t^{-1}$，得：

$$
\boxed{\;\mu_t = \bar\mu_t + K_t(z_t - C_t \bar\mu_t)\;}
$$

即 Kalman 公式第 5 行的内容。

### 7.9 消除 $\Sigma_t$ 化简 $K_t$

现在尝试在 $K_t$ 中消除 $\Sigma_t$：

$$
\begin{aligned}
K_t &= \Sigma_t C_t^\top Q_t^{-1} = \Sigma_t C_t^\top Q_t^{-1}(C_t \bar\Sigma_t C_t^\top + Q_t)(C_t \bar\Sigma_t C_t^\top + Q_t)^{-1} \\
&= \Sigma_t (C_t^\top Q_t^{-1} C_t \bar\Sigma_t C_t^\top + C_t^\top Q_t^{-1} Q_t)(C_t \bar\Sigma_t C_t^\top + Q_t)^{-1} \\
&= \Sigma_t (C_t^\top Q_t^{-1} C_t \bar\Sigma_t C_t^\top + C_t^\top)(C_t \bar\Sigma_t C_t^\top + Q_t)^{-1} \\
&= \Sigma_t (C_t^\top Q_t^{-1} C_t \bar\Sigma_t C_t^\top + \bar\Sigma_t^{-1}\bar\Sigma_t C_t^\top)(C_t \bar\Sigma_t C_t^\top + Q_t)^{-1} \\
&= \Sigma_t (C_t^\top Q_t^{-1} C_t + \bar\Sigma_t^{-1}) \bar\Sigma_t C_t^\top (C_t \bar\Sigma_t C_t^\top + Q_t)^{-1} \\
&= \bar\Sigma_t C_t^\top (C_t \bar\Sigma_t C_t^\top + Q_t)^{-1}
\end{aligned}
$$

我们得到：

$$
\boxed{\;K_t = \bar\Sigma_t C_t^\top (C_t \bar\Sigma_t C_t^\top + Q_t)^{-1}\;}
$$

即 Kalman Filter 算法的第 4 行（**这样计算可以减少逆矩阵的计算次数**）。

### 7.10 计算 $\Sigma_t$

$$
\begin{aligned}
\Sigma_t &= (\bar\Sigma_t^{-1} + C_t^\top Q_t^{-1} C_t)^{-1} \\
&= \bar\Sigma_t - \bar\Sigma_t C_t^\top (Q_t + C_t \bar\Sigma_t C_t^\top)^{-1} C_t \bar\Sigma_t \quad \text{（应用矩阵求逆引理）} \\
&= (I - K_t C_t) \bar\Sigma_t
\end{aligned}
$$

即

$$
\boxed{\;\Sigma_t = (I - K_t C_t) \bar\Sigma_t\;}
$$

算法第 6 行。

> 至此完成 Kalman Filter 算法 5 个递推式的全部推导。

### 7.11 协方差更新的 Joseph 形式

7.10 节得到的协方差更新

$$
\Sigma_t = (I - K_t H_t)\,\bar\Sigma_t
$$

存在一个等价的写法，称为 **Joseph 形式**：

$$
\boxed{\;\Sigma_t = (I - K_t H_t)\,\bar\Sigma_t\,(I - K_t H_t)^\top + K_t\, Q_t\, K_t^\top\;}
$$

#### 7.11.1 等价性证明

从 Joseph 形式出发，记 $S_t = H_t \bar\Sigma_t H_t^\top + Q_t$（新息协方差），展开：

$$
\begin{aligned}
\Sigma_t^{\text{Joseph}}
&= (I - K_t H_t)\,\bar\Sigma_t\,(I - H_t^\top K_t^\top) + K_t Q_t K_t^\top \\
&= (I - K_t H_t)\,\bar\Sigma_t - (I - K_t H_t)\,\bar\Sigma_t H_t^\top K_t^\top + K_t Q_t K_t^\top \\
&= \underbrace{(I - K_t H_t)\,\bar\Sigma_t}_{\text{标准形式}} \;-\; \bar\Sigma_t H_t^\top K_t^\top + K_t H_t \bar\Sigma_t H_t^\top K_t^\top + K_t Q_t K_t^\top \\
&= (I - K_t H_t)\,\bar\Sigma_t \;-\; \bar\Sigma_t H_t^\top K_t^\top + K_t\,\underbrace{(H_t \bar\Sigma_t H_t^\top + Q_t)}_{=\,S_t}\,K_t^\top \\
&= (I - K_t H_t)\,\bar\Sigma_t \;-\; \bar\Sigma_t H_t^\top K_t^\top + K_t S_t K_t^\top
\end{aligned}
$$

现在代入最优卡尔曼增益（7.9 节结论，EKF的结果中用 $H_t$ 替换 $C_t$ ）：

$$
K_t = \bar\Sigma_t H_t^\top S_t^{-1} \quad\Longrightarrow\quad K_t S_t = \bar\Sigma_t H_t^\top
$$

两边右乘 $K_t^\top$：

$$
K_t S_t K_t^\top = \bar\Sigma_t H_t^\top K_t^\top
$$

因此最后两项**恰好相消**：

$$
-\,\bar\Sigma_t H_t^\top K_t^\top + K_t S_t K_t^\top = 0
$$

代回得：

$$
\Sigma_t^{\text{Joseph}} = (I - K_t H_t)\,\bar\Sigma_t
$$

#### 7.11.2 Joseph 形式的结构性安全

在浮点运算下，标准形式 $(I - K_t H_t)\,\bar\Sigma_t$ 有两个结构性弱点：

1. **不显式对称**。标准形式是两个一般矩阵的乘积，输出在浮点累积下会出现和积累 $\Sigma_t \neq \Sigma_t^\top$ 的小量偏差。
2. **不显式半正定**。若 $K_t$ 因为 $S_t$ 病态、$Q_t$ 估计偏小或 Jacobian 误差而**偏离最优值**，标准形式可能直接产生负特征值。

Joseph 形式是**结构性安全**的：

$$
\Sigma_t = \underbrace{M \bar\Sigma_t M^\top}_{\text{对称 PSD}} + \underbrace{K_t Q_t K_t^\top}_{\text{对称 PSD}},\qquad M := I - K_t H_t
$$

只要 $\bar\Sigma_t$ 和 $Q_t$ 为对称 PSD，输出就**结构性保证**为对称 PSD，且这一性质**与 $K_t$ 是否最优无关**：

- EKF 中的 $H_t$ 来自 Jacobian 线性化，本身就是近似；
- 当线性化误差较大时，"由近似的 $H_t, G_t$ 计算出的 $K_t$" 已经不是真正意义下的最优增益；
- 在这种"近似 $K_t$"下，标准形式不再保证 PSD，而 Joseph 形式仍然保证。

---

## 8. 扩展卡尔曼滤波器（EKF）

### 8.1 动机

Kalman Filter 要求下一个状态是上一个状态的线性函数，同时要求观测是状态的线性函数。这在高斯形式的变换过程中至关重要。但在现实中，转移和测量方程几乎都不是线性的，**Extended Kalman Filter 放宽了这个假设**——这里的假设是状态转移概率和测量概率由非线性函数控制：

$$
x_t = g(u_t, x_{t-1}) + \varepsilon_t, \qquad z_t = h(x_t) + \delta_t
$$

EKF 依然继承了 Kalman Filter 的方法，但其目的从"精确计算后验高斯"变为"**高效近似地计算后验高斯**"。

### 8.2 核心思想：线性化

EKF 的核心思想是线性化——通过在高斯分布的均值处与 $g$ 相切的线性函数来近似非线性函数。EKF 线性化的过程采用 **Taylor 展开**。

**对运动方程 $g$：**

$$
g(u_t, x_{t-1}) \approx g(u_t, \mu_{t-1}) + g'(u_t, \mu_{t-1})(x_{t-1} - \mu_{t-1}) = g(u_t, \mu_{t-1}) + G_t (x_{t-1} - \mu_{t-1})
$$

其中

$$
G_t = g'(u_t, \mu_{t-1}) = \frac{\partial g(u_t, \mu_{t-1})}{\partial x_{t-1}}
$$

$G_t$ 为一个 $n \times n$ 的雅可比矩阵。此时

$$
p(x_t \mid u_t, x_{t-1}) \approx \det(2\pi R_t)^{-\frac{1}{2}} \exp\!\left\{-\tfrac{1}{2}[x_t - g(u_t, \mu_{t-1}) - G_t(x_{t-1} - \mu_{t-1})]^\top R_t^{-1}[x_t - g(u_t, \mu_{t-1}) - G_t(x_{t-1} - \mu_{t-1})]\right\}
$$

**对观测方程 $h$：** 同样地，对 $z_t = h(x_t) + \delta_t$ 中的 $h$ 做 Taylor 展开

$$
h(x_t) \approx h(\bar\mu_t) + h'(\bar\mu_t)(x_t - \bar\mu_t) = h(\bar\mu_t) + H_t(x_t - \bar\mu_t)
$$

其中

$$
H_t = h'(\bar\mu_t) = \frac{\partial h(\bar\mu_t)}{\partial x_t}
$$

因此

$$
p(z_t \mid x_t) \approx \det(2\pi Q_t)^{-\frac{1}{2}} \exp\!\left\{-\tfrac{1}{2}[z_t - h(\bar\mu_t) - H_t(x_t - \bar\mu_t)]^\top Q_t^{-1}[z_t - h(\bar\mu_t) - H_t(x_t - \bar\mu_t)]\right\}
$$

### 8.3 EKF 算法

经过线性化后，我们得到以下算法的递推公式：

**Algorithm Extended_Kalman_Filter($\mu_{t-1}, \Sigma_{t-1}, u_t, z_t$):**

```
1: μ_bar_t = g(u_t, μ_{t-1})                                      # 用非线性函数 g 预测均值
2: Σ_bar_t = G_t Σ_{t-1} G_t^T + R_t                              # 用 Jacobian G_t 预测协方差
3: K_t     = Σ_bar_t H_t^T (H_t Σ_bar_t H_t^T + Q_t)^{-1}         # 卡尔曼增益
4: μ_t     = μ_bar_t + K_t (z_t - h(μ_bar_t))                     # 用非线性函数 h 计算新息
5: Σ_t     = (I - K_t H_t) Σ_bar_t                                # 更新协方差
6: return μ_t, Σ_t
```

数学表达：

$$
\begin{aligned}
\bar\mu_t &= g(u_t, \mu_{t-1}) \\
\bar\Sigma_t &= G_t \Sigma_{t-1} G_t^\top + R_t \\
K_t &= \bar\Sigma_t H_t^\top (H_t \bar\Sigma_t H_t^\top + Q_t)^{-1} \\
\mu_t &= \bar\mu_t + K_t(z_t - h(\bar\mu_t)) \\
\Sigma_t &= (I - K_t H_t) \bar\Sigma_t
\end{aligned}
$$

$\Sigma_t$ 的 Joseph 形式：

$$\Sigma_t = (I - K_t H_t)\,\bar\Sigma_t\,(I - K_t H_t)^\top + K_t\, Q_t\, K_t^\top$$

### 8.4 EKF 与 KF 的对照

| 步骤             | Kalman Filter                     | Extended Kalman Filter            |
|----------------|-----------------------------------|-----------------------------------|
| 状态预测（均值）       | $A_t \mu_{t-1} + B_t u_t$         | $g(u_t, \mu_{t-1})$               |
| 状态预测（协方差）      | $A_t \Sigma_{t-1} A_t^\top + R_t$ | $G_t \Sigma_{t-1} G_t^\top + R_t$ |
| 新息（innovation） | $z_t - C_t \bar\mu_t$             | $z_t - h(\bar\mu_t)$              |
| 卡尔曼增益矩阵        | 含 $C_t$                           | 含 $H_t$                           |

**核心区别：** 均值传播使用**原始非线性函数** $g, h$ —— 这能保留非线性的精确表达；协方差传播使用**雅可比矩阵** $G_t, H_t$ —— 这是不可避免的近似。

## 9. 卡方分布与一致性检验

### 9.1 卡方分布
**卡方分布** $\chi^2_k$ 是 $k$ 个独立标准正态随机变量的平方和的分布。

**定义**:$Z_1, \ldots, Z_k$ 独立标准正态,平方和

$$X = Z_1^2 + Z_2^2 + \cdots + Z_k^2 \sim \chi^2(k)$$

其中"自由度" $k$ 为被平方求和的独立标准正态变量个数。

对于 $n$ 维高斯分布，马氏距离平方 $d_M^2(x) = (x - \mu)^\top \Sigma^{-1} (x - \mu)$ 的精确分布就是 $\chi^2(n)$：

若 $x \sim \mathcal{N}(\mu, \Sigma)$,白化后 $y \sim \mathcal{N}(0, I)$,每分量独立 $\mathcal{N}(0, 1)$。所以

$$d_M^2(x) = \|y\|^2 = \sum y_i^2 \sim \chi^2(n)$$

其意义是把几何上的距离转化成统计上可检验的随机变量。

$\chi^2(n)$ 的 95% 置信区间是 $[\chi^2_{0.025}(n), \chi^2_{0.975}(n)]$。例如 $n=3$ 时约 $[0.22, 9.35]$。若 $d_M^2$ 落在范围外,95% 的概率说明模型有问题。

### 9.2 NIS（Normalized Innovation Squared）
**NIS** 是 EKF 中新息 $z_t - h(\bar\mu_t)$ 的马氏距离平方：
$$\mathrm{NIS} = (z_t - h(\bar\mu_t))^\top S_t^{-1} (z_t - h(\bar\mu_t))$$
其中 $S_t = H_t \bar\Sigma_t H_t^\top + Q_t$ 是新息的协方差矩阵。

NIS 的分布近似为 $\chi^2(m)$,其中 $m$ 是观测维度。通过检验 NIS 是否落在 $\chi^2(m)$ 的置信区间内,可以评估 EKF 的性能。

- 若 NIS 大于 $\chi^2_{0.975}(m)$,说明新息过大,模型可能过于乐观（低估了不确定性）。
- 若 NIS 小于 $\chi^2_{0.025}(m)$,说明新息过小,模型可能过于悲观（高估了不确定性）。
- 若 NIS 落在区间内,说明新息与模型预期一致,EKF 可能表现正常。

---

## 附录：常用符号速查

| 符号                         | 含义                                          |
|----------------------------|---------------------------------------------|
| $x_t$                      | $t$ 时刻状态向量                                  |
| $u_t$                      | $t$ 时刻控制输入                                  |
| $z_t$                      | $t$ 时刻观测向量                                  |
| $\mu_t,\ \Sigma_t$         | 后验信念 $\mathrm{bel}(x_t)$ 的均值与协方差            |
| $\bar\mu_t,\ \bar\Sigma_t$ | 先验信念 $\overline{\mathrm{bel}}(x_t)$ 的均值与协方差 |
| $A_t,\ B_t,\ C_t$          | KF 中的线性状态转移、控制、观测矩阵                         |
| $g(\cdot),\ h(\cdot)$      | EKF 中的非线性状态转移、观测函数                          |
| $G_t,\ H_t$                | EKF 中 $g, h$ 关于状态的雅可比矩阵                     |
| $R_t$                      | 状态转移噪声协方差（运动模型噪声）                           |
| $Q_t$                      | 观测噪声协方差（测量模型噪声）                             |
| $K_t$                      | 卡尔曼增益                                       |
| $\eta$                     | 归一化常数                                       |

---
