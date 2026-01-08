# 利用径向基函数进行隐式边界建模

**学习目标 (Learning Objectives)**
*   回顾平稳域的重要性
*   回顾径向基函数 (RBF) 的框架
*   比较 RBF 插值和对偶克里金法 (Dual Kriging)

## 1 引言 (Introduction)

地质统计建模中的一项关键任务是**界定域边界** (delimitation of domain boundaries)。进行此操作是为了满足地质统计工作流程中使用的**平稳性** (stationarity) 决策。这些域可以基于品位、岩性、蚀变、矿化、构造或这些因素的组合。历史上，域边界的界定是基于地质学家的经验和知识进行**手动解释**的。在数据间隔较宽的情况下，可能会存在不同的解释 (Silva & Deutsch, 2012)，并且这些解释可能耗时且难以重现。

Figure 1: 相同边界建模问题（红色和蓝色之间）的不同解释

GeostatisticsLessons.com©2022 S. Sanchez and C.V. Deutsch 1

为了补充手动解释，人们开发了若干用于**隐式建模**的数学方法；一种常用的方法是使用径向基函数 (RBF) 对**符号距离函数** (signed distance functions) 进行插值 (Cowan et al., 2003)。符号距离函数已被广泛用于隐式曲面推断 (implicit surface inference) (Osher & Fedkiw, 2003)。RBF 最早由 (Hardy, 1971) 在地质文献中提及。RBF 的发展与 Matheron 关于区域化变量的工作 (Matheron, 1963) 是并行进行的。RBF 和克里金法 (Kriging) 的一个关键区别在于：RBF 使用**正定基函数**进行插值，而克里金法使用**协方差函数**进行插值 (Cowan et al., 2003)。此外，克里金法通常使用局部搜索邻域为每个估计值计算权重；RBF 传统上使用所有样本一次性计算权重。存在一种与克里金法形式相似的方法，称为对偶克里金 (Dual Kriging) (Chilès & Delfiner, 1999)，它也只计算一次权重。下面将讨论 RBF 和克里金插值。

对于隐式边界建模，首先计算样本与**最近的不同域样本**之间的距离，对于落在正在建模域**内部**的样本设置负值，对于落在**外部**的样本设置正值。然后，对这些距离进行插值，并在 $distance = 0$ 的位置提取域之间的边界。有几种插值方法；离散平滑插值 (Mallet, 1989)、经典地质统计方法 (Blanchin & Chilès, 1993) 和 RBF 方法 (Cowan et al., 2003)，带有梯度和约束的 RBF (Hillier, Schetselaar, Kemp, & Perron, 2014) 以及带有局部各向异性的 RBF (Martin & Boisvert, 2017)。在本课中，将回顾径向基函数 (RBF) 方法。并将与克里金法进行比较。

## 2 平稳域 (Stationary Domains)

给定一组样本，可以使用两种类型的模型来预测未知位置的值。如果控制样本行为的定律是众所周知的，则可以推断出确定性模型，并且值的预测相当直接。不幸的是，矿床通常是许多复杂且混沌的地质过程的结果。这些过程过于复杂，无法基于稀疏数据定义确定性模型，因此，假定样本值是**随机过程**的结果。在这种情况下，可以使用概率模型 (Isaaks & Srivastava, 1989)。

在概率模型中，样本被认为是随机变量 $Z(u_i)$ 的实现，其中 $i = 1, ..., N$。需要每个位置 $u$ 上的多个样本才能计算概率参数，例如均值和方差。由于每个位置只有一个样本，这是不可能的。为了解决这个问题，随机变量 $Z(u_i)$ 被假定为**平稳**的。平稳性意味着随机变量遵循相同的概率定律，与其位置无关，因此，概率参数，如均值 $m(u)$，在所有位置都相同。平稳性假设是推导克里金法等地质统计工具的关键组成部分 (Isaaks & Srivastava, 1989)。

地质区域被划分为被假定为平稳的域 (McLennan, 2007)。这些域被认为是统计上同质的，并共享共同的地质特征。域的定义应能更好地重现地质特征并产生更可靠的模型。

GeostatisticsLessons.com©2022 S. Sanchez and C.V. Deutsch 2

## 3 距离函数 (Distance Functions)

为了清晰起见，下面假设存在两个域：一个在**内部**，另一个在**外部**。该形式主义可以扩展到两个以上的域。第一步是为每个样本分配一个指示符。如果样本在域内部，指示符取值为 1，如果在外部，则取值为 0 (Silva & Deutsch, 2012)：

$$I(u_i) = \begin{cases} 1 & \text{if } u_i \text{ belongs to the domain} \\ 0 & \text{if } u_i \text{ otherwise} \end{cases}$$
其中，$u_i$, $i = 1, ..., N$ 是样本的位置。然后，计算到具有不同指示符的最近样本的距离：

$$DF(u_i) = \begin{cases} - \text{argmin}(r) & \text{if } I(u_i) = 1 \\ + \text{argmin}(r) & \text{if } I(u_i) = 0 \end{cases}$$
其中 $\text{argmin}()$ 返回最小值，而 $r$ 是**欧几里得直线距离**：

$$r(u_i,u_j) = \sqrt{(x_i - x_j)^2 + (y_i - y_j)^2 + (z_i - z_j)^2}$$
通常，地质域具有**各向异性** (anisotropy)，即在不同方向上具有不同的连续性长度尺度。在这些情况下，样本之间的距离可以按如下方式计算：

$$r(u_i,u_j) = \sqrt{\left(\frac{x_i - x_j}{a_x}\right)^2 + \left(\frac{y_i - y_j}{a_y}\right)^2 + \left(\frac{z_i - z_j}{a_z}\right)^2}$$
其中 $a_x, a_y, a_z$ 是各向异性的范围 (ranges of anisotropy)。坐标 $x, y$ 和 $z$ 被旋转以与主要连续性方向对齐 (Silva & Deutsch, 2012)。这些距离函数值将用作插值的测量值。

## 4 径向基函数框架 (Radial Basis Function Framework)

给定在位置 $u_i$ 处的一组测量值 $f(u_i)$，其中 $i = 1, ..., N$，可以定义一个插值器 $s(u)$ 来预测任何未采样位置的值。插值器应该在测量值的位置重现测量值 $s(u_i) = f(u_i)$ (Fasshauer, 2007)。

为了定义 $s(u)$，一种常见的方法是考虑函数 $B_i$ 的加权线性组合 (Fasshauer, 2007)：
$$s(u) = \sum_{i=1}^{N} w_i B_i(u)$$
函数 $B_i$ 被称为**基函数** (*basis functions*)。插值器 $s(u_i)$ 必须在各自的位置复制测量值：$s(u_i) = f(u_i)$，这导致了一个线性方程组：

$$\mathbf{Aw} = \boldsymbol{f(u_i)}$$

$$\begin{bmatrix}
B_1(u_1) & B_2(u_1) & \cdots & B_N(u_1) \\
B_1(u_2) & B_2(u_2) & \cdots & B_N(u_2) \\
\vdots & \vdots & \ddots & \vdots \\
B_1(u_N) & B_2(u_N) & \cdots & B_N(u_N)
\end{bmatrix}
\begin{bmatrix}
w_1 \\
w_2 \\
\vdots \\
w_N
\end{bmatrix} =
\begin{bmatrix}
f(u_1) \\
f(u_2) \\
\vdots \\
f(u_N)
\end{bmatrix}$$

GeostatisticsLessons.com©2022 S. Sanchez and C.V. Deutsch 3

Figure 2: 空间插值问题的示意图，即五个数据位置和一个正在被估计的位置。

其中 $\mathbf{A}$ 是**基函数** (*basis functions*) 矩阵：$A_{ij} = B_j(u_i)$，其中 $i, j = 1, ..., N$，$f(u_i)$ 是测量值的列向量，而 $w_i$ 是权重。距离函数的一个变换 $\phi(r(u,u_i))$ 可以用作**基函数** $B_i(u) = \phi(r(u,u_i))$：

$$s(u) = \sum_{i=1}^{N} w_i\phi(r(u,u_i))$$
如果我们认为 $\phi(r(u,u_1)) = \phi(r(u,u_2))$，只要 $r(u,u_1) = r(u,u_2)$，那么 $\phi(r(u,u_i))$ 对于到中心位置 $u$ 的固定距离 $r$ 具有相同的值。这意味着 $\phi$ 相对于 $u$ 是**径向对称**的，因此被称为**径向基函数** (*Radial Basis Function*, RBF) (Fasshauer, 2007)。

如前所述，通过求解线性方程组找到权重。为了确保系统有解，RBF 矩阵要求是**正定**的 (positive definite) (Fasshauer, 2007)。下表显示了一些常用的正定 RBF：

| RBF 类型 | 方程 $\phi(r)$ | 属性 |
| :--- | :--- | :--- |
| 高斯函数 (Gaussian) | $\phi(r) = e^{-\epsilon^2r^2}$ | 正定 (Positive definite) |
| 球面函数 (Spherical) | $\phi(r) = 1.5\epsilon r - 0.5(\epsilon r)^3$ | 正定，对于 $r<1/\epsilon$ |
| 指数函数 (Exponential) | $\phi(r) = e^{-3r/\epsilon}$ | 正定 |
| 多二次函数 (Multiquadratic) | $\phi(r) = \sqrt{1 + (\epsilon r)^2}$ | 条件正定 (Conditionally positive definite) |
| 线性函数 (Linear) | $\phi(r) = r$ | 仅在一维中条件正定，无参数 |

Table 1: 选定的径向基函数 (Fasshauer, 2007) 其中 $\epsilon$ 是一个选定的参数。克里金法也许是地质统计学中使用最广泛的插值器。

GeostatisticsLessons.com©2022 S. Sanchez and C.V. Deutsch 4

## 5 普通克里金法 (Ordinary Kriging)

普通克里金法 (OK) 是一种广泛使用的线性估计器：
$$z^*(u) = \sum_{i=1}^{N} \lambda_i \cdot z(u_i)$$
其中 $z^*(u)$ 是未采样位置 $u$ 的估计值，$\lambda_i$ 是权重，而 $z(u_i)$ 是位置 $u_i$ 处的样本。OK 估计器约束估计器为无偏。为了满足无偏性的约束，权重的总和必须为一 (Isaaks & Srivastava, 1989)。权重是通过最小化误差方差来计算的。误差方差的偏导数和无偏性约束提供了 $n+1$ 个方程，用于求解 $n$ 个未知权重和一个拉格朗日乘数 (Isaaks & Srivastava, 1989)：

$$\sum_{j=1}^{N} \lambda_j \text{Cov}(u_i,u_j) + \mu = \text{Cov}(u,u_i) \quad \forall i = 1, ..., N$$
$$\sum_{i=1}^{N} \lambda_i = 1$$
其中 $\text{Cov}(u_i,u_j)$ 是位置 $u_i$ 和 $u_j$ 之间的协方差。这个问题的解可以写成：

$$\mathbf{C}\boldsymbol{\lambda} = \boldsymbol{c}$$

$$\begin{bmatrix}
C_{11} & C_{12} & \cdots & C_{1N} & 1 \\
\vdots & \vdots & \ddots & \vdots & 1 \\
C_{N1} & C_{N2} & \cdots & C_{NN} & 1 \\
1 & 1 & \cdots & 1 & 0
\end{bmatrix}
\begin{bmatrix}
\lambda_1 \\
\vdots \\
\lambda_N \\
\mu
\end{bmatrix} =
\begin{bmatrix}
c_1 \\
\vdots \\
c_N \\
1
\end{bmatrix}$$
其中 $C_{ij} = \text{Cov}(u_i,u_j)$ 且 $c_i = \text{Cov}(u,u_i)$。所有位置对之间的协方差是从变异函数模型中获得的。通常，OK 为了计算效率使用搜索邻域，但在距离函数的情况下，可以考虑所有样本以减少伪影 (Silva & Deutsch, 2012)。

## 6 对偶克里金法 (Dual Kriging)

克里金权重的方程（见上文）可以写成：
$$\boldsymbol{\lambda}^T = \boldsymbol{c}^T\mathbf{C}^{-1}$$
将这些权重插入估计器方程，得到：
$$z^*(u) = \boldsymbol{c}^T\mathbf{C}^{-1}\boldsymbol{z}$$
乘积 $\mathbf{C}^{-1}\boldsymbol{z}$ 可以表示为：
$$\boldsymbol{d} = \mathbf{C}^{-1}\boldsymbol{z}$$
$$\mathbf{C}\boldsymbol{d} = \boldsymbol{z}$$

GeostatisticsLessons.com©2022 S. Sanchez and C.V. Deutsch 5

$$\begin{bmatrix}
C_{11} & C_{12} & \cdots & C_{1N} & 1 \\
\vdots & \vdots & \ddots & \vdots & 1 \\
C_{N1} & C_{N2} & \cdots & C_{NN} & 1 \\
1 & 1 & \cdots & 1 & 0
\end{bmatrix}
\begin{bmatrix}
d_1 \\
\vdots \\
d_N \\
b
\end{bmatrix} =
\begin{bmatrix}
z(u_1) \\
\vdots \\
z(u_N) \\
0
\end{bmatrix}$$
其中 $\boldsymbol{d} = [d_1, d_2, ..., d_N, b]$ 是乘积 $\mathbf{C}^{-1}\boldsymbol{z}$ 的结果。

然后，估计器可以写成对偶形式：
$$z^*(u) = \boldsymbol{c}^T\boldsymbol{d}$$
$$z^*(u) = \sum_{i=1}^{N} d_i \text{Cov}(u,u_i) + b$$
这被称为对偶克里金形式 (Dual Kriging form) (Chilès & Delfiner, 1999)。对偶克里金比克里金法的原始形式更具计算效率，因为权重 ($d_i$) 只计算一次用于所有估计 (Stewart, Lacey, Hodkiewicz, & Lane, 2014)。估计值是通过估计位置和样本之间协方差的加权线性组合来计算的。注意，对偶克里金与 RBF 插值具有相似的形式，其中：

$$\phi(r(u,u_i)) = \text{Cov}(u,u_i)$$
协方差函数可以被视为一个径向基函数，并且权重是通过类似的线性方程组计算出来的。

## 7 RBF 插值示例 (Example of RBF Interpolation)

将从两个不同域的四个样本中计算边界限制。计算样本与来自不同域的最近样本之间的距离。按照约定，将负距离值分配给正在建模的域的样本：

| 样本 (Sample) | 域 (Domain) | x | y |
| :--- | :--- | :--- | :--- |
| $u_1$ | 1 | 5 | 4 |
| $u_2$ | 1 | 10 | 6 |
| $u_3$ | 2 | 3 | 10 |
| $u_4$ | 2 | 11 | 8 |

举例来说，使用高斯 RBF $\phi(r) = e^{-\epsilon^2r^2}$ 且 $\epsilon = 0.1$，计算位置 $p = (7, 8)$ 处的插值距离。使用以下方程计算权重：

$$\begin{bmatrix}
\phi(r(u_1,u_1)) & \phi(r(u_1,u_2)) & \phi(r(u_1,u_3)) & \phi(r(u_1,u_4)) \\
\phi(r(u_2,u_1)) & \phi(r(u_2,u_2)) & \phi(r(u_2,u_3)) & \phi(r(u_2,u_4)) \\
\phi(r(u_3,u_1)) & \phi(r(u_3,u_2)) & \phi(r(u_3,u_3)) & \phi(r(u_3,u_4)) \\
\phi(r(u_4,u_1)) & \phi(r(u_4,u_2)) & \phi(r(u_4,u_3)) & \phi(r(u_4,u_4))
\end{bmatrix}
\begin{bmatrix}
w_1 \\
w_2 \\
w_3 \\
w_4
\end{bmatrix} =
\begin{bmatrix}
f(u_1) \\
f(u_2) \\
f(u_3) \\
f(u_4)
\end{bmatrix}$$

GeostatisticsLessons.com©2022 S. Sanchez and C.V. Deutsch 6

Figure 3: 到最近不同域样本的距离

$$\begin{bmatrix}
1 & 0.75 & 0.67 & 0.59 \\
0.75 & 1 & 0.52 & 0.95 \\
0.67 & 0.52 & 1 & 0.51 \\
0.59 & 0.95 & 0.51 & 1
\end{bmatrix}
\begin{bmatrix}
w_1 \\
w_2 \\
w_3 \\
w_4
\end{bmatrix} =
\begin{bmatrix}
-6.3 \\
-2.2 \\
6.3 \\
2.2
\end{bmatrix}$$
求解该方程组得到：
$$\begin{bmatrix}
w_1 \\
w_2 \\
w_3 \\
w_4
\end{bmatrix} =
\begin{bmatrix}
-12.2 \\
-27.8 \\
14.5 \\
28.5
\end{bmatrix}$$
插值距离为：
$$s(u) = \sum_{i=1}^{N} w_i\phi(r(u,u_i))$$
$$-12.2 * 0.82 + -27.8 * 0.88 + 14.5 * 0.82 + 28.5 * 0.85 = 1.65$$
由于权重不依赖于正在估计的位置，因此可以在所有位置计算插值距离。下图中显示了使用高斯 RBF 在网格上进行 RBF 插值的示例（仅限网络版），其中 $\epsilon$ 取不同值。在 $distance = 0$ 的位置绘制边界线。

符号距离函数的 RBF 插值不限于 2 维网格。它可以应用于 3 维网格，也可以用于两个以上域的边界界定：

扩展到两个以上域的内容将在下面回顾。

GeostatisticsLessons.com©2022 S. Sanchez and C.V. Deutsch 7

Figure 4: RBF 3D 插值

## 8 RBF 与克里金法比较 (RBF and Kriging Comparison)

推导克里金法公式时假定**平稳性**。测量值应满足此假设才能获得可靠的估计。由于距离函数的非平稳性，插值距离时，这个约束具有挑战性。此外，克里金法使用变异函数来获得权重。在某些情况下，变异函数难以建模且不可靠（稀疏采样）。由于 RBF 公式不依赖于平稳性假设，也不依赖于变异函数，因此在这种情况下可能更稳健 (Cowan et al., 2003)，尽管如果平稳性得到合理满足，估计值将更准确 (Stewart et al., 2014)。

克里金法旨在最小化误差或估计方差。最小化的估计方差可以计算并用作表示数据配置的局部估计误差的量度 (Rossi & Deutsch, 2013)。在 RBF 或对偶克里金框架中，没有估计方差的量化。
变异函数模型是描述样本空间相关性的一种方式。通常，变异函数是使用具有不同形状和各向异性的嵌套结构来建模的 (Rossi & Deutsch, 2013)。克里金法使用从变异函数模型获得的协方差值来最优地加权样本。某些 RBF 可以在插值中发挥类似的作用。高斯函数和反多二次函数就是这种特性的例子，其中参数 $\epsilon$ 决定了样本的最大影响距离。由于用于建模变异函数的函数是正定的，它们也可以用于 RBF 插值。

下面的示例显示了三种不同 RBF 的隐式边界建模（具有相似的 $\epsilon$ 参数）与普通克里金对偶形式的比较。RBF 和对偶克里金生成的边界形状存在显著差异。高斯 RBF 倾向于生成平滑的边界，而其他 RBF 和对偶克里金遵循样本的形状。在指数 RBF 的情况下，RBF 值比高斯和球面 RBF 值更快地接近零，从而产生更小的边界。边界的体积和形状可以通过以下方式修改

GeostatisticsLessons.com©2022 S. Sanchez and C.V. Deutsch 8

Figure 5: 使用不同 RBF 和普通对偶克里金的隐式边界建模示例

通过改变 $\epsilon$ 参数，因为它代表样本的**影响半径** (radius of influence)。边界的形状在靠近样本的地方相当相似，但在稀疏区域差异很大。因此，边界形状将取决于用于插值的 RBF 类型、$\epsilon$ 参数和数据间距。边界形状的不确定性可以计算，但 RBF 和对偶克里金不提供不确定性估计。尽管普通克里金对偶形式使用球面 RBF 作为协方差函数（与球面 RBF 插值使用的 $\epsilon$ 参数相同），但它生成了一个封闭边界。根据 $\epsilon$ 值，数据可能不会影响很远处的未采样位置。在对偶克里金的情况下，估计器中的分量 $b$ 是对全局均值的估计。当位置很远时，求和趋于 0，而 $b$ 值占主导地位。在这个例子中，$b$ 具有正值，限制了边界的延伸。简单克里金对偶形式 (Simple Kriging Dual form) 更类似于 RBF 公式，因为它没有 $b$ 分量。

GeostatisticsLessons.com©2022 S. Sanchez and C.V. Deutsch 9

RBF 插值和对偶克里金似乎比克里金法计算效率更高，但数据集的大小必须合理小，例如少于 10000 个。更大的系统需要进行修改，例如迭代求解方法 (Beatson, Cherrie, & Mouat, 1999) 或域分解方法，如单元划分 (partition of unity) (Martin & Boisvert, 2015)。

## 9 讨论 (Discussion)

尽管距离函数的 RBF 插值是隐式建模的既定方法，但它有一些必须考虑的缺点。根据使用的核函数，RBF 插值在稀疏数据区域往往不太可靠，并可能在模型的边缘产生有偏差的外推体积 (Silva, 2015)。此外，添加新数据可能会以一种看似武断的方式，改变距离新数据很远处的边界限制。边界限制取决于钻孔间距；RBF 插值可能无法将边界限制公平均匀地放置在不同类别样本之间。此外，很难修改插值方法以考虑地质趋势。最后，没有直接的不确定性推断。在存在间隔较宽数据的情况下，边界位置存在模糊性，但插值算法只提供一个“最佳”估计。其中一些问题已在文献中得到解决。下面简要讨论 RBF 扩展和其他应用。

(Martin & Boisvert, 2015) 提出了一种具有**局部各向异性**的 RBF 方法。为了在 RBF 框架中包含局部各向异性，向公式中添加了额外的分量：
$$s(u) = \sum_{i=1}^{N} w_i\phi(r(u,u_i)) + \sum_{j=N+1}^{M} \alpha_j\nabla\phi(r(u,u_j)) + \sum_{k=M+1}^{O} \beta_k t_k \nabla\phi(r(u,u_k))$$
其中 $i = 1, . . . , N$ 是样本位置 $f(u_i)$，$j = N + 1, . . . , M$ 是带有走向-倾角数据的梯度位置，其中标量函数的势场垂直于平面方向，$\nabla f(u_i) = n_j$，$k = M +1, . . . , O$ 是切线位置，其中 $t_k$ 是与感兴趣表面相切的线，$t_k * \nabla f(u_i) = 0$。在某些情况下，样本集可能具有应被重现的底层函数。在这些情况下，可以将**多项式分量**添加到公式中 (Fasshauer, 2007)：

$$\sum_{l=1}^{M} b_l p_l(u)$$
其中 $p_l(u)$ 是多项式，而 $b_l$ 是系数。边界界定的公式可以扩展到**多个域** (multiple domains) (Silva & Deutsch, 2012)。不是分配一个单一指示符，而是为每个样本分配一个大小为 $K$（假设域的数量）的指示符向量 $I_k$。属于域 $k$ 的样本将在向量的第 $k$ 个元素中为 1，在其余 $K-1$ 个元素中为零。与之前类似，计算向量每个元素的距离函数。然后，使用 RBF 或克里金法对每个 $k$ 域的符号距离函数进行插值。通过在每个位置取**最小估计符号距离函数**来分配该位置的域。对该方法的更深入了解可以在地质统计课程 *Signed Distance Function Modeling with Multiple Categories* 中回顾。

GeostatisticsLessons.com©2022 S. Sanchez and C.V. Deutsch 10

另一种隐式边界建模方法是**指示符插值** (interpolation of indicators)。该方法由 (Mancell & Deutsch, 2020) 详细描述。对 $1/0$ 指示符进行插值，并在导致域内部无偏比例的值处设置阈值。使用最近邻模型来确定无偏比例。该方法的一个优点是，它可以通过改变阈值来提供**不确定性模型**。可以创建乐观和悲观的边界模型。

矿化不确定性的主要组成部分之一是地质不确定性。隐式建模仅提供确定性域模型；存在纳入不确定性的修改 (Munroe & Deutsch, 2008) 和 (Wilde & Deutsch, 2011)。

## 10 总结 (Summary)

RBF 和克里金法是用于隐式域边界界定的距离函数的插值方法。它们的公式相似，但在**平稳性假设**、**误差量化**、**计算效率**和**参数选择**方面存在差异。近年来，已开发出变体以包含地质趋势、各向异性和不确定性。许多软件使用这些方法进行隐式地质建模。地质建模人员几乎总是需要干预，以确保生成的模型满足复杂的、难以简单纳入隐式建模的地质约束。

## 11 参考文献 (References)

Beatson, R. K., Cherrie, J. B., & Mouat, C. T. (1999). Fast fitting of radial basis functions: Methods based on preconditioned GMRES iteration. *Advances in Computational Mathematics, 11(2),* 253–270.
Blanchin, R., & Chilès, J.-P. (1993). The channel tunnel: Geostatistical prediction of the geological conditions and its validation by the reality. *Mathematical Geology, 25(7),* 963–974.
Chilès, J.-P., & Delfiner, P. (1999). *Geostatistics: Modeling spatial uncertainty* (p. 695). Wiley: New York.
Cowan, J., Beatson, R., Ross, H. J., Fright, W. R., McLennan, T. J., Evans, T. R., … Titley, M. (2003). Practical implicit geological modelling. In *5th International Mining Geology Conference*.
Fasshauer, G. E. (2007). *Meshfree approximation methods with MATLAB* (Vol. 6). World Scientific.
Hardy, R. L. (1971). Multiquadric equations of topography and other irregular surfaces. *Journal of Geophysical Research, 76(8),* 1905–1915.
Hillier, M. J., Schetselaar, E.M., Kemp, E. A. de, & Perron, G. (2014). Three-dimensional modelling of geological surfaces using generalized interpolation with radial basis functions. *Mathematical Geosciences, 46(8),* 931–953.
Isaaks, E. H., & Srivastava, R.M. (1989). *Applied geostatistics* (Vol. 561, p. 561). Oxford University Press, New York.
Mallet, J.-L. (1989). Discrete smooth interpolation. *ACM Transactions on Graphics (TOG), 8(2),* 121–144.
Mancell, S., & Deutsch, C. (2020). Application of indicator interpolated thresholding for implicit modeling with uncertainty. *Center for Computational Geostatistics Annual Report 17. University of Alberta. Paper 118*.

GeostatisticsLessons.com©2022 S. Sanchez and C.V. Deutsch 11

Martin, R., & Boisvert, J. (2015). Review of radial basis functions and domain decomposition for implicit geological modelling. *Center for Computational Geostatistics Annual Report 17. University of Alberta. Paper 118*.
Martin, R., & Boisvert, J. B. (2017). Iterative refinement of implicit boundary models for improved geological feature reproduction. *Computers & Geosciences, 109,* 1– 15.
Matheron, G. (1963). Principles of geostatistics. *Economic Geology, 58(8),* 1246–1266.
McLennan, J. (2007). *The decision of stationarity.* Ph.D. Thesis, University of Alberta, Edmonton, Canada.
Munroe, M. J., & Deutsch, C. V. (2008). A new way to calibrate distance function uncertainty. *Center for Computational Geostatistics Annual Report 10. University of Alberta. Paper 302*.
Osher, S., & Fedkiw, R. (2003). Signed distance functions. In *Level set methods and dynamic implicit surfaces* (pp. 17–22). Springer.
Rossi, M. E., & Deutsch, C. V. (2013). Mineral resource estimation. In (pp. 106, 217). Springer Science & Business Media.
Silva, D. A. (2015). Signed distance function modeling with multiple categories. Retrieved from http://geostatisticslessons.com/lessons/signeddistancefunctions.
Silva, D. A., & Deutsch, C. V. (2012). Modeling multiple rock types with distance functions: Methodology and software. *Center for Computational Geostatistics Annual Report 14. University of Alberta. Paper 307*.
Stewart, M., Lacey, J. de, Hodkiewicz, P., & Lane, R. (2014). Grade estimation from radial basis functions–how does it compare with conventional geostatistical estimation. In *Ninth international mining geology conference, adelaide, australia* (Vol. 129, p. 139).
Wilde, B. J., & Deutsch, C. V. (2011). A new way to calibrate distance function uncertainty. *Center for Computational Geostatistics Annual Report 17. University of Alberta. Paper 118*.
