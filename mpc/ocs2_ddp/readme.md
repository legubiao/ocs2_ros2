# Differential Dynamic Programming

Introduction to the Differential Dynamic Programming (DDP) algorithm on Wikipedia: [Differential Dynamic Programming](https://en.wikipedia.org/wiki/Differential_dynamic_programming)


微分动态规划（Differential Dynamic Programming，简称DDP）是一种用于求解最优控制问题的算法。它基于动态规划的思想，通过对系统的状态和控制变量进行微分，来近似求解最优控制策略。DDP特别适用于非线性系统的优化问题。

### 基本原理
DDP的基本思想是利用贝尔曼最优性原理，将问题分解为一系列子问题，通过递归求解这些子问题来找到全局最优解。具体步骤如下：

1. **选定标称轨迹**：首先选择一个初始的标称轨迹，这个轨迹是对目标轨迹的一个近似。
2. **线性化系统**：在标称轨迹附近，对系统的状态和控制变量进行线性化处理，通常使用泰勒展开。
3. **求解二次优化问题**：在每个时间步上，求解一个二次优化问题，得到当前时间步的最优控制策略。
4. **更新轨迹**：利用当前时间步的最优控制策略，更新系统状态，得到新的标称轨迹。
5. **迭代**：重复上述步骤，直到轨迹收敛到最优解。

### 应用场景
DDP广泛应用于机器人控制、路径规划和动态系统优化等领域。例如，在机器人路径规划中，DDP可以用于计算机器人从起点到终点的最优路径，同时考虑到路径上的障碍物和动态环境的变化。