# Successive linear programming

Introduction to the Successive linear programming (SLP) algorithm on Wikipedia: [Sequential Linear Programming](https://en.wikipedia.org/wiki/Sequential_linear_programming)

在最优化控制领域，SLP通常指的是**序列线性规划**（Sequential Linear Programming）。这是一种用于求解非线性优化问题的技术。SLP通过将非线性问题分解为一系列线性子问题来逐步逼近最优解。

### 基本原理
1. **初始解**：从一个初始解开始。
2. **线性化**：在当前解附近，对非线性问题进行线性化处理，通常使用泰勒展开。
3. **求解线性问题**：求解线性化后的子问题，得到一个新的解。
4. **迭代**：将新的解作为新的初始解，重复上述步骤，直到收敛到最优解。

### 应用
SLP广泛应用于各种非线性优化问题，包括但不限于：
- **机器人路径规划**：计算机器人从起点到终点的最优路径。
- **控制系统设计**：优化控制器参数以实现最佳系统性能。
- **经济学和金融学**：优化投资组合和风险管理策略。

### 优缺点
- **优点**：SLP算法相对简单，不需要高阶导数信息，适用于大规模问题。
- **缺点**：可能会陷入局部最优解，收敛速度可能较慢。