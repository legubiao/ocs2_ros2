# Sequential Quadratic Programming

Introduction to the Sequential Quadratic Programming (SQP) algorithm on Wikipedia: [Sequential Quadratic Programming](https://en.wikipedia.org/wiki/Sequential_quadratic_programming)

序列二次规划（Sequential Quadratic Programming，简称SQP）是一种用于求解非线性优化问题的迭代算法。它通过将复杂的非线性优化问题转化为一系列较简单的二次规划（Quadratic Programming，QP）子问题来逐步逼近最优解。

### 基本原理
1. **初始解**：从一个初始解开始。
2. **线性化**：在当前解附近，对非线性目标函数和约束函数进行线性化处理，通常使用泰勒展开。
3. **构造二次规划问题**：将线性化后的问题转化为一个二次规划问题。
4. **求解二次规划问题**：求解这个二次规划问题，得到一个新的解。
5. **迭代**：将新的解作为新的初始解，重复上述步骤，直到收敛到最优解。

### 应用
SQP算法广泛应用于各种非线性优化问题，包括但不限于：
- **机器人控制**：优化控制策略以实现最佳性能。
- **路径规划**：计算机器人或车辆的最优路径。
- **经济学和金融学**：优化投资组合和风险管理策略。

### 优缺点
- **优点**：
    - 能处理复杂的非线性约束问题。
    - 收敛速度较快，通常能找到全局最优解。
- **缺点**：
    - 计算复杂度较高。
    - 需要对目标函数和约束函数进行精确的线性化和二次近似。