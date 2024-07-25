import numpy as np
import matplotlib.pyplot as plt

# 读取参数文件
with open('../params1.txt', 'r') as f:
    parameter_lines = f.readlines()

# 读取数据点文件
data_points = np.loadtxt('../data/homework_data_1.txt', delimiter=' ', skiprows=0)

# 分离 x 和 y 坐标
x = data_points[:, 0]
y = data_points[:, 1]

# 确保参数行数与数据点行数相同
num_lines = min(len(parameter_lines), len(data_points))

# 初始化存储预测 y 值的列表
predicted_y = []

# 遍历每行数据进行预测
for i in range(num_lines):
    # 提取当前行的参数 a 和 b
    a, b = map(float, parameter_lines[i].strip().split())

    # 当前行的数据点
    current_x = x[i]
    
    # 预测的 y 值
    predicted_y.append(a * current_x + b)

# 绘制原始数据和预测数据的图像
plt.figure(figsize=(12, 8))

# 原始数据
plt.plot(x, y, 'b-', label='Original Data')

# 预测数据
plt.plot(x[:num_lines], predicted_y, 'r-', label='Predicted Data')
#plt.ylim(85, 103)
plt.xlabel('Day')
plt.ylabel('Price')
plt.title('Original vs Predicted Data')
plt.grid(True)
plt.legend()
plt.show()
