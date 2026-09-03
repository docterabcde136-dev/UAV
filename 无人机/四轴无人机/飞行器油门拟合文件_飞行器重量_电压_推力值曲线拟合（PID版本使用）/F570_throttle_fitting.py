import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression


# 油门基础值和步长
base_throttle = 48
step = 50

# 电压列表
voltages = [10,10.5, 11, 11.5, 12, 12.5]

# 各电压下的推力数据(油门从48开始,以50为步长递增)
thrust_data = [
    [0, 0, 0, 4, 9,  15, 22, 30, 39, 48, 57, 65, 76,  86,  100, 112, 125, 143, 157, 174, 187, 205, 220],  # 10V
    [0, 0, 2, 6, 12, 19, 26, 35, 44, 53, 62, 72, 83,  95,  108, 122, 138, 154, 168, 188, 205, 225, 242],  # 10.5V
    [0, 0, 2, 7, 13, 20, 28, 37, 48, 58, 68, 81, 92,  105, 121, 138, 153, 169, 185, 203, 223, 243, 262],  # 11V
    [0, 0, 2, 7, 14, 22, 30, 40, 50, 62, 74, 85, 97,  113, 128, 146, 166, 179, 200, 217, 238, 260, 280],  # 11.5V
    [0, 0, 2, 8, 15, 24, 33, 44, 54, 66, 79, 92, 104, 121, 137, 159, 176, 194, 211, 239, 258, 280, 300],  # 12V
    [0, 0, 3, 8, 16, 26, 35, 46, 57, 70, 84, 98, 112, 130, 147, 170, 189, 207, 223, 252, 275, 293, 319]   # 12.5
]
#max 1148

# 生成油门、推力、电压数据
throttle_values = []
thrust_values = []
voltage_values = []

for i, voltage in enumerate(voltages):
    for j, thrust in enumerate(thrust_data[i]):
        throttle_values.append(base_throttle + j * step)
        thrust_values.append(thrust)
        voltage_values.append(voltage)

# 转换为 NumPy 数组
throttle_values = np.array(throttle_values)
thrust_values = np.array(thrust_values)
voltage_values = np.array(voltage_values)

# 组合输入特征 (推力, 电压)
X = np.column_stack((thrust_values, voltage_values))
y = throttle_values  # 目标变量：油门值

# 生成二次多项式特征
poly = PolynomialFeatures(degree=2, include_bias=False)  # 二次多项式
X_poly = poly.fit_transform(X)

# 线性回归拟合
model = LinearRegression()
model.fit(X_poly, y)

# 获取拟合系数
coefficients = model.coef_
intercept = model.intercept_

# 打印拟合公式
feature_names = ["F", "V", "F^2", "FV", "V^2"]
formula_terms = [f"{coeff:.5f} * {name}" for coeff, name in zip(coefficients, feature_names)]
formula = f"T = {intercept:.5f} + " + " + ".join(formula_terms)
print("拟合公式：")
print(formula)

# 画出曲面图
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection='3d')

# 生成网格数据
F_range = np.linspace(min(thrust_values), max(thrust_values), 50)
V_range = np.linspace(min(voltage_values), max(voltage_values), 50)
F_mesh, V_mesh = np.meshgrid(F_range, V_range)
X_mesh = np.column_stack((F_mesh.ravel(), V_mesh.ravel()))
X_mesh_poly = poly.transform(X_mesh)
T_pred = model.predict(X_mesh_poly).reshape(F_mesh.shape)

# 画出曲面
ax.plot_surface(F_mesh, V_mesh, T_pred, cmap='viridis', alpha=0.7)
ax.scatter(thrust_values, voltage_values, throttle_values, color='r', label="真实数据")  # 真实数据点

# 设置坐标轴标签
ax.set_xlabel("thrust/g") # 推力,单位克g
ax.set_ylabel("vol")      # 电压值,单位V
ax.set_zlabel("throttle") # 油门值
ax.set_title("throttle-thrust-vol")

plt.legend()
plt.show()