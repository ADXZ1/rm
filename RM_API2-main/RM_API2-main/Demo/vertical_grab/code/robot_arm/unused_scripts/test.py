import plotly.express as px
import pandas as pd
import numpy as np

# 生成模拟数据（36组样本）
np.random.seed(42)
data = pd.DataFrame({
    "Temperature": np.random.uniform(50, 150, 36),
    "Pressure": np.random.uniform(0.1, 10, 36),
    "Concentration": np.random.uniform(0.5, 5, 36),
    "Time": np.random.uniform(1, 24, 36),
    "Growth Rate": np.random.uniform(0.1, 2.0, 36)
})

# 绘制交互式3D图
fig = px.scatter_3d(
    data,
    x='Temperature',
    y='Pressure',
    z='Concentration',
    color='Time',          # 第四维度：颜色
    size='Growth Rate',    # 第五维度：点大小
    color_continuous_scale='Viridis',
    title="5D Crystal Growth Kinetics"
)
fig.show()