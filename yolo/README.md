# YOLO 训练文件夹

此文件夹用于存放 YOLO 模型训练相关的文件。

## 文件夹结构

```
yolo/
├── yolo11n.pt          # YOLO11 预训练模型（需要下载）
├── dataset/            # 数据集文件夹
│   ├── images/         # 训练图片
│   ├── labels/         # 标注文件
│   └── data.yaml       # 数据集配置文件
├── dataset_/           # 备用数据集文件夹
│   ├── images/         # 训练图片
│   ├── labels/         # 标注文件
│   └── data.yaml       # 数据集配置文件
└── README.md           # 本文件
```

## 使用说明

1. **下载预训练模型**：
   - 需要下载 `yolo11n.pt` 预训练模型文件
   - 可以从 Ultralytics 官方下载：https://github.com/ultralytics/ultralytics

2. **准备数据集**：
   - 将训练图片放入 `dataset/images/` 或 `dataset_/images/`
   - 将标注文件放入 `dataset/labels/` 或 `dataset_/labels/`
   - 配置 `data.yaml` 文件

3. **训练模型**：
   - 训练结果会保存在 `runs/train/` 目录下
   - 最佳模型权重保存在 `runs/train/bottle/weights/best.pt`

## 注意事项

- 原始配置指向 `G:/rm/yolo/`，如果需要在 G 盘使用，请相应调整路径
- 如果数据集在其他位置，请更新训练脚本中的路径配置


