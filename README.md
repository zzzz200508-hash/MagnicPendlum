# Magnetic Pendulum Fractal Generator (Rust Implementation)

这是一个基于 Rust 语言编写的高性能磁摆（Magnetic Pendulum）混沌分形生成器。该程序通过数值模拟受重力和多点磁力共同作用下的刚体摆运动，绘制出精细的动力学吸引域（Basins of Attraction）分形图像。

## ✨ 核心特性 (Key Features)

### 1. 高精度物理内核

* **RK4 积分器**: 采用四阶 Runge-Kutta 方法（RK4）进行数值积分，保证了在混沌边缘区域的轨迹精度。
* **严格 3D 约束 (Rigorous Mode)**: 不同于常见的二维平面近似，本程序实现了**严格的球面约束**。
* 实时修正位置漂移，强制摆球位于  的球面上。
* 实时切除径向速度分量，保证能量守恒和物理真实性。


* **可变近似模型**: 支持 `SmallAngle`（小角近似）和 `Rigour`（严格刚体）两种模式切换。

### 2. 智能收敛判定 (Smart Convergence)

为了解决混沌系统模拟耗时久的问题，引入了基于**李雅普诺夫函数 (Lyapunov Function)** 的能量判定机制：

* **能量势阱检测**: 自动计算每个磁铁周围的“逃逸势能”（Saddle Point Energy）。
* **提前终止**: 一旦粒子进入某磁铁的势阱且总能量（动能+势能）低于逃逸阈值，程序判定其**物理上无法逃逸**，直接终止模拟。这比传统的“速度+距离”判定快数倍，且消除了伪影。

### 3. 自适应视觉渲染

* **动态 HSL 着色**: 摒弃了硬编码的调色板。程序根据磁铁数量  自动在 HSL 色环上均匀分配色相。无论是 3 个还是 100 个磁铁，颜色都能完美区分。
* **深度阴影 (Depth Shading)**: 像素亮度与收敛步数（Time to Convergence）挂钩。
* 亮色：快速被捕获的区域。
* 暗色/阴影：在混沌边缘徘徊很久的轨迹。
* 这种着色法直观地展示了相空间中的“地形”深浅。



### 4. 高性能并行计算

* **Rayon 并行加速**: 利用 Rust 的 `Rayon` 库，自动将百万级像素的计算任务分配到所有 CPU 核心上，极大缩短渲染时间。
* **预计算优化**: 模拟开始前预先计算所有磁铁的逃逸阈值和模拟边界，避免在循环中重复计算。

---

## 📐 物理模型 (Physics Model)

系统的动力学方程由以下力场叠加而成：

1. **重力 (Gravity)**:



在严格模式下，重力与绳张力共同作用，仅保留切向分量。
2. **磁力 (Magnetic Force)**:



其中  为磁铁强度。程序支持将磁铁放置在摆动平面下方 () 以避免奇点（Singularity），获得更平滑的力场。
3. **阻尼 (Damping)**:



线性空气阻力，通过 `config.json` 或代码中的摩擦系数进行调节。

---

## ⚙️ 配置文件说明 (Configuration)

程序通过 `config.json` 驱动。以下是一个标准的五边形磁铁布局示例：

```json
{
  "pendulum": {
    "suspension_point": {"x": 0.0, "y": 0.0, "z": 10.0},
    "mass": 1.0,
    "approximate": "Rigour" 
  },
  "magnets": [
    {
      "position": {"x": 3.0, "y": 0.0, "z": -0.3},
      "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
      "direction": "Positive",
      "strength": 200.0
    },
    {
      "position": {"x": 0.927, "y": 2.853, "z": -0.3},
      "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
      "direction": "Positive",
      "strength": 200.0
    },
    // ... 更多磁铁
  ]
}

```

* **`strength`**: 建议设置在 `100.0` ~ `200.0` 之间，以确保磁力足以抗衡重力回得力。
* **`z`**: 建议磁铁 Z 坐标略低于摆球最低点（例如 `-0.3`），防止除零错误并平滑图像。

---

## 🚀 快速开始

1. **依赖安装**:
确保 `Cargo.toml` 包含以下依赖：
```toml
[dependencies]
serde = { version = "1.0", features = ["derive"] }
serde_json = "1.0"
image = "0.24"
rayon = "1.7"
indicatif = "0.17"

```


2. **编译与运行**:
由于计算量巨大，**务必**使用 Release 模式运行：
```bash
cargo run --release

```


3. **输出**:
程序将在目录下生成 `magnetic_fractal.png`。

---

## 🖼️ 渲染效果示例

* **混沌边缘**: 在不同颜色交界处，可以看到精细的分形结构（Wada Basins）。
* **中心区域**: 如果参数设置得当，中心区域将呈现出复杂的纠缠态，反映出摆球在多个磁铁间极其敏感的平衡选择。

---
一些生成的图片示例：
<img width="2000" height="2000" alt="magnetic_fractal-3 1" src="https://github.com/user-attachments/assets/135b06cc-199d-4ad5-9529-7c4fffdaa887" />
<img width="2000" height="2000" alt="image" src="https://github.com/user-attachments/assets/5b1328f9-5444-4fd0-92af-9254add9d40f" />



*文档生成时间: 2026-01-04*
