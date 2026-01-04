mod structs;
mod physics;
mod RK4;

use crate::physics::*;
use std::sync::Arc;
use std::fs::File;
use std::path::Path;
use image::{ImageBuffer, RgbImage, Rgb};
use rayon::prelude::*; // 引入并行迭代器
use indicatif::{ProgressBar, ProgressStyle}; // 引入进度条

use crate::structs::Vector3D;
use crate::derivative::MagneticPendulumSystem;
use crate::simulation::{SimConfig, SimResult, EndReason};

// ==========================================
// 全局配置：图像分辨率
// ==========================================
const WIDTH: u32 = 3000;
const HEIGHT: u32 = 3000;
const OUTPUT_FILENAME: &str = "magnetic_fractal.png";

// ==========================================
// 调色板 (对应不同的磁铁索引)
// ==========================================
// 颜色格式: [R, G, B]
const PALETTE: [[u8; 3]; 6] = [
    [255, 65, 54],   // Red
    [46, 204, 64],   // Green
    [0, 116, 217],   // Blue
    [255, 220, 0],   // Yellow
    [177, 13, 201],  // Purple
    [255, 133, 27],  // Orange
];

fn main() {
    println!("=== Magnetic Pendulum Fractal Generator ===");

    // 1. 加载物理系统配置
    println!("Loading configuration...");
    let config_data = match physicial_structs::load_system_config("config.json") {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Error loading config: {}", e);
            return;
        }
    };

    // 构建物理系统
    // 这里假设阻尼系数 friction=0.2, 重力 g=9.8，你也可以从 JSON 读取
    let system = MagneticPendulumSystem::new(
        config_data.magnets,
        config_data.pendulum,
        0.01, // 阻尼系数 (关键参数：越小图像越混沌)
        9.8, // 重力加速度
    );

    // 2. 预计算分析 (优化步骤)
    println!("Pre-calculating energy thresholds and bounds...");

    // 计算每个磁铁的逃逸能量 (李雅普诺夫判定)
    let escape_thresholds = lyapunov_function::calculate_escape_thresholds(&system);

    // 自动规划物理坐标范围 (padding 1.2 倍)
    let bounds = lyapunov_function::suggest_simulation_bounds(&system, 0.5, 0.5); // padding=0.2, height_ratio=0.2
    let (min_x, max_x, min_y, max_y) = bounds;

    println!("Physics Bounds: X[{:.2}, {:.2}], Y[{:.2}, {:.2}]", min_x, max_x, min_y, max_y);
    println!("Escape Thresholds: {:?}", escape_thresholds);

    // 3. 准备模拟参数
    // 为了线程安全，我们不需要把 system 包在 Arc 里，因为它是只读的，且引用实现了 Sync
    // 但为了方便闭包调用，直接引用即可

    let sim_config = SimConfig {
        time_step: 0.01,        // dt
        max_steps: 5000,        // 最大迭代步数
        capture_radius: 0.15,   // 物理接触半径
        basin_radius: 2.0,      // 能量判定半径 (进入此范围开始检查能量)
        check_interval: 5,     // 每10步检查一次
    };

    //自适应着色器
    fn hsl_to_rgb(h: f64, s: f64, l: f64) -> [u8; 3] {
        let c = (1.0 - (2.0 * l - 1.0).abs()) * s;
        let x = c * (1.0 - ((h / 60.0) % 2.0 - 1.0).abs());
        let m = l - c / 2.0;

        let (r_prime, g_prime, b_prime) = if h < 60.0 { (c, x, 0.0) }
        else if h < 120.0 { (x, c, 0.0) }
        else if h < 180.0 { (0.0, c, x) }
        else if h < 240.0 { (0.0, x, c) }
        else if h < 300.0 { (x, 0.0, c) }
        else { (c, 0.0, x) };

        [
            ((r_prime + m) * 255.0) as u8,
            ((g_prime + m) * 255.0) as u8,
            ((b_prime + m) * 255.0) as u8,
        ]
    }

    // 4. 创建图像缓冲区
    // 我们将其展平为 Vec<(u32, u32, Rgb<u8>)> 或者直接并行生成像素数据
    // 为了利用 image 库和 rayon，我们创建一个扁平的 buffer
    let mut img_buffer: Vec<u8> = vec![0; (WIDTH * HEIGHT * 3) as usize];

    // 初始化进度条
    let bar = ProgressBar::new((WIDTH * HEIGHT) as u64);
    bar.set_style(ProgressStyle::default_bar()
        .template("[{elapsed_precise}] {bar:40.cyan/blue} {pos}/{len} ({eta})")
        .unwrap()
        .progress_chars("=>-"));

    // 5. 并行计算循环 (Rayon Magic)
    // chunks_exact_mut(3) 每次拿到一个 RGB 像素的 slice
    img_buffer.par_chunks_exact_mut(3)
        .enumerate()
        .for_each(|(i, pixel)| {
            let px = i as u32 % WIDTH;
            let py = i as u32 / WIDTH;

            // 映射坐标 (2D平面投影)
            let fx = min_x + (max_x - min_x) * (px as f64 / WIDTH as f64);
            let fy = max_y - (max_y - min_y) * (py as f64 / HEIGHT as f64);

            // ====================================================
            // 核心修改：Z 轴坐标计算 (Projection Logic)
            // ====================================================
            let start_pos_opt = match system.pendulum.approximate {
                crate::physicial_structs::Approximate::SmallAngle => {
                    // 要求：小角近似下，默认 z=0.1
                    Some(Vector3D::new(fx, fy, 0.1))
                },
                crate::physicial_structs::Approximate::Rigour => {
                    // 要求：严格模式，投影到球面上
                    let suspension = system.pendulum.suspension_point;
                    let l = suspension.z; // 假设摆长等于悬挂高度 (或从 config 读取摆长)

                    let r_sq = (fx - suspension.x).powi(2) + (fy - suspension.y).powi(2);

                    // 检查是否超出摆长 (即点在球的投影之外)
                    if r_sq > l * l {
                        None // 无效点，无法投影到球面上
                    } else {
                        // 求解球面方程: (z - zs)^2 = L^2 - r^2
                        // z = zs - sqrt(L^2 - r^2)  (取下半球面)
                        let z = suspension.z - (l * l - r_sq).sqrt();
                        Some(Vector3D::new(fx, fy, z))
                    }
                }
            };

            // 如果坐标无效 (None)，直接渲染背景色并跳过模拟
            if let Some(start_pos) = start_pos_opt {
                let result = simulation::run_simulation(
                    &system,
                    start_pos,
                    &sim_config,
                    &escape_thresholds,
                    bounds
                );

                // ... (着色逻辑不变) ...
                let color = match result.captured_magnet_index {
                    Some(idx) => {
                        let total_magnets = system.magnets.len() as f64;

                        // 1. 动态计算色相 (Hue)
                        // 让颜色均匀分布在 360 度色环上
                        // 例如 3个磁铁：0°, 120°, 240° (红, 绿, 蓝)
                        // 例如 4个磁铁：0°, 90°, 180°, 270° (红, 黄-绿, 青, 紫)
                        let hue = (idx as f64 / total_magnets) * 360.0;

                        // 2. 计算阴影 (Lightness)
                        // 步数越少越亮 (0.6)，步数越多越暗 (0.1)
                        // 使用 sqrt 让过渡更柔和
                        let convergence_ratio = result.steps_taken as f64 / sim_config.max_steps as f64;
                        let lightness = 0.6 * (1.0 - convergence_ratio.sqrt()).max(0.1);

                        // 3. 饱和度 (Saturation) 设为 1.0 (最鲜艳)
                        hsl_to_rgb(hue, 1.0, lightness)
                    },
                    None => [0, 0, 0] // 未收敛显示黑色
                };
            }
        });

    bar.finish_with_message("Simulation Complete!");

    // 7. 保存图像
    println!("Saving image to {}...", OUTPUT_FILENAME);
    let image: RgbImage = ImageBuffer::from_raw(WIDTH, HEIGHT, img_buffer).unwrap();
    image.save(Path::new(OUTPUT_FILENAME)).unwrap();

    println!("Done! Check the output file.");
}
