use crate::structs::Vector3D;
use crate::physics::derivative::MagneticPendulumSystem;
use crate::physics::physicial_structs::{Approximate, MagnetDirection};

/// 计算系统的总势能 V(r)
/// 包括重力势能和磁势能
pub fn calculate_potential_energy(system: &MagneticPendulumSystem, pos: Vector3D) -> f64 {
    let mut pe = 0.0;

    // 1. 重力势能 (Gravitational Potential)
    match system.pendulum.approximate {
        Approximate::Rigour => {
            // V = m * g * z
            // 注意：这里假设 z=0 是势能零点。如果摆球在 z < 0 运动，势能为负。
            pe += system.pendulum.mass * system.gravity_accel * pos.z;
        },
        Approximate::SmallAngle => {
            // V = 1/2 * k * r^2
            // k = m * g / L
            let length = system.pendulum.suspension_point.z.abs();
            // 在小角近似中，通常只考虑 xy 平面的位移
            let r_sq = pos.x * pos.x + pos.y * pos.y;
            let k = system.pendulum.mass * system.gravity_accel / length;
            pe += 0.5 * k * r_sq;
        }
    }

    // 2. 磁势能 (Magnetic Potential)
    // 对应力 F = Strength / r^2
    // 势能 V = - Strength / r (对于吸引 Positive)
    // 势能 V = + Strength / r (对于排斥 Negative)
    for mag in &system.magnets {
        let dist = (pos - mag.position).length();
        // 加上一个小量防止除零 (虽然计算势能时摆球很难正好重合)
        let safe_dist = if dist < 1e-6 { 1e-6 } else { dist };

        let potential_term = mag.strength / safe_dist;

        match mag.direction {
            MagnetDirection::Positive => pe -= potential_term, // 势阱
            MagnetDirection::Negative => pe += potential_term, // 势垒
        }
    }

    pe
}

/// 计算动能 T = 1/2 * m * v^2
pub fn calculate_kinetic_energy(system: &MagneticPendulumSystem, velocity: Vector3D) -> f64 {
    0.5 * system.pendulum.mass * velocity.length_squared()
}

/// 计算总能量 E = T + V
pub fn calculate_total_energy(system: &MagneticPendulumSystem, pos: Vector3D, vel: Vector3D) -> f64 {
    calculate_kinetic_energy(system, vel) + calculate_potential_energy(system, pos)
}

/// 估算每个磁铁的逃逸势能阈值
///
/// 返回一个 Vec<f64>，索引对应 system.magnets 中的磁铁顺序。
///
/// 算法原理：
/// 对于磁铁 A，它与磁铁 B 之间存在一个势能“山脊”。
/// 我们在 A 和 B 的连线上采样，找到该连线上的势能最大值 V_max_AB (鞍点近似)。
/// 磁铁 A 的逃逸能量 E_escape_A = min(V_max_AB, V_max_AC, ...)
/// 即它所有逃逸路径中门槛最低的那一个。
pub fn calculate_escape_thresholds(system: &MagneticPendulumSystem) -> Vec<f64> {
    let mut thresholds = Vec::new();
    let sample_points = 50; // 连线采样点数

    for i in 0..system.magnets.len() {
        let current_mag = &system.magnets[i];

        // 如果是排斥磁铁，它是山峰不是山谷，没有“捕获”一说，设为负无穷
        if let MagnetDirection::Negative = current_mag.direction {
            thresholds.push(f64::NEG_INFINITY);
            continue;
        }

        let mut min_barrier_height = f64::INFINITY;
        let mut has_neighbor = false;

        for j in 0..system.magnets.len() {
            if i == j { continue; }
            let neighbor_mag = &system.magnets[j];

            // 我们只在 z=0 平面 (或磁铁所在平面) 寻找鞍点
            // 这是合理的近似，因为垂直方向通常是重力势壁
            let start = current_mag.position;
            let end = neighbor_mag.position;

            // 在连线上寻找最大势能（鞍点）
            let mut max_pe_on_link = f64::NEG_INFINITY;

            for k in 1..sample_points {
                let t = k as f64 / sample_points as f64;
                //线性插值
                let sample_pos = start.scale(1.0 - t) + end.scale(t);
                //手动插值:
                // let sample_pos = Vector3D::new(
                //     start.x + (end.x - start.x) * t,
                //     start.y + (end.y - start.y) * t,
                //     start.z + (end.z - start.z) * t
                // );

                let pe = calculate_potential_energy(system, sample_pos);
                if pe > max_pe_on_link {
                    max_pe_on_link = pe;
                }
            }

            if max_pe_on_link < min_barrier_height {
                min_barrier_height = max_pe_on_link;
            }
            has_neighbor = true;
        }

        // 如果是孤立磁铁，或者计算异常，给一个默认的高阈值（比如 0.0 或基于重力）
        if !has_neighbor {
            thresholds.push(0.0);
        } else {
            // 保险起见，稍微降低一点阈值 (0.95)，确保不会误判
            thresholds.push(min_barrier_height);
        }
    }

    thresholds
}

/// 自动规划合理的求解/绘图范围 (Bounding Box)（限制最高初始高度0.2l）
/// 返回 (min_x, max_x, min_y, max_y)
pub fn suggest_simulation_bounds(
    system: &MagneticPendulumSystem,
    padding_ratio: f64,
    height_limit_ratio: f64
) -> (f64, f64, f64, f64) {

    // 1. 计算基于磁铁分布的边界
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;

    if system.magnets.is_empty() {
        min_x = -1.0; max_x = 1.0; min_y = -1.0; max_y = 1.0;
    } else {
        for mag in &system.magnets {
            if mag.position.x < min_x { min_x = mag.position.x; }
            if mag.position.x > max_x { max_x = mag.position.x; }
            if mag.position.y < min_y { min_y = mag.position.y; }
            if mag.position.y > max_y { max_y = mag.position.y; }
        }
    }

    // 包含中心点
    min_x = min_x.min(0.0);
    max_x = max_x.max(0.0);
    min_y = min_y.min(0.0);
    max_y = max_y.max(0.0);

    // 基础宽高 + Padding
    let width = max_x - min_x;
    let height = max_y - min_y;
    let pad_x = if width == 0.0 { 1.0 } else { width * padding_ratio };
    let pad_y = if height == 0.0 { 1.0 } else { height * padding_ratio };

    let mut final_min_x = min_x - pad_x;
    let mut final_max_x = max_x + pad_x;
    let mut final_min_y = min_y - pad_y;
    let mut final_max_y = max_y + pad_y;

    // 2. 应用高度限制 (Height Constraint)
    // 只有在严格模式下，摆长限制才重要
    if let Approximate::Rigour = system.pendulum.approximate {
        let suspension_z = system.pendulum.suspension_point.z;
        // 假设摆长 L ≈ Suspension Z (即摆球刚好扫过 z=0 平面)
        let l = suspension_z;

        // 计算最大允许高度对应的水平半径 R
        // Height_max = ratio * L
        // Z_lowest = 0, Z_release = 0.2 * L
        // cos(theta) = (L - 0.2L) / L = 0.8
        // sin(theta) = sqrt(1 - 0.8^2) = 0.6
        // R_max = L * 0.6

        // 这里的 height_limit_ratio 就是 1/5 = 0.2
        let z_release_max = l * height_limit_ratio;
        // 对应的垂直距离 (从悬挂点向下) = L - z_release_max
        let dist_vertical = l - z_release_max;

        // 勾股定理求水平半径限制
        if dist_vertical < l && dist_vertical > 0.0 {
            let r_limit = (l * l - dist_vertical * dist_vertical).sqrt();

            // 将边界限制在 [-r_limit, r_limit] 之间
            // 注意：我们取交集（即取较小的范围），确保不超出用户要求的 1/5 高度角
            // 但如果磁铁在很远的地方，这样切会导致磁铁在图外。
            // 通常为了视觉效果，如果磁铁真的很远，我们应该优先显示磁铁，
            // 但既然你明确要求了“最大释放高度”，我们将强制裁剪或取舍。
            // 这里我们采取“取并集”还是“取交集”？
            // 既然是为了生成分形图，通常是想看在这个高度限制内，点的归属。
            // 所以我们限制视图框不超过这个圆。

            final_min_x = final_min_x.max(-r_limit);
            final_max_x = final_max_x.min(r_limit);
            final_min_y = final_min_y.max(-r_limit);
            final_max_y = final_max_y.min(r_limit);
        }
    }

    (final_min_x, final_max_x, final_min_y, final_max_y)
}