use crate::structs::Vector3D;
use crate::RK4::RungeKuttaSolver;
use crate::physics::derivative::MagneticPendulumSystem;
use crate::physics::physicial_structs::Approximate;
use crate::physics::lyapunov_function; // 引入这一步新增的计算模块

#[derive(Clone, Copy)]
pub struct SimConfig {
    pub time_step: f64,
    pub max_steps: usize,
    pub capture_radius: f64,      // 物理捕获半径（用于最终停止）
    pub basin_radius: f64,        // 盆地判定半径（用于能量判定，通常比 capture_radius 大）
    pub check_interval: usize,    // 每隔多少步进行一次昂贵的能量/收敛检查
}

#[derive(Debug, Clone)]
pub struct SimResult {
    pub captured_magnet_index: Option<usize>,
    pub final_position: Vector3D,
    pub steps_taken: usize,
    pub end_reason: EndReason, // 用于调试：是撞上了？还是能量耗尽了？
}

#[derive(Debug, Clone, PartialEq)]
pub enum EndReason {
    MaxStepsReached,
    PhysicalCapture,  // 速度极小且撞上磁铁
    EnergyTrap,       // 能量低于逃逸阈值 (李雅普诺夫判定)
    OutOfBounds,      // 飞出模拟边界 (发散)
}

/// 运行单个点的模拟
///
/// * `escape_thresholds`: 从 main 中预先算好的逃逸能量数组，避免重复计算
/// * `bounds`: (min_x, max_x, min_y, max_y) 模拟的安全边界
pub fn run_simulation(
    system: &MagneticPendulumSystem,
    start_position: Vector3D,
    config: &SimConfig,
    escape_thresholds: &[f64],
    bounds: (f64, f64, f64, f64)
) -> SimResult {

    let initial_state = vec![start_position, Vector3D::new(0.0, 0.0, 0.0)];
    let mut solver = RungeKuttaSolver::new(0.0, initial_state);

    // 严格模式下的摆长约束
    let pendulum_length = if let Approximate::Rigour = system.pendulum.approximate {
        (start_position - system.pendulum.suspension_point).length()
    } else {
        0.0
    };

    // 预计算平方值以加速比较
    let capture_r_sq = config.capture_radius * config.capture_radius;
    let basin_r_sq = config.basin_radius * config.basin_radius;
    let (min_x, max_x, min_y, max_y) = bounds;

    for step in 0..config.max_steps {
        //RK4 步进
        solver.step(system, config.time_step);

        //几何约束
        if let Approximate::Rigour = system.pendulum.approximate {
            let current_pos = solver.state[0];
            let suspension = system.pendulum.suspension_point;
            let rel_vec = current_pos - suspension;
            // 消除数值漂移
            let corrected_rel = rel_vec.scale(pendulum_length / rel_vec.length());
            solver.state[0] = suspension + corrected_rel;
        }

        //检查
        if step % config.check_interval == 0 {
            let current_pos = solver.state[0];
            let current_vel = solver.state[1];

            //边界检查（已移除对运动中摆的边界检查，更改为更宽泛的检查）
            if current_pos.x < 2.0*min_x || current_pos.x > 2.0*max_x ||
                current_pos.y < 2.0*min_y || current_pos.y > 2.0*max_y {
                return SimResult {
                    captured_magnet_index: None,
                    final_position: current_pos,
                    steps_taken: step,
                    end_reason: EndReason::OutOfBounds,
                };
            }

            // B. 寻找最近的磁铁
            let mut closest_magnet_idx = None;
            let mut min_dist_sq = f64::MAX;

            for (i, magnet) in system.magnets.iter().enumerate() {
                let dist_sq = (current_pos - magnet.position).length_squared();
                if dist_sq < min_dist_sq {
                    min_dist_sq = dist_sq;
                    closest_magnet_idx = Some(i);
                }
            }

            if let Some(idx) = closest_magnet_idx {

                // 判定1: 靠近中心且速度极低（已被优化）
                // 这里我们简化一下，如果离得非常近 (capture_radius)，直接算捕获
                // 因为在 singularity 附近力非常大，步长不够小会导致弹飞，不如直接捕获
                // min_dist_sq < capture_r_sq {
                //    return SimResult {
                //        captured_magnet_index: Some(idx),
                //        final_position: current_pos,
                //        steps_taken: step,
                //        end_reason: EndReason::PhysicalCapture,
                //    };
                //}

                // 判定2: 李雅普诺夫能量判定 (Advanced)
                // 只有当粒子在“盆地范围”内时才检查能量
                if min_dist_sq < basin_r_sq {
                    // 计算当前总能量 E = T + V
                    let current_energy = lyapunov_function::calculate_total_energy(
                        system, current_pos, current_vel
                    );

                    // 获取该磁铁的逃逸阈值
                    let escape_e = escape_thresholds[idx];

                    // 如果 E < E_escape，则粒子被永久捕获
                    if current_energy < escape_e {
                        return SimResult {
                            captured_magnet_index: Some(idx),
                            final_position: current_pos, // 注意：此时可能还没到中心，但已确认归属
                            steps_taken: step,
                            end_reason: EndReason::EnergyTrap,
                        };
                    }
                }
            }
        }
    }

    //超时
    SimResult {
        captured_magnet_index: None,
        final_position: solver.state[0],
        steps_taken: config.max_steps,
        end_reason: EndReason::MaxStepsReached,
    }
}