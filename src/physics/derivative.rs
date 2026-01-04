use crate::structs::*;
use crate::RK4::{OdeSystem};
use crate::physics::physicial_structs::*;

pub struct MagneticPendulumSystem {
    pub magnets: Vec<Magnet>,
    pub pendulum: PendulumInfo,
    pub friction_coefficient: f64, //阻尼系数
    pub gravity_accel: f64,        //重力加速度
}

impl MagneticPendulumSystem {
    pub fn new(magnets: Vec<Magnet>, pendulum: PendulumInfo, friction: f64, g: f64) -> Self {
        Self {
            magnets,
            pendulum,
            friction_coefficient: friction,
            gravity_accel: g,
        }
    }
}

impl OdeSystem<Vec<Vector3D>> for MagneticPendulumSystem {
    fn derivatives(&self, _t: f64, state: &Vec<Vector3D>) -> Vec<Vector3D> {
        let position = state[0];
        let velocity = state[1];

        let mut total_force = Vector3D::new(0.0, 0.0, 0.0);

        //重力,回复力
        match self.pendulum.approximate {
            Approximate::SmallAngle => {
                //小角近似
                let vector_to_suspension = self.pendulum.suspension_point - position;
                let k = self.pendulum.mass * self.gravity_accel / (self.pendulum.suspension_point.z.abs() + 0.1);//增加一点小角近似下的摆动平面高度,使后面磁铁距离更安全.
                total_force = total_force + vector_to_suspension * k;
            },
            Approximate::Rigour => {
                //严格计算
                let gravity_force = Vector3D::new(0.0, 0.0, -self.gravity_accel * self.pendulum.mass);
                total_force = total_force + gravity_force;
            }
        }

        //磁力
        for mag in &self.magnets {
            let r_vec = mag.position - position; //从摆球指向磁铁
            let dist = r_vec.length();

            //安全保证
            let safe_dist = if dist < 1e-4 { 1e-4 } else { dist };

            //磁单极子^3，偶极子是^5)
            let force_magnitude = mag.strength / (safe_dist * safe_dist * safe_dist);

            let mut magnetic_force = r_vec * force_magnitude;

            // 根据磁极方向调整力的方向
            match mag.direction {
                MagnetDirection::Positive => {}, //默认无需变号
                MagnetDirection::Negative => {
                    magnetic_force = magnetic_force * -1.0;
                }
            }

            total_force = total_force + magnetic_force;
        }

        //阻尼,加快收敛
        // F_d = -c * v
        let damping_force = velocity * (-self.friction_coefficient);
        total_force = total_force + damping_force;

        //加速度
        let mut acceleration = total_force / self.pendulum.mass;

        //约束处理
        //将加速度投影到以绳子为法线的切平面上。
        if let Approximate::Rigour = self.pendulum.approximate {
            let rope_vec = position - self.pendulum.suspension_point;
            let rope_len = rope_vec.x.hypot(rope_vec.y).hypot(rope_vec.z); //手动算模长

            if rope_len > 1e-6 {
                let rope_unit = rope_vec / rope_len;

                //绳子方向上的分量
                let radial_accel_mag = acceleration.x * rope_unit.x + acceleration.y * rope_unit.y + acceleration.z * rope_unit.z;
                let radial_accel = rope_unit * radial_accel_mag;

                // 切向加速度 = 总加速度 - 径向加速度
                // 还要考虑向心加速度修正吗？RK4处理速度导数，几何约束最好通过校正位置或拉格朗日乘子法，
                // 这里仅去除径向合力分量以模拟刚性杆支撑。
                acceleration = acceleration - radial_accel;

                // 注意：在长时间模拟中，纯切向加速度可能会导致数值漂移（摆长变长），
                // 实际模拟中通常需要在主循环里做一个 position.normalize() 的位置校正，
                // 或者在 acceleration 中加入一个向心力项 -v^2/L * n。
                let v_sq = velocity.x*velocity.x + velocity.y*velocity.y + velocity.z*velocity.z;
                let centripetal_accel = rope_unit * (-v_sq / rope_len);
                acceleration = acceleration + centripetal_accel;
            }
        }

        vec![velocity, acceleration]
    }
}