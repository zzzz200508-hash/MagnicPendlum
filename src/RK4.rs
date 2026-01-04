use crate::structs;
use crate::structs::Vector3D;

pub trait VectorSpace: Sized + Clone {
    // 向量加法: self + other
    fn add(&self, other: &Self) -> Self;

    // 标量乘法: self * scalar
    fn scale(&self, scalar: f64) -> Self;
}

// 为 Vec<f64> 实现这个特征 (动态维度)
impl VectorSpace for Vec<f64> {
    fn add(&self, other: &Self) -> Self {
        assert_eq!(self.len(), other.len(), "向量维度必须一致");
        self.iter().zip(other.iter())
            .map(|(a, b)| a + b)
            .collect()
    }

    fn scale(&self, scalar: f64) -> Self {
        self.iter().map(|x| x * scalar).collect()
    }
}

impl VectorSpace for Vec<Vector3D>{
    fn add(&self, other: &Self) -> Self {
        assert_eq!(self.len(), other.len(), "向量维度必须一致");
        self.iter().zip(other.iter())
            .map(|(a, b)| *a + *b)
            .collect()
    }

    fn scale(&self, scalar: f64) -> Self {
        self.iter().map(|x| *x * scalar).collect()
    }
}

// T 代表状态类型，它必须满足 VectorSpace 特征
pub trait OdeSystem<T: VectorSpace> {
    fn derivatives(&self, t: f64, y: &T) -> T;
}

pub struct RungeKuttaSolver<T> {
    pub t: f64,
    pub state: T, // 泛型状态，不再是硬编码的数组或 Vec
}

impl<T: VectorSpace> RungeKuttaSolver<T> {
    // 初始化
    pub fn new(t0: f64, y0: T) -> Self {
        Self {
            t: t0,
            state: y0,
        }
    }

    // 核心：泛型 RK4 步进
    // S 是实现了 OdeSystem<T> 的物理系统
    pub fn step<S: OdeSystem<T>>(&mut self, system: &S, dt: f64) {
        let y = &self.state;
        let t = self.t;

        // k1 = f(t, y)
        let k1 = system.derivatives(t, y);

        // k2 = f(t + dt/2, y + k1 * dt/2)
        // 利用 trait 中的 add 和 scale 方法
        let k2_state = y.add(&k1.scale(0.5 * dt));
        let k2 = system.derivatives(t + 0.5 * dt, &k2_state);

        // k3 = f(t + dt/2, y + k2 * dt/2)
        let k3_state = y.add(&k2.scale(0.5 * dt));
        let k3 = system.derivatives(t + 0.5 * dt, &k3_state);

        // k4 = f(t + dt, y + k3 * dt)
        let k4_state = y.add(&k3.scale(dt));
        let k4 = system.derivatives(t + dt, &k4_state);

        // y_{n+1} = y + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
        // 这一步利用线性组合
        let delta = k1
            .add(&k2.scale(2.0))
            .add(&k3.scale(2.0))
            .add(&k4)
            .scale(dt / 6.0);

        self.state = y.add(&delta);
        self.t += dt;
    }
}