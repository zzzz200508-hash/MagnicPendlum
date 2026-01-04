use std::error::Error;
use crate::structs::Vector3D;
use std::fs;
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
pub enum MagnetDirection {
    Positive,//吸引
    Negative,//排斥
}

impl MagnetDirection {
    pub(crate) fn tostring(&self)  -> String {
        match self {
            MagnetDirection::Positive => return ("吸引".to_string()),
            MagnetDirection::Negative => return ("排斥".to_string())
        }
    }
}

#[derive(Debug, Deserialize, Serialize)]
pub enum Approximate{
    SmallAngle,//应用小角近似
    Rigour,//严格计算
}

#[derive(Debug, Deserialize, Serialize)]
pub struct Magnet {
    pub position: Vector3D,
    pub velocity: Vector3D,
    pub direction: MagnetDirection,
    pub strength: f64,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct PendulumInfo {
    pub suspension_point: Vector3D,
    pub mass: f64,
    pub approximate: Approximate,
}

pub struct PendulumPhysics {
    pub position: Vector3D,
    pub velocity: Vector3D,
}

impl Magnet {
    pub fn new(position: Vector3D, velocity: Vector3D, direction: MagnetDirection, strength: f64) -> Magnet {
        Magnet {
            position: position,
            velocity: velocity,
            direction: direction,
            strength: strength,
        }
    }
}
#[derive(Debug, Deserialize)]
pub struct SystemConfig {
    pub magnets: Vec<Magnet>,
    pub pendulum: PendulumInfo,
}
pub fn load_system_config(file_path: &str) -> Result<SystemConfig, Box<dyn Error>> {
    let content = fs::read_to_string(file_path)?;

    let system_config:SystemConfig= serde_json::from_str(&content)?;

    println!("成功加载了 {} 个磁铁。", system_config.magnets.len());

    Ok(system_config)
}