use anyhow::Result;
use rclrs::*;
use sensor_msgs::msg::Imu as ImuMsg;
use geometry_msgs::msg::Vector3;
use builtin_interfaces::msg::Time;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;
use std::fmt::Debug;
mod imu;
use imu::{Mpu6500, SensorData, MockI2c};
use embedded_hal::i2c::{I2c, ErrorType};

struct IMUNode<I2C> {
    _node: rclrs::Node,
    imu_publisher: Arc<rclrs::Publisher<ImuMsg>>,
    mpu: Arc<Mutex<Mpu6500<I2C>>>,
}

impl<I2C> IMUNode<I2C>
where
    I2C: I2c + ErrorType + Send + 'static,
    I2C::Error: Debug + 'static,
{
    fn new(executor: &Executor, mut mpu: Mpu6500<I2C>) -> Result<Self> {
        let node = executor.create_node("imu_node")?;
        
        // Initialize the MPU-6500
        mpu.setup().map_err(|e| anyhow::anyhow!("Failed to setup MPU-6500: {:?}", e))?;
        
        // Create publisher for IMU data
        let imu_publisher = Arc::new(node.create_publisher::<ImuMsg>("/imu/data")?);
        
        let mpu_arc = Arc::new(Mutex::new(mpu));
        let mpu_clone = Arc::clone(&mpu_arc);
        let publisher_clone = Arc::clone(&imu_publisher);
        let node_clone = Arc::new(node.clone());
        
        // Spawn a thread to read and publish IMU data periodically
        thread::spawn(move || {
            loop {
                thread::sleep(Duration::from_millis(100)); // 10 Hz update rate
                
                if let Ok(mut mpu_guard) = mpu_clone.lock() {
                    match mpu_guard.read_sensor_data() {
                        Ok(data) => {
                            Self::publish_imu_data(&publisher_clone, &node_clone, &data);
                        }
                        Err(e) => {
                            eprintln!("Failed to read IMU data: {:?}", e);
                        }
                    }
                }
            }
        });
        
        Ok(IMUNode {
            _node: node,
            imu_publisher,
            mpu: mpu_arc,
        })
    }
    
    fn publish_imu_data(
        publisher: &rclrs::Publisher<ImuMsg>,
        node: &rclrs::Node,
        data: &SensorData,
    ) {
        let now = node.get_clock().now();
        
        // Convert accelerometer data from g to m/s^2
        let (gx, gy, gz) = data.accel_to_g();
        const G_TO_MS2: f64 = 9.80665;
        let linear_acceleration = Vector3 {
            x: (gx as f64) * G_TO_MS2,
            y: (gy as f64) * G_TO_MS2,
            z: (gz as f64) * G_TO_MS2,
        };
        
        // Convert gyroscope data from degrees/sec to rad/sec
        let (dpsx, dpsy, dpsz) = data.gyro_to_dps();
        const DPS_TO_RADPS: f64 = std::f64::consts::PI / 180.0;
        let angular_velocity = Vector3 {
            x: (dpsx as f64) * DPS_TO_RADPS,
            y: (dpsy as f64) * DPS_TO_RADPS,
            z: (dpsz as f64) * DPS_TO_RADPS,
        };
        
        // Create IMU message
        let mut imu_msg = ImuMsg::default();
        imu_msg.header.stamp = Time {
            sec: (now.nsec / 1_000_000_000) as i32,
            nanosec: now.nsec as u32,
        };
        imu_msg.header.frame_id = "imu_link".to_string();
        imu_msg.linear_acceleration = linear_acceleration;
        imu_msg.angular_velocity = angular_velocity;
        // Orientation is not provided by MPU-6500, so leave it as default (zero quaternion)
        // Set covariance[0] to -1 to indicate orientation is not available
        imu_msg.orientation_covariance[0] = -1.0;
        
        match publisher.publish(imu_msg) {
            Ok(_) => {}
            Err(e) => eprintln!("Failed to publish IMU data: {:?}", e),
        }
    }
}

fn main() -> Result<()> {
    

    // let i2c = I2cdev::new("/dev/i2c-1").map_err(|e| anyhow::anyhow!("Failed to create I2C device: {:?}", e))?;
    let mock_i2c = MockI2c;
    let mpu = Mpu6500::new(mock_i2c);
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _imu_node = IMUNode::new(&executor, mpu)?;
    executor.spin(rclrs::SpinOptions::default()).first_error()?;
    Ok(())
}


