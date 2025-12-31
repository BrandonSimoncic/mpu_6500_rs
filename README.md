# MPU-6500 ROS2 Driver (Rust)

A ROS2 node written in Rust for interfacing with the MPU-6500 IMU sensor via I2C.

## Features

- **MPU-6500 Driver**: Full driver implementation for accelerometer and gyroscope
- **ROS2 Integration**: Publishes `sensor_msgs/Imu` messages
- **Mock I2C Support**: Test without hardware using the built-in mock implementation
- **Type-Safe**: Uses Rust's type system and `embedded-hal` traits for I2C communication

## Dependencies

- `embedded-hal` - Hardware abstraction layer for I2C
- `rclrs` - ROS2 client library for Rust
- `sensor_msgs`, `geometry_msgs`, `builtin_interfaces` - ROS2 message types

## Building

```bash
cd src/hardware_drivers/mpu_6500_rs
cargo build --release
```

## Running

### With Mock I2C (Testing)

The default configuration uses a mock I2C implementation for testing without hardware:

```bash
cargo run
# or
ros2 run mpu_6500_rs mpu_6500_rs
```

### With Real Hardware

To use real hardware, you'll need to:

1. Add `linux-embedded-hal` and `i2cdev` to `Cargo.toml`:
   ```toml
   linux-embedded-hal = "0.3"
   i2cdev = "0.5"
   ```

2. Update `main.rs` to initialize real I2C:
   ```rust
   use linux_embedded_hal::I2cdev;
   let i2c = I2cdev::new("/dev/i2c-1")?;
   let mpu = Mpu6500::new(i2c);
   ```

3. Ensure I2C permissions:
   ```bash
   sudo usermod -a -G i2c $USER
   # Log out and back in, or use: newgrp i2c
   ```

## ROS2 Topics

- **Publishes**: `/imu/data` (`sensor_msgs/Imu`)
  - Linear acceleration (m/s²)
  - Angular velocity (rad/s)
  - Orientation covariance set to -1 (not available from MPU-6500)

## Configuration

The MPU-6500 is configured with:
- Accelerometer: ±2g range
- Gyroscope: ±250°/s range
- Update rate: 10 Hz (100ms interval)

## License

TODO: License declaration
