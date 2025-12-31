use embedded_hal::i2c::{I2c, ErrorType};
use embedded_hal::i2c::Error as I2cError;
use thiserror::Error;

/// MPU-6500 I2C address (when AD0 pin is connected to GND)
pub const MPU6500_ADDRESS: u8 = 0x68;

/// Register addresses for MPU-6500
mod registers {
    pub const PWR_MGMT_1: u8 = 0x6B;
    pub const GYRO_CONFIG: u8 = 0x1B;
    pub const ACCEL_CONFIG: u8 = 0x1C;
    pub const ACCEL_XOUT_H: u8 = 0x3B;
    pub const GYRO_XOUT_H: u8 = 0x43;
}

/// Errors that can occur when communicating with MPU-6500
#[derive(Error, Debug)]
pub enum Mpu6500Error<I2cError> {
    #[error("I2C communication error: {0}")]
    I2cError(I2cError),
}

/// MPU-6500 sensor driver
pub struct Mpu6500<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> Mpu6500<I2C>
where
    I2C: I2c + ErrorType,
{
    /// Create a new MPU-6500 driver instance
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            address: MPU6500_ADDRESS,
        }
    }

    /// Create a new MPU-6500 driver instance with a custom I2C address
    pub fn new_with_address(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Initialize and configure the MPU-6500 sensor
    pub fn setup(&mut self) -> Result<(), Mpu6500Error<I2C::Error>> {
        // Wake up MPU-6500 (clear sleep bit)
        self.write_register(registers::PWR_MGMT_1, 0x00)?;
        
        // Configure gyroscope to ±250 degrees/sec
        self.write_register(registers::GYRO_CONFIG, 0x00)?;
        
        // Configure accelerometer to ±2g
        self.write_register(registers::ACCEL_CONFIG, 0x00)?;
        
        Ok(())
    }

    /// Read accelerometer and gyroscope data
    pub fn read_sensor_data(&mut self) -> Result<SensorData, Mpu6500Error<I2C::Error>> {
        // Read accelerometer data (6 bytes: X, Y, Z high and low)
        let mut accel_buf = [0u8; 6];
        self.read_registers(registers::ACCEL_XOUT_H, &mut accel_buf)?;
        
        // Read gyroscope data (6 bytes: X, Y, Z high and low)
        let mut gyro_buf = [0u8; 6];
        self.read_registers(registers::GYRO_XOUT_H, &mut gyro_buf)?;
        
        // Convert raw bytes to i16 values
        let accel_x = ((accel_buf[0] as i16) << 8) | accel_buf[1] as i16;
        let accel_y = ((accel_buf[2] as i16) << 8) | accel_buf[3] as i16;
        let accel_z = ((accel_buf[4] as i16) << 8) | accel_buf[5] as i16;
        
        let gyro_x = ((gyro_buf[0] as i16) << 8) | gyro_buf[1] as i16;
        let gyro_y = ((gyro_buf[2] as i16) << 8) | gyro_buf[3] as i16;
        let gyro_z = ((gyro_buf[4] as i16) << 8) | gyro_buf[5] as i16;
        
        Ok(SensorData {
            accelerometer: AccelerometerData {
                x: accel_x,
                y: accel_y,
                z: accel_z,
            },
            gyroscope: GyroscopeData {
                x: gyro_x,
                y: gyro_y,
                z: gyro_z,
            },
        })
    }

    /// Write a byte to a register
    fn write_register(&mut self, reg: u8, data: u8) -> Result<(), Mpu6500Error<I2C::Error>> {
        let buffer = [reg, data];
        self.i2c
            .write(self.address, &buffer)
            .map_err(Mpu6500Error::I2cError)
    }

    /// Read multiple bytes starting from a register
    fn read_registers(&mut self, reg: u8, buffer: &mut [u8]) -> Result<(), Mpu6500Error<I2C::Error>> {
        // Write register address, then read
        let reg_buf = [reg];
        self.i2c
            .write_read(self.address, &reg_buf, buffer)
            .map_err(Mpu6500Error::I2cError)
    }

    /// Release the I2C bus and return the I2C instance
    pub fn release(self) -> I2C {
        self.i2c
    }
}

/// Accelerometer data structure
#[derive(Debug, Clone, Copy)]
pub struct AccelerometerData {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// Gyroscope data structure
#[derive(Debug, Clone, Copy)]
pub struct GyroscopeData {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// Combined sensor data structure
#[derive(Debug, Clone, Copy)]
pub struct SensorData {
    pub accelerometer: AccelerometerData,
    pub gyroscope: GyroscopeData,
}

impl SensorData {
    /// Convert raw accelerometer values to g-force (assuming ±2g range)
    pub fn accel_to_g(&self) -> (f32, f32, f32) {
        const SCALE: f32 = 2.0 / 32768.0; // ±2g range, 16-bit resolution
        (
            self.accelerometer.x as f32 * SCALE,
            self.accelerometer.y as f32 * SCALE,
            self.accelerometer.z as f32 * SCALE,
        )
    }

    /// Convert raw gyroscope values to degrees per second (assuming ±250°/s range)
    pub fn gyro_to_dps(&self) -> (f32, f32, f32) {
        const SCALE: f32 = 250.0 / 32768.0; // ±250°/s range, 16-bit resolution
        (
            self.gyroscope.x as f32 * SCALE,
            self.gyroscope.y as f32 * SCALE,
            self.gyroscope.z as f32 * SCALE,
        )
    }
}

// For testing purposes only
pub struct MockI2c;

impl embedded_hal::i2c::ErrorType for MockI2c {
    type Error = embedded_hal::i2c::ErrorKind;
}

impl embedded_hal::i2c::I2c for MockI2c {
    fn write(&mut self, _address: u8, _buffer: &[u8]) -> Result<(), Self::Error> {
        Ok(())
    }

    fn read(&mut self, _address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        // Return mock data
        for b in buffer.iter_mut() {
            *b = 0;
        }
        Ok(())
    }

    fn write_read(
        &mut self,
        _address: u8,
        _write_buffer: &[u8],
        read_buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        // Return mock data
        for b in read_buffer.iter_mut() {
            *b = 0;
        }
        Ok(())
    }

    fn transaction(
        &mut self,
        _address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        // Mock implementation - simulate operations
        for op in operations {
            match op {
                embedded_hal::i2c::Operation::Write(write_buf) => {
                    // Simulate write - do nothing
                    let _ = write_buf;
                }
                embedded_hal::i2c::Operation::Read(read_buf) => {
                    // Simulate read - fill with zeros
                    for b in read_buf.iter_mut() {
                        *b = 0;
                    }
                }
            }
        }
        Ok(())
    }
}

