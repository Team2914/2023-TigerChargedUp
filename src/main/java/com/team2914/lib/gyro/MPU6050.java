package com.team2914.lib.gyro;

import edu.wpi.first.wpilibj.I2C;

public class MPU6050 {
    private static MPU6050 instance = null;
    private final static byte MPU6050_ADDRESS = 0x68;
    private final static int REGISTER_POWER_MANAGEMENT_1 = 0x6B;
    private final static int REGISTER_GYRO = 0x43;
    private final I2C accelerometer;
    private byte[] buffer = new byte[6];
    public double gyroX = 0.0;
    public double gyroY = 0.0;
    public double gyroZ = 0.0;
    private double previousTime = 0.0;
    private double currentTime = 0.0;


    private MPU6050(I2C.Port port) {
        accelerometer = new I2C(port, MPU6050_ADDRESS);
        accelerometer.write(REGISTER_POWER_MANAGEMENT_1, 0);
    }

    public static MPU6050 getInstance(I2C.Port port) {
        if (instance == null) {
            instance = new MPU6050(port);
        }

        return instance;
    }

    public double[] readGryo() {
        previousTime = currentTime;
        currentTime = System.currentTimeMillis();
        
        accelerometer.read(REGISTER_GYRO, 6, buffer);
        double[] gyro = {
            ((buffer[0] << 8) | buffer[1])/131.0,
            ((buffer[2] << 8) | buffer[3])/131.0,
            ((buffer[4] << 8) | buffer[5])/131.0
        };
        
        double elapsed = (currentTime - previousTime) / 1000.0;
        gyroX = gyroX + gyro[0] * elapsed;
        gyroY = gyroY + gyro[1] * elapsed;
        gyroZ = gyroZ + gyro[2] * elapsed; 


        double[] out = {gyroX, gyroY, gyroZ};
        return out;
    }
}
