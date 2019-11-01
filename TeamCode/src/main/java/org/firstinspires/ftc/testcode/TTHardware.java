package org.firstinspires.ftc.testcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.Servo;

public class TTHardware {

  public BNO055IMU imu1 = null;
  // public BNO055IMU imu2 = null;
  // public ModernRoboticsI2cGyro gyro = null;

  /* Public OpMode members. */
  public DcMotor motorFrontLeft = null;
  public DcMotor motorFrontRight = null;
  public DcMotor motorRearLeft = null;
  public DcMotor motorRearRight = null;

  /* local OpMode members. */
  HardwareMap hwMap = null;

  /* Constructor */
  public TTHardware() {}

  /* Initialize standard Hardware interfaces */
  public void init(HardwareMap ahwMap) {
    // Save reference to Hardware map
    hwMap = ahwMap;

    // Define and Initialize Motors
    motorFrontLeft = hwMap.get(DcMotor.class, "motorFrontLeft");
    motorFrontRight = hwMap.get(DcMotor.class, "motorFrontRight");
    motorRearLeft = hwMap.get(DcMotor.class, "motorRearLeft");
    motorRearRight = hwMap.get(DcMotor.class, "motorRearRight");

    motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
    motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
    motorRearLeft.setDirection(DcMotor.Direction.FORWARD);
    motorRearRight.setDirection(DcMotor.Direction.FORWARD);

    // Set all motors to zero power
    motorFrontLeft.setPower(0.0);
    motorFrontRight.setPower(0.0);
    motorRearLeft.setPower(0.0);
    motorRearRight.setPower(0.0);

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.calibrationDataFile =
        "AdafruitIMUCalibration.json"; // see the calibration sample opmode
    imu1 = hwMap.get(BNO055IMU.class, "imu1");
    imu1.initialize(parameters);
    // imu2 = hwMap.get(BNO055IMU.class, "imu2");
    // imu2.initialize(parameters);

    // gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");

  }
}
