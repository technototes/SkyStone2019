package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class TTRobot {
  // The power applied to the wheels for robot rotation
  private final double TURNSPEEDFACTOR = 0.5;
  // the grab rotation position for snapping to horizontal or vertical
  private final double GRABBERPOSITIONCUTOFF = 0.25;
  // the grab rotation 'horizontal' position
  private final double HORIZONTALGRABBERPOSITION = 0.0;
  // the grab rotation 'vertical' position
  private final double VERTICALGRABBERPOSITION = 0.5;
  //the power of the linear slide
  private final double LINEARSLIDEPOWER = 0.5;

  private boolean isGrabberOpened = true;
  private LinearSlidePosition position = LinearSlidePosition.In;

  private DigitalChannel lslideSwitch = null;
  private CRServo slide = null;
  private DcMotor flMotor = null;
  private DcMotor frMotor = null;
  private DcMotor rlMotor = null;
  private DcMotor rrMotor = null;
  private DcMotor lLiftMotor = null;
  private DcMotor rLiftMotor = null;
  private Servo turn = null;
  private CRServo claw = null;
  private TouchSensor extended = null;
  private TouchSensor retracted = null;
  private ElapsedTime runtime = new ElapsedTime();
  private Servo basePlateGrabber = null;
  private TouchSensor touch = null;

  private Telemetry telemetry = null;

  // Stuff for the on-board "inertial measurement unit" (aka gyro)
  // The IMU sensor object
  private BNO055IMU imu;
  // State used for updating telemetry
  private Orientation angles;
  private Acceleration gravity;

  private final void sleep(long milliseconds) {
    try {
      Thread.sleep(milliseconds);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }

  public TTRobot() {}

  public void init(HardwareMap hardwareMap, Telemetry tel) {
    telemetry = tel;
    // Get handles to all the hardware
    slide = hardwareMap.get(CRServo.class, "servo");
    turn = hardwareMap.get(Servo.class, "grabTurn");
    claw = hardwareMap.get(CRServo.class, "claw");
    basePlateGrabber = hardwareMap.get(Servo.class, "BPGrabber");
    extended = hardwareMap.get(TouchSensor.class, "extLimitSwitch");
    retracted = hardwareMap.get(TouchSensor.class, "retLimitSwitch");

    flMotor = hardwareMap.get(DcMotor.class, "flMotor");
    frMotor = hardwareMap.get(DcMotor.class, "frMotor");
    rlMotor = hardwareMap.get(DcMotor.class, "rlMotor");
    rrMotor = hardwareMap.get(DcMotor.class, "rrMotor");
    lLiftMotor = hardwareMap.get(DcMotor.class, "lLiftMotor");
    rLiftMotor = hardwareMap.get(DcMotor.class, "rLiftMotor");

    imu = hardwareMap.get(BNO055IMU.class, "imu");
    touch = hardwareMap.get(TouchSensor.class, "touch");

    // Setup the IMU
    // Set up the parameters with which we will use our IMU. Note that integration
    // algorithm here just reports accelerations to the logcat log; it doesn't actually
    // provide positional information.
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.calibrationDataFile =
        "AdafruitIMUCalibration.json"; // see the calibration sample opmode
    parameters.loggingEnabled = true;
    parameters.loggingTag = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    imu.initialize(parameters);
  }

  public void calibrate() {
    // make lift motors work together
    lLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    rLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);



    // Shamelessly copied from example code...
    while (imu.getCalibrationStatus().calibrationStatus != 0
        || imu.getSystemStatus() != BNO055IMU.SystemStatus.RUNNING_FUSION) {
      sleep(10);
    }
    // Start the logging of measured acceleration
    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
  }

  // Linear slide stuff:
  public boolean isLinearSlideFullyExtended() {
    return extended.isPressed();
  }

  public boolean isLinearSlideFullyRetracted() {
    return retracted.isPressed();
  }

  public void lslide(LinearSlideOperation inOrOut) {
    if(position == LinearSlidePosition.In){
        if(inOrOut == LinearSlideOperation.Extend){
          while(lslideSwitch.getState()){
            slide.setPower(LINEARSLIDEPOWER);
          }
          slide.setPower(LINEARSLIDEPOWER);
          while(!lslideSwitch.getState()){
            slide.setPower(LINEARSLIDEPOWER);
          }
          position = LinearSlidePosition.Middle;
        }
    }else if(position == LinearSlidePosition.Middle){
      if(inOrOut == LinearSlideOperation.Extend){
        while(lslideSwitch.getState()){
          slide.setPower(LINEARSLIDEPOWER);
        }
        while(!lslideSwitch.getState()){
          slide.setPower(LINEARSLIDEPOWER);
        }
        position = LinearSlidePosition.Middle;
      }else{
        while(lslideSwitch.getState()){
          slide.setPower(-LINEARSLIDEPOWER);
        }
        while(!lslideSwitch.getState()){
          slide.setPower(-LINEARSLIDEPOWER);
        }
        position = LinearSlidePosition.Middle;
      }
    }else{
      if(inOrOut == LinearSlideOperation.Retract){
        while(lslideSwitch.getState()){
          slide.setPower(-LINEARSLIDEPOWER);
        }
        while(!lslideSwitch.getState()){
          slide.setPower(-LINEARSLIDEPOWER);
        }
        position = LinearSlidePosition.Middle;
      }
    }
  }

  // Grabber stuff:
  public void grabberClutch() {
    if (isGrabberOpened) {
      claw.setPower(-1);
      isGrabberOpened = false;
    } else {
      claw.setPower(1);
    }
  }

  public GrabberPosition getGrabberPosition() {
    // TODO: Check this...
    double pos = turn.getPosition();
    if (pos < GRABBERPOSITIONCUTOFF) {
      return GrabberPosition.Horizontal;
    } else {
      return GrabberPosition.Vertical;
    }
  }

  public void snapGrabberPosition(GrabberPosition position) {
    switch (position) {
      case Horizontal:
        turn.setPosition(HORIZONTALGRABBERPOSITION);
        break;
      case Vertical:
        turn.setPosition(VERTICALGRABBERPOSITION);
        break;
    }
  }

  public void turnGrabber(GrabberPosition position) {
    switch (position) {
      case Horizontal:
        turn.setPosition(turn.getPosition() - 0.1);
        break;
      case Vertical:
        turn.setPosition(turn.getPosition() + 0.1);
        break;
    }
  }

  // Lift stuff:
  public void setLift(double speed) {
    lLiftMotor.setPower(speed);
    rLiftMotor.setPower(speed);
  }

  public boolean isLiftAtUpperLimit() {
    // TODO: Read the upper limit switch
    return false;
  }

  public boolean isLiftAtLowerLimit() {
    // TODO: Read the lower limit switch
    return false;
  }

  // Drive train:
  // These should just be used by the drive train
  public void motorFrontLeft(double power) {
    flMotor.setPower(Range.clip(power, -1, 1));
  }

  public void motorFrontRight(double power) {
    frMotor.setPower(Range.clip(power, -1, 1));
  }

  public void motorRearLeft(double power) {
    rlMotor.setPower(Range.clip(power, -1, 1));
  }

  public void motorRearRight(double power) {
    rrMotor.setPower(Range.clip(power, -1, 1));
  }

  public double gyroHeading() {
    // UNTESTED!
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    gravity = imu.getGravity();
    return angles.firstAngle;
  }

  /*
  fl  +-------+  fr
     /         \
    /           \
   +             +
   |    motor    |
   |   position  |
   +             +
    \           /
     \         /
  rl  +-------+  rr
  */
  public double flPower;
  public double frPower;
  public double rrPower;
  public double rlPower;
  public double tturn;
  private ElapsedTime runtime2 = new ElapsedTime();
  private ElapsedTime timer = new ElapsedTime();

  // State used for updating telemetry
  private Orientation angles1;
  private Orientation angles2;

  private double leftStickY = 0;
  private double leftStickX = 0;
  private double rightStickX = 0;

  private double robotHeadingRad = 0.0;
  private double powerCompY = 0.0;
  private double powerCompX = 0.0;

  private double powerFrontLeft = 0.0;
  private double powerFrontRight = 0.0;
  private double powerRearLeft = 0.0;
  private double powerRearRight = 0.0;

  // leave gyroAngle at zero to set relative angle
  public void joystickDrive(Direction j1, Direction j2, double gyroAngle) {
    if (j1.Y != 0) {
      leftStickY = stepInput(j1.Y);
    } else {
      leftStickY = 0.0;
    }
    if (j1.X != 0) {
      leftStickX = stepInput(j1.X);
    } else {
      leftStickX = 0.0;
    }
    rightStickX = stepInputRotate(j2.X);

    if (leftStickY != 0 || leftStickX != 0 || rightStickX != 0) {
//                robotHeadingRad = Math.toRadians(((360 - robot.gyro.getHeading()) % 360));
      robotHeadingRad = Math.toRadians(gyroAngle);
      powerCompY = (Math.cos(robotHeadingRad) * leftStickY) + (Math.sin(robotHeadingRad) * leftStickX);
      powerCompX = -(Math.sin(robotHeadingRad) * leftStickY) + (Math.cos(robotHeadingRad) * leftStickX);

      powerFrontLeft = powerCompY + powerCompX + rightStickX;
      powerFrontRight = -powerCompY + powerCompX + rightStickX;
      powerRearLeft = powerCompY - powerCompX + rightStickX;
      powerRearRight = -powerCompY - powerCompX + rightStickX;
    } else {
      powerFrontLeft = 0.0;
      powerFrontRight = 0.0;
      powerRearLeft = 0.0;
      powerRearRight = 0.0;
    }

    motorFrontLeft(Range.clip(powerFrontLeft, -1.0, 1.0));
    motorFrontRight(Range.clip(powerFrontRight, -1.0, 1.0));
    motorRearLeft(Range.clip(powerRearLeft, -1.0, 1.0));
    motorRearRight(Range.clip(powerRearRight, -1.0, 1.0));
  }

  public void drive(double joystickAngle, double gyroAngle, double power, double turn) {
    tturn = turn * TURNSPEEDFACTOR;
    double angle = joystickAngle + gyroAngle;
    flPower = power * Math.cos(-Math.PI*angle-Math.PI/4) + tturn;
    frPower = -power * Math.cos(-Math.PI*angle+Math.PI/4) + tturn;
    rrPower = -power * Math.cos(-Math.PI*angle-Math.PI/4) + tturn;
    rlPower = power * Math.cos(-Math.PI*angle+Math.PI/4) + tturn;
    this.motorFrontLeft(flPower);
    this.motorFrontRight(frPower);
    this.motorRearLeft(rlPower);
    this.motorRearRight(rrPower);
  }
  // set nearestSnap to true to snap to nearest 90 dgree angle, or set nearestSnap to false and
  // input angle to snap to.
  public double snapToAngle(double gyroAngle) {
    double test = 0.0;
    if (gyroAngle > 50 && gyroAngle < 130) {
      test = 90 - gyroAngle;

    } else if (gyroAngle > 140 && gyroAngle < 220) {
      test = 180 - gyroAngle;

    } else if (gyroAngle > 230 && gyroAngle < 310) {
      test = 270 - gyroAngle;

    } else if ((gyroAngle >= 0 && gyroAngle < 40) || (gyroAngle > 320 && gyroAngle <= 360)) {
      test = 0 - gyroAngle;
    }
    return test;
  }

  // Snap the robot to the closest 90 degree angle
  public void snap() {
    double curr = gyroHeading();
    double newangle = snapToAngle(curr);
    snap(newangle);
  }

  // Turn the robot to a specific angle
  public void snap(double angle) {
    drive(0.0, 0.0, 0.0, angle);
  }

  public void timeDrive(double speed, double time, double angle) {
    ElapsedTime driveTime = new ElapsedTime();
    double robotHeadingRad = 0.0;
    double angleRad = Math.toRadians(angle);
    double powerCompY = 0.0;
    double powerCompX = 0.0;

    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    // Ensure that the opmode is still active
    if (true) {
      driveTime.reset();

      speed = Range.clip(speed, 0.0, 1.0);
      //            robotHeadingRad = Math.toRadians(360 - robot.gyro.getHeading());
      robotHeadingRad = Math.toRadians(this.gyroHeading());
      powerCompY =
          (Math.cos(robotHeadingRad) * (Math.cos(angleRad) * speed))
              + (Math.sin(robotHeadingRad) * (Math.sin(angleRad) * speed));
      powerCompX =
          -(Math.sin(robotHeadingRad) * (Math.cos(angleRad) * speed))
              + (Math.cos(robotHeadingRad) * (Math.sin(angleRad) * speed));

      frontLeftSpeed = powerCompY + powerCompX;
      frontRightSpeed = -powerCompY + powerCompX;
      rearLeftSpeed = powerCompY - powerCompX;
      rearRightSpeed = -powerCompY - powerCompX;

      // keep looping while we are still active, and BOTH motors are running.
      while (true && driveTime.seconds() < time) {
        this.motorFrontLeft(frontLeftSpeed);
        this.motorFrontRight(frontRightSpeed);
        this.motorRearLeft(rearLeftSpeed);
        this.motorRearRight(rearRightSpeed);

        // Display drive status for the driver.
        // telemetry.addData("Speed",  "FL %5.2f:FR %5.2f:RL %5.2f:RR %5.2f", frontLeftSpeed,
        // frontRightSpeed, rearLeftSpeed, rearRightSpeed);
        // telemetry.addData("Gyro", "Heading: " + robot.gyro.getHeading() + " | IntZValue: " +
        // robot.gyro.getIntegratedZValue());
        // telemetry.addData("Gyro", "Heading: " + getRobotHeading());
        // telemetry.update();
      }

      // Stop all motion;
      this.motorFrontLeft(0);
      this.motorFrontRight(0);
      this.motorRearLeft(0);
      this.motorRearRight(0);
    }
  }
}
