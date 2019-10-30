package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// @TeleOp(name = "Basic: Robot op mode", group = "Linear Opmode")
public class Robot /*extends LinearOpMode*/ {
  private final double TURNSPEEDFACTOR = 0.5; // turn speed factor
  private final double  LINEARSLIDEPOSITION = 0.8;
  private final int LINEARSLIDESLEEP = 500;
  private final double LINEARSLIDEOFFPOWER = 0.0;
  private final double CLOSECLAWPOSITION = 0.0;
  private final double OPENCLAWPOSITION = 0.5;
  private final double OFFCLAWPOSITION = 0.2;
  private final double GRABBERPOSITIONCUTOFF = 0.25;
  private final double HORIZONTALGRABBERPOSITION = 0.0;
  private final double VERTICALGRABBERPOSITION = 0.5;
  private final double ORIGINALLIFTPOWER = 0.0;
  private final double LIFTGOINGDOWN = 0.8;
  private final double LIFTGOINGUP = -0.8;
  private CRServo slide = null;
  private DcMotor flMotor = null;
  private DcMotor frMotor = null;
  private DcMotor rlMotor = null;
  private DcMotor rrMotor = null;
  private DcMotor lLiftMotor = null;
  private DcMotor rLiftMotor = null;
  private Servo turn = null;
  private Servo claw = null;
  private TouchSensor extended = null;
  private TouchSensor retracted = null;
  private ElapsedTime runtime = new ElapsedTime();
  private Servo basePlateGrabber = null;
  private TouchSensor touch = null;

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

  public Robot() {}

  public void init(HardwareMap hardwareMap) {
    // Get handles to all the hardware
    slide = hardwareMap.get(CRServo.class, "servo");
    turn = hardwareMap.get(Servo.class, "grabTurn");
    claw = hardwareMap.get(Servo.class, "claw");
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
    lLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    rLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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

  public void lslide(LinearSlideOperation dir) {
    // Set the linear slide motor to the position
    DcMotorSimple.Direction d = DcMotorSimple.Direction.FORWARD;
    switch (dir) {
      case Off:
        slide.setPower(LINEARSLIDEOFFPOWER);
        return;
      case Extend:
        d = DcMotorSimple.Direction.FORWARD;
        break;
      case Retract:
        d = DcMotorSimple.Direction.REVERSE;
        break;
    }
    slide.setDirection(d);

    // TODO: This is probably wrong
    slide.setPower(LINEARSLIDEPOWER);
    sleep(LINEARSLIDESLEEP);
    slide.setPower(LINEARSLIDEOFFPOWER);
}

  // Grabber stuff:
  public void grabberClutch(GrabberMotorOperation operation) {
    // TODO: Check this...
    switch (operation) {
      case Close:
        claw.setPosition(CLOSECLAWPOSITION);
        break;
      case Open:
        claw.setPosition(OPENCLAWPOSITION);
        break;
      case Off:
        claw.setPosition(OFFCLAWPOSITION);
        break;
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

  public void setGrabberPosition(GrabberPosition position) {
    switch (position) {
      case Horizontal:
        turn.setPosition(HORIZONTALGRABBERPOSITION);
        break;
      case Vertical:
        turn.setPosition(VERTICALGRABBERPOSITION);
        break;
    }
  }

  // Lift stuff:
  public void setLift(LiftDirection dir) {
    double power = ORIGINALLIFTPOWER;
    switch (dir) {
      case Off:
        return;
      case Up:
        power = LIFTGOINGDOWN;
        break;
      case Down:
        power = LIFTGOINGUP;
        break;
    }
    lLiftMotor.setPower(power);
    rLiftMotor.setPower(power);
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


  private Robot robot;
  private Controller controller;
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

  // leave gyroAngle at zero to set relative angle
  public void joystickDrive(Direction j1, Direction j2, double gyroAngle) {
    double hypotenuse = Functions.pyt(j1.X, j1.Y);
    drive(Math.acos(j1.X / hypotenuse), gyroAngle, Range.clip(hypotenuse, -1.0, 1.0), j2.X);
  }

  public void drive(double joystickAngle, double gyroAngle, double power, double turn) {
    tturn = turn * TURNSPEEDFACTOR;
    double angle = joystickAngle + robot.gyroHeading();
    flPower = power * Math.cos((angle - 45) / (180 / Math.PI)) + tturn;
    frPower = -power * Math.cos((angle + 45) / (180 / Math.PI)) + tturn;
    rrPower = -power * Math.cos((angle - 45) / (180 / Math.PI)) + tturn;
    rlPower = power * Math.cos((angle + 45) / (180 / Math.PI)) + tturn;
    robot.motorFrontLeft(flPower);
    robot.motorFrontRight(frPower);
    robot.motorRearLeft(rlPower);
    robot.motorRearRight(rrPower);
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
      robotHeadingRad = Math.toRadians(robot.gyroHeading());
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
        robot.motorFrontLeft(frontLeftSpeed);
        robot.motorFrontRight(frontRightSpeed);
        robot.motorRearLeft(rearLeftSpeed);
        robot.motorRearRight(rearRightSpeed);

        // Display drive status for the driver.
        // telemetry.addData("Speed",  "FL %5.2f:FR %5.2f:RL %5.2f:RR %5.2f", frontLeftSpeed,
        // frontRightSpeed, rearLeftSpeed, rearRightSpeed);
        // telemetry.addData("Gyro", "Heading: " + robot.gyro.getHeading() + " | IntZValue: " +
        // robot.gyro.getIntegratedZValue());
        // telemetry.addData("Gyro", "Heading: " + getRobotHeading());
        // telemetry.update();
      }

      // Stop all motion;
      robot.motorFrontLeft(0);
      robot.motorFrontRight(0);
      robot.motorRearLeft(0);
      robot.motorRearRight(0);
    }
    // TODO: Alex, this stuff clearly isn't finished
    // Please finish it :)

    // turn hasn't been defined: What is it?
    //   turn *= SCALEFACTOR;
    Direction j = controller.rstick();
    double hypotenuse = Functions.pyt(j.X, j.X);
    double power = Range.clip(hypotenuse, -1.0, 1.0);
    double joystickAngle = Math.acos(j.X / hypotenuse);
    // Now we're using hypotenuse, power, joystickAngle, and maybe turn?
  }
}
