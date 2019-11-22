package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class TTRobot {
  // Scaling values

  // The amount we divide speed by when dropping the lift
  private static final double DOWNWARDLIFTSCALE = 2.0;
  // the power of the linear slide
  private static final double LINEARSLIDEPOWER = 1.0;

  // Dead zones

  // This is the middle 'dead zone' of the analog sticks
  public static final double STICKDEADZONE = 0.05;
  // Triggers must be pushed at least this far
  public static final double TRIGGERTHRESHOLD = 0.25;

  // Claw grab positions
  public static final double CLAWOPENPOSITION = 0.4;
  public static final double CLAWCLOSEPOSITION = 0.6;

  // Unused stuff

  // the grab rotation position for snapping to horizontal or vertical
  private static final double GRABBERPOSITIONCUTOFF = 0.25;
  // the grab rotation 'horizontal' position
  private static final double HORIZONTALGRABBERPOSITION = 0.0;
  // the grab rotation 'vertical' position
  private static final double VERTICALGRABBERPOSITION = 0.5;

  enum LinearSlidePosition {
    In,
    Middle,
    Out
  }

  private boolean isGrabberOpened = true;
  private LinearSlidePosition slidePosition = LinearSlidePosition.In;

  private DigitalChannel lslideSwitch = null;
  private DigitalChannel liftSwitch = null;
  private CRServo slide = null;
  private DcMotor lLiftMotor = null;
  private DcMotor rLiftMotor = null;
  private Servo turn = null;
  private Servo lClaw = null;
  private Servo rClaw = null;
  private TouchSensor extended = null;
  private TouchSensor retracted = null;
  private Servo blockFlipper = null;
  private TouchSensor touch = null;
  private CRServo cap = null;
  private ColorSensor sensorColorBottom = null;
  private DistanceSensor sensorRangeRear = null;
  private DistanceSensor sensorRangeLeft = null;
  private DistanceSensor sensorRangeRight = null;
  private Servo lGrabber = null;
  private Servo rGrabber = null;

  private XDrive driveTrain = null;
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

  public TTRobot() {
  }

  public void init(HardwareMap hardwareMap, Telemetry tel) {
    telemetry = tel;
    // Get handles to all the hardware
    slide = hardwareMap.get(CRServo.class, "lslideServo");
    turn = hardwareMap.get(Servo.class, "grabTurn");
    lClaw = hardwareMap.get(Servo.class, "lClaw");
    rClaw = hardwareMap.get(Servo.class, "rClaw");
    blockFlipper = hardwareMap.get(Servo.class, "blockFlipper");
    cap = hardwareMap.get(CRServo.class, "cap");
    // extended = hardwareMap.get(TouchSensor.class, "extLimitSwitch");
    // retracted = hardwareMap.get(TouchSensor.class, "retLimitSwitch");
    lslideSwitch = hardwareMap.get(DigitalChannel.class, "slideLimit");
    liftSwitch = hardwareMap.get(DigitalChannel.class, "liftLimit");
    sensorRangeRear = hardwareMap.get(DistanceSensor.class, "sensorRangeRear");
    sensorRangeLeft = hardwareMap.get(DistanceSensor.class, "sensorRangeLeft");
    sensorRangeRight = hardwareMap.get(DistanceSensor.class, "sensorRangeRight");

    lLiftMotor = hardwareMap.get(DcMotor.class, "motorLiftLeft");
    rLiftMotor = hardwareMap.get(DcMotor.class, "motorLiftRight");
    sensorColorBottom = hardwareMap.get(ColorSensor.class, "sensorColorBottom");

    lGrabber = hardwareMap.get(Servo.class, "lGrabber");
    rGrabber = hardwareMap.get(Servo.class, "rGrabber");

    DcMotor flMotor = hardwareMap.get(DcMotor.class, "motorFrontLeft");
    DcMotor frMotor = hardwareMap.get(DcMotor.class, "motorFrontRight");
    DcMotor rlMotor = hardwareMap.get(DcMotor.class, "motorRearLeft");
    DcMotor rrMotor = hardwareMap.get(DcMotor.class, "motorRearRight");
    driveTrain = new XDrive(flMotor, frMotor, rlMotor, rrMotor);

    //    touch = hardwareMap.get(TouchSensor.class, "touch");

    // Setup the IMU
    // Set up the parameters with which we will use our IMU. Note that integration
    // algorithm here just reports accelerations to the logcat log; it doesn't actually
    // provide positional information.
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.calibrationDataFile =
      "AdafruitIMUCalibration.json"; // see the calibration sample opmode
    imu = hardwareMap.get(BNO055IMU.class, "imu1");
    imu.initialize(parameters);
  }

  public void calibrate() {
    // make lift motors work together: they're facing opposite directions
    lLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    rLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    // Set the digital channel mode to input
    // Output mode can be used to blink LED's
    lslideSwitch.setMode(DigitalChannel.Mode.INPUT);
    liftSwitch.setMode(DigitalChannel.Mode.INPUT);

    lGrabber.setDirection(Servo.Direction.FORWARD);
    rGrabber.setDirection(Servo.Direction.REVERSE);

    lClaw.setDirection(Servo.Direction.FORWARD);
    rClaw.setDirection(Servo.Direction.REVERSE);
    // TODO: Add initialization / calibration for the slide and lift?

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

  private boolean slideSwitchSignaled() {
    return !lslideSwitch.getState();
  }

  private LinearSlidePosition currentPos = LinearSlidePosition.In;

  public void setLinearSlideDirection(LinearSlideOperation operation, boolean override) {
    double power = 0;
    switch (operation) {
      case Extend:
        power = LINEARSLIDEPOWER;
        break;
      case Retract:
        power = -LINEARSLIDEPOWER;
        break;

    }

    switch (slidePosition) {
      case In:
        switch (operation) {
          case Extend:
            if (!slideSwitchSignaled()) {
              slidePosition = LinearSlidePosition.Middle;
            }
            break;
          case Retract:
            if (override)
              break;
            //otherwise do not move

          default:
            // Do nothing
            power = 0;
            break;
        }
        break;

      case Middle:
        // Hit a limit
        if (!override && slideSwitchSignaled()) {
          // Stop the slide
          power = 0;

          // Update the state
          switch (operation) {
            case Extend:
              slidePosition = LinearSlidePosition.Out;
              break;
            case Retract:
              slidePosition = LinearSlidePosition.In;
              break;
          }
        }
        break;

      case Out:
        switch (operation) {
          case Retract:
            if (!slideSwitchSignaled()) {
              slidePosition = LinearSlidePosition.Middle;
            }
            break;

          case Extend:
            if (override)
              break;
            //otherwise do not move

          default:
            // Do nothing
            power = 0;
            break;
        }
        break;
    }

    slide.setPower(power);
  }


  // Grabber stuff:
  public void claw(double val) {
    lClaw.setPosition(val);
    rClaw.setPosition(val);
  }

  public void turnn(double val) {
    turn.setPosition(val);
  }

  public void setClawPosition(ClawPosition position) {
    switch (position) {
      case Open:
        claw(0); // Open
        break;
      case Close:
        claw(1); // Closed
        break;
    }
  }

  // Lift stuff:
  enum LiftState {
    Above,
    At,
    Below
  }

  public void bpGrabber(double pos) {
    lGrabber.setPosition(pos);
    rGrabber.setPosition(pos);
  }

  private void setLiftPower(double val) {
    if (val > 0)
      val = val / DOWNWARDLIFTSCALE;
    lLiftMotor.setPower(val);
    rLiftMotor.setPower(val);
  }

  // Positive is down, Negative is up!
  public void setLift(double speed) {
    // If we're at the lower limit, only allow upward motion
    if (!isLiftAtLowerLimit() || speed <= 0) {
      setLiftPower(speed);
    }
  }

  public void blockFlipper(double pos) {
    blockFlipper.setPosition(pos);
  }

  public void capstone(double speed) {
    cap.setPower(-speed);
  }

  public boolean isLiftAtLowerLimit() {
    return !liftSwitch.getState();
  }

  public double gyroHeading() {
    Orientation angles1 =
      imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    return -AngleUnit.DEGREES.fromUnit(angles1.angleUnit, angles1.firstAngle + 180);
  }

  void setServoDirection(Servo.Direction direction) {
    turn.setDirection(direction);
  }

  void setServoPosition(double position) {
    turn.setPosition(position);
  }

  // Drive train:

  public void speedSnail() {
    driveTrain.setSpeed(XDrive.DriveSpeed.Snail);
  }

  public void speedTurbo() {
    driveTrain.setSpeed(XDrive.DriveSpeed.Turbo);
  }

  public void speedNormal() {
    driveTrain.setSpeed(XDrive.DriveSpeed.Normal);
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
    //drive(0.0, 0.0, 0.0, angle);
  }

  // leave gyroAngle at zero to set relative angle
  public void joystickDrive(Direction j1, Direction j2, double gyroAngle) {
    driveTrain.setStickVector(j1.X, j1.Y, j2.X, gyroAngle);
    telemetry.addData("control rstick X: ", j2.X);
  }

  public void timeDrive(double speed, double time, double angle) {
    driveTrain.timeDrive(speed, time, angle, gyroHeading());
  }

  public void lineDrive(double speed, double time, double angle) {
    driveTrain.setDriveVector(speed, angle, gyroHeading());
    ElapsedTime tm = new ElapsedTime();
    int red, blue;
    do {
      sleep(10);
      red = sensorColorBottom.red();
      blue = sensorColorBottom.blue();
    } while (tm.seconds() < time && !(Math.abs(red - blue) > 50));
    driveTrain.stop();
  }

  public void driveToLine(double speed, double direction) {
    lineDrive(speed, 5, direction);
  }

  public void distRearDrive(double speed, double dist, double angle) {
    // TODO: We don't currently have a range sensor available in software
    driveTrain.setDriveVector(speed, angle, gyroHeading());
    ElapsedTime tm = new ElapsedTime();
    do {
      sleep(10);
    } while (sensorRangeRear.getDistance(DistanceUnit.CM) > dist && tm.time() < 3.0);
    driveTrain.stop();
  }
}
