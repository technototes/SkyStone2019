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

public class TTRoboTest {
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
  private Servo blockFlipper = null;
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

  public static final void sleep(long milliseconds) {
    try {
      Thread.sleep(milliseconds);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }

  public TTRoboTest(HardwareMap hardwareMap, Telemetry tel) {
    telemetry = tel;
    // Get handles to all the hardware
    slide = hardwareMap.get(CRServo.class, "lslideServo");
    turn = hardwareMap.get(Servo.class, "grabTurn");
    lClaw = hardwareMap.get(Servo.class, "lClaw");
    rClaw = hardwareMap.get(Servo.class, "rClaw");
    blockFlipper = hardwareMap.get(Servo.class, "blockFlipper");
    cap = hardwareMap.get(CRServo.class, "cap");
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

    // Calibrate
    // make lift motors work together: they're facing opposite directions
    lLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    rLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    // Set the digital channel mode to
    // Output mode can be used to blink LED's
    lslideSwitch.setMode(DigitalChannel.Mode.INPUT);
    liftSwitch.setMode(DigitalChannel.Mode.INPUT);

    lGrabber.setDirection(Servo.Direction.FORWARD);
    rGrabber.setDirection(Servo.Direction.REVERSE);

    lClaw.setDirection(Servo.Direction.FORWARD);
    rClaw.setDirection(Servo.Direction.REVERSE);
    // TODO: Add initialization / calibration for the slide and lift?

    // Shamelessly copied from example code...
    sleep(2000);
    // Start the logging of measured acceleration
    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
  }

  // Linear slide stuff:
  public boolean slideSwitchSignaled() {
    return !lslideSwitch.getState();
  }

  public boolean liftSwitchSignaled() {
    return !liftSwitch.getState();
  }

  public void setLinearSlideDirectionRyan(LinearSlideOperation operation, boolean override) {
    double power = 0;
    switch (operation) {
      case Extend:
        power = LINEARSLIDEPOWER;
        break;
      case Retract:
        power = -LINEARSLIDEPOWER;
        break;
      case None:
        power = 0;
        break;
    }
    slide.setPower(power);
    while (!override && power != 0 && slideSwitchSignaled()) {
      slide.setPower(-power);
      sleep(1);
    }
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

  public void setLinearSlideDirectionEmily(LinearSlideOperation operation, boolean override) {
    double power = 0;
    switch (operation) {
      case Extend:
        power = LINEARSLIDEPOWER;
        break;
      case Retract:
        power = -LINEARSLIDEPOWER;
        break;

    }

    // Hit a limit
    if (!override && slideSwitchSignaled()) {
      // Stop the slide
      while (slideSwitchSignaled() == true) {
        slide.setPower(-power);
        sleep(1);
      }
      power = 0;
    }


    slide.setPower(power);
  }


  // Grabber stuff:
  public void claw(double val) {
    lClaw.setPosition(val);
    rClaw.setPosition(val);
  }

  public void rotateClaw(double val) {
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

  public void liftUp() {
    setLiftPower(-1.0);
  }

  public void liftDown() {
    if (!isLiftAtLowerLimit())
      setLiftPower(1.0);
  }

  public void liftStop() {
    setLiftPower(0);
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

  public double gyroHeading2() {
    Orientation angles1 =
      imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    return -AngleUnit.DEGREES.fromUnit(angles1.angleUnit, angles1.firstAngle);
  }

  void setServoDirection(Servo.Direction direction) {
    turn.setDirection(direction);
  }

  void setServoPosition(double position) {
    turn.setPosition(position);
  }

  public int getSkystonePosition() {
    //add vuforia+tristan vision processing
    return 1;
  }

  // Distance sensing
  public double rearDistance() {
    return sensorRangeRear.getDistance(DistanceUnit.CM);
  }

  public double leftDistance() {
    return sensorRangeLeft.getDistance(DistanceUnit.CM);
  }

  public double rightDistance() {
    return sensorRangeRight.getDistance(DistanceUnit.CM);
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
  private double toNearestAngle(double gyroAngle) {
    double test = 0.0;
    if (gyroAngle > 50 && gyroAngle < 130) {
      test = 90 - gyroAngle;
    } else if (gyroAngle > 140 && gyroAngle < 180) {
      test = 180 - gyroAngle;
    } else if (gyroAngle > -180 && gyroAngle < -140) {
      test = -180 - gyroAngle;
    } else if (gyroAngle > -130 && gyroAngle < -50) {
      test = -90 - gyroAngle;
    } else if (gyroAngle > -40 && gyroAngle < 40) {
      test = 0 - gyroAngle;
    }
    return test;
  }

  // This returns a normalized angle difference (-180 to 180) from gyroHeading
  private double angleDiff(double target) {
    return AngleUnit.normalizeDegrees(gyroHeading() - target);
  }

  private Direction getDirectionTowardAngle(double to) {
    double dir = angleDiff(to);
    double rotSpeed = XDrive.getSteppedValue(TTRoboTest.snapSteps, dir / 180);
    return new Direction(rotSpeed, 0);
  }

  //for autonomous only
  public void toAngleSync(double to) {
    double dir;
    do {
      Direction turn = getDirectionTowardAngle(to);
      dir = turn.X;
      joystickDrive(Direction.None, turn, gyroHeading());
      sleep(10);
    } while (Math.abs(dir) > 3);
  }

  // Snap the robot to the closest 90 degree angle
  public double snap(Telemetry tel) {
    double curr = gyroHeading();
    double newangle = toNearestAngle(curr);
    tel.addData("Snap:", String.format("curr: %3.3f new: %3.3f", curr, newangle));
    return scaledSnap(newangle);
    //return snap(newangle); replaced with above scaledSnap
  }

  private static double[] snapSteps = new double[]{0.0, .2, .4, .7, 1.0, 1.0};

  private double scaledSnap(double targetAngle) {
    return XDrive.getSteppedValue(snapSteps, targetAngle / 90.0);
  }

  public void stop() {
    driveTrain.stop();
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

  // This will travel toward the rear until it gets to 'dist'
  public void distRearDrive(double speed, double dist) {
    // Update: No attention should be paid to 'speed'
    // Just drive and slow down when we get slow to the target
    ElapsedTime tm = new ElapsedTime();
    double curDistance = 0;
    do {
      curDistance = rearDistance();
      telemetry.addData("Current Distance", curDistance);
      telemetry.update();
      double dir = (dist < curDistance) ? 1 : -1;
      double magnitude = Math.abs(dist - curDistance);
      if (magnitude < 10) {
        speedSnail();
      } else if (magnitude > 50) {
        speedTurbo();
      } else {
        speedNormal();
      }
      Direction turn = getDirectionTowardAngle(0);
      joystickDrive(new Direction(0, dir), turn, gyroHeading());
      sleep(10);
    } while (Math.abs(curDistance - dist) > 2 && tm.time() < 3.0);
    driveTrain.stop();
    speedNormal();
  }

  public void syncTurn(double angle, double time) {

    ElapsedTime runTime = new ElapsedTime();
    runTime.reset();
    while (runTime.seconds() < time) {
      if (gyroHeading2() > angle + 5) {
        joystickDrive(Direction.None, new Direction(-0.5, 0), gyroHeading2());
      } else if (gyroHeading2() < angle - 5) {
        joystickDrive(Direction.None, new Direction(0.5, 0), gyroHeading2());
      } else {
        joystickDrive(Direction.None, new Direction(0, 0), gyroHeading2());
        time = 0;
      }
      telemetry.addData("gyro:", gyroHeading());
      telemetry.addData("gyro2:", gyroHeading2());
      telemetry.update();
    }
    //new code
    //._. ._.
    // \_(0.0)_/

  }

  public void distLeftDrive(double speed, double angle, double leftDist) {
    // TODO: Check this angle
    driveTrain.setDriveVector(speed, angle, gyroHeading());
    ElapsedTime tm = new ElapsedTime();
    do {
      sleep(10);
    } while (getCappedRange(sensorRangeLeft, 1500) > leftDist && tm.time() < 10.0);
    driveTrain.stop();
  }

  public void distRightDrive(double speed, double angle, double rightDist) {
    // TODO: Check this angle
    driveTrain.setDriveVector(speed, angle, gyroHeading());

    ElapsedTime tm = new ElapsedTime();
    tm.reset();
    do {
      sleep(10);
    } while (Math.abs(getCappedRange(sensorRangeRight, 1500) - rightDist) > 2 && tm.seconds() < 5.0);
    driveTrain.stop();
  }

  // This attempts to drive in a straight(ish) line toward a corner
  public void distRearLeftDrive(double speed, double rearDist, double leftDist) {
    double rDist, ltDist;
    ElapsedTime tm = new ElapsedTime();
    do {
      rDist = getCappedRange(sensorRangeRear, 1500);
      ltDist = getCappedRange(sensorRangeLeft, 1500);
      // Let's figure out what angle to drive toward to make a straightish line
      // TODO: I have no idea if this is the proper angle or not
      // TODO: Might need to do something like 90 - angle
      double angle = Math.atan2(rDist, -ltDist);
      angle = AngleUnit.DEGREES.fromRadians(angle);
      driveTrain.setDriveVector(speed, angle, gyroHeading());
      sleep(10);
    } while (rDist > rearDist && ltDist > leftDist && tm.time() < 10.0);
    driveTrain.stop();
  }

  // This attempts to drive in a straight(ish) line toward a corner
  public void distRearRightDrive(double speed, double rearDist, double rightDist) {
    double rDist, rtDist;
    ElapsedTime tm = new ElapsedTime();
    do {
      rDist = getCappedRange(sensorRangeRear, 1500);
      rtDist = getCappedRange(sensorRangeLeft, 1500);
      // Let's figure out what angle to drive toward to make a straightish line
      // TODO: I have no idea if this is the proper angle or not
      // TODO: Might need to do something like 90 - angle
      double angle = Math.atan2(rDist, rtDist);
      angle = AngleUnit.DEGREES.fromRadians(angle);
      driveTrain.setDriveVector(speed, angle, gyroHeading());
      sleep(10);
    } while (rDist > rearDist && rtDist > rightDist && tm.time() < 10.0);
    driveTrain.stop();
  }

  private double getCappedRange(DistanceSensor sens, double cap) {
    double res = sens.getDistance(DistanceUnit.CM);
    return Range.clip(res, 0.01, cap);
  }

  public void driveWallRear(double speed, double time, double angle, double distance) {
    double gyroAngle = gyroHeading();

    if (rearDistance() < distance && (angle < 180 && angle > 0)) {

      gyroAngle = gyroHeading() + 3;
    } else if (rearDistance() > distance && (angle > 180 && angle < 260)) {
      gyroAngle = gyroHeading() - 3;
    } else {
      gyroAngle = gyroHeading();
    }


    driveTrain.timeDrive(speed, time, angle, gyroHeading());
  }
}
