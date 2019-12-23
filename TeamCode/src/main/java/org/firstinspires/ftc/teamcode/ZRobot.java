package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

public class ZRobot implements IRobot {
  // Scaling values

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

  // Distance speeds in cm
  public static final double TURBODISTANCE = 65;
  public static final double SNAILDISTANCE = 15;

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

  private CRServo slide = null;
  private ColorSensor sensorColorBottom = null;
  private DistanceSensor sensorRangeRear = null;
  private DistanceSensor sensorRangeLeft = null;
  private DistanceSensor sensorRangeRight = null;

  public LiftControl lift = null;
  private XDrive driveTrain = null;
  private Telemetry telemetry = null;
  private LinearOpMode opMode = null;
  // Stuff for the on-board "inertial measurement unit" (aka gyro)
  // The IMU sensor object
  private BNO055IMU imu;
  // State used for updating telemetry
  private Orientation angles;
  private Acceleration gravity;

  private static boolean UNTESTED = false;

  // This is an 'opMode aware' sleep: It will stop if you hit 'stop'!
  public final void sleep(long milliseconds) {
    try {
      if (UNTESTED) {
        ElapsedTime tm = new ElapsedTime();
        while (tm.milliseconds() < milliseconds && opMode.opModeIsActive()) {
          Thread.sleep(Math.min(milliseconds - (long) tm.milliseconds(), 50));
        }
      } else {
        Thread.sleep(milliseconds);
      }
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }

  public ZRobot(LinearOpMode op, HardwareMap hardwareMap, Telemetry tel) {
    opMode = op;
    telemetry = tel;
    // Get handles to all the hardware
    slide = hardwareMap.get(CRServo.class, "slide");
//    turn = hardwareMap.get(Servo.class, "grabTurn");
//    lClaw = hardwareMap.get(Servo.class, "lClaw");
//    rClaw = hardwareMap.get(Servo.class, "rClaw");
//    blockFlipper = hardwareMap.get(Servo.class, "blockFlipper");
//    cap = hardwareMap.get(CRServo.class, "cap");
//    lslideSwitch = hardwareMap.get(DigitalChannel.class, "slideLimit");
    sensorRangeRear = hardwareMap.get(DistanceSensor.class, "sensorRangeRear");
    sensorRangeLeft = hardwareMap.get(DistanceSensor.class, "sensorRangeLeft");
    sensorRangeRight = hardwareMap.get(DistanceSensor.class, "sensorRangeRight");

    DcMotor lLiftMotor = hardwareMap.get(DcMotor.class, "motorLiftLeft");
    DcMotor rLiftMotor = hardwareMap.get(DcMotor.class, "motorLiftRight");
    lift = new LiftControl(op, lLiftMotor, rLiftMotor);
    sensorColorBottom = hardwareMap.get(ColorSensor.class, "sensorColorBottom");

//    lGrabber = hardwareMap.get(Servo.class, "lGrabber");
//    rGrabber = hardwareMap.get(Servo.class, "rGrabber");

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
    // Set the digital channel mode to
    // Output mode can be used to blink LED's
//    lslideSwitch.setMode(DigitalChannel.Mode.INPUT);

    //lGrabber.setDirection(Servo.Direction.FORWARD);
    //rGrabber.setDirection(Servo.Direction.REVERSE);

    //lClaw.setDirection(Servo.Direction.FORWARD);
    //rClaw.setDirection(Servo.Direction.REVERSE);
    // TODO: Add initialization / calibration for the slide and lift?

    // Shamelessly copied from example code...
    sleep(2000);
    // Start the logging of measured acceleration
    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
  }

  // Linear slide stuff:
/*  public boolean slideSwitchSignaled() {
    return !lslideSwitch.getState();
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
    //lClaw.setPosition(val);
    //rClaw.setPosition(val);
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

  public void bpGrabber(double pos) {
    lGrabber.setPosition(pos);
    rGrabber.setPosition(pos);
  }

  public void blockFlipper(double pos) {
    blockFlipper.setPosition(pos);
  }

  public void capstone(double speed) {
    cap.setPower(-speed);
  }
*/
  // 0 = facing toward the driver (6 O'Clock)
  // 90 = 9 O'clock
  // -90 = 3:00
  public double gyroHeading() {
    Orientation angles1 =
      imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    return -AngleUnit.DEGREES.fromUnit(angles1.angleUnit, angles1.firstAngle + 180);
  }

  // 0 = facing away from driver (12 O'Clock)
  // 90 degrees: 3:00
  // -90 degrees: 9:00
  public double gyroHeading2() {
    Orientation angles1 =
      imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    return -AngleUnit.DEGREES.fromUnit(angles1.angleUnit, angles1.firstAngle);
  }

/*  void setServoDirection(Servo.Direction direction) {
    turn.setDirection(direction);
  }

  void setServoPosition(double position) {
    turn.setPosition(position);
  }
*/
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
    } else if (gyroAngle > 140 && gyroAngle <= 180) {
      test = 180 - gyroAngle;
    } else if (gyroAngle >= -180 && gyroAngle < -140) {
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
    double rotSpeed = XDrive.getSteppedValue(ZRobot.snapSteps, dir / 180);
    return new Direction(rotSpeed, 0);
  }

  //for autonomous only
  private void toAngleSync(double to) {
    double dir;
    do {
      Direction turn = getDirectionTowardAngle(to);
      dir = turn.X;
      joystickDrive(Direction.None, turn, gyroHeading());
      sleep(10);
    } while (Math.abs(dir) > 3);
  }

  // Snap the robot to the closest 90 degree angle
  public double snap() {
    double curr = gyroHeading();
    double newangle = toNearestAngle(curr);
    //telemetry.addData("Snap:", String.format("curr: %3.3f new: %3.3f", curr, newangle));
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
    //telemetry.addData("control rstick X: ", j2.X);
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

  public void setTurningSpeed(double angleDelta) {
    if (Math.abs(angleDelta) < 25) {
      speedSnail();
    } else if (Math.abs(angleDelta) < 75) {
      speedNormal();
    } else {
      speedTurbo();
    }
  }

  // Turn to the angle specified
  // FYI: 0 is facing 'away' from the driver
  // 90 == 3:00, -90 == 9:00, 0 == 6:00
  public void fastSyncTurn(double angle, double time) {
    ElapsedTime runTime = new ElapsedTime();
    runTime.reset();
    while (opMode.opModeIsActive() &&
      runTime.seconds() < time &&
      Math.abs(gyroHeading2() - angle) > 4) {
      if (gyroHeading2() > angle + 4) {
        setTurningSpeed(gyroHeading2() - angle);
        joystickDrive(Direction.None, new Direction(-0.5, 0), gyroHeading2());
      } else if (gyroHeading2() < angle - 4) {
        setTurningSpeed(angle - gyroHeading2());
        joystickDrive(Direction.None, new Direction(0.5, 0), gyroHeading2());
      }
      //telemetry.addData("gyro:", gyroHeading());
      //telemetry.addData("gyro2:", gyroHeading2());
      //telemetry.update();
    }
    stop();
  }

  // This will travel toward the rear until it gets to 'dist'
  public void fastRearDrive(double dist) {
    // Update: No attention should be paid to 'speed'
    // Just drive and slow down when we get slow to the target
    ElapsedTime tm = new ElapsedTime();
    double curDistance = rearDistance();
    fastSyncTurn(0, 2);
    while (opMode.opModeIsActive() && Math.abs(curDistance - dist) > 4 && tm.time() < 3.0) {
      //telemetry.addData("Current Distance", curDistance);
      //telemetry.update();
      double dir = (dist < curDistance) ? 1 : -1;
      double magnitude = Math.abs(dist - curDistance);
      if (magnitude < SNAILDISTANCE) {
        speedSnail();
      } else if (magnitude < TURBODISTANCE) {
        speedNormal();
      } else {
        speedTurbo();
      }
      double heading = gyroHeading();
      //double turn = (heading < 0) ? .5 : -.5;
      //Direction rotation = new Direction((Math.abs(heading) > 175) ? turn : 0, 0);
      joystickDrive(new Direction(0, dir), Direction.None, heading);
      curDistance = rearDistance();
      sleep(10);
    }
    driveTrain.stop();
    speedNormal();
  }

  public void fastLeftDrive(double dist) {
    // Update: No attention should be paid to 'speed'
    // Just drive and slow down when we get slow to the target
    ElapsedTime tm = new ElapsedTime();
    double curDistance = leftDistance();
    fastSyncTurn(0, 2);
    while (opMode.opModeIsActive() && Math.abs(curDistance - dist) > 4 && tm.time() < 3.0) {
      telemetry.addData("Current Distance", curDistance);
      telemetry.update();
      double dir = (dist < curDistance) ? -1 : 1;
      double magnitude = Math.abs(dist - curDistance);
      if (magnitude < SNAILDISTANCE) {
        speedSnail();
      } else if (magnitude < TURBODISTANCE) {
        speedNormal();
      } else {
        speedTurbo();
      }
      double heading = gyroHeading();
      //double turn = (heading < 0) ? .5 : -.5;
      //Direction rotation = new Direction((Math.abs(heading) > 175) ? turn : 0, 0);
      joystickDrive(new Direction(dir, 0), Direction.None, heading);
      curDistance = leftDistance();
      sleep(10);
    }
    driveTrain.stop();
    speedNormal();
  }

  public void fastRightDrive(double dist) {
    // Update: No attention should be paid to 'speed'
    // Just drive and slow down when we get slow to the target
    ElapsedTime tm = new ElapsedTime();
    double curDistance = rightDistance();
    fastSyncTurn(0, 2);
    while (opMode.opModeIsActive() && Math.abs(curDistance - dist) > 4 && tm.time() < 3.0) {
      telemetry.addData("Current Distance", curDistance);
      telemetry.update();
      double dir = (dist < curDistance) ? -1 : 1;
      double magnitude = Math.abs(dist - curDistance);
      if (magnitude < SNAILDISTANCE) {
        speedSnail();
      } else if (magnitude < TURBODISTANCE) {
        speedNormal();
      } else {
        speedTurbo();
      }
      // This should be "close" to 0 degrees
      double heading = gyroHeading2();
      Direction rotation = new Direction(-heading / 100, 0);
      joystickDrive(new Direction(dir, 0), rotation, heading);
      curDistance = rightDistance();
      sleep(10);
    }
    driveTrain.stop();
    speedNormal();
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

  public void initGyro() {
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    imu.initialize(parameters);
    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
  }


}
