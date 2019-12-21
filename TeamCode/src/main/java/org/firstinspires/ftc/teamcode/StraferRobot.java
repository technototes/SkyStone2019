package org.firstinspires.ftc.teamcode;

import android.content.res.Resources;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.robot.Robot;

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

public class StraferRobot implements IRobot {
  // Scaling values

  // The amount we divide speed by when dropping the lift
  // private static final double DOWNWARDLIFTSCALE = 2.0;
  // // the power of the linear slide
  // private static final double LINEARSLIDEPOWER = 1.0;

  // // Dead zones

  // // This is the middle 'dead zone' of the analog sticks
  // public static final double STICK_DEAD_ZONE = 0.05;
  // // Triggers must be pushed at least this far
  // public static final double TRIGGERTHRESHOLD = 0.25;

  // // Claw grab positions
  // public static final double CLAWOPENPOSITION = 0.4;
  // public static final double CLAWCLOSEPOSITION = 0.6;

  // Distance speeds in cm
  public static final double TURBODISTANCE = 65;
  public static final double SNAILDISTANCE = 15;

  // // Unused stuff
  // // the grab rotation position for snapping to horizontal or vertical
  // private static final double GRABBERPOSITIONCUTOFF = 0.25;
  // // the grab rotation 'horizontal' position
  // private static final double HORIZONTALGRABBERPOSITION = 0.0;
  // // the grab rotation 'vertical' position
  // private static final double VERTICALGRABBERPOSITION = 0.5;


  private boolean isGrabberOpened = true;


  private CRServo slide = null;

  private Servo turn = null;
  private Servo lClaw = null;
  private Servo rClaw = null;
  private Servo blockFlipper = null;
  private CRServo cap = null;
  private ColorSensor sensorColorBottom = null;
  public DistanceSensor sensorRangeRear = null;
  public DistanceSensor sensorRangeLeft = null;
  public DistanceSensor sensorRangeRight = null;
  public DistanceSensor sensorRangeFront = null;
  private Servo lGrabber = null;
  private Servo rGrabber = null;

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

  public StraferRobot(LinearOpMode op, HardwareMap hardwareMap, Telemetry tel) {
    opMode = op;
    telemetry = tel;
    // // Get handles to all the hardware
    // slide = hardwareMap.get(CRServo.class, "lslideServo");
    // turn = hardwareMap.get(Servo.class, "grabTurn");
    // lClaw = hardwareMap.get(Servo.class, "lClaw");
    // rClaw = hardwareMap.get(Servo.class, "rClaw");
    // blockFlipper = hardwareMap.get(Servo.class, "blockFlipper");
    // cap = hardwareMap.get(CRServo.class, "cap");
    // lslideSwitch = hardwareMap.get(DigitalChannel.class, "slideLimit");
    // liftSwitch = hardwareMap.get(DigitalChannel.class, "liftLimit");
    //sensorRangeRear = hardwareMap.get(DistanceSensor.class, "sRear");
    //sensorRangeLeft = hardwareMap.get(DistanceSensor.class, "sLeft");
    //sensorRangeRight = hardwareMap.get(DistanceSensor.class, "sRight");
    //sensorRangeFront = hardwareMap.get(DistanceSensor.class, "sFront");
    // lLiftMotor = hardwareMap.get(DcMotor.class, "motorLiftLeft");
    // rLiftMotor = hardwareMap.get(DcMotor.class, "motorLiftRight");
    // sensorColorBottom = hardwareMap.get(ColorSensor.class, "sensorColorBottom");

    // lGrabber = hardwareMap.get(Servo.class, "lGrabber");
    // rGrabber = hardwareMap.get(Servo.class, "rGrabber");

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


    // Start the logging of measured acceleration
    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
  }


  // 0 = facing toward the driver (6 O'Clock)
  // 90 = 9 O'clock
  // -90 = 3:00
  public double gyroHeading() {
    Orientation angles1 =
      imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    return -AngleUnit.DEGREES.fromUnit(angles1.angleUnit, angles1.firstAngle);
  }

  public double getXAccel(){
    return imu.getAcceleration().xAccel;
  }
  public double getYAccel(){
    return imu.getAcceleration().yAccel;
  }

  // 0 = facing away from driver (12 O'Clock)
  // 90 degrees: 3:00
  // -90 degrees: 9:00
  public double gyroHeading2() {
    Orientation angles1 =
      imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    return -AngleUnit.DEGREES.fromUnit(angles1.angleUnit, angles1.firstAngle);
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
    double rotSpeed = XDrive.getSteppedValue(StraferRobot.snapSteps, dir / 180);
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
  public double snap() {
    double curr = gyroHeading();
    double newangle = toNearestAngle(curr);
    telemetry.addData("Snap:", String.format("curr: %3.3f new: %3.3f", curr, newangle));
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
  //better now
  public void fastSyncTurn(double angle, double time) {
    ElapsedTime runTime = new ElapsedTime();
    runTime.reset();
    while (opMode.opModeIsActive() &&
      runTime.seconds() < time &&
      Math.abs(gyroHeading() - angle) > 10) {
      if (angle > gyroHeading() - 2) {
        setTurningSpeed(gyroHeading() - angle);
        joystickDrive(Direction.None, new Direction(-0.5, 0), gyroHeading());
      } else if (angle < gyroHeading() + 2) {
        setTurningSpeed(angle - gyroHeading());
        joystickDrive(Direction.None, new Direction(0.5, 0), gyroHeading());
      }
      telemetry.addData("gyro:", gyroHeading());
      //telemetry.addData("gyro2:", gyroHeading());
      telemetry.update();
    }
    stop();
  }

  // This will travel toward the rear until it gets to 'dist'


  // This attempts to drive in a straight(ish) line toward a corner
  public void distRearLeftDrive(double speed, double rearDist, double leftDist) {
    double rDist, lDist;
    ElapsedTime tm = new ElapsedTime();
    fastSyncTurn(0, 2);
    do {

      // Let's figure out what angle to drive toward to make a straightish line
      // TODO: I have no idea if this is the proper angle or not
      // TODO: Might need to do something like 90 - angle
      rDist = getCappedRange(sensorRangeRear, 1500);
      lDist = getCappedRange(sensorRangeLeft, 1500);
      //double dir = (rearDist < rDist) ? -1 : 1;
      double magnitude = Math.abs(rearDist - rDist);
      if (magnitude < SNAILDISTANCE) {
        speedSnail();
      } else if (magnitude < TURBODISTANCE) {
        speedNormal();
      } else {
        speedTurbo();
      }
      //double angle = Math.atan2(rDist, rtDist);
      //angle = AngleUnit.DEGREES.fromRadians(angle);
      joystickDrive(new Direction((leftDist-lDist)/(rDist-rearDist), (rDist-rearDist)/(lDist-leftDist)), new Direction(0, 0), gyroHeading());
      sleep(10);
      telemetry.addData("rdist ", rDist);
      telemetry.addData("ldist ", lDist);
      telemetry.update();

    } while ((rDist > rearDist|| lDist > leftDist) && tm.time() < 10.0 && opMode.opModeIsActive());
    driveTrain.stop();
  }

  // This attempts to drive in a straight(ish) line toward a corner
  public void distRearRightDrive(double speed, double rearDist, double rightDist) {
    double rDist, rtDist;
    ElapsedTime tm = new ElapsedTime();
    //fastSyncTurn(0, 2);
    do {
      // Let's figure out what angle to drive toward to make a straightish line
      // TODO: I have no idea if this is the proper angle or not
      // TODO: Might need to do something like 90 - angle
      rDist = getCappedRange(sensorRangeRear, 1500);
      rtDist = getCappedRange(sensorRangeRight, 1500);
      //double dir = (rearDist < rDist) ? -1 : 1;
      double magnitude = Math.abs(rearDist - rDist);
      if (magnitude < SNAILDISTANCE) {
        speedSnail();
      } else if (magnitude < TURBODISTANCE) {
        speedNormal();
      } else {
        speedTurbo();
      }
      //double angle = Math.atan2(rDist, rtDist);
      //angle = AngleUnit.DEGREES.fromRadians(angle);
      joystickDrive(new Direction((rtDist-rightDist)/(rDist-rearDist), (rDist-rearDist)/(rtDist-rightDist)), new Direction(0, 0), gyroHeading());
      sleep(10);
      telemetry.addData("rdist ", rDist);
      telemetry.addData("rtdist ", rtDist);
      telemetry.update();

    } while ((rDist > rearDist || rtDist > rightDist) && tm.time() < 10.0 && opMode.opModeIsActive());
    driveTrain.stop();
  }
  public void fastRearDrive(double dist) {
    // Update: No attention should be paid to 'speed'
    // Just drive and slow down when we get slow to the target
    ElapsedTime tm = new ElapsedTime();
    double curDistance = sensorRangeRear.getDistance(DistanceUnit.CM);
    fastSyncTurn(0, 2);
    while (opMode.opModeIsActive() && Math.abs(curDistance - dist) > 4 && tm.time() < 3.0) {
      telemetry.addData("Current Distance", curDistance);
      telemetry.update();
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
      curDistance = sensorRangeRear.getDistance(DistanceUnit.CM);
      sleep(10);
    }
    driveTrain.stop();
    speedNormal();
  }
  public void fastFrontDrive(double dist) {
    // Update: No attention should be paid to 'speed'
    // Just drive and slow down when we get slow to the target
    ElapsedTime tm = new ElapsedTime();
    double curDistance = sensorRangeFront.getDistance(DistanceUnit.CM);
    fastSyncTurn(0, 2);
    while (opMode.opModeIsActive() && Math.abs(curDistance - dist) > 4 && tm.time() < 3.0) {
      telemetry.addData("Current Distance", curDistance);
      telemetry.update();
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
      joystickDrive(new Direction(0, -dir), Direction.None, heading);
      curDistance = sensorRangeFront.getDistance(DistanceUnit.CM);
      sleep(10);
    }
    driveTrain.stop();
    speedNormal();
  }

  public void fastLeftDrive(double dist) {
    // Update: No attention should be paid to 'speed'
    // Just drive and slow down when we get slow to the target
    ElapsedTime tm = new ElapsedTime();
    double curDistance = sensorRangeLeft.getDistance(DistanceUnit.CM);
    //fastSyncTurn(0, 2);
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
      curDistance = sensorRangeLeft.getDistance(DistanceUnit.CM);
      sleep(10);
    }
    driveTrain.stop();
    speedNormal();
  }

  public void fastRightDrive(double dist) {
    // Update: No attention should be paid to 'speed'
    // Just drive and slow down when we get slow to the target
    ElapsedTime tm = new ElapsedTime();
    double curDistance = sensorRangeRight.getDistance(DistanceUnit.CM);
    //fastSyncTurn(0, 2);
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
      double heading = gyroHeading();
      Direction rotation = new Direction(-heading / 100, 0);
      joystickDrive(new Direction(dir, 0), rotation, heading);
      curDistance = sensorRangeRight.getDistance(DistanceUnit.CM);
      sleep(10);
    }
    driveTrain.stop();
    speedNormal();
  }

  private double getCappedRange(DistanceSensor sens, double cap) {
    double res = sens.getDistance(DistanceUnit.CM);
    return Range.clip(res, 0.01, cap);
  }
  public void initGyro() {
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    imu.initialize(parameters);
    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
  }
  public void rearDist(){
    while(opMode.opModeIsActive()) {
      telemetry.addData("sensorVal: ", sensorRangeRear.getDistance(DistanceUnit.CM));
      telemetry.addData("val: ", Math.cos(Math.toRadians(gyroHeading()))*sensorRangeRear.getDistance(DistanceUnit.CM));
      telemetry.addData("gyro: ", gyroHeading());
      telemetry.update();
    }
  }
}
