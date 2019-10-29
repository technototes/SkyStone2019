package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//@TeleOp(name = "Basic: Robot op mode", group = "Linear Opmode")
public class Robot /*extends LinearOpMode*/ {

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
    public GyroSensor gyro = null;
    private TouchSensor touch = null;

    private final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public Robot() {
    }

    public void init(HardwareMap hardwareMap) {
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

        gyro = hardwareMap.get(GyroSensor.class, "gyro");
        touch = hardwareMap.get(TouchSensor.class, "touch");

    }

    public void calibrate() {
        lLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        gyro.calibrate();
        while (gyro.isCalibrating()) {
            sleep(50);
            Thread.yield();
        }
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
                slide.setPower(0.0);
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
        slide.setPower(0.8);
        sleep(500);
        slide.setPower(0);
    }

    // Grabber stuff:
    public void grabberClutch(GrabberMotorOperation operation) {
        // TODO: Check this...
        switch (operation) {
            case Close:
                claw.setPosition(0.0);
                break;
            case Open:
                claw.setPosition(0.5);
                break;
            case Off:
                claw.setPosition(0.2);
                break;
        }
    }

    public GrabberPosition getGrabberPosition() {
        // TODO: Check this...
        double pos = turn.getPosition();
        if (pos < 0.25) {
            return GrabberPosition.Horizontal;
        } else {
            return GrabberPosition.Vertical;
        }
    }

    public void setGrabberPosition(GrabberPosition position) {
        switch (position) {
            case Horizontal:
                turn.setPosition(0.0);
                break;
            case Vertical:
                turn.setPosition(0.25);
                break;
        }
    }

    // Lift stuff:
    public void fourBarMotor(FourBarDirection dir) {
        double power = 0.0;
        switch (dir) {
            case Off:
                return;
            case Up:
                power = 0.8;
                break;
            case Down:
                power = -0.8;
                break;
        }
        lLiftMotor.setPower(power);
        rLiftMotor.setPower(power);
    }

    public boolean isFourBarUpperLimit() {
        // TODO: Read the upper limit switch
        return false;
    }

    public boolean isFourBarLowerLimit() {
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
        // TODO: Fix this. This is clearly busted :/
        return gyro.getHeading();
    }

    public final double SCALEFACTOR = 0.5;//turn speed factor
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

    //leave gyroAngle at zero to set relative angle
    public void joystickDrive(Direction j1, Direction j2, double gyroAngle) {
        double hypotenuse = Functions.pyt(j1.X, j1.Y);
        drive(Math.acos(j1.X / hypotenuse), gyroAngle, Range.clip(hypotenuse, -1.0, 1.0), j2.X);
    }

    public void drive(double joystickAngle, double gyroAngle, double power, double turn) {
        tturn = turn * SCALEFACTOR;
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

    //set nearestSnap to true to snap to nearest 90 dgree angle, or set nearestSnap to false and input angle to snap to.
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
            powerCompY = (Math.cos(robotHeadingRad) * (Math.cos(angleRad) * speed)) + (Math.sin(robotHeadingRad) * (Math.sin(angleRad) * speed));
            powerCompX = -(Math.sin(robotHeadingRad) * (Math.cos(angleRad) * speed)) + (Math.cos(robotHeadingRad) * (Math.sin(angleRad) * speed));

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
                //telemetry.addData("Speed",  "FL %5.2f:FR %5.2f:RL %5.2f:RR %5.2f", frontLeftSpeed, frontRightSpeed, rearLeftSpeed, rearRightSpeed);
                //telemetry.addData("Gyro", "Heading: " + robot.gyro.getHeading() + " | IntZValue: " + robot.gyro.getIntegratedZValue());
                //telemetry.addData("Gyro", "Heading: " + getRobotHeading());
                //telemetry.update();
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
