package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//@TeleOp(name = "Basic: Robot op mode", group = "Linear Opmode")
public class Robot /*extends LinearOpMode*/ {

    private Servo slide = null;
    private DcMotor flMotor = null;
    private DcMotor frMotor = null;
    private DcMotor rlMotor = null;
    private DcMotor rrMotor = null;
    private DcMotor liftMotor = null;
    private Servo Turn = null;
    private Servo claw = null;
    private TouchSensor extended = null;
    private TouchSensor retracted = null;
    private ElapsedTime runtime = new ElapsedTime();
    private Servo basePlateGrabber = null;
    public GyroSensor gyro = null;

    public Robot() {
    }

    public void init(HardwareMap hardwareMap) {
        slide = hardwareMap.get(Servo.class, "lslide");
        Turn = hardwareMap.get(Servo.class, "grabTurn");
        claw = hardwareMap.get(Servo.class, "claw");
        basePlateGrabber = hardwareMap.get(Servo.class, "BPGrabber");
        extended = hardwareMap.get(TouchSensor.class, "extLimitSwitch");
        retracted = hardwareMap.get(TouchSensor.class, "retLimitSwitch");

        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        rlMotor = hardwareMap.get(DcMotor.class, "rlMotor");
        rrMotor = hardwareMap.get(DcMotor.class, "rrMotor");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

        gyro = hardwareMap.get(GyroSensor.class, "gyro");
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
        if (dir == LinearSlideOperation.Extend) {
            // TODO: This is probably wrong
            slide.setPosition(1.0);
        } else if (dir == LinearSlideOperation.Retract) {
            // TODO: This is probably wrong & silly
            slide.setPosition(-1.0);
        }
    }

    // Grabber stuff:
    public void grabberClutch(GrabberMotorOperation operation) {
        // TODO: Fill this in
        if (operation == GrabberMotorOperation.Close) {
            //claw.
        }
    }

    public GrabberPosition getGrabberPosition() {
        // TODO: Fill this in (read the switch?)
        return GrabberPosition.Horizontal;
    }

    public void setGrabberPosition(GrabberPosition position) {
        // TODO: Fill this in: Set the servo!
    }

    // Lift stuff:
    public void fourBarMotor(FourBarDirection dir) {
        // TODO: Fill this in
        // Set the motor power
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

    public void motorLift(double power) {
        liftMotor.setPower(power);
    }

    public double gyroHeading() {
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

    public void snap() {
        double curr = gyroHeading();
        double newangle = snapToAngle(curr);
        drive(0.0, 0.0,0.0, newangle);

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
