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

    public Robot() { }

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
}
