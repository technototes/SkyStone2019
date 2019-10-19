package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.robotcore.external.Func;

public class XDrive {
    public static final double SCALEFACTOR = 0.5;//turn speed factor
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
    public static double flPower;
    public static double frPower;
    public static double rrPower;
    public static double rlPower;
    public static double tturn;
    //leave gyroAngle at zero to set relative angle
    public void joystickDrive(Direction j1, Direction j2, double gyroAngle){
        double hypotenuse = Functions.pyt(j1.X, j1.Y);
        drive(Math.acos(j1.X/hypotenuse), gyroAngle, Range.clip(hypotenuse, -1.0, 1.0), j2.X);
    }
    public void drive(double joystickAngle, double gyroAngle, double power, double turn) {
        tturn = turn*SCALEFACTOR;
        double angle = joystickAngle+robot.gyro();
        flPower = power*Math.cos((angle-45)/(180/Math.PI))+tturn;
        frPower = -power*Math.cos((angle+45)/(180/Math.PI))+tturn;
        rrPower = -power*Math.cos((angle-45)/(180/Math.PI))+tturn;
        rlPower = power*Math.cos((angle+45)/(180/Math.PI))+tturn;
        robot.motorFrontLeft(flPower);
        robot.motorFrontRight(frPower);
        robot.motorRearLeft(rlPower);
        robot.motorRearRight(rrPower);
    }
    //set nearestSnap to true to snap to nearest 90 dgree angle, or set nearestSnap to false and input angle to snap to.
    public double snapToAngle(double gyroAngle){
        double test = 0;
        bb:
        if(true) {
            if (gyroAngle > 50 && gyroAngle < 130) {
                test = 90 - gyroAngle;
                break bb;
            }
            if (gyroAngle > 140 && gyroAngle < 220) {
                test = 180 - gyroAngle;
                break bb;
            }
            if (gyroAngle > 230 && gyroAngle < 310) {
                test =  270 - gyroAngle;
                break bb;
            }
            if ((gyroAngle >= 0 && gyroAngle < 40) || (gyroAngle > 320 && gyroAngle <= 360)) {
                test = 0 - gyroAngle;
                break bb;
            }
            test = gyroAngle;
        }
        return test;
    }
}


