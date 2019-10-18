package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.*;

public class XDrive {
    public static final double SCALEFACTOR = 0.2;//turn speed factor
    public double joystickAngle;//input from joystick
    public double dangle;//travel angle DO NOT SET
    public double sangle;//gyro-sensor angle
    public double power ;
    public Robot robot;
    public Controller controller;
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
    public static double turn;
    public void drive() {
        while(true) {
            readController();
            setDriveMotorPower();
            //something like this
            robot.motorFrontLeft(flPower);
            robot.motorFrontRight(frPower);
            robot.motorRearLeft(rlPower);
            robot.motorRearRight(rrPower);
        }

    }
    public void setDriveMotorPower() {
        dangle = joystickAngle+robot.gyro();
        flPower = power*Math.cos((dangle-45)/(180/Math.PI))+turn;
        frPower = -power*Math.cos((dangle+45)/(180/Math.PI))+turn;
        rrPower = -power*Math.cos((dangle-45)/(180/Math.PI))+turn;
        rlPower = power*Math.cos((dangle+45)/(180/Math.PI))+turn;
    }
    public void readController() {
        Button left = controller.lbump();
        Button right = controller.rbump();
        if(left != right) {
            if(left == Button.Pressed) {
                turn = -1;
            }else {
                turn = 1;
            }
        }
        turn *= SCALEFACTOR;
        Direction j = controller.rstick();
        double hypotenuse = Functions.pyt(j.X, j.X);
        power = Range.clip(hypotenuse, -1.0, 1.0);
        joystickAngle = Math.acos(j.X/hypotenuse);
    }
}


