package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
@TeleOp(name = "Skeleton code")
public class SkeletonCode extends LinearOpMode {
    private TTRobot robot;
    private Controller control;
    private Controller driver;

    @Override
    public void runOpMode() {
        robot = new TTRobot();
        driver = new Controller(gamepad1);
        control = new Controller(gamepad2);
        robot.init(hardwareMap);
        while (true) {
            //ASSERT(we'realreadyatthebrick)
            if (control.buttonA() == Button.Pressed) {
                if (robot.getGrabberPosition() == GrabberPosition.Vertical) {
                    robot.snapGrabberPosition(GrabberPosition.Horizontal);
                } else {//It'sHORIZONTAL!
                    robot.snapGrabberPosition(GrabberPosition.Vertical);
                }
            }
            if (control.buttonX() == Button.Pressed) {
                robot.grabberClutch();
            }
            // I think Alex wants this to be read from the right stick
            Direction rstickDir = control.rstick();
            robot.lslide(rstickDir.X);

            Direction dir = control.lstick();
            robot.setLift(dir.Y);

            Direction L = driver.lstick();
            Direction R = driver.rstick();
            if (L.X > 0.01 || L.X < -0.01 ||
                L.Y > 0.01 || L.Y < -0.01 ||
                R.X > 0.01 || R.X < -0.01) {
                robot.joystickDrive(L, R, robot.gyroHeading());
           }
        }
    }
}
