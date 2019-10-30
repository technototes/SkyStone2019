package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
@TeleOp(name = "Skeleton code")
public class DirectControl extends LinearOpMode {
    private TTRobot robot;
    private Controller control;
    private Controller driver;

    @Override
    public void runOpMode() {
        robot = new TTRobot();
        driver = new Controller(gamepad1, telemetry);
        control = new Controller(gamepad2, telemetry);
        robot.init(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()) {

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
            //redid this to work with magnetic limit switch
            Direction dpad = control.dpad();
            if(dpad.X == 1){
                robot.lslide(LinearSlideOperation.Extend);
            }else if(dpad.X == -1){
                robot.lslide(LinearSlideOperation.Retract);
            }

            Direction dir = control.lstick();
            robot.setLift(dir.Y);

            Direction L = driver.lstick();
            Direction R = driver.rstick();
            if (L.X > 0.01 || L.X < -0.01 ||
                L.Y > 0.01 || L.Y < -0.01 ||
                R.X > 0.01 || R.X < -0.01) {
                robot.joystickDrive(L, R, robot.gyroHeading());
            }
            if(Math.abs(control.rstick().X) > 0.05){
                robot.drive(0,0, 0,  control.rstick().X);
            }
            telemetry.update();
        }
    }
}
