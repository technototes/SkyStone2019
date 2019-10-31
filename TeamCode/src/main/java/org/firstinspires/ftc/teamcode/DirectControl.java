package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Skeleton code")
public class DirectControl extends LinearOpMode {
    private TTRobot robot;
    private Controller control;
    private Controller driver;

    @Override
    public void runOpMode() {
        robot = new TTRobot();
        // If you want telemetry, include a name as a string
        // If you don't want telemetry, pass a null:
        driver = new Controller(gamepad1, telemetry, "driver");
        control = new Controller(gamepad2, telemetry, null);
        robot.init(hardwareMap, telemetry);
        robot.calibrate(); // OOPS!
        waitForStart();
        while (opModeIsActive()) {

            // Handle Grabber rotation
            if (control.buttonA() == Button.Pressed) {
                if (robot.getGrabberPosition() == GrabberPosition.Vertical) {
                    robot.snapGrabberPosition(GrabberPosition.Horizontal);
                } else {//It'sHORIZONTAL!
                    robot.snapGrabberPosition(GrabberPosition.Vertical);
                }
            }
            // Handle Grabber clutch
            if (control.buttonX() == Button.Pressed) {
                robot.grabberClutch();
            }
            //redid this to work with magnetic limit switch
            /*Direction dpad = control.dpad();
            if(dpad.X == 1){
                robot.lslide(LinearSlideOperation.Extend);
            }else if(dpad.X == -1){
                robot.lslide(LinearSlideOperation.Retract);
            }*/

            // Lift control:
            Direction dir = control.lstick();
            if(Math.abs(dir.Y) > robot.STICKDEADZONE)
                robot.setLift(dir.Y);

            // Driver control:
            Direction L = driver.lstick();
            Direction R = driver.rstick();
            if (Math.abs(L.X) > robot.STICKDEADZONE ||
                Math.abs(L.Y) > robot.STICKDEADZONE ||
                Math.abs(R.X) > robot.STICKDEADZONE) {
                robot.joystickDrive(L, R, robot.gyroHeading());
            }
            // The attachments controller can also rotate the robot
            R = control.rstick();
            if(Math.abs(R.X) > robot.STICKDEADZONE){
                robot.drive(0,0, 0,  R.X);
            }
            telemetry.update();
        }
    }
}
