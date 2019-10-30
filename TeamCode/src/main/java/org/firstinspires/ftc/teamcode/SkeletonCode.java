package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
@TeleOp(name = "Skeleton code")
public class SkeletonCode extends OpMode {
    private TTRobot robot;
    private Controller control;
    private Controller driver;

    @Override
    public void runOpMode() {
        robot = new TTRobot();
        driver = new Controller(gamepad1);
        control = new Controller(gamepad2);
        robot.init(harwareMap);
        while (true) {
            //ASSERT(we'realreadyatthebrick)
            if (control.buttonA() == Button.Pressed) {
                if (robot.getGrabberPosition() == GrabberPosition.Vertical) {
                    robot.setGrabberPosition(GrabberPosition.Horizontal);
                } else {//It'sHORIZONTAL!
                    robot.setGrabberPosition(GrabberPosition.Vertical);
                }
            }
            if (control.buttonX() == Button.Pressed) {
                robot.grabberClutch(GrabberMotorOperation.Open);
            } else if (control.buttonY() == Button.Pressed) {
                robot.grabberClutch(GrabberMotorOperation.Close);
            } else {
                robot.grabberClutch(GrabberMotorOperation.Off);
            }
            Direction dpadDirection = control.dpad();
            if (dpadDirection.isRight()) {
                if (robot.isLinearSlideFullyExtended() == true) {
                    robot.lslide(LinearSlideOperation.Off);
                } else if (robot.isLinearSlideFullyRetracted() == true) {
                    robot.lslide(LinearSlideOperation.Off);
                } else {
                    robot.lslide(LinearSlideOperation.Extend);
                }
            } else if (dpadDirection.isLeft()) {
                if (robot.isLinearSlideFullyExtended() == true) {
                    robot.lslide(LinearSlideOperation.Off);
                } else if (robot.isLinearSlideFullyRetracted() == true) {
                    robot.lslide(LinearSlideOperation.Off);
                } else {
                    robot.lslide(LinearSlideOperation.Retract);
//while(true){
                    Direction dir = control.lstick();
                    /*
                    if (dir.isUp()) {
//Pushedup:
                        /*
                        if (robot.isFourBarUpperLimit() == true) {
//HITTHEMLIMIT!
                            robot.fourBarMotor(LiftDirection.Off);
                        } else {
                            robot.fourBarMotor(LiftDirection.Up);
                        }
                    }
                    if (dir.isDown()) {
//Pusheddown
                        if (robot.isFourBarLowerLimit() == true) {
                            robot.fourBarMotor(LiftDirection.Off);
                        } else {
                            robot.fourBarMotor(LiftDirection.Down);
                        }
                    }
                     */
                }

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
}
