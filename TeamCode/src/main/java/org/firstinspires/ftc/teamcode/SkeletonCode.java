package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class SkeletonCode {
    private Robot robot;
    private Controller control;
    private Controller driver;

    public SkeletonCode(Gamepad ctrl, Gamepad drv, Robot r) {
        control = new Controller(ctrl);
        driver = new Controller(drv);
        robot = r;
    }


    public void directcontrol() {
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
                    if (dir.isUp()) {
//Pushedup:
                        if (robot.isFourBarUpperLimit() == true) {
//HITTHEMLIMIT!
                            robot.fourBarMotor(FourBarDirection.Off);
                        } else {
                            robot.fourBarMotor(FourBarDirection.Up);
                        }
                    }
                    if (dir.isDown()) {
//Pusheddown
                        if (robot.isFourBarLowerLimit() == true) {
                            robot.fourBarMotor(FourBarDirection.Off);
                        } else {
                            robot.fourBarMotor(FourBarDirection.Down);
                        }
                    }
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
