package org.firstinspires.ftc.teamcode;

public class SkeletonCode {
    private Robot robot;
    private Controller control;
    private Controller driver;


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
            }
//Drivecontrol

//voidlinearSlide()
//    while(true)

            // {//therobotoverthebaseplate:)
            pos = control.Rstick();
            if (pos.x != 0) {
                dt.rotate(pos.x);
//}
                pos = control.Lstick();
                double x = pos.x;
                double y = pos.y;
//Emily
                double power = Math.sqrt(x * x + y * y);
                double angle = Math.acos(x / power);
                dt.drive(power, angle, robot.gyro);

            }

        }
    }


    void grabber() {
        // ASSERT(we're already at the brick)
        if (control.buttonA() == Button.Pressed) {
            if (robot.getGrabberPosition() == GrabberPosition.Vertical) {
                robot.setGrabberPosition(GrabberPosition.Horizontal);
            } else { // It's HORIZONTAL!
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
    }

    void linearSlide() {
        while (true) { // the robot over the baseplate :)
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
                }
            }
        }
    }

    void lift() {
        while (true) {
            Direction dir = control.lstick();
            if (dir.isUp()) {
                // Pushed up:
                if (robot.isFourBarUpperLimit() == true) {
                    // HIT THEM LIMIT!
                    robot.fourBarMotor(FourBarDirection.Off);
                } else {
                    robot.fourBarMotor(FourBarDirection.Up);
                }
            }
            if (dir.isDown()) {
                // Pushed down
                if (robot.isFourBarLowerLimit() == true) {
                    robot.fourBarMotor(FourBarDirection.Off);
                } else {
                    robot.fourBarMotor(FourBarDirection.Down);
                }
            }
        }
    }
}
