package org.firstinspires.ftc.teamcode;

public class SkeletonCode {
  private Robot robot;
  private Controller control;
  private Controller driver;

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
      if (dpadDirection == Direction.Right) {
        if (robot.isLinearSlideFullyExtended() == true) {
          robot.lslide(LinearSlideOperation.Off);
        } else if (robot.isLinearSlideFullyRetracted() == true) {
          robot.lslide(LinearSlideOperation.Off);
        } else {
          robot.lslide(LinearSlideOperation.Extend);
        }
      } else if (dpadDirection == Direction.Left) {
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
      if (dir == Direction.Up) {
        // Pushed up:
        if (robot.isFourBarUpperLimit() == true) {
          // HIT THEM LIMIT!
          robot.fourBarMotor(FourBarDirection.Off);
        } else {
          robot.fourBarMotor(FourBarDirection.Up);
        }
      }
      if (dir == Direction.Down) {
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
