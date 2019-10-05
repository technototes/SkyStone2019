package org.firstinspires.ftc.teamcode;

public class Robot {

  // Linear slide stuff:
  public boolean isLinearSlideFullyExtended() {
    // TODO: Read the linear slide limit switch
    return false;
  }
  public boolean isLinearSlideFullyRetracted() {
    // TODO: Read the linear slide retract switch
    return false;
  }
  public void lslide(LinearSlideOperation dir) {
    // Set the linear slide motor to the position
    if (dir == LinearSlideOperation.Extend) {
      // TODO: Fill this in
    } else if (dir == LinearSlideOperation.Retract) {
      // TODO: Fill this in
    }
  }

  // Grabber stuff:
  public void grabberClutch(GrabberMotorOperation operation) {
  }
  public GrabberPosition getGrabberPosition() {
    return GrabberPosition.Horizontal;
  }
  public void setGrabberPosition(GrabberPosition position) {
  }

  // Lift stuff:
  public void fourBarMotor(FourBarDirection dir) {}
  public boolean isFourBarUpperLimit() { return false; }
  public boolean isFourBarLowerLimit() { return false; }
}
