package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Robot op mode", group="Linear Opmode")
public class Robot extends LinearOpMode {

  private DcMotor linearSlideMotor = null;
  private Servo grab = null;
  private Servo claw = null;
  private TouchSensor extended = null;
  private TouchSensor retracted = null;
  private ElapsedTime runtime = new ElapsedTime();


  public Robot() {
    linearSlideMotor = hardwareMap.get(DcMotor.class, "lslide");
    grab = hardwareMap.get(Servo.class, "grabTurn");
    claw = hardwareMap.get(Servo.class, "claw");
    extended = hardwareMap.get(TouchSensor.class, "extLimitSwitch");
    retracted = hardwareMap.get(TouchSensor.class, "retLimitSwitch");
  }

  @Override
  public void runOpMode() throws InterruptedException {

  }

  // Linear slide stuff:
  public boolean isLinearSlideFullyExtended() {
    return extended.isPressed();
  }
  public boolean isLinearSlideFullyRetracted() {
    return retracted.isPressed();
  }
  public void lslide(LinearSlideOperation dir) {
    // Set the linear slide motor to the position
    if (dir == LinearSlideOperation.Extend) {
      // TODO: This is probably wrong & silly
      linearSlideMotor.setPower(1.0);
    } else if (dir == LinearSlideOperation.Retract) {
      // TODO: This is probably wrong & silly
      linearSlideMotor.setPower(-1.0);
    }
  }

  // Grabber stuff:
  public void grabberClutch(GrabberMotorOperation operation) {
    if (operation == GrabberMotorOperation.Close) {
      //claw.
    }
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
