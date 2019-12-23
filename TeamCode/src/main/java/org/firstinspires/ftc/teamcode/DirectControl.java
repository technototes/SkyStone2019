package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Direct Control")
public class DirectControl extends LinearOpMode {
  private static double FINEDRIVESPEED = 0.2;
  private TTRobot robot;
  private TTRobot robotForTest = null;
  private Controller control;
  private Controller driver;
  private XDriveManualControl manualCtrl;

  void SetTestRobot(TTRobot testRobot) {
    robotForTest = testRobot;
  }

  @Override
  public void runOpMode() {
    // If you want telemetry, include a name as a string
    // If you don't want telemetry, pass a null:
    driver = new Controller(gamepad1, telemetry, "driver");
    control = new Controller(gamepad2, telemetry, "controller");
    robot = (robotForTest != null) ? robotForTest : new TTRobot(this, hardwareMap, telemetry);
    manualCtrl = new XDriveManualControl(robot, driver, control, telemetry);

    waitForStart();
    ElapsedTime sinceLastUsedGrabRotate = new ElapsedTime();
    ElapsedTime timeSinceStart = new ElapsedTime();
    ElapsedTime loopTime = new ElapsedTime();

    while (opModeIsActive()) {
      loopTime.reset();
      // Handle Grabber rotation
      /*if (control.buttonA() == Button.Pressed) {
        if (robot.getGrabberPosition() == GrabberPosition.Vertical) {
          robot.snapGrabberPosition(GrabberPosition.Horizontal);
        } else { // It'sHORIZONTAL!
          robot.snapGrabberPosition(GrabberPosition.Vertical);
        }
      }*/
      // Handle Grabber clutch
      if (control.rtrigger() > robot.TRIGGERTHRESHOLD) {
        robot.setClawPosition(ClawPosition.Open); // Open
      } else if (control.ltrigger() > robot.TRIGGERTHRESHOLD) {
        robot.setClawPosition(ClawPosition.Close); // Closed
      }
      // Grabber rotation
      final double grabRotationDebounceSecs = 0.25;
      if (sinceLastUsedGrabRotate.seconds() > grabRotationDebounceSecs) {
        if (control.lbump() == Button.Pressed) {
          robot.rotateClaw(true);
          telemetry.addLine("rotateClaw true");
          sinceLastUsedGrabRotate.reset();
        } else if (control.rbump() == Button.Pressed) {
          robot.rotateClaw(false);
          telemetry.addLine("rotateClaw false");
          sinceLastUsedGrabRotate.reset();
        }
      }



      // Override the linear slide limit switches
      boolean slideOverride = (control.rbump() == Button.Pressed) && (control.lbump() == Button.Pressed);
      Direction slide = control.dpad();
      if (slide.isLeft()) {
        robot.setLinearSlideDirectionRyan(LinearSlideOperation.Extend, !slideOverride);
      } else if (slide.isRight()) {
        robot.setLinearSlideDirectionRyan(LinearSlideOperation.Retract, !slideOverride);
      } else {
        robot.setLinearSlideDirectionRyan(LinearSlideOperation.None, !slideOverride);
      }
      Direction dcontrols = driver.dpad();
      if (dcontrols.isUp()) {
        robot.blockFlipper(0.15);
      } else {
        robot.blockFlipper(0.8);
      }
      if (dcontrols.isDown()) {
        robot.bpGrabber(0);
      } else {
        robot.bpGrabber(1);
      }
      if (dcontrols.isLeft()) {
        robot.capstone(-1);
      } else if (dcontrols.isRight()) {
        robot.capstone(1);
      } else {
        robot.capstone(0);
      }
      // Lift control:
      Direction dir = control.dpad();
      if (dir.isUp()) {
        robot.liftUp();
      } else if (dir.isDown()) {
        robot.liftDown();
      } else {
        robot.liftStop();
      }
      if (driver.ltrigger() >  0.8 && driver.rtrigger() > 0.8 && driver.rbump().isPressed() && driver.lbump().isPressed()) {
        robot.initGyro();
      }

      telemetry.addData("Left trigger pos: ", driver.ltrigger());
      telemetry.addData("Right trigger pos: ", driver.rtrigger());
      // This is just steering
      manualCtrl.Steer();

      telemetry.addLine(String.format("Timing: %.1f, %.1f", timeSinceStart.seconds(), loopTime.seconds()));
      telemetry.update();
    }
  }
}
