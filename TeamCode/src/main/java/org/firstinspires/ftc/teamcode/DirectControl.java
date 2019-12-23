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

    int curBrickHeight = -1;

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
      Direction ctrlDpad = control.dpad();
      if (ctrlDpad.isLeft()) {
        robot.setLinearSlideDirectionRyan(LinearSlideOperation.Extend, !slideOverride);
      } else if (ctrlDpad.isRight()) {
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

      if ((control.rtrigger() > robot.TRIGGERTHRESHOLD) && (control.ltrigger() > robot.TRIGGERTHRESHOLD)) {
        if (control.buttonX().isPressed()) {
          robot.lift.overrideDown();
        } else {
          robot.lift.stop();
          robot.lift.ResetZero();
        }
      } else {
        // More automated control of the lift:
        // Y for 'up a brick'
        // X for 'down a brick'
        // A for 'position current brick to place'
        // B for 'grab a brick'
        if (control.buttonA().isPressed()) {
          robot.lift.SetBrickWait();
        } else if (control.buttonY().isPressed()) {
          robot.lift.LiftBrickWait(++curBrickHeight);
        } else if (control.buttonX().isPressed() && curBrickHeight > 0) {
          robot.lift.LiftBrickWait(--curBrickHeight);
        } else if (control.buttonB().isPressed()) {
          robot.lift.AcquireBrickWait();
          curBrickHeight = -1;
        }
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
