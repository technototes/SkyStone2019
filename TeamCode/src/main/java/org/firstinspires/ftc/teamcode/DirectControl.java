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

  // This is the middle 'dead zone' of the analog sticks
  private static final double STICKDEADZONE = 0.05;
  // Triggers must be pushed at least this far
  private static final double TRIGGERTHRESHOLD = 0.25;

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
    ElapsedTime timeSinceStart = new ElapsedTime();
    ElapsedTime loopTime = new ElapsedTime();

    final double grabRotateDebounceSecs = 0.25;
    ElapsedTime grabRotateTime = new ElapsedTime();

    final double brickCommandDebounceSecs = 0.25;
    ElapsedTime setBrickTime = new ElapsedTime();
    ElapsedTime brickUpTime = new ElapsedTime();
    ElapsedTime brickDownTime = new ElapsedTime();
    ElapsedTime acquireBrickTime = new ElapsedTime();
    int curBrickHeight = -1;
    boolean liftOverrideDownEnabled = false;

    while (opModeIsActive()) {
      loopTime.reset();

      // Handle Grabber clutch
      if (control.rtrigger() > TRIGGERTHRESHOLD) {
        robot.setClawPosition(ClawPosition.Open); // Open
      } else if (control.ltrigger() > TRIGGERTHRESHOLD) {
        robot.setClawPosition(ClawPosition.Close); // Closed
      }
      // Grabber rotation
      if (grabRotateTime.seconds() > grabRotateDebounceSecs) {
        if (control.lbump() == Button.Pressed) {
          robot.rotateClaw(true);
          telemetry.addLine("rotateClaw true");
          grabRotateTime.reset();
        } else if (control.rbump() == Button.Pressed) {
          robot.rotateClaw(false);
          telemetry.addLine("rotateClaw false");
          grabRotateTime.reset();
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
        robot.blockFlipper(FlipperPosition.Down);
      } else {
        robot.blockFlipper(FlipperPosition.Up);
      }

      if (dcontrols.isLeft()) {
        robot.capstone(-1);
      } else if (dcontrols.isRight()) {
        robot.capstone(1);
      } else {
        robot.capstone(0);
      }

      if ((control.ltrigger() > 0.8) && (control.rtrigger() > 0.8) &&
           control.rbump().isPressed() && control.lbump().isPressed()) {
        if (control.buttonX().isPressed()) {
          robot.lift.overrideDown();
          liftOverrideDownEnabled = true;
        } else {
          robot.lift.stop();
          robot.lift.ResetZero();
          liftOverrideDownEnabled = false;
        }
      } else {
        // Released all the buttons simultaneously, so need to stop & reset the lift now
        if (liftOverrideDownEnabled) {
          robot.lift.stop();
          robot.lift.ResetZero();
          liftOverrideDownEnabled = false;
        }

        // More automated control of the lift:
        // Y for 'up a brick'
        // X for 'down a brick'
        // A for 'position current brick to place'
        // B for 'grab a brick'
        if (control.buttonA().isPressed()) {
          if (setBrickTime.seconds() > brickCommandDebounceSecs) {
            robot.lift.SetBrickAsync();
            setBrickTime.reset();
          }
        } else if (control.buttonY().isPressed() && (curBrickHeight < LiftControl.MAX_BRICK_HEIGHT)) {
          if (brickUpTime.seconds() > brickCommandDebounceSecs) {
            robot.lift.LiftBrickAsync(++curBrickHeight);
            brickUpTime.reset();
          }
        } else if (control.buttonX().isPressed() && (curBrickHeight > 0)) {
          if (brickDownTime.seconds() > brickCommandDebounceSecs) {
            robot.lift.LiftBrickAsync(--curBrickHeight);
            brickDownTime.reset();
          }
        } else if (control.buttonB().isPressed()) {
          if (acquireBrickTime.seconds() > brickCommandDebounceSecs) {
            robot.lift.AcquireBrickAsync();
            curBrickHeight = -1;
            acquireBrickTime.reset();
          }
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
