package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Z-DCTest")
public class ZDirectControl extends LinearOpMode {
  private static double FINEDRIVESPEED = 0.2;
  private ZRobot robot;
  private Controller control;
  private Controller driver;
  private XDriveManualControl manualCtrl;

  @Override
  public void runOpMode() {
    // If you want telemetry, include a name as a string
    // If you don't want telemetry, pass a null:
    driver = new Controller(gamepad1, telemetry, "driver");
    control = new Controller(gamepad2, telemetry, "controller");
    robot = new ZRobot(this, hardwareMap, telemetry);
    manualCtrl = new XDriveManualControl(robot, driver, control, telemetry);

    waitForStart();
    boolean liftMoving = false;
    int curBrickHeight = -1;
    while (opModeIsActive()) {

      // Manual control of the lift from the control dpad
      Direction dir = control.dpad();
      if (dir.isUp()) {
        robot.lift.up();
        liftMoving = true;
      } else if (dir.isDown()) {
        robot.lift.down();
        liftMoving = true;
      } else if (liftMoving) {
        liftMoving = false;
        robot.lift.stop();
      }
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

      // This is just steering
      manualCtrl.Steer();
      telemetry.update();
    }
  }
}
