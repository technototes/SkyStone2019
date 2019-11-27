package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Direct Drive Refactor test")
public class a extends LinearOpMode {
  private static double FINEDRIVESPEED = 0.2;
  private TTRobot robot;
  private XDriveManualControl manualCtrl;
  private Controller control;
  private Controller driver;

  @Override
  public void runOpMode() {
    // If you want telemetry, include a name as a string
    // If you don't want telemetry, pass a null:
    driver = new Controller(gamepad1, telemetry, "driver");
    control = new Controller(gamepad2, telemetry, null);
    robot = new TTRobot(hardwareMap, telemetry);
    manualCtrl = new XDriveManualControl(robot, driver, control, telemetry);

    telemetry.addLine("Hello!");
    telemetry.update();

    waitForStart();
    while (opModeIsActive()) {
      Direction test = control.dpad();
      telemetry.addLine("IsUP: " + test.isUp() + " IsDown: " + test.isDown() + " IsLeft " + test.isLeft() + " IsRight: " + test.isRight());
      telemetry.addLine("IsOnlyUP: " + test.isOnlyUp() + " IsOnlyDown: " + test.isOnlyDown() + " IsOnlyLeft " + test.isOnlyLeft() + " IsOnlyRight: " + test.isOnlyRight());

      // Allow the robot to move around
      manualCtrl.Steer();

      if (driver.buttonB().isPressed()) {
        robot.distRearDrive(15);
      }
      if (control.buttonA().isPressed()) {
        robot.stop();
        robot.toAngleSync(180);
      } else if (control.buttonB().isPressed()) {
        robot.stop();
        robot.toAngleSync(90);
      } else if (control.buttonY().isPressed()) {
        robot.stop();
        robot.toAngleSync(0);
      } else if (control.buttonX().isPressed()) {
        robot.stop();
        robot.toAngleSync(-90);
      }
      telemetry.update();
    }
  }
}
