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
    robot = new TTRobot(hardwareMap, telemetry);
    // If you want telemetry, include a name as a string
    // If you don't want telemetry, pass a null:
    driver = new Controller(gamepad1, telemetry, "driver");
    control = new Controller(gamepad2, telemetry, null);

    telemetry.addLine("Hello!");
    telemetry.update();
    manualCtrl = new XDriveManualControl(robot, driver, control, telemetry);

    waitForStart();
    while (opModeIsActive()) {
      Direction test = control.dpad();
      telemetry.addLine("IsUP: " + test.isUp() + " IsDown: " + test.isDown() + " IsLeft " + test.isLeft() + " IsRight: " + test.isRight());
      telemetry.addLine("IsOnlyUP: " + test.isOnlyUp() + " IsOnlyDown: " + test.isOnlyDown() + " IsOnlyLeft " + test.isOnlyLeft() + " IsOnlyRight: " + test.isOnlyRight());

      // Allow the robot to move around
      manualCtrl.Steer();

      if (control.buttonB().isPressed()) {
        robot.distRearDrive(0.5, 15, 0);
      }
      telemetry.update();
    }
  }
}
