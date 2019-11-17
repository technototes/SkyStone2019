package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Direct Control test")
public class a extends LinearOpMode {
  private static double FINEDRIVESPEED = 0.2;
  private TTRobot robot;
  private Controller control;
  private Controller driver;

  @Override
  public void runOpMode() {
    robot = new TTRobot();
    // If you want telemetry, include a name as a string
    // If you don't want telemetry, pass a null:
    driver = new Controller(gamepad1, telemetry, "driver");
    control = new Controller(gamepad2, telemetry, null);
    robot.init(hardwareMap, telemetry);
    robot.calibrate(); // OOPS
    telemetry.addLine("Hello!");
    telemetry.update();
    waitForStart();
    while (opModeIsActive()) {
      Direction test = control.dpad();
      telemetry.addLine("IsUP: " + test.isUp() + " IsDown: " + test.isDown() + " IsLeft " + test.isLeft() + " IsRight: " + test.isRight());
      telemetry.addLine("IsOnlyUP: " + test.isOnlyUp() + " IsOnlyDown: " + test.isOnlyDown() + " IsOnlyLeft " + test.isOnlyLeft() + " IsOnlyRight: " + test.isOnlyRight());
      telemetry.update();

    }
  }
}
