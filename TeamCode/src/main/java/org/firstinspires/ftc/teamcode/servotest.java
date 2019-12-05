package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@TeleOp(name = "servo test")
public class servotest extends LinearOpMode {
  private static double FINEDRIVESPEED = 0.2;
  private TTRobot robot;
  private Controller control;
  private Controller driver;

  @Override
  public void runOpMode() {
    robot = new TTRobot(this, hardwareMap, telemetry);
    // If you want telemetry, include a name as a string
    // If you don't want telemetry, pass a null:
    driver = new Controller(gamepad1, telemetry, "driver");
    control = new Controller(gamepad2, telemetry, null);
    telemetry.addLine("Hello!");
    telemetry.update();
    waitForStart();
    while (opModeIsActive()) {

      double rt = control.rtrigger ();
      telemetry.addData("Servo", "Position (%.2f)", rt);

      robot.setServoPosition(rt);
      if(control.dpad().isLeft()){
        robot.setServoDirection(Servo.Direction.REVERSE);
      }
      if(control.dpad().isRight()) {
        robot.setServoDirection(Servo.Direction.FORWARD);
      }

      telemetry.update();

    }
  }
}
