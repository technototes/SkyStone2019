package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Z-DCTest")
public class ZDirectControl extends LinearOpMode {
  private static double FINEDRIVESPEED = 0.2;
  private ZRobot robot;
  private Controller control;
  private Controller driver;
  private ZXDriveManualControl manualCtrl;

  @Override
  public void runOpMode() {
    // If you want telemetry, include a name as a string
    // If you don't want telemetry, pass a null:
    driver = new Controller(gamepad1, telemetry, "driver");
    control = new Controller(gamepad2, telemetry, "controller");
    robot = new ZRobot(this, hardwareMap, telemetry);
    manualCtrl = new ZXDriveManualControl(robot, driver, control, telemetry);

    waitForStart();
    robot.rotateClaw(1);
    while (opModeIsActive()) {

      telemetry.addData("rear distance", "%3.3f", robot.rearDistance());
      telemetry.addData("gyroHeading:", "%3.3f", robot.gyroHeading());
      telemetry.addData("gyroHeading2", "%3.3f", robot.gyroHeading2());
      Direction clst = control.lstick();
      if (clst.isDown()) {
        robot.distRearDrive(30);
      }
      else if (clst.isLeft()) {
        robot.distLeftDrive(100);
      } else if (clst.isRight()) {
        robot.distRightDrive(30);
      }

      // This is just steering
      //manualCtrl.Steer();
      telemetry.update();
    }
  }
}
