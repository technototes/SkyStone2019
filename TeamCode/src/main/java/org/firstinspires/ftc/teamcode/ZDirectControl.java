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
    while (opModeIsActive()) {

      //telemetry.addData("rear distance", "%3.3f", robot.sensorRangeRear.getDistance(DistanceUnit.CM));
      //telemetry.addData("left distance", "%3.3f", robot.sensorRangeLeft.getDistance(DistanceUnit.CM));
      //telemetry.addData("right distance", "%3.3f", robot.sensorRangeRight.getDistance(DistanceUnit.CM));
      //telemetry.addData("front distance", "%3.3f", robot.sensorRangeFront.getDistance(DistanceUnit.CM));

      //telemetry.addData("gyroHeading:", "%3.3f", robot.gyroHeading());
      //telemetry.addData("gyroHeading2", "%3.3f", robot.gyroHeading2());

      // Lift stuff
      double ltr = control.ltrigger();
      double rtr = control.rtrigger();
      if (Math.abs(ltr) > ZRobot.STICKDEADZONE) {
        robot.liftUp();
        liftMoving = true;
      } else if (Math.abs(rtr) > ZRobot.STICKDEADZONE) {
        robot.liftDown();
        liftMoving = true;
      } else if (liftMoving) {
        robot.liftStop();
        liftMoving = false;
      }

      // This is just steering
      manualCtrl.Steer();
      telemetry.update();
    }
  }
}
