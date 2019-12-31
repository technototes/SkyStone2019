package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "JamesAutoTest", group = "TT")
public class JamesAutoTest extends LinearOpMode {
  private TTRobot robot = null;

  // This will move the robot so it's "dist" away based on the rear sensor
  void distRearDrivePID(TTRobot robot, double targetDist) {
    final double epsilon = 2; // Stop if "dist" is within this many units of the target
    final double speedIncrementValue = 0.1; // Amount to change speed per speedIncrementTime
    final double speedIncrementTime = 0.2; // Time (in sec) between changes in speed

    ElapsedTime tm = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    double curDistance = robot.rearDistance();
    double distFromGoal = Math.abs(curDistance - targetDist);
    double speed = 0;
    double timeOfLastSpeedChange = -1;
    while (opModeIsActive() && (distFromGoal > epsilon) && tm.seconds() < 6) {
      final double dir = (targetDist < curDistance) ? 1 : -1;
      final double desiredSpeed = dir * ((distFromGoal > 40) ? 1.0 : (distFromGoal / 40));

      double now = tm.seconds();
      if (Math.abs(desiredSpeed) < Math.abs(speed)) {
        // Slow down quickly
        speed = desiredSpeed;
        timeOfLastSpeedChange = now;
      } else if (Math.abs(desiredSpeed - speed) > speedIncrementValue) {
        // Speed up slowly
        if (now - timeOfLastSpeedChange > speedIncrementTime) {
          speed += speedIncrementValue * dir;
          timeOfLastSpeedChange = now;
        }
      } else {
        speed = desiredSpeed;
      }

      robot.vectorDrive(Math.abs(speed), (speed < 0) ? 0 : 180);

      telemetry.addData("Current Distance", curDistance);
      telemetry.addData("Current Speed", speed);
      telemetry.update();

      curDistance = robot.rearDistance();
      distFromGoal = Math.abs(curDistance - targetDist);
    }

    robot.stop();
  }

  @Override
  public void runOpMode() {
    /*
     * Initialize the standard drive system variables.
     * The init() method of the hardware class does most of the work here
     */
    robot = new TTRobot(this, hardwareMap, telemetry);
    waitForStart();

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {
      telemetry.addLine("Dist (R, F, L, R): " + robot.rearDistance() + ", " + robot.frontDistance() + ", " + robot.leftDistance() + ", " + robot.rightDistance());
      telemetry.update();
    }
  }
}
