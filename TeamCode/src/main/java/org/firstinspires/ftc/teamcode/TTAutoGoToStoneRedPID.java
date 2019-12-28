package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "TTAutoGoToStoneRedPID", group = "TT")
public class TTAutoGoToStoneRedPID extends LinearOpMode {

  // States
  private enum AutoState {
    INITIALIZE,
    GOTOBLOCK1,
    GOTOBLOCK2,
    GOTOBLOCK3,
    GOFORWARD,
    STOP
  }


  private AutoState currentState = AutoState.INITIALIZE;
  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime timer = new ElapsedTime();
  private ElapsedTime driveTime = new ElapsedTime();


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

      robot.driveTrain.setDriveVector(Math.abs(speed), (speed < 0) ? 0 : 180, robot.gyroHeading());

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
    double x = 0.0, y = 0.0, z = 0.0;

    /*
     * Initialize the standard drive system variables.
     * The init() method of the hardware class does most of the work here
     */
    TTRobot robot = new TTRobot(this, hardwareMap, telemetry);
    Truphoria tf = new Truphoria(hardwareMap, telemetry);

    // Put vuforia Here

    waitForStart();

    distRearDrivePID(robot,7);
  }
}
