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

    robot.distRearDrivePID(20);
    /*
    robot.distLeftDrive(this,  -90, 60);
    runtime.reset();
    while (opModeIsActive() && runtime.seconds() < 2) {
      tf.takeALook();
      telemetry.addData("tfdata ", tf.whichColumn());
      telemetry.addData("tfconf ", tf.confidence());
      telemetry.update();
    }
    // run until the end of the match (driver presses STOP)
    int blockPos = tf.whichColumn();

    telemetry.update();
    while (opModeIsActive()) {
      telemetry.addData("Status", "Run Time: " + runtime.toString());
      switch (currentState) {
        case INITIALIZE:
          telemetry.addData("state", currentState.toString());
          runtime.reset();
          switch (blockPos) {
            case 0:
              currentState = AutoState.GOTOBLOCK1;
              break;
            case 1:
              currentState = AutoState.GOTOBLOCK2;
              break;
            case 2:
              currentState = AutoState.GOTOBLOCK3;
              break;
          }
          break;

        case GOTOBLOCK1:
          telemetry.addData("state", currentState.toString());
          robot.distLeftDrive(0.5, 90, 74);
          currentState = AutoState.GOFORWARD;
          break;
        case GOTOBLOCK2:
          telemetry.addData("state", currentState.toString());
          robot.distLeftDrive(0.5, 90, 60);
          currentState = AutoState.GOFORWARD;
          break;
        case GOTOBLOCK3:
          telemetry.addData("state", currentState.toString());
          robot.distLeftDrive(0.5, -90, 47);
          currentState = AutoState.GOFORWARD;
          break;
        case GOFORWARD:
          telemetry.addData("state", currentState.toString());
          robot.distRearDrive(0.5, 80);
          currentState = AutoState.STOP;
          break;
        case STOP:
          telemetry.addData("state", currentState.toString());

          stop();
          break;

        default:
          telemetry.addData("state", currentState.toString());

          stop();
          break;
      }
      telemetry.update();
    }
    */
  }
}
