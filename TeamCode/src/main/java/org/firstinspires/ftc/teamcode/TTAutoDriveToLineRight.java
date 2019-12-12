package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
@Disabled

@Autonomous(name = "TTAutoDriveToLineRight", group = "TT")
public class TTAutoDriveToLineRight extends LinearOpMode {

  // States
  private enum AutoState {
    INITIALIZE,
    GO_TO_LINE,
    STOP
  }


  private AutoState currentState = AutoState.INITIALIZE;
  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime timer = new ElapsedTime();
  private TTRobot robot;

  @Override
  public void runOpMode() {
    double x = 0.0, y = 0.0, z = 0.0;

    /*
     * Initialize the standard drive system variables.
     * The init() method of the hardware class does most of the work here
     */
    robot = new TTRobot(this, hardwareMap, telemetry);

    // Put vuforia Here

    waitForStart();
    /*
    if (tfod != null) {
        tfod.deactivate();
    }
    */
    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {
      telemetry.addData("Status", "Run Time: " + runtime.toString());

      switch (currentState) {
          case INITIALIZE:
            if (opModeIsActive()) {
            telemetry.addData("state", currentState.toString());
            runtime.reset();
          /*
          if (skystonepos.equals(SkyStonePos.UNKNOWN) && tfod != null) {
              tfod.activate();
          }
          */
        }
          currentState = AutoState.GO_TO_LINE;

          break;


        case GO_TO_LINE:
          if (opModeIsActive()) {
            telemetry.addData("state", currentState.toString());
            robot.driveToLine(0.5, 90);
            robot.driveToLine(0.2, 270);
            currentState = AutoState.STOP;
            // distToLine(x, y, z);
          }
          break;
        case STOP:
          if (opModeIsActive()) {
            telemetry.addData("state", currentState.toString());

            stop();
          }
          break;

        default:
          telemetry.addData("state", currentState.toString());

          stop();
          break;
      }
      telemetry.update();
    }
  }
}
