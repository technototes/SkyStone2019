package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "TTAutoGoToStoneBlue", group = "TT")
public class TTAutoGoToStoneBlue extends LinearOpMode {

  // States
  private enum AutoState {
    INITIALIZE,
    GOTOBLOCK,

    GOFORWARD,
    STOP
  }


  private AutoState currentState = AutoState.INITIALIZE;
  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime timer = new ElapsedTime();
  private TTRobot robot;
  private ElapsedTime driveTime = new ElapsedTime();


  @Override
  public void runOpMode() {
    double x = 0.0, y = 0.0, z = 0.0;

    /*
     * Initialize the standard drive system variables.
     * The init() method of the hardware class does most of the work here
     */
    robot = new TTRobot(hardwareMap, telemetry);



    // Put vuforia Here

    waitForStart();

    Truphoria tf = new Truphoria(hardwareMap, telemetry);
    robot.distRearDrive(0.5, 9);
    robot.distRightDrive(0.5, 90, 60);
    runtime.reset();
    while(runtime.seconds() < 2){
      tf.takeALook();
      telemetry.addData("tfdata ", tf.whichColumn());
      telemetry.addData("tfconf ", tf.confidence());
      telemetry.update();
    }
    /*
    if (tfod != null) {
        tfod.deactivate();
    }
    */
    // run until the end of the match (driver presses STOP)
    int blockPos = tf.whichColumn();

    telemetry.update();
    while (opModeIsActive()) {
      telemetry.addData("Status", "Run Time: " + runtime.toString());
      switch (currentState) {
        case INITIALIZE:
          telemetry.addData("state", currentState.toString());
          runtime.reset();
          /*
          if (skystonepos.equals(SkyStonePos.UNKNOWN) && tfod != null) {
              tfod.activate();
          }
          */

        case GOTOBLOCK:
          telemetry.addData("state", currentState.toString());

          if (blockPos == 0) {
            robot.timeDrive(0.25, 0.3, 90);
            robot.timeDrive(0.25, 0.5, 0);
          }
          if (blockPos == 1) {
            robot.timeDrive(0.25, 0.5, 0);
          }
          if (blockPos ==  2) {
            robot.timeDrive(0.25, 0.3, 270);
            robot.timeDrive(0.25, 0.5, 0);
          }

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
  }
}
