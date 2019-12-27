package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//Test
@Autonomous(name = "TTAutoStoneMovedNotWallRedSpeed", group = "TT")
public class TTAutoStoneMovedNotWallRedSpeed extends LinearOpMode {

  // States
  private enum AutoState {
    //times in sec
    INITIALIZE,         //3
    GOTOBLOCK1,         //3
    GOTOBLOCK2,
    GOTOBLOCK3,
    GRABBLOCK,          //1
    GOTOMOVEDBASEPLATE, //2
    PLACEBLOCK,         //1
    GOTOBLOCK4,         //3
    GOTOBLOCK5,
    GOTOBLOCK6,
    GRABBLOCK2,         //1
    GOTOMOVEDBASEPLATE2,//2
    PLACEBLOCK2,        //2
    GOTOBLOCK12,        //2
    GOTOBLOCK22,
    GRABBLOCK3,         //1
    GOTOMOVEDBASEPLATE3,//2
    PLACEBLOCK3,        //1
    GOTOLINE,           //2
    STOP

  }


  private AutoState currentState = AutoState.INITIALIZE;
  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime timer = new ElapsedTime();
  private TTRobot robot;
  private ElapsedTime driveTime = new ElapsedTime();
  private ElapsedTime runTime = new ElapsedTime();


  @Override
  public void runOpMode() {
    double x = 0.0, y = 0.0, z = 0.0;

    /*
     * Initialize the standard drive system variables.
     * The init() method of the hardware class does most of the work here
     */
    robot = new TTRobot(this, hardwareMap, telemetry);


    // Put vuforia Here

    Truphoria tf = new Truphoria(hardwareMap, telemetry);
    while (!isStarted()) {
      tf.takeALook();
      telemetry.addData("tfdata ", tf.whichColumn());
      telemetry.addData("tfconf ", tf.confidence());
      telemetry.addData("isActive:", isStarted());
      telemetry.update();
    }

    //robot.distRearDrive(1, 8);
    //robot.distRightDrive(0.5, 90, 40);
    //robot.distRightDrive(0.3, -90, 47);
    runtime.reset();

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
          if (opModeIsActive()) {
            telemetry.addData("state", currentState.toString());
            runtime.reset();
            /*
            if (skystonepos.equals(SkyStonePos.UNKNOWN) && tfod != null) {
                tfod.activate();
            }
            */
            switch (blockPos) {
              case 2:
                currentState = AutoState.GOTOBLOCK3;
                break;
              case 1:
                currentState = AutoState.GOTOBLOCK2;
                break;
              case 0:
                currentState = AutoState.GOTOBLOCK1;
                break;
            }
          }
          robot.syncTurn(0, 2);
          driveTime.reset();
          robot.centerClaw();
          robot.setClawPosition(ClawPosition.Open);
          while (driveTime.seconds() < 1.3) {
            robot.setLinearSlideDirection(LinearSlideOperation.Extend, false);
          }
          robot.setLinearSlideDirection(LinearSlideOperation.None, false);
          break;

        case GOTOBLOCK1:
          telemetry.addData("state", currentState.toString());
          robot.distRearLeftDrive(0.5, 80, 90);
          currentState = AutoState.GRABBLOCK;
          break;
        case GOTOBLOCK2:
          telemetry.addData("state", currentState.toString());
          robot.distRearLeftDrive(0.5, 90, 70);
          currentState = AutoState.GRABBLOCK;
          break;
        case GOTOBLOCK3:
          telemetry.addData("state", currentState.toString());
          robot.fastRearDrive(80);
          robot.fastSyncTurn(0, 2);
          currentState = AutoState.GRABBLOCK;
          break;
        case GRABBLOCK:
          telemetry.addData("state", currentState.toString());
          //stop();
          robot.setClawPosition(ClawPosition.Close);
          sleep(500);
          driveTime.reset();
          while (driveTime.seconds() < 0.2) {
            robot.lift.up();
          }
          robot.lift.stop();
          currentState = AutoState.GOTOMOVEDBASEPLATE;
          break;
        case GOTOMOVEDBASEPLATE:
          robot.fastRearDrive(60);
          robot.fastSyncTurn(90, 2);

          //TODO add distFrontRightDrive
          //robot.distRearRightDrive(1, 90, 40);
          currentState = AutoState.PLACEBLOCK;
          break;
        case PLACEBLOCK:
          robot.setClawPosition(ClawPosition.Open);
          switch (blockPos) {
            case 0:
              currentState = AutoState.GOTOBLOCK4;
              break;
            case 1:
              currentState = AutoState.GOTOBLOCK5;
              break;
            case 2:
              currentState = AutoState.GOTOBLOCK6;
              break;
          }
          robot.timeDrive(0.5, 0.5, -90);
          robot.fastSyncTurn(0, 2);
          //lower lift
          driveTime.reset();
          while (driveTime.seconds() < 0.2) {
            robot.lift.down();
          }
          robot.lift.stop();
          break;
        case GOTOBLOCK4:
          robot.distRearLeftDrive(1, 80, 30);
          robot.fastSyncTurn(0, 1);
          currentState = AutoState.GRABBLOCK2;
          break;
        case GOTOBLOCK5:
          robot.distRearLeftDrive(1, 80, 10);
          robot.fastSyncTurn(0, 1);
          currentState = AutoState.GRABBLOCK2;
          break;
        case GOTOBLOCK6:
          //how are we going to grab this one?
          robot.distRearLeftDrive(1, 80, 0);
          robot.fastSyncTurn(0, 1);
          currentState = AutoState.GRABBLOCK2;
          break;
        case GRABBLOCK2:
          robot.setClawPosition(ClawPosition.Close);
          sleep(500);
          driveTime.reset();
          while (driveTime.seconds() < 0.2) {
            robot.lift.up();
          }
          robot.lift.stop();
          currentState = AutoState.GOTOMOVEDBASEPLATE;
          break;
        case GOTOMOVEDBASEPLATE2:
          robot.fastRearDrive(60);
          robot.fastSyncTurn(90, 2);

          //TODO make this distfront to a dist of 80 maybe
          robot.timeDrive(0.75, 3, 90);
          currentState = AutoState.PLACEBLOCK2;
          break;
        case PLACEBLOCK2:
          robot.setClawPosition(ClawPosition.Open);
          switch (blockPos){
            case 0:
              currentState = AutoState.GOTOBLOCK22;
              break;
            case 1:
              currentState = AutoState.GOTOBLOCK12;
              break;
            case 2:
              currentState = AutoState.GOTOBLOCK12;
              break;
          }
          robot.timeDrive(0.5, 0.5, -90);
          robot.fastSyncTurn(0, 2);
          //lower lift
          driveTime.reset();
          while (driveTime.seconds() < 0.2) {
            robot.lift.down();
          }
          robot.lift.stop();

          break;
        case GOTOBLOCK12:
          robot.distRearLeftDrive(1, 80, 90);
          currentState = AutoState.GRABBLOCK3;
          break;
        case GOTOBLOCK22:
          robot.distRearLeftDrive(1, 80, 70);
          currentState = AutoState.GRABBLOCK3;
          break;
        case GRABBLOCK3:
          robot.setClawPosition(ClawPosition.Close);
          sleep(500);
          driveTime.reset();
          while (driveTime.seconds() < 0.2) {
            robot.lift.up();
          }
          robot.lift.stop();
          currentState = AutoState.GOTOMOVEDBASEPLATE3;
          break;
        case GOTOMOVEDBASEPLATE3:

          robot.fastRearDrive(60);
          robot.fastSyncTurn(90, 2);

          //TODO make this distfront to a dist of 80 maybe
          robot.timeDrive(0.75, 3, 90);
          currentState = AutoState.PLACEBLOCK3;
          break;
        case PLACEBLOCK3:
          robot.setClawPosition(ClawPosition.Open);
          currentState = AutoState.GOTOLINE;
          break;
        case GOTOLINE:
          robot.driveToLine(0.5, -90);
          currentState = AutoState.STOP;
          break;
        case STOP:
          stop();
      }
    }
  }
}
