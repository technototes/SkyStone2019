package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//Test
@Autonomous(name = "TTAutoStoneMovedWallBlueSpeed", group = "TT")
public class TTAutoStoneMovedWallBlueSpeed extends LinearOpMode {

  // States
  private enum AutoState {
    //times in sec
    INITIALIZE,         //3
    GOTOBLOCK1,         //3
    GOTOBLOCK2,
    GOTOBLOCK3,
    GRABBLOCK,          //1
    GOTOBASEPLATE,      //3
    PLACEBLOCK,         //1
    MOVEBASEPLATE,      //4
    GOTOBLOCK4,         //5
    GOTOBLOCK5,
    GOTOBLOCK6,
    GRABBLOCK2,         //1
    GOTOMOVEDBASEPLATE, //3
    PLACEBLOCK2,        //1
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
                currentState = AutoState.GOTOBLOCK1;
                break;
              case 1:
                currentState = AutoState.GOTOBLOCK2;
                break;
              case 0:
                currentState = AutoState.GOTOBLOCK3;
                break;
            }
          }
          //robot.syncTurn(0, 2);
          robot.centerClaw();
          robot.setClawPosition(ClawPosition.Open);
          driveTime.reset();
          //robot.turnAndDrive(90, 0.4, -90);
          //sleep(100);
          //robot.fastSyncTurn(90, 1);
          //currentState = AutoState.STOP;
          robot.setLinearSlideDirection(LinearSlideOperation.Extend, true);
          //sleep(1300);

          break;

        case GOTOBLOCK1:
          telemetry.addData("state", currentState.toString());
          robot.distRearRightDrive(1, 85, 90);
          robot.fastSyncTurn(0, 2);
          currentState = AutoState.GRABBLOCK;
          break;
        case GOTOBLOCK2:
          telemetry.addData("state", currentState.toString());
          //robot.distRearRightDrive(1, 90, 70);
          robot.fastRearDrive(90);
          robot.fastSyncTurn(0, 2);
          currentState = AutoState.GRABBLOCK;
          break;
        case GOTOBLOCK3:
          telemetry.addData("state", currentState.toString());
          robot.distRearRightDrive(1, 85, 50);
          robot.fastSyncTurn(0, 2);
          currentState = AutoState.GRABBLOCK;
          break;
        case GRABBLOCK:
          telemetry.addData("state", currentState.toString());
          //stop();
          while(driveTime.seconds() < 1.1){
            sleep(10);
          }
          robot.setLinearSlideDirection(LinearSlideOperation.None, false);
          robot.setClawPosition(ClawPosition.Close);
          sleep(500);
          robot.lift.LiftBrickWait(0);
          currentState = AutoState.GOTOBASEPLATE;
          break;
        case GOTOBASEPLATE:
          robot.fastRearDrive(70);
          robot.turnAndDrive(90, 0.5, -90);
          robot.turnAndDrive(0, 0.5, -90);
          sleep(100);
          //robot.distRearRightDrive(1, 70, 100);
          robot.fastSyncTurn(0, 1);
          //robot.distRearLeftDrive(1, 90, 40);
          //robot.fastLeftDrive(40);
          robot.fastRearDrive(100);
          currentState = AutoState.PLACEBLOCK;
          break;
        case PLACEBLOCK:
          robot.setClawPosition(ClawPosition.Open);
          currentState = AutoState.MOVEBASEPLATE;
          break;
        case MOVEBASEPLATE:
          //TODO test this
          robot.fastRearDrive(70);
          robot.fastSyncTurn(180, 2);
          //robot.timeDrive(0.5, 0.5, 0);
          robot.blockFlipper(FlipperPosition.Up);
          robot.fastSyncTurn(135, 2);
          robot.timeDrive(0.75, 1, 135);
          robot.fastSyncTurn(90, 2);
          robot.timeDrive(0.75, 1, -90);
          //TODO make this dist front drive to a distance of 10
          //robot.timeDrive(0.75, 2, 180);
          robot.blockFlipper(FlipperPosition.Down);
          robot.timeDrive(1, 0.5, 90);
          robot.fastSyncTurn(0, 2);

          //choice for secondary skystone
          switch (blockPos) {
            case 0:
              currentState = AutoState.GOTOBLOCK6;
              break;
            case 1:
              currentState = AutoState.GOTOBLOCK5;
              break;
            case 2:
              currentState = AutoState.GOTOBLOCK4;
              break;
          }
          //lower lift
          robot.lift.AcquireBrickWait();
          break;
        case GOTOBLOCK4:
          robot.distRearRightDrive(1, 85, 30);
          robot.fastSyncTurn(0, 1);
          currentState = AutoState.GRABBLOCK2;
          break;
        case GOTOBLOCK5:
          robot.distRearRightDrive(1, 85, 10);
          robot.fastSyncTurn(0, 1);
          currentState = AutoState.GRABBLOCK2;
          break;
        case GOTOBLOCK6:
          //how are we going to grab this one?
          robot.distRearRightDrive(1, 85, 0);
          robot.fastSyncTurn(0, 1);
          currentState = AutoState.GRABBLOCK2;
          break;
        case GRABBLOCK2:
          robot.setClawPosition(ClawPosition.Close);
          sleep(500);
          robot.lift.LiftBrickWait(0);
          currentState = AutoState.GOTOMOVEDBASEPLATE;
          break;
        case GOTOMOVEDBASEPLATE:
          robot.fastRearDrive(60);
          robot.fastSyncTurn(-90, 1);

          //TODO make this distfront to a dist of 80 maybe
          robot.driveToLine(0.75, -90);
          robot.timeDrive(0.75, 2, -90);
          currentState = AutoState.PLACEBLOCK2;
          break;
        case PLACEBLOCK2:
          robot.setClawPosition(ClawPosition.Open);
          currentState = AutoState.GOTOLINE;
          break;
        case GOTOLINE:
          robot.driveToLine(0.75, 90);
          currentState = AutoState.STOP;
          break;
        case STOP:
          stop();
      }
    }
  }
}
