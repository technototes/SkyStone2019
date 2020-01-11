package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//Test
@Autonomous(name = "TTAutoOnlyStoneRedSpeed", group = "TT")
public class TTAutoOnlyStoneRedSpeed extends LinearOpMode {

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
          }
          robot.setClawPosition(ClawPosition.Open);
          driveTime.reset();
          robot.setLinearSlideDirection(LinearSlideOperation.Extend, true);
          break;

        //funtions to drive to different stones depending on configuration
        case GOTOBLOCK1:
          //robot.distRearRightDrive(1, 85, 90);
          robot.fastLeftDrive(87);
          robot.fastSyncTurn(0,1);
          robot.fastRearDrive(80);
          currentState = AutoState.GRABBLOCK;
          break;
        case GOTOBLOCK2:
          //robot.distRearRightDrive(1, 85, 75);
          //robot.fastRightDrive(80);
          robot.fastLeftDrive(73);
          robot.fastRearDrive(80);
          //robot.fastRightDrive(70);
          currentState = AutoState.GRABBLOCK;
          break;
        case GOTOBLOCK3:
          //robot.fastRightDrive(40);
          robot.fastLeftDrive(50);
          robot.fastSyncTurn(0,1);

          robot.fastRearDrive(80);
          //robot.distRearRightDrive(1, 85, 50);
          currentState = AutoState.GRABBLOCK;
          break;

        case GRABBLOCK:
          //grab the stone
          telemetry.addData("state", currentState.toString());
          while (driveTime.seconds() < 1) {
            sleep(10);
          }
          robot.setLinearSlideDirection(LinearSlideOperation.None, false);
          //robot.fastRearDrive(82);
          robot.fastSyncTurn(0, 1);
          robot.setClawPosition(ClawPosition.Close);
          sleep(500);
          robot.lift.LiftBrickWait(0);
          currentState = AutoState.GOTOBASEPLATE;
          break;

        case GOTOBASEPLATE:
          //robot.fastRearDrive(65);
          robot.fastRearDrive(55);
          //robot.fastRightDrive(80);
          robot.fastSyncTurn(-90, 2);
          robot.distLeftDrive(0.4, 0, 62);
          //robot.fastRearDrive(65);
          //robot.distRearRightDrive(1, 55, 75);
          //robot.turnAndDrive(80, 0.5, -90);
          robot.driveToLine(0.5, 90);
          robot.turnAndDrive(0, 0.5, 90);
          sleep(100);
          //robot.distRearRightDrive(1, 70, 100);
          robot.fastSyncTurn(0, 1);
          //robot.distRearLeftDrive(1, 90, 65);
          robot.fastRightDrive(5);
          //robot.lift.up();
          //robot.timeDrive(0.7, 0.2, 0);
          //robot.lift. stop();
          //robot.timeDrive(0.7, 0.3, 0);
          //robot.lift.LiftBrickWait(1);
          robot.lift.LiftBrickWait(1);
          robot.fastRearDrive(90);
          //robot.timeDrive(0.5, 0.2, 0);
          currentState = AutoState.PLACEBLOCK;
          break;
        case PLACEBLOCK:

          robot.setClawPosition(ClawPosition.Open);
          sleep(500);
          robot.setLinearSlideDirection(LinearSlideOperation.Retract, false);
          sleep(1500);
          robot.setLinearSlideDirection(LinearSlideOperation.None, false);
          driveTime.reset();
          robot.fastSyncTurn(0, 1);
          //robot.driveToLine(0.75, 90);
          //robot.stop();
          currentState = AutoState.MOVEBASEPLATE;
          break;
        case MOVEBASEPLATE:
          robot.fastRearDrive(80);
          robot.fastRightDrive(30);
          //robot.distRearLeftDrive(1, 80, 30);
          robot.fastSyncTurn(-170, 4);
          robot.timeDrive(0.5, 0.5, 0);
          robot.blockFlipper(FlipperPosition.Down);
          //robot.timeDrive(0.3, 0.1, 180);
          //robot.timeDrive(0.6, 0.1, 180);
          sleep(200);
          //robot.fastSyncTurn(135, 2);
          //robot.timeDrive(0.75, 0.5, 135);
          //robot.fastSyncTurn(90, 2);
          robot.fastSyncTurn(-170, 1);
          robot.timeDrive(0.5, 0.5, 180);
          robot.fastSyncTurn(-170, 1);
          robot.turnAndDrive(-110, 0.4, -150);
          robot.blockFlipper(FlipperPosition.Up);
          robot.fastSyncTurn(90, 1);
          //robot.fastLeftDrive(70);

          robot.timeDrive(0.5, 0.5, 0);
          //robot.distRightDrive(0.5, 0, 40);
          //robot.timeDrive(1, 2, -90);
          robot.lift.AcquireBrickWait();
          robot.timeDrive(0.75, 1, 90);
          robot.timeDrive(0.5, 0.5, -90);
          robot.fastSyncTurn(0, 2);
          robot.fastRearDrive(60);
          robot.setClawPosition(ClawPosition.Close);
          currentState = AutoState.GOTOLINE;
          break;
        case GOTOLINE:
          robot.driveToLine(0.75, -90);
          robot.driveToLine(0.25, 90);
          currentState = AutoState.STOP;
          break;
        case STOP:
          stop();
      }
    }
  }
}
