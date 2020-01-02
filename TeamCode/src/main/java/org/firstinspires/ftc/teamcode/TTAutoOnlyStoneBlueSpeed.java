package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//DOCUMENTED FOR ENG NOTEBOOK

//AUTONOMOUS TO GRAB ONE SKYSTONE ON BLUE SIDE, PLACE IT ON THE BASEPLATE, AND PARK ON THE CENTER LINE.
//WORTH ?? POINTS zx

//Name as appears in Driver Station Autonomous menu
@Autonomous(name = "TTAutoOnlyStoneBlueSpeed", group = "TT")

public class TTAutoOnlyStoneBlueSpeed extends LinearOpMode {

  //states
  private enum AutoState {
    INITIALIZE,
    GOTOBLOCK1,
    GOTOBLOCK2,
    GOTOBLOCK3,
    GRABBLOCK,
    GOTOBASEPLATE,
    PLACEBLOCK,
    GOTOLINE,
    STOP
  }

  //declare stuff
  private AutoState currentState = AutoState.INITIALIZE;
  private ElapsedTime runtime = new ElapsedTime();
  private TTRobot robot;
  private ElapsedTime driveTime = new ElapsedTime();

  //starting function
  @Override
  public void runOpMode() {

    //robot
    robot = new TTRobot(this, hardwareMap, telemetry);

    //vision processing
    Truphoria tf = new Truphoria(hardwareMap, telemetry);
    robot.centerClaw();

    //scan
    while (!isStarted()) {
      tf.takeALook();
      telemetry.addData("tfdata ", tf.whichColumn());
      telemetry.addData("tfconf ", tf.confidence());
      telemetry.addData("isActive:", isStarted());
      telemetry.update();
    }
    runtime.reset();
    // run until the end of the match (driver presses STOP)

    //make final decision on skystone position
    int blockPos = tf.whichColumn();

    telemetry.update();

    //main loop
    while (opModeIsActive()) {
      telemetry.addData("Status", "Run Time: " + runtime.toString());
      switch (currentState) {
        case INITIALIZE:
          if (opModeIsActive()) {
            telemetry.addData("state", currentState.toString());
            runtime.reset();

            //choose how to grab skystone
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

          robot.setClawPosition(ClawPosition.Open);
          driveTime.reset();
          robot.setLinearSlideDirection(LinearSlideOperation.Extend, true);
          break;

        //funtions to drive to different stones depending on configuration
        case GOTOBLOCK1:
          telemetry.addData("state", currentState.toString());
          robot.fastRightDrive(85);
          currentState = AutoState.GRABBLOCK;
          break;
        case GOTOBLOCK2:
          telemetry.addData("state", currentState.toString());
          robot.fastRightDrive(58);
          currentState = AutoState.GRABBLOCK;
          break;
        case GOTOBLOCK3:
          telemetry.addData("state", currentState.toString());
          robot.fastRightDrive(40);
          robot.fastRightDrive(40);
          currentState = AutoState.GRABBLOCK;
          break;

        case GRABBLOCK:
          //grab the stone
          telemetry.addData("state", currentState.toString());
          while (driveTime.seconds() < 1.3) {
            sleep(10);
          }
          robot.setLinearSlideDirection(LinearSlideOperation.None, false);
          robot.fastRearDrive(82);
          robot.fastSyncTurn(0, 1);
          robot.setClawPosition(ClawPosition.Close);
          sleep(500);
          robot.lift.LiftBrickWait(0);
          currentState = AutoState.GOTOBASEPLATE;
          break;

        case GOTOBASEPLATE:
          //go to the baseplate
          robot.fastRearDrive(55);
          robot.fastRightDrive(75);
          robot.fastSyncTurn(0, 1);
          robot.turnAndDrive(90, 0.5, -90);
          robot.turnAndDrive(0, 0.5, -90);
          sleep(100);
          robot.fastSyncTurn(0, 1);
          robot.fastLeftDrive(40);
          robot.timeDrive(0.5, 0.7, 0);
          currentState = AutoState.PLACEBLOCK;
          break;

        case PLACEBLOCK:
          //place block and prepare to drive to line
          robot.setClawPosition(ClawPosition.Open);
          sleep(200);
          driveTime.reset();
          robot.setLinearSlideDirection(LinearSlideOperation.Retract, false);
          robot.fastRearDrive(85);
          while (driveTime.seconds() < 1.1) {
            sleep(10);
          }
          robot.setLinearSlideDirection(LinearSlideOperation.None, false);
          robot.fastSyncTurn(0, 1);
          currentState = AutoState.GOTOLINE;
          break;

        case GOTOLINE:
          //drive to center line
          robot.driveToLine(0.75, 90);
          currentState = AutoState.STOP;
          break;

        case STOP:
          //stop robot
          stop();
      }
    }
  }
}
