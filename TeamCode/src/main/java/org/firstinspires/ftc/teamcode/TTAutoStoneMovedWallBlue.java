package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//Test
@Disabled
@Autonomous(name = "TTAutoStoneMovedWallBlue", group = "TT")
public class TTAutoStoneMovedWallBlue extends LinearOpMode {

  // States
  private enum AutoState {
    INITIALIZE,
    GOTOBLOCK1,
    GOTOBLOCK2,
    GOTOBLOCK3,
    GOFORWARD,
    EXTENDSLIDE,
    DROPLIFT,
    GRABBLOCK,
    GO_TO_BASE_PLATE,
    PLACE_STONE,
    GO_TO_LINE,
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

    robot.distRearDrive(1, 8);
    robot.distRightDrive(0.5, 90, 40);
    robot.distRightDrive(0.3, -90, 43);
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
          robot.centerClaw();
          robot.syncTurn(0, 2);
          break;

        case GOTOBLOCK1:
          telemetry.addData("state", currentState.toString());
          robot.fastRightDrive(90);
          currentState = AutoState.GOFORWARD;
          break;
        case GOTOBLOCK2:
          telemetry.addData("state", currentState.toString());
          //robot.distRightDrive(0.5, -90, 70);
          //robot.distRightDrive(0.3, 90, 70);
          currentState = AutoState.GOFORWARD;
          break;
        case GOTOBLOCK3:
          telemetry.addData("state", currentState.toString());
          robot.fastRightDrive(50);
//          robot.timeDrive(0.25, 0.25, 90);
          currentState = AutoState.GOFORWARD;
          break;
        case GOFORWARD:
          telemetry.addData("state", currentState.toString());
          robot.fastRearDrive(90);
          driveTime.reset();
          while (driveTime.seconds() < 2) {
            if (driveTime.seconds() < 0.75) {
              robot.lift.up();
            }
            robot.setLinearSlideDirection(LinearSlideOperation.Extend, true);
            robot.lift.stop();
          }

          robot.setLinearSlideDirection(LinearSlideOperation.None, true);

          currentState = AutoState.EXTENDSLIDE;
          break;
        case EXTENDSLIDE:

          telemetry.addData("state", currentState.toString());
          driveTime.reset();

          //while (driveTime.seconds() < 4 && !robot.slideSwitchSignaled()) {
          //robot.setLinearSlideDirection(LinearSlideOperation.Extend, false);
          //}
          robot.setLinearSlideDirection(LinearSlideOperation.None, false);
          driveTime.reset();
          //robot.rotateClaw(true);
          robot.setClawPosition(ClawPosition.Open);
          //while(driveTime.seconds() < 0.9) {
          //robot.setLinearSlideDirection(LinearSlideOperation.Retract, true);
          //}
          driveTime.reset();
          while (driveTime.seconds() < 1 && !robot.lift.atLowerLimit()) {
            robot.lift.down();

          }
          robot.lift.stop();

          robot.setLinearSlideDirection(LinearSlideOperation.None, false);

          currentState = AutoState.GRABBLOCK;

          // distToLine(x, y, z);
          break;
        case GRABBLOCK:

          telemetry.addData("state", currentState.toString());

          robot.setClawPosition(ClawPosition.Close);
          //sleep(1000);
          currentState = AutoState.GO_TO_BASE_PLATE;
          // distToLine(x, y, z);
          robot.fastRearDrive(55);
          //robot.syncTurn(-90, 3);
          driveTime.reset();
          while (driveTime.seconds() < 0.25) {
            robot.lift.up();
          }
          robot.lift.stop();

          break;


        case GO_TO_BASE_PLATE:


          telemetry.addData("state", currentState.toString());

          runTime.reset();


          runTime.reset();
          while (runTime.seconds() < 0.3) {
            robot.setLinearSlideDirection(LinearSlideOperation.Extend, false);
          }


          runTime.reset();
          while (runTime.seconds() < 0.5) {
            robot.lift.down();
          }

          robot.lift.stop();


          robot.syncTurn(-90, 3);


          if (tf.whichColumn() == 0) {
            robot.timeDrive(0.5, 3, -90);
            runtime.reset();
            while (runtime.seconds() < 1.25) {
              robot.lift.up();
            }
            robot.lift.stop();
            robot.rotateClaw(false);

            robot.timeDrive(0.3, 1.5, -90);
          } else if (tf.whichColumn() == 1) {
            robot.timeDrive(0.5, 2.7, -90);
            runtime.reset();
            while (runtime.seconds() < 1.25) {
              robot.lift.up();
            }
            robot.lift.stop();
            robot.rotateClaw(false);
            robot.timeDrive(0.4, 1.6, -90);
          } else if (tf.whichColumn() == 2) {
            robot.timeDrive(0.5, 2.5, -90);
            runtime.reset();
            while (runtime.seconds() < 1.25) {
              robot.lift.up();
            }
            robot.lift.stop();
            robot.rotateClaw(false);

            robot.timeDrive(0.3, 1.5, -90);
          }
          currentState = AutoState.PLACE_STONE;


          break;

        case PLACE_STONE:

          robot.rotateClaw(false);


          telemetry.addData("state", currentState.toString());
          telemetry.update();

          runTime.reset();


          runTime.reset();


          runTime.reset();
         /* while (runTime.seconds() < 1) {
            robot.setLinearSlideDirection(LinearSlideOperation.Extend, false);
          }*/
          robot.rotateClaw(true);

          runTime.reset();
          while (runTime.seconds() < 1.25) {
            robot.lift.down();
          }
          robot.lift.stop();

          robot.setClawPosition(ClawPosition.Open);

          runtime.reset();
          while (runtime.seconds() < 0.5) {
            robot.lift.up();
          }

          robot.lift.stop();


          runtime.reset();
          while (runtime.seconds() < 1.25) {
            robot.setLinearSlideDirection(LinearSlideOperation.Retract, false);
          }
          robot.timeDrive(0.4, 0.25, 90);
          runtime.reset();
          while (runtime.seconds() < 1) {
            robot.lift.down();
          }
          robot.lift.stop();
          robot.syncTurn(-90, 2);
          robot.fastLeftDrive(50);

          currentState = AutoState.GO_TO_LINE;

          break;

        case GO_TO_LINE:


          telemetry.addData("state", currentState.toString());
          runtime.reset();
          robot.driveToLine(0.5, 90);

          robot.stop();
          currentState = AutoState.STOP;
      }
    }
  }
}
