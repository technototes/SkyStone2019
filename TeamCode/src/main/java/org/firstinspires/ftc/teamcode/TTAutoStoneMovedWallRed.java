package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Disabled
@Autonomous(name = "TTAutoStoneMovedWallRed", group = "TT")
public class TTAutoStoneMovedWallRed extends LinearOpMode {

  // States
  private enum AutoState {
    INITIALIZE,
    GOTOBLOCK1,
    GOTOBLOCK2,
    GOTOBLOCK3,
    GOFORWARD,
    EXTENDSLIDE,
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

    waitForStart();

    Truphoria tf = new Truphoria(hardwareMap, telemetry);
    //robot.distRearDrive(1, 8);
    //robot.distRightDrive(0.5, 90, 40);
    //robot.distRightDrive(0.3, -90, 47);
    runtime.reset();
    while (runtime.seconds() < 2) {
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
          robot.centerClaw();
          break;

        case GOTOBLOCK1:
          if (opModeIsActive()) {
            telemetry.addData("state", currentState.toString());
            robot.distLeftDrive(0.5, 90, 90);
            robot.distLeftDrive(0.3, -90, 90);
            currentState = AutoState.GOFORWARD;
          }
          break;
        case GOTOBLOCK2:
          if (opModeIsActive()) {
            telemetry.addData("state", currentState.toString());
            //robot.distLeftDrive(0.5, 90, 70);
            //robot.distLeftDrive(0.3, -90, 70);
            currentState = AutoState.GOFORWARD;
          }
          break;
        case GOTOBLOCK3:
          if (opModeIsActive()) {
            telemetry.addData("state", currentState.toString());
            robot.distLeftDrive(0.5, -90, 50);
            robot.distLeftDrive(0.3, 90, 50);
            currentState = AutoState.GOFORWARD;
          }
          break;
        case GOFORWARD:
          if (opModeIsActive()) {
            telemetry.addData("state", currentState.toString());
            robot.distRearDrive(0.75, 100);
            driveTime.reset();
            while (driveTime.seconds() < 1) {
              robot.lift.up();
              robot.setLinearSlideDirection(LinearSlideOperation.Extend, true);
            }
            robot.lift.stop();
            while (driveTime.seconds() < 0.75) {
              robot.setLinearSlideDirection(LinearSlideOperation.Extend, true);
            }

            robot.setLinearSlideDirection(LinearSlideOperation.None, true);

            currentState = AutoState.EXTENDSLIDE;
          }
          break;
        case EXTENDSLIDE:
          if (opModeIsActive()) {

            telemetry.addData("state", currentState.toString());
            driveTime.reset();
            while (driveTime.seconds() < 1.3) {
              robot.setLinearSlideDirection(LinearSlideOperation.Extend, true);
            }
            //while (driveTime.seconds() < 4 && !robot.slideSwitchSignaled()) {
            //robot.setLinearSlideDirection(LinearSlideOperation.Extend, false);
            //}
            robot.setLinearSlideDirection(LinearSlideOperation.None, false);
            driveTime.reset();
            //robot.rotateClaw();
            robot.setClawPosition(ClawPosition.Open);
            //while(driveTime.seconds() < 0.9) {
            //robot.setLinearSlideDirection(LinearSlideOperation.Retract, true);
            //}
            robot.setLinearSlideDirection(LinearSlideOperation.None, true);
            driveTime.reset();
            while (driveTime.seconds() < 1.5 && !robot.lift.atLowerLimit()) {
              robot.lift.down();

            }
            robot.lift.stop();

            robot.setLinearSlideDirection(LinearSlideOperation.None, false);

            currentState = AutoState.GRABBLOCK;

            // distToLine(x, y, z);
          }
          break;
        case GRABBLOCK:
          if (opModeIsActive()) {

            telemetry.addData("state", currentState.toString());

            robot.setClawPosition(ClawPosition.Close);
            sleep(500);
            driveTime.reset();
            while (driveTime.seconds() < 0.25) {

              robot.lift.up();
            }
            robot.lift.stop();
            currentState = AutoState.GO_TO_BASE_PLATE;
            // distToLine(x, y, z);
            robot.distRearDrive(0.75, 40);
            robot.syncTurn(-90, 3);
            robot.distRearDrive(0.75, 40);
            robot.syncTurn(90, 3);
            driveTime.reset();
          }
          robot.setClawPosition(ClawPosition.Open);
          sleep(1000);
          currentState = AutoState.GO_TO_BASE_PLATE;
          // distToLine(x, y, z);
          robot.distRearDrive(0.5, 45);
          robot.syncTurn(90, 3);
          driveTime.reset();
          while (driveTime.seconds() < 0.25) {
            robot.lift.up();
          }
          robot.lift.stop();

          break;


        case GO_TO_BASE_PLATE:

          if (opModeIsActive()) {

            telemetry.addData("state", currentState.toString());

            runTime.reset();
            runTime.reset();
            while (runtime.seconds() < 2) {
              robot.lift.up();
            }
            robot.lift.stop();


            runTime.reset();

            while (runTime.seconds() < 0.4) {
              robot.setLinearSlideDirectionRyan(LinearSlideOperation.Retract, false);
            }
            runTime.reset();
            while (runTime.seconds() < 0.3) {
              robot.setLinearSlideDirection(LinearSlideOperation.Extend, false);
            }
            robot.rotateClaw(false);

            runTime.reset();
            while (runTime.seconds() < 0.5) {
              robot.lift.down();
            }

            robot.distLeftDrive(0.5, 0, 60);
            robot.syncTurn(-90, 3);
            robot.timeDrive(0.45, 3, -90);
            runtime.reset();
            while (runtime.seconds() < 1.5) {
              robot.lift.up();
            }
            robot.lift.stop();

            robot.timeDrive(0.3, 0.7, -90);
            robot.timeDrive(0.4, 1.6, 90);
          } else if (tf.whichColumn() == 0) {
            robot.timeDrive(0.5, 2.2, 90);
            runtime.reset();
            while (runtime.seconds() < 1.25) {
              robot.lift.up();
            }
            robot.lift.stop();
            robot.rotateClaw(false);

            robot.timeDrive(0.3, 1, 90);
          }
          currentState = AutoState.PLACE_STONE;

          break;
        case PLACE_STONE:

          if (opModeIsActive()) {
            runTime.reset();
            robot.rotateClaw(false);


            telemetry.addData("state", currentState.toString());
            telemetry.update();

            runTime.reset();


            runTime.reset();
            while (runTime.seconds() < 0.25) {
              robot.lift.up();
            }
            robot.lift.stop();
            runTime.reset();
            stop();
            currentState = TTAutoStoneMovedWallRed.AutoState.STOP;
            while (runTime.seconds() < 1.5) {
              robot.setLinearSlideDirection(LinearSlideOperation.Extend, false);
            }

            runTime.reset();


            runTime.reset();
            while (runTime.seconds() < 1.5) {
              robot.setLinearSlideDirection(LinearSlideOperation.Extend, false);
            }
            robot.rotateClaw(true);

            runTime.reset();
            while (runTime.seconds() < 0.3) {
              robot.lift.down();
            }
            robot.setClawPosition(ClawPosition.Open);
            telemetry.addData("should stop", "");
            telemetry.update();
            sleep(2000);
            stop();
            currentState = TTAutoStoneMovedWallRed.AutoState.STOP;
          }
          break;

        case GO_TO_LINE:

          if (opModeIsActive()) {
            telemetry.addData("state", currentState.toString());
            runtime.reset();
//          robot.driveToLine(1.0, 90);
          }

          break;
        case STOP:
          telemetry.addData("state", currentState.toString());

          stop();
          break;

      }
    }
  }
}
