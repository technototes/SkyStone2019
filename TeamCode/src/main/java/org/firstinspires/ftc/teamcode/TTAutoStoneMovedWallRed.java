package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "TTAutoMovedWallRed", group = "TT")
public class TTAutoStoneMovedWallRed extends LinearOpMode {

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

    waitForStart();

    Truphoria tf = new Truphoria(hardwareMap, telemetry);
    robot.distRearDrive(1, 8);
    robot.distRightDrive(0.5, 90, 45);
    robot.distRightDrive(0.3, -90, 45);
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
          break;

        case GOTOBLOCK1:
          telemetry.addData("state", currentState.toString());
          robot.distRightDrive(0.5, -90, 90);
          robot.distRightDrive(0.3, 90, 90);
          currentState = AutoState.GOFORWARD;
          break;
        case GOTOBLOCK2:
          telemetry.addData("state", currentState.toString());
          robot.distRightDrive(0.5, -90, 70);
          robot.distRightDrive(0.3, 90, 70);
          currentState = AutoState.GOFORWARD;
          break;
        case GOTOBLOCK3:
          telemetry.addData("state", currentState.toString());
          //robot.distRightDrive(0.5, 90, 45);
          //robot.distRightDrive(0.3, -90, 55);
          currentState = AutoState.GOFORWARD;
          break;
        case GOFORWARD:
          telemetry.addData("state", currentState.toString());
          robot.distRearDrive(0.75, 100);
          driveTime.reset();
          while(driveTime.seconds() < 1) {
            robot.liftUp();
            robot.setLinearSlideDirection(LinearSlideOperation.Extend, true);
          }
          robot.liftStop();
          while(driveTime.seconds() < 0.75){
            robot.setLinearSlideDirection(LinearSlideOperation.Extend, true);
          }

          robot.setLinearSlideDirection(LinearSlideOperation.None, true);

          currentState = AutoState.EXTENDSLIDE;
          break;
        case EXTENDSLIDE:

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
          robot.rotateClaw(1);
          robot.claw(1.0);
          //while(driveTime.seconds() < 0.9) {
          //robot.setLinearSlideDirection(LinearSlideOperation.Retract, true);
          //}
          robot.setLinearSlideDirection(LinearSlideOperation.None, true);
          driveTime.reset();
          while(driveTime.seconds() < 1.5 && !robot.liftSwitchSignaled()){
            robot.liftDown();

          }
          robot.liftStop();

          robot.setLinearSlideDirection(LinearSlideOperation.None, false);

          currentState = AutoState.GRABBLOCK;

          // distToLine(x, y, z);
          break;
        case GRABBLOCK:

          telemetry.addData("state", currentState.toString());

          robot.claw(0.0);
          sleep(500);
          driveTime.reset();
          while(driveTime.seconds() < 0.25){

            robot.liftUp();
          }
          robot.liftStop();
          currentState = AutoState.GO_TO_BASE_PLATE;
          // distToLine(x, y, z);
          robot.distRearDrive(0.75, 40);
          robot.syncTurn(-90, 3);
          driveTime.reset();


          break;



        case GO_TO_BASE_PLATE:


          telemetry.addData("state", currentState.toString());

          runTime.reset();
          while (runtime.seconds() < 2) {
            robot.liftUp();
          }
          robot.liftStop();


          runTime.reset();

          while (runTime.seconds() < 0.4) {
            robot.setLinearSlideDirectionRyan(LinearSlideOperation.Retract, false);
          }
          runTime.reset();
          while (runTime.seconds() < 0.3) {
            robot.setLinearSlideDirection(LinearSlideOperation.Extend, false);
          }
          robot.rotateClaw(0);




          robot.distLeftDrive(0.5, 0, 60);
          robot.syncTurn(-90, 3);
          robot.timeDrive(0.55, 3, -90);
          runtime.reset();
          while (runtime.seconds() < 1.5) {
            robot.liftUp();
          }
          robot.liftStop();

          robot.timeDrive(0.3, 0.7, -90);


          runtime.reset();



          break;


        case PLACE_STONE:



          runTime.reset();



          runTime.reset();
          while (runTime.seconds() < 0.75) {
            robot.liftUp();
          }
          robot.liftStop();
          runTime.reset();
          stop();
          currentState = TTAutoStoneMovedWallRed.AutoState.STOP;
          while (runTime.seconds() < 1.5) {
            robot.setLinearSlideDirection(LinearSlideOperation.Extend, false);
          }

          runTime.reset();
          while (runTime.seconds() < 0.3) {
            robot.liftDown();
          }

          robot.claw(0.0);

          stop();
          currentState = TTAutoStoneMovedWallRed.AutoState.STOP;
          break;

        case GO_TO_LINE:


          telemetry.addData("state", currentState.toString());
          runtime.reset();
//          robot.driveToLine(1.0, 90);


          break;
        case STOP:
          telemetry.addData("state", currentState.toString());

          stop();
          break;

      }
    }
  }
}
