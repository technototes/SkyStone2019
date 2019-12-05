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
    robot.distRearDrive(1, 10);
    robot.distRightDrive(0.5, 90, 55);
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
          robot.distRightDrive(0.5, -90, 85);
          robot.distRightDrive(0.3, 90, 85);
          currentState = AutoState.GOFORWARD;
          break;
        case GOTOBLOCK2:
          telemetry.addData("state", currentState.toString());
          robot.distRightDrive(0.5, -90, 65);
          robot.distRightDrive(0.3, 90, 65);
          currentState = AutoState.GOFORWARD;
          break;
        case GOTOBLOCK3:
          telemetry.addData("state", currentState.toString());
          robot.distRightDrive(0.5, 90, 45);
          robot.distRightDrive(0.3, -90, 45);
          currentState = AutoState.GOFORWARD;
          break;
        case GOFORWARD:
          telemetry.addData("state", currentState.toString());
          robot.distRearDrive(0.5, 80);
          driveTime.reset();
          while(driveTime.seconds() < 1) {
            robot.liftUp();
            robot.setLinearSlideDirection(LinearSlideOperation.Extend, true);
          }
          robot.liftStop();
          robot.setLinearSlideDirection(LinearSlideOperation.None, true);

          currentState = AutoState.EXTENDSLIDE;
          break;
        case EXTENDSLIDE:

          telemetry.addData("state", currentState.toString());
          driveTime.reset();
          while (driveTime.seconds() < 2) {
            robot.setLinearSlideDirection(LinearSlideOperation.Extend, true);
          }
          while (driveTime.seconds() < 4 && !robot.slideSwitchSignaled()) {
            robot.setLinearSlideDirection(LinearSlideOperation.Extend, false);
          }
          robot.setLinearSlideDirection(LinearSlideOperation.None, false);
          driveTime.reset();
          robot.rotateClaw(1);
          robot.claw(1.0);
          while(driveTime.seconds() < 0.9) {
            robot.setLinearSlideDirection(LinearSlideOperation.Retract, true);
          }
          robot.setLinearSlideDirection(LinearSlideOperation.None, true);
          driveTime.reset();
          while(driveTime.seconds() < 2 && !robot.liftSwitchSignaled()){
            robot.liftDown();

          }
          robot.liftStop();

          robot.setLinearSlideDirection(LinearSlideOperation.None, false);

          currentState = AutoState.GRABBLOCK;

          // distToLine(x, y, z);
          break;
        case DROPLIFT:
          robot.rotateClaw(1);
          telemetry.addData("state", currentState.toString());
          robot.claw(1.0);
          driveTime.reset();
          while(driveTime.seconds() < 3 && !robot.liftSwitchSignaled()){
            robot.liftDown();
          }
          robot.liftStop();
          currentState = AutoState.GRABBLOCK;
          // distToLine(x, y, z);
          break;
        case GRABBLOCK:

          telemetry.addData("state", currentState.toString());

          robot.claw(0.0);
          sleep(1000);
          currentState = AutoState.GO_TO_BASE_PLATE;
          // distToLine(x, y, z);
          robot.distRearDrive(0.5, 60);
          robot.syncTurn(-90, 3);
          break;
        case GO_TO_BASE_PLATE:


          telemetry.addData("state", currentState.toString());
          runtime.reset();

//          robot.timeDrive(1.0, 3.0, 180);

//          robot.syncTurn(0, 2);
          //robot.distRearDrive(0.5, 65);
          robot.timeDrive(1, 1, -90);
          robot.timeDrive(0.3, 1, -90);
          robot.timeDrive(0.5, 0.2, 90);

          //robot.syncTurn(-90, 3);


          robot.timeDrive(0.5, 0.3, -90);

          currentState = AutoState.PLACE_STONE;
          break;



        case PLACE_STONE:


          telemetry.addData("state", currentState.toString());
          runTime.reset();
          while (runTime.seconds() < 2) {
            robot.liftUp();
          }
          runTime.reset();
          while (runTime.seconds() < 2) {
            robot.setLinearSlideDirection(LinearSlideOperation.Extend, false);
          }

          runTime.reset();
          while (runTime.seconds() < 2) {
            robot.liftDown();
          }

          robot.claw(0);
          currentState = AutoState.GO_TO_LINE;

          break;

        case GO_TO_LINE:


          telemetry.addData("state", currentState.toString());
          runtime.reset();
          robot.driveToLine(0.5, 90);

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
