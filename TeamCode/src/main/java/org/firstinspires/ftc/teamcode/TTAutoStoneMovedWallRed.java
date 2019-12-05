package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "TTAutoStoneMovedWallRed", group = "TT")

public class TTAutoStoneMovedWallRed extends LinearOpMode {


  private double halfSpeed = 0.5;
  // States
  private enum AutoState {
    INITIALIZE,

    LINE_UP_STONE,
    PICK_UP_STONE,

    GO_TO_BASE_PLATE,
    PLACE_STONE,
    GO_TO_LINE,

    STOP
  }



  private AutoState currentState = AutoState.INITIALIZE;
  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime timer = new ElapsedTime();
  private TTRobot robot;
  private ElapsedTime runTime = new ElapsedTime();

  @Override
  public void runOpMode() {
    double x = 0.0, y = 0.0, z = 0.0;

    /*
     * Initialize the standard drive system variables.
     * The init() method of the hardware class does most of the work here
     */
    robot = new TTRobot(this, hardwareMap, telemetry);

    telemetry.addData(">", "Robot Heading = %f", robot.gyroHeading());
    telemetry.update();
    robot.claw(1.0);
    sleep(5000);
    robot.claw(0.0);
    robot.rotateClaw(1);

    // Put truphoria Here

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
          telemetry.addData("state", currentState.toString());
          runtime.reset();
          //robot.getSkystonePosition();
          currentState = AutoState.GO_TO_BASE_PLATE;
          break;
        case LINE_UP_STONE:
          telemetry.addData("state", currentState.toString());
          runtime.reset();

          //robot.goToStone();

          currentState = AutoState.PICK_UP_STONE;
          break;
        case PICK_UP_STONE:
          telemetry.addData("state", currentState.toString());
          runtime.reset();

//          robot.grabBlock();

          break;
        case GO_TO_BASE_PLATE:


          telemetry.addData("state", currentState.toString());

          runTime.reset();

          while (runTime.seconds() < 3) {
            robot.setLinearSlideDirectionRyan(LinearSlideOperation.Retract, false);
          }
          runTime.reset();
          while (runTime.seconds() <  0.4) {
            robot.setLinearSlideDirection(LinearSlideOperation.Extend, false);
          }
          robot.rotateClaw(0);

          runTime.reset();
          while (runTime.seconds() < 1.1) {
            robot.liftDown();
          }

          robot.liftStop();
          telemetry.addData("Preparing to move from wall", "");
          telemetry.update();
          sleep(5000);

          robot.distRearDrive(halfSpeed, 60);
          robot.syncTurn(-90  , 3);
          robot.timeDrive(halfSpeed, 2.5, -90);
          runtime.reset();
          while(runtime.seconds() < 0.4) {
            robot.liftUp();
          }
          telemetry.addData("Lift should have moved up", "");
          telemetry.update();
          sleep(5000);
          robot.liftStop();
          robot.timeDrive(0.3, 0.7, -90);
          robot.timeDrive(halfSpeed, 0.2, 90);

          runtime.reset();

          robot.timeDrive(halfSpeed, 0.3, -90);
          telemetry.addData("about to start placing stone", "");
          telemetry.update();
          sleep(5000);

          break;



        case PLACE_STONE:


          telemetry.addData("state", currentState.toString());
          telemetry.update();
          sleep(2000);
          runTime.reset();


          telemetry.addData("starting to place stone", "");
          while (runTime.seconds() < 0.4) {
            robot.liftUp();
          }
          runTime.reset();
          while (runTime.seconds() < 1.5) {
            robot.setLinearSlideDirection(LinearSlideOperation.Extend, false);
          }

          runTime.reset();
          while (runTime.seconds() < 0.3) {
            robot.liftDown();
          }

          robot.claw(0.0);
          telemetry.addData("should stop", "");
          telemetry.update();
          sleep(2000);
          stop();
          currentState = AutoState.STOP;
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

        default:
          telemetry.addData("state", currentState.toString());

          stop();
          break;
      }
      telemetry.update();
    }
  }
}
