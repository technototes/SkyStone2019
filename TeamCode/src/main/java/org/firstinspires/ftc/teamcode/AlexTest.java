package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "AlexTest", group = "TT")
public class AlexTest extends LinearOpMode {

  // States
  private enum AutoState {
    INITIALIZE,
    EXTENDSLIDE,
    DROPLIFT,
    GRABBLOCK,
    RAISELIFT,
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
    robot = new TTRobot();
    robot.init(hardwareMap, telemetry);
    robot.calibrate(); // OOPS!

    sleep(2000);
    // start calibrating the gyro.
    telemetry.addData(">", "Gyro Calibrating. Do Not move!");
    //robot.calibrate();
    //telemetry.addData(">", "Robot Heading = %d", robot.gyroHeading());

    // Put vuforia Here

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
          /*
          if (skystonepos.equals(SkyStonePos.UNKNOWN) && tfod != null) {
              tfod.activate();
          }
          */
          currentState = AutoState.EXTENDSLIDE;
          break;


        case EXTENDSLIDE:



          telemetry.addData("state", currentState.toString());
          driveTime.reset();
          while (driveTime.seconds() < 3 && !robot.slideSwitchSignaled()) {
            robot.setLinearSlideDirection(LinearSlideOperation.Extend, false);
          }
          robot.setLinearSlideDirection(LinearSlideOperation.None, false);
          driveTime.reset();
          while (driveTime.seconds() < 1.12) {
            robot.setLinearSlideDirection(LinearSlideOperation.Retract, false);
          }
          robot.setLinearSlideDirection(LinearSlideOperation.None, false);

          currentState = AutoState.DROPLIFT;

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
          currentState = AutoState.RAISELIFT;
          // distToLine(x, y, z);
          break;
        case RAISELIFT:
          telemetry.addData("state", currentState.toString());
          driveTime.reset();
          while(driveTime.seconds() < 1){
            robot.liftDown();
          }
          robot.liftStop();
          currentState = AutoState.STOP;
          // distToLine(x, y, z);
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
