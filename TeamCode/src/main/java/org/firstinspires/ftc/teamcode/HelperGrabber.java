package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class HelperGrabber {
  private enum AutoState {
    INITIALIZE,
    EXTENDSLIDE,
    GRABBLOCK,
    STOP
  }


  private HelperGrabber.AutoState currentState = AutoState.INITIALIZE;
  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime timer = new ElapsedTime();
  private TTRobot robot;
  private ElapsedTime driveTime = new ElapsedTime();

  public void grabBlock(){
    switch (currentState) {
      case INITIALIZE:
        runtime.reset();
          /*
          if (skystonepos.equals(SkyStonePos.UNKNOWN) && tfod != null) {
              tfod.activate();
          }
          */

        currentState = AutoState.EXTENDSLIDE;
        break;


      case EXTENDSLIDE:

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
        while(driveTime.seconds() < 0.7) {
          robot.setLinearSlideDirection(LinearSlideOperation.Retract, true);
        }
        driveTime.reset();
        while(driveTime.seconds() < 2 && !robot.liftSwitchSignaled()){
          robot.liftDown();

        }
        robot.liftStop();

        robot.setLinearSlideDirection(LinearSlideOperation.None, false);

        currentState = AutoState.GRABBLOCK;

        // distToLine(x, y, z);
        break;
      case GRABBLOCK:


        robot.claw(0.0);
        TTRobot.sleep(1000);
        currentState = AutoState.STOP;
        // distToLine(x, y, z);
        break;
      case STOP:
        robot.stop();
        break;

      default:
        robot.stop();
        break;
    }

  }
}
