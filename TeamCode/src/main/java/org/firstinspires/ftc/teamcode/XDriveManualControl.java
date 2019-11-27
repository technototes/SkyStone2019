package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// For use by opModes for driving the robot manually
public class XDriveManualControl {
  TTRobot robot;
  Controller driver;
  Controller control;
  Telemetry telemetry;

  /**
   * Initialize the manual controller
   * @param r     The TTRobot
   * @param drv   The driver side controller
   * @param ctrl  The controller side controller
   * @param tel   The Telemetry thing (not currently used)
   */
  public XDriveManualControl(TTRobot r, Controller drv, Controller ctrl, Telemetry tel) {
    robot = r;
    driver = drv;
    control = ctrl;
    telemetry = tel;
  }

  // This handles all the 'read the controllers, and drive the robot that direction' stuff
  public void Steer() {
    // Driver control:
    Direction Dpad = driver.dpad();
    Direction L = driver.lstick();
    Direction R1 = driver.rstick();
    Direction R2 = control.rstick();
    Direction D = new Direction(0, 0);
    Direction L2 = new Direction(0, 0);
    if (Math.abs(R2.X) > robot.STICKDEADZONE) {
      D.X = R2.X;
    } else if (Math.abs(R1.X) > robot.STICKDEADZONE) {
      D.X = R1.X;
    }
    if (Math.abs(L.X) > robot.STICKDEADZONE) {
      L2.X = L.X;
    }
    if (Math.abs(L.Y) > robot.STICKDEADZONE) {
      L2.Y = L.Y;
    }

    // Turbo Mode (insert Tristan happy face)
    if ((control.rtrigger() == 1.0 || control.ltrigger() == 1.0)) {
      robot.speedSnail();
    } else if ((driver.rtrigger() == 1.0 || driver.ltrigger() == 1.0)) {
      robot.speedTurbo();
    } else {
      robot.speedNormal();
    }
    // If the snap-to-angle button has been pressed, override rotation with the snap angle
    if (driver.buttonY() == Button.Pressed) {
      robot.speedNormal();
      D.X = robot.snap(telemetry);
    }

    if (driver.buttonA() == Button.Pressed && driver.buttonB() == Button.Pressed) {
      robot.joystickDrive(L2, D, 0);
    } else {
      robot.joystickDrive(L2, D, robot.gyroHeading());
    }
  }
}
