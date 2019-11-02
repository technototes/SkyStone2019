package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Direct Control")
public class DirectControl extends LinearOpMode {
  private static double FINEDRIVESPEED = 0.2;
  private TTRobot robot;
  private Controller control;
  private Controller driver;

  @Override
  public void runOpMode() {
    robot = new TTRobot();
    // If you want telemetry, include a name as a string
    // If you don't want telemetry, pass a null:
    driver = new Controller(gamepad1, telemetry, "driver");
    control = new Controller(gamepad2, telemetry, null);
    robot.init(hardwareMap, telemetry);
    robot.calibrate(); // OOPS!
    telemetry.addLine("Hello!");
    telemetry.update();
    waitForStart();
    while (opModeIsActive()) {

      // Handle Grabber rotation
      /*if (control.buttonA() == Button.Pressed) {
        if (robot.getGrabberPosition() == GrabberPosition.Vertical) {
          robot.snapGrabberPosition(GrabberPosition.Horizontal);
        } else { // It'sHORIZONTAL!
          robot.snapGrabberPosition(GrabberPosition.Vertical);
        }
      }*/
      // Handle Grabber clutch
      if(control.buttonA() == Button.Pressed) {
        robot.gyroHold(1.0, 0, 3);
      }
      if (control.buttonY() == Button.Pressed){
        robot.gyroHold(1, 180, 3);
      }
      // Grabber rotation
      if(control.buttonX() == Button.Pressed) {
        robot.gyroHold(1,90, 3);
        telemetry.addLine("Open 0.4");
      }
      if (control.buttonB() == Button.Pressed){
        robot.gyroHold(1, 270, 3);
      }
      if (control.rbump() == Button.Pressed) {
        robot.turnn(1.0);
      }

      // redid this to work with magneticx limit switch
      /*Direction dpad = control.dpad();
      if(dpad.X == 1){
          robot.lslide(LinearSlideOperation.Extend);
      }else if(dpad.X == -1){
          robot.lslide(LinearSlideOperation.Retract);
      }*/
      Direction slide = control.dpad();
      if (slide.isRight()) {
        robot.simpleSlide(1);
      }
      else if (slide.isLeft()){
        robot.simpleSlide(-1);
      }
      else {
        robot.simpleSlide(0);
      }




      // Lift control:
      Direction dir = control.dpad();
      if (dir.isUp()) {
        robot.setLift(1.0);
      }
      else if (dir.isDown()) {
        robot.setLift(-1);
      }
      else {
        robot.setLift(0);
      }

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

      //Turbo Mode (insert Tristan happy face)
      if ((control.rtrigger() == 1.0 || control.ltrigger() == 1.0 )) {
        L2.turbo(0.5);
        D.turbo(0.5);
      } else if ((driver.rtrigger() == 1.0 || driver.ltrigger() == 1.0 )) {
        L2.turbo(2.0);
        D.turbo(2.0);
      }


      robot.joystickDrive(L2, D, robot.gyroHeading());
      /*if (control.buttonY() == Button.Pressed) {
        robot.lslide(LinearSlideOperation.Extend);
      } else if (control.buttonA() == Button.Pressed) {
        robot.lslide(LinearSlideOperation.Retract);
      } else {
        // DO NOTHING, not "return;" :D
      }*/


      if(driver.dpad().isDown()){
        robot.joystickDrive(new Direction(-0,-FINEDRIVESPEED), new Direction(0,0), robot.gyroHeading());
      }
      if(driver.dpad().isUp()){
        robot.joystickDrive(new Direction(0,FINEDRIVESPEED), new Direction(0,0), robot.gyroHeading());
      }
      if(driver.dpad().isLeft()){
        robot.joystickDrive(new Direction(-FINEDRIVESPEED,0), new Direction(0,0), robot.gyroHeading());
      }
      if(driver.dpad().isRight()){
        robot.joystickDrive(new Direction(FINEDRIVESPEED,0), new Direction(0,0), robot.gyroHeading());
      }
      if (driver.buttonA() == Button.Pressed && driver.buttonB() == Button.Pressed) {
        robot.joystickDrive(L2, D, 0);
      }
      else {
        robot.joystickDrive(L2, D, robot.gyroHeading());
      }
    }
    telemetry.update();
  }
}
