package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Direct Control")
public class DirectControl extends LinearOpMode {
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
        robot.claw(0.4); // Open
        telemetry.addLine("Open .4");
      }else if (control.buttonY() == Button.Pressed){
        robot.claw(0.6); // CLosed
        telemetry.addLine("Close .6");
      }
      if(control.buttonX() == Button.Pressed) {
        robot.turnn(0.4); // Open
        telemetry.addLine("Open 0.4");
      }else if (control.buttonB() == Button.Pressed){
        robot.turnn(0.6); // CLosed
        telemetry.addLine("Close 0.6");
      }
      if (control.rbump() == Button.Pressed) {
        robot.turnn();
      }

      // redid this to work with magnetic limit switch
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
       */
      if(Math.abs(slide)>robot.STICKDEADZONE){
        if(slide>0){
          robot.lslide(LinearSlideOperation.Extend);
        }else{
          robot.lslide((LinearSlideOperation.Retract));
        }
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
      robot.joystickDrive(L2, D, robot.gyroHeading());
      /*if (control.buttonY() == Button.Pressed) {
        robot.lslide(LinearSlideOperation.Extend);
      } else if (control.buttonA() == Button.Pressed) {
        robot.lslide(LinearSlideOperation.Retract);
      } else {
        // DO NOTHING, not "return;" :D
      }*/

    }
    telemetry.update();
  }
}
