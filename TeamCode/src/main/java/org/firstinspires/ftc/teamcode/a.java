package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "test")
public class a extends LinearOpMode {

  private Servo claw;
  private Servo turn;
  private boolean openclose = true;

  private void sleep(int n) {
    try {
      Thread.sleep(n);
    } catch (Exception e){}
  }
  @Override
  public void runOpMode() {
    /*slideLimit = hardwareMap.get(DigitalChannel.class, "slideLimit");
    l11 = hardwareMap.get(DigitalChannel.class, "limit11");
    liftLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");
    l13 = hardwareMap.get(DigitalChannel.class, "limit13");
    l20 = hardwareMap.get(DigitalChannel.class, "limit20");
    l21 = hardwareMap.get(DigitalChannel.class, "limit21");
    l22 = hardwareMap.get(DigitalChannel.class, "limit22");
    l23 = hardwareMap.get(DigitalChannel.class, "limit23");

    slideLimit.setMode(DigitalChannel.Mode.INPUT);
    l11.setMode(DigitalChannel.Mode.INPUT);
    liftLimit.setMode(DigitalChannel.Mode.INPUT);
    l13.setMode(DigitalChannel.Mode.INPUT);
    l20.setMode(DigitalChannel.Mode.INPUT);
    l21.setMode(DigitalChannel.Mode.INPUT);
    l22.setMode(DigitalChannel.Mode.INPUT);
    l23.setMode(DigitalChannel.Mode.INPUT);*/

    claw = hardwareMap.get(Servo.class, "claw");
    turn = hardwareMap.get(Servo.class, "grabTurn");

    waitForStart();
    int i = 0;
    while (opModeIsActive()) {
      //double val = gamepad1.left_stick_x;
      final double clawMin = claw.MIN_POSITION;
      if(gamepad2.a == true) {
        claw.setPosition(0.4); // Open
        telemetry.addLine("Open .4");
      }else if (gamepad2.y == true){
        claw.setPosition(0.6); // CLosed
        telemetry.addLine("Close .6");
      }
      if(gamepad2.x == true) {
        turn.setPosition(0.4); // Open
        telemetry.addLine("Open 0.4");
      }else if (gamepad2.b == true){
        turn.setPosition(0.6); // CLosed
        telemetry.addLine("Close 0.6");
      }
      i++;
      //telemetry.addData("Stick:", "%3.3f", val);
      telemetry.addData("servo:", "%3.3f", claw.getPosition());
      telemetry.addData("spin:", "%3.3f", turn.getPosition());
      //telemetry.addData("liftLimit:", "%d: %s", i, val(liftLimit));
      //telemetry.addData("slideLimit:", "%d: %s", i, val(slideLimit));
      sleep(10);
      telemetry.update();
    }
  }
}

