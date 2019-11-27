package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Truphoria Test")
public class TruphoriaTest extends LinearOpMode {

  Truphoria truphoria = null;

  @Override
  public void runOpMode() {
    truphoria = new Truphoria(hardwareMap, telemetry);

    waitForStart();
    while (opModeIsActive()) {
      truphoria.takeALook();
      telemetry.addData("Vote data:", "Column %d, %2.1f%% Confidence",
        truphoria.whichColumn(), truphoria.confidence() * 100.0);
      telemetry.update();
    }
  }
}
