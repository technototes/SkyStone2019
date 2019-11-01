/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import java.util.Locale;

@TeleOp(name = "TT POV Drive", group = "TT")
public class TTPOVDrive extends TTLinearOpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime timer = new ElapsedTime();

  // State used for updating telemetry
  private Orientation angles1;
  private Orientation angles2;

  private double leftStickY = 0;
  private double leftStickX = 0;
  private double rightStickX = 0;

  private double robotHeadingRad = 0.0;
  private double powerCompY = 0.0;
  private double powerCompX = 0.0;

  private double powerFrontLeft = 0.0;
  private double powerFrontRight = 0.0;
  private double powerRearLeft = 0.0;
  private double powerRearRight = 0.0;

  @Override
  public void runOpMode() throws InterruptedException {

    robot = new TTRobot();
    robot.init(hardwareMap, telemetry);
    robot.calibrate();

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    // Wait for the game to start (driver presses PLAY)
    waitForStart();

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {
      // Gamepad 1 - Driving
      if (gamepad1.left_stick_y != 0) {
        leftStickY = stepInput(-gamepad1.left_stick_y);
      } else if (gamepad1.dpad_up) {
        leftStickY = 0.25;
      } else if (gamepad1.dpad_down) {
        leftStickY = -0.25;
      } else {
        leftStickY = 0.0;
      }
      if (gamepad1.left_stick_x != 0) {
        leftStickX = stepInput(gamepad1.left_stick_x);
      } else if (gamepad1.dpad_right) {
        leftStickX = 0.25;
      } else if (gamepad1.dpad_left) {
        leftStickX = -0.25;
      } else {
        leftStickX = 0.0;
      }
      rightStickX = stepInputRotate(-gamepad1.right_stick_x);

      if (leftStickY != 0 || leftStickX != 0 || rightStickX != 0) {
        //                robotHeadingRad = Math.toRadians(((360 - robot.gyro.getHeading()) % 360));
        robotHeadingRad = Math.toRadians(getRobotHeading());
        powerCompY =
            (Math.cos(robotHeadingRad) * leftStickY) + (Math.sin(robotHeadingRad) * leftStickX);
        powerCompX =
            -(Math.sin(robotHeadingRad) * leftStickY) + (Math.cos(robotHeadingRad) * leftStickX);

        powerFrontLeft = powerCompY + powerCompX + rightStickX;
        powerFrontRight = -powerCompY + powerCompX + rightStickX;
        powerRearLeft = powerCompY - powerCompX + rightStickX;
        powerRearRight = -powerCompY - powerCompX + rightStickX;
      } else {
        powerFrontLeft = 0.0;
        powerFrontRight = 0.0;
        powerRearLeft = 0.0;
        powerRearRight = 0.0;
      }

      robot.motorFrontLeft(Range.clip(powerFrontLeft, -1.0, 1.0));
      robot.motorFrontRight(Range.clip(powerFrontRight, -1.0, 1.0));
      robot.motorRearLeft(Range.clip(powerRearLeft, -1.0, 1.0));
      robot.motorRearRight(Range.clip(powerRearRight, -1.0, 1.0));

      // Gamepad 1 - Y
      if (gamepad1.y) {
        gyroHold(0.35, 45.0, 2.0);
      }
      // Gamepad 1 - B
      if (gamepad1.b) {
        gyroHold(0.35, 135.0, 2.0);
      }
      // Gamepad 1 - A
      if (gamepad1.a) {
        gyroHold(0.35, -135.0, 2.0);
      }
      // Gamepad 1 - X
      if (gamepad1.x) {
        gyroHold(0.35, -45.0, 2.0);
      }
      // Gamepad 2 - y
      if (gamepad2.y) {
        robot.setLift(.5); // TODO: This is wrong gamepad2.y);
      } else {
        robot.setLift(0);
      }
    }
  }

  String formatAngle(AngleUnit angleUnit, double angle) {
    return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
  }

  String formatDegrees(double degrees) {
    return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
  }
}
