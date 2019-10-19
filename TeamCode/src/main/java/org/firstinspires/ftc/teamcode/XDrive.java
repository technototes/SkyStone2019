package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;




@TeleOp(name = "TT POV Drive Test", group = "TT")

public class TTPOVDRIVETEST extends TTLinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();

    // State used for updating telemetry
    private Orientation angles1;

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

        robot = new TTHardware();
        robot.init(hardwareMap);



        sleep(2000);

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
            rightStickX = stepInputRotate(gamepad1.right_stick_x);

            if (leftStickY != 0 || leftStickX != 0 || rightStickX != 0) {
                robotHeadingRad = Math.toRadians(getRobotHeading());
                powerCompY = (Math.cos(robotHeadingRad) * leftStickY) + (Math.sin(robotHeadingRad) * leftStickX);
                powerCompX = -(Math.sin(robotHeadingRad) * leftStickY) + (Math.cos(robotHeadingRad) * leftStickX);

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

            robot.motorFrontLeft.setPower(Range.clip(powerFrontLeft, -1.0, 1.0));
            robot.motorFrontRight.setPower(Range.clip(powerFrontRight, -1.0, 1.0));
            robot.motorRearLeft.setPower(Range.clip(powerRearLeft, -1.0, 1.0));
            robot.motorRearRight.setPower(Range.clip(powerRearRight, -1.0, 1.0));

        }
    }
}