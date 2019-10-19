package org.firstinspires.ftc.teamcode;

import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "TTAutoStone", group = "TT")
public class TTAutoStoneUnmoved extends Robot {

    // States
    private enum AutoState {
        INITIALIZE,

        LINE_UP_STONE,
        MOVE_TO_STONE,
        PICK_UP_STONE,

        GO_TO_BASE_PLATE,
        PLACE_STONE,
        MOVE_BASE_PLATE,
        GO_TO_LINE,

        STOP
    }

    private AutoState currentState = AutoState.INITIALIZE;
    private SkyStonePos skystonepos = SkyStonePos.UNKNOWN;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    private Robot robot;

    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);


        sleep(2000);
        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        robot.gyro.calibrate();
        // make sure the gyro is calibrated.
        while (robot.gyro.isCalibrating()) {
            sleep(50);
            idle();
        }
        telemetry.addData(">", "Robot Heading = %d", robot.gyro.getIntegratedZValue());
        telemetry.update();

        //Put vuforia Here

        waitForStart();

        if (tfod != null) {
            tfod.deactivate();
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            switch (currentState) {
                case INITIALIZE:
                    telemetry.addData("state", currentState.toString());
                    runtime.reset();

                    if (SkyStonePos.equals(SkyStonePos.UNKNOWN) && tfod != null) {
                        tfod.activate();
                    }

                    currentState = AutoState.LINE_UP_STONE;
                    break;
                switch (currentState) {
                    case LINE_UP_STONE:
                        telemetry.addData("state", currentState.toString());
                        runtime.reset();
                        if (SkyStonePos.equals(SkyStonePos.OneAndFour)) {
                            timeDrive(x, y, z);
                            gyroHold(x, y, z);
                        }
                        if (SkyStonePos.equals(SkyStonePos.TwoAndFive)) {
                            timeDrive(x, y, z);
                            gyroHold(x, y, z);
                        }
                        if (SkyStonePos.equals(SkyStonePos.ThreeAndSix)) {
                            timeDrive(x, y, z);
                            gyroHold(x, y, z);
                        }
                switch (currentState) {
                    case PICK_UP_STONE:
                        telemetry.addData("state", currentState.toString());
                        runtime.reset();
                        robot.linearSlideMotor.setPower(x);
                        robot..setPower(x);

                    }
                }
            }
        }
    }
}