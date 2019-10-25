package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "TTAutoStoneMovedWallRed", group = "TT")
public class TTAutoStoneMovedWallRed extends LinearOpMode {

    // States
    private enum AutoState {
        INITIALIZE,

        LINE_UP_STONE,
        PICK_UP_STONE,

        GO_TO_BASE_PLATE,
        PLACE_STONE,
        GO_TO_LINE,

        STOP
    }

    private enum SkyStonePos {
        UNKNOWN,
        OneAndFour,
        TwoAndFive,
        ThreeAndSix
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
        telemetry.addData(">", "Robot Heading = %d", robot.gyroHeading());
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

                    if (skystonepos.equals(SkyStonePos.UNKNOWN) && tfod != null) {
                        tfod.activate();
                    }

                    currentState = AutoState.LINE_UP_STONE;
                    break;
                case LINE_UP_STONE:
                    telemetry.addData("state", currentState.toString());
                    runtime.reset();
                    //deciding which position to go to depending on where the stone is
                    if (skystonepos.equals(SkyStonePos.OneAndFour)) {
                        timeDrive(x, y, z);
                        gyroHold(x, y, z);
                    }
                    if (skystonepos.equals(SkyStonePos.TwoAndFive)) {
                        timeDrive(x, y, z);
                        gyroHold(x, y, z);
                    }
                    if (skystonepos.equals(SkyStonePos.ThreeAndSix)) {
                        timeDrive(x, y, z);
                        gyroHold(x, y, z);
                    }
                    currentState = AutoState.PICK_UP_STONE;
                    break;
                case PICK_UP_STONE:
                    telemetry.addData("state", currentState.toString());
                    runtime.reset();
                    //positioning the attachments to collect the stone
                    robot.slide.setPower(x);
                    robot.liftMotor.setPower(x);
                    robot.claw.setPosition(x);
                    robot.liftMotor.setPower(x);
                    break;
                case GO_TO_BASE_PLATE:
                    telemetry.addData("state", currentState.toString());
                    runtime.reset();
                    //driving to the baseplate
                    timeDrive(x, y, z);
                    distDriveRear(x, y, z);
                    gyroHold(x, y z);
                    timeDrive(x, y, z);
                    break;
                case PLACE_STONE:
                    telemetry.addData("state", currentState.toString());
                    runtime.reset();
                    robot.liftMotor.setPower(x);
                    robot.claw.setPosition(x);
                    timeDrive(x, y, z);
                    robot.motorLift(x);
                    robot.claw.setPosition(x);
                    robot.motorLift(x);
                    timeDrive(x, y, z);
                    robot.claw.setPosition(x);
                    robot.motorLift(x);
                    break;

                case GO_TO_LINE:
                    telemetry.addData("state", currentState.toString());
                    runtime.reset();
                    distToLine(x, y, z);
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
        }
    }
}
