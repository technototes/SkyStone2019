package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.Mockito;
import org.mockito.invocation.InvocationOnMock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.stubbing.Answer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.AdditionalMatchers.gt;
import static org.mockito.AdditionalMatchers.lt;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
class TTRobotTest {
  private TTRobot ttRobot;
  private @Mock LinearOpMode opMode;
  private HardwareMap hardwareMap = mock(HardwareMap.class);
  private Telemetry telemetry = mock(Telemetry.class);

  private BNO055IMU mockImu = mock(BNO055IMU.class);
  private Orientation gyroOrientation = new Orientation();

  private DcMotor lLiftMotor = mock(DcMotor.class);
  private DcMotor rLiftMotor = mock(DcMotor.class);

  private DigitalChannel lslideSwitch = mock(DigitalChannel.class);
  private DigitalChannel liftSwitch = mock(DigitalChannel.class);

  private Servo claw = mock(Servo.class);
  private Servo lGrabber = mock(Servo.class);
  private Servo rGrabber = mock(Servo.class);
  private Servo grabTurn = mock(Servo.class);
  private Servo blockFlipper = mock(Servo.class);

  private CRServo lslideServo = mock(CRServo.class);
  private CRServo capServo = mock(CRServo.class);

  private DistanceSensor sensorRangeRear = mock(DistanceSensor.class);
  private double rearRangePositionCm = 0;
  private DistanceSensor sensorRangeLeft = mock(DistanceSensor.class);
  private DistanceSensor sensorRangeRight = mock(DistanceSensor.class);
  private ColorSensor sensorColorBottom = mock(ColorSensor.class);

  private DcMotor flMotor = mock(DcMotor.class);
  private DcMotor frMotor = mock(DcMotor.class);
  private DcMotor rlMotor = mock(DcMotor.class);
  private DcMotor rrMotor = mock(DcMotor.class);

  @org.junit.jupiter.api.Test
  void setLinearSlideDirection() {
    //when(lslideSwitch.getState()).thenReturn(true).thenReturn(false).thenReturn(true);
    ttRobot.setLinearSlideDirection(LinearSlideOperation.Retract, false);
    verify(lslideServo).setPower(0.0);

    ttRobot.setLinearSlideDirection(LinearSlideOperation.Retract, false);
    verify(lslideServo, times(2)).setPower(0.0);

    ttRobot.setLinearSlideDirection(LinearSlideOperation.Retract, true);
    verify(lslideServo).setPower(-0.5);
    verify(lslideSwitch, never()).getState();
  }

  @org.junit.jupiter.api.Test
  void liftUp() {
  }

  @org.junit.jupiter.api.Test
  void liftDown() {
  }

  @Test
  void liftStop() {
  }

  @Test
  void distRearDrivePIDNone() {
    rearRangePositionCm = 20;
    ttRobot.distRearDrivePID(20);
    verify(flMotor).setPower(0.0);
    verify(frMotor).setPower(0.0);
    verify(rlMotor).setPower(0.0);
    verify(rrMotor).setPower(0.0);
    verify(flMotor, never()).setPower(gt(0.0));
    verify(frMotor, never()).setPower(gt(0.0));
    verify(rlMotor, never()).setPower(gt(0.0));
    verify(rrMotor, never()).setPower(gt(0.0));
    verify(flMotor, never()).setPower(lt(0.0));
    verify(frMotor, never()).setPower(lt(0.0));
    verify(rlMotor, never()).setPower(lt(0.0));
    verify(rrMotor, never()).setPower(lt(0.0));
  }

  @Test
  void distRearDrivePIDForward() {
    Thread stopTimer = new Thread(
      new Runnable() {
        @Override
        public void run() {
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {}
          rearRangePositionCm = 22;
        }
      }
    );

    rearRangePositionCm = 0;
    ElapsedTime runTime = new ElapsedTime();
    opMode.start();
    stopTimer.start();
    ttRobot.distRearDrivePID(20);
    double msDuration = runTime.milliseconds();
    assertTrue(msDuration < 110, "msDuration less than 110");
    assertTrue(msDuration >= 100, "msDuration greater or equal to 100");
  }

  @BeforeEach
  void setUp() {
    ttRobot = buildMockRobot();
  }

  TTRobot buildMockRobot() {
    Mockito.lenient().when(hardwareMap.get(BNO055IMU.class, "imu1")).thenReturn(mockImu);
    Mockito.lenient().when(mockImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)).thenReturn(gyroOrientation);

    Mockito.lenient().when(hardwareMap.get(DcMotor.class, "motorLiftLeft")).thenReturn(lLiftMotor);
    Mockito.lenient().when(hardwareMap.get(DcMotor.class, "motorLiftRight")).thenReturn(rLiftMotor);

    Mockito.lenient().when(hardwareMap.get(DigitalChannel.class, "slideLimit")).thenReturn(lslideSwitch);
    Mockito.lenient().when(hardwareMap.get(DigitalChannel.class, "liftLimit")).thenReturn(liftSwitch);

    Mockito.lenient().when(hardwareMap.get(CRServo.class, "slide")).thenReturn(lslideServo);
    Mockito.lenient().when(hardwareMap.get(Servo.class, "claw")).thenReturn(claw);

    Mockito.lenient().when(hardwareMap.get(Servo.class, "lGrabber")).thenReturn(lGrabber);
    Mockito.lenient().when(hardwareMap.get(Servo.class, "rGrabber")).thenReturn(rGrabber);

    Mockito.lenient().when(hardwareMap.get(Servo.class, "grabTurn")).thenReturn(grabTurn);
    Mockito.lenient().when(hardwareMap.get(Servo.class, "blockFlipper")).thenReturn(blockFlipper);

    Mockito.lenient().when(hardwareMap.get(CRServo.class, "cap")).thenReturn(capServo);

    Mockito.lenient().when(hardwareMap.get(DistanceSensor.class, "sensorRangeRear")).thenReturn(sensorRangeRear);
    Mockito.lenient().when(sensorRangeRear.getDistance(DistanceUnit.CM)).then(new Answer<Double>() {
      public Double answer(InvocationOnMock invocation) {
        return rearRangePositionCm;
      }
    });

    Mockito.lenient().when(hardwareMap.get(DistanceSensor.class, "sensorRangeLeft")).thenReturn(sensorRangeLeft);
    Mockito.lenient().when(hardwareMap.get(DistanceSensor.class, "sensorRangeRight")).thenReturn(sensorRangeRight);

    Mockito.lenient().when(hardwareMap.get(ColorSensor.class, "sensorColorBottom")).thenReturn(sensorColorBottom);

    Mockito.lenient().when(hardwareMap.get(DcMotor.class, "motorFrontLeft")).thenReturn(flMotor);
    Mockito.lenient().when(hardwareMap.get(DcMotor.class, "motorFrontRight")).thenReturn(frMotor);
    Mockito.lenient().when(hardwareMap.get(DcMotor.class, "motorRearLeft")).thenReturn(rlMotor);
    Mockito.lenient().when(hardwareMap.get(DcMotor.class, "motorRearRight")).thenReturn(rrMotor);

    return new TTRobot(opMode, hardwareMap, telemetry);
  }
}