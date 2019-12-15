package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.Mockito;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
class TTRobotTest {
  private TTRobot ttRobot;
  private @Mock LinearOpMode opMode;
  private HardwareMap hardwareMap = mock(HardwareMap.class);
  private Telemetry telemetry = mock(Telemetry.class);
  private CRServo lslideServo = mock(CRServo.class);
  private BNO055IMU mockImu = mock(BNO055IMU.class);

  private DcMotor lLiftMotor = mock(DcMotor.class);
  private DcMotor rLiftMotor = mock(DcMotor.class);

  private DigitalChannel lslideSwitch = mock(DigitalChannel.class);
  private DigitalChannel liftSwitch = mock(DigitalChannel.class);

  private Servo lClaw = mock(Servo.class);
  private Servo rClaw = mock(Servo.class);
  private Servo lGrabber = mock(Servo.class);
  private Servo rGrabber = mock(Servo.class);
  private Servo grabTurn = mock(Servo.class);

  @org.junit.jupiter.api.Test
  void setLinearSlideDirection() {
    //when(lslideSwitch.getState()).thenReturn(true).thenReturn(false).thenReturn(true);
    ttRobot.setLinearSlideDirection(LinearSlideOperation.Retract, false);
    verify(lslideServo).setPower(0.0);

    ttRobot.setLinearSlideDirection(LinearSlideOperation.Retract, false);
    verify(lslideServo, times(2)).setPower(0.0);

    ttRobot.setLinearSlideDirection(LinearSlideOperation.Retract, true);
    verify(lslideServo).setPower(-1.0);
    verify(lslideSwitch, never()).getState();
  }

  @org.junit.jupiter.api.Test
  void liftUp() {
  }

  @org.junit.jupiter.api.Test
  void liftDown() {
  }

  @org.junit.jupiter.api.Test
  void liftStop() {
  }

  @BeforeEach
  void setUp() {
    ttRobot = buildMockRobot();
  }

  TTRobot buildMockRobot() {
    Mockito.lenient().when(hardwareMap.get(CRServo.class, "lslideServo")).thenReturn(lslideServo);
    Mockito.lenient().when(hardwareMap.get(BNO055IMU.class, "imu1")).thenReturn(mockImu);

    Mockito.lenient().when(hardwareMap.get(DcMotor.class, "motorLiftLeft")).thenReturn(lLiftMotor);
    Mockito.lenient().when(hardwareMap.get(DcMotor.class, "motorLiftRight")).thenReturn(rLiftMotor);

    Mockito.lenient().when(hardwareMap.get(DigitalChannel.class, "slideLimit")).thenReturn(lslideSwitch);
    Mockito.lenient().when(hardwareMap.get(DigitalChannel.class, "liftLimit")).thenReturn(liftSwitch);

    Mockito.lenient().when(hardwareMap.get(Servo.class, "lClaw")).thenReturn(lClaw);
    Mockito.lenient().when(hardwareMap.get(Servo.class, "rClaw")).thenReturn(rClaw);

    Mockito.lenient().when(hardwareMap.get(Servo.class, "lGrabber")).thenReturn(lGrabber);
    Mockito.lenient().when(hardwareMap.get(Servo.class, "rGrabber")).thenReturn(rGrabber);

    Mockito.lenient().when(hardwareMap.get(Servo.class, "grabTurn")).thenReturn(grabTurn);

    return new TTRobot(opMode, hardwareMap, telemetry);
  }
}