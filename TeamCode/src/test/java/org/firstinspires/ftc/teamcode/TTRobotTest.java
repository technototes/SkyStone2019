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
import org.mockito.stubbing.Answer;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
class TTRobotTest {
  private TTRobot ttRobot;
  private @Mock LinearOpMode opMode;
  private @Mock HardwareMap hardwareMap;
  private @Mock Telemetry telemetry;
  private @Mock CRServo lslideServo;

  private @Mock DcMotor lLiftMotor;
  private @Mock DcMotor rLiftMotor;

  private @Mock DigitalChannel lslideSwitch;
  private @Mock DigitalChannel liftSwitch;

  private @Mock Servo lClaw;
  private @Mock Servo rClaw;
  private @Mock Servo lGrabber;
  private @Mock Servo rGrabber;

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
  void setUp(@Mock BNO055IMU mockImu) {
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

    ttRobot = new TTRobot(opMode, hardwareMap, telemetry);
  }
}