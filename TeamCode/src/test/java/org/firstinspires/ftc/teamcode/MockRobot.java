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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mockito;
import org.mockito.invocation.InvocationOnMock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.stubbing.Answer;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.Mockito.mock;

@ExtendWith(MockitoExtension.class)
class MockRobot {
  BNO055IMU mockImu = mock(BNO055IMU.class);

  DcMotor lLiftMotor = mock(DcMotor.class);
  private int leftLiftPosition = 0;
  double lLiftPower = 0.0;
  DcMotor rLiftMotor = mock(DcMotor.class);
  private int rightLiftPosition = 0;
  double rLiftPower = 0.0;

  DigitalChannel lslideSwitch = mock(DigitalChannel.class);
  DigitalChannel liftSwitch = mock(DigitalChannel.class);

  Servo claw = mock(Servo.class);
  Servo lGrabber = mock(Servo.class);
  Servo rGrabber = mock(Servo.class);
  Servo grabTurn = mock(Servo.class);
  Servo blockFlipper = mock(Servo.class);

  CRServo lslideServo = mock(CRServo.class);
  CRServo capServo = mock(CRServo.class);

  DistanceSensor sensorRangeRear = mock(DistanceSensor.class);
  DistanceSensor sensorRangeLeft = mock(DistanceSensor.class);
  DistanceSensor sensorRangeRight = mock(DistanceSensor.class);
  ColorSensor sensorColorBottom = mock(ColorSensor.class);

  DcMotor flMotor = mock(DcMotor.class);
  DcMotor frMotor = mock(DcMotor.class);
  DcMotor rlMotor = mock(DcMotor.class);
  DcMotor rrMotor = mock(DcMotor.class);

  private HardwareMap hardwareMap = mock(HardwareMap.class);
  private Orientation gyroOrientation = new Orientation();

  private class HardwareEntry {
    Class<?> hwClass;
    String name;

    HardwareEntry(Class<?> hwClass, String name) {
      this.hwClass = hwClass;
      this.name = name;
    }

    @Override
    public boolean equals(Object other) {
      if (this == other) return true;
      if (other == null) return false;
      if (getClass() != other.getClass()) return false;
      HardwareEntry otherEntry = (HardwareEntry) other;
      return Objects.equals(hwClass, otherEntry.hwClass)
        && Objects.equals(name, otherEntry.name);
    }

    @Override
    public int hashCode() {
      return Objects.hash(hwClass, name);
    }
  };

  private Map<HardwareEntry, Object> mockHardwareMap = new HashMap<HardwareEntry, Object>();

  MockRobot() {
    mockHardwareMap.put(new HardwareEntry(BNO055IMU.class, "imu1"), mockImu);

    mockHardwareMap.put(new HardwareEntry(DcMotor.class, "motorLiftLeft"), lLiftMotor);
    Mockito.lenient().when(lLiftMotor.getCurrentPosition()).then(
      new Answer<Integer>() {
        @Override
        public Integer answer(InvocationOnMock invocation) {
          return leftLiftPosition;
        }
      }
    );
    Mockito.lenient().doAnswer(new Answer<Void>() {
      @Override
      public Void answer(InvocationOnMock invocation) throws Throwable {
        lLiftPower = (double)invocation.getArgument(0);
        return null;
      }
    }).when(lLiftMotor).setPower(anyDouble());

    mockHardwareMap.put(new HardwareEntry(DcMotor.class, "motorLiftRight"), rLiftMotor);
    Mockito.lenient().when(rLiftMotor.getCurrentPosition()).then(
      new Answer<Integer>() {
        @Override
        public Integer answer(InvocationOnMock invocation) {
          return rightLiftPosition;
        }
      }
    );
    Mockito.lenient().doAnswer(new Answer<Void>() {
      @Override
      public Void answer(InvocationOnMock invocation) throws Throwable {
        rLiftPower = (double)invocation.getArgument(0);
        return null;
      }
    }).when(rLiftMotor).setPower(anyDouble());

    mockHardwareMap.put(new HardwareEntry(DigitalChannel.class, "slideLimit"), lslideSwitch);
    mockHardwareMap.put(new HardwareEntry(DigitalChannel.class, "liftLimit"), liftSwitch);

    mockHardwareMap.put(new HardwareEntry(CRServo.class, "slide"), lslideServo);
    mockHardwareMap.put(new HardwareEntry(Servo.class, "claw"), claw);

    mockHardwareMap.put(new HardwareEntry(Servo.class, "lGrabber"), lGrabber);
    mockHardwareMap.put(new HardwareEntry(Servo.class, "rGrabber"), rGrabber);

    mockHardwareMap.put(new HardwareEntry(Servo.class, "grabTurn"), grabTurn);
    mockHardwareMap.put(new HardwareEntry(Servo.class, "blockFlipper"), blockFlipper);

    mockHardwareMap.put(new HardwareEntry(CRServo.class, "cap"), capServo);

    mockHardwareMap.put(new HardwareEntry(DistanceSensor.class, "sensorRangeRear"), sensorRangeRear);
    mockHardwareMap.put(new HardwareEntry(DistanceSensor.class, "sensorRangeLeft"), sensorRangeLeft);
    mockHardwareMap.put(new HardwareEntry(DistanceSensor.class, "sensorRangeRight"), sensorRangeRight);

    mockHardwareMap.put(new HardwareEntry(ColorSensor.class, "sensorColorBottom"), sensorColorBottom);

    mockHardwareMap.put(new HardwareEntry(DcMotor.class, "motorFrontLeft"), flMotor);
    mockHardwareMap.put(new HardwareEntry(DcMotor.class, "motorFrontRight"), frMotor);
    mockHardwareMap.put(new HardwareEntry(DcMotor.class, "motorRearLeft"), rlMotor);
    mockHardwareMap.put(new HardwareEntry(DcMotor.class, "motorRearRight"), rrMotor);

    Mockito.lenient().when(hardwareMap.get(any(Class.class), anyString())).then(
      new Answer<Object>() {
        @Override
        public Object answer(InvocationOnMock invocation) throws Throwable {
          HardwareEntry entry = new HardwareEntry((Class<?>)invocation.getArgument(0), (String)invocation.getArgument(1));
          return mockHardwareMap.get(entry);
        }
      }
    );

    Mockito.lenient().when(mockImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)).thenReturn(gyroOrientation);
  }

  TTRobot buildMockRobot(LinearOpMode opMode, Telemetry telemetry) {
    return new TTRobot(opMode, hardwareMap, telemetry);
  }

  void setGyroOrientation(Orientation orientation) {
    gyroOrientation = orientation;
  }

  void setLiftPositions(int leftPosition, int rightPosition) {
    leftLiftPosition = leftPosition;
    rightLiftPosition = rightPosition;
  }
}
