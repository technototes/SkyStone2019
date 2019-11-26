package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class IMUAccelerationIntegratorTest implements BNO055IMU.AccelerationIntegrator {
  BNO055IMU.Parameters parameters;
  Acceleration acceleration;
  Position curPos;
  Velocity curVel;

  @Override
  public void initialize(BNO055IMU.Parameters parameters, Position initialPosition, Velocity initialVelocity) {
    this.parameters = parameters;
    curPos = new Position(
      DistanceUnit.CM,
      initialPosition.unit.toCm(initialPosition.x),
      initialPosition.unit.toCm(initialPosition.y),
      initialPosition.unit.toCm(initialPosition.z),
      initialPosition.acquisitionTime);
    curVel = initialVelocity;
  }

  @Override
  public Position getPosition() {
    return curPos;
  }

  @Override
  public Velocity getVelocity() {
    return curVel;
  }

  @Override
  public Acceleration getAcceleration() {
    return this.acceleration == null ? new Acceleration() : this.acceleration;
  }

  @Override
  public void update(Acceleration linearAcceleration) {
    // We should always be given a timestamp here

    // velocity = accel*dt (dt in seconds)
    // position = 0.5*accel*dt^2

    if (linearAcceleration.acquisitionTime != 0) {
      if (acceleration != null) {
        // Nanoseconds...
        double dt = 1e-9 * (linearAcceleration.acquisitionTime - acceleration.acquisitionTime);
        Acceleration accelPrev = acceleration;
        acceleration = linearAcceleration;
        double posMul = .5 * dt * dt;
        curPos = new Position(
          curPos.unit,
          curPos.x * acceleration.xAccel * posMul,
          curPos.y * acceleration.yAccel * posMul,
          curPos.z * acceleration.zAccel * posMul,
          acceleration.acquisitionTime);
        curVel = new Velocity(
          curVel.unit,
          curVel.xVeloc * acceleration.xAccel * dt,
          curVel.yVeloc * acceleration.yAccel * dt,
          curVel.zVeloc * acceleration.zAccel * dt,
          acceleration.acquisitionTime
        );
      } else
        acceleration = linearAcceleration;
    }
  }

}
