package org.firstinspires.ftc.teamcode;

public class Direction {
  public double X, Y;

  public Direction(double x, double y) {
    X = x;
    Y = y;
  }

  public static Direction None = new Direction(0.0, 0.0);

  public boolean isRight() {
    return (Math.abs(Y) < .1) && (X < -.9);
  }

  public boolean isLeft() {
    return (Math.abs(Y) < .1) && (X > .9);
  }

  public boolean isUp() {
    return (Math.abs(X) < .1) && (Y > .9);
  }

  public boolean isDown() {
    return (Math.abs(X) < .1) && (Y < -.9);
  }
}
