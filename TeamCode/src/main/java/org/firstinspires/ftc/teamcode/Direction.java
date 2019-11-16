package org.firstinspires.ftc.teamcode;

public class Direction {
  public double X, Y;

  public Direction(double x, double y) {
    X = x;
    Y = y;
  }

  public static Direction None = new Direction(0.0, 0.0);

  public boolean isOnlyRight() {
    return (Math.abs(Y) < .3) && (X < -.7);
  }

  public boolean isOnlyLeft() {
    return (Math.abs(Y) < .3) && (X > .7);
  }

  public boolean isOnlyUp() {
    return (Math.abs(X) < .3) && (Y > .7);
  }

  public boolean isOnlyDown() {
    return (Math.abs(X) < .3) && (Y < -.7);
  }

  public boolean isRight() {
    return (X < -.9);
  }

  public boolean isLeft() {
    return (X > .9);
  }

  public boolean isUp() {
    return (Y > .9);
  }

  public boolean isDown() {
    return (Y < -.9);
  }

  public void turbo(double value) {
    X = X * value;
    Y = Y * value;
    }
  }
