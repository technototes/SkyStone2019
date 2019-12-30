package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class LiftControl {
  // The amount we divide speed by when dropping the lift
  private static final double DOWNWARD_SCALE = 2.0;

  // This is how many 'ticks' a brick is
  private static final int BRICK_HEIGHT = 1200;

  // This is how high the base plate is (to get *over* it while holding a brick)
  private static final int BASE_PLATE_HEIGHT = 400;

  // This is the height offset for placing a brick
  private static final int PLACE_HEIGHT_OFFSET = 100;

  // How many ticks should we be within for 'zero'
  private static final int ZERO_TICK_RANGE = 150;

  // How many ticks should we be within for a given height
  private static final int POSITION_TICK_RANGE = 75;

  // Maximum height in bricks, to avoid damaging robot
  private static final int MAX_BRICK_HEIGHT = 5;

  // Maximum height in 'ticks', to avoid damaging robot
  private static final int MAX_HEIGHT = PLACE_HEIGHT_OFFSET + BASE_PLATE_HEIGHT + (MAX_BRICK_HEIGHT * BRICK_HEIGHT);

  // Power value to move the lifts up and down
  private static final double LIFT_UP_POWER = -1.0;
  private static final double LIFT_DOWN_POWER = 1.0;

  // Amount of time to wait (in milliseconds) for a new motor command before auto-stopping the lift motors
  private static final long AUTO_STOP_DELAY_MS = 500;

  private final DcMotor left;
  private final DcMotor right;

  private int lZero;
  private int rZero;

  // to check opModeIsActive()...
  private LinearOpMode opMode;

  private SingleCommandExecutor commandExecutor;
  private SingleCommandDelayedExecutor watchdogExecutor;

  // Asynchronous command execution helpers

  // Execute a single command at a time, interrupting any existing command when a new command is executes
  private static class SingleCommandExecutor {
    private ExecutorService executor = Executors.newSingleThreadExecutor();
    private Future<?> currentCommand = null;

    // Schedule a new command to interrupt any existing command
    public synchronized void execute(Runnable command) {
      cancel();
      currentCommand = executor.submit(command);
    }

    // Cancel and interrupt any command that's not yet done
    public synchronized void cancel() {
      if ((currentCommand != null) && !currentCommand.isDone()) {
        currentCommand.cancel(true);
      }

      currentCommand = null;
    }

    // Check if a command is executing or not
    public synchronized boolean isDone() {
      return (currentCommand == null) || currentCommand.isDone();
    }
  }

  // Execute a single command at a specified time in the future, interrupting any existing command when a new command is scheduled
  private static class SingleCommandDelayedExecutor {
    private ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
    private Future<?> currentCommand = null;

    // Schedule a new command to interrupt any existing command after delayMS milliseconds
    public synchronized void schedule(Runnable command, long delayMS) {
      cancel();
      currentCommand = executor.schedule(command, delayMS, TimeUnit.MILLISECONDS);
    }

    // Cancel and interrupt any command that's not yet done
    public synchronized void cancel() {
      if ((currentCommand != null) && !currentCommand.isDone()) {
        currentCommand.cancel(true);
      }

      currentCommand = null;
    }
  }

  // Move the lift to the specified number of 'brick' increments above the baseplate
  private static class LiftBrickCommand implements Runnable {
    private LiftControl liftControl;
    private LinearOpMode opMode;
    private int brickHeight;

    public LiftBrickCommand(LiftControl liftControl, LinearOpMode opMode, int brickHeight) {
      if ((brickHeight < 0) || (brickHeight > MAX_BRICK_HEIGHT)) {
        throw new IllegalArgumentException("Invalid brickHeight");
      }

      this.liftControl = liftControl;
      this.opMode = opMode;
      this.brickHeight = brickHeight;
    }

    @Override
    public void run() {
      while (!liftControl.LiftBrick(brickHeight) && opMode.opModeIsActive()) {
        opMode.sleep(1);
      }
    }
  }

  // Abort any commands in the given SingleCommandExecutor and stop lift
  private static class StopCommand implements Runnable {
    private LiftControl liftControl;
    private SingleCommandExecutor commandExecutor;

    public StopCommand(LiftControl liftControl, SingleCommandExecutor commandExecutor) {
      this.liftControl = liftControl;
      this.commandExecutor = commandExecutor;
    }

    @Override
    public void run() {
      commandExecutor.cancel();
      liftControl.stop();
    }
  }

  public LiftControl(LinearOpMode op, DcMotor leftMotor, DcMotor rightMotor) {
    opMode = op;

    left = leftMotor;
    right = rightMotor;

    commandExecutor = new SingleCommandExecutor();
    watchdogExecutor = new SingleCommandDelayedExecutor();

    // make lift motors work together: they're facing opposite directions
    left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    left.setDirection(DcMotor.Direction.FORWARD);
    right.setDirection(DcMotor.Direction.REVERSE);

    // Make the motors *stop* when we hit zero, not float around
    left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    lZero = left.getCurrentPosition();
    rZero = right.getCurrentPosition();
  }

  public void up() {
    setLiftPower(LIFT_UP_POWER);
  }

  public void down() {
    if (!atLowerLimit())
      setLiftPower(LIFT_DOWN_POWER);
  }

  public void overrideDown() {
    setLiftPower(LIFT_DOWN_POWER);
  }

  public void stop() {
    setLiftPower(0);
  }

  // This is a little more paranoid that 'In the bottom end of the range'
  // to try to prevent more lift axle carnage...
  public boolean atLowerLimit() {
    return BothInRange(0, ZERO_TICK_RANGE) || LeftPos() < 0 || RightPos() < 0;
  }
  // Crash recovery here
  public void ResetZero() {
    lZero = left.getCurrentPosition();
    rZero = right.getCurrentPosition();
  }

  private void setLiftPower(double val) {
    if (val == 0) {
      watchdogExecutor.cancel();
    } else {
      watchdogExecutor.schedule(new StopCommand(this, commandExecutor), AUTO_STOP_DELAY_MS);
    }

    if (val > 0) // If we're headed down, scale the power
      val = val / DOWNWARD_SCALE;
    left.setPower(val);
    right.setPower(val);
  }

  // Head toward the height at 'zero' to grab a brick in front of the bot
  // Returns true (which you can ignore) if it's stopped
  public boolean AcquireBrick() {
    // This just goes down until we're at the lower limit
    if (atLowerLimit()) {
      stop();
      return true;
    } else {
      down();
      return false;
    }
  }

  // Synchronously moves to the 'grab a brick' height
  public void AcquireBrickWait() {
    while (!AcquireBrick() && opMode.opModeIsActive()) {
      opMode.sleep(1);
    }
  }

  public boolean GoToPosition(int target) {
    if (AverageInRange(target, POSITION_TICK_RANGE)) {
      stop();
      return true;
    }

    int avg = AveragePos();
    if (avg > target) {
      down();
    } else {
      up();
    }
    return false;
  }

  // Lift a brick to 'positioning' height (0: baseplate placing height, 1: bp+1, etc...)
  private boolean LiftBrick(int brickHeight) {
    return GoToPosition(brickHeight * BRICK_HEIGHT + BASE_PLATE_HEIGHT);
  }

  // Wait to lift a brick to 'positioning' height
  public void LiftBrickWait(int brickHeight) {
    brickHeight = Math.max(0, Math.min(brickHeight, MAX_BRICK_HEIGHT));
    LiftBrickCommand liftBrickCommand = new LiftBrickCommand(this, opMode, brickHeight);
    commandExecutor.execute(liftBrickCommand);

    while (!commandExecutor.isDone() && opMode.opModeIsActive()) {
      opMode.sleep(1);
    }
  }

  // Asynchronously lift a brick to 'brickHeight' height
  public void LiftBrickAsync(int brickHeight) {
    brickHeight = Math.max(0, Math.min(brickHeight, MAX_BRICK_HEIGHT));
    LiftBrickCommand liftBrickCommand = new LiftBrickCommand(this, opMode, brickHeight);
    commandExecutor.execute(liftBrickCommand);
  }

  // Move *down* to the nearest 'whole brick on top of the baseplate' height
  // This height should be the right height to release a brick on the stack
  public boolean SetBrick() {
    int cur = AveragePos();
    int targetLevel = cur / BRICK_HEIGHT;
    int target = targetLevel * BRICK_HEIGHT + PLACE_HEIGHT_OFFSET;
    if (Math.abs(cur - target) < POSITION_TICK_RANGE) {
      stop();
      return true;
    }
    // Just in case, let's make sure we don't try to slam into the bot...
    if (atLowerLimit()) {
      stop();
      return true;
    }
    // We're not within that range, so keep moving down
    // This might result in overshooting. Could make this thing use PID at some point...
    down();
    return false;
  }

  // Synchronous version of the SetBrick function
  public void SetBrickWait() {
    while (!SetBrick() && opMode.opModeIsActive()) {
      opMode.sleep(1);
    }
  }

  // Helpers

  private int LeftPos() {
    return Math.max(0, left.getCurrentPosition() - lZero);
  }

  private int RightPos() {
    return Math.max(0, right.getCurrentPosition() - rZero);
  }

  private int AveragePos() {
    return (LeftPos() + RightPos()) / 2;
  }

  private boolean BothInRange(int target, int error) {
    return (Math.abs(target - LeftPos()) < error) && (Math.abs(target - RightPos()) < error);
  }

  private boolean AverageInRange(int target, int error) {
    return Math.abs(target - AveragePos()) < error;
  }
}
