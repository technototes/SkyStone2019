package org.firstinspires.ftc.teamcode.infra;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

// Execute a single command at a time, interrupting any existing command when a new command is executes
public class SingleCommandExecutor {
  private ExecutorService executor = Executors.newSingleThreadExecutor();
  private Future<?> currentFuture = null;
  private BaseCommand currentCommand = null;

  public static abstract class BaseCommand implements Runnable {
    private LinearOpMode opMode;
    private volatile boolean running = true;
    private volatile boolean interrupted = false;

    protected BaseCommand(LinearOpMode opMode) {
      this.opMode = opMode;
    }

    @Override
    public void run() {
      while (!interrupted && opMode.opModeIsActive() && !doWork()) {
        try {
          Thread.sleep(1);
        } catch (InterruptedException e) {
          interrupted = true;
        }
      }

      stopWork();
      running = false;
    }

    public void interrupt() {
      interrupted = true;
    }

    public boolean isRunning() {
      return running;
    }

    // Return true if work is done, false if it's still needed
    protected abstract boolean doWork();

    // Stop work - called after doWork returns true or when command is interrupted
    protected abstract void stopWork();
  }

  // Schedule a new command to interrupt any existing command
  public synchronized void execute(BaseCommand command) {
    cancel();
    currentCommand = command;
    currentFuture = executor.submit(command);
  }

  // Cancel and interrupt any command that's not yet done
  public synchronized void cancel() {
    if ((currentFuture != null) && !currentFuture.isDone()) {
      currentCommand.interrupt();
      currentFuture.cancel(true);

      while (currentCommand.running) {
        try {
          Thread.sleep(1);
        } catch (InterruptedException ignored) {}
      }
    }

    currentFuture = null;
    currentCommand = null;
  }

  // Check if a command is executing or not
  public synchronized boolean isDone() {
    return (currentFuture == null) || currentFuture.isDone();
  }
}
