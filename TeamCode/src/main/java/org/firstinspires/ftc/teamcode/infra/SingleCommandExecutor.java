package org.firstinspires.ftc.teamcode.infra;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

// Execute a single command at a time, interrupting any existing command when a new command is executes
public class SingleCommandExecutor {
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
