package org.firstinspires.ftc.teamcode.infra;

import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

// Execute a single command at a specified time in the future, interrupting any existing command when a new command is scheduled
public class SingleCommandDelayedExecutor {
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
