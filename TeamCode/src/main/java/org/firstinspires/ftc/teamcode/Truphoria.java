package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

// This is based on an idea from Tristan where we just look at the image ourselves
// He prototyped it with a JPG on a full Java runtime
// Given that image, he was looking at the average image color with a set of 3 rectangles
// This version gives each of the 3 'columns' of the image a vote for how many pixels are yellow
// Whichever column has the least yellow 'wins'

// I've also pulled all the vuforia nonsense into this class, to make it easier to use
// At the end of the day, the only thing we're using vuforia for is access to the camera

public class Truphoria {

  public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.FRONT;
  public static final boolean PHONE_IS_PORTRAIT = true;
  private static final String VUFORIA_KEY =
    "AX76a6T/////AAABmZU2tJ9jdUCdhRKzbUVAd/MCbSt73wl/qTBBrkwbMoAmX27XgROvE/abAz7wLqYrhdvrU7rQM4T3jprBs9uy1hSfvVVEWmwV6a0NchoYQLsVLpVzjF5G5N0VBN2aHLw5klCeT+5ZOeFhBrmlv0l/kajhYGTX3zM4FKRpdpFGqsdSX7QVcY73ay9VaGmwbejwbnwQ60qmg47t884/UE7PNxdzpR+2XV+RBvBXDng/R5fLj1A2DpkrdBDfLjS1wHb4EvJTcu065t0imRwDtpr0iLRbZrg4gjzlb4m4tq4qVU6mzib4o3kz/qHjqOqPqvMkMXFfbUWWABaMcyCLHbNLopb4uWxd1OhRUhq1p35Oe0HZ";
  private VuforiaLocalizer vuforia = null;

  Bitmap image = null;
  Telemetry telemetry = null;

  public Truphoria(HardwareMap hardwareMap, Telemetry tel) {
    telemetry = tel;

    // Configure Vuforia
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
    parameters.vuforiaLicenseKey = VUFORIA_KEY;
    parameters.cameraDirection = CAMERA_CHOICE;
    vuforia = ClassFactory.getInstance().createVuforia(parameters);
    // Enable the image acquisition stuff (the only thing we're using)
    vuforia.enableConvertFrameToBitmap();
  }

  int votes = 0;
  int for0 = 0;
  int for1 = 0;
  int for2 = 0;

  /**
   * Return the # of images that have been analyzed
   * @return # of images
   */
  public int observations() {
    return votes;
  }

  /**
   * This just cranks the state machine to collect & analyze data
   * Keep running it until you've got enough observations & confidence to be happy
   */
  public void takeALook() {
    switch (curState) {
      case Empty:
        this.captureFrameToFile();
        break;
      case Complete:
        telemetry.addData("Info:", "Width %d Height %d (%s)", image.getWidth(), image.getHeight(), error);
        votes++;
        if(votes > 50){
          votes = 0;
          for0 = 0;
          for1 = 0;
          for2 = 0;
        }
        switch (tallyYellows()) {
          case 0:
            for0++;
            break;
          case 1:
            for1++;
            break;
          case 2:
            for2++;
            break;
          default:
            break;
        }
        curState = CaptureState.Empty;
        break;
      case Error:
        telemetry.addLine("Error!");
        curState = CaptureState.Empty;
        break;
      case Requested:
        telemetry.addData("Requested", error);
        break;
      case Processing:
        telemetry.addData("Processing", error);
        break;
      default:
        telemetry.addData("Wut?", "Error: %s", error);
        break;
    }
    telemetry.addData("Vote:", "%d %d %d (%d)", for0, for1, for2, votes);
  }
  /**
   * This tells you which column we believe has the Skystone in it currently
   * @return 0, 1, or 2 (assuming normal orientation!)
   */
  public int whichColumn() {
    if (for0 > for1 && for0 > for2) {
      return 0;
    } else if (for1 > for2) {
      return 1;
    } else {
      return 2;
    }
  }

  /**
   * Tells you the 0-1 confidence for the current vote
   * @return 0.0 -> 1.0 percentage confidence
   */
  public double confidence() {
    double sum = for0 + for1 + for2;
    if (sum > 0)
      return Math.max(for0, Math.max(for1, for2)) / sum;
    else
      return 1.0;
  }

  enum CaptureState {Empty, Requested, Processing, Complete, Error}

  // Hurray for threading synchronization...
  volatile CaptureState curState = CaptureState.Empty;
  volatile String error = "";

  // This uses Vuforia to get an image from the camera
  private boolean captureFrameToFile() {
    if (curState != CaptureState.Empty) {
      return false;
    }
    curState = CaptureState.Requested;
    telemetry.addLine("REQUESTED!");
    vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
      @Override
      public void accept(Frame frame) {
        if (curState != CaptureState.Requested) {
          error = "Trying to get the frame, but ready is already true";
          return;
        } else {
          error = "Processing";
          curState = CaptureState.Processing;
        }
        Bitmap tmpImage = vuforia.convertFrameToBitmap(frame);
        if (tmpImage != null) {
          image = tmpImage;
          curState = CaptureState.Complete;
          error = "";
        } else {
          curState = CaptureState.Error;
          error = "Failed to capture frame";
        }
      }
    }));
    return true;
  }

  // Shameless stolen from
  // https://www.researchgate.net/post/How_to_find_the_intensity_of_yellow_color_from_an_image
  private static double yellowness(int r, int g, int b) {
    double R = r / 255.0;
    double G = g / 255.0;
    double B = b / 255.0;
    double u = Math.atan2((R - G) * 0.7071067812, (R + G - 2 * B) * 0.4082482905);
    double V = Math.max(R, Math.max(G, B));
    double v = Math.min(R, Math.min(G, B));
    double S = 2 * (V - v) / (1 + Math.abs(V - 0.5) + Math.abs(v - 0.5));
    return S * Math.cos(u);
  }

  private static boolean shouldCount(int pxl) {
    int r = (pxl >> 16) & 0xff;
    int g = (pxl >> 8) & 0xff;
    int b = pxl & 0xff;
    return yellowness(r, g, b) > .5;
  }

  // This counts which of the 3 columns of the image have the most yellow
  // (or whatever the shouldCount function decides to count)
  private int tallyYellows() {
    // Scale the image *way* down
    Bitmap newImg = Bitmap.createScaledBitmap(image, 32, 18, false);
    int w = newImg.getWidth();
    int h = newImg.getHeight();
    int min = 192;
    int which = 3;
    int wholeCount = 0;
    for (int i = 0; i < h; i++) {
      int count = 0;
      String line = "";
      for (int j = 0; j < w; j++) {
        boolean yellow = shouldCount(newImg.getPixel(j, i));
        if (yellow) {
          count++;
          line = line.concat(".");
        } else {
          line = line.concat(" ");
        }
      }
      telemetry.addData(">", "%s - %d yellow", line, count);
      wholeCount += count;
      if (i == 5) {
        if (wholeCount < min) {
          min = wholeCount;
          which = 0;
        }
        wholeCount = 0;
      } else if (i == 11) {
        if (wholeCount < min) {
          min = wholeCount;
          which = 1;
        }
        wholeCount = 0;
      } else if (i == 17) {
        if (wholeCount < min) {
          min = wholeCount;
          which = 2;
        }
        wholeCount = 0;
      }
    }
    return which;
  }
}
