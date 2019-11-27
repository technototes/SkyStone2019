package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Truphoria {

  Bitmap image;

  public Truphoria(Bitmap img) {
    image = img;
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
  public int whichColumn(Telemetry tel) {
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
      if (tel != null) {
        tel.addData(">", "%s - %d yellow", line, count);
      }
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
