package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

public class Truphoria {

  Bitmap image;

  public Truphoria(Bitmap img) {
    image = img;
  }

  // Bounding boxes for where to look for the color of the SkyStone

  // rectangles go from the top left of the rectangle to the top right-1
  // they go from the top left to the bottom-1
  // Minimum size is 2x2

  static int[] tl1 = new int[]{30, 75};
  static int[] br1 = new int[]{32, 83};

  static int[] tl2 = new int[]{60, 75};
  static int[] br2 = new int[]{62, 83};

  static int[] tl3 = new int[]{90, 75};
  static int[] br3 = new int[]{92, 83};

  int avg1, avg2, avg3;


  public int whichPosition() {

    // pixels = imageRedo();

    avg1 = average(tl1, br1);
    avg2 = average(tl2, br2);
    avg3 = average(tl3, br3);

    System.out.println("Stone 1:" + avg1 + " Stone 2: " + avg2 + " Stone 3: " + avg3);

    if (avg1 < avg2 && avg1 < avg3) {
      return 1;
    } else if (avg2 < avg1 && avg2 < avg3) {
      return 2;
    } else if (avg3 < avg1 && avg3 < avg2) {
      return 3;
    }
    return 0;
  }

  private int getColor(int x, int y, int k) {
    int c = image.getPixel(x, y);
    if (k == 0) {
      // red
      return (c >> 16) & 0xff;
    } else if (k == 1) {
      // green
      return (c >> 8) & 0xff;
    } else {
      // blue
      return c & 0xff;
    }
  }

  public int average(int[] tl, int[] br) {
    int nowAvg = 0;
    int nowAvg2 = 0;
    int nowAvg3 = 0;
    for (int i = tl[1]; i < tl[1] + br[1] - tl[1]; i++) {
      nowAvg2 = 0;
      for (int j = tl[0]; j < tl[0] + br[0] - tl[0]; j++) {
        nowAvg = 0;
        for (int k = 0; k < 2; k++) {
          nowAvg += getColor(i, j, k);
          // print(pixels[j][i][k] + " ");
        }
        nowAvg /= 2;
        // println("x: " + j + " y: " + i);
        // println();
        // println(nowAvg);
        nowAvg2 += nowAvg;
      }
      nowAvg2 /= br[0] - tl[0];
      nowAvg3 += nowAvg2;
    }
    nowAvg3 /= br[1] - tl[1];
    return nowAvg3;
  }

}
