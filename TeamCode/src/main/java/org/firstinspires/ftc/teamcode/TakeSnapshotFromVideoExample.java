/*package hourglass;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.List;

import javax.imageio.ImageIO;
import com.github.sarxos.webcam.Webcam;
import processing.core.*;

public class TakeSnapshotFromVideoExample extends PApplet {
	public int skystoneLocation;
	static boolean usingWebcam = true;

	public static void main(String[] args) {
		if (usingWebcam) {
			List<Webcam> webcams = Webcam.getWebcams();
			Webcam webcam = webcams.get(0);
			webcam.open();
			BufferedImage image = webcam.getImage();
			try {
				ImageIO.write(image, "jpg", new File("test.jpg"));
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		PApplet.main("hourglass.TakeSnapshotFromVideoExample");
	}

	PImage test;

	public void settings() {
		size(200, 160);
	}

	// rectangles go from the top left of the rectangle to the top right-1
	// they go from the top left to the bottom-1
	// Minimum size is 2x2
	
	int[] tl1 = new int[] { 30, 75 };
	int[] br1 = new int[] { 32, 83 };
	int[] tl2 = new int[] { 60, 75 };
	int[] br2 = new int[] { 62, 83 };
	int[] tl3 = new int[] { 90, 75 };
	int[] br3 = new int[] { 92, 83 };
	int[][][] pixels;
	int avg1;
	int avg2;
	int avg3;

	public int[] fix(int[] pt) {
		int pt2 = pt[0];
		pt[0] = pt[1];
		pt[1] = pt2;
		return pt;

	}

	public void setup() {
		 // tl1 = fix(tl1); tl2 = fix(tl2); tl3 = fix(tl3); br1 = fix(br1); br2 =
		 // fix(br2); br3 = fix(br3);

		println("Setting up");
		test = loadImage("test.jpg");
		println(test.pixelWidth + ", " + test.height);
		pixels = imageRedo(test);

		noStroke();

		for (int i = 0; i < pixels.length; i++) {
			for (int j = 0; j < pixels[i].length; j++) {
				fill(pixels[i][j][0], pixels[i][j][1], pixels[i][j][2]);
				rect(i, j, 1, 1);
			}
		}

		avg1 = average(pixels, tl1, br1);
		println();
		avg2 = average(pixels, tl2, br2);
		println();
		avg3 = average(pixels, tl3, br3);
		println();

		System.out.println("Stone 1:" + avg1 + " Stone 2: " + avg2 + " Stone 3: " + avg3);

		if (avg1 < avg2 && avg1 < avg3) {
			skystoneLocation = 1;
		} else if (avg2 < avg1 && avg2 < avg3) {
			skystoneLocation = 2;
		} else if (avg3 < avg1 && avg3 < avg2) {
			skystoneLocation = 3;
		}
		println(skystoneLocation);
	}

	private int[][][] imageRedo(PImage test) {
		int[][][] pixels = new int[test.pixelWidth][test.pixelHeight][3];
		for (int i = 0; i < test.height; i++) {
			for (int j = 0; j < test.width; j++) {
				for (int k = 0; k < 3; k++) {
					if (k == 0) {
						int c = test.pixels[i * test.pixelWidth + j];
						int r = (c >> 16) & 0xff;
						pixels[j][i][k] = r;
					} else if (k == 1) {
						int c = test.pixels[i * test.pixelWidth + j];
						int g = (c >> 8) & 0xff;
						pixels[j][i][k] = g;
					} else if (k == 2) {
						int c = test.pixels[i * test.pixelWidth + j];
						int b = c & 0xff;
						pixels[j][i][k] = b;
					}
				}
			}
		}
		return pixels;
	}

	public int average(int[][][] pixels, int[] tl, int[] br) {
		int nowAvg = 0;
		int nowAvg2 = 0;
		int nowAvg3 = 0;
		for (int i = tl[1]; i < tl[1] + br[1] - tl[1]; i++) {
			nowAvg2 = 0;
			for (int j = tl[0]; j < tl[0] + br[0] - tl[0]; j++) {
				nowAvg = 0;
				for (int k = 0; k < 2; k++) {
					nowAvg += pixels[j][i][k];
					print(pixels[j][i][k] + " ");
				}
				nowAvg /= 2;
				println("x: " + j + " y: " + i);
				println();
				// println(nowAvg);
				nowAvg2 += nowAvg;
			}
			nowAvg2 /= br[0] - tl[0];
			nowAvg3 += nowAvg2;
		}
		nowAvg3 /= br[1] - tl[1];
		return nowAvg3;
	}

	public void draw() {
		stroke(255);
		strokeWeight(1);
		fill(0);
		rect(tl1[0], tl1[1], (br1[0] - tl1[0]), (br1[1] - tl1[1]));
		rect(tl2[0], tl2[1], (br2[0] - tl2[0]), (br2[1] - tl2[1]));
		rect(tl3[0], tl3[1], (br3[0] - tl3[0]), (br3[1] - tl3[1]));
	}
}
*/


//to you don't have an excuse

