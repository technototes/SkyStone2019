package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@TeleOp(name = "Vuforia Test Rahul")
public class VuforiaTestRahul extends LinearOpMode {

  private static final String VUFORIA_KEY = "AU0AKZ7/////AAABmf3wqjeGoEHZtXmxQw86ukY2Hd8BXkhGFHpX/ZW2IIi4b57Q9jfQ83BzwEHyIFWc4igMUWhS7hZSOVUJxYtiAepAYFEcGYJJ3C3H2GKKTrVDTteL5RbRRGFg3ytes4oUhIHj4iz8MwhAlSZv5IawvQLyYo6f80LQHLcZ9E4JGuo0ztT4/k4z/H3pocQ7sj6ycaPt3vA2WbphneH+7rxbW/vcU9ttwCBZtZlfwxcRrGdxveDlwtfAhO4g+jd6f3guy3Jk02QIlmmbmDh0z5jgvmlSkoEmvTt9TO01v5uBjToFaikIWo/swTU4CTkr6+DWBuI3nbiKrsQ7CxLeslG3+6PWWIikeDGEy2bwnOgLSWu6";
  private VuforiaLocalizer vuforia;
  public TFObjectDetector tfod;

  public void initVuforia() {
    /*
     * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
     */
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    parameters.vuforiaLicenseKey = VUFORIA_KEY;
    parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

    //  Instantiate the Vuforia engine
    vuforia = ClassFactory.getInstance().createVuforia(parameters);
  }

  /**
   * Initialize the Tensor Flow Object Detection engine.
   */
  public void initTfod() {
    int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
    tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    tfod.loadModelFromAsset(TFOD_MODEL_ASSET, SKYSTONE);
  }

  @Override
  public void runOpMode() {
    robot = new TTRobot();

    waitForStart();
    while (opModeIsActive()) {
      if (tfod != null && updatedRecognitions != null) {
        boolean isGoldDetected = false;
        int gMineralPos = -1;
        int sMineralPos = -1;

        telemetry.addData("# Object Detected", updatedRecognitions.size());
        for (Recognition recognition : updatedRecognitions) {
          if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
            isGoldDetected = true;
          }
          int top = (int) recognition.getTop();
          int left = (int) recognition.getLeft();
          telemetry.addData("Pos", "Left: " + left +  " | Top: "+ top + " | Label: " + recognition.getLabel());
        }
        if (updatedRecognitions.size() == 2) {
          if (isGoldDetected) {
            for (Recognition recognition : updatedRecognitions) {
              if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                gMineralPos = (int) recognition.getTop();
              } else {
                sMineralPos = (int) recognition.getTop();
              }
            }
            if (gMineralPos < sMineralPos) {
              goldMineralPosition = GoldMineralPos.LEFT;
            } else {
              goldMineralPosition = GoldMineralPos.MIDDLE;
            }
          } else {
            goldMineralPosition = GoldMineralPos.RIGHT;
          }
        }
      }

      return goldMineralPosition;
    }
    }
  }
}
