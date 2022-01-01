package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class TensorflowController {
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BC.tflite";
    private static final String[] LABELS = {
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY =
            " ASxWKxX/////AAABmSF4eat8sUkxvdtWRwyP9DY+VoUtavSfblX3C9SsIUO3jHJ+WdvoYKEmByzo86qe/Mg4M0Xk0XIUtrYmNyErHwILvJJxOveLX2A2UAE3jO4yeYNszMt3GKQQEbv/QkiTHQLvN/uqrXBlBowX1nDOf7HeOCSIgjyZHmX5AuPIZw4TLMZ6xs+u8gup23vhSJjPROnD9Cr6+mJpwxIhpcLDtUtNL0IQdVDTPMxEoipVDIsnLuf3vCCcV/jIK5I6a2R8sPpDaGU0v4p0r+yDVAGH6TWt5pwRuV5a5GLyhjkwih7bNyIUfWCo4bNKvPXzV4wSK6DeNxtxgHquj2xLSkyG2o+4HGMdHXEsuekdjVJtYYDu ";

    public  TensorflowController(VuforiaLocalizer vuforia, TFObjectDetector tfod){
        this.vuforia = vuforia;
        this.tfod = tfod;
    }

    public void initVuforia(){
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void initTfod(){
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.5f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

    }

    public void activation(){
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16/9);
        }
    }

    public int recognition(int position){
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                telemetry.addData("position", position);
                int i = 0;
                String labelONe = "None";
                String labelTwo = "None";
                String labelThree = "None";
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    if ((recognition.getLeft() > 450 && (recognition.getLabel() == "Duck" || recognition.getLabel() == "Cube"))){
                        position = 3;
                    } else
                    if ((recognition.getLeft() > 140 && (recognition.getLabel() == "Duck" ||
                            recognition.getLabel() == "Cube") && recognition.getLeft() < 300) ||
                            (recognition.getLeft() > 450 && recognition.getLabel() == "Marker")){
                        position = 2;
                    }

                    if (i == 0){
                        labelONe = recognition.getLabel();
                    }
                    if (i == 1){
                        labelTwo = recognition.getLabel();
                    }
                    if (i == 2){
                        labelThree = recognition.getLabel();
                    }

                    if (labelONe == "Marker" && labelTwo == "Marker" || labelThree == "Marker"){
                        position = 1;
                        labelONe = "None";
                        labelTwo = "None";
                        labelThree = "None";
                    }
                    i++;
                }
                telemetry.addData("detected position", position);
                telemetry.update();
            }
        }
        return position;
    }

}
