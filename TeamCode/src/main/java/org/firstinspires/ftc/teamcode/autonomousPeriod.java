                                          package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.NaiveAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

import java.util.List;
import java.util.Locale;

@Autonomous(name = "autonomousPeriod")
public class autonomousPeriod extends LinearOpMode {

    private Servo arm;
    private Servo claw;
    private DcMotor topleft;
    private DcMotor bottomleft;
    private VuforiaCurrentGame vuforiaUltimateGoal;
    private TfodCurrentGame tfodUltimateGoal;
    private DcMotor topright;
    private DcMotor bottomright;
    private DcMotor shooterleft;
    private DcMotor shooterright;
    private Servo pusher;
    private ColorSensor colorsensorright;
    private ColorSensor colorsensorleft;

    Recognition recognition;
    int zone;
    //Orientation lastAngles = new Orientation();
    double globalAngle;

    BNO055IMU gyro;

    Orientation angles;

    static final double     COUNTS_PER_MOTOR_REV    = 28;
    static final double     DRIVE_GEAR_REDUCTION    = 19.2;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.93701;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415) * 0.9;


    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.07;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.14;     // Larger is more responsive, but also less stable


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        List<Recognition> recognitions;
        double index;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        topleft = hardwareMap.get(DcMotor.class, "topleft");
        bottomleft = hardwareMap.get(DcMotor.class, "bottomleft");
        vuforiaUltimateGoal = new VuforiaCurrentGame();
        tfodUltimateGoal = new TfodCurrentGame();
        topright = hardwareMap.get(DcMotor.class, "topright");
        bottomright = hardwareMap.get(DcMotor.class, "bottomright");
        shooterleft = hardwareMap.get(DcMotor.class, "shooter-left");
        shooterright = hardwareMap.get(DcMotor.class, "shooter-right");
        pusher = hardwareMap.get(Servo.class, "pusher");
        colorsensorright = hardwareMap.get(ColorSensor.class, "color sensor right");
        colorsensorleft = hardwareMap.get(ColorSensor.class, "color sensor left");

        gyro.initialize(parameters);

        // Put initialization blocks here.
        arm.setPosition(0);
        claw.setPosition(0);
        topleft.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomleft.setDirection(DcMotorSimple.Direction.REVERSE);

        topleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vuforiaUltimateGoal.initialize(
                "", // vuforiaLicenseKey
                hardwareMap.get(WebcamName.class, "Webcam "), // cameraName
                "", // webcamCalibrationFilename
                false, // useExtendedTracking
                false, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                0, // xAngle
                0, // yAngle
                0, // zAngle
                true); // useCompetitionFieldTargetLocations
        tfodUltimateGoal.initialize(vuforiaUltimateGoal, 0.5F, true, true);
        // Init TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfodUltimateGoal.activate();
        tfodUltimateGoal.setZoom(2, 16 / 9);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        while (!opModeIsActive()) {
            // Put loop blocks here.
            // Get a list of recognitions from TFOD.
            recognitions = tfodUltimateGoal.getRecognitions();
            // If list is empty, inform the user. Otherwise, go
            // through list and display info for each recognition.
            if (recognitions.size() == 0) {
                telemetry.addData("TFOD", "No items detected.");
                telemetry.addData("Zone", "A");
                zone = 0;
            } else {
                index = 0;
                // Iterate through list and call a function to
                // display info for each recognized object.
                for (Recognition recognition_item : recognitions) {
                    recognition = recognition_item;
                    // Display info.
                    displayInfo(index);
                    // Increment index.
                    index = index + 1;
                }
            }
            telemetry.addData("startAngle", getAngle());
            telemetry.update();
        }

        waitForStart();

        //
        // gyro.startAccelerationIntegration(new Position(), new Velocity(), 50);

        if (opModeIsActive()) {
            // Put run blocks here.
            telemetry.addData("zone", zone);
            telemetry.update();
            switch(zone){
                case 0:
                    zoneA();
                    break;

                case 1:
                    zoneB();
                    break;

                case 4:
                    zoneC();
                    break;

                default:

                    break;
            }

            lineDetection();

            gyroTurn(0.5,0);

            shooterleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            shooterleft.setPower(-0.42);
            shooterright.setPower(0.42);
            sleep(1000);
            for (int count = 0; count < 5; count++) {
                pusher.setPosition(1);
                sleep(1000);
                pusher.setPosition(0);
                sleep(1000);
            }
        }

        vuforiaUltimateGoal.close();
        tfodUltimateGoal.close();
    }
    private void zoneA() {
        rightByTime(294);
        gyroDrive(0.7, 100, 0);
        sleep(1000);
        arm.setPosition(1);
        claw.setPosition(1);
        sleep(1000);
        arm.setPosition(0);
        leftByTime(70);
        sleep(1000);
        //gyroHorizontal(1, -38);
        gyroDrive(0.7, -37, 0);
        sleep(1000);
        rightByTime(550);
    }

    private void zoneB() {
        leftByTime(500);
        gyroDrive(0.7, 127, 0);
        sleep(1000);
        arm.setPosition(1);
        claw.setPosition(1);
        sleep(1000);
        arm.setPosition(0);
        leftByTime(700);
        gyroDrive(0.7, -75, 0);
        rightByTime(680);
    }

    private void zoneC() {
        leftByTime(900);
        gyroDrive(0.7, 140, 0);
        gyroTurn(0.5, 90);
        gyroDrive(0.7, -22, 90);
        sleep(1000);
        arm.setPosition(1);
        claw.setPosition(1);
        sleep(1000);
        arm.setPosition(0);
        gyroDrive(0.7, 25, 0);
        gyroTurn(0.5, 0);
        gyroDrive(0.7, -85, 0);
        rightByTime(300);
        /*
        leftByTime(300);
        gyroDrive(0.7, 140, 0);
        gyroTurn(0.5, 90);
        sleep(1000);
        arm.setPosition(1);
        claw.setPosition(1);
        sleep(1000);
        arm.setPosition(0);
        gyroDrive(0.7, 24, 0);
        gyroTurn(0.5, 0);
        gyroDrive(0.7, -90, 0);
        rightByTime(450);
         */
    }

    /**
     * Display info (using telemetry) for a recognized object.
     */
    private void displayInfo(double i) {
        // Display label info.
        // Display the label and index number for the recognition.
        telemetry.addData("label " + i, recognition.getLabel());
        // Display upper corner info.
        // Display lower corner info.
        if (recognition.getLabel().equals("Single")) {
            telemetry.addData("zone", "B");
            zone = 1;
        } else if (recognition.getLabel().equals("Quad")) {
            telemetry.addData("zone", "C");
            zone = 4;
        } else {
            telemetry.addData("zone", "unkown");
        }
    }

    /**
     * Describe this function...
     */
    private void rightByTime(long time2) {
        topleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topleft.setPower(0.8);
        bottomleft.setPower(-0.8);
        topright.setPower(-1);
        bottomright.setPower(1);
        sleep(time2);


    }

    /**
     * Describe this function...
     */
    private void leftByTime(long time2) {
        topleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topleft.setPower(-1);
        bottomleft.setPower(1);
        topright.setPower(0.8);
        bottomright.setPower(-0.8);
        sleep(time2);
    }

    /**
     * Describe this function...
     */
    private void forwardByTime(long time2) {
        topleft.setPower(0.5);
        bottomleft.setPower(0.5);
        topright.setPower(0.5);
        bottomright.setPower(0.5);
        sleep(time2);
    }

    /**
     * Describe this function...
     */
    private void backwardByTime(long time2) {
        topleft.setPower(-0.5);
        bottomleft.setPower(-0.5);
        topright.setPower(-0.5);
        bottomright.setPower(-0.5);
        sleep(time2);
    }

    /**
     * Describe this function...
     */
    private void stopAllMotors() {
        topleft.setPower(0);
        bottomleft.setPower(0);
        topright.setPower(0);
        bottomright.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void lineDetection() {
        topleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (colorsensorright.red() < 1000 && colorsensorleft.red() < 2000) {
            topleft.setPower(0.2);
            bottomleft.setPower(0.2);
            topright.setPower(0.2);
            bottomright.setPower(0.2);
        }
        stopAllMotors();
    }

    public void gyroHorizontal( double speed,
                                double distance){
        double newTopleftTarget;
        double newTopRightTarget;
        double newBottomLeftTarget;
        double newBottomRightTarget;
        int moveCounts;
        moveCounts = (int)(distance * COUNTS_PER_INCH * 1.4);

        if (distance > 0){
            topright.setDirection(DcMotorSimple.Direction.REVERSE);
            bottomleft.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else if(distance < 0){
            topleft.setDirection(DcMotorSimple.Direction.REVERSE);
            bottomright.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        topleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (distance > 0){
            newTopleftTarget = (topleft.getCurrentPosition() + moveCounts) * 0.8;
            newTopRightTarget = (topright.getCurrentPosition() + moveCounts) * 1;
            newBottomLeftTarget = (bottomleft.getCurrentPosition() + moveCounts) * 0.8;
            newBottomRightTarget = (bottomright.getCurrentPosition() + moveCounts) * 1;
        }
        else{
            newTopleftTarget = (topleft.getCurrentPosition() + moveCounts) * 1;
            newTopRightTarget = (topright.getCurrentPosition() + moveCounts) * 0.8;
            newBottomLeftTarget = (bottomleft.getCurrentPosition() + moveCounts) * 1;
            newBottomRightTarget = (bottomright.getCurrentPosition() + moveCounts) * 0.8;
        }

        topleft.setTargetPosition((int)newTopleftTarget);
        topright.setTargetPosition((int)newTopRightTarget);
        bottomleft.setTargetPosition((int)newBottomLeftTarget);
        bottomright.setTargetPosition((int)newBottomRightTarget);

        topleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        topleft.setPower(speed);
        topright.setPower(speed);
        bottomleft.setPower(speed);
        bottomright.setPower(speed);

        /*
        if (opModeIsActive() &&
                (!topleft.isBusy() && !topright.isBusy() &&
                        !bottomleft.isBusy() && !bottomright.isBusy())) {
            // Stop all motion;
            stopAllMotors();

            // Turn off RUN_TO_POSITION
            topleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            topright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }*/
    }

    /*
    public void gyroHorizontal( double speed,
                                double distance){
        double newTopleftTarget;
        double newTopRightTarget;
        double newBottomLeftTarget;
        double newBottomRightTarget;
        int moveCounts;
        moveCounts = (int)(distance * COUNTS_PER_INCH * 1.4);

        if (distance > 0){
            newTopleftTarget = (topleft.getCurrentPosition() + moveCounts) * 0.8;
            newTopRightTarget = (topright.getCurrentPosition() + moveCounts) * -1;
            newBottomLeftTarget = (bottomleft.getCurrentPosition() + moveCounts) * -0.8;
            newBottomRightTarget = (bottomright.getCurrentPosition() + moveCounts) * 1;
        }
        else{
            newTopleftTarget = (topleft.getCurrentPosition() + moveCounts) * -1;
            newTopRightTarget = (topright.getCurrentPosition() + moveCounts) * 0.8;
            newBottomLeftTarget = (bottomleft.getCurrentPosition() + moveCounts) * 1;
            newBottomRightTarget = (bottomright.getCurrentPosition() + moveCounts) * -0.8;
        }

        topleft.setTargetPosition((int)newTopleftTarget);
        topright.setTargetPosition((int)newTopRightTarget);
        bottomleft.setTargetPosition((int)newBottomLeftTarget);
        bottomright.setTargetPosition((int)newBottomRightTarget);

        topleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        topleft.setPower(speed);
        topright.setPower(speed);
        bottomleft.setPower(speed);
        bottomright.setPower(speed);

        if (opModeIsActive() &&
                (!topleft.isBusy() && !topright.isBusy() &&
                        !bottomleft.isBusy() && !bottomright.isBusy())) {
            // Stop all motion;
            stopAllMotors();

            // Turn off RUN_TO_POSITION
            topleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            topright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    */

    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        topleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.addData("heading", getAngle());
            telemetry.update();
        }
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.

        topleft.setPower(leftSpeed);
        topright.setPower(rightSpeed);
        bottomleft.setPower(leftSpeed);
        bottomright.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }


    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int newTopRightTarget;
        int newBottomRightTarget;
        int newTopleftTarget;
        int newBottomLeftTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  topLeftSpeed;
        double  topRightSpeed;
        double  bottomLeftSpeed;
        double  bottomRightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newTopleftTarget = topleft.getCurrentPosition() + moveCounts;
            newTopRightTarget = topright.getCurrentPosition() + moveCounts;
            newBottomLeftTarget = bottomleft.getCurrentPosition() + moveCounts;
            newBottomRightTarget = bottomright.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            topleft.setTargetPosition(newTopleftTarget);
            topright.setTargetPosition(newTopRightTarget);
            bottomleft.setTargetPosition(newBottomLeftTarget);
            bottomright.setTargetPosition(newBottomRightTarget);

            topleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            topright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottomleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottomright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            topleft.setPower(speed);
            topright.setPower(speed);
            bottomleft.setPower(speed);
            bottomright.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (topleft.isBusy() && topright.isBusy() &&
                            bottomleft.isBusy() && bottomright.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                topLeftSpeed = speed - steer;
                topRightSpeed = speed + steer;
                bottomLeftSpeed = speed - steer;
                bottomRightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(topLeftSpeed), Math.abs(topRightSpeed));
                if (max > 1.0)
                {
                    topLeftSpeed /= max;
                    topRightSpeed /= max;
                }
                max = Math.max(Math.abs(bottomLeftSpeed), Math.abs(bottomRightSpeed));
                if (max > 1.0)
                {
                    bottomLeftSpeed /= max;
                    bottomRightSpeed /= max;
                }

                topleft.setPower(topLeftSpeed);
                topright.setPower(topRightSpeed);
                bottomleft.setPower(bottomLeftSpeed);
                bottomright.setPower(bottomRightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newTopleftTarget,  newTopRightTarget,
                        newBottomLeftTarget, newBottomRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      topleft.getCurrentPosition(),
                        topright.getCurrentPosition(), bottomleft.getCurrentPosition(),
                        bottomright.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  topLeftSpeed, topRightSpeed,
                        bottomLeftSpeed, bottomRightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            stopAllMotors();

            // Turn off RUN_TO_POSITION
            topleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            topright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public double getError(double targetAngle) {

        double robotError;
        double heading;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getAngle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        //return Range.clip(error * PCoeff, -1, 1);
        if ((error * PCoeff < 0.2) && (error * PCoeff > P_TURN_COEFF * HEADING_THRESHOLD)) return 0.3;
        if ((error * PCoeff > -0.2) && (error * PCoeff < -(P_TURN_COEFF * HEADING_THRESHOLD))) return -0.3;

        if (error * PCoeff < -0.75) return -0.75;
        if (error * PCoeff > 0.75) return 0.75;

        return error * PCoeff;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        /*
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
        */

        Orientation angles;
        angles  = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

}
