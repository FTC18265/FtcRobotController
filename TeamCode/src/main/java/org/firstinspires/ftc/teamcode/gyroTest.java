package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class gyroTest extends LinearOpMode {
    BNO055IMU imu;
    private DcMotor topright;
    private DcMotor bottomright;
    private DcMotor topleft;
    private DcMotor bottomleft;

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
    static final double     P_DRIVE_COEFF           = -0.07;     // Larger is more responsive, but also less stable
    static final double     P_HORIZONTAL_COEFF      = -0.01;     // Larger is more responsive, but also less stable



    @Override
    public void runOpMode(){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        topright = hardwareMap.get(DcMotor.class, "topright");
        bottomright = hardwareMap.get(DcMotor.class, "bottomright");
        topleft = hardwareMap.get(DcMotor.class, "topleft");
        bottomleft = hardwareMap.get(DcMotor.class, "bottomleft");

//        bottomright.setDirection(DcMotorSimple.Direction.REVERSE);
//        topright.setDirection(DcMotorSimple.Direction.REVERSE);
        topleft.setDirection(DcMotorSimple.Direction.FORWARD);
        topright.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomleft.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomright.setDirection(DcMotorSimple.Direction.REVERSE);

        topleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);

        waitForStart();

        gyroDrive(0.3, -10, 0);

    }

    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        topleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.addData("heading", getAngle());
            telemetry.update();


        }
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
            speed = Range.clip(Math.abs(speed), 0.0, 1);
            topleft.setPower(speed);
            topright.setPower(speed);
            bottomleft.setPower(speed);
            bottomright.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (topleft.isBusy() && topright.isBusy() &&
                            bottomleft.isBusy() && bottomright.isBusy()) {

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

    public void gyroHorizontal( double speed,
                                double distance,
                                double angle) {
        int newTopRightTarget;
        int newBottomRightTarget;
        int newTopleftTarget;
        int newBottomLeftTarget;
        double  max;
        double  error;
        double  steer;
        double  topLeftSpeed;
        double  topRightSpeed;
        double  bottomLeftSpeed;
        double  bottomRightSpeed;
        int moveCounts;
        moveCounts = (int) (distance * COUNTS_PER_INCH * 1.4);

        if (distance > 0) {
            topleft.setDirection(DcMotorSimple.Direction.FORWARD);
            topright.setDirection(DcMotorSimple.Direction.FORWARD);
            bottomleft.setDirection(DcMotorSimple.Direction.REVERSE);
            bottomright.setDirection(DcMotorSimple.Direction.REVERSE);
//            topright.setDirection(DcMotorSimple.Direction.REVERSE);
//            bottomleft.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if (distance < 0) {
            topleft.setDirection(DcMotorSimple.Direction.REVERSE);
            topright.setDirection(DcMotorSimple.Direction.REVERSE);
            bottomleft.setDirection(DcMotorSimple.Direction.FORWARD);
            bottomright.setDirection(DcMotorSimple.Direction.FORWARD);
//            topleft.setDirection(DcMotorSimple.Direction.REVERSE);
//            bottomright.setDirection(DcMotorSimple.Direction.REVERSE);
        }

//        topleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        topright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bottomleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bottomright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        if (distance > 0) {
//            newTopleftTarget = (topleft.getCurrentPosition() + moveCounts) * 0.8;
//            newTopRightTarget = (topright.getCurrentPosition() + moveCounts) * 1;
//            newBottomLeftTarget = (bottomleft.getCurrentPosition() + moveCounts) * 0.8;
//            newBottomRightTarget = (bottomright.getCurrentPosition() + moveCounts) * 1;
//        } else {
//            newTopleftTarget = (topleft.getCurrentPosition() + moveCounts) * 1;
//            newTopRightTarget = (topright.getCurrentPosition() + moveCounts) * 0.8;
//            newBottomLeftTarget = (bottomleft.getCurrentPosition() + moveCounts) * 1;
//            newBottomRightTarget = (bottomright.getCurrentPosition() + moveCounts) * 0.8;
//        }


        newTopleftTarget = (topleft.getCurrentPosition() + moveCounts);
        newTopRightTarget = (topright.getCurrentPosition() + moveCounts);
        newBottomLeftTarget = (bottomleft.getCurrentPosition() + moveCounts);
        newBottomRightTarget = (bottomright.getCurrentPosition() + moveCounts);

        topleft.setTargetPosition((int) newTopleftTarget);
        topright.setTargetPosition((int) newTopRightTarget);
        bottomleft.setTargetPosition((int) newBottomLeftTarget);
        bottomright.setTargetPosition((int) newBottomRightTarget);

        topleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        topleft.setPower(speed);
        topright.setPower(speed);
        bottomleft.setPower(speed);
        bottomright.setPower(speed);

        while (topleft.isBusy() && topright.isBusy() &&
                bottomleft.isBusy() && bottomright.isBusy()) {

            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, P_HORIZONTAL_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
//            if (distance > 0) {
//                topLeftSpeed = speed - steer;
//                topRightSpeed = speed - steer;
//                bottomLeftSpeed = speed + steer;
//                bottomRightSpeed = speed + steer;
//            } else {
//                topLeftSpeed = speed + steer;
//                topRightSpeed = speed + steer;
//                bottomLeftSpeed = speed - steer;
//                bottomRightSpeed = speed - steer;
//            }
            if (distance < 0)
                steer *= -1.0;

            topLeftSpeed = speed + steer;
            topRightSpeed = speed - steer;
            bottomLeftSpeed = speed + steer;
            bottomRightSpeed = speed - steer;

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

        stopAllMotors();

        topleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("targetposition", newBottomLeftTarget);
        telemetry.update();
    }

    private void stopAllMotors() {
        topleft.setPower(0);
        bottomleft.setPower(0);
        topright.setPower(0);
        bottomright.setPower(0);
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
            leftSpeed  = speed * steer;
            rightSpeed   = -leftSpeed;
        }

        leftSpeed = deadZone(leftSpeed);
        rightSpeed = deadZone(rightSpeed);

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
        Orientation angles;
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    private double deadZone(double speed){
        final double MIN_SPEED = 0.2;
        if(speed > 0 && speed < MIN_SPEED){
            speed = MIN_SPEED;
        }
        if(speed < 0 && speed > -MIN_SPEED){
            speed = -MIN_SPEED;
        }
        return speed;
    }
}
