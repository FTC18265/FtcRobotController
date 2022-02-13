package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class GyroController extends MecanumController{
    BNO055IMU imu;

    static final double     COUNTS_PER_MOTOR_REV    = 28;
    static final double     DRIVE_GEAR_REDUCTION    = 19.2;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.93701;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415) * 0.9;

    static final double     HEADING_THRESHOLD       = 3 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.07;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = -0.07;     // Larger is more responsive, but also less stable


    public GyroController (DcMotor topleft, DcMotor topright, DcMotor bottomleft, DcMotor bottomright, BNO055IMU imu){
        super(topleft,topright,bottomleft,bottomright);
        this.imu = imu;
    }

    public void init(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        topleft.setDirection(DcMotorSimple.Direction.FORWARD);
        topright.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomleft.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomright.setDirection(DcMotorSimple.Direction.REVERSE);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        imu.initialize(parameters);
    }

    public void gyroTurn (  double speed, double angle) {
        //left positive
        //right negative
        // keep looping while we are still active, and not on heading.
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!onHeading(speed, angle, P_TURN_COEFF)) {

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
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1);
        setPower(speed);

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
            if (max > 1.0) {
                topLeftSpeed /= max;
                topRightSpeed /= max;
            }
            max = Math.max(Math.abs(bottomLeftSpeed), Math.abs(bottomRightSpeed));
            if (max > 1.0) {
                bottomLeftSpeed /= max;
                bottomRightSpeed /= max;
            }

            topleft.setPower(topLeftSpeed);
            topright.setPower(topRightSpeed);
            bottomleft.setPower(bottomLeftSpeed);
            bottomright.setPower(bottomRightSpeed);

        }
        // Stop all motion;
        stopAllMotors();

        // Turn off RUN_TO_POSITION
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        // Send desired speeds to motors.
        leftSpeed = deadZone(leftSpeed);
        rightSpeed = deadZone(rightSpeed);

        topleft.setPower(leftSpeed);
        topright.setPower(rightSpeed);
        bottomleft.setPower(leftSpeed);
        bottomright.setPower(rightSpeed);

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
