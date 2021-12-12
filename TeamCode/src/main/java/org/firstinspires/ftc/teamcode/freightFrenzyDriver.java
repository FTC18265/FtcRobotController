package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DriverControlPeriod")
public class freightFrenzyDriver extends LinearOpMode {

    private DcMotor topright;
    private DcMotor topleft;
    private DcMotor bottomright;
    private DcMotor bottomleft;
    private DcMotorEx arm;
    private DcMotorEx susan;
    private DcMotor intake;
    private Servo door;
    private DcMotor carousel;

    private AnalogInput potentiometer;

    private int level = 0;
    private int degree = 0;
    private double lasttime;
    private double currenttime;

    private String button;

    public static final double NEW_P = 1.5;
    public static final double NEW_I = 0.5;
    public static final double NEW_D = 0.0;
    public static final double NEW_F = 12.6;

    public static final double turning_NEW_P = 30;
    public static final double turning_NEW_I = 1.5;
    public static final double turning_NEW_D = 0.5;
    public static final double turning_NEW_F = 0;

    @Override
    public void runOpMode() {

        topright = hardwareMap.get(DcMotor.class, "topright");
        topleft = hardwareMap.get(DcMotor.class, "topleft");
        bottomright = hardwareMap.get(DcMotor.class, "bottomright");
        bottomleft = hardwareMap.get(DcMotor.class, "bottomleft");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        susan = hardwareMap.get(DcMotorEx.class, "susan");
        intake = hardwareMap.get(DcMotor.class, "intake");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        door = hardwareMap.get(Servo.class, "door");


        topright.setDirection(DcMotorSimple.Direction.REVERSE);
        topleft.setDirection(DcMotorSimple.Direction.REVERSE);

        door.setPosition(0);
        carousel.setPower(0);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        susan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int position = arm.getCurrentPosition();
        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(0);

        int turningPosition = susan.getCurrentPosition();
        susan.setTargetPosition(turningPosition);
        susan.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        susan.setPower(0);

        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients pidOrig = arm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients turningPidOrig = susan.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


        // change coefficients.
        arm.setVelocityPIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        arm.setPositionPIDFCoefficients(5.0);

        susan.setVelocityPIDFCoefficients(turning_NEW_P, turning_NEW_I, turning_NEW_D, turning_NEW_F);
        susan.setPositionPIDFCoefficients(3.0);
        susan.setTargetPositionTolerance(3);

        // re-read coefficients and verify change.
        PIDFCoefficients pidModified = arm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients turningPidModified = susan.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


        arm.setPower(0.5);
        susan.setPower(0.5);

//        arm.setTargetPosition(300);

        while (opModeIsActive()) {
            //drive train
            topright.setPower
                (0.5 * (gamepad1.left_stick_x + gamepad1.right_stick_x + gamepad1.right_stick_y));
            bottomright.setPower
                (0.5 * (-gamepad1.left_stick_x + (gamepad1.right_stick_x - gamepad1.right_stick_y)));
            topleft.setPower
                (0.5 * (gamepad1.left_stick_x + (gamepad1.right_stick_x - gamepad1.right_stick_y)));
            bottomleft.setPower
                (0.5 * (-gamepad1.left_stick_x + gamepad1.right_stick_x + gamepad1.right_stick_y));
            telemetry.update();



            //carousel
            if (gamepad1.b){
                button = "gamepad1.b";
                carousel.setPower(0.3);
            }
            if (gamepad1.x) {
                button = "gamepad1.x";
                carousel.setPower(-0.3);
            }
            if (gamepad1.y) {
                button = "gamepad1.y";
                carousel.setPower(0);
            }



            //potentiometer test
            telemetry.addData("voltage", potentiometer.getVoltage());
//            telemetry.addData("Runtime", "%.03f", getRuntime());
//            telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.04f, %.04f",
//                    pidOrig.p, pidOrig.i, pidOrig.d, pidOrig.f);
//            telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f, %.04f",
//                    pidModified.p, pidModified.i, pidModified.d, pidModified.d);
            telemetry.addData("Arm", "Level: %d, Target: %d, Current: %d",
                    level, arm.getTargetPosition(), arm.getCurrentPosition());
            telemetry.addData("susan", "Degree: %d, Target: %d, Current: %d",
                    degree, susan.getTargetPosition(), susan.getCurrentPosition());
            telemetry.addData("Gamepad", button);
            telemetry.update();




            //arm
//            setArmLevel(level);
//            arm.setPower(0.5);
            currenttime = getRuntime();
            if(gamepad2.a && currenttime - lasttime > 1){
                button = "gamepad2.a";
                level--;
                lasttime = currenttime;
                if (level < 0){
                    level = 0;
                }
                setArm(level);
            }
            if(gamepad2.y && currenttime - lasttime > 1){
                button = "gamepad2.y";
                level++;
                lasttime = currenttime;
                if (level > 2){
                    level = 2;
                }
                setArm(level);
            }

            position = arm.getCurrentPosition();
            if (gamepad2.dpad_down && currenttime - lasttime > 0.1) {
                button = "gamepad2.dpad_down";
                position += 35;
                arm.setTargetPosition(position);
                lasttime = currenttime;
            }
            if (gamepad2.dpad_up && currenttime - lasttime > 0.1) {
                button = "gamepad2.dpad_up";
                position -= 35;
                arm.setTargetPosition(position);
                lasttime = currenttime;
            }



            //turn
            if(gamepad2.x && currenttime - lasttime > 1){
                button = "gamepad2.x";
                degree--;
                lasttime = currenttime;
                if (degree < -1){
                    degree = -1;
                }
                setTurn(degree);
            }
            if(gamepad2.b && currenttime - lasttime > 1){
                button = "gamepad2.b";
                degree++;
                lasttime = currenttime;
                if (degree > 1){
                    degree = 1;
                }
                setTurn(degree);
            }

            turningPosition = susan.getCurrentPosition();
            if (gamepad2.left_bumper && currenttime - lasttime > 0.1) {
                button = "gamepad2.left_bumper";
                turningPosition -= 15;
                if (turningPosition < -90){
                    turningPosition = -90;
                }
                susan.setTargetPosition(turningPosition);
                lasttime = currenttime;

            }
            if (gamepad2.right_bumper && currenttime - lasttime > 0.1) {
                button = "gamepad2.right_bumper";
                turningPosition += 15;
                if (turningPosition > 90){
                    turningPosition = 90;
                }
                susan.setTargetPosition(turningPosition);
                lasttime = currenttime;
            }


            //intake
            if (gamepad1.right_bumper) {
                button = "gamepad1.right_bumper";
                intake.setPower(-1);
            }
            else{
                intake.setPower(0);
            }

            //output
            if (gamepad1.left_bumper) {
                button = "gamepad1.left_bumper";
                door.setPosition(1);
            }
            else{
                door.setPosition(0);
//                setArm(0);
            }

        }

    }

    public void setArm (int level){
        if (level == 1){
//            while (potentiometer.getVoltage() < 1.2165){
//                arm.setPower(0.3);
//            }
            arm.setTargetPosition(-327);
        }
        else if (level == 2){
//            while (potentiometer.getVoltage() < 1.4214){
//                arm.setPower(0.3);
//            }
            arm.setTargetPosition(-1120);
        }

        else if (level == 0){
//            while (potentiometer.getVoltage() > 1.076){
//                arm.setPower(0.3);
//            }
            arm.setTargetPosition(3);
        }
//        arm.setPower(0);

    }

    public void setTurn (int degree){
        if (degree == 1){
            susan.setTargetPosition(-80);
        }
        else if (degree == -1){
            susan.setTargetPosition(80);
        }

        else if (degree == 0){
            susan.setTargetPosition(0);
        }

    }



//    public void setArmLevel(int level) {
//        double P = 0.05;
//        switch (level) {
//            case 0:
//                PController(1.076, P);
//                break;
//            case 1:
//                PController(1.2165, P);
//                break;
//            case 2:
//                PController(1.4214, P);
//                break;
//            case 3:
//                PController(1.7637, P);
//                break;
//            default:
//                break;
//        }
//    }
//
//    public void PController(double target, double p) {
//        int currentPosition = arm.getCurrentPosition();
//        int targetPosition;
//        double maxPower = 0.3;
//        double input = potentiometer.getVoltage();
//        double error = target - input;
//        double power = error * p;
//        power = Math.max(power, -maxPower);
//        power = Math.min(power, maxPower);
//        arm.setPower(power);
////            targetPosition = currentPosition + (error > 0 ? 10 : -10);
////            arm.setTargetPosition(targetPosition);
//    }
}
