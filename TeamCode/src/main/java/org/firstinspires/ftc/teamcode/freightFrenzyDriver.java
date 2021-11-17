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
    private DcMotor intake;
    private Servo pusher;
    private DcMotor carousel;

    private AnalogInput potentiometer;

    private int level = 0;
    private double lasttime;
    private double currenttime;

    private String button;

    public static final double NEW_P = 1.5;
    public static final double NEW_I = 0.5;
    public static final double NEW_D = 0.0;
    public static final double NEW_F = 12.6;

    @Override
    public void runOpMode() {

        topright = hardwareMap.get(DcMotor.class, "topright");
        topleft = hardwareMap.get(DcMotor.class, "topleft");
        bottomright = hardwareMap.get(DcMotor.class, "bottomright");
        bottomleft = hardwareMap.get(DcMotor.class, "bottomleft");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        intake = hardwareMap.get(DcMotor.class, "intake");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        pusher = hardwareMap.get(Servo.class, "pusher");


        topright.setDirection(DcMotorSimple.Direction.REVERSE);
        topleft.setDirection(DcMotorSimple.Direction.REVERSE);

        pusher.setPosition(1);
        carousel.setPower(0);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int position = arm.getCurrentPosition();
        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(0);

        waitForStart();

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients pidOrig = arm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients.
        arm.setVelocityPIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        arm.setPositionPIDFCoefficients(5.0);

        // re-read coefficients and verify change.
        PIDFCoefficients pidModified = arm.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setPower(0.5);
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

                carousel.setPower(0.8 );
            }
            if (gamepad1.y) {
                button = "gamepad1.y";
                carousel.setPower(-0.15);
            }
            else{
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
            telemetry.addData("Gamepad", button);


            telemetry.update();


            //arm
//            setArmLevel(level);
//            arm.setPower(0.5);
            currenttime = getRuntime();
            if(gamepad1.left_bumper && currenttime - lasttime > 1){
                button = "gamepad1.left_bumper";
                level--;
                lasttime = currenttime;
                if (level < 0){
                    level = 0;
                }
                setArm(level);
            }
            if(gamepad1.right_bumper && currenttime - lasttime > 1){
                button = "gamepad1.right_bumper";
                level++;
                lasttime = currenttime;
                if (level > 3){
                    level = 3;
                }
                setArm(level);
            }
            position = arm.getCurrentPosition();
            if (gamepad1.dpad_down && currenttime - lasttime > 0.1) {
                button = "gamepad1.dpad_down";
                position -= 35;
                arm.setTargetPosition(position);
                lasttime = currenttime;
            }
            if (gamepad1.dpad_up && currenttime - lasttime > 0.1) {
                button = "gamepad1.dpad_up";
                position += 35;
                arm.setTargetPosition(position);
                lasttime = currenttime;
            }

            //intake
            if (gamepad1.a) {
                button = "gamepad1.a";
                intake.setPower(-1);
            }
            else{
                intake.setPower(0);
            }

            //output
            if (gamepad1.x) {
                button = "gamepad1.x";
                intake.setPower(1);
                pusher.setPosition(0);
            }
            else{
                pusher.setPosition(1);
//                setArm(0);
            }

        }
    }

    public void setArm (int level){
        if (level == 1){
//            while (potentiometer.getVoltage() < 1.2165){
//                arm.setPower(0.3);
//            }
            arm.setTargetPosition(-1000);
        }
        else if (level == 2){
//            while (potentiometer.getVoltage() < 1.4214){
//                arm.setPower(0.3);
//            }
            arm.setTargetPosition(-860);
        }
        else if (level == 3){
//            while (potentiometer.getVoltage() < 1.7637){
//                arm.setPower(0.3);
//            }
            arm.setTargetPosition(-600);
        }
        else if (level == 0){
//            while (potentiometer.getVoltage() > 1.076){
//                arm.setPower(0.3);
//            }
            arm.setTargetPosition(-1100);
        }
//        arm.setPower(0);

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
