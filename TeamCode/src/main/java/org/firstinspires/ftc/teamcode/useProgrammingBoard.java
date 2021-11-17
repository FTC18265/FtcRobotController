package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class useProgrammingBoard extends OpMode {
    boolean result;
    int mode;
    programmingBoard2 programmingBoard2 = new programmingBoard2();

    @Override
    public void init() {
        programmingBoard2.init(hardwareMap);

    }

    @Override
    public void loop() {
        result = programmingBoard2.isTouchSensorPressed();

        //telemetry.addLine(programmingBoard2.isTouchSensorPressed());
        
        if(result = false){
            telemetry.addLine("not pressed");
        }
        else {
            telemetry.addLine("pressed");
        }

        if(programmingBoard2.getDistance(DistanceUnit.CM) < 10){
            programmingBoard2.setMotorSpeed(0);
        }
        else{
            programmingBoard2.setMotorSpeed(0.5);
        }

        if (programmingBoard2.getHeading(AngleUnit.DEGREES) == 0){
            programmingBoard2.setMotorSpeed(0);
            telemetry.addData("Motor", "0");
        }
        else if (programmingBoard2.getHeading(AngleUnit.DEGREES) > 0){
            programmingBoard2.setMotorSpeed(-gamepad1.right_stick_y);
            telemetry.addData("Motor", "negative");
        }
        else if (programmingBoard2.getHeading(AngleUnit.DEGREES) < 0){
            programmingBoard2.setMotorSpeed(gamepad1.right_stick_y);
            telemetry.addData("Motor", "positive");
        }

        mode = programmingBoard2.setMode(gamepad1.a, gamepad1.b);
        programmingBoard2.setServoPosition(gamepad1.left_trigger);
        programmingBoard2.setServoPosition(programmingBoard2.getPotAngleZeroToOne());

        telemetry.addData("mode", mode);

        telemetry.addData("Amount red", programmingBoard2.getAmountRed());
        telemetry.addData("Amount blue", programmingBoard2.getAmountBlue());
        telemetry.addData("Distance (CM)", programmingBoard2.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance (IN)", programmingBoard2.getDistance(DistanceUnit.INCH));
        telemetry.addData("Our Heading", programmingBoard2.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Our Radian", programmingBoard2.getHeading(AngleUnit.RADIANS));
    }
}