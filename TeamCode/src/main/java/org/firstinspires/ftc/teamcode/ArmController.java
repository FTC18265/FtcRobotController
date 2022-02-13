package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ArmController {
    private DcMotorEx arm;
    private static final double NEW_P = 1.5;
    private static final double NEW_I = 1;
    private static final double NEW_D = 0.0;
    private static final double NEW_F = 13.5;
//    private static final double NEW_P = 1.5;
//    private static final double NEW_I = 1;
//    private static final double NEW_D = 0.0;
//    private static final double NEW_F = 12.6;
    private int level = 0;
    public int position;

    public ArmController (DcMotorEx arm){
        this.arm = arm;
    }

    public void init (){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int position = arm.getCurrentPosition();
        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0);

        arm.setVelocityPIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        arm.setPositionPIDFCoefficients(5);
        //5

        arm.setPower(0.5);
    }

    public void adjustLevel(int offset){
        level = level + offset;
        if (level < 0){
            level = 0;
        }
        else if (level > 2){
            level = 2;
        }

        setArm(level, false);
    }

    public void setArm (int level, boolean mode){
        if (level == 0){
            arm.setPower(0.5);
            arm.setTargetPosition(0);
        }
        else if (level == 1){
            arm.setPower(0.5);
            arm.setTargetPosition(-900);
        }

        if(mode == false){
            if (level == 2){
                arm.setPower(0.5);
                arm.setTargetPosition(-1500);
            }
        }
        else if(mode == true){
            if (level == 2){
                arm.setPower(0.5);
                arm.setTargetPosition(-1650);
            }
        }
    }

    public void microAjust(int offset){
        position = arm.getCurrentPosition();
        position = position + offset;

        arm.setTargetPosition(position);
    }

    public void autoLevel(int level){
        if (level == 0){
            arm.setTargetPosition(0);
        }
        if (level == 1){
            arm.setTargetPosition(-475);
        }
        if (level == 2){
            arm.setTargetPosition(-750);
        }
        if (level == 3){
            arm.setTargetPosition(-1500);
        }
    }

    public int getLevel(){
        return level;
    }
    public int getCurrentPosition(){
        return arm.getCurrentPosition();
    }
}