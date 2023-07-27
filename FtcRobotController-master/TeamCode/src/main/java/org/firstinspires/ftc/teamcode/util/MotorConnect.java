package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorConnect {

    public static DcMotor leftWheel, leftWheel2, rightWheel, rightWheel2, 
                    lift, arm;
    public double ticksPerRotationArm;
    public double liftTickPerRotation;
    public double driveMotorTickPerRotation;
    public int tgtPosition = 0;

    public MotorConnect (HardwareMap hmMap) {
        //map the motors from the config file to code
        //for all motors use .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftWheel = hmMap.get(DcMotor.class, "leftWheel");
        this.leftWheel2 = hmMap.get(DcMotor.class, "leftWheel2");
        this.rightWheel = hmMap.get(DcMotor.class, "rightWheel");
        this.rightWheel2 = hmMap.get(DcMotor.class, "rightWheel2");
        this.lift = hmMap.get(DcMotor.class, "lift");
        this.arm = hmMap.get(DcMotor.class, "arm");

        this.leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        this.leftWheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftWheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftWheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        this.rightWheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightWheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightWheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        this.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.lift.setDirection(DcMotorSimple.Direction.REVERSE);
        this.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //tick per rotation for arm motor
        this.ticksPerRotationArm = arm.getMotorType().getTicksPerRev();

        //tick per rotation for the lift motor
        this.liftTickPerRotation = lift.getMotorType().getTicksPerRev();

        //tick per rotation for the drive chain motor
        this.driveMotorTickPerRotation = this.leftWheel.getMotorType().getTicksPerRev();
    }

    //Set the motor speed for left and right wheels to go forward or backwards
    public void setWheelSpeed(double speed, double speed2) {
        this.leftWheel.setPower(speed);
        this.leftWheel2.setPower(speed);
        this.rightWheel.setPower(speed2);
        this.rightWheel2.setPower(speed2);
    }

    public void turnWheelSpeed(double left, double right) {
        this.leftWheel.setPower(left);
        this.leftWheel2.setPower(left);
        this.rightWheel.setPower(right);
        this.rightWheel2.setPower(right);
    }

    public void turnRight(double x) {
        this.leftWheel.setPower(x);
        this.leftWheel2.setPower(x);
        this.rightWheel.setPower(-x);
        this.rightWheel2.setPower(-x);
    }
    
    public void turnLeft(double x) {
        this.leftWheel.setPower(x);
        this.leftWheel2.setPower(x);
        this.rightWheel.setPower(-x);
        this.rightWheel2.setPower(-x);
    }
    
    //Raise the Lift
    public void raiseLift() {
        this.lift.setTargetPosition((int) (3.5 * this.liftTickPerRotation));
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setPower(0.8);
        /*The while loop is not really needed just make sure to not move the
        arm at the same time.*/
        // while(this.lift.isBusy());
        // this.lift.setPower(0);
        // this.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Lower the Lift
    public void lowerLift() {
        this.lift.setTargetPosition(0/*(int) -(3.5 * this.liftTickPerRotation)*/);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setPower(-0.8);
        /*The while looop is not really needed just make sure to not move the
        arm at the same time.*/
        // while(this.lift.isBusy());
        // this.lift.setPower(0);
        // this.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // the value of y has the - already applied
    public void armRaiser(double yPosi) {
        if(yPosi > 0) {
                this.tgtPosition += 20;
                arm.setTargetPosition(this.tgtPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.5);
                while(arm.isBusy());
        }else if(yPosi < 0) {
            if(this.tgtPosition != 0) {
                this.tgtPosition -= 20;
                arm.setTargetPosition(this.tgtPosition);
            } else{
                arm.setTargetPosition(this.tgtPosition);
            }
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(-0.5);
            while(arm.isBusy());
        }
    }
    
    public void armHigh() {
        this.tgtPosition = 720;
        arm.setTargetPosition(this.tgtPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
    }
    
    public void armLow() {
        this.tgtPosition = 10;
        arm.setTargetPosition(this.tgtPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(-0.5);
    }

    /*
        Gets number of rotations.
    */
    public double getArmRotation() {
        return this.arm.getCurrentPosition() / ticksPerRotationArm;
    }

}