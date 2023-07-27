package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import org.firstinspires.ftc.teamcode.util.MotorConnect;
import org.firstinspires.ftc.teamcode.util.ServoConnect;

/**
 * DriveOp is the file used by RGB to control their robot via 2 logitech game controllers)
 * 
 * @author Uday K
 */
public class DriveOpInver extends LinearOpMode{

    private Gamepad previousGamepad2 = new Gamepad();
   
    public void runOpMode() throws InterruptedException {
        MotorConnect m = new MotorConnect(hardwareMap);
        ServoConnect s = new ServoConnect(hardwareMap);
    
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Waiting for the game to start
        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();
        
        double tgtPower = 0;
        double xValue;
        while (opModeIsActive()) {
           
                /*
                    The gamepad1 "left stick" is used for froward and back.
                    The gamepad1 "right stick" is used for right and left.
                    The gampad1 "a" is use for opening claw and "b" for 
                    clossing the claw.
                */
                
                //To make the robot go forward or backwards
                if(this.gamepad1.left_stick_y != 0) {
                    tgtPower =  -this.gamepad1.left_stick_y;
                    m.setWheelSpeed(tgtPower / 2, tgtPower / 2);
                }else {
                    xValue = this.gamepad1.right_stick_x;
                    if(xValue > 0) {
                        //If positive x go right
                        m.turnRight(xValue / 2);
                    } 
                    else if (xValue < 0) {
                        //If negative x go left
                        m.turnLeft(xValue / 2);
                    } else{
                        m.setWheelSpeed(0, 0);
                    }
                }
                
                //m.setWheelSpeed((this.gamepad1.right_stick_y + this.gamepad1.right_stick_x) * 0.5, (this.gamepad1.right_stick_y - this.gamepad1.right_stick_x) * 0.5);
                //Closing and Opeing both the claw and the lift
                if (gamepad2.a /*&& !this.previousGamepad2.a*/){
                    s.closeClaw();
                    // telemetry.addData("position", s.claw.getPosition());
                    // telemetry.update();
                }else if (gamepad2.b /*&& !this.previousGamepad2.b*/) {
                    s.openClaw();
                    // telemetry.addData("position", s.claw.getPosition());
                    // telemetry.update();
                }else if (gamepad2.x /*&& !this.previousGamepad2.x*/) {
                    // telemetry.addData("enter", 0);
                    // telemetry.update();
                    m.raiseLift();
                }else if (gamepad2.y /*&& !this.previousGamepad2.y*/) {
                    // telemetry.addData("enter", 0);
                    // telemetry.update();
                    m.lowerLift();
                    // telemetry.addData("exist", 0);
                    // telemetry.update();
                }else if (gamepad2.dpad_up) {
                    m.armHigh();
                }else if (gamepad2.dpad_down) {
                    m.armLow();
                }
                
                telemetry.addData("Target Position: ", m.lift.getTargetPosition());
                telemetry.update();
                
                double yPosit = -gamepad2.left_stick_y;
                m.armRaiser(yPosit);
                
                // if(gamepad2.dpad_up) {
                //     m.armHigh();
                // } else if (gamepad2.dpad_down) {
                //     m.armLow();
                // }
                // get the previous gampad state
                this.previousGamepad2.copy(this.gamepad2);
            }
        }
}
    
