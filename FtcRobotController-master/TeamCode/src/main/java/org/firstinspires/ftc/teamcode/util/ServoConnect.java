package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ServoConnect {

    public Servo claw;
    
    //Get the servo
    public ServoConnect (HardwareMap hwMap){
        claw = hwMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.FORWARD);
        claw.scaleRange(0.0, 1.0);
    }
    
    /*
        position is a fraction value between 0.0 and 1.0
        where 0.0 is 0 degrees and 1.0 is a full 180 
        degree turn.
        
        The servo is set to maxium to 180 degrees in 
        build config on the driver hub.
    */
    public void openClaw() {
        claw.setPosition(0.0);
    }
    
    public void closeClaw(){
        claw.setPosition(0.4);
    }
    
    public void setClaw(double p){
        claw.setPosition(p);
    }
    
}