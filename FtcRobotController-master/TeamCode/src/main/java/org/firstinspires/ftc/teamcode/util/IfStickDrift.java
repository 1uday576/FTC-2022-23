package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class IfStickDrift extends LinearOpMode{

    public void runOpMode() throws InterruptedException{
        telemetry.addData("Gamepad 1 Y stick ", gamepad1.left_stick_y);
        telemetry.addData("Gamepad 1 X stick ", gamepad1.right_stick_x);
        telemetry.addData("Gamepad 2 Y stick ", gamepad2.left_stick_y);
        telemetry.addData("Gamepad 2 X stick ", gamepad1.left_stick_x);
        telemetry.update();
    }
}