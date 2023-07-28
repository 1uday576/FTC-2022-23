/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.ServoConnect;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


import java.util.ArrayList;

@Autonomous
public class RightSideAuton extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    static final double TICKS_COUNT = 537.7;
    static final double DIAMETER = 3.75;
    static final double COUNTS_PER_INCH = TICKS_COUNT / (DIAMETER * 3.14159);
    static final int ARM_TARGET = 740;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // using tags 1, 2, and 3 from the 36h11 family
    static final int LEFT = 1, MIDDLE = 2, RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    // creating an instance of MotorConnect
    //MotorConnect mc = new MotorConnect(hardwareMap);

    /*final int MOTOR_TICKS_COUNT = 0;
    final double CIRCUMFERENCE = 11.781;*/

    DcMotor leftWheel, leftWheel1, rightWheel, rightWheel1, arm;

    static int leftPos = 0, rightPos = 0, armPos = 0;

    ServoConnect sc;

    @Override
    public void runOpMode()
    {
        // getting the dc motors
        this.leftWheel = hardwareMap.get(DcMotor.class, "leftWheel");
        this.rightWheel = hardwareMap.get(DcMotor.class, "rightWheel");
        this.leftWheel1 = hardwareMap.get(DcMotor.class, "leftWheel2");
        this.rightWheel1 = hardwareMap.get(DcMotor.class, "rightWheel2");
        this.arm = hardwareMap.get(DcMotor.class, "arm");

        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftWheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int driveMotorTickPerRotation = (int) this.leftWheel.getMotorType().getTicksPerRev();

        // servo connect
        sc = new ServoConnect(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        waitForStart();
        long startTime = System.nanoTime();
        boolean tagFound = false;
        while (!tagFound) //!isStarted() && !isStopRequested())
        {

            long elapsedSeconds = (System.nanoTime() - startTime) / (long) 1e9;

            if (elapsedSeconds > 7) break;

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                //boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    String secs = elapsedSeconds == 1 ? " second" : " seconds";
                    String elapsed = "Elapsed time: " + elapsedSeconds + secs;
                    telemetry.addLine(elapsed);


                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {

                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);


        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        sleep(750);


        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Actually do something useful */

        sc.closeClaw();





        //
        // WIP
        /*

        TO-DO: lift arm, open claw, park

        740 TICKS TO LIFT ARM


         */

        // drive roughly 30 inches forward to score on medium junct
        driveToPos(32, 32, 0.5);

        // turn left to line up with the junct
        driveToPos(-10, 10, 0.5);
        sleep(500);
        //setArmPos(740/COUNTS_PER_INCH, 0.4);
        sleep(500);
        //driveToPos(3, 3, 0.5);

        driveToPos(-4, -4, 0.5);
        sleep(200);
        // lift the arm and open claw
        raiseArm();
        while (arm.isBusy());

        driveToPos(6, 6, 0.5);
        sleep(100);

        // drop cone
        sc.openClaw();
        driveToPos(-3, -3, 0.5);
        lowerArm();
        while (arm.isBusy());
        //forwardOneSquare();


        // parking code:
        if (tagOfInterest == null || tagOfInterest.id == MIDDLE) {
            telemetry.addLine("middle");
            driveToPos(3, 3, 0.5);
            driveToPos(-3, 3, 0.5);
            telemetry.update();

        }


        else if (tagOfInterest.id == LEFT) {
            driveToPos(-7, 7, 0.5);
            driveToPos(5, 5, 0.5);
            driveToPos(3, -3, 0.5);
            driveToPos(18, 18, 0.5);
        }
        // right
        else {
            telemetry.addLine("right");
            telemetry.update();
            // forward three inches

            driveToPos(-16, -16, 0.5);
            driveToPos(-5, 5, 0.5);
            driveToPos(-5, -5, 0.5);
        }


        telemetry.addData("exist", 0);
        telemetry.update();

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    /*void forwardOneSquare() {
        double rotationsNeeded = 23.5 / CIRCUMFERENCE;
        int encoderDrivingTarget = (int) rotationsNeeded * MOTOR_TICKS_COUNT;
        this.leftWheel.setTargetPosition(encoderDrivingTarget);
        this.rightWheel.setTargetPosition(encoderDrivingTarget);

        // Setting the motor power
        setWheelSpeed(0.5);

        // Set motors to RUN_TO_POSITION
        this.leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // do nothing while we wait for robot to move to correct position
        while (this.leftWheel.isBusy() || this.rightWheel.isBusy());

        // Stop all motion
        setWheelSpeed(0);
    }*/

    void driveToPos(double left, double right, double speed) {
        right = -right;
        leftPos = (int) (leftWheel.getCurrentPosition() + left * COUNTS_PER_INCH);
        rightPos = (int) (rightWheel.getCurrentPosition() + right * COUNTS_PER_INCH);

        leftWheel.setTargetPosition(leftPos);
        leftWheel1.setTargetPosition(leftPos);
        rightWheel.setTargetPosition(rightPos);
        rightWheel1.setTargetPosition(rightPos);

        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWheel1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftWheel.setPower(speed);
        leftWheel1.setPower(speed);
        rightWheel.setPower(speed);
        rightWheel1.setPower(speed);

        while (opModeIsActive() && leftWheel.isBusy() && leftWheel1.isBusy() && rightWheel.isBusy() && rightWheel1.isBusy()) {
            telemetry.addData("Left Target: ", leftPos);
            telemetry.addData("Right Target: ", rightPos);
            telemetry.addData("Left Wheel: ", leftWheel.getCurrentPosition());
            telemetry.addData("Right Wheel: ", rightWheel.getCurrentPosition());
            telemetry.update();
            idle();
        }

        leftWheel.setPower(0);
        leftWheel1.setPower(0);
        rightWheel.setPower(0);
        rightWheel1.setPower(0);

        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(50);
    }

    void raiseArm() {
        armPos = 740;
        arm.setTargetPosition(armPos);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.2);

    }

    void lowerArm() {
        armPos = 50;
        arm.setTargetPosition(0);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.2);
    }


    /*void setArmPos(double target, double speed) {
        armPos = (int) (arm.getCurrentPosition() + target * COUNTS_PER_INCH);

        arm.setTargetPosition(armPos);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setPower(speed);

        while (opModeIsActive() && arm.isBusy()) {
            telemetry.addData("Arm Target: ", armPos);
            telemetry.addData("Arm Position: ", arm.getCurrentPosition());
            idle();
        }

        //arm.setPower(0);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }*/



    /*void makeLeftTurn() {
        double rotationsNeeded = 13.548 / CIRCUMFERENCE;
        int encoderDrivingTarget = (int) rotationsNeeded * MOTOR_TICKS_COUNT;
        this.rightWheel.setTargetPosition(encoderDrivingTarget);
        this.leftWheel.setTargetPosition(-encoderDrivingTarget);

        setWheelSpeed(0.5);

        this.leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (this.leftWheel.isBusy() || this.rightWheel.isBusy());

        setWheelSpeed(0);

    }

    void makeRightTurn() {
        double rotationsNeeded = 13.548 / CIRCUMFERENCE;
        int encoderDrivingTarget = (int) rotationsNeeded * MOTOR_TICKS_COUNT;
        this.rightWheel.setTargetPosition(-encoderDrivingTarget);
        this.leftWheel.setTargetPosition(encoderDrivingTarget);

        setWheelSpeed(0.5);

        this.leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (this.leftWheel.isBusy() || this.rightWheel.isBusy());

        setWheelSpeed(0);

    }*/

    void backupForward() {
        setWheelSpeed(1);
        sleep(410);
        setWheelSpeed(0);
    }

    void backupTurnLeft() {
        turnWheelSpeed(-1, 1);
        sleep(220);
        setWheelSpeed(0);
    }

    void backupTurnRight () {
        turnWheelSpeed(1, -1);
        sleep(220);
        setWheelSpeed(0);
    }

    void turnWheelSpeed(double left, double right) {
        this.leftWheel.setPower(left);
        this.rightWheel.setPower(-right);
    }

    void setWheelSpeed(double x) {
        this.leftWheel.setPower(x);
        this.rightWheel.setPower(-x);
    }

}