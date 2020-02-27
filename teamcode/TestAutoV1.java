/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "TestAutoV1", group = "Auto")
public class TestAutoV1 extends LinearOpMode {

    HardwareInit robot = new HardwareInit();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 560;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    //declare motor power variables and set to 0
    private double LFMP = 0;
    private double RFMP = 0;
    private double LBMP = 0;
    private double RBMP = 0;
    private double LIP = 0;
    private double RIP = 0;

    private double error = 0;
    private double startAngleG;
    private double startAngle;
    private double currentAngle;

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f / 8f;
    private static float rectWidth = 1.5f / 8f;

    private static float offsetX = 0f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f / 8f + offsetX, 4f / 8f + offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f / 8f + offsetX, 4f / 8f + offsetY};
    private static float[] rightPos = {6f / 8f + offsetX, 4f / 8f + offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    private final double kP = 1.305;

    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() {
        //code to be run after hitting init
        telemetry.update();
        TelemetryS("Init", "init started");
        robot.initRobot(hardwareMap);
        initCV();

        runtime.reset();

        if (opModeIsActive()) {
            TelemetryS("test", "started");
            telemetry.update();
            if (valLeft == 0) {
                left();
            } else if (valMid == 0) {
                mid();
            } else if (valRight == 0) {
                right();
            } else {
                left();
            }
            TelemetryS("test", "finished");
            telemetry.update();
        }
    }

    //init LFMo - 0.69, RFMo - 0.35
    //skystone LFMo - 0.15, RFMo - 0.90
    //foundation LFMo - 0.09, RFMo - 0.97

    private void left() {
        TelemetryS("Skystone detected", "Running case 1 and 4");
        telemetry.update();
        //bring 1st stone under bridge - 10 pts
        encoderDrive(0.7, -33.195, 10, true);  //strafe left up to stones
        gyroDrive(startAngleG);                                           //correct angle
        encoderDrive(0.7, -2.3-6, 10, false);    //move backwards to line up RFMo with stone 4
        servoPos(robot.RFMo, 0.90, 1750);                       //lower RFMo onto stone 5
        encoderDrive(0.7, 11.65, 10, true);     //strafe right to line up with skybridge
        gyroDrive(startAngleG);                                           //correct angle
        encoderDrive(0.7, 55.1+6, 10, false);    //move forward to other side of skybridge
        gyroDrive(startAngleG);                                           //correct angle
        servoPos(robot.RFMo, 0.35, 1750);                       //raise RFMo

        //bring 2nd stone under bridge - 10 pts
        encoderDrive(0.7, -68-6, 10, false);   //move back to back to stone 2
        gyroDrive(startAngleG);                                           //correct angle
        encoderDrive(0.7, -12.2, 10, true);    //strafe left up to stones
        gyroDrive(startAngleG);                                           //correct angle
        servoPos(robot.LFMo, 0.15, 1750);                       //lower LFMo onto stone 1
        encoderDrive(0.7, 12.315, 10, true);  //strafe right to line up with skybridge
        gyroDrive(startAngleG);                                           //correct angle
        encoderDrive(0.7, 70+6, 10, false);    //move forward to other side of skybridge
        servoPos(robot.LFMo, 0.69, 1750);                       //raise LFMo

        //park - 5 pts
        encoderDrive(0.7, -4, 10, true);   //strafe left away form alliance partner
        encoderDrive(0.7, -18, 10, false);   //move backwards to park
    }

    private void mid() {
        TelemetryS("Skystone detected", "Running case 2 and 5");
        telemetry.update();
        //bring 1st stone under bridge - 10 pts
        encoderDrive(0.7, -33.15, 10, true);  //strafe left up to stones
        gyroDrive(startAngleG);                                           //correct angle
        encoderDrive(0.7, -2, 10, false);    //move backwards to line up RFMo with stone 5
        servoPos(robot.RFMo, 0.90, 1750);                       //lower RFMo onto stone 5
        encoderDrive(0.7, 11.5, 10, true);     //strafe right to line up with skybridge
        gyroDrive(startAngleG);                                           //correct angle
        encoderDrive(0.7, 54, 10, false);    //move forward to other side of skybridge
        gyroDrive(startAngleG);                                           //correct angle
        servoPos(robot.RFMo, 0.35, 1750);                       //raise RFMo

        //bring 2nd stone under bridge - 10 pts
        encoderDrive(0.7, -68, 10, false);   //move back to back to stone 2
        gyroDrive(startAngleG);                                           //correct angle
        encoderDrive(0.7, -12.3, 10, true);    //strafe left up to stones
        gyroDrive(startAngleG);                                           //correct angle
        servoPos(robot.LFMo, 0.15, 1750);                       //lower LFMo onto stone 2
        encoderDrive(0.7, 12.5, 10, true);  //strafe right to line up with skybridge
        gyroDrive(startAngleG);                                           //correct angle
        encoderDrive(0.7, 70, 10, false);    //move forward to other side of skybridge
        servoPos(robot.LFMo, 0.69, 1750);                       //raise LFMo

        //park - 5 pts
        encoderDrive(0.7, -4, 10, true);   //strafe left away form alliance partner
        encoderDrive(0.7, -18, 10, false);   //move backwards to park
    }

    private void right() {
        TelemetryS("Skystone detected", "Running case 3 and 6");
        telemetry.update();
        //bring 1st stone under bridge - 10 pts
        encoderDrive(0.7, -33.195, 10, true);  //strafe left up to stones
        gyroDrive(startAngleG);                                           //correct angle
        encoderDrive(0.7, -1.7+6, 10, false);    //move backwards to line up RFMo with stone 5
        servoPos(robot.RFMo, 0.90, 1750);                       //lower RFMo onto stone 5
        encoderDrive(0.7, 11.5, 10, true);     //strafe right to line up with skybridge
        gyroDrive(startAngleG);                                           //correct angle
        encoderDrive(0.7, 54-6, 10, false);    //move forward to other side of skybridge
        gyroDrive(startAngleG);                                           //correct angle
        servoPos(robot.RFMo, 0.35, 1750);                       //raise RFMo

        //bring 2nd stone under bridge - 10 pts
        encoderDrive(0.7, -67+6, 10, false);   //move back to back to stone 2
        gyroDrive(startAngleG);                                           //correct angle
        encoderDrive(0.7, -12.315, 10, true);    //strafe left up to stones
        gyroDrive(startAngleG);                                           //correct angle
        servoPos(robot.LFMo, 0.15, 1750);                       //lower LFMo onto stone 2
        encoderDrive(0.7, 12.315, 10, true);  //strafe right to line up with skybridge
        gyroDrive(startAngleG);                                           //correct angle
        encoderDrive(0.7, 70-6, 10, false);    //move forward to other side of skybridge
        servoPos(robot.LFMo, 0.69, 1750);                       //raise LFMo

        //park - 5 pts
        encoderDrive(0.7, -4, 10, true);   //strafe left away form alliance partner
        encoderDrive(0.7, -18, 10, false);   //move backwards to park
    }

    private void encoderDrive(double speed, double inches, double timeoutS, boolean strafe) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;
        int lFPos = robot.LFM.getCurrentPosition();
        int rFPos = robot.RFM.getCurrentPosition();
        int lBPos = robot.LBM.getCurrentPosition();
        int rBPos = robot.RBM.getCurrentPosition();

        if (opModeIsActive()) {
            if (strafe) {
                newLFTarget = lFPos + (int) (inches * COUNTS_PER_INCH);
                newRFTarget = rFPos - (int) (inches * COUNTS_PER_INCH);
                newLBTarget = lBPos - (int) (inches * COUNTS_PER_INCH);
                newRBTarget = rBPos + (int) (inches * COUNTS_PER_INCH);
            } else {
                newLFTarget = lFPos + (int) (inches * COUNTS_PER_INCH);
                newRFTarget = rFPos + (int) (inches * COUNTS_PER_INCH);
                newLBTarget = lBPos + (int) (inches * COUNTS_PER_INCH);
                newRBTarget = rBPos + (int) (inches * COUNTS_PER_INCH);
            }

            robot.LFM.setTargetPosition(newLFTarget);
            robot.RFM.setTargetPosition(newRFTarget);
            robot.LBM.setTargetPosition(newLBTarget);
            robot.RBM.setTargetPosition(newRBTarget);

            robot.LFM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RFM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.LBM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RBM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.LFM.setPower(Math.abs(speed));
            robot.RFM.setPower(Math.abs(speed));
            robot.LBM.setPower(Math.abs(speed));
            robot.RBM.setPower(Math.abs(speed));

            startAngle = robot.angles.firstAngle;

            while (opModeIsActive() &&
                    runtime.seconds() < timeoutS &&
                    robot.LFM.isBusy() &&
                    robot.RFM.isBusy() &&
                    robot.LBM.isBusy() &&
                    robot.RBM.isBusy()) {
                getImu();
                currentAngle = robot.angles.firstAngle;
                error = 1.305 * (startAngle - currentAngle);
                if (inches > 0) {
                    robot.LFM.setPower(speed - error);
                    robot.RFM.setPower(speed + error);
                    robot.LBM.setPower(speed - error);
                    robot.RBM.setPower(speed + error);
                } else if (inches < 0) {
                    robot.LFM.setPower(speed + error);
                    robot.RFM.setPower(speed - error);
                    robot.LBM.setPower(speed + error);
                    robot.RBM.setPower(speed - error);
                }
                TelemetryD("LFM Current Pos", robot.LFM.getCurrentPosition());
                TelemetryD("RFM Current Pos", robot.RFM.getCurrentPosition());
                TelemetryD("LBM Current Pos", robot.LBM.getCurrentPosition());
                TelemetryD("RBM Current Pos", robot.RBM.getCurrentPosition());
                TelemetryD("LFM Current Power", robot.LFM.getPower());
                TelemetryD("RFM Current Power", robot.RFM.getPower());
                TelemetryD("LBM Current Power", robot.LBM.getPower());
                TelemetryD("RBM Current Power", robot.RBM.getPower());
                TelemetryD("Start Angle", startAngle);
                TelemetryD("Current Angle", currentAngle);
                TelemetryD("error", error);
                telemetry.update();
            }

            robot.LFM.setPower(0);
            robot.RFM.setPower(0);
            robot.LBM.setPower(0);
            robot.RBM.setPower(0);

            robot.LFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.LBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);
        }
    }

    private void gyroDrive(double targetAngle) {
        //+ is cc-clockwise
        //- is clockwise
        boolean finished = false;
        while (!finished) {
            getImu();
            currentAngle = robot.angles.firstAngle;
            error = 0.5269 * (targetAngle - currentAngle);
            robot.LFM.setPower(-error);
            robot.RFM.setPower(error);
            robot.LBM.setPower(-error);
            robot.RBM.setPower(error);
            TelemetryD("targetAngle", targetAngle);
            TelemetryD("currentAngle", currentAngle);
            TelemetryD("error", error);
            TelemetryD("targetAngle - currentAngle", targetAngle - currentAngle);
            TelemetryB("finished", finished);

            TelemetryD("LFM Current Power", robot.LFM.getPower());
            TelemetryD("RFM Current Power", robot.RFM.getPower());
            TelemetryD("LBM Current Power", robot.LBM.getPower());
            TelemetryD("RBM Current Power", robot.RBM.getPower());

            telemetry.update();
            if (Math.abs(targetAngle - currentAngle) < 0.04) {
                finished = true;
            }
        }
        sleep(100);
    }

    private void timeDrive(double speed, long timeS) {
        timeS *= 1000;
        robot.LFM.setPower(speed);
        robot.RFM.setPower(speed);
        robot.LBM.setPower(speed);
        robot.RBM.setPower(speed);
        sleep(timeS);
    }

    /*
    private void diagonalStrafe(double speed, double inches, double timeoutS, boolean right, boolean forward) {
        TelemetryS("encoderDrive", "Encoder Drive");
        telemetry.update();

        int lFPos = LFM.getCurrentPosition();
        int rFPos = RFM.getCurrentPosition();
        int lBPos = LBM.getCurrentPosition();
        int rBPos = RBM.getCurrentPosition();
        int newLFTarget = lFPos;
        int newRFTarget = rFPos;
        int newLBTarget = lBPos;
        int newRBTarget = rBPos;

        if (opModeIsActive()) {
            if (right) {
                if (forward) {
                    newLFTarget = lFPos + (int) (inches * COUNTS_PER_INCH);
                    newRBTarget = rBPos + (int) (inches * COUNTS_PER_INCH);
                } else if (!forward) {
                    newRFTarget = lFPos + (int) (inches * COUNTS_PER_INCH);
                    newLBTarget = rBPos + (int) (inches * COUNTS_PER_INCH);
                }
            } else if (!right) {
                if (forward) {
                    newRFTarget = lFPos + (int) (inches * COUNTS_PER_INCH);
                    newLBTarget = rBPos + (int) (inches * COUNTS_PER_INCH);
                } else if (!forward) {
                    newLFTarget = lFPos + (int) (inches * COUNTS_PER_INCH);
                    newRBTarget = rBPos + (int) (inches * COUNTS_PER_INCH);
                }
            }

            TelemetryD("speed", speed);
            TelemetryD("inches", inches);
            TelemetryD("timeoutS", timeoutS);
            TelemetryD("newLFTarget", newLFTarget);
            TelemetryD("newRFTarget", newRFTarget);
            TelemetryD("newLBTarget", newLBTarget);
            TelemetryD("newRBTarget", newRBTarget);


            LFM.setTargetPosition(newLFTarget);
            RFM.setTargetPosition(newRFTarget);
            LBM.setTargetPosition(newLBTarget);
            RBM.setTargetPosition(newRBTarget);

            LFM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            LFM.setPower(Math.abs(speed));
            RFM.setPower(Math.abs(speed));
            LBM.setPower(Math.abs(speed));
            RBM.setPower(Math.abs(speed));

            startAngle = angles.firstAngle;

            while (opModeIsActive() &&
                    runtime.seconds() < timeoutS &&
                    LFM.isBusy() &&
                    RFM.isBusy() &&
                    LBM.isBusy() &&
                    RBM.isBusy()) {
                getImu();
                currentAngle = angles.firstAngle;
                error = 0.005 * (startAngle - currentAngle);
                LFM.setPower(speed + error);
                RFM.setPower(speed - error);
                LBM.setPower(speed + error);
                RBM.setPower(speed - error);
                TelemetryD("LFM Current Pos", LFM.getCurrentPosition());
                TelemetryD("RFM Current Pos", RFM.getCurrentPosition());
                TelemetryD("LBM Current Pos", LBM.getCurrentPosition());
                TelemetryD("RBM Current Pos", RBM.getCurrentPosition());
                TelemetryD("LFM Current Power", LFM.getPower());
                TelemetryD("RFM Current Power", RFM.getPower());
                TelemetryD("LBM Current Power", LBM.getPower());
                TelemetryD("RBM Current Power", RBM.getPower());
                TelemetryD("Start Angle", startAngle);
                TelemetryD("Current Angle", startAngle);
                telemetry.update();
            }

            LFM.setPower(0);
            RFM.setPower(0);
            LBM.setPower(0);
            RBM.setPower(0);

            LFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);
        }
    }

     */


    private void servoPos(Servo servo, double position, long timeMs) {
        //init LFMo - 0.69, RFMo - 0.31
        //skystone LFMo - 0.15, RFMo - 0.84
        //foundation LFMo - 0.09, RFMo - 0.93
        servo.setPosition(position);
        sleep(timeMs);
    }

    private void getImu() {
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }

    private void initCV() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        TelemetryS("Init", "vision initialized");
        telemetry.update();

        while (!(isStarted() || isStopRequested())) {
            telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();
            sleep(100);
        }
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int) (input.rows() * midPos[1]), (int) (input.cols() * midPos[0]));//gets value at circle
            valMid = (int) pixMid[0];

            double[] pixLeft = thresholdMat.get((int) (input.rows() * leftPos[1]), (int) (input.cols() * leftPos[0]));//gets value at circle
            valLeft = (int) pixLeft[0];

            double[] pixRight = thresholdMat.get((int) (input.rows() * rightPos[1]), (int) (input.cols() * rightPos[0]));//gets value at circle
            valRight = (int) pixRight[0];

            //create three points
            Point pointMid = new Point((int) (input.cols() * midPos[0]), (int) (input.rows() * midPos[1]));
            Point pointLeft = new Point((int) (input.cols() * leftPos[0]), (int) (input.rows() * leftPos[1]));
            Point pointRight = new Point((int) (input.cols() * rightPos[0]), (int) (input.rows() * rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointLeft, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointRight, 5, new Scalar(255, 0, 0), 1);//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols() * (leftPos[0] - rectWidth / 2),
                            input.rows() * (leftPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (leftPos[0] + rectWidth / 2),
                            input.rows() * (leftPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols() * (midPos[0] - rectWidth / 2),
                            input.rows() * (midPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (midPos[0] + rectWidth / 2),
                            input.rows() * (midPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols() * (rightPos[0] - rectWidth / 2),
                            input.rows() * (rightPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (rightPos[0] + rectWidth / 2),
                            input.rows() * (rightPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport) {
                case THRESHOLD: {
                    return thresholdMat;
                }

                case detection: {
                    return all;
                }

                case RAW_IMAGE: {
                    return input;
                }

                default: {
                    return input;
                }
            }
        }

    }

    private void TelemetryS(String caption, String message) {
        telemetry.addLine(caption + ": " + message);
    }

    private void TelemetryD(String caption, double value) {
        telemetry.addLine(caption + ": " + value);
    }

    private void TelemetryB(String caption, boolean tf) {
        telemetry.addLine(caption + ": " + tf);
    }

    private void TelemetryI(String caption, int value) {
        telemetry.addLine(caption + ": " + value);
    }
}
