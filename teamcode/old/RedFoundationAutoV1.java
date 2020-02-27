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

    package org.firstinspires.ftc.teamcode.old;

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

    @Autonomous(name = "RedFoundationAutoV1", group = "Foundation")
    public class RedFoundationAutoV1 extends LinearOpMode {

        private ElapsedTime runtime = new ElapsedTime();

        static final double COUNTS_PER_MOTOR_REV = 560;    // eg: TETRIX Motor Encoder
        static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);

        // State used for updating telemetry
        public Orientation angles;
        // The IMU sensor object
        BNO055IMU imu;

        //initialize motors and set to null
        private DcMotor LFM = null;
        private DcMotor RFM = null;
        private DcMotor LBM = null;
        private DcMotor RBM = null;
        private DcMotor LI = null;
        private DcMotor RI = null;
        //declare motor power variables and set to 0

        private double LFMP = 0;
        private double RFMP = 0;
        private double LBMP = 0;
        private double RBMP = 0;
        private double LIP = 0;
        private double RIP = 0;

        private Servo LFMo = null;
        private Servo RFMo = null;

        private double error = 0;
        private double startAngle;
        private double currentAngle;


        @Override
        public void runOpMode() {
            //code to be run after hitting init
            TelemetryS("Init", "init started");
            initDrive();
            TelemetryS("Init", "drive initialized");
            initIntake();
            TelemetryS("Init", "intake initialized");
            initImu();
            TelemetryS("Init", "imu initialized");
            initServo();
            TelemetryS("Init", "servos initialized");

            waitForStart();

            if (opModeIsActive()) {
                TelemetryS("test", "started");
                getImu();
                encoderDrive(0.5, -35, 10, true);
                encoderDrive(0.5, 6, 10, false);
                while(LFMo.getPosition() != 0 && RFMo.getPosition() != 0) {
                    LFMo.setPosition(LFMo.getPosition()-0.1);
                    RFMo.setPosition(RFMo.getPosition()-0.1);
                }
                encoderDrive(0.5, 18, 10, true);
                gyroDrive(0.5, angles.firstAngle + 90, false);
                encoderDrive(0.5, -12, 10, true);
                LFMo.setPosition(0.6);
                RFMo.setPosition(0.4);
                sleep(2000);
                encoderDrive(0.5, 60, 10, true);
            }
        }

        private void encoderDrive(double speed, double inches, double timeoutS, boolean strafe) {

            TelemetryS("encoderDrive", "Encoder Drive");

            int newLFTarget;
            int newRFTarget;
            int newLBTarget;
            int newRBTarget;
            int lFPos = LFM.getCurrentPosition();
            int rFPos = RFM.getCurrentPosition();
            int lBPos = LBM.getCurrentPosition();
            int rBPos = RBM.getCurrentPosition();

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

                while (opModeIsActive() && runtime.seconds() < timeoutS && (LFM.isBusy() && RFM.isBusy() && LBM.isBusy() && RBM.isBusy())) {
                    getImu();
                    currentAngle = angles.firstAngle;
                    error = 0.0025 * (startAngle - currentAngle);
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

                sleep(500);
            }
        }

        private void timeDrive(double speed, long timeS) {
            timeS *= 1000;
            LFM.setPower(speed);
            RFM.setPower(speed);
            LBM.setPower(speed);
            RBM.setPower(speed);
            sleep(timeS);
        }

        private void gyroDrive(double speed, double targetAngle, boolean spin) {
            boolean finished = false;
            while (!finished) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentAngle = angles.firstAngle;
                if (spin) {
                    error = 0.003 * (targetAngle - currentAngle);
                } else {
                    error = 0.001 * (targetAngle - currentAngle);
                }
                LFM.setPower(speed + error);
                RFM.setPower(speed - error);
                LBM.setPower(speed + error);
                RBM.setPower(speed - error);
                TelemetryD("speed", speed);
                TelemetryD("targetAngle", targetAngle);
                TelemetryD("currentAngle", currentAngle);
                TelemetryD("error", error);
                TelemetryB("finished", finished);

                TelemetryD("LFM Current Power", LFM.getPower());
                TelemetryD("RFM Current Power", RFM.getPower());
                TelemetryD("LBM Current Power", LBM.getPower());
                TelemetryD("RBM Current Power", RBM.getPower());

                telemetry.update();
                if (Math.abs(targetAngle - currentAngle) < 5) {
                    finished = true;
                }
            }
        }

        private void initServo() {
            LFMo = hardwareMap.get(Servo.class, "LFMo");
            RFMo = hardwareMap.get(Servo.class, "RFMo");
        }

        private void servoPos(Servo servo, double position) {
            servo.setPosition(position);
        }

        private void initDrive() {
            //define DcMotor variables to motors defined in REV Expansion Hub config
            LFM = hardwareMap.get(DcMotor.class, "LFM");
            RFM = hardwareMap.get(DcMotor.class, "RFM");
            LBM = hardwareMap.get(DcMotor.class, "LBM");
            RBM = hardwareMap.get(DcMotor.class, "RBM");

            //set the motors on the right side of the robot to reverse for intuitive code with flipped motors
            LFM.setDirection(DcMotor.Direction.REVERSE);
            RFM.setDirection(DcMotor.Direction.FORWARD);
            LBM.setDirection(DcMotor.Direction.REVERSE);
            RBM.setDirection(DcMotor.Direction.FORWARD);

            //set the motors to stop instead of coast on 0 power
            LFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            LFM.setPower(0);
            RFM.setPower(0);
            LBM.setPower(0);
            RBM.setPower(0);
        }

        private void initIntake() {
            RI = hardwareMap.get(DcMotor.class, "RI");
            LI = hardwareMap.get(DcMotor.class, "LI");

            LI.setDirection(DcMotor.Direction.REVERSE);
            RI.setDirection(DcMotor.Direction.FORWARD);
        }

        private void initImu() {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            imu.startAccelerationIntegration(new Position(), new Velocity(), 800);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }

        private void getImu() {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        private void TelemetryS(String caption, String message) {
            telemetry.addLine(caption + ": " + message);
        }
        private void TelemetryD(String caption, double value) {
            telemetry.addLine(caption + ": " + value);
        }
        private void TelemetryB(String caption, boolean tf) {
            telemetry.addLine(caption + ": " + String.valueOf(tf));
        }
        private void TelemetryI(String caption, int value) {
            telemetry.addLine(caption + ": " + value);
        }
    }
