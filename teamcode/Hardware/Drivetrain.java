package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class Drivetrain extends Mechanism {

    static final double COUNTS_PER_MOTOR_REV = 560;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0 * 1.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    //declare motor variables
    //driver 1 motors
    public DcMotor LFM = null;
    public DcMotor RFM = null;
    public DcMotor LBM = null;
    public DcMotor RBM = null;

    public double LFMP = 0;
    public double RFMP = 0;
    public double LBMP = 0;
    public double RBMP = 0;

    private BNO055IMU imu; // The IMU sensor object
    Orientation lastAngles = new Orientation(); // State used for updating telemetry

    PIDController pidRotate;

    double globalAngle, minPower = 0.05, power = 0.50;

    boolean input_past = false;

    public Drivetrain() {
    }

    public Drivetrain(OpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hwMap) {
        //map drivetrain DcMotor variables to motors defined in REV Expansion Hub config
        LFM = hwMap.get(DcMotor.class, "LFM");
        RFM = hwMap.get(DcMotor.class, "RFM");
        LBM = hwMap.get(DcMotor.class, "LBM");
        RBM = hwMap.get(DcMotor.class, "RBM");

        encoderInit();

        //set the motors on the left side of the robot to reverse for intuitive code with flipped motors
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

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void fieldCentricDrive(double x, double y, double turn) {
        double vel = Math.hypot(x, y); //find the hypotenuse if the joystick was a unit circle
        double basetheta = Math.atan2(x, y); //find the inside angle of the triangle formed
        double realtheta = basetheta - getAngle(); //subtract the angle heading to create field-relative movement
        double vx = (vel * Math.sin(realtheta));
        double vy = (vel * Math.cos(realtheta));
        LFMP = vy + vx + turn;
        RFMP = vy - vx - turn;
        LBMP = vy - vx + turn;
        RBMP = vy + vx - turn;
        setDrive();
    }

    public void robotCentricDrive(double x, double y, double turn) {
        double vel = Math.hypot(x, y); //find the hypotenuse if the joystick was a unit circle
        double theta = Math.atan2(x, y); //find the inside angle of the triangle formed
        double vx = (vel * Math.sin(theta));
        double vy = (vel * Math.cos(theta));
        LFMP = vy + vx + turn;
        RFMP = vy - vx - turn;
        LBMP = vy - vx + turn;
        RBMP = vy + vx - turn;
        setDrive();
    }

    public void arcadeDrive(double l, double r, double turn) {
        LFMP = l + turn;
        RFMP = r - turn;
        LBMP = l - turn;
        RBMP = r + turn;
        setDrive();
    }

    public void minPowerTest(boolean input) {
        if (input && !input_past) {
            LFMP += 0.005;
            RFMP += 0.005;
            LBMP += 0.005;
            RBMP += 0.005;
            input_past = true;
        } else if (!input) {
            input_past = false;
        }
        setDrive();
    }

    private void setDrive() {
        List<Double> list = Arrays.asList(Math.abs(LFMP), Math.abs(RFMP), Math.abs(LBMP), Math.abs(RBMP));
        double max = Collections.max(list);
        if (max > 1) {
            LFMP /= max;
            RFMP /= max;
            LBMP /= max;
            RBMP /= max;
        }
        LFM.setPower(LFMP);
        RFM.setPower(RFMP);
        LBM.setPower(LBMP);
        RBM.setPower(RBMP);
    }

    public void encoderInit() {
        LFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void encoderDrive(double speed, double inches, boolean strafe) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;
        int lFPos = LFM.getCurrentPosition();
        int rFPos = RFM.getCurrentPosition();
        int lBPos = LBM.getCurrentPosition();
        int rBPos = RBM.getCurrentPosition();

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

        LFM.setTargetPosition(newLFTarget);
        RFM.setTargetPosition(newRFTarget);
        LBM.setTargetPosition(newLBTarget);
        RBM.setTargetPosition(newRBTarget);

        LFM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LFM.setPower(Math.abs(speed));
        RFM.setPower(Math.abs(speed));
        LBM.setPower(Math.abs(speed));
        RBM.setPower(Math.abs(speed));

        resetAngle();
            while (LFM.isBusy() && RFM.isBusy() && LBM.isBusy() && RBM.isBusy() && !strafe) {
                double error = 1.305 * getDeltaAngle();
                if (error > 0) {
                    LFM.setPower(speed - error);
                    RFM.setPower(speed + error);
                    LBM.setPower(speed - error);
                    RBM.setPower(speed + error);
                } else if (error < 0) {
                    LFM.setPower(speed + error);
                    RFM.setPower(speed - error);
                    LBM.setPower(speed + error);
                    RBM.setPower(speed - error);
                }
            }

        LFM.setPower(0);
        RFM.setPower(0);
        LBM.setPower(0);
        RBM.setPower(0);

        LFM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        delay(50);
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private double getDeltaAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        lastAngles = angles;

        return deltaAngle;
    }

    public void turn(int degrees, double power) {
        // restart imu angle tracking.
        resetAngle();
        pidRotate.reset();
        pidRotate.setSetpoint(getAngle() + degrees);
        pidRotate.setInputRange(-180, 180);
        pidRotate.setOutputRange(minPower, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();
        if (degrees > 0) {
            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                LFM.setPower(power);
                LBM.setPower(power);
                RFM.setPower(-power);
                RBM.setPower(-power);
                sendTelemetry();
            }
            while (!pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                LFM.setPower(-power);
                LBM.setPower(-power);
                RFM.setPower(power);
                RBM.setPower(power);
                sendTelemetry();
            }
            while (!pidRotate.onTarget());

        // turn the motors off.
        LFM.setPower(0);
        LBM.setPower(0);
        RFM.setPower(0);
        RBM.setPower(0);
        resetAngle();
    }

    private void delay(int mS) {
        try {
            Thread.sleep(mS);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void sendTelemetry() {
        opMode.telemetry.addData("LBM Pow", LBM.getPower());
        opMode.telemetry.addData("LFM Pow", LFM.getPower());
        opMode.telemetry.addData("RBM Pow", RBM.getPower());
        opMode.telemetry.addData("RFM Pow", RFM.getPower());

        /*
        opMode.telemetry.addData("LBM Pos", LBM.getCurrentPosition());
        opMode.telemetry.addData("LFM Pos", LFM.getCurrentPosition());
        opMode.telemetry.addData("RBM Pos", RBM.getCurrentPosition());
        opMode.telemetry.addData("RFM Pos", RFM.getCurrentPosition());

        opMode.telemetry.addData("Global Angle", globalAngle);
        opMode.telemetry.addData("Heading", lastAngles.firstAngle);
         */


    }
}
