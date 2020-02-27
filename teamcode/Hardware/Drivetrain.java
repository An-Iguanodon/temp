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

public class Drivetrain extends Mechanism {

    static final double COUNTS_PER_MOTOR_REV = 560;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
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

    double globalAngle, power = 0.50;

    PIDController pidRotate, pidDrive;

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

        pidRotate = new PIDController(0, 0, 0);

        pidDrive = new PIDController(0, 0, 0);
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.enable();
    }

    public void fieldCentricDrive(double x, double y, double gyro, double turn) {
        double vel = Math.hypot(x, y); //find the hypotenuse if the joystick was a unit circle
        double basetheta = Math.atan2(x, y); //find the inside angle of the triangle formed
        double realtheta = basetheta - gyro; //subtract the angle heading to create field-relative movement
        double vx = (vel * Math.sin(realtheta));
        double vy = (vel * Math.cos(realtheta));
        LFMP = vy + vx + turn;
        RFMP = vy - vx - turn;
        LBMP = vy - vx + turn;
        RBMP = vy + vx - turn;
    }

    public void robotCentricDrive(double x, double y, double turn) {
        double vel = Math.hypot(x, y); //find the hypotenuse if the joystick was a unit circle
        double basetheta = Math.atan2(x, y); //find the inside angle of the triangle formed
        double realtheta = basetheta;
        double vx = (vel * Math.sin(realtheta));
        double vy = (vel * Math.cos(realtheta));
        LFMP = vy + vx + turn;
        RFMP = vy - vx - turn;
        LBMP = vy - vx + turn;
        RBMP = vy + vx - turn;
    }

    public void arcadeDrive(double l, double r, double turn) {
        LFMP = l + turn;
        RFMP = r - turn;
        LBMP = l - turn;
        RBMP = r + turn;
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

    public double getEncoderPos(){
        return LFM.getCurrentPosition();
    }

    public void driveToPos(int inches, double power) {
        pidDrive.reset();
        pidDrive.setSetpoint(getEncoderPos() + inchesToTicks(inches));
        pidDrive.setInputRange(getEncoderPos(), pidDrive.getSetpoint());
        pidDrive.setOutputRange(.05, power);
        pidDrive.setTolerance(20);
        pidDrive.enable();
        if (inches < 0) {
            while (getEncoderPos() == 0) {
                LFM.setPower(power);
                LBM.setPower(power);
                RFM.setPower(power);
                RBM.setPower(power);
            }
            do {
                power = pidDrive.performPID(getAngle());
                LFM.setPower(power);
                LBM.setPower(power);
                RFM.setPower(power);
                RBM.setPower(power);
            }
            while (!pidDrive.onTarget());
        } else
            do {
                power = pidDrive.performPID(getEncoderPos());
                LFM.setPower(-power);
                LBM.setPower(-power);
                RFM.setPower(-power);
                RBM.setPower(-power);
            }
            while (!pidDrive.onTarget());

        // turn the motors off.
        LFM.setPower(0);
        LBM.setPower(0);
        RFM.setPower(0);
        RBM.setPower(0);
    }

    private double inchesToTicks(int inches) {
        return inches*COUNTS_PER_INCH;
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double getAngle() {
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

    public void turn(int degrees, double power) {
        // restart imu angle tracking.
        resetAngle();
        pidRotate.reset();
        pidRotate.setContinuous();
        pidRotate.setSetpoint(getAngle() + degrees);
        pidRotate.setInputRange(-180, 180);
        pidRotate.setOutputRange(.05, power);
        pidRotate.setTolerance(0.5);
        pidRotate.enable();
        if (degrees < 0) {
            while (getAngle() == 0) {
                LFM.setPower(-power);
                LBM.setPower(-power);
                RFM.setPower(power);
                RBM.setPower(power);
            }
            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                LFM.setPower(power);
                LBM.setPower(power);
                RFM.setPower(-power);
                RBM.setPower(-power);
            }
            while (!pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                LFM.setPower(power);
                LBM.setPower(power);
                RFM.setPower(-power);
                RBM.setPower(-power);
            }
            while (!pidRotate.onTarget());

        // turn the motors off.
        LFM.setPower(0);
        LBM.setPower(0);
        RFM.setPower(0);
        RBM.setPower(0);
        resetAngle();
    }

    /*
    PT = P0 + V0T + 1/2A0T2 + 1/6JT3
    VT = V0 + A0T + 1/2 JT2
    AT = A0 + JT

    Discrete time form:

    PT = PT + VT +1/2AT + 1/6J
    VT = VT+AT + 1/2JT
    AT = AT+JT
    where
    P0, V0, and A0 are the starting position, velocity, and accelerations
    PT , VT, and AT are the position, velocity, and acceleration at time T
    J is the profile jerk (time rate of change of acceleration)
    */

    public void sendTelemetry() {
        opMode.telemetry.addData("LBM Pow", LBM.getPower());
        opMode.telemetry.addData("LFM Pow", LFM.getPower());
        opMode.telemetry.addData("RBM Pow", RBM.getPower());
        opMode.telemetry.addData("RFM Pow", RFM.getPower());

        opMode.telemetry.addData("LBM Pos", LBM.getCurrentPosition());
        opMode.telemetry.addData("LFM Pos", LFM.getCurrentPosition());
        opMode.telemetry.addData("RBM Pos", RBM.getCurrentPosition());
        opMode.telemetry.addData("RFM Pos", RFM.getCurrentPosition());

        opMode.telemetry.addData("Global Angle", globalAngle);
    }
}
