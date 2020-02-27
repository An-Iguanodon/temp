package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class HardwareInit {

    HardwareMap hwMap;
    //declare motor variables
    //driver 1 motors
    public DcMotor LFM = null;
    public DcMotor RFM = null;
    public DcMotor LBM = null;
    public DcMotor RBM = null;

    //driver 2 motors
    public DcMotor LI = null;
    public DcMotor RI = null;
    public DcMotor B = null;

    //declare servos
    //servos
    public Servo LFMo = null;
    public Servo RFMo = null;
    public Servo Cap = null;
    public Servo BRot = null;
    public Servo BClamp = null;

    public Orientation angles; // State used for updating telemetry
    BNO055IMU imu; // The IMU sensor object

    public HardwareInit(){

    }

    public void initRobot(HardwareMap ahwMap) {
        hwMap = ahwMap;
        initImu();
        initDrive();
        initIntake();
        initOutput();
    }

    private void initDrive() {
        //map drivetrain DcMotor variables to motors defined in REV Expansion Hub config
        LFM = hwMap.get(DcMotor.class, "LFM");
        RFM = hwMap.get(DcMotor.class, "RFM");
        LBM = hwMap.get(DcMotor.class, "LBM");
        RBM = hwMap.get(DcMotor.class, "RBM");

        //set the motors to stop instead of coast on 0 power
        LFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set the motors on the left side of the robot to reverse for intuitive code with flipped motors
        LFM.setDirection(DcMotor.Direction.REVERSE);
        RFM.setDirection(DcMotor.Direction.FORWARD);
        LBM.setDirection(DcMotor.Direction.REVERSE);
        RBM.setDirection(DcMotor.Direction.FORWARD);
    }

    private void initIntake() {
        //map intake DcMotor variables to motors defined in the REV Expansion Hub config
        RI = hwMap.get(DcMotor.class, "RI");
        LI = hwMap.get(DcMotor.class, "LI");

        //set the motors on the left side of the robot to reverse for intuitive code with flipped motors
        LI.setDirection(DcMotor.Direction.REVERSE);
        RI.setDirection(DcMotor.Direction.FORWARD);
    }

    private void initOutput() {
        //map output DcMotor variables to motors defined in the REV Expansion Hub config
        B = hwMap.get(DcMotor.class, "B");

        Cap = hwMap.get(Servo.class, "Cap");
        BRot = hwMap.get(Servo.class, "BRot");
        BClamp = hwMap.get(Servo.class, "BClamp");
        LFMo = hwMap.get(Servo.class, "LFMo");
        RFMo = hwMap.get(Servo.class, "RFMo");

        BRot.setPosition(1);
        BClamp.setPosition(0);

        LFMo.setPosition(0.69);
        RFMo.setPosition(0.35);
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

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }
}

