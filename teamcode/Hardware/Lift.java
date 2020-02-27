package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift extends Mechanism {

    static final double COUNTS_PER_MOTOR_REV = 560;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public DcMotor LLM = null;
    public DcMotor LRM = null;

    public Lift() {

    }

    public Lift(OpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hwMap) {
        LLM = hwMap.get(DcMotor.class, "LLM");
        LRM = hwMap.get(DcMotor.class, "LRM");

        LLM.setDirection(DcMotor.Direction.REVERSE);
        LRM.setDirection(DcMotor.Direction.FORWARD);

        LLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LLM.setPower(0);
        LRM.setPower(0);
    }
}
