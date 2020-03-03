package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift extends Mechanism {

    static final double COUNTS_PER_MOTOR_REV = 560;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 0.5 * 1.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public DcMotor LLM = null;
    public DcMotor LRM = null;

    public double LLMP = 0;
    public double LRMP = 0;

    private int level = 0;
    private boolean input_past = false;

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

    public void minPowerTest(boolean input) {
        if (input && !input_past) {
            LLMP += 0.005;
            LRMP += 0.005;
            setPower();
            input_past = true;
        } else if (!input) {
            input_past = false;
        }
    }

    public void raiseLift(boolean input) {
        if (input && !input_past) {
            level++;
            setLevel();
            input_past = true;
        } else if (!input) {
            input_past = false;
        }
    }

    public void lowerLift(boolean input) {
        if (input && !input_past) {
            level--;
            setLevel();
            input_past = true;
        } else if (!input) {
            input_past = false;
        }
    }

    public void resetLift(boolean input) {
        if (input && !input_past) {
            level = 0;
            setLevel();
            input_past = true;
        } else if (!input) {
            input_past = false;
        }
    }

    private void setLevel() {
        int newLTarget;
        int newRTarget;
        int lPos = LLM.getCurrentPosition();
        int rPos = LRM.getCurrentPosition();

        newLTarget = lPos + (int) ((level * 4) * COUNTS_PER_INCH);
        newRTarget = rPos + (int) ((level * 4) * COUNTS_PER_INCH);


        LLM.setTargetPosition(newLTarget);
        LRM.setTargetPosition(newRTarget);

        LLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LLM.setPower(0.5);
        LRM.setPower(0.5);

        while (LLM.isBusy() && LRM.isBusy()) {
            double error = LLM.getCurrentPosition() - LRM.getCurrentPosition();
            if (error > 20) {
                LLM.setPower(0.5 - error);
                LRM.setPower(0.5 + error);
            } else if (error < 20) {
                LLM.setPower(0.5 + error);
                LRM.setPower(0.5 - error);
            }
        }

        LLM.setPower(0);
        LRM.setPower(0);

        LLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void adjustLevel(double power) {
        LLMP = power/2;
        LRMP = power/2;
        setPower();
    }

    private void setPower() {
        LLM.setPower(LLMP);
        LRM.setPower(LRMP);
    }

    public void sendTelemetry() {
        opMode.telemetry.addData("LLM Pow", LLM.getPower());
        opMode.telemetry.addData("LRM Pow", LRM.getPower());
        //opMode.telemetry.addData("LLM Pos", LLM.getCurrentPosition());
        //opMode.telemetry.addData("LRM Pos", LRM.getCurrentPosition());
        opMode.telemetry.addData("Level", level);
    }
}
