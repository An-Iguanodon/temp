package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

public class Intake extends Mechanism {

    private ExpansionHubMotor ILM = null;
    private ExpansionHubMotor IRM = null;

    private boolean jamLeft = false;
    private boolean jamRight = false;

    public Intake() {
    }

    public Intake(OpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hwMap) {
        //map intake DcMotor variables to motors defined in the REV Expansion Hub config
        ILM = hwMap.get(ExpansionHubMotor.class, "ILM");
        IRM = hwMap.get(ExpansionHubMotor.class, "IRM");

        //set the motors on the left side of the robot to reverse for intuitive code with flipped motors
        ILM.setDirection(DcMotor.Direction.REVERSE);
        IRM.setDirection(DcMotor.Direction.FORWARD);

        //set the motors to stop instead of coast on 0 power
        ILM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ILM.setPower(0);
        IRM.setPower(0);
    }

    public void setIntakePower(double power) {
        detectJam();
        setLeftPower(power);
        setRightPower(power);
    }

    private void setLeftPower(double power) {
        if(jamLeft) power *= -1;
        ILM.setPower(power);
    }

    private void setRightPower(double power) {
        if(jamRight) power *= -1;
        IRM.setPower(power);
    }

    private void detectJam() {
        int stallThreshold = 6100;
        int jamThreshold = 100;
        double leftCurrent = ILM.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS);
        double rightCurrent = IRM.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS);
        double difference = leftCurrent - rightCurrent;

        // check if the motors are stalling
        if ((leftCurrent + rightCurrent) / 2 >= stallThreshold) {

            if (difference >= jamThreshold) {
                jamLeft = true;
            } else if (difference <= -jamThreshold) {
                jamRight = true;
            }

        } else {
            if (jamLeft || jamRight) {
                jamLeft = false;
                jamRight = false;
            }
        }
    }

    public void sendTelemetry() {
        opMode.telemetry.addData("ILM Pow", ILM.getPower());
        opMode.telemetry.addData("IRM Pow", IRM.getPower());

        opMode.telemetry.addData("ILM Pos", ILM.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS));
        opMode.telemetry.addData("IRM Pos", IRM.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS));
    }
}
