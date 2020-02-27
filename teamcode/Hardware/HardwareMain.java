package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareMain {
    public OpMode opMode;
    public HardwareMap hwMap;
    public Drivetrain drivetrain;
    public Intake intake;
    public Lift lift;
    public Vision vision;

    public HardwareMain(OpMode opMode) {
        this.opMode = opMode;
        drivetrain = new Drivetrain(opMode);
        intake = new Intake(opMode);
        lift = new Lift(opMode);
        vision = new Vision(opMode);
    }

    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        intake.init(hwMap);
        lift.init(hwMap);
        vision.init(hwMap);
    }


}
