package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HardwareMain;

@Autonomous(name = "TestAutoV2", group = "Autonomous")
public class TestAutoV2 extends LinearOpMode {
    private HardwareMain robot = new HardwareMain(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //init
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            robot.drivetrain.encoderDrive(0.6, 60, false);
            stop();
        }

    }
}
