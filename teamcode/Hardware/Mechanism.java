package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Mechanism {
        protected OpMode opMode;

        public abstract void init(HardwareMap hwMap);
}
