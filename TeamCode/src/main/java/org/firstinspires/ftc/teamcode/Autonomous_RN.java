package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Red Near", preselectTeleOp = "SCOTBOT-DRIVE")
/*Robot is in red area near the audience*/
public class Autonomous_RN extends org.firstinspires.ftc.teamcode.Autonomous {
    @Override
    protected int STARTING_POSITION() { return 3; }
}
