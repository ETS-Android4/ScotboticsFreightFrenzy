package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Blue Near", preselectTeleOp = "SCOTBOT-DRIVE")
/*Robot is in blue area near the audience*/
public class Autonomous_BN extends org.firstinspires.ftc.teamcode.Autonomous {
    @Override
    protected int STARTING_POSITION() { return 1; }
}
