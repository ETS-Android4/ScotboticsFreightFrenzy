package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Blue Far", preselectTeleOp = "SCOTBOT-DRIVE")
/*Robot is in blue area further from the audience*/
public class Autonomous_BF extends org.firstinspires.ftc.teamcode.Autonomous {
    @Override
    protected int STARTING_POSITION() { return 0; }
}
