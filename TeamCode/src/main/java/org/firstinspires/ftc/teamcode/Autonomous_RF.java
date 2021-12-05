package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Red Far", preselectTeleOp = "SCOTBOT-DRIVE")
/*Robot is in red area further from the audience*/
public class Autonomous_RF extends org.firstinspires.ftc.teamcode.Autonomous {
    @Override
    protected int STARTING_POSITION() { return 2; }
}
