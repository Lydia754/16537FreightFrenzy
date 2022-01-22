

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous
public class LoadCellTest extends LinearOpMode {
private LoadCellI2CXL test;
    @Override
    public void runOpMode() {
    test = hardwareMap.get(LoadCellI2CXL.class,"test");


    }
}
