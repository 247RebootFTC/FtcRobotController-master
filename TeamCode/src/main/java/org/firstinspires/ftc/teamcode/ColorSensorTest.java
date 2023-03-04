package org.firstinspires.ftc.teamcode;
//package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;

@Autonomous(name="ColorSensorTest")
@Disabled
public class ColorSensorTest extends LinearOpMode {
    AnalogInput ColorSensor = new AnalogInput(new AnalogInputController, 1);
    int input;

    public void runOpMode() throws InterruptedException {
        //input = ColorSensor.getVoltage();
        //telemetry.addLine(input);
        sleep(1000);
        }
    }