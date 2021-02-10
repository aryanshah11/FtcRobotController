package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.*;

@TeleOp(name="Vanguard Omni TeleOp", group="Vanguard123")
//@Disabled
public class VanguardOmniTeleOp extends LinearOpMode {

    //Hello new members!
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Init hardware
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        //Ensures correct motor rotation
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        //Time to start!
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //Input from gamepad
            //Left joystick controls movement
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;

            //Vectors for each mecanum wheel
            double[] speeds = {
                    (drive + turn),
                    (drive - turn),
            };

            //Calculates the largest vector and scales it down to a max of 1
            double max = abs(speeds[0]);
            for(int i = 0; i < speeds.length; i++) {
                if (max < abs(speeds[i])) {
                    max = abs(speeds[i]);
                }
            }
            if (max > 1) {
                for (int i = 0; i < speeds.length; i++) {
                    speeds[i] /= max;
                }
            }

            //Set movement
            leftDrive.setPower(speeds[0]);
            rightDrive.setPower(speeds[1]);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftDrive (%.2f), rightDrive (%.2f)", speeds[0], speeds[1]);
            telemetry.update();
        }
    }
}