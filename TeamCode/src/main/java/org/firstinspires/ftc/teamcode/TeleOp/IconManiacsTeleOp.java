package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMap_Example;
import org.firstinspires.ftc.teamcode.IMHardwareBot;

// CHAWKS: Name it something useful!
@TeleOp(name = "IM Op Mode", group = "A")
//@Disabled
public class IconManiacsTeleOp extends HardwareMap_Example {

    IMHardwareBot bot = new IMHardwareBot();
    double leftPower;
    double rightPower;
    double armPower;
    double conveyorPower;
    double drive;
    double strafe;
    double rotate;
    @Override
    public void runOpMode() {
        /*
            CHAWKS: On Driver Station, telemetry will be display!
                    How is this useful for debugging?
        */
        // Send telemetry message to Driver Station
        telemetry.addData("Status: ", "Hit [Init] to Initialize ze bot");    //
        telemetry.update();

        /*
            CHAWKS: Step 0. Initialize OUR ROBOT
        */
        // MUST HAVE THIS LINE BELOW
        bot.init(hardwareMap);

        // Send telemetry message to "Driver Station" signify robot waiting;
        telemetry.addData("Status: ", "Hit [PLAY] to start!");    //
        telemetry.update();

        /*
            CHAWKS: Step 1. Hit Play to run through the code!
        */
        // MUST HAVE THIS LINE BELOW
        waitForStart();

        /*
            CHAWKS: Remember opModeIsActive?! It's a loop!
        */
        // run until the end of the match (driver presses [STOP])
        // MUST HAVE!
        while (opModeIsActive()) {

            drive = -gamepad1.left_stick_y;  // maps the joysticks to the motors respective of the sides
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;
            bot.frontLeft.setPower((drive + strafe + rotate));
            bot.backRight.setPower((drive + strafe - rotate));
            bot.backLeft.setPower(drive - strafe + rotate);
            bot.frontRight.setPower(drive - strafe - rotate);

            if(gamepad2.right_bumper){ // increases the platform's current POS
                bot.intake.setPower(1);
            } else if(gamepad2.left_bumper) { // decreases the platform's current POS
                bot.intake.setPower(-1);
            } else {
                bot.intake.setPower(0);
            }

            armPower = 0.4 * -gamepad2.left_stick_y;
            conveyorPower  = 0.65 * -gamepad2.right_stick_y;

            bot.arm.setPower(armPower); // moves the arm
            bot.conveyorBelt.setPower(conveyorPower); // moves the conveyor belt

            if(gamepad2.a) {
                bot.shooter.setPower(0.75);
            } else if(gamepad2.y) {
                bot.shooter.setPower(0);
            }

            if(gamepad2.right_trigger == 1){ // if the gamepad 2 right trigger gets pressed....
                bot.clawPOS += bot.SERVO_UP_POWER;// the claw position increases
            } else if(gamepad2.left_trigger == 1){ // if the gamepad 2 left trigger gets pressed...
                bot.clawPOS -= bot.SERVO_DOWN_POWER; // the claw position reduces
            }

            //bot.clawPOS = Range.clip(bot.clawPOS, bot.SERVO_LOW_RANGE, bot.SERVO_HIGH_RANGE); // sets the cap for the range of movement for the claw
            bot.claw.setPosition(bot.clawPOS); // this is the function that actually moves the servo
            // Show the elapsed game time and wheel power.
            telemetry.addData("G2:claw", "%.2f", bot.clawPOS); // shows the current position of the claw servo
            //   telemetry.addData("G1:platform", "%.2f", bot.platformPOS); // shows the current position of the platform servo
            // telemetry.addData("Status", "Run Time: " + runtime.toString()); // idk what this does
            //       telemetry.addData("G1:Motors", "left (%.2f), right (%.2f)", leftPower, rightPower); // shows the current position of the motors that move the robot itself
            telemetry.update(); //updates the info to the bottom of the driver station phone
        }
    }

}
