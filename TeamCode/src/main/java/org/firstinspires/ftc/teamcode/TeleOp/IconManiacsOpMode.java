/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMap_Example;
import org.firstinspires.ftc.teamcode.IMHardwareBot;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
// List where other files are located that are used in this OpMode

/**
 * This OpMode uses the HardwareMap_Example class to define the devices on the robot.
 */
// CHAWKS: Name it something useful!
@TeleOp(name="IconManiacsOpMode", group="LinearOpMode")
// CHAWKS: What does @Disabled mean? what happens if we remove it?
//@Disabled
public class IconManiacsOpMode extends OpMode
{
    // Declare OpMode members.
    IMHardwareBot bot = new IMHardwareBot();
    double leftPower;
    double rightPower;
    double armPower;
    double conveyorPower;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // tells the driver the status of the robot
        telemetry.addData("Status:", "Getting Ready");
        telemetry.update();
        bot.init(hardwareMap);
        telemetry.addData("Status:", "Initialized Successfully");
        telemetry.update();

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        telemetry.addData("Status:", "Getting Ready");
        telemetry.update();
        telemetry.addData("Status:", "Initialized Successfully");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        // run until the end of the match (driver presses STOP)

        // Setup a variable for each drive wheel to save power level for telemetry


        double drive = -gamepad1.left_stick_y;  // maps the joysticks to the motors respective of the sides
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        bot.frontLeft.setPower((drive + strafe + rotate));
        bot.backRight.setPower((drive + strafe - rotate));
        bot.backLeft.setPower(drive - strafe + rotate);
        bot.frontRight.setPower(drive - strafe - rotate);



        // both left and right power sets the cap for the motors' range of motion
        //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

//        //gamepad2
//        // same thing as gamepad 1 right above ^^^^^
//        double drive2 = -gamepad2.left_stick_y;
//        double turn2  = gamepad2.right_stick_x;
//        leftPower2    = Range.clip(drive2 + turn2, -1.0, 1.0) ;
//        rightPower2   = Range.clip(drive2 - turn2, -1.0, 1.0) ;

        //gamepad 1
        leftPower =  -gamepad1.left_stick_y;  // maps the joysticks to the motors respective of the sides
        rightPower  = -gamepad1.right_stick_y;

        //gamepad 1 buttom mapping
        // Send calculated power to wheels
        bot.frontLeft.setPower(leftPower);
        bot.backLeft.setPower(leftPower);
        bot.frontRight.setPower(rightPower);
        bot.backRight.setPower(rightPower);

        if(gamepad2.right_bumper){ // increases the platform's current POS
            //platformPOS += SERVO_UP_POWER;
            bot.intake.setPower(1);
        } else if(gamepad2.left_bumper) { // decreases the platform's current POS
            // platformPOS -= SERVO_DOWN_POWER;
            bot.intake.setPower(-1);
        } else {
            bot.intake.setPower(0);
        }

        //      bot.platformPOS = Range.clip(bot.platformPOS, bot.SERVO_LOW_RANGE, bot.SERVO_HIGH_RANGE); // sets the cap for the platform's range of movement
        //    bot.platform.setPosition(platformPOS); // this is the one function that actually sets the position of the platform

        if(gamepad1.right_bumper){ //strafes right
            bot.frontLeft.setPower(1);
            bot.backRight.setPower(1);
            bot.backLeft.setPower(-1);
            bot.frontRight.setPower(-1);
        } else if(gamepad1.left_bumper){ //strafes left
            bot.frontLeft.setPower(-1);
            bot.backRight.setPower(-1);
            bot.frontRight.setPower(1);
            bot.backLeft.setPower(1);
        } else {
            bot.frontLeft.setPower(0);
            bot.backRight.setPower(0);
            bot.frontRight.setPower(0);
            bot.backLeft.setPower(0);
        }

        //gamepad2
        // same thing as gamepad 1 right above ^^^^^
        armPower = 0.4 * -gamepad2.left_stick_y;
       conveyorPower  = -gamepad2.right_stick_y;
       // armPower = Range.clip(drive2 + turn2, -1.0, 1.0) ;
     //   conveyorPower = Range.clip(drive2 - turn2, -1.0, 1.0) ;

        //gamepad 2 button/controls mapping
        bot.arm.setPower(armPower); // moves the arm
        bot.conveyorBelt.setPower(conveyorPower); // moves the conveyor belt

        // still under the works

        if(gamepad2.a) {
                bot.shooter.setPower(0.75);
        } else if(gamepad2.y) {
            bot.shooter.setPower(0);
        }

        if(gamepad2.right_trigger == 1){ // if the gamepad 2 right trigger gets pressed....
            bot.clawPOS += bot.SERVO_UP_POWER; // the claw position increases
        } else if(gamepad2.left_trigger == 1){ // if the gamepad 2 left trigger gets pressed...
            bot.clawPOS -= bot.SERVO_DOWN_POWER; // the claw position reduces
        }


      //  bot.ringBringerPOS = Range.clip(bot.ringBringerPOS, bot.SERVO_LOW_RANGE, bot.SERVO_HIGH_RANGE); //sets the range of motion for the ringBringer servo


        //bot.clawPOS = Range.clip(bot.clawPOS, bot.SERVO_LOW_RANGE, bot.SERVO_HIGH_RANGE); // sets the cap for the range of movement for the claw
        bot.claw.setPosition(bot.clawPOS); // this is the function that actually moves the servo
        // Show the elapsed game time and wheel power.
        telemetry.addData("G2:claw", "%.2f", bot.clawPOS); // shows the current position of the claw servo
        //   telemetry.addData("G1:platform", "%.2f", bot.platformPOS); // shows the current position of the platform servo
        // telemetry.addData("Status", "Run Time: " + runtime.toString()); // idk what this does
 //       telemetry.addData("G1:Motors", "left (%.2f), right (%.2f)", leftPower, rightPower); // shows the current position of the motors that move the robot itself
        telemetry.update(); //updates the info to the bottom of the driver station phone
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Status:", "Stopped"); // tells the driver that the robot is stopeped
        telemetry.update(); // stuff gets updated
    }

}
