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

package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.IMHardwareBot;

import static org.firstinspires.ftc.teamcode.IMHardwareBot.clawClose;
import static org.firstinspires.ftc.teamcode.IMHardwareBot.clawOpen;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
// List where other files are located that are used in this OpMode

/**
 * In this example:
 * This file illustrates the concept of driving OUR robot in HardwareMap_Example
 *
 */
// CHAWKS: Name it something useful!
@Autonomous(name="IM Autonomous B", group="RedTest")
// CHAWKS: What does @Disabled mean? what happens if we remove it?
//@Disabled

public class IconManiacsAutonomous_B extends LinearOpMode {

    IMHardwareBot bot = new IMHardwareBot();
    /*
        CHAWKS: It has begun!!! Run the OpMode!!! Make the robot execute all our code!!!
    */

    // MUST HAVE
    @Override
    public void runOpMode() throws InterruptedException {

      Thread shoot = new Thread () {
            public void run () {
                try {
                    bot.shoot(0.605, 10000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        };

        Thread moveRing = new Thread (){
            public void run () {
                try {
                   bot.moveConveyorBelt(1, 8000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        };
         // Initialize the drive system variables.
         // The init() method of the hardware class does all the work here

        /*
            CHAWKS: On Driver Station, telemetry will be display!
                    Why is this good for the Drivers?
        */
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status: ", "Hit [Init] to Initialize");    //
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
            CHAWKS: Step 1. Hit P
           lay to run through the code!
        */
        // MUST HAVE THIS LINE BELOW
        // waits for driver to hit start
        waitForStart();
        // turns off the claw and arm
        bot.claw.setPosition(0);
        bot.arm.setPower(0);

        // drives forward and starts the shooter
        shoot.start();
        bot.driveForward(0.75, 3250);

        // sets arm down
        bot.moveArm(0.45, 450);

        // pauses
        sleep(500);

        // releases the wobble goal
        bot.moveClaw(clawOpen, 1000);

        // moves backwards
        bot.driveBackwards(0.75, 1400);

        //pause
        sleep(600);
        //turns right
        bot.turnRight(0.7, 60);

        // pause
        sleep(1000);

        // turns on the shooter at the same time

       moveRing.start();

       sleep(2000);

        shoot.join();
        moveRing.join();

        //turns left
        bot.turnLeft(0.7, 40);

        //pause
        sleep(400);

        bot.driveForward(0.7, 450);

        telemetry.addData("Path", "Complete!");
        telemetry.update();
    }

}
