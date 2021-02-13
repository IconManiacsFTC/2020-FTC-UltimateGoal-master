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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.function.InterruptableThrowingRunnable;

import static java.lang.StrictMath.abs;
import static java.lang.Thread.sleep;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */

//0 - frontLeft (front left motor)
//1 - backLeft (back left motor)
//2 - backRight (back right motor)
//3 - frontRight (front right motor)
public class IMHardwareBot
{
    /* Public OpMode members. */
    // DCMotors and Servos declaration
    public DcMotor  frontLeft   = null;
    public DcMotor  frontRight  = null;
    public DcMotor  backLeft = null;
    public DcMotor  backRight = null;
    public DcMotor  conveyorBelt = null;
    public DcMotor  arm = null;
    public DcMotor shooter = null;
    public Servo claw = null;
    public DcMotor intake = null; //name will most likely change


    // constants and variables to be used when running the code (specifically servos)
    public static final double SERVO_HOME =  0.0 ;
    public static final double SERVO_UP_POWER =  .008d ;
    public static final double SERVO_DOWN_POWER  = .008d ;

    // initial positions of the servos used in the program
    public static final double clawOpen = 11.5;
    public static final double clawClose = 0.0;
    public static double clawPOS = clawOpen;


    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public IMHardwareBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors\
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backRight = hwMap.get(DcMotor.class, "backRight");
        conveyorBelt = hwMap.get(DcMotor.class, "conveyorBelt");
        arm = hwMap.get(DcMotor.class, "arm");
        shooter = hwMap.get(DcMotor.class, "shooter");
        intake = hwMap.get(DcMotor.class, "intake");

        // Set all motors to zero power
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        arm.setPower(0);
        conveyorBelt.setPower(0);
        shooter.setPower(0);
        intake.setPower(0);


        // sets zeroPowerBehavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyorBelt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Sets motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        conveyorBelt.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        // sets enconder mode to run with encoder
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyorBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        claw = hwMap.get(Servo.class, "claw");

        //platform.setPosition(SERVO_HOME);
        claw.setPosition(SERVO_HOME);


    }

    public void driveForward(double power, int time) throws InterruptedException {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        sleep(time);
        powerOff();
    }

    public void turnLeft(double power, int time) throws InterruptedException {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        frontRight.setPower(-power);
        sleep(time);
        powerOff();
    }

    public void turnRight(double power, int time) throws InterruptedException{
        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
        frontRight.setPower(power);
        sleep(time);
        powerOff();
    }

    public void driveBackwards(double power, int time) throws InterruptedException {
        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);
        frontRight.setPower(-power);
        sleep(time);
        powerOff();
    }

    public void powerOff(){
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }

    public void strafeLeft(double power, int time) throws InterruptedException{
        frontLeft.setPower(-power);
        backRight.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        sleep(time);
        powerOff();
    }

    public void strafeRight(double power, int time) throws InterruptedException{
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(-power);
        frontRight.setPower(-power);
        sleep(time);
        powerOff();
    }

    public void moveArm(double power, int time) throws InterruptedException {
        arm.setPower(power);
        sleep(time);
        arm.setPower(0);
    }

    public void moveClaw(double position, int time) throws InterruptedException {
        claw.setPosition(position);
        sleep(time);
        claw.setPosition(SERVO_HOME);
    }

    public void shoot(double power, int time) throws InterruptedException {
        shooter.setPower(power);
        sleep(time);
    }

    public void moveConveyorBelt(double power, int time) throws InterruptedException {
        conveyorBelt.setPower(power);
        sleep(time);
        conveyorBelt.setPower(0);
    }

}