/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common OmegaBots hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *

 *   The desired path in this example is:
 *   - Shoot two balls
 *   - Drive forward for 3 seconds
 * The code is written in a simple form with no optimizations.
 * However, there are several ways that this type of sequence could be streamlined,
 */

@Autonomous(name="Omegabot: Autonomous - Right", group="OmegaBot")
public class OmegabotAutonomousRight extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareOmegaBots robot   = new HardwareOmegaBots();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    private ElapsedTime     runtimestart = new ElapsedTime();

    static final double     INIT_TIMER    = 1.0;
    static final double     MOVE_TIMER    = 3.5;



    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        robot.rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE if using AndyMark motors

        robot.leftMotor.setPower(robot.ROBOT_SPEED);
        robot.rightMotor.setPower(robot.ROBOT_SPEED);
        runtimestart.reset();
        while (opModeIsActive() && (runtimestart.seconds() < INIT_TIMER)) {
            telemetry.addData("INIT Timer", "Leg 1: %2.5f S Elapsed", runtimestart.seconds());
            telemetry.update();
            idle();
        }
        robot.leftMotor.setPower(robot.MOTOR_OFF_SPEED);
        robot.rightMotor.setPower(robot.MOTOR_OFF_SPEED);
        //Step 1 Initialize Scooper and Shooter
        telemetry.addData("Feeder Initial Position",  robot.feeder.getPosition());
        telemetry.update();
        robot.shooterMotor.setPower(robot.SHOOTER_SPEED);
        sleep(1000);
        catapult();
        sleep(2000);
        robot.scooperMotor.setPower(robot.SCOOPER_SPEED);
        catapult();
        sleep(1000);


        // Step 2:  Drive forward to the center mat.
        robot.leftMotor.setPower(robot.ROBOT_SPEED);
        robot.rightMotor.setPower(robot.ROBOT_SPEED);
        runtime.reset();
               while (opModeIsActive() && (runtime.seconds() < MOVE_TIMER)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }


        robot.leftMotor.setPower(robot.MOTOR_OFF_SPEED);
        robot.rightMotor.setPower(robot.MOTOR_OFF_SPEED);
        robot.scooperMotor.setPower(robot.MOTOR_OFF_SPEED);
        robot.shooterMotor.setPower(robot.MOTOR_OFF_SPEED);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void catapult() throws InterruptedException {
        robot.feeder.setPosition(robot.FEEDER_LOW);
        sleep(2000);
        robot.feeder.setPosition(robot.FEEDER_HIGH);
        sleep(1000);
        robot.feeder.setPosition(robot.FEEDER_LOW);
    }

}

