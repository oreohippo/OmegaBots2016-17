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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Omegabot: Teleop - Both Gamepad OpMode ", group="Omegabot")

public class OmegabotTeleopBothGamepadOpMode extends OpMode{

    /* Declare OpMode members. */
    HardwareOmegaBots robot       = new HardwareOmegaBots(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.
    double          beaconOffset  = 0.2 ;                  // Servo mid position
    double          beaconOffset2  = 0.2 ;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

       // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // Set the direction of the robot for Teleop
        robot.leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE
        robot.rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robot.leftMotor.setPower(robot.MOTOR_OFF_SPEED);
        robot.rightMotor.setPower(robot.MOTOR_OFF_SPEED);

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;
        double left2;
        double right2;
        double max;
        // Run wheels in POV mode (note: The jdouble max;oystick goes negative when pushed forwards, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        left  = -gamepad1.left_stick_y + gamepad1.right_stick_x;
        right = -gamepad1.left_stick_y - gamepad1.right_stick_x;

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        robot.leftMotor.setPower(left);
        robot.rightMotor.setPower(right);

         /* left2  = -gamepad2.left_stick_y + gamepad2.right_stick_x;
            right2 = -gamepad2.left_stick_y - gamepad2.right_stick_x;
            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left2), Math.abs(right2));
            if (max > 1.0)
            {
                left2 /= max;
                right2 /= max;
            }

            robot.leftMotor.setPower(left2);
            robot.rightMotor.setPower(right2);*/

        // Use gamepad buttons to move the arm up (Y) and down (A)
        if(gamepad1.b || gamepad2.b) {
            robot.scooperMotor.setPower(robot.SCOOPER_SPEED);
            robot.shooterMotor.setPower(robot.SHOOTER_SPEED);
        }
        else if(gamepad1.x ||gamepad2.x) {
            robot.scooperMotor.setPower(robot.MOTOR_OFF_SPEED);
            robot.shooterMotor.setPower(robot.MOTOR_OFF_SPEED);
        }

        /// Use gamepad left & right Bumpers to catapault
        if (gamepad1.right_bumper || gamepad2.right_bumper)
            robot.feeder.setPosition(robot.FEEDER_LOW);
        else if (gamepad1.left_bumper ||gamepad2.left_bumper){
            robot.feeder.setPosition(robot.FEEDER_HIGH);
        }


        // Move both servos to new position.  Assume servos are mirror image of each other.
        beaconOffset=gamepad1.right_trigger;
        robot.beacon.setPosition(HardwareOmegaBots.BEACON_SERVO + beaconOffset);

           /* beaconOffset2=gamepad2.right_trigger;
            robot.beacon.setPosition(HardwareOmegaBots.BEACON_SERVO + beaconOffset2);*/

        //Reverse the direction of the scooper for the corner vertices.
        if(gamepad1.y){
            robot.scooperMotor.setDirection(DcMotor.Direction.FORWARD);
            robot.scooperMotor.setPower(robot.SCOOPER_SPEED);
        }

        //Reverse the direction of the scooper for the corner vertices.
        if(gamepad1.y ||gamepad2.y){
            robot.scooperMotor.setDirection(DcMotor.Direction.FORWARD);
            robot.scooperMotor.setPower(robot.SCOOPER_SPEED);
        }
        else if(gamepad1.a ||gamepad2.a){
            robot.scooperMotor.setDirection(DcMotor.Direction.REVERSE);
            robot.scooperMotor.setPower(robot.SCOOPER_SPEED);
        }


    }

    /*
     * Code to run ONCE after the driver hits STOP
     *
     */
    @Override
    public void stop() {
        robot.leftMotor.setPower(robot.MOTOR_OFF_SPEED);
        robot.rightMotor.setPower(robot.MOTOR_OFF_SPEED);
        robot.scooperMotor.setPower(robot.MOTOR_OFF_SPEED);
        robot.shooterMotor.setPower(robot.MOTOR_OFF_SPEED);

        telemetry.addData("Say", "Good Job Driver");
        updateTelemetry(telemetry);
    }

}
