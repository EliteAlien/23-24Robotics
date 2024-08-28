/* Copyright (c) 2022 FIRST. All rights reserved.
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

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 *  This OpMode illustrates the concept of driving an autonomous path based on Gyro (IMU) heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code uses the Universal IMU interface so it will work with either the BNO055, or BHI260 IMU.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *  The REV Logo should be facing UP, and the USB port should be facing forward.
 *  If this is not the configuration of your REV Control Hub, then the code should be modified to reflect the correct orientation.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: This code implements the requirement of calling setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  https://ftc-docs.firstinspires.org/field-coordinate-system
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="RedRightCameraV2", group="RedRight")
//@Disabled
public class RedRightCameraV2 extends LinearOpMode {
    RobotHardwareV1 robot = new RobotHardwareV1(this);
    private ElapsedTime runtime = new ElapsedTime();

    /* Declare OpMode members. */
//    private DcMotor         leftDrive   = null;
//    private DcMotor         rightDrive  = null;
    private IMU             imu         = null;      // Control/Expansion Hub IMU

    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private double timeoutS = 10;

    private int block_x_coord =150;
    private HuskyLens.Block currentBlock = null;
    private int propLocation = 0;


    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 2000 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 0.9 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 1.8897637795275 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final int        DOWN_POSITION          = 0;
    static final int        DOWN_POSITION2          = 10;
    static final int        UP_POSITION          = 100;
    static final int        UP_POSITION2          = 250;
    static final int        UP_POSITION3          = 750;
    static final double     FAST_SPEED              = 0.5;
    static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
                                                               // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
//        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
//        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//        leftDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.init();
        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
//        robot.encoderone.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.encoderone.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        robot.encodertwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the encoders for closed loop speed control, and reset the heading.
        robot.leftfrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightfrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftbackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightbackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();


        // Wait for the game to start
//        HuskyLens.Block[] blocks = robot.huskyLens.blocks();


        while (opModeInInit()) {
            // waiting for START
            telemetry.addData("READY TO START >", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < 2.0){
            HuskyLens.Block[] blocks = robot.huskyLens.blocks();
            telemetry.addData("Looking for Blocks","%1.1f", runtime.seconds());
            telemetry.addData("Block count", blocks.length);
            telemetry.update();
            if(blocks.length>0){
                block_x_coord = blocks[0].x;
            }
        }

        // Step through each leg of the path,
        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //          holdHeading() is used after turns to let the heading stabilize
        //          Add a sleep(2000) after any step to keep the telemetry data visible for review

            if (block_x_coord < 100){
                // pos #1
                propLocation=4;
                telemetry.addData("Found block at position 1","%4d",block_x_coord);

                robot.Lift.setTargetPosition(UP_POSITION);
                robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.Lift.setPower(0.5);
                while ( opModeIsActive() && robot.Lift.isBusy()) {
                    telemetry.addData("SlideUp", "");
                    telemetry.update();
                }
                robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Lift.setPower(0);

                robot.Claw.setPosition(0.4);
                sleep(1000);

                robot.Lift.setTargetPosition(UP_POSITION2);
                robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.Lift.setPower(0.5);
                while ( opModeIsActive() && robot.Lift.isBusy()) {
                    telemetry.addData("SlideUp", "");
                    telemetry.update();
                }
                robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Lift.setPower(0);

                mecanumDrivebyDistanceX(0.5, 90, 30);

                turnToHeading(TURN_SPEED, 90);
                holdHeading(TURN_SPEED, 90, 0.5);

                mecanumDrivebyDistanceX(0.5, -90, 20);

                turnToHeading(TURN_SPEED, -90);
                holdHeading(TURN_SPEED, -90, 0.5);

                mecanumDrivebyDistanceY(0.5, 0, 10);
            }
            else if (block_x_coord >= 200 && block_x_coord < 300) {
                // pos #3
                propLocation=6;
                telemetry.addData("Found block at position 3","%4d", block_x_coord);

                robot.Lift.setTargetPosition(UP_POSITION);
                robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.Lift.setPower(0.5);
                while ( opModeIsActive() && robot.Lift.isBusy()) {
                    telemetry.addData("SlideUp", "");
                    telemetry.update();
                }
                robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Lift.setPower(0);

                robot.Claw.setPosition(0.4);
                sleep(1000);

                robot.Lift.setTargetPosition(UP_POSITION2);
                robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.Lift.setPower(0.5);
                while ( opModeIsActive() && robot.Lift.isBusy()) {
                    telemetry.addData("SlideUp", "");
                    telemetry.update();
                }
                robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Lift.setPower(0);

                mecanumDrivebyDistanceX(0.5, 90, 30);

                turnToHeading(TURN_SPEED, -90.0);
                holdHeading(TURN_SPEED, -90.0, 0.5);

                mecanumDrivebyDistanceX(0.4, 90, 4);
                sleep(1000);
                mecanumDrivebyDistanceX(0.4, -90, 1.5);
                mecanumDrivebyDistanceY(0.5, 0, 15);
                holdHeading(TURN_SPEED, -90, 0.5);
                mecanumDrivebyDistanceX(0.5, 90, 20);
                holdHeading(TURN_SPEED, -90, 0.5);
            }
        else {
            // no prop found
            // make default drive here
                propLocation=5;
            telemetry.addData("No Block Found", "");

            robot.Lift.setTargetPosition(UP_POSITION);
            robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Lift.setPower(0.5);
            while ( opModeIsActive() && robot.Lift.isBusy()) {
                telemetry.addData("SlideUp", "");
                telemetry.update();
            }
            robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Lift.setPower(0);

                robot.Claw.setPosition(0.4);
                sleep(1000);

                robot.Lift.setTargetPosition(UP_POSITION2);
                robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.Lift.setPower(0.5);
                while ( opModeIsActive() && robot.Lift.isBusy()) {
                    telemetry.addData("SlideUp", "");
                    telemetry.update();
                }
                robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Lift.setPower(0);

                mecanumDrivebyDistanceX(0.4, 90, 33);
                mecanumDrivebyDistanceX(0.4, -90, 9);

                turnToHeading(TURN_SPEED, -90);
                holdHeading(TURN_SPEED, -90, 0.5);

                mecanumDrivebyDistanceX(0.5, 90, 20);
                holdHeading(TURN_SPEED, -90, 0.5);
        }

        robot.huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        currentBlock = getHuskyBlock(propLocation);
        mecanumDrive(0.1, 180);
        while (opModeIsActive()  && currentBlock == null || currentBlock.x < 100) {
            currentBlock = getHuskyBlock(propLocation);
            telemetry.addData("prop ID", "%d", propLocation);
            if ( currentBlock != null) {
                telemetry.addData("BlockX", "%d", currentBlock.x);
                telemetry.addData("TagID", "%d", currentBlock.id);
            }
            telemetry.addData("StrafeRight", "");
            telemetry.update();
        }
        robot.all_stop();
        sleep(1000);

        if (propLocation == 4) {
            mecanumDrivebyDistanceX(0.5, 90, 15);
            holdHeading(TURN_SPEED, -90, 0.5);

            robot.Lift.setTargetPosition(UP_POSITION3);
            robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Lift.setPower(0.5);
            while ( opModeIsActive() && robot.Lift.isBusy()) {
                telemetry.addData("SlideUp", "");
                telemetry.update();
            }
            robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Lift.setPower(0);
            sleep(1000);

            robot.Vert.setPosition(0.2);
            sleep(500);
            robot.Claw.setPosition(0.5);
            sleep(500);

            mecanumDrivebyDistanceX(0.5, -90, 5);
            robot.Vert.setPosition(0.45);
            robot.Lift.setTargetPosition(DOWN_POSITION2);
            robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Lift.setPower(0.5);
            while ( opModeIsActive() && robot.Lift.isBusy()) {
                telemetry.addData("SlideUp", "");
                telemetry.update();
            }
            robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Lift.setPower(0);

            telemetry.addData("Parking","in progress");
            telemetry.update();

            mecanumDrivebyDistanceY(0.5, 0, 30);
            mecanumDrivebyDistanceX(0.5, 90, 10);
        }else if ( propLocation == 6){
            telemetry.addData("TagWidth", "%d", currentBlock.width);
            telemetry.update();
            mecanumDrivebyDistanceX(0.5, 90, 11);
            holdHeading(TURN_SPEED, -90, 0.5);

            robot.Lift.setTargetPosition(UP_POSITION3);
            robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Lift.setPower(0.5);
            while ( opModeIsActive() && robot.Lift.isBusy()) {
                telemetry.addData("TagWidth", "%d", currentBlock.width);
                telemetry.addData("SlideUp", "");
                telemetry.update();
            }
            robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Lift.setPower(0);
            sleep(1000);

            robot.Vert.setPosition(0.2);
            sleep(500);
            robot.Claw.setPosition(0.5);
            sleep(500);

            mecanumDrivebyDistanceX(0.5, -90, 5);
            robot.Vert.setPosition(0.45);
            robot.Lift.setTargetPosition(DOWN_POSITION2);
            robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Lift.setPower(0.5);
            while ( opModeIsActive() && robot.Lift.isBusy()) {
                telemetry.addData("SlideUp", "");
                telemetry.update();
            }
            robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Lift.setPower(0);

            telemetry.addData("Parking","in progress");
            telemetry.update();

            mecanumDrivebyDistanceY(0.5, 0, 15);
            mecanumDrivebyDistanceX(0.5, 90, 10);
        }else {
            turnToHeading(TURN_SPEED, -90);
            mecanumDrivebyDistanceX(0.5, 90, 12);

            turnToHeading(TURN_SPEED, -90);
            holdHeading(TURN_SPEED, -90, 0.5);

            robot.Lift.setTargetPosition(740);
            robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Lift.setPower(0.5);
            while ( opModeIsActive() && robot.Lift.isBusy()) {
                telemetry.addData("SlideUp", "");
                telemetry.update();
            }
            robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Lift.setPower(0);
            sleep(1000);

            robot.Vert.setPosition(0.2);
            sleep(500);
            robot.Claw.setPosition(0.5);
            sleep(500);

            mecanumDrivebyDistanceX(0.5, -90, 3);
            sleep(1000);
            mecanumDrivebyDistanceX(0.5, -90, 3);
            robot.Vert.setPosition(0.45);
            robot.Lift.setTargetPosition(DOWN_POSITION2);
            robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Lift.setPower(0.5);
            while ( opModeIsActive() && robot.Lift.isBusy()) {
                telemetry.addData("SlideUp", "");
                telemetry.update();
            }
            robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Lift.setPower(0);

            telemetry.addData("Parking","in progress");
            telemetry.update();

            mecanumDrivebyDistanceY(0.5, 0, 20);
            mecanumDrivebyDistanceX(0.5, 90, 10);
        }

        telemetry.addData("Parking","Done");
        telemetry.update();

        robot.Lift.setTargetPosition(DOWN_POSITION);
        robot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Lift.setPower(0.5);
        while ( opModeIsActive() && robot.Lift.isBusy()) {
            telemetry.addData("SlideUp", "");
            telemetry.update();
        }
        robot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Lift.setPower(0);

        robot.huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        robot.all_stop();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    public void mecanumDrive(double power, double theta) {

        double sin= Math.sin(theta * Math.PI/180 - Math.PI/4);
        double cos= Math.cos(theta * Math.PI/180 - Math.PI/4);
        double max= Math.max(Math.abs(sin), Math.abs(cos));


        double leftFront = power * cos/max;
        double rightFront = power * sin/max;
        double leftBack = power * sin/max;
        double rightBack = power * cos/max;


        robot.leftfrontDrive.setPower(leftFront);
        robot.rightfrontDrive.setPower(rightFront);
        robot.leftbackDrive.setPower(leftBack);
        robot.rightbackDrive.setPower(rightBack);

        telemetry.addData( "Angle", " %2f", theta);
        telemetry.addData("CurrentTimer", "%2f", runtime.seconds());


        telemetry.addData( "leftFront", " %2f", leftFront);
        telemetry.addData( "rightFront", " %2f", rightFront);
        telemetry.addData( "leftBack", " %2f", leftBack);
        telemetry.addData( "rightBack", " %2f", rightBack);
        telemetry.update();
    }

    public void mecanumDrivebyDistanceX(double power, double theta, double distance) {

        int TIMEOUT = 5;

        double sin= Math.sin(theta * Math.PI/180 - Math.PI/4);
        double cos= Math.cos(theta * Math.PI/180 - Math.PI/4);
        double max= Math.max(Math.abs(sin), Math.abs(cos));


        double leftFront = power * cos/max;
        double rightFront = power * sin/max;
        double leftBack = power * sin/max;
        double rightBack = power * cos/max;

        // encoderone= front to back     encodertwo= left to right
        robot.encoderone.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        int moveCounts = (int)(distance * COUNTS_PER_INCH);
        int xTarget = robot.encoderone.getCurrentPosition() + moveCounts;



        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < TIMEOUT && Math.abs(robot.encoderone.getCurrentPosition()) < xTarget) {
            robot.leftfrontDrive.setPower(leftFront);
            robot.rightfrontDrive.setPower(rightFront);
            robot.leftbackDrive.setPower(leftBack);
            robot.rightbackDrive.setPower(rightBack);

            telemetry.addData( "Angle", "%2f", theta);
            telemetry.addData("CurrentTimer", "%2f", runtime.seconds());
            telemetry.addData("TargetDistance", "%2f", distance);

            telemetry.addData( "leftFront", "%2f", leftFront);
            telemetry.addData( "rightFront", "%2f", rightFront);
            telemetry.addData( "leftBack", "%2f", leftBack);
            telemetry.addData( "rightBack", "%2f", rightBack);
            telemetry.update();

        }

        robot.leftfrontDrive.setPower(0);
        robot.rightfrontDrive.setPower(0);
        robot.leftbackDrive.setPower(0);
        robot.rightbackDrive.setPower(0);


    }

    public void mecanumDrivebyDistanceY(double power, double theta, double distance) {

        int TIMEOUT = 5;

        double sin= Math.sin(theta * Math.PI/180 - Math.PI/4);
        double cos= Math.cos(theta * Math.PI/180 - Math.PI/4);
        double max= Math.max(Math.abs(sin), Math.abs(cos));


        double leftFront = power * cos/max;
        double rightFront = power * sin/max;
        double leftBack = power * sin/max;
        double rightBack = power * cos/max;

        // encoderone= front to back     encodertwo= left to right
        robot.encodertwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        int moveCounts = (int)(distance * COUNTS_PER_INCH);
        int yTarget = robot.encodertwo.getCurrentPosition() + moveCounts;



        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < TIMEOUT && Math.abs(robot.encodertwo.getCurrentPosition()) < yTarget) {
            robot.leftfrontDrive.setPower(leftFront);
            robot.rightfrontDrive.setPower(rightFront);
            robot.leftbackDrive.setPower(leftBack);
            robot.rightbackDrive.setPower(rightBack);

            telemetry.addData( "Angle", "%2f", theta);
            telemetry.addData("CurrentTimer", "%2f", runtime.seconds());
            telemetry.addData("TargetDistance", "%2f", distance);

            telemetry.addData( "leftFront", "%2f", leftFront);
            telemetry.addData( "rightFront", "%2f", rightFront);
            telemetry.addData( "leftBack", "%2f", leftBack);
            telemetry.addData( "rightBack", "%2f", rightBack);
            telemetry.update();

        }

        robot.leftfrontDrive.setPower(0);
        robot.rightfrontDrive.setPower(0);
        robot.leftbackDrive.setPower(0);
        robot.rightbackDrive.setPower(0);


    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        robot.leftfrontDrive.setPower(leftSpeed);
        robot.rightfrontDrive.setPower(rightSpeed);
        robot.leftbackDrive.setPower(leftSpeed);
        robot.rightbackDrive.setPower(rightSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      robot.encoderone.getCurrentPosition(),
                    robot.encodertwo.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private HuskyLens.Block getHuskyBlock(int tagID) {
        HuskyLens.Block returnBlock = null;

        //get array of blocks from camera
        HuskyLens.Block[] blocks= robot.huskyLens.blocks(tagID);

        if (blocks.length ==0) {
            telemetry.addData("No blocks identified", "");
        } else {
            for (int i = 0; i < blocks.length; i++) {
                returnBlock = blocks[i];
                break;
            }  //end of for
        }  // end of if
        return returnBlock;
    } //end of getHusky


}
