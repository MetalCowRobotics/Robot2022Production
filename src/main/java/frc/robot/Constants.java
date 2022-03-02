// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */

    /////////////////////////////////////////////////////////
    /////::::::::::::::::CAN/DIO INDEXES:::::::::::::::://///
    /////////////////////////////////////////////////////////

    //Magazine
    public static final int MAGAZINE_MOTOR = 15;
    public static final int MAGAZINE_SENSOR = 0;

    //Intake
    public static final int INTAKE_MOTOR = 16;
    public static final int INTAKE_DEPLOYMENT_EXTEND = 0;
    public static final int INTAKE_DEPLOYMENT_RETRACT = 1;

    //Climber
    public static final int CLIMBER_DRIVE_MOTOR_1 = 18;
    public static final int CLIMBER_DRIVE_MOTOR_2 = 19;
    public static final int CLIMBER_DEPLOY = 2;
    public static final int CLIMBER_RETRACT = 3;

    //Shooter
    public static final int SHOOTER_MOTOR_LEFT = 17;
    public static final int SHOOTER_MOTOR_RIGHT = 16; //Needs changing

    //Drivetrain

        //Pigeon
        public static final int DRIVETRAIN_PIGEON_ID = 14;

        //Front Left Module
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 8;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 7;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(59);

        //Front Right Module
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 6;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(2 + 180);

        //Back Left Module
        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 11;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 10;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(344);

        //Back Right Module
        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 2;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 1;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(95 + 180);

    /////////////////////////////////////////////////////////
    /////:::::::::::::::::::CONSTANTS::::::::::::::::::://///
    /////////////////////////////////////////////////////////

    //Drivetrain
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5;
    public static final double DRIVETRAIN_RAMP_SPEED = 0.6;

    //Magazine Speed
    public static final double MAGAZINE_SPEED = 0.05;

    //Intake Speed
    // TODO add speed
}
