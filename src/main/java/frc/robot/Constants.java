// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

public final class Constants {
    
    private final static XboxController driverControls = new XboxController(0);
    private final static XboxController operatorControls = new XboxController(1);

    /////////////////////////////////////////////////////////
    /////::::::::::::::::CAN/DIO INDEXES:::::::::::::::://///
    /////////////////////////////////////////////////////////

    //Magazine
    public static final int MAGAZINE_MOTOR = 13;
    public static final int MAGAZINE_SENSOR_FRONT = 0;
    public static final int MAGAZINE_SENSOR_TOP = 1;

    //Intake
    public static final int INTAKE_MOTOR = 15;
    public static final int INTAKE_DEPLOYMENT_EXTEND = 2;
    public static final int INTAKE_DEPLOYMENT_RETRACT = 3;

    //Climber
    public static final int CLIMBER_DRIVE_MOTOR_1 = 18;
    public static final int CLIMBER_DRIVE_MOTOR_2 = 19;
    public static final int CLIMBER_DEPLOY = 0;
    public static final int CLIMBER_RETRACT = 1;

    //Shooter
    public static final int SHOOTER_MOTOR_LEFT = 17;
    public static final int SHOOTER_MOTOR_RIGHT = 16;

    //Drivetrain

        //Pigeon
        public static final int DRIVETRAIN_PIGEON_ID = 14;

        //Front Left Module
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 8;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 7;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(59.6 + 180);

        //Front Right Module
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 6;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(1.5);

        //Back Left Module
        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 11;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 10;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(345.4 + 180);

        //Back Right Module
        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 2;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 1;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(94.5);

    /////////////////////////////////////////////////////////
    /////:::::::::::::::::::CONSTANTS::::::::::::::::::://///
    /////////////////////////////////////////////////////////

    //Drivetrain
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5;
    public static final double DRIVETRAIN_RAMP_SPEED = 0.6;
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5;

    public static final double BASE_SPEED = 0.5;
    public static final double SPRINT_SCALAR = 2;
    public static final double CRAWL_SCALAR = 0.2;

    //Magazine Speed
    public static final double MAGAZINE_SPEED = 0.3;

    //Intake Speed
    public static final double INTAKE_SPEED = 0.8;

    //Climber
    public static final double CLIMB_SPEED = 0.4;

    //Shooter
    public static final double HIGH_SHOT_SPEED = 3200;
    public static final double LOW_SHOT_SPEED = 1750;


    /////////////////////////////////////////////////////////
    /////::::::::::::::CONTROLLER BINDINGS:::::::::::::://///
    /////////////////////////////////////////////////////////

    //Driver
        //Pigeon
        public final static Button CONT_RESET_GYRO = new Button(driverControls::getBackButton);

        //Intake
        public final static Button CONT_INTAKE_DEPLOY = new Button(driverControls::getRightBumper);
        public final static Button CONT_INTAKE_RETRACT = new Button(driverControls::getLeftBumper);

        //Sprint
        public final static Button CONT_SPRINT = new Button(() -> driverControls.getLeftTriggerAxis() > 0.7);

        //Crawl
        public final static Button CONT_CRAWL = new Button(() -> driverControls.getRightTriggerAxis() > 0.7);

        // Swerve
        public final static Button CONT_FIELD_ORIENTED = new Button(driverControls::getAButton);
        public final static Button CONT_ROBOT_ORIENTED = new Button(driverControls::getXButton);

    //Operator
        //Shooter
        public final static Button CONT_SHOOTER_LOW = new Button(() -> operatorControls.getRightTriggerAxis() > 0.3);
        public final static Button CONT_SHOOTER_FIRE = new Button(operatorControls::getBButton);

        //Field Mode
        public final static Button CONT_SWITCH_FIELD_MODE = new Button(operatorControls::getBackButton);

        //Climber
        public final static Button CONT_CLIMBER_UP = new Button(operatorControls::getRightBumper);
        public final static Button CONT_CLIMBER_DOWN = new Button(operatorControls::getLeftBumper);
        public final static Button CONT_CLIMBER_OUT = new Button(operatorControls::getYButton);
        public final static Button CONT_CLIMBER_IN = new Button(operatorControls::getAButton);
}
