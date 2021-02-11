package com.swervedrivespecialties.exampleswerve.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.swervedrivespecialties.exampleswerve.RobotMap;
import com.swervedrivespecialties.exampleswerve.commands.DriveCommand;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;
//import org.frcteam2910.common.robot.drivers.Mk2SwerveModuleBuilder;
import org.frcteam2910.common.robot.drivers.NavX;

import com.swervedrivespecialties.exampleswerve.Mk2SwerveModuleBuilder;

public class DrivetrainSubsystem extends Subsystem {
    private static final double TRACKWIDTH = 19.5;
    private static final double WHEELBASE = 23.5;

    private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(0.0);
    private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(0.0);
    private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(0.0);
    private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(0.0);

    private static DrivetrainSubsystem instance;

    private static final CANSparkMax FLDMotor = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    
    private static final CANCoder frontLeftCANCoder = new CANCoder(RobotMap.DRIVETRAIN_FRONT_LEFT_ENCODER);
    private static final CANCoder frontRightCANCoder = new CANCoder(RobotMap.DRIVETRAIN_FRONT_RIGHT_ENCODER);
    private static final CANCoder backLeftCANCoder = new CANCoder(RobotMap.DRIVETRAIN_BACK_LEFT_ENCODER);
    private static final CANCoder backRightCANCoder = new CANCoder(RobotMap.DRIVETRAIN_BACK_RIGHT_ENCODER);

    private final SwerveModule frontLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(frontLeftCANCoder)
            .angleMotor(new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(FLDMotor,
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    private final SwerveModule frontRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(frontRightCANCoder)
            //.angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER), FRONT_RIGHT_ANGLE_OFFSET)
            .angleMotor(new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    private final SwerveModule backLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(backLeftCANCoder)
            //.angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER), BACK_LEFT_ANGLE_OFFSET)
            .angleMotor(new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    private final SwerveModule backRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(backRightCANCoder)
            //.angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER), BACK_RIGHT_ANGLE_OFFSET)
            .angleMotor(new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();

// Locations for the swerve drive modules relative to the robot center.
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)
    );

    private final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);

    public DrivetrainSubsystem() {
        gyroscope.calibrate();
        gyroscope.setInverted(true); // You might not need to invert the gyro

        frontLeftModule.setName("Front Left");
        frontRightModule.setName("Front Right");
        backLeftModule.setName("Back Left");
        backRightModule.setName("Back Right");

        frontLeftCANCoder.setPosition(0);
        frontRightCANCoder.setPosition(0);
        backLeftCANCoder.setPosition(0);
        backRightCANCoder.setPosition(0);
    }

    public static DrivetrainSubsystem getInstance() {
        if (instance == null) {
            instance = new DrivetrainSubsystem();
        }

        return instance;
    }

    @Override
    public void periodic() {
        frontLeftModule.updateSensors();
        frontRightModule.updateSensors();
        backLeftModule.updateSensors();
        backRightModule.updateSensors();

        SmartDashboard.putString("Modules/Module Anlges", "Angles");
        SmartDashboard.putNumber("Modules/Front Left Module Angle", Math.toDegrees(frontLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Modules/Front Right Module Angle", Math.toDegrees(frontRightModule.getCurrentAngle()));
        SmartDashboard.putNumber("Modules/Back Left Module Angle", Math.toDegrees(backLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Modules/Back Right Module Angle", Math.toDegrees(backRightModule.getCurrentAngle()));

        // The distance a specific module has driven ever snce the robot started up
        SmartDashboard.putString("Modules/Module Distances", "Distance module has drive since robot started up");
        SmartDashboard.putNumber("Modules/Front Left Module Distance", frontLeftModule.getCurrentDistance());
        SmartDashboard.putNumber("Modules/Front Right Module Distance", frontRightModule.getCurrentDistance());
        SmartDashboard.putNumber("Modules/Back Left Module Distance", backLeftModule.getCurrentDistance());
        SmartDashboard.putNumber("Modules/Back Right Module Distance", backRightModule.getCurrentDistance());


// Current position of module
        SmartDashboard.putString("Modules/Current Position", "Position x-Coordinate (relative to center)");
        SmartDashboard.putNumber("Modules/Front Left Module Current Position x-Coordinate", frontLeftModule.getCurrentPosition().x);
        SmartDashboard.putNumber("Modules/Front Right Module Current Position x-Coordinate", frontRightModule.getCurrentPosition().x);
        SmartDashboard.putNumber("Modules/Back Left Module Current Position x-Coordinate", backLeftModule.getCurrentPosition().x);
        SmartDashboard.putNumber("Modules/Back Right Module Current Position x-Coordinate", backRightModule.getCurrentPosition().x);

        SmartDashboard.putString("Modules/Current Position", "Position y-Coordinate (relative to center)");
        SmartDashboard.putNumber("Modules/Front Left Module Current Position y-Coordinate", frontLeftModule.getCurrentPosition().y);
        SmartDashboard.putNumber("Modules/Front Right Module Current Position y-Coordinate", frontRightModule.getCurrentPosition().y);
        SmartDashboard.putNumber("Modules/Back Left Module Current Position y-Coordinate", backLeftModule.getCurrentPosition().y);
        SmartDashboard.putNumber("Modules/Back Right Module Current Position y-Coordinate", backRightModule.getCurrentPosition().y);

        
// A module's location relative to the center of mass the robot
        SmartDashboard.putString("Modules/Module Location Relative to Center of Robot", "Location x-Coordinate");
        SmartDashboard.putNumber("Modules/Front Left Module x-Coordinate", frontLeftModule.getModulePosition().x);
        SmartDashboard.putNumber("Modules/Front Right Module x-Coordinate", frontRightModule.getModulePosition().x);
        SmartDashboard.putNumber("Modules/Back Left Module x-Coordinate", backLeftModule.getModulePosition().x);
        SmartDashboard.putNumber("Modules/Back Right Module x-Coordinate", backRightModule.getModulePosition().x);

        SmartDashboard.putString("Modules/Module Location Relative to Center of Robot", "Location y-Coordinate");
        SmartDashboard.putNumber("Modules/Front Left Module y-Coordinate", frontLeftModule.getModulePosition().y);
        SmartDashboard.putNumber("Modules/Front Right Module y-Coordinate", frontRightModule.getModulePosition().y);
        SmartDashboard.putNumber("Modules/Back Left Module y-Coordinate", backLeftModule.getModulePosition().y);
        SmartDashboard.putNumber("Modules/Back Right Module y-Coordinate", backRightModule.getModulePosition().y);


// Velocity that modules are at
        SmartDashboard.putString("Module Velocity", "Velocity");
        SmartDashboard.putNumber("Front Left Module Velocity", frontLeftModule.getCurrentVelocity());
        SmartDashboard.putNumber("Front Right Module Velocity", frontRightModule.getCurrentVelocity());
        SmartDashboard.putNumber("Back Left Module Velocity", backLeftModule.getCurrentVelocity());
        SmartDashboard.putNumber("Back Right Module Velocity", backRightModule.getCurrentVelocity());


// Target velocity
        SmartDashboard.putString("Target Velocity", "Velocity Magnitude");
        SmartDashboard.putNumber("Front Left Module Target Velocity Degree", frontLeftModule.getTargetVelocity().length);
        SmartDashboard.putNumber("Front Right Module Target Velocity Degree", frontRightModule.getTargetVelocity().length);
        SmartDashboard.putNumber("Back Left Module Target Velocity Degree", backLeftModule.getTargetVelocity().length);
        SmartDashboard.putNumber("Back Right Module Target Velocity Degree", backRightModule.getTargetVelocity().length);

        SmartDashboard.putString("Target Velocity", "Velocity Degree");
        SmartDashboard.putNumber("Front Left Module Target Velocity Degree", frontLeftModule.getTargetVelocity().getAngle().toDegrees());
        SmartDashboard.putNumber("Front Right Module Target Velocity Degree", frontRightModule.getTargetVelocity().getAngle().toDegrees());
        SmartDashboard.putNumber("Back Left Module Target Velocity Degree", backLeftModule.getTargetVelocity().getAngle().toDegrees());
        SmartDashboard.putNumber("Back Right Module Target Velocity Degree", backRightModule.getTargetVelocity().getAngle().toDegrees());




//CAN Encoder stuff
        SmartDashboard.putString("CAN_Encoders/CAN Encoder Absolute Position", "Absolute Position");
        SmartDashboard.putNumber("CAN_Encoders/CAN Encoder Absolute Position", frontLeftCANCoder.getAbsolutePosition());
        SmartDashboard.putNumber("CAN_Encoders/CAN Encoder Absolute Position", frontRightCANCoder.getAbsolutePosition());
        SmartDashboard.putNumber("CAN_Encoders/CAN Encoder Absolute Position", backLeftCANCoder.getAbsolutePosition());
        SmartDashboard.putNumber("CAN_Encoders/CAN Encoder Absolute Position", backRightCANCoder.getAbsolutePosition());

        SmartDashboard.putString("CAN_Encoders/CAN Encoder Position", "Position");
        SmartDashboard.putNumber("CAN_Encoders/CAN Encoder Position", frontLeftCANCoder.getPosition());
        SmartDashboard.putNumber("CAN_Encoders/CAN Encoder Position", frontRightCANCoder.getPosition());
        SmartDashboard.putNumber("ACAN_Encoders/CAN Encoder Position", backLeftCANCoder.getPosition());
        SmartDashboard.putNumber("CAN_Encoders/CAN Encoder Position", backRightCANCoder.getPosition());

        SmartDashboard.putString("CAN_Encoders/CAN Encoder Velocity", "Velocity");
        SmartDashboard.putNumber("CAN_Encoders/CAN Encoder Velocity", frontLeftCANCoder.getVelocity());
        SmartDashboard.putNumber("CAN_Encoders/CAN Encoder Velocity", frontRightCANCoder.getVelocity());
        SmartDashboard.putNumber("CAN_Encoders/CAN Encoder Velocity", backLeftCANCoder.getVelocity());
        SmartDashboard.putNumber("CAN_Encoders/CAN Encoder Velocity", backRightCANCoder.getVelocity());

        SmartDashboard.putString("CAN_Encoders/CAN Encoder Previous Error", "Previous Error");
        SmartDashboard.putString("CAN_Encoders/CAN Encoder Previous Error", frontLeftCANCoder.getLastError().toString());
        SmartDashboard.putString("CAN_Encoders/CAN Encoder Previous Error", frontRightCANCoder.getLastError().toString());
        SmartDashboard.putString("CAN_Encoders/CAN Encoder Previous Error", backLeftCANCoder.getLastError().toString());
        SmartDashboard.putString("CAN_Encoders/AN Encoder Previous Error", backRightCANCoder.getLastError().toString());

        SmartDashboard.putNumber("Gyroscope Angle", gyroscope.getAngle().toDegrees());
        // Added Gyroscope Info
        SmartDashboard.putString("Gyro Info", "Gyro");
        SmartDashboard.putNumber("Gyroscope Rate", gyroscope.getRate());
        SmartDashboard.putNumber("Gyroscope Unadjusted Rate", gyroscope.getUnadjustedRate());
        SmartDashboard.putNumber("Gyrscope Adjustement Angle (degrees)", gyroscope.getAdjustmentAngle().toDegrees());
        SmartDashboard.putNumber("Gyriscioe Unadjusted Angle (degrees)", gyroscope.getUnadjustedAngle().toDegrees());


// FLDmotor stuff
        SmartDashboard.putString("FLD Motor Information", "FLD Motor Information");
        SmartDashboard.putNumber("FLDMotor Current Set Speed", FLDMotor.get());
        SmartDashboard.putNumber("FLDMotor Applied Output Duty Cycle", FLDMotor.getAppliedOutput());
        SmartDashboard.putNumber("FLDMotor Bus Voltage", FLDMotor.getBusVoltage());
        SmartDashboard.putNumber("FLDMotor Max Rate Controller's Output Can Change", FLDMotor.getClosedLoopRampRate());
        SmartDashboard.putNumber("FLDMotor Output Current", FLDMotor.getOutputCurrent());
        SmartDashboard.putNumber("FLDMotor Ideal Votlage Value", FLDMotor.getVoltageCompensationNominalVoltage());
        SmartDashboard.putBoolean("FLDMotor Is Inverted?", FLDMotor.getInverted());
        SmartDashboard.putString("FLDMotor Previous Error", FLDMotor.getLastError().toString());
        SmartDashboard.putNumber("FLDMotor stuff", FLDMotor.getOpenLoopRampRate());





        frontLeftModule.updateState(TimedRobot.kDefaultPeriod);
        frontRightModule.updateState(TimedRobot.kDefaultPeriod);
        backLeftModule.updateState(TimedRobot.kDefaultPeriod);
        backRightModule.updateState(TimedRobot.kDefaultPeriod);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
        rotation *= 2.0 / Math.hypot(WHEELBASE, TRACKWIDTH);
        ChassisSpeeds speeds;
  
    if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                    Rotation2d.fromDegrees(gyroscope.getAngle().toDegrees()));
        } else {

            speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
       }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        frontLeftModule.setTargetVelocity(states[0].speedMetersPerSecond, states[0].angle.getRadians());
        frontRightModule.setTargetVelocity(states[1].speedMetersPerSecond, states[1].angle.getRadians());
        backLeftModule.setTargetVelocity(states[2].speedMetersPerSecond, states[2].angle.getRadians());
        backRightModule.setTargetVelocity(states[3].speedMetersPerSecond, states[3].angle.getRadians());
    }

    public void resetGyroscope() {
        gyroscope.setAdjustmentAngle(gyroscope.getUnadjustedAngle());
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveCommand());
    }
}
