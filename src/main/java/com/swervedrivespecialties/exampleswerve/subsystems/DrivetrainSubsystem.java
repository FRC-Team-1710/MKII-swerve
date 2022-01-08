package com.swervedrivespecialties.exampleswerve.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.swervedrivespecialties.exampleswerve.RobotMap;
import com.swervedrivespecialties.exampleswerve.commands.DriveCommand;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModuleBuilder;
import org.frcteam2910.common.robot.drivers.NavX;

/** Subsystem for controlling the drivetrain */
public class DrivetrainSubsystem extends Subsystem {
    private static final double TRACKWIDTH = 23;
    private static final double WHEELBASE = 23;

    private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(127);
    private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(245);
    private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(240);
    private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(25);

    private static DrivetrainSubsystem instance;
        /** Front left swerve module object */
    private final SwerveModule frontLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
             .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER), FRONT_LEFT_ANGLE_OFFSET)
            .angleMotor(new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
            /** Front right swerve module object */
    private final SwerveModule frontRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER), FRONT_RIGHT_ANGLE_OFFSET)
            .angleMotor(new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
            /** Back left swerve module object */
    private final SwerveModule backLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER), BACK_LEFT_ANGLE_OFFSET)
            .angleMotor(new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
            /** Back right swerve module object */
    private final SwerveModule backRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER), BACK_RIGHT_ANGLE_OFFSET)
            .angleMotor(new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
        /** Ratios for swerve calculations */
    public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)
    );
    
    private final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometry;
    private Pose2d pose;

    public DrivetrainSubsystem() {
        gyroscope.calibrate();
        gyroscope.setInverted(true); // You might not need to invert the gyro
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(gyroscope.getAngle().toRadians()));
        frontLeftModule.setName("Front Left");
        frontRightModule.setName("Front Right");
        backLeftModule.setName("Back Left");
        backRightModule.setName("Back Right");
    }
    /** @return An instance of the DrivetrainSubsystem class */
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
        //Outputs encoder values to Shuffleboard
        SmartDashboard.putNumber("Front Left Module Angle", Math.toDegrees(frontLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Front Right Module Angle", Math.toDegrees(frontRightModule.getCurrentAngle()));
        SmartDashboard.putNumber("Back Left Module Angle", Math.toDegrees(backLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Back Right Module Angle", Math.toDegrees(backRightModule.getCurrentAngle()));

        SmartDashboard.putNumber("Gyroscope Angle", gyroscope.getAngle().toDegrees());

        frontLeftModule.updateState(TimedRobot.kDefaultPeriod);
        frontRightModule.updateState(TimedRobot.kDefaultPeriod);
        backLeftModule.updateState(TimedRobot.kDefaultPeriod);
        backRightModule.updateState(TimedRobot.kDefaultPeriod);
    }
    /** 
     * Method for controlling all modules
     * 
     * <p>The rotation value is multiplied by 2 and then divided by the hypotenuse of the WHEELBASE and TRACKWIDTH.
     * The values of the forward, strafe, and rotation are outputted to Shuffleboard. 
     * The speed is then calculated using the ChassisSpeeds class.
     * Finally, the speeds are put into an array and set using {@link #setTargetVelocity(speed, angle)}.
     * <p>Also, the gyroscope is be reset here when the correct button is pressed
     * 
     * @param translation The forward and strafe values sent through the Translation2d class
     * @param rotation The rotation value.
     * @param fieldOriented Boolean value that determines whether field orientation is used
     *
     */
    public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
        rotation *= 2.0 / Math.hypot(WHEELBASE, TRACKWIDTH);
        rotation *= .5;
        SmartDashboard.putNumber("Left Joystick x", translation.getX());
        SmartDashboard.putNumber("Left Joystick y", translation.getY());
        SmartDashboard.putNumber("Rotation", rotation);
        
        ChassisSpeeds speeds;
        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                    Rotation2d.fromDegrees(gyroscope.getAngle().toDegrees()));
        } else {
            speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        }
        
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        pose = odometry.update(new Rotation2d(gyroscope.getAngle().toRadians()), states);
        
        frontLeftModule.setTargetVelocity(states[0].speedMetersPerSecond, states[0].angle.getRadians());
        frontRightModule.setTargetVelocity(states[1].speedMetersPerSecond, states[1].angle.getRadians());
        backLeftModule.setTargetVelocity(states[2].speedMetersPerSecond, states[2].angle.getRadians());
        backRightModule.setTargetVelocity(states[3].speedMetersPerSecond, states[3].angle.getRadians());
        
        SmartDashboard.putString("Pose Get String", pose.toString());
        SmartDashboard.putNumber("Pose Get X", pose.getX());
        SmartDashboard.putNumber("Pose Get Y", pose.getY());
        SmartDashboard.putNumber("Pose Rotation", pose.getRotation().getDegrees());
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 1);
        pose = odometry.update(new Rotation2d(gyroscope.getAngle().toRadians()), desiredStates);
        frontLeftModule.setTargetVelocity(desiredStates[0].speedMetersPerSecond, desiredStates[0].angle.getRadians());
        frontRightModule.setTargetVelocity(desiredStates[1].speedMetersPerSecond, desiredStates[1].angle.getRadians());
        backLeftModule.setTargetVelocity(desiredStates[2].speedMetersPerSecond, desiredStates[2].angle.getRadians());
        backRightModule.setTargetVelocity(desiredStates[3].speedMetersPerSecond, desiredStates[3].angle.getRadians());
        SmartDashboard.putNumber("Pose Get X", pose.getX());
        SmartDashboard.putNumber("Pose Get Y", pose.getY());
        SmartDashboard.putNumber("Pose Rotation", pose.getRotation().getDegrees());
      }

    public void resetGyroscope() {
        gyroscope.setAdjustmentAngle(gyroscope.getUnadjustedAngle());
        pose = new Pose2d(new Translation2d(0,0), new Rotation2d(0));
        odometry.resetPosition(pose, new Rotation2d(gyroscope.getAngle().toRadians()));
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
      }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveCommand());
    }
}