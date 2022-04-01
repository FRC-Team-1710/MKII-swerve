package com.swervedrivespecialties.exampleswerve.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.swervedrivespecialties.exampleswerve.RobotMap;
import com.swervedrivespecialties.exampleswerve.commands.DriveCommand;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModuleBuilder;
import org.frcteam2910.common.robot.drivers.NavX;

/** Subsystem for controlling the drivetrain */
public class DrivetrainSubsystem extends Subsystem {
    private static final double TRACKWIDTH = Units.inchesToMeters(23);
    private static final double WHEELBASE = Units.inchesToMeters(23);

    private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(127);
    private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(248);
    private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(237);
    private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(26);

    private static DrivetrainSubsystem instance;

	private CANSparkMax backLeftAngle = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR,
	CANSparkMaxLowLevel.MotorType.kBrushless);
	private CANSparkMax backRightAngle = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR,
	CANSparkMaxLowLevel.MotorType.kBrushless);
	private CANSparkMax backLeftDrive = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
	CANSparkMaxLowLevel.MotorType.kBrushless);
	private CANSparkMax backRightDrive = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
	CANSparkMaxLowLevel.MotorType.kBrushless);
	private CANSparkMax frontLeftAngle = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR,
	CANSparkMaxLowLevel.MotorType.kBrushless);
	private CANSparkMax frontRightAngle = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR,
	CANSparkMaxLowLevel.MotorType.kBrushless);
	private CANSparkMax frontLeftDrive = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
	CANSparkMaxLowLevel.MotorType.kBrushless);
	private CANSparkMax frontRightDrive = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
	CANSparkMaxLowLevel.MotorType.kBrushless);
	

        /** Front left swerve module object */
    private final SwerveModule frontLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
             .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER), FRONT_LEFT_ANGLE_OFFSET)
            .angleMotor(frontLeftAngle,
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(frontLeftDrive,
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
            /** Front right swerve module object */
    private final SwerveModule frontRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER), FRONT_RIGHT_ANGLE_OFFSET)
            .angleMotor(frontRightAngle,
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(frontRightDrive,
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
            /** Back left swerve module object */
    private final SwerveModule backLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER), BACK_LEFT_ANGLE_OFFSET)
            .angleMotor(backLeftAngle,
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(backLeftDrive,
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
            /** Back right swerve module object */
    private final SwerveModule backRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER), BACK_RIGHT_ANGLE_OFFSET)
            .angleMotor(backRightAngle,
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(backRightDrive,
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
    public ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    private NetworkTableEntry backLeft;
    private NetworkTableEntry backRight;
    private NetworkTableEntry frontLeft;
    private NetworkTableEntry frontRight;
    private NetworkTableEntry gyroRot;

    public DrivetrainSubsystem() {
        gyroscope.calibrate();
        gyroscope.setInverted(true); // You might not need to invert the gyro
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(gyroscope.getAngle().toRadians()));
        frontLeftModule.setName("Front Left");
        frontRightModule.setName("Front Right");
        backLeftModule.setName("Back Left");
        backRightModule.setName("Back Right");

        frontRight = tab.add("Front Right Angle", 0).getEntry();
        frontLeft = tab.add("Front Left Angle", 0).getEntry();
        backRight = tab.add("Back Right Angle", 0).getEntry();
        backLeft = tab.add("Back Left Angle", 0).getEntry();
        gyroRot = tab.add("Gyro Angle", 0).getEntry();
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
        frontRight.setDouble(Math.toDegrees(frontRightModule.getCurrentAngle()));
        frontLeft.setDouble(Math.toDegrees(frontLeftModule.getCurrentAngle()));
        backRight.setDouble(Math.toDegrees(backRightModule.getCurrentAngle()));
        backLeft.setDouble(Math.toDegrees(backLeftModule.getCurrentAngle()));

        gyroRot.setDouble(gyroscope.getAngle().toDegrees());

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
        pose = odometry.update(Rotation2d.fromDegrees(gyroscope.getAngle().toDegrees()), states);
        
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
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 0.5);
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
        pose = new Pose2d(new Translation2d(3,6), new Rotation2d(0));
        odometry.resetPosition(pose, new Rotation2d(gyroscope.getAngle().toRadians()));
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
      }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveCommand());
    }


	public void setAmpLimit(){
		backLeftAngle.setSmartCurrentLimit(20);
		backRightAngle.setSmartCurrentLimit(20);
		backLeftDrive.setSmartCurrentLimit(20);
		backRightDrive.setSmartCurrentLimit(20);
		frontLeftAngle.setSmartCurrentLimit(20);
		frontLeftDrive.setSmartCurrentLimit(20);
		frontRightAngle.setSmartCurrentLimit(20);
		frontRightDrive.setSmartCurrentLimit(20);
	}
}