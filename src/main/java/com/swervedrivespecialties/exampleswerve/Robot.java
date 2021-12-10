package com.swervedrivespecialties.exampleswerve;

import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;

/**
 * Entry point for program. Init methods and periodic methods are located here.
 * Schedules commands
 */
public class Robot extends TimedRobot {
        private static OI oi;
        private static DrivetrainSubsystem drivetrain;
        private final Timer timer = new Timer();
        private HolonomicDriveController controller;
        private Trajectory trajectory;
        /*
         * private Command autonomousCommand; private final RamseteController
         * ramseteController = new RamseteController(); private Timer timer;
         */

        // Initialization of CAN Sparks for limit setting
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

        TrajectoryConfig config;

        public static OI getOi() {
                return oi;
        }

        @Override
        public void robotInit() {
                oi = new OI();
                drivetrain = DrivetrainSubsystem.getInstance();
                backLeftAngle.setSmartCurrentLimit(25);
                backRightAngle.setSmartCurrentLimit(25);
                backLeftDrive.setSmartCurrentLimit(25);
                backRightDrive.setSmartCurrentLimit(25);
                frontLeftAngle.setSmartCurrentLimit(25);
                frontLeftDrive.setSmartCurrentLimit(25);
                frontRightAngle.setSmartCurrentLimit(25);
                frontRightDrive.setSmartCurrentLimit(25);
                drivetrain.resetGyroscope();
                ProfiledPIDController thetaController = new ProfiledPIDController(0, 0, 0,
                                new TrapezoidProfile.Constraints(5, 3.14));
                config = new TrajectoryConfig(3, 4).setKinematics(drivetrain.kinematics);
                trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                                List.of(new Translation2d(0.5, 0), new Translation2d(0.5, -0.4)),
                                new Pose2d(0.55, -0.5, Rotation2d.fromDegrees(0)), config);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);
                controller = new HolonomicDriveController(new PIDController(0.5, 0, 0), new PIDController(0.5, 0, 0),
                                thetaController);
        }

        @Override
        public void robotPeriodic() {
                Scheduler.getInstance().run();
        }

        @Override
        public void autonomousInit() {
                /*
                 * autonomousCommand = new autonomousDrive(); if (autonomousCommand != null)
                 * autonomousCommand.start();
                 * 
                 * timer = new Timer(); timer.start();
                 */
                timer.reset();
                timer.start();
        }

        @Override
        public void autonomousPeriodic() {
                /*
                 * if (timer.get() < trajectory.getTotalTimeSeconds()) { // Get the desired pose
                 * from the trajectory. var desiredPose = trajectory.sample(timer.get());
                 * 
                 * // Get the reference chassis speeds from the Ramsete controller. var
                 * refChassisSpeeds = ramseteController.calculate(drivetrain.getPose(),
                 * desiredPose);
                 * 
                 * // Set the linear and angular speeds. drivetrain.drive( new
                 * Translation2d(refChassisSpeeds.vxMetersPerSecond,
                 * refChassisSpeeds.vyMetersPerSecond), refChassisSpeeds.omegaRadiansPerSecond,
                 * false); } else { drivetrain.drive(new Translation2d(0, 0), 0.0, false); }
                 */
                if (timer.get() <= trajectory.getTotalTimeSeconds()) {
                        State desiredState = trajectory.sample(timer.get());

                        ChassisSpeeds targetChassisSpeeds = controller.calculate(drivetrain.getPose(), desiredState,
                                        desiredState.poseMeters.getRotation());
                        SwerveModuleState[] targetModuleStates = drivetrain.kinematics
                                        .toSwerveModuleStates(targetChassisSpeeds);
                        drivetrain.setModuleStates(targetModuleStates);
                }
        }
}