package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;

import com.pathplanner.lib.*;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

/**
 * Entry point for program. Init methods and periodic methods are located here.
 * Schedules commands
 */
public class Robot extends TimedRobot {
	private static OI oi;
	private static DrivetrainSubsystem drivetrain;
	private final Timer timer = new Timer();
	private HolonomicDriveController controller;
	private PathPlannerTrajectory trajectory;
	private NetworkTableEntry holoRot;
	private NetworkTableEntry chassisRot;
	/*
	 * private Command autonomousCommand; private final RamseteController
	 * ramseteController = new RamseteController(); private Timer timer;
	 */

	// Initialization of CAN Sparks for limit setting

	public static OI getOi() {
		return oi;
	}

	@Override
	public void robotInit() {
		oi = new OI();
		drivetrain = DrivetrainSubsystem.getInstance();
		holoRot = drivetrain.tab.add("Holonomic Rotation", "test").getEntry();
		chassisRot = drivetrain.tab.add("Chassis Rotation", 0).getEntry();
		drivetrain.resetGyroscope();
		drivetrain.setAmpLimit();
		/*
		 * SwerveDriveKinematicsConstraint constraint = new
		 * SwerveDriveKinematicsConstraint(drivetrain.kinematics, 6);
		 * CentripetalAccelerationConstraint constraint2 = new
		 * CentripetalAccelerationConstraint(5);
		 * config = new TrajectoryConfig(6, 6).setKinematics(drivetrain.kinematics);
		 * config.addConstraint(constraint);
		 * config.addConstraint(constraint2);
		 * trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0,
		 * Rotation2d.fromDegrees(0)),
		 * List.of(new Translation2d(0.4, 0), new Translation2d(0.5, -0.4)),
		 * new Pose2d(0.55, -0.4, Rotation2d.fromDegrees(0)), config);
		 */
		trajectory = PathPlanner.loadPath("Test 2", 0.4, .75);
		ProfiledPIDController thetaController = new ProfiledPIDController(1, 0.5, 0.1,
				new TrapezoidProfile.Constraints(Math.PI, Math.PI));
		thetaController.enableContinuousInput(-Math.PI, Math.PI);
		PIDController pid1 = new PIDController(0, 0, 0);
		PIDController pid2 = new PIDController(0, 0, 0);
		controller = new HolonomicDriveController(pid1, pid2, thetaController);
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
		drivetrain.resetGyroscope();
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
			PathPlannerState desiredState = (PathPlannerState) trajectory.sample(timer.get());
			
			ChassisSpeeds targetChassisSpeeds = controller.calculate(drivetrain.getPose(), desiredState,
					desiredState.holonomicRotation);
					
			SwerveModuleState[] targetModuleStates = drivetrain.kinematics
					.toSwerveModuleStates(targetChassisSpeeds);
			drivetrain.setModuleStates(targetModuleStates);
			holoRot.setString(desiredState.holonomicRotation.toString());
			chassisRot.setDouble(targetChassisSpeeds.omegaRadiansPerSecond);
		}
	}
}