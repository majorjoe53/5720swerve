// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveDiagnostics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import edu.wpi.first.wpilibj.PowerDistribution;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  PowerDistribution m_pdh = new PowerDistribution(9, ModuleType.kRev);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putBoolean("clear faults", false);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, false),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

  public void periodicDiagnostics() {
    DriveDiagnostics diagnostics = m_robotDrive.getDiagnostics();

    SmartDashboard.putNumber("FL Voltage", diagnostics.frontLeftInputVoltage);
    SmartDashboard.putNumber("FL Current", diagnostics.frontLeftOutputCurrent);
    SmartDashboard.putNumber("FL Temp", diagnostics.frontLeftTemperature);
    SmartDashboard.putBoolean("FL sHasReset", diagnostics.frontLeftStickyFaults.hasReset);
    SmartDashboard.putBoolean("FL sBrownout", diagnostics.frontLeftStickyFaults.brownout);
    SmartDashboard.putBoolean("FL sOverCurrent", diagnostics.frontLeftStickyFaults.overCurrent);
    SmartDashboard.putBoolean("FL sDRV", diagnostics.frontLeftStickyFaults.gateDriverFault);
    SmartDashboard.putBoolean("FL sMotor", diagnostics.frontLeftStickyFaults.motorFault);
    SmartDashboard.putBoolean("FL sSensor", diagnostics.frontLeftStickyFaults.sensorFault);
    SmartDashboard.putBoolean("FL sStall", diagnostics.frontLeftStickyFaults.stall);
    SmartDashboard.putBoolean("FL sIWDT", diagnostics.frontLeftStickyFaults.iwdtReset);
    SmartDashboard.putBoolean("FL sCANRX", diagnostics.frontLeftStickyFaults.canRxFault);
    SmartDashboard.putBoolean("FL sCANTX", diagnostics.frontLeftStickyFaults.canTxFault);
    //
    SmartDashboard.putBoolean("FL hasReset", diagnostics.frontLeftFaults.hasReset);
    SmartDashboard.putBoolean("FL brownout", diagnostics.frontLeftFaults.brownout);
    SmartDashboard.putBoolean("FL overCurrent", diagnostics.frontLeftFaults.overCurrent);
    SmartDashboard.putBoolean("FL DRV", diagnostics.frontLeftFaults.gateDriverFault);
    SmartDashboard.putBoolean("FL motor", diagnostics.frontLeftFaults.motorFault);
    SmartDashboard.putBoolean("FL sensor", diagnostics.frontLeftFaults.sensorFault);
    SmartDashboard.putBoolean("FL stall", diagnostics.frontLeftFaults.stall);
    SmartDashboard.putBoolean("FL IWDT", diagnostics.frontLeftFaults.iwdtReset);
    SmartDashboard.putBoolean("FL CANRX", diagnostics.frontLeftFaults.canRxFault);
    SmartDashboard.putBoolean("FL CANTX", diagnostics.frontLeftFaults.canTxFault);

    
    SmartDashboard.putNumber("FR Voltage", diagnostics.frontRightInputVoltage);
    SmartDashboard.putNumber("FR Current", diagnostics.frontRightOutputCurrent);
    SmartDashboard.putNumber("FR Temp", diagnostics.frontRightTemperature);
    SmartDashboard.putBoolean("FR sHasReset", diagnostics.frontRightStickyFaults.hasReset);
    SmartDashboard.putBoolean("FR sBrownout", diagnostics.frontRightStickyFaults.brownout);
    SmartDashboard.putBoolean("FR sOverCurrent", diagnostics.frontRightStickyFaults.overCurrent);
    SmartDashboard.putBoolean("FR sDRV", diagnostics.frontRightStickyFaults.gateDriverFault);
    SmartDashboard.putBoolean("FR sMotor", diagnostics.frontRightStickyFaults.motorFault);
    SmartDashboard.putBoolean("FR sSensor", diagnostics.frontRightStickyFaults.sensorFault);
    SmartDashboard.putBoolean("FR sStall", diagnostics.frontRightStickyFaults.stall);
    SmartDashboard.putBoolean("FR sIWDT", diagnostics.frontRightStickyFaults.iwdtReset);
    SmartDashboard.putBoolean("FR sCANRX", diagnostics.frontRightStickyFaults.canRxFault);
    SmartDashboard.putBoolean("FR sCANTX", diagnostics.frontRightStickyFaults.canTxFault);
    //
    SmartDashboard.putBoolean("FR hasReset", diagnostics.frontRightFaults.hasReset);
    SmartDashboard.putBoolean("FR brownout", diagnostics.frontRightFaults.brownout);
    SmartDashboard.putBoolean("FR overCurrent", diagnostics.frontRightFaults.overCurrent);
    SmartDashboard.putBoolean("FR DRV", diagnostics.frontRightFaults.gateDriverFault);
    SmartDashboard.putBoolean("FR motor", diagnostics.frontRightFaults.motorFault);
    SmartDashboard.putBoolean("FR sensor", diagnostics.frontRightFaults.sensorFault);
    SmartDashboard.putBoolean("FR stall", diagnostics.frontRightFaults.stall);
    SmartDashboard.putBoolean("FR IWDT", diagnostics.frontRightFaults.iwdtReset);
    SmartDashboard.putBoolean("FR CANRX", diagnostics.frontRightFaults.canRxFault);
    SmartDashboard.putBoolean("FR CANTX", diagnostics.frontRightFaults.canTxFault);

    SmartDashboard.putNumber("RL Voltage", diagnostics.rearLeftInputVoltage);
    SmartDashboard.putNumber("RL Current", diagnostics.rearLeftOutputCurrent);
    SmartDashboard.putNumber("RL Temp", diagnostics.rearLeftTemperature);
    SmartDashboard.putBoolean("RL sHasReset", diagnostics.rearLeftStickyFaults.hasReset);
    SmartDashboard.putBoolean("RL sBrownout", diagnostics.rearLeftStickyFaults.brownout);
    SmartDashboard.putBoolean("RL sOverCurrent", diagnostics.rearLeftStickyFaults.overCurrent);
    SmartDashboard.putBoolean("RL sDRV", diagnostics.rearLeftStickyFaults.gateDriverFault);
    SmartDashboard.putBoolean("RL sMotor", diagnostics.rearLeftStickyFaults.motorFault);
    SmartDashboard.putBoolean("RL sSensor", diagnostics.rearLeftStickyFaults.sensorFault);
    SmartDashboard.putBoolean("RL sStall", diagnostics.rearLeftStickyFaults.stall);
    SmartDashboard.putBoolean("RL sIWDT", diagnostics.rearLeftStickyFaults.iwdtReset);
    SmartDashboard.putBoolean("RL sCANRX", diagnostics.rearLeftStickyFaults.canRxFault);
    SmartDashboard.putBoolean("RL sCANTX", diagnostics.rearLeftStickyFaults.canTxFault);
    //
    SmartDashboard.putBoolean("RL hasReset", diagnostics.rearLeftFaults.hasReset);
    SmartDashboard.putBoolean("RL brownout", diagnostics.rearLeftFaults.brownout);
    SmartDashboard.putBoolean("RL overCurrent", diagnostics.rearLeftFaults.overCurrent);
    SmartDashboard.putBoolean("RL DRV", diagnostics.rearLeftFaults.gateDriverFault);
    SmartDashboard.putBoolean("RL motor", diagnostics.rearLeftFaults.motorFault);
    SmartDashboard.putBoolean("RL sensor", diagnostics.rearLeftFaults.sensorFault);
    SmartDashboard.putBoolean("RL stall", diagnostics.rearLeftFaults.stall);
    SmartDashboard.putBoolean("RL IWDT", diagnostics.rearLeftFaults.iwdtReset);
    SmartDashboard.putBoolean("RL CANRX", diagnostics.rearLeftFaults.canRxFault);
    SmartDashboard.putBoolean("RL CANTX", diagnostics.rearLeftFaults.canTxFault);

    SmartDashboard.putNumber("RR Voltage", diagnostics.rearRightInputVoltage);
    SmartDashboard.putNumber("RR Current", diagnostics.rearRightOutputCurrent);
    SmartDashboard.putNumber("RR Temp", diagnostics.rearRightTemperature);
    SmartDashboard.putBoolean("RR sHasReset", diagnostics.rearRightStickyFaults.hasReset);
    SmartDashboard.putBoolean("RR sBrownout", diagnostics.rearRightStickyFaults.brownout);
    SmartDashboard.putBoolean("RR sOverCurrent", diagnostics.rearRightStickyFaults.overCurrent);
    SmartDashboard.putBoolean("RR sDRV", diagnostics.rearRightStickyFaults.gateDriverFault);
    SmartDashboard.putBoolean("RR sMotor", diagnostics.rearRightStickyFaults.motorFault);
    SmartDashboard.putBoolean("RR sSensor", diagnostics.rearRightStickyFaults.sensorFault);
    SmartDashboard.putBoolean("RR sStall", diagnostics.rearRightStickyFaults.stall);
    SmartDashboard.putBoolean("RR sIWDT", diagnostics.rearRightStickyFaults.iwdtReset);
    SmartDashboard.putBoolean("RR sCANRX", diagnostics.rearRightStickyFaults.canRxFault);
    SmartDashboard.putBoolean("RR sCANTX", diagnostics.rearRightStickyFaults.canTxFault);
    //
    SmartDashboard.putBoolean("RR hasReset", diagnostics.rearRightFaults.hasReset);
    SmartDashboard.putBoolean("RR brownout", diagnostics.rearRightFaults.brownout);
    SmartDashboard.putBoolean("RR overCurrent", diagnostics.rearRightFaults.overCurrent);
    SmartDashboard.putBoolean("RR DRV", diagnostics.rearRightFaults.gateDriverFault);
    SmartDashboard.putBoolean("RR motor", diagnostics.rearRightFaults.motorFault);
    SmartDashboard.putBoolean("RR sensor", diagnostics.rearRightFaults.sensorFault);
    SmartDashboard.putBoolean("RR stall", diagnostics.rearRightFaults.stall);
    SmartDashboard.putBoolean("RR IWDT", diagnostics.rearRightFaults.iwdtReset);
    SmartDashboard.putBoolean("RR CANRX", diagnostics.rearRightFaults.canRxFault);
    SmartDashboard.putBoolean("RR CANTX", diagnostics.rearRightFaults.canTxFault);

    // Clear faults
    if (SmartDashboard.getBoolean("clear faults", false)) {
        SmartDashboard.putBoolean("clear faults", false);
        m_robotDrive.clearAllStickyFaults();
    }

    // Other
    SmartDashboard.putNumber("bus voltage", m_pdh.getVoltage());
  }
}
