// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveXMeters;
import frc.robot.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private RobotContainer robot;
  private Trajectory autoTrajectory;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    PowerDistribution pdp = new PowerDistribution();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    robot = RobotContainer.getInstance();
    pdp.clearStickyFaults();
    autoTrajectory = RobotContainer.getTrajectory();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  RamseteController controller;
  Timer autoTimer;
  @Override
  public void autonomousInit() {
    controller = new RamseteController();
    autoTimer = new Timer();
    autoTimer.start();
    if(m_chooser.getSelected().equals(kDefaultAuto)) {
      CommandScheduler.getInstance().schedule(new DriveXMeters(-Units.FeetToMeters(3), 0.2, 0.1));
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch(m_chooser.getSelected()) {
      case kCustomAuto:
        Trajectory.State goal = autoTrajectory.sample(autoTimer.get());
        ChassisSpeeds adjustedSpeeds = controller.calculate(Drivetrain.ODOMETRY.getPoseMeters(), goal);
        DifferentialDriveWheelSpeeds wheelSpeeds = Drivetrain.KINEMATICS.toWheelSpeeds(adjustedSpeeds);
        double left = wheelSpeeds.leftMetersPerSecond;
        double right = wheelSpeeds.rightMetersPerSecond;
        SmartDashboard.putNumber("leftTrajSpeed", left);
        SmartDashboard.putNumber("rightTrajSpeed", right);
        left = Drivetrain.FEEDFORWARD.calculate(left);
        right = Drivetrain.FEEDFORWARD.calculate(right);
        SmartDashboard.putNumber("leftTrajVoltage", left);
        SmartDashboard.putNumber("rightTrajVoltage", right);
        SmartDashboard.putNumber("Trajectory Duration", autoTrajectory.getTotalTimeSeconds());
        Drivetrain.setOpenLoop(left/Constants.kMaxVoltage, right/Constants.kMaxVoltage);
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    Drivetrain.getInstance().setDefaultCommand(new Drive(Drive.State.CheesyDriveOpenLoop));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
