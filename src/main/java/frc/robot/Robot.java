// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private static final String kAuton1 = "1. Drive Forward";
  private static final String kAuton2 = "2. Back, Drop, Forward";
  //private static final String kAuton3 = "3. B, D, F, B, Balance";
  //private static final String kAuton4 = "Unused";
  //private static final String kAuton5 = "ZZZ KKEP UNUSED";
  //private static final String kAuton6 = "balance test";

  private String m_autoSelected; // This selects between the two autonomous
  public SendableChooser<String> m_chooser = new SendableChooser<>(); 

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    m_chooser.addOption("1. pick up cone inside robot and drive out of comm", kAuton1);
    m_chooser.setDefaultOption("2. Drop cone on mid and drive out of comm", kAuton2);
    // m_chooser.addOption("3. Drop cone on mid, drive and balance on charge station", kAuton3);
    // m_chooser.addOption("4. WIP DO NOT USE", kAuton4);
    // m_chooser.addOption("5. ZZZ KEEP UNUSED", kAuton5);
    // m_chooser.addOption("6. balance test", kAuton6);
    // prettyLights1.setLEDs(.01);

    SmartDashboard.putData(m_chooser); // displays the auton options in shuffleboard, put in init block


  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_autoSelected = m_chooser.getSelected(); // pulls auton option selected from shuffleboard
    SmartDashboard.putString("Current Auton:", m_autoSelected);

    switch (m_autoSelected) {

      case kAuton1: 



  }
}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}