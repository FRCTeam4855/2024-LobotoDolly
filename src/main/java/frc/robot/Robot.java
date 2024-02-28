// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants.ArmSetpoint;
import frc.robot.commands.ArmSetpointCommand;
import frc.robot.commands.IntakePickupCommand;
import frc.robot.commands.IntakeDropCommand;
import frc.robot.commands.IntakeStopCommand;
import frc.robot.commands.FlywheelStartCommand;
import frc.robot.commands.FlywheelStopCommand;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.filter.Debouncer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  // public boolean intakeSensor, useSensor;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  // private DriveSubsystem driveSubsystem;
  private FlywheelSubsystem flywheelSubsystem;
  // private IntakeSubsystem intakeSubsystem;
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  // IntakeSubsystem intakeSubsystem;
  private static final String kAuton1 = "1. Drive Forward";
  private static final String kAuton2 = "2. Back, Drop, Forward";
  // private static final String kAuton3 = "3. B, D, F, B, Balance";
  // private static final String kAuton4 = "Unused";
  // private static final String kAuton5 = "ZZZ KKEP UNUSED";
  // private static final String kAuton6 = "balance test";

  private String m_autoSelected; // This selects between the two autonomous
  public SendableChooser<String> m_chooser = new SendableChooser<>();
  ArmSetpoint currentSetpoint;
  ArmPivot armPivot = new ArmPivot();

  @Override
  public void robotInit() {
    // intakeSubsystem = new IntakeSubsystem();
    // Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    // SmartDashboard.putString("Current Auton:", m_autoSelected);
    // if (isReal()) {
    // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    // Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    // Pose2d poseA, poseB, poseC;
    // //Logger.recordOutput("MyPose2d", poseA);
    // //Logger.recordOutput("MyPose2dArray", poseA, poseB);
    // //Logger.recordOutput("MyPose2dArray", new Pose2d[] { poseA, poseB });
    // new PowerDistribution(1, ModuleType.kRev);
    // // Enables power distribution logging
    // } else {
    // setUseTiming(false); // Run as fast as possible
    // String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from
    // AdvantageScope (or prompt the user)
    // Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,
    // "_sim"))); // Save outputs to a new log
    // }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in
    // the "Understanding Data Flow" page
    // Logger.start(); // Start logging! No more data receivers, replay sources, or
    // metadata values may
    // be added.

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    // intakeSubsystem = new IntakeSubsystem();
    m_robotContainer = new RobotContainer();
    m_robotContainer.m_robotDrive.m_gyro.reset();
    // driveSubsystem = new DriveSubsystem();
    // flywheelSubsystem = new FlywheelSubsystem();

    m_robotContainer.intakeSubsystem.IntakeStop();
    armPivot.initPivot();
    // flywheelSubsystem.FlywheelStop();

    m_chooser.setDefaultOption("1. Drive Backwards", kAuton1);
    m_chooser.addOption("2. Epic 50 Note auto", kAuton2);
    // m_chooser.addOption("3. Drop cone on mid, drive and balance on charge
    // station", kAuton3);
    // m_chooser.addOption("4. WIP DO NOT USE", kAuton4);
    // m_chooser.addOption("5. ZZZ KEEP UNUSED", kAuton5);
    // m_chooser.addOption("6. balance test", kAuton6);
    // prettyLights1.setLEDs(.01);

    SmartDashboard.putData(m_chooser); // displays the auton options in shuffleboard, put in init block

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("ArmEncoder", armPivot.getPivotPostion());
    // SmartDashboard.putNumber("Proximity", Intake.noteSensor.getProximity());

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("GyroYaw", m_robotContainer.m_robotDrive.m_gyro.getYaw());
    SmartDashboard.putNumber("GyroAngle", m_robotContainer.m_robotDrive.m_gyro.getAngle());
    SmartDashboard.putBoolean("Field Oriented", m_robotContainer.fieldOriented);

    // m_robotContainer.intakeSubsystem.intakeSensor=m_robotContainer.intakeSubsystem.m_noteSensor.get();

    // SmartDashboard.putNumber("FlywheelRunning?", flywheelSubsystem.runFlywheel);
    // SmartDashboard.putNumber("Flywheel Setpoint", flywheelSubsystem.setpoint);
    // flywheelSubsystem.periodicFlywheel();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {

    // SmartDashboard.putBoolean("Proximity",
    // m_robotContainer.intakeSubsystem.m_noteSensor.get());
    SmartDashboard.putBoolean("Proximity", m_robotContainer.intakeSubsystem.intakeSensor);

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.m_robotDrive.m_gyro.reset();
    m_autoSelected = m_chooser.getSelected(); // pulls auton option selected from shuffleboard
    SmartDashboard.putString("Current Auton:", m_autoSelected);

    switch (m_autoSelected) {

      case kAuton1:
        m_autonomousCommand = m_robotContainer.getAutoBackwardsCommand();
        CommandScheduler.getInstance()
            .schedule(m_autonomousCommand);
      case kAuton2:
        m_autonomousCommand = m_robotContainer.getSquareCommand();
        CommandScheduler.getInstance()
            .schedule(m_autonomousCommand);

    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_robotContainer.m_robotDrive.m_gyro.reset();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // SmartDashboard.putBoolean("Proximity",
    // m_robotContainer.intakeSubsystem.m_noteSensor.get());
    SmartDashboard.putBoolean("Proximity", m_robotContainer.intakeSubsystem.intakeSensor);
    //Driver Controls
    if (m_robotContainer.m_driverControllerRight.getRawButtonPressed(2)) { //Toggles Field Oriented Mode
      if (m_robotContainer.fieldOriented == true) {
        m_robotContainer.fieldOriented = false;
      } else {
        m_robotContainer.fieldOriented = true;
      }
    }
    if (m_robotContainer.m_driverControllerLeft.getRawButtonPressed(2)) { //Resets the Gyro
      m_robotContainer.m_robotDrive.m_gyro.reset();
    }
    //Operator Controls
    if (m_robotContainer.m_operatorController.getRawButtonPressed(kIntakePickup_LB)) { //Runs the Intake
      CommandScheduler.getInstance()
          .schedule((new IntakePickupCommand(m_robotContainer.intakeSubsystem)));
    }

    if (m_robotContainer.m_operatorController.getRawButtonPressed(kIntakeStop_Back)) { //Stops the Intake
      CommandScheduler.getInstance()
          .schedule((new IntakeStopCommand(m_robotContainer.intakeSubsystem)));
    }

    if (m_robotContainer.m_operatorController.getRawButtonPressed(kIntakeDrop_RB)) { //Reverses the Intake
      CommandScheduler.getInstance()
          .schedule((new IntakeDropCommand(m_robotContainer.intakeSubsystem)));
    }

    if (m_robotContainer.m_operatorController.getRawButton(kArmSetpoint1Button_A)) { //Sets the Arm to intake position and runs the intake
      CommandScheduler.getInstance().schedule(
          (new ArmSetpointCommand(armPivot, ArmSetpoint.One, currentSetpoint))
              .andThen(new IntakePickupCommand(m_robotContainer.intakeSubsystem)));
      currentSetpoint = ArmSetpoint.One;
    }
    if (m_robotContainer.m_operatorController.getRawButton(kArmSetpoint2Button_B)) { //Spear position not in use
      CommandScheduler.getInstance()
          .schedule((new ArmSetpointCommand(armPivot, ArmSetpoint.Two, currentSetpoint)));
      currentSetpoint = ArmSetpoint.Two;

    }
    if (m_robotContainer.m_operatorController.getRawButton(kArmSetpoint3Button_X)) { //Goes to amp position and runs the flywheel and intake.
      CommandScheduler.getInstance()
          .schedule((new ArmSetpointCommand(armPivot, ArmSetpoint.Three, currentSetpoint))
          .andThen(new FlywheelStartCommand(flywheelSubsystem))
          .andThen(new IntakePickupCommand(m_robotContainer.intakeSubsystem)));
      currentSetpoint = ArmSetpoint.Three;

    }
    if (m_robotContainer.m_operatorController.getRawButton(kArmSetpoint4Button_Y)) { //Goes to speaker position and runs the flywheel and intake
      CommandScheduler.getInstance().schedule(
          (new ArmSetpointCommand(armPivot, ArmSetpoint.Four, currentSetpoint))
              .andThen(new FlywheelStartCommand(flywheelSubsystem))
              .andThen(new WaitCommand(2))
              .andThen(new IntakePickupCommand(m_robotContainer.intakeSubsystem)));
      currentSetpoint = ArmSetpoint.Four;

    }
    if (m_robotContainer.m_operatorController.getLeftTriggerAxis() > .5) { //Controls the flywheel
      CommandScheduler.getInstance()
          .schedule((new FlywheelStartCommand(m_robotContainer.flywheelSubsystem)));
    } else {
      CommandScheduler.getInstance()
          .schedule((new FlywheelStopCommand(m_robotContainer.flywheelSubsystem)));
    }
    if (m_robotContainer.m_operatorController.getRawButtonPressed(kFlywheelStart_Start)) {
      CommandScheduler.getInstance()
          .schedule((new FlywheelStartCommand(m_robotContainer.flywheelSubsystem)));
      // flywheelSubsystem.flywheelRunning = true;
      // } else {
      // flywheelSubsystem.flywheelRunning = false;
    }
    if (m_robotContainer.m_operatorController.getRightTriggerAxis() > .5) { //Runs the intake
      m_robotContainer.intakeSubsystem.m_intakeSparkMax.set(.5);
      // CommandScheduler.getInstance()
      // .schedule((new FlywheelStartCommand(m_robotContainer.flywheelSubsystem)));
      // flywheelSubsystem.flywheelRunning = true;
      // } else {
      // flywheelSubsystem.flywheelRunning = false;
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}