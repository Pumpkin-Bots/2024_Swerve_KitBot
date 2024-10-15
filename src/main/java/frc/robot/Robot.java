// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SmartDashboardConstants;
import frc.robot.Constants.OperatorConstants.ControllerConfigMode;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // Choose if we want to use single or dual XBOX Controller Mode
  public ControllerConfigMode m_controllerModeSelected;
  private final SendableChooser<String> m_controllerModeChooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    m_controllerModeChooser.setDefaultOption("Single Controller", OperatorConstants.ControllerConfigMode.SINGLE.toString());
    m_controllerModeChooser.addOption("Dual Controller", OperatorConstants.ControllerConfigMode.DUAL.toString());
    SmartDashboard.putData("controller_mode", m_controllerModeChooser);
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    maybeUpdateControllerBindings();
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}

  private void maybeUpdateControllerBindings() {
    SendableChooser<String> controllerModeChooser = (SendableChooser<String>) SmartDashboard.getData(SmartDashboardConstants.kControllerMode);
    ControllerConfigMode selectedMode = ControllerConfigMode.fromString(controllerModeChooser.getSelected()); // Get the currently selected mode
    
    // Compare to previously selected value and remap bindings only if value changed or if not previously set
    if (selectedMode != m_controllerModeSelected) {
      System.out.printf(String.format("Controller mode selected: %s", selectedMode));
      selectedMode = m_controllerModeSelected;
      m_robotContainer.configureBindings(selectedMode);

    }
  }
}
