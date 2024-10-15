// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
//import frc.robot.subsystems.PWMDrivetrain;
//import frc.robot.subsystems.PWMLauncher;
//start kitbot additions
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.ControllerConfigMode;
import frc.robot.Constants.SmartDashboardConstants;
//import frc.robot.commands.Autos;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
//import frc.robot.subsystems.CANDrivetrain;
import frc.robot.subsystems.CANLauncher;
// import frc.robot.subsystems.PWMDrivetrain;
// import frc.robot.subsystems.PWMLauncher;


public class RobotContainer {
  // Choose if we want to use single or dual XBOX Controller Mode
  private ControllerConfigMode m_controllerModeSelected;
  private final SendableChooser<String> m_controllerModeChooser = new SendableChooser<>();

//start kitbot additions
// private final PWMDrivetrain m_drivetrain = new PWMDrivetrain();
//private final CANDrivetrain m_drivetrain = new CANDrivetrain();
// private final PWMLauncher m_launcher = new PWMLauncher();
private final CANLauncher m_launcher = new CANLauncher();
//end kitbot additions


  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  
  //start kitbot addition
  //commenting out drive controller since already put in swerve code
  //private final CommandXboxController m_driverController =
  //new CommandXboxController(OperatorConstants.kDriverControllerPort);
  // this is for joystick for operator
  private final CommandXboxController m_operatorController =
  new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  //end kitbot addition

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings(ControllerConfigMode mode) {
    boolean isOneControllerDriving = mode == OperatorConstants.ControllerConfigMode.SINGLE;
    
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

//start kitbot additions
//need to resolve that both sets of code use left bumper

 /*Create an inline sequence to run when the operator presses and holds the A (green) button. Run the PrepareLaunch
     * command for 1 seconds and then run the LaunchNote command */
    if(isOneControllerDriving){
      joystick
          .x()
          .whileTrue(
              new PrepareLaunch(m_launcher)
                  .withTimeout(LauncherConstants.kLauncherDelay)
                  .andThen(new LaunchNote(m_launcher))
                  .handleInterrupt(() -> m_launcher.stop()));
    }
    else {
      m_operatorController
          .x()
          .whileTrue(
              new PrepareLaunch(m_launcher)
                  .withTimeout(LauncherConstants.kLauncherDelay)
                  .andThen(new LaunchNote(m_launcher))
                  .handleInterrupt(() -> m_launcher.stop()));
    }

    // Set up a binding to run the intake command while the operator is pressing and holding the
    // left Bumper
    if(isOneControllerDriving){
        joystick.y().whileTrue(m_launcher.getIntakeCommand());
    } else {
        m_operatorController.y().whileTrue(m_launcher.getIntakeCommand());
    }
 //end kitbot additions

  }

  public RobotContainer() {
    configureSmartDashboardOptions();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void maybeUpdateControllerBindings() {
    // Get the currently selected mode
    SendableChooser<String> controllerModeChooser = (SendableChooser<String>) SmartDashboard.getData(SmartDashboardConstants.kControllerMode);
    ControllerConfigMode selectedMode = ControllerConfigMode.fromString(controllerModeChooser.getSelected()); 
    
    // Compare to previously selected value and remap bindings only if value changed or if not previously set
    if (selectedMode != m_controllerModeSelected) {
      System.out.printf(String.format("Controller mode selected: %s", selectedMode));
      selectedMode = m_controllerModeSelected;
      configureBindings(selectedMode);
    }
  }

  private void configureSmartDashboardOptions() {
    m_controllerModeChooser.setDefaultOption("Single Controller", OperatorConstants.ControllerConfigMode.SINGLE.toString());
    m_controllerModeChooser.addOption("Dual Controller", OperatorConstants.ControllerConfigMode.DUAL.toString());
    SmartDashboard.putData("controller_mode", m_controllerModeChooser);
  }
}
