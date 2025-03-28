// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.DrivetrainConstants;
//import frc.robot.subsystems.PWMDrivetrain;
//import frc.robot.subsystems.PWMLauncher;
//start kitbot additions
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
import frc.robot.subsystems.CANdleSystem;


public class RobotContainer {
  // Choose if we want to use single or dual XBOX Controller Mode
  private ControllerConfigMode m_controllerModeSelected;
  private final SendableChooser<String> m_controllerModeChooser = new SendableChooser<>();

  double tx = LimelightHelpers.getTX("");
  
//start kitbot additions
// private final PWMDrivetrain m_drivetrain = new PWMDrivetrain();
//private final CANDrivetrain m_drivetrain = new CANDrivetrain();
// private final PWMLauncher m_launcher = new PWMLauncher();
private final CANLauncher m_launcher = new CANLauncher();
//end kitbot additions


  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * DrivetrainConstants.kSpeedLimit; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(OperatorConstants.kDriverControllerPort); // My joystick
  
  //start kitbot addition
  //commenting out drive controller since already put in swerve code
  //private final CommandXboxController m_driverController =
  //new CommandXboxController(OperatorConstants.kDriverControllerPort);
  // this is for joystick for operator
  private final CommandXboxController m_operatorController =
  new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  //end kitbot addition

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

   /* Path follower */
  public Command runAuto = drivetrain.getAutoPath("AutoOne");

  private final Telemetry logger = new Telemetry(MaxSpeed);

  CANdleSystem CANdle = new CANdleSystem(joystick);

  
  /*
   * adds an exponential curve to joystick from -1 to 1
   * smoother at lower values, higher slope at higher values 
   */
  
  private double velocityCurveTranslate(double joystickInput){ 
    if(joystickInput > 0){
      return Math.pow(joystickInput, 2.1);
    } else if (joystickInput < 0){
      return -Math.pow(-joystickInput, 2.1);
    } else {
      return 0;
    }
  }

  private void configureBindings(ControllerConfigMode mode) {

    boolean isOneControllerDriving = mode == OperatorConstants.ControllerConfigMode.SINGLE;

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-velocityCurveTranslate(joystick.getLeftY()) * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-velocityCurveTranslate(joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    //joystick.b().whileTrue(drivetrain
    //    .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    drivetrain.registerTelemetry(logger::telemeterize);


    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    //start kitbot additions
//need to resolve that both sets of code use left bumper

 /*Create an inline sequence to run when the operator presses and holds the A (green) button. Run the PrepareLaunch
     * command for 1 seconds and then run the LaunchNote command */
    if(isOneControllerDriving){
      joystick
          .x()
          .whileTrue(
              new PrepareLaunch(m_launcher)
                  .handleInterrupt(() -> m_launcher.stop()));              
    } else {
      m_operatorController
          .x()
          .whileTrue(
              new PrepareLaunch(m_launcher)
                  .handleInterrupt(() -> m_launcher.stop()));
    }

    
    if(isOneControllerDriving){
      joystick
          .b()
          .whileTrue(
              new LaunchNote(m_launcher)
              .handleInterrupt(() -> m_launcher.stop()));
    } else {
      m_operatorController
          .b()
          .whileTrue(
              new LaunchNote(m_launcher)
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



 /* old code with Tyler's limelight addition 
   drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-velocityCurveTranslate(joystick.getLeftY()) * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-velocityCurveTranslate(joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(joystick.getRightX() * MaxAngularRate + LimelightHelpers.getTX("") * 0.1
            ) // Drive counterclockwise with negative X (left)
        ));
      //  above code follows target rotationally using limelight commands
 */
 /* old code
    joystick.rightBumper().whileTrue(
      drivetrain.applyRequest(() -> drive.withVelocityX(-velocityCurveTranslate(joystick.getLeftY()) * MaxSpeed) // Drive forward with
                                                                                          // negative Y (forward)
            .withVelocityY(-velocityCurveTranslate(joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(joystick.getRightX() * MaxAngularRate + (LimelightHelpers.getTX("") * 0.1)
            ) // Drive counterclockwise with negative X (left)
        ));
 */ 

    //joystick.button(8).whileTrue(CANdle.getChangeAnimationCommand()); // menu button (3 lines)    
  }

  public RobotContainer() {
    configureSmartDashboardOptions();
  }

  public Command getAutonomousShootAndLeaveCommand() {
    return new PrepareLaunch(m_launcher)
        .withTimeout(5)
        .andThen(new LaunchNote(m_launcher)
        .withTimeout(7))
        .andThen(drivetrain.applyRequest(() -> drive.withVelocityX(1))
        .withTimeout(2))
        ;
  
  }

  public Command getAutonomousShootCommand() {
    return new PrepareLaunch(m_launcher)
        .withTimeout(5)
        .andThen(new LaunchNote(m_launcher)
        .withTimeout(1))
        ;
  
  }

  public Command getAutonomousLeaveCommand() {
    return drivetrain.applyRequest(() -> drive.withVelocityX(1))
        .withTimeout(2)
        ;
  
  }

  public void maybeUpdateControllerBindings() {
    // Get the currently selected mode
    SendableChooser<String> controllerModeChooser = (SendableChooser<String>) SmartDashboard.getData(SmartDashboardConstants.kControllerMode);
    ControllerConfigMode selectedMode = ControllerConfigMode.fromString(controllerModeChooser.getSelected()); 
    
    // Compare to previously selected value and remap bindings only if value changed or if not previously set
    if (selectedMode != m_controllerModeSelected) {
      System.out.printf(String.format("Controller mode selected: %s", selectedMode));
      m_controllerModeSelected = selectedMode;
      configureBindings(selectedMode);
    }
  }


  private void configureSmartDashboardOptions() {
    m_controllerModeChooser.setDefaultOption("Single Controller", OperatorConstants.ControllerConfigMode.SINGLE.toString());
    m_controllerModeChooser.addOption("Dual Controller", OperatorConstants.ControllerConfigMode.DUAL.toString());
    SmartDashboard.putData("controller_mode", m_controllerModeChooser);
  }
}
