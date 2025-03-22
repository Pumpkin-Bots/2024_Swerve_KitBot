// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class SmartDashboardConstants {
    public static final String kControllerMode = "controller_mode";
  }

  public static class OperatorConstants {
    // Port numbers for driver and operator gamepads. These correspond with the numbers on the USB
    // tab of the DriverStation
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public enum ControllerConfigMode {
      SINGLE,
      DUAL;

      static ControllerConfigMode fromString(String mode) {
        try {
          return ControllerConfigMode.valueOf(mode.toUpperCase());
        } catch (IllegalArgumentException e) {
            throw new IllegalArgumentException("Invalid mode: " + mode);
        }
      }
    }
  }

  public static class DrivetrainConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kLeftRearID = 1;
    public static final int kLeftFrontID = 2;
    public static final int kRightRearID = 3;
    public static final int kRightFrontID = 4;

    // Current limit for drivetrain motors
    public static final double kCurrentLimit = 80.0;

    // Speed limiter for demo bot to slow it down
    // 1.0 = full speed, .5 = 50% max speed
    public static final double kSpeedLimit = 1; 
  }

  public static class LauncherConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kFeederID = 15;
    public static final int kLauncherID = 16;

    // Current limit for launcher and feed wheels
    public static final int kLauncherCurrentLimit = 80;
    public static final int kFeedCurrentLimit = 80;

    // Speeds for wheels when intaking and launching. Intake speeds are negative to run the wheels
    // in reverse
    public static final double kLauncherSpeed = -1; // 1
    public static final double kLaunchFeederSpeed = -1; // 1
    public static final double kIntakeLauncherSpeed = 1; // -1
    public static final double kIntakeFeederSpeed = .2; // -.2

    public static final double kLauncherDelay = 1;
  }
  public static class CANdleConstants{
    public static final int kCANdleID = 17;
  }
}
