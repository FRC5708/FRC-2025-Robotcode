// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

//This is Physical Properties Varibles for swerve json file 
//"drive": 0.047286787200699704,
//"angle": 16.8


public final class Constants {
  public static class Operator {
    public static final int kDriverControllerPort = 0;
    public static final double driveDeadband = 0.05;
  }

  public class NonChassis {
    public static final int elevatorMotorID1 = 31;
    public static final int elevatorMotorID2 = 32;
    // public static final int coralIntakeMotorID1 = 11;
    // public static final int coralIntakeMotorID2 = 12;   
    public static final int coralPlaceMotorID3 = 13;   
    public static final int coralPlaceMotorID4 = 14;  
    public static final double millisToL4 = 2710;   
    public static final double millisDownL4 = 2100;
    public static final double millisToSpit = 250;
    public static final double millisToIntake = 800; 
    public static final double ticksToL1 = 5;
    public static final double ticksToL2 = 17.119016647338867;
    public static final double ticksToL3 = 35.99966049194336;
    public static final double ticksToL4 = 66.810821533203125; // added 3
  }

  public static class Drive {
    // The max free speed of the module
    public static final double maxSpeed = 4.5;

    public static final double trackWidth = 0.5588;
    public static final double wheelBase = 0.5588;

    public static Translation2d frontLeftPosition = new Translation2d(trackWidth/2,wheelBase/2);
    public static Translation2d frontRightPosition = new Translation2d(-trackWidth/2,wheelBase/2);
    public static Translation2d backLeftPosition = new Translation2d(trackWidth/2,-wheelBase/2);
    public static Translation2d backRightPosition = new Translation2d(-trackWidth/2,-wheelBase/2);

    public static SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
      frontLeftPosition,
      frontRightPosition,
      backLeftPosition,
      backRightPosition
    );
  }
}
