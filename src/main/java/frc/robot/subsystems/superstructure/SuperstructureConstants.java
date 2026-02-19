// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class SuperstructureConstants {
  public static final int currentLimit = 60;
  public static final double voltageCompensation = 12.0;

  public static final int feederCanId = 4;
  public static final double feederMotorReduction = 1.0;
  public static final boolean feederInverted = false;

  public static final int indexerCanId = 3;
  public static final double indexerWheelRadius = Units.inchesToMeters(2.0);
  public static final double indexerMotorReduction =
      indexerWheelRadius * 2.0 * Math.PI; // rotations -> surface distance as meters
  public static final boolean indexerInverted = true;

  public static final int shooterLeaderCanId = 2;
  public static final double shooterWheelRadius = Units.inchesToMeters(2.0);
  public static final double shooterLeaderMotorReduction =
      shooterWheelRadius * 2.0 * Math.PI; // rotations -> surface distance as meters
  public static final boolean shooterLeaderInverted = true;

  public static final int shooterFollowerCanId = 1;
  public static final double shooterFollowerMotorReduction = shooterLeaderMotorReduction;
  public static final boolean shooterFollowerInverted = false;

  public static final double spinUpSeconds = 1.0;

  public static double spinUpFeederVoltage = 0.0;
  public static double spinUpIndexerVoltage = 3.0;
  public static double spinUpShooterVoltage = 6.6;

  public static double shootingFeederVoltage = 6.0;
  public static double shootingIndexerVoltage = 3.0;
  public static double shootingShooterVoltage = 6.6;

  public static final LinearVelocity shootingIndexerVelocity = MetersPerSecond.of(20.0);
  public static final LinearVelocity shootingShooterVelocity = MetersPerSecond.of(20.0);

  public static final LinearVelocity shooterVelocityTolerance = MetersPerSecond.of(1.0);
  public static final LinearAcceleration shooterMaxAcceleration = MetersPerSecondPerSecond.of(20.0);

  public static final double indexerKs = 0.0;
  public static final double indexerKv = 0.0;
  public static final double indexerKa = 0.0;
  public static final double indexerKp = 0.0;

  public static final double shooterKs = 0.040528;
  public static final double shooterKv = 0.3373;
  public static final double shooterKa = 0.069009;
  public static final double shooterKp = 0.03;
}
