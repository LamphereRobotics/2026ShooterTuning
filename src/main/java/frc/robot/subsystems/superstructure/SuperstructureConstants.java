// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class SuperstructureConstants {
  public static final int currentLimit = 60;
  public static final double voltageCompensation = 12.0;

  public static final int feederCanId = 4;
  public static final double feederMotorReduction = 1.0;
  public static final boolean feederInverted = false;

  public static final int indexerCanId = 3;
  public static final double indexerMotorReduction = 1.0;
  public static final boolean indexerInverted = true;

  public static final int shooterLeaderCanId = 2;
  public static final double shooterLeaderMotorReduction = 1.0;
  public static final boolean shooterLeaderInverted = true;

  public static final int shooterFollowerCanId = 1;
  public static final double shooterFollowerMotorReduction = 1.0;
  public static final boolean shooterFollowerInverted = false;

  public static final double spinUpSeconds = 1.0;

  public static double spinUpFeederVoltage = 0.0;
  public static double spinUpIndexerVoltage = 3.0;
  public static double spinUpShooterVoltage = 6.6;

  public static double shootingFeederVoltage = 6.0;
  public static double shootingIndexerVoltage = 3.0;
  public static double shootingShooterVoltage = 6.6;

  public static final AngularVelocity spinUpFeederVelocity = RotationsPerSecond.of(0.0);
  public static final AngularVelocity spinUpIndexerVelocity = RotationsPerSecond.of(3000.0);
  public static final AngularVelocity spinUpShooterVelocity = RotationsPerSecond.of(3000.0);

  public static final AngularVelocity shootingFeederVelocity = RotationsPerSecond.of(3000.0);
  public static final AngularVelocity shootingIndexerVelocity = RotationsPerSecond.of(3000.0);
  public static final AngularVelocity shootingShooterVelocity = RotationsPerSecond.of(3000.0);

  public static final double indexerKp = 0.0;
  public static final double indexerKd = 0.0;
  public static final double indexerKs = 0.0;
  public static final double indexerKv = 0.0;

  public static final double shooterKp = 0.0;
  public static final double shooterKd = 0.0;
  public static final double shooterKs = 0.0;
  public static final double shooterKv = 0.0;
}
