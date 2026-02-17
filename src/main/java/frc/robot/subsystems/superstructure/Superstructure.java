// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.shootingFeederVoltage;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.shootingIndexerVelocity;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.shootingIndexerVoltage;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.shootingShooterVelocity;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.shootingShooterVoltage;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.spinUpFeederVoltage;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.spinUpIndexerVelocity;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.spinUpIndexerVoltage;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.spinUpSeconds;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.spinUpShooterVelocity;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.spinUpShooterVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final SuperstructureIO io;
  private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

  public Superstructure(SuperstructureIO io) {
    this.io = io;
    SmartDashboard.putNumber("SpinUp/Feeder/Voltage", spinUpFeederVoltage);
    SmartDashboard.putNumber("SpinUp/Indexer/Voltage", spinUpIndexerVoltage);
    SmartDashboard.putNumber("SpinUp/Shooter/Voltage", spinUpShooterVoltage);
    SmartDashboard.putNumber("Shooting/Feeder/Voltage", shootingFeederVoltage);
    SmartDashboard.putNumber("Shooting/Indexer/Voltage", shootingIndexerVoltage);
    SmartDashboard.putNumber("Shooting/Shooter/Voltage", shootingShooterVoltage);

    SmartDashboard.putNumber(
        "SpinUp/Indexer/Velocity", spinUpIndexerVelocity.in(RotationsPerSecond));
    SmartDashboard.putNumber(
        "SpinUp/Shooter/Velocity", spinUpShooterVelocity.in(RotationsPerSecond));
    SmartDashboard.putNumber(
        "Shooting/Indexer/Velocity", shootingIndexerVelocity.in(RotationsPerSecond));
    SmartDashboard.putNumber(
        "Shooting/Shooter/Velocity", shootingShooterVelocity.in(RotationsPerSecond));

    SmartDashboard.putNumber("Tuning/Indexer/Velocity", 0.0);
    SmartDashboard.putNumber("Tuning/Shooter/Velocity", 0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure", inputs);
  }

  /** Set the rollers to the values for launching. Spins up before feeding fuel. */
  public Command launchVoltage() {
    return run(() -> {
          io.setFeederVoltage(spinUpFeederVoltage);
          io.setIndexerVoltage(spinUpIndexerVoltage);
          io.setShooterVoltage(spinUpShooterVoltage);
        })
        .withTimeout(spinUpSeconds)
        .andThen(
            run(
                () -> {
                  io.setFeederVoltage(shootingFeederVoltage);
                  io.setIndexerVoltage(shootingIndexerVoltage);
                  io.setShooterVoltage(shootingShooterVoltage);
                }))
        .finallyDo(
            () -> {
              io.setFeederVoltage(0.0);
              io.setIndexerVoltage(0.0);
              io.setShooterVoltage(0.0);
            });
  }

  /** Set the rollers to the values for launching. Spins up before feeding fuel. */
  public Command launchVelocity() {
    return run(() -> {
          io.setFeederVoltage(spinUpFeederVoltage);
          io.setIndexerVelocity(spinUpIndexerVelocity);
          io.setShooterVelocity(spinUpShooterVelocity);
        })
        .withTimeout(spinUpSeconds)
        .andThen(
            run(
                () -> {
                  io.setFeederVoltage(shootingFeederVoltage);
                  io.setIndexerVelocity(shootingIndexerVelocity);
                  io.setShooterVelocity(shootingShooterVelocity);
                }))
        .finallyDo(
            () -> {
              io.setFeederVoltage(0.0);
              io.setIndexerVoltage(0.0);
              io.setShooterVoltage(0.0);
            });
  }

  public Command shootDashboardVoltage() {
    return run(() -> {
          double dashboardSpinUpFeederVoltage =
              SmartDashboard.getNumber("SpinUp/Feeder/Voltage", 0.0);
          double dashboardSpinUpIndexerVoltage =
              SmartDashboard.getNumber("SpinUp/Indexer/Voltage", 0.0);
          double dashboardSpinUpShooterVoltage =
              SmartDashboard.getNumber("SpinUp/Shooter/Voltage", 0.0);
          io.setFeederVoltage(dashboardSpinUpFeederVoltage);
          io.setIndexerVoltage(dashboardSpinUpIndexerVoltage);
          io.setShooterVoltage(dashboardSpinUpShooterVoltage);
        })
        .withTimeout(spinUpSeconds)
        .andThen(
            run(
                () -> {
                  double dashboardShootingFeederVoltage =
                      SmartDashboard.getNumber("Shooting/Feeder/Voltage", 0.0);
                  double dashboardShootingIndexerVoltage =
                      SmartDashboard.getNumber("Shooting/Indexer/Voltage", 0.0);
                  double dashboardShootingShooterVoltage =
                      SmartDashboard.getNumber("Shooting/Shooter/Voltage", 0.0);
                  io.setFeederVoltage(dashboardShootingFeederVoltage);
                  io.setIndexerVoltage(dashboardShootingIndexerVoltage);
                  io.setShooterVoltage(dashboardShootingShooterVoltage);
                }))
        .finallyDo(
            () -> {
              io.setFeederVoltage(0.0);
              io.setIndexerVoltage(0.0);
              io.setShooterVoltage(0.0);
            });
  }

  public Command shootDashboardVelocity() {
    return run(() -> {
          double dashboardSpinUpFeederVoltage =
              SmartDashboard.getNumber("SpinUp/Feeder/Voltage", 0.0);
          double dashboardSpinUpIndexerVelocity =
              SmartDashboard.getNumber("SpinUp/Indexer/Velocity", 0.0);
          double dashboardSpinUpShooterVelocity =
              SmartDashboard.getNumber("SpinUp/Shooter/Velocity", 0.0);
          io.setFeederVoltage(dashboardSpinUpFeederVoltage);
          io.setIndexerVelocity(RotationsPerSecond.of(dashboardSpinUpIndexerVelocity));
          io.setShooterVelocity(RotationsPerSecond.of(dashboardSpinUpShooterVelocity));
        })
        .withTimeout(spinUpSeconds)
        .andThen(
            run(
                () -> {
                  double dashboardShootingFeederVoltage =
                      SmartDashboard.getNumber("Shooting/Feeder/Voltage", 0.0);
                  double dashboardShootingIndexerVelocity =
                      SmartDashboard.getNumber("Shooting/Indexer/Velocity", 0.0);
                  double dashboardShootingShooterVelocity =
                      SmartDashboard.getNumber("Shooting/Shooter/Velocity", 0.0);
                  io.setFeederVoltage(dashboardShootingFeederVoltage);
                  io.setIndexerVelocity(RotationsPerSecond.of(dashboardShootingIndexerVelocity));
                  io.setShooterVelocity(RotationsPerSecond.of(dashboardShootingShooterVelocity));
                }))
        .finallyDo(
            () -> {
              io.setFeederVoltage(0.0);
              io.setIndexerVoltage(0.0);
              io.setShooterVoltage(0.0);
            });
  }

  public Command tuneIndexer() {
    double dashboardTuningIndexerVelocity =
        SmartDashboard.getNumber("Tuning/Indexer/Velocity", 0.0);
    return runEnd(
        () -> io.setIndexerVelocity(RotationsPerSecond.of(dashboardTuningIndexerVelocity)),
        () -> io.setIndexerVoltage(0.0));
  }

  public Command tuneShooter() {
    double dashboardTuningShooterVelocity =
        SmartDashboard.getNumber("Tuning/Shooter/Velocity", 0.0);
    return runEnd(
        () -> io.setShooterVelocity(RotationsPerSecond.of(dashboardTuningShooterVelocity)),
        () -> io.setShooterVoltage(0.0));
  }

  public Command justGo() {
    return run(() -> {
          io.setFeederVoltage(shootingFeederVoltage);
          io.setIndexerVoltage(shootingIndexerVoltage);
          io.setShooterVoltage(shootingShooterVoltage);
        })
        .finallyDo(
            () -> {
              io.setFeederVoltage(0.0);
              io.setIndexerVoltage(0.0);
              io.setShooterVoltage(0.0);
            });
  }
}
