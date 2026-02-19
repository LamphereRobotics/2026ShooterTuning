// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.function.DoubleSupplier;

/**
 * This superstructure implementation is for Spark devices. It defaults to brushless control, but
 * can be easily adapted for a brushed motor. One or more Spark Flexes can be used by swapping
 * relevant instances of "SparkFlex" with "SparkFlex".
 */
public class SuperstructureIOSpark implements SuperstructureIO {
  private final SparkFlex feeder = new SparkFlex(feederCanId, MotorType.kBrushless);
  private final SparkFlex indexer = new SparkFlex(indexerCanId, MotorType.kBrushless);
  private final SparkFlex shooterLeader = new SparkFlex(shooterLeaderCanId, MotorType.kBrushless);
  private final SparkFlex shooterFollower =
      new SparkFlex(shooterFollowerCanId, MotorType.kBrushless);

  private final RelativeEncoder feederEncoder = feeder.getEncoder();
  private final RelativeEncoder indexerEncoder = indexer.getEncoder();
  private final RelativeEncoder shooterEncoder = shooterLeader.getEncoder();

  private final SparkClosedLoopController indexerController = indexer.getClosedLoopController();
  private final SparkClosedLoopController shooterLeaderController =
      shooterLeader.getClosedLoopController();

  public SuperstructureIOSpark() {
    var feederConfig = configureMotor(feeder, feederMotorReduction, feederInverted);
    var indexerConfig = configureMotor(indexer, indexerMotorReduction, indexerInverted);
    var shooterLeaderConfig =
        configureMotor(shooterLeader, shooterLeaderMotorReduction, shooterLeaderInverted);
    var shooterFollowerConfig =
        configureMotor(shooterFollower, shooterFollowerMotorReduction, shooterFollowerInverted);

    indexerConfig.closedLoop.feedForward.sva(indexerKs, indexerKv, indexerKa);
    indexerConfig.closedLoop.pid(indexerKp, 0.0, 0.0);

    shooterLeaderConfig.closedLoop.feedForward.sva(shooterKs, shooterKv, shooterKa);
    shooterLeaderConfig.closedLoop.pid(shooterKp, 0.0, 0.0);
    shooterLeaderConfig.closedLoop.maxMotion.maxAcceleration(
        shooterMaxAcceleration.in(MetersPerSecondPerSecond));

    shooterFollowerConfig.follow(shooterLeaderCanId, true);

    applyConfig(feeder, feederConfig);
    applyConfig(indexer, indexerConfig);
    applyConfig(shooterLeader, shooterLeaderConfig);
    applyConfig(shooterFollower, shooterFollowerConfig);
  }

  @Override
  public void updateInputs(SuperstructureIOInputs inputs) {
    var feederInputs = getInputs(feeder, feederEncoder);
    var indexerInputs = getInputs(indexer, indexerEncoder);
    var shooterInputs = getInputs(shooterLeader, shooterEncoder);

    inputs.feederPositionRad = feederInputs.positionRad;
    inputs.feederVelocityRadPerSec = feederInputs.velocityRadPerSec;
    inputs.feederAppliedVolts = feederInputs.appliedVolts;
    inputs.feederCurrentAmps = feederInputs.currentAmps;

    inputs.indexerPositionRad = indexerInputs.positionRad;
    inputs.indexerVelocityRadPerSec = indexerInputs.velocityRadPerSec;
    inputs.indexerAppliedVolts = indexerInputs.appliedVolts;
    inputs.indexerCurrentAmps = indexerInputs.currentAmps;

    inputs.shooterPositionRad = shooterInputs.positionRad;
    inputs.shooterVelocityRadPerSec = shooterInputs.velocityRadPerSec;
    inputs.shooterAppliedVolts = shooterInputs.appliedVolts;
    inputs.shooterCurrentAmps = shooterInputs.currentAmps;

    inputs.shooterSetpoint = shooterLeaderController.getSetpoint();
  }

  @Override
  public void setFeederVoltage(double volts) {
    feeder.setVoltage(volts);
  }

  @Override
  public void setIndexerVoltage(double volts) {
    indexer.setVoltage(volts);
  }

  @Override
  public void setShooterVoltage(double volts) {
    shooterLeader.setVoltage(volts);
  }

  @Override
  public void setIndexerVelocity(LinearVelocity velocity) {
    indexerController.setSetpoint(velocity.in(MetersPerSecond), ControlType.kVelocity);
    // indexerController.setSetpoint(currentLimit, ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void setShooterVelocity(LinearVelocity velocity) {
    // shooterLeaderController.setSetpoint(velocity.in(MetersPerSecond), ControlType.kVelocity);
    shooterLeaderController.setSetpoint(
        velocity.in(MetersPerSecond), ControlType.kMAXMotionVelocityControl);
  }

  private static SparkFlexConfig configureMotor(
      SparkBase motor, double motorReduction, boolean inverted) {
    var config = new SparkFlexConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(voltageCompensation)
        .inverted(inverted);
    config
        .encoder
        .positionConversionFactor(motorReduction) // Rotor Rotations -> Roller Radians
        .velocityConversionFactor(motorReduction / 60)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    return config;
  }

  private static void applyConfig(SparkBase motor, SparkFlexConfig config) {
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  private static Inputs getInputs(SparkBase motor, RelativeEncoder encoder) {
    Inputs inputs = new Inputs();

    ifOk(motor, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAmps = value);

    return inputs;
  }

  private static class Inputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }
}
