// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface SuperstructureIO {
  @AutoLog
  public static class SuperstructureIOInputs {
    public double feederPositionRad = 0.0;
    public double feederVelocityRadPerSec = 0.0;
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;

    public double indexerPositionRad = 0.0;
    public double indexerVelocityRadPerSec = 0.0;
    public double indexerAppliedVolts = 0.0;
    public double indexerCurrentAmps = 0.0;

    public double shooterPositionRad = 0.0;
    public double shooterVelocityRadPerSec = 0.0;
    public double shooterAppliedVolts = 0.0;
    public double shooterCurrentAmps = 0.0;
  }

  /** Update the set of loggable inputs. */
  public abstract void updateInputs(SuperstructureIOInputs inputs);

  /** Run the feeder at the specified voltage. */
  public abstract void setFeederVoltage(double volts);

  /** Run the indexer at the specified voltage. */
  public abstract void setIndexerVoltage(double volts);

  /** Run the shooter at the specified voltage. */
  public abstract void setShooterVoltage(double volts);

  public abstract void setIndexerVelocity(AngularVelocity velocity);

  public abstract void setShooterVelocity(AngularVelocity velocity);
}
