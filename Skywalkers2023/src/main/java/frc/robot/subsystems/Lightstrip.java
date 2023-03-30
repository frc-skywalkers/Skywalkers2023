// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.lightstripConstants;
import frc.robot.lightstrip.LedState;
import frc.robot.lightstrip.TempLedState;

public class Lightstrip extends SubsystemBase {
  private PWM redOutput = new PWM(lightstripConstants.redPort);
  private PWM greenOutput = new PWM(lightstripConstants.greenPort);
  private PWM blueOutput = new PWM(lightstripConstants.bluePort);

  private LedState defaultColor = lightstripConstants.defaultState;
  private Timer defaultTimer = new Timer();
  private LedState currentColor = null;
  private Timer currentTimer = new Timer();
  private TempLedState tempColor = null;
  private Timer tempTimer = new Timer();

  /** Creates a new Lightstrip. */
  public Lightstrip() {
    defaultTimer.reset();
    defaultTimer.start();
  }

  @Override
  public void periodic() {
    if(tempColor != null) {
      if(tempColor.getSeconds() < tempTimer.get()) {
        tempColor = null;
        tempTimer.stop();
      } else {
        update(toLedState(tempColor), tempTimer);
      }
    } else if(currentColor != null) {
      update(currentColor, currentTimer);
    } else {
      update(defaultColor, defaultTimer);
    }

    SmartDashboard.putNumber("Red", redOutput.getRaw());
    SmartDashboard.putNumber("Green", greenOutput.getRaw());
    SmartDashboard.putNumber("Blue", blueOutput.getRaw());

    if(currentColor != null) {
      SmartDashboard.putNumberArray("Current", currentColor.getState());
    }

    if(tempColor != null) {
      SmartDashboard.putNumberArray("Temp", tempColor.getState());
    }
  }

  private LedState toLedState(TempLedState state) {
    return new LedState(state.getRed(), state.getGreen(), state.getBlue(), state.getEffect());
  }

  private void update(LedState state, Timer timer) {
    double secondCycle = timer.get() % 1 - 0.50;
    SmartDashboard.putNumber("Timer", timer.get() % 1 - 0.50 * state.getRed());
    if(state.getEffect() == "Solid") {
      setColor(state.getRed(), state.getGreen(), state.getBlue());
    } else if(state.getEffect() == "Blink") {
      if((timer.get() % 2) < 1) {
        setColor(state.getRed(), state.getGreen(), state.getBlue());
      } else if((timer.get() % 2) >= 1) {
        setColor(0, 0, 0);
      }
    } else if(state.getEffect() == "Fast Blink") {
      if((timer.get() % 0.5) < 0.25) {
        setColor(state.getRed(), state.getGreen(), state.getBlue());
      } else if((timer.get() % 0.5) >= 0.25) {
        setColor(0, 0, 0);
      }
    }
  }

  public void toggleOnColor(LedState state) {
    setColor(state);
  }

  public void toggleOffColor(LedState state) {
    if(currentColor != null && state.compare(currentColor)) {
      currentColor = null;
      currentTimer.stop();
    }
  }

  private void resetCurrent() {
    currentTimer.reset();
    currentTimer.start();
  }

  public void setColor(LedState state) {
    if(currentColor == null) {
      currentColor = state;
      resetCurrent();
    } else if(!state.compare(currentColor)) {
      currentColor = state;
      resetCurrent();
    }
  }

  public void tempColor(TempLedState state) {
    tempColor = state;
    resetTemp();
  }

  private void resetTemp() {
    tempTimer.reset();
    tempTimer.start();
  }

  private void setColor(int red, int green, int blue) {
    redOutput.setRaw(red);
    greenOutput.setRaw(green);
    blueOutput.setRaw(blue);
  }
}
