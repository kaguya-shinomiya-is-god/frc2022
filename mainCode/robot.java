// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

 import com.ctre.phoenix.motorcontrol.ControlMode;
 import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private double mag, mag2;
  private double seno, seno2;
  private int pov;
  private double spd = 1;
  private double mL = 0,mR = 0;
  private double x1,y1,x2,y2;
  private boolean reverse, analogic1, analogic2;

  VictorSPX bolsonarista1 = new VictorSPX(1);
  VictorSPX bolsonarista2 = new VictorSPX(2);
  VictorSPX petista1 = new VictorSPX(3);
  VictorSPX petista2 = new VictorSPX(4);

  
  Joystick xbox = new Joystick(0);

  

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    petista2.follow(petista1);
    bolsonarista2.follow(bolsonarista2);
    petista1.configNeutralDeadband(0.04);
    bolsonarista1.configNeutralDeadband(0.04);
  }

  @Override
  public void teleopPeriodic() {
    x1 = xbox.getRawAxis(0);
    y1 = - xbox.getRawAxis(1);
    x2 = xbox.getRawAxis(4);
    y2 = - xbox.getRawAxis(5);
    pov = xbox.getPOV();
    buttonSe();
      SmartDashboard.putNumber("Velocidade", spd);

    analogicVer();
    povCalc(pov);

    if(!reverse){
      SmartDashboard.putNumber("ForcaMotor Esquerdo", minMotor(mL));
      SmartDashboard.putNumber("ForçaMotor Direito", minMotor(mR));
    }else if(reverse){
      SmartDashboard.putNumber("ForcaMotor Esquerdo", - minMotor(mL));
      SmartDashboard.putNumber("ForçaMotor Direito", - minMotor(mR));
      reverse = false;
    }
      SmartDashboard.putBoolean("Analogico 1", analogic1);
      SmartDashboard.putBoolean("Analogico 2", analogic2);
  }

public void quadCalc(double y,double x){
    mag = Math.hypot(x, y);
    seno = y / mag;
    // Quadrante 1  
    if(y >= 0 && x >= 0){
      mR = seno * mag * spd; // Varia
      mL = mag * spd; // Constante
    // Quadrante 2
    }else if(y >= 0 && x < 0){
      mR = mag * spd; // Constante
      mL = seno * mag * spd; // Varia
    // Quadrante 3
    }else if(y < 0 && x < 0){
      mR = seno * mag * spd; // Varia
      mL = - mag * spd; // Constante
    // Quadrante 4
    }else if(y < 0 && x >= 0){
      mR = - mag * spd; // Constante
      mL = seno * mag * spd; // Varia

  }

  
//  petista1.set(ControlMode.PercentOutput, mL);
//  bolsonarista1.set(ControlMode.PercentOutput, - mR);
}

  public void povCalc(int pov){
    switch(pov){
      case 0:
      mR = 0.25;
      mL = 0.25;
      break;

      case 45:
      mR = 0;
      mL = 0.25;
      break;

      case 90:
      mR = -0.25;
      mL = 0.25;
      break;

      case 135:
      mR = -0.25;
      mL = 0;
      break;

      case 180:
      mR = -0.25;
      mL = -0.25;
      break;

      case 225:
      mR = 0;
      mL = -0.25;
      break;

      case 270:
      mR = 0.25;
      mL = -0.25;
      break;

      case 315:
      mR = 0.25;
      mL = 0; 
      break;
      
    }
      
      
  }

  private void analogicVer(){
    if(Math.abs(y1) > 0.04 && Math.abs(x1) > 0.04){
      analogic1 = true;
      analogic2 = false;
      quadCalc(y1, x1);
    }

    else if(Math.abs(y2) > 0.04 && Math.abs(x2) > 0.04){
      reverseQuadCalc(); 
      analogic1 = false;
      analogic2 = true;
  }
}

  private void reverseQuadCalc() {
    mag2 = Math.hypot(x2, y2);
      seno2 = y2 / mag2;
        // Quadrante 1  
      if(y2 >= 0 && x2 >= 0){
      mR = seno2 * mag2 * spd; // Varia
      mL = - mag2 * spd; // Constante
        // Quadrante 2
      }else if(y2 >= 0 && x2 < 0){
      mR = - mag2 * spd; // Constante
      mL = seno2 * mag2 * spd; // Varia
        // Quadrante 3
      }else if(y2 < 0 && x2 < 0){
      mR = seno2 * mag2 * spd; // Varia
      mL = mag2 * spd; // Constante
        // Quadrante 4
      }else if(y2 < 0 && x2 >= 0){
      mR = mag2 * spd; // Constante
      mL = seno2 * mag2 * spd; // Varia
      }
  }

  private double buttonSe(){
    if(xbox.getRawButton(3))
      spd = 1;

      else if(xbox.getRawButton(1))
        spd = 0.5;

          else if(xbox.getRawButton(2))
            spd = 0.25;

              return spd;
  }

  private double minMotor(double motor){

    if(Math.abs(motor) < 0.04)
      return 0;

    else
      return motor;
    
    }


}
