// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private double mag, mag2;
  private double seno, seno2;
  private int pov;    
  private double spd = 1;
  private double mL = 0,mR = 0;
  private double x1,y1,x2,y2;
  private boolean analogic1, analogic2;
  private double rt, lt;

  VictorSPX m_direita1 = new VictorSPX(1);
  VictorSPX m_direita2 = new VictorSPX(2);
  VictorSPX m_esquerda1 = new VictorSPX(3);
  VictorSPX m_esquerda2 = new VictorSPX(4);

  
  Joystick joystick1 = new Joystick(0);

  

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_esquerda2.follow(m_esquerda1);
    m_direita2.follow(m_direita2);
    m_esquerda1.configNeutralDeadband(0.04);
    m_direita1.configNeutralDeadband(0.04);
  }

  @Override
  public void teleopPeriodic() {
    // Atribuição de valores as variaveis
    x1 = joystick1.getRawAxis(0);
    y1 = - joystick1.getRawAxis(1);
    x2 = joystick1.getRawAxis(4);
    y2 = - joystick1.getRawAxis(5);
    pov = joystick1.getPOV();
    rt = joystick1.getRawAxis(3);
    lt = - joystick1.getRawAxis(2);

    // Calculo das magnitudes
    mag = Math.hypot(x1, y1);
    mag2 = Math.hypot(x2, y2);

    // Verificação do uso de botões
    buttonSe(joystick1.getRawButton(3), joystick1.getRawButton(1), joystick1.getRawButton(2));

    // Verificação dos analogicos e triggers
    analogicVer();
    // Reiniciação dos valores insignificantes
    resetAxis();
    // Calculo do POV
    povCalc(pov);
      
      // Exibição dos valores na simulação
      SmartDashboard.putNumber("Velocidade", spd);
      SmartDashboard.putNumber("ForcaMotor Esquerdo", minMethod(mL));
      SmartDashboard.putNumber("ForçaMotor Direito", minMethod(mR));
      SmartDashboard.putNumber("Magnitude Esquerda", mag);
      SmartDashboard.putNumber("Magnitude Direita", mag2);
      SmartDashboard.putNumber("Trigger Esquerdo", -lt);
      SmartDashboard.putNumber("Trigger Direito", rt);
      SmartDashboard.putString("Analogico ativo",analogicGate(analogic1,analogic2));
      SmartDashboard.putString("Trigger ativo", triggerGate(lt,rt));

  }

  private String analogicGate(boolean a, boolean b){
    
    if(a) return "Esquerdo"; // Verificação do uso do analogico esquerdo

    else if(b) return "Direito"; // Verificação do uso do analogico direito

    else return "Nenhum"; // Verificação da inutilização dos dois lados

  }

  private String triggerGate(double a,double b){

    if(a != 0 && b != 0) return "Ambos";

    else if(a != 0) return "Esquerda";

    else if(b != 0) return "Direita";

    else return "Nenhuma";
  }

  

  private void resetAxis() {

    // Verificação de inatividade dos analogicos
    if(mag < 0.1){
      x1 = 0;
      y1 = 0;
      mag = 0;
    }
    if(mag2 < 0.1){
      x2 = 0;
      y2 = 0;
      mag2 = 0;
    }
    // Verificação da inatividade de ambos analogicos
    if(mag < 0.1 && mag2 < 0.1){
      mL = 0;
      mR = 0;
    }

  }

  public void triggerCalc(double rt,double lt,double x){
    if(rt != 0){
      if(x >= 0){
        mL = rt;
        mR = rt * (1 - x);
      }else if(x < 0){
        mL = rt * (1 + x);
        mR = rt;
      }
    }else if(lt != 0){
      if(x >= 0){
        mL = lt * (x-1);
        mR = lt;
      }else if(x < 0){
        mL = lt * (x + 1);
        mR = lt;
      }
    }
  }

  public void quadCalc(double y, double x) {
      seno = y / mag;
      // Quadrante 1  
    if(y >= 0 && x >= 0){
      mR = seno * mag * spd; // Varia
      mL = mag * spd; // Constante
      // Quadrante 2
    }else if(y >= 0 && x <= 0){
      mR = mag * spd; // Constante
      mL = seno * mag * spd; // Varia
      // Quadrante 3
    }else if(y < 0 && x < 0){
      mR = -mag * spd; // Constante
      mL = seno * mag * spd; // Varia
      // Quadrante 4
    }else if(y < 0 && x >= 0){
      mR = seno * mag * spd; // Varia
      mL = -mag * spd; // Constante
  }

  
//  m_esquerda1.set(ControlMode.PercentOutput, mL);
//  m_direita1.set(ControlMode.PercentOutput, - mR);
}

  public void povCalc(int pov){
    // Calculo do POV
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

    //Verificação do analogico esquerdo
    if(minMethod(mag) != 0){
      analogic1 = true;
      analogic2 = false;
      // Calculo dos quadrantes
      quadCalc(y1, x1);
      // Calculo dos triggers
      //triggerCalc(rt,lt,x1);
    } 

    else if(minMethod(mag2)!=0){
      // Calculo dos quadrantes
      reverseQuadCalc(); 
      // Calculo dos triggers
      //triggerCalc(rt,lt,x2);
      analogic1 = false;
      analogic2 = true;
    }

    else{
      analogic1 = false;
      analogic2 = false;
    }
}

  private void reverseQuadCalc() {
      seno2 = y2 / mag2;
        // Quadrante 1  
      if(y2 >= 0 && x2 >= 0){
        mR = - mag2 * spd;
        mL = - seno2 * mag2 * spd;
        // Quadrante 2
      }else if(y2 >= 0 && x2 < 0){
        mR = - seno2 * mag2 * spd;
        mL = - mag2 * spd;
        // Quadrante 3
      }else if(y2 < 0 && x2 < 0){
        mR = mag2 * spd;
        mL = - seno2 * mag2 * spd;
        // Quadrante 4
      }else if(y2 < 0 && x2 >= 0){
        mR = - seno2 * mag2 * spd;
        mL = mag2 * spd;
      }
  }

  private double buttonSe(boolean x,boolean a, boolean b){
    
    if (x)
      spd = 1;

    else if(a)
      spd = 0.5;

    else if(b)
      spd = 0.25;

      return spd;
  }

  private double minMethod(double motor){

    if(Math.abs(motor) < 0.04) return 0;

    else return motor;
    
    }

  }
