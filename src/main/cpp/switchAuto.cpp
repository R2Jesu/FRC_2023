#include "Robot.h"

void Robot::R2Jesu_SwitchAuto(void)
{
    double balanceDirection = 0.0;
    if ((m_DriveEncoder1.GetPosition() > -4.0) && !(initialBack))
    {
        m_SwerveDrive1.Set(-0.1);
        pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), 90.0);
        m_SwerveTurn1.Set(pidOutput1);

        m_SwerveDrive2.Set(-0.1);
        pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), 90.0);
        m_SwerveTurn2.Set(pidOutput2);

        m_SwerveDrive3.Set(-0.1);
        pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), 90.0);
        m_SwerveTurn3.Set(pidOutput3);

        m_SwerveDrive4.Set(-0.1);
        pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), 90.0);
        m_SwerveTurn4.Set(pidOutput4);
        frc::SmartDashboard::PutNumber("encoder x", m_DriveEncoder1.GetPosition());
    }
    else {
        if ((m_DriveEncoder1.GetPosition() <= -4.0) && !(initialBack))
        {
            initialBack = true;
            m_DriveEncoder1.SetPosition(0.0);
            m_SwerveDrive1.Set(0.0);
            m_SwerveDrive2.Set(0.0);
            m_SwerveDrive3.Set(0.0);
            m_SwerveDrive4.Set(0.0);
            m_SwerveTurn1.Set(0.0);
            m_SwerveTurn2.Set(0.0);
            m_SwerveTurn3.Set(0.0);
            m_SwerveTurn4.Set(0.0);
        }
    }
    if ((m_DriveEncoder1.GetPosition() < 62.0) && initialBack && !(yDisplaceDone))
    {
        frc::SmartDashboard::PutNumber("displacement y",n++);
        frc::SmartDashboard::PutNumber("encoder y", m_DriveEncoder1.GetPosition());
        m_SwerveDrive1.Set(0.1);
        pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), 0.0);
        m_SwerveTurn1.Set(pidOutput1);

        m_SwerveDrive2.Set(0.1);
        pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), 0.0);
        m_SwerveTurn2.Set(pidOutput2);

        m_SwerveDrive3.Set(0.1);
        pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), 0.0);
        m_SwerveTurn3.Set(pidOutput3);

        m_SwerveDrive4.Set(0.1);
        pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), 0.0);
        m_SwerveTurn4.Set(pidOutput4);
    }
    else
    {
        if ((m_DriveEncoder1.GetPosition() >= 62.0) && initialBack)
        {
            yDisplaceDone = true;
            m_DriveEncoder1.SetPosition(0.0);
            m_DriveEncoder3.SetPosition(0.0);
            R2Jesu_Drive(0.0, 0.0, 0.0);
        }
    }
    
    if ((m_DriveEncoder3.GetPosition() > -60.0) && !(hasRun) && yDisplaceDone)
    {
        m_SwerveDrive1.Set(-0.3);
        pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), 90.0);
        m_SwerveTurn1.Set(pidOutput1);

        m_SwerveDrive2.Set(-0.3);
        pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), 90.0);
        m_SwerveTurn2.Set(pidOutput2);

        m_SwerveDrive3.Set(-0.3);
        pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), 90.0);
        m_SwerveTurn3.Set(pidOutput3);

        m_SwerveDrive4.Set(-0.3);
        pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), 90.0);
        m_SwerveTurn4.Set(pidOutput4);
        frc::SmartDashboard::PutNumber("encoder x", m_DriveEncoder1.GetPosition() * encoderConversion);
    }
    else {
        if ((m_DriveEncoder3.GetPosition() <= -60.0) && yDisplaceDone)
        {
            hasRun = true;
            m_SwerveDrive1.Set(0.0);
            m_SwerveDrive2.Set(0.0);
            m_SwerveDrive3.Set(0.0);
            m_SwerveDrive4.Set(0.0);
            m_SwerveTurn1.Set(0.0);
            m_SwerveTurn2.Set(0.0);
            m_SwerveTurn3.Set(0.0);
            m_SwerveTurn4.Set(0.0);
        }
    }

    if ((ahrs->GetPitch() >= 10.0 || ahrs->GetPitch() <= -10.0) && hasRun)
    {
        balanceDirection = 270.0;
        frc::SmartDashboard::PutNumber("Pitch", ahrs->GetPitch());
        switchPidOutput = m_switchController.Calculate((ahrs->GetPitch()), 0.0);
        frc::SmartDashboard::PutNumber("switch pid", switchPidOutput);
        m_SwerveDrive1.Set(switchPidOutput);
        pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), balanceDirection);
        m_SwerveTurn1.Set(pidOutput1);

        m_SwerveDrive2.Set(switchPidOutput);
        pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), balanceDirection);
        m_SwerveTurn2.Set(pidOutput2);

        m_SwerveDrive3.Set(switchPidOutput);
        pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), balanceDirection);
        m_SwerveTurn3.Set(pidOutput3);

        m_SwerveDrive4.Set(switchPidOutput);
        pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), balanceDirection);
        m_SwerveTurn4.Set(pidOutput4);

    }
    else
    {
        if (hasRun) {
            m_SwerveDrive1.Set(0.0);
            m_SwerveDrive2.Set(0.0);
            m_SwerveDrive3.Set(0.0);
            m_SwerveDrive4.Set(0.0);
            m_SwerveTurn1.Set(0.0);
            m_SwerveTurn2.Set(0.0);
            m_SwerveTurn3.Set(0.0);
            m_SwerveTurn4.Set(0.0);
    
    
            pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), 45.0);
            m_SwerveTurn1.Set(pidOutput1);
            pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), 135.0);
            m_SwerveTurn2.Set(pidOutput2);
            pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), 45.0);
            m_SwerveTurn3.Set(pidOutput3);
            pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), 135.0);
            m_SwerveTurn4.Set(pidOutput4);
        }
    } 

    frc::SmartDashboard::PutBoolean("initalback", initialBack);
    frc::SmartDashboard::PutBoolean("yDisplaceDone", yDisplaceDone);
    frc::SmartDashboard::PutNumber("position3.2", m_DriveEncoder3.GetPosition());
} 