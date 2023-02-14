#include "Robot.h"

void Robot::R2Jesu_SwitchAuto(void)
{
    double balanceDirection = 0.0;
    m_DriveEncoder1.SetPosition(0.0);
    m_DriveEncoder2.SetPosition(0.0);
    m_DriveEncoder3.SetPosition(0.0);
    m_DriveEncoder4.SetPosition(0.0);
    frc::SmartDashboard::PutNumber("position",0.0);
    while (ahrs->GetDisplacementX() > -1.25 && frc::DriverStation::IsAutonomousEnabled() && !(hasRun))
    {
        //printf("In the while loop\n");
        m_SwerveDrive1.Set(-0.4);
        pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), 90.0);
        m_SwerveTurn1.Set(pidOutput1);

        m_SwerveDrive2.Set(-0.4);
        pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), 90.0);
        m_SwerveTurn2.Set(pidOutput2);

        m_SwerveDrive3.Set(-0.4);
        pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), 90.0);
        m_SwerveTurn3.Set(pidOutput3);

        m_SwerveDrive4.Set(-0.4);
        pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), 90.0);
        m_SwerveTurn4.Set(pidOutput4);
        frc::SmartDashboard::PutNumber("position", m_DriveEncoder1.GetPosition());
        frc::SmartDashboard::PutNumber("displacement x", ahrs->GetDisplacementX());
        frc::SmartDashboard::PutNumber("displacement y", ahrs->GetDisplacementY());
    }
    hasRun = true;
    m_SwerveDrive1.Set(0.0);
    m_SwerveDrive2.Set(0.0);
    m_SwerveDrive3.Set(0.0);
    m_SwerveDrive4.Set(0.0);
    m_SwerveTurn1.Set(0.0);
    m_SwerveTurn2.Set(0.0);
    m_SwerveTurn3.Set(0.0);
    m_SwerveTurn4.Set(0.0);
    frc::SmartDashboard::PutNumber("position2",0.0);

    /*while (m_DriveEncoder1.GetPosition() < 35.0 && frc::DriverStation::IsAutonomousEnabled())
    {
        m_SwerveDrive1.Set(0.2);
        pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), 90.0);
        m_SwerveTurn1.Set(pidOutput1);

        m_SwerveDrive2.Set(0.2);
        pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), 90.0);
        m_SwerveTurn2.Set(pidOutput2);

        m_SwerveDrive3.Set(0.2);
        pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), 90.0);
        m_SwerveTurn3.Set(pidOutput3);

        m_SwerveDrive4.Set(0.2);
        pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), 90.0);
        m_SwerveTurn4.Set(pidOutput4);
        frc::SmartDashboard::PutNumber("position2", m_DriveEncoder1.GetPosition());
    }*/
    m_SwerveDrive1.Set(0.0);
    m_SwerveDrive2.Set(0.0);
    m_SwerveDrive3.Set(0.0);
    m_SwerveDrive4.Set(0.0);
    m_SwerveTurn1.Set(0.0);
    m_SwerveTurn2.Set(0.0);
    m_SwerveTurn3.Set(0.0);
    m_SwerveTurn4.Set(0.0);
    if ((ahrs->GetPitch() >= 10.0 || ahrs->GetPitch() <= -10.0) && frc::DriverStation::IsAutonomousEnabled())
    {
        balanceDirection = 90.0;
        frc::SmartDashboard::PutNumber("Pitch", ahrs->GetPitch());
        //printf("Pitch %d\n",ahrs->GetPitch());
        switchPidOutput = m_switchController.Calculate((ahrs->GetPitch()), 0.0);
        frc::SmartDashboard::PutNumber("switch pid", switchPidOutput);
        //printf("switch pid %d\n", switchPidOutput);
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
        //frc::SmartDashboard::PutNumber("position", m_DriveEncoder1.GetPosition());
    }
    else
    {
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