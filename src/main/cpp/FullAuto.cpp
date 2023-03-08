#include "Robot.h"

void Robot::R2Jesu_FullAuto()
{
    printf("Starting full auto\n");

    //TODO: What tag are we going to face?
    //1
    if ((m_DriveEncoder1.GetPosition() < 40) && !((m_aprilSelected == 2.0) || 
    (m_aprilSelected == 7.0)) && (fullAuto1 == false))
    {
        R2Jesu_Drive(0.0, 1.0, 0.0);
    }
    else
    {
        fullAuto1 = true;
    }

    //2
    if ((tid != m_aprilSelected) && fullAuto1 && (fullAuto2 == false))
    {
        alignYaw = ahrs->GetYaw();
        aTurnPidOutput = m_aTurnController.Calculate(alignYaw, 180);
        R2Jesu_Drive(0.0, 0.0, aTurnPidOutput);
    } else
    {
        if ((tid == m_aprilSelected) && fullAuto1 && (fullAuto2 == false))
        {
            fullAuto2 = true;
            R2Jesu_Drive(0.0, 0.0, 0.0);
        }
    }


    if (((currentDistance > autoAprilDistance) || (currentDistance < 5.00))  && fullAuto2 && (fullAuto2_5 == false))
    {
        R2Jesu_Drive(0.0, -0.8, 0.0);
    }
    else
    {
        if ((currentDistance <= autoAprilDistance) && fullAuto2 && (fullAuto2_5 == false))
        {
            R2Jesu_Drive(0.0, 0.0, 0.0);
            fullAuto2_5 = true;
        }
    }
    //3
    //TODO: What tag are we going to face?
    //TODO: What tag are we going to face?
    if (fullAuto2_5 && (fullAuto3 == false))
    {
        if (R2Jesu_Align(180.0) && fullAuto2_5 && (fullAuto3 == false)) 
        {
            fullAuto3 = true;
            m_DriveEncoder1.SetPosition(0.0);
            R2Jesu_Drive(0.0, 0.0, 0.0);
        }
    }
    //4
    if (fullAuto3 && (fullAuto4 == false))
    { 
        if ((((m_SwerveAnalog1.GetVoltage() * conversion1) > 5.0) && ((m_SwerveAnalog1.GetVoltage() * conversion1) < 175.0)) ||
        (((m_SwerveAnalog2.GetVoltage() * conversion2) > 5.0) && ((m_SwerveAnalog2.GetVoltage() * conversion2) < 175.0)) ||
        (((m_SwerveAnalog3.GetVoltage() * conversion3) > 5.0) && ((m_SwerveAnalog3.GetVoltage() * conversion3) < 175.0)) ||
        (((m_SwerveAnalog4.GetVoltage() * conversion4) > 5.0) && ((m_SwerveAnalog4.GetVoltage() * conversion4) < 175.0)))
        {
            m_SwerveDrive1.Set(0.0);
            pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), 0.0);
            m_SwerveTurn1.Set(pidOutput1);

            m_SwerveDrive2.Set(0.0);
            pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), 0.0);
            m_SwerveTurn2.Set(pidOutput2);

            m_SwerveDrive3.Set(0.0);
            pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), 0.0);
            m_SwerveTurn3.Set(pidOutput3);

            m_SwerveDrive4.Set(0.0);
            pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), 0.0);
            m_SwerveTurn4.Set(pidOutput4);
            wheelAngleCheck = false;
        } else
        {
            wheelAngleCheck = true;
        }
        if (((m_gridSelected == 1.0) || (m_gridSelected == 4.0)) && (m_DriveEncoder1.GetPosition() < 34.5) && wheelAngleCheck)
        {
        gridPidOutput = m_gridController.Calculate(m_DriveEncoder1.GetPosition(), 36.5);
        m_SwerveDrive1.Set(gridPidOutput);
        pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), 0.0);
        m_SwerveTurn1.Set(pidOutput1);

        m_SwerveDrive2.Set(gridPidOutput);
        pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), 0.0);
        m_SwerveTurn2.Set(pidOutput2);

        m_SwerveDrive3.Set(gridPidOutput);
        pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), 0.0);
        m_SwerveTurn3.Set(pidOutput3);

        m_SwerveDrive4.Set(gridPidOutput);
        pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), 0.0);
        m_SwerveTurn4.Set(pidOutput4);
        } else
        {
            if((m_DriveEncoder1.GetPosition() >= 34.5) && fullAuto3 && (fullAuto4 == false))
            {
                m_SwerveDrive1.Set(0.0);
                m_SwerveDrive2.Set(0.0);
                m_SwerveDrive3.Set(0.0);
                m_SwerveDrive4.Set(0.0);
                armSetPoint = autoArmSet;
                fullAuto4 = true;
            }
        }

        if (((m_gridSelected == 3.0) || (m_gridSelected == 6.0)) && (m_DriveEncoder1.GetPosition() > -34.5) && wheelAngleCheck)
        {
            gridPidOutput = m_gridController.Calculate(m_DriveEncoder1.GetPosition(), -36.5);
            m_SwerveDrive1.Set(gridPidOutput);
            pidOutput1 = m_angleController1.Calculate((m_SwerveAnalog1.GetVoltage() * conversion1), 0.0);
            m_SwerveTurn1.Set(pidOutput1);

            m_SwerveDrive2.Set(gridPidOutput);
            pidOutput2 = m_angleController2.Calculate((m_SwerveAnalog2.GetVoltage() * conversion2), 0.0);
            m_SwerveTurn2.Set(pidOutput2);

            m_SwerveDrive3.Set(gridPidOutput);
            pidOutput3 = m_angleController3.Calculate((m_SwerveAnalog3.GetVoltage() * conversion3), 0.0);
            m_SwerveTurn3.Set(pidOutput3);

            m_SwerveDrive4.Set(gridPidOutput);
            pidOutput4 = m_angleController4.Calculate((m_SwerveAnalog4.GetVoltage() * conversion4), 0.0);
            m_SwerveTurn4.Set(pidOutput4);
        } else
        {
            if((m_DriveEncoder1.GetPosition() <= -34.5) && fullAuto3 && (fullAuto4 == false))
            {
                m_SwerveDrive1.Set(0.0);
                m_SwerveDrive2.Set(0.0);
                m_SwerveDrive3.Set(0.0);
                m_SwerveDrive4.Set(0.0);
                armSetPoint = autoArmSet;
                fullAuto4 = true;
            }
        }
        if ((m_gridSelected == 2.0) || m_gridSelected == 5.0)
        {
            m_SwerveDrive1.Set(0.0);
            m_SwerveDrive2.Set(0.0);
            m_SwerveDrive3.Set(0.0);
            m_SwerveDrive4.Set(0.0);
            armSetPoint = autoArmSet;
            fullAuto4 = true;
        }
    }

    //5
    if (fullAuto4 && (fullAuto5 == false) && (m_encArm.GetAbsolutePosition() > armSetPoint - .05) 
    &&  (m_encArm.GetAbsolutePosition() < armSetPoint + .05))
    {
        if ((stringDude.GetVoltage() > 2.4))
        {
        chewyMotor.Set(-0.6);
        }
        else
        {
        fullAuto5 = true;
        }
    }
    if (fullAuto5 && (fullAuto6 == false))
    {
        chewyMotor.Set(0.8);
        //armSetPoint = 0.91;
        fullAuto6 = true;
        m_DriveEncoder1.SetPosition(0.0);
    } 

    tid = limelight_Table->GetNumber("tid", 0.0);
}