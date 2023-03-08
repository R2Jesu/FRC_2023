#include "Robot.h"


void Robot::R2Jesu_Grid()
{

  if (gridPad.GetRawButton(1))
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
     if ((m_DriveEncoder1.GetPosition() < 36.5) && wheelAngleCheck)
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
        m_SwerveDrive1.Set(0.0);
        m_SwerveDrive2.Set(0.0);
        m_SwerveDrive3.Set(0.0);
        m_SwerveDrive4.Set(0.0);
        //arm
    }
  }

  if (gridPad.GetRawButton(3))
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
     if ((m_DriveEncoder1.GetPosition() > -36.5) && wheelAngleCheck)
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
        m_SwerveDrive1.Set(0.0);
        m_SwerveDrive2.Set(0.0);
        m_SwerveDrive3.Set(0.0);
        m_SwerveDrive4.Set(0.0);
        //arm
    }
  }

    if (gridPad.GetRawButton(4))
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
        if ((m_DriveEncoder1.GetPosition() < 36.5) && wheelAngleCheck)
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
        m_SwerveDrive1.Set(0.0);
        m_SwerveDrive2.Set(0.0);
        m_SwerveDrive3.Set(0.0);
        m_SwerveDrive4.Set(0.0);
            //arm
        }
  }

  if (gridPad.GetRawButton(5))
  {
     //arm
  }

  if (gridPad.GetRawButton(6))
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
     if ((m_DriveEncoder1.GetPosition() > -36.5) && wheelAngleCheck)
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
        m_SwerveDrive1.Set(0.0);
        m_SwerveDrive2.Set(0.0);
        m_SwerveDrive3.Set(0.0);
        m_SwerveDrive4.Set(0.0);
        //arm
    }
  }
}