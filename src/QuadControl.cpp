#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Map collective thrust and 3-axis moments to 4 motor commands via force allocation
  float l = L / sqrtf(2.f);
  float t1 = momentCmd.x / l;
  float t2 = momentCmd.y / l;
  float t3 = -momentCmd.z / kappa;
  float t4 = collThrustCmd;

  // Force allocation: solve for F1, F2, F3, F4 from T, Mx, My, Mz
  cmd.desiredThrustsN[0] = ( t1 + t2 + t3 + t4) / 4.f;  // front left
  cmd.desiredThrustsN[1] = (-t1 + t2 - t3 + t4) / 4.f;  // front right
  cmd.desiredThrustsN[2] = ( t1 - t2 - t3 + t4) / 4.f;  // rear left
  cmd.desiredThrustsN[3] = (-t1 - t2 + t3 + t4) / 4.f;  // rear right

  // Enforce motor thrust limits
  for (int i = 0; i < 4; i++)
    cmd.desiredThrustsN[i] = CONSTRAIN(cmd.desiredThrustsN[i], minMotorThrust, maxMotorThrust);


  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // P controller on body rates: M = I * kpPQR * (ωcmd - ω)
  V3F pqr_err = pqrCmd - pqr;
  V3F momentCmd = V3F(Ixx, Iyy, Izz) * kpPQR * pqr_err;


  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

    
    float b_x = R(0,2), b_y = R(1,2);
    float b_x_target = 0.f, b_y_target = 0.f;

    if (collThrustCmd > 0.f)
    {
      float c = -collThrustCmd / mass;          
      b_x_target = CONSTRAIN(accelCmd.x / c, -maxTiltAngle, maxTiltAngle);
      b_y_target = CONSTRAIN(accelCmd.y / c, -maxTiltAngle, maxTiltAngle);
    }

    float b_x_dot = kpBank * (b_x_target - b_x);
    float b_y_dot = kpBank * (b_y_target - b_y);

    pqrCmd.x = ( R(1,0)*b_x_dot - R(0,0)*b_y_dot) / R(2,2);
    pqrCmd.y = ( R(1,1)*b_x_dot - R(0,1)*b_y_dot) / R(2,2);
    pqrCmd.z = 0.f;


  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;


    float z_err = posZCmd - posZ;
    integratedAltitudeError += z_err * dt;

    velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);
    float z_err_dot = velZCmd - velZ;

    float u_1_bar = kpPosZ * z_err
                  + kpVelZ * z_err_dot
                  + KiPosZ * integratedAltitudeError
                  + accelZCmd;

    thrust = mass * ((float)CONST_GRAVITY - u_1_bar) / R(2,2);

  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

    if (velCmd.mag() > maxSpeedXY)          velCmd = velCmd.norm() * maxSpeedXY;

    V3F posErr, velErr;
    V3F p_term, d_term;
        
    posErr = posCmd - pos;
    velErr = velCmd - vel;
    velErr.z = 0;
    
    p_term.x = kpPosXY * posErr.x;
    p_term.y = kpPosXY * posErr.y;
    d_term.x = kpVelXY * velErr.x;
    d_term.y = kpVelXY * velErr.y;
    
    accelCmd = p_term + d_term + accelCmdFF;

    if (accelCmd.mag() > maxAccelXY)      accelCmd = accelCmd.norm() * maxAccelXY;
    accelCmd.z = 0;


  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  float yawRateCmd=0;
    
    float yawErr = yawCmd - yaw;
    yawErr = fmodf(yawErr, 2.f * F_PI);   // now in (-2pi, 2pi)
    if (yawErr > F_PI)       yawErr -= 2.f * F_PI;
    else if (yawErr <= -F_PI) yawErr += 2.f * F_PI;
    yawRateCmd = kpYaw * yawErr;


  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
