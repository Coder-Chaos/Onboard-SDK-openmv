/*! @file flight_control_sample.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "flight_control_demo.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

/*! Monitored Takeoff (Blocking API call). Return status as well as ack.
    This version of takeoff makes sure your aircraft actually took off
    and only returns when takeoff is complete.
    Use unless you want to do other stuff during takeoff - this will block
    the main thread.
!*/
bool
monitoredTakeoff(Vehicle* vehicle, int timeout)
{
  //@todo: remove this once the getErrorCode function signature changes
  char func[50];
  int  pkgIndex;

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    // Telemetry: Verify the subscription
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(timeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      return false;
    }

    // Telemetry: Subscribe to flight status and mode at freq 10 Hz
    pkgIndex                  = 0;
    int       freq            = 10;
    TopicName topicList10Hz[] = { TOPIC_STATUS_FLIGHT,
                                  TOPIC_STATUS_DISPLAYMODE };
    int  numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
    bool enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
      return pkgStatus;
    }
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, timeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      // Cleanup before return
      vehicle->subscribe->removePackage(pkgIndex, timeout);
      return false;
    }
  }

  // Start takeoff
  ACK::ErrorCode takeoffStatus = vehicle->control->takeoff(timeout);
  if (ACK::getError(takeoffStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(takeoffStatus, func);
    return false;
  }

  // First check: Motors started
  int motorsNotStarted = 0;
  int timeoutCycles    = 20;

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
             VehicleStatus::FlightStatus::ON_GROUND &&
           vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
             VehicleStatus::DisplayMode::MODE_ENGINE_START &&
           motorsNotStarted < timeoutCycles)
    {
      motorsNotStarted++;
      usleep(100000);
    }

    if (motorsNotStarted == timeoutCycles)
    {
      std::cout << "Takeoff failed. Motors are not spinning." << std::endl;
      // Cleanup
      if (!vehicle->isM100() && !vehicle->isLegacyM600())
      {
        vehicle->subscribe->removePackage(0, timeout);
      }
      return false;
    }
    else
    {
      std::cout << "Motors spinning...\n";
    }
  }
  else if (vehicle->isLegacyM600())
  {
    while ((vehicle->broadcast->getStatus().flight <
            DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND) &&
           motorsNotStarted < timeoutCycles)
    {
      motorsNotStarted++;
      usleep(100000);
    }

    if (motorsNotStarted < timeoutCycles)
    {
      std::cout << "Successful TakeOff!" << std::endl;
    }
  }
  else // M100
  {
    while ((vehicle->broadcast->getStatus().flight <
            DJI::OSDK::VehicleStatus::M100FlightStatus::TAKEOFF) &&
           motorsNotStarted < timeoutCycles)
    {
      motorsNotStarted++;
      usleep(100000);
    }

    if (motorsNotStarted < timeoutCycles)
    {
      std::cout << "Successful TakeOff!" << std::endl;
    }
  }

  // Second check: In air
  int stillOnGround = 0;
  timeoutCycles     = 110;

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
             VehicleStatus::FlightStatus::IN_AIR &&
           (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
              VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
              VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) &&
           stillOnGround < timeoutCycles)
    {
      stillOnGround++;
      usleep(100000);
    }

    if (stillOnGround == timeoutCycles)
    {
      std::cout << "Takeoff failed. Aircraft is still on the ground, but the "
                   "motors are spinning."
                << std::endl;
      // Cleanup
      if (!vehicle->isM100() && !vehicle->isLegacyM600())
      {
        vehicle->subscribe->removePackage(0, timeout);
      }
      return false;
    }
    else
    {
      std::cout << "Ascending...\n";
    }
  }
  else if (vehicle->isLegacyM600())
  {
    while ((vehicle->broadcast->getStatus().flight <
            DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR) &&
           stillOnGround < timeoutCycles)
    {
      stillOnGround++;
      usleep(100000);
    }

    if (stillOnGround < timeoutCycles)
    {
      std::cout << "Aircraft in air!" << std::endl;
    }
  }
  else // M100
  {
    while ((vehicle->broadcast->getStatus().flight !=
            DJI::OSDK::VehicleStatus::M100FlightStatus::IN_AIR_STANDBY) &&
           stillOnGround < timeoutCycles)
    {
      stillOnGround++;
      usleep(100000);
    }

    if (stillOnGround < timeoutCycles)
    {
      std::cout << "Aircraft in air!" << std::endl;
    }
  }

  // Final check: Finished takeoff
  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
           vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF)
    {
      sleep(1);
    }

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
      if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_P_GPS ||
          vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_ATTITUDE)
      {
        std::cout << "Successful takeoff!\n";
      }
      else
      {
        std::cout
          << "Takeoff finished, but the aircraft is in an unexpected mode. "
             "Please connect DJI GO.\n";
        vehicle->subscribe->removePackage(0, timeout);
        return false;
      }
    }
  }
  else
  {
    float32_t                 delta;
    Telemetry::GlobalPosition currentHeight;
    Telemetry::GlobalPosition deltaHeight =
      vehicle->broadcast->getGlobalPosition();

    do
    {
      sleep(4);
      currentHeight = vehicle->broadcast->getGlobalPosition();
      delta         = fabs(currentHeight.altitude - deltaHeight.altitude);
      deltaHeight.altitude = currentHeight.altitude;
    } while (delta >= 0.009);

    std::cout << "Aircraft hovering at " << currentHeight.altitude << "m!\n";
  }

  // Cleanup
  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
    if (ACK::getError(ack))
    {
      std::cout
        << "Error unsubscribing; please restart the drone/FC to get back "
           "to a clean state.\n";
    }
  }

  return true;
}

/*! Position Control. Allows you to set an offset from your current
    location. The aircraft will move to that position and stay there.
    Typical use would be as a building block in an outer loop that does not
    require many fast changes, perhaps a few-waypoint trajectory. For smoother
    transition and response you should convert your trajectory to attitude
    setpoints and use attitude control or convert to velocity setpoints
    and use velocity control.
!*/
bool
moveByPositionOffset(Vehicle *vehicle, float xOffsetDesired,
                     float yOffsetDesired, float zOffsetDesired,
                     float yawDesired, float posThresholdInM,
                     float yawThresholdInDeg)
{
  // Set timeout: this timeout is the time you allow the drone to take to finish
  // the
  // mission
  int responseTimeout              = 1;
  int timeoutInMilSec              = 40000;
  int controlFreqInHz              = 50; // Hz
  int cycleTimeInMs                = 1000 / controlFreqInHz;
  int outOfControlBoundsTimeLimit  = 10 * cycleTimeInMs; // 10 cycles
  int withinControlBoundsTimeReqmt = 50 * cycleTimeInMs; // 50 cycles
  int pkgIndex;

  //@todo: remove this once the getErrorCode function signature changes
  char func[50];

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    // Telemetry: Verify the subscription
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      return false;
    }

    // Telemetry: Subscribe to quaternion, fused lat/lon and altitude at freq 50
    // Hz
    pkgIndex                  = 0;
    int       freq            = 50;
    TopicName topicList50Hz[] = { TOPIC_QUATERNION, TOPIC_GPS_FUSED };
    int       numTopic = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
    bool      enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
      return pkgStatus;
    }
    subscribeStatus =
      vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      // Cleanup before return
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      return false;
    }

    // Also, since we don't have a source for relative height through subscription,
    // start using broadcast height
    if (!startGlobalPositionBroadcast(vehicle))
    {
      // Cleanup before return
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      return false;
    }
  }

  // Wait for data to come in
  sleep(1);

  // Get data

  // Global position retrieved via subscription
  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originSubscriptionGPS;
  // Global position retrieved via broadcast
  Telemetry::GlobalPosition currentBroadcastGP;
  Telemetry::GlobalPosition originBroadcastGP;

  // Convert position offset from first position to local coordinates
  Telemetry::Vector3f localOffset;

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    originSubscriptionGPS  = currentSubscriptionGPS;
    localOffsetFromGpsOffset(vehicle, localOffset,
                             static_cast<void*>(&currentSubscriptionGPS),
                             static_cast<void*>(&originSubscriptionGPS));

    // Get the broadcast GP since we need the height for zCmd
    currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
  }
  else
  {
    currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
    originBroadcastGP  = currentBroadcastGP;
    localOffsetFromGpsOffset(vehicle, localOffset,
                             static_cast<void*>(&currentBroadcastGP),
                             static_cast<void*>(&originBroadcastGP));
  }

  // Get initial offset. We will update this in a loop later.
  double xOffsetRemaining = xOffsetDesired - localOffset.x;
  double yOffsetRemaining = yOffsetDesired - localOffset.y;
  double zOffsetRemaining = zOffsetDesired - localOffset.z;

  // Conversions
  double yawDesiredRad     = DEG2RAD * yawDesired;
  double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;

  //! Get Euler angle

  // Quaternion retrieved via subscription
  Telemetry::TypeMap<TOPIC_QUATERNION>::type subscriptionQ;
  // Quaternion retrieved via broadcast
  Telemetry::Quaternion broadcastQ;

  double yawInRad;
  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
    yawInRad = toEulerAngle((static_cast<void*>(&subscriptionQ))).z / DEG2RAD;
  }
  else
  {
    broadcastQ = vehicle->broadcast->getQuaternion();
    yawInRad   = toEulerAngle((static_cast<void*>(&broadcastQ))).z / DEG2RAD;
  }

  int   elapsedTimeInMs     = 0;
  int   withinBoundsCounter = 0;
  int   outOfBounds         = 0;
  int   brakeCounter        = 0;
  int   speedFactor         = 2;
  float xCmd, yCmd, zCmd;

  /*! Calculate the inputs to send the position controller. We implement basic
   *  receding setpoint position control and the setpoint is always 1 m away
   *  from the current position - until we get within a threshold of the goal.
   *  From that point on, we send the remaining distance as the setpoint.
   */
  if (xOffsetDesired > 0)
    xCmd = (xOffsetDesired < speedFactor) ? xOffsetDesired : speedFactor;
  else if (xOffsetDesired < 0)
    xCmd =
      (xOffsetDesired > -1 * speedFactor) ? xOffsetDesired : -1 * speedFactor;
  else
    xCmd = 0;

  if (yOffsetDesired > 0)
    yCmd = (yOffsetDesired < speedFactor) ? yOffsetDesired : speedFactor;
  else if (yOffsetDesired < 0)
    yCmd =
      (yOffsetDesired > -1 * speedFactor) ? yOffsetDesired : -1 * speedFactor;
  else
    yCmd = 0;

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    zCmd = currentBroadcastGP.height + zOffsetDesired; //Since subscription cannot give us a relative height, use broadcast.
  }
  else
  {
    zCmd = currentBroadcastGP.height + zOffsetDesired;
  }

  //! Main closed-loop receding setpoint position control
  while (elapsedTimeInMs < timeoutInMilSec)
  {
    vehicle->control->positionAndYawCtrl(xCmd, yCmd, zCmd,
                                         yawDesiredRad / DEG2RAD);

    usleep(cycleTimeInMs * 1000);
    elapsedTimeInMs += cycleTimeInMs;

    //! Get current position in required coordinates and units
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
      subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
      yawInRad      = toEulerAngle((static_cast<void*>(&subscriptionQ))).z;
      currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
      localOffsetFromGpsOffset(vehicle, localOffset,
                               static_cast<void*>(&currentSubscriptionGPS),
                               static_cast<void*>(&originSubscriptionGPS));

      // Get the broadcast GP since we need the height for zCmd
      currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
    }
    else
    {
      broadcastQ         = vehicle->broadcast->getQuaternion();
      yawInRad           = toEulerAngle((static_cast<void*>(&broadcastQ))).z;
      currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
      localOffsetFromGpsOffset(vehicle, localOffset,
                               static_cast<void*>(&currentBroadcastGP),
                               static_cast<void*>(&originBroadcastGP));
    }

    //! See how much farther we have to go
    xOffsetRemaining = xOffsetDesired - localOffset.x;
    yOffsetRemaining = yOffsetDesired - localOffset.y;
    zOffsetRemaining = zOffsetDesired - localOffset.z;

    //! See if we need to modify the setpoint
    if (std::abs(xOffsetRemaining) < speedFactor)
    {
      xCmd = xOffsetRemaining;
    }
    if (std::abs(yOffsetRemaining) < speedFactor)
    {
      yCmd = yOffsetRemaining;
    }

    if (vehicle->isM100() && std::abs(xOffsetRemaining) < posThresholdInM &&
        std::abs(yOffsetRemaining) < posThresholdInM &&
        std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
    {
      //! 1. We are within bounds; start incrementing our in-bound counter
      withinBoundsCounter += cycleTimeInMs;
    }
    else if (std::abs(xOffsetRemaining) < posThresholdInM &&
             std::abs(yOffsetRemaining) < posThresholdInM &&
             std::abs(zOffsetRemaining) < posThresholdInM &&
             std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
    {
      //! 1. We are within bounds; start incrementing our in-bound counter
      withinBoundsCounter += cycleTimeInMs;
    }
    else
    {
      if (withinBoundsCounter != 0)
      {
        //! 2. Start incrementing an out-of-bounds counter
        outOfBounds += cycleTimeInMs;
      }
    }
    //! 3. Reset withinBoundsCounter if necessary
    if (outOfBounds > outOfControlBoundsTimeLimit)
    {
      withinBoundsCounter = 0;
      outOfBounds         = 0;
    }
    //! 4. If within bounds, set flag and break
    if (withinBoundsCounter >= withinControlBoundsTimeReqmt)
    {
      break;
    }
  }

  //! Set velocity to zero, to prevent any residual velocity from position
  //! command
  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    while (brakeCounter < withinControlBoundsTimeReqmt)
    {
      vehicle->control->emergencyBrake();
      usleep(cycleTimeInMs * 10);
      brakeCounter += cycleTimeInMs;
    }
  }

  if (elapsedTimeInMs >= timeoutInMilSec)
  {
    std::cout << "Task timeout!\n";
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
      ACK::ErrorCode ack =
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      if (ACK::getError(ack))
      {
        std::cout << "Error unsubscribing; please restart the drone/FC to get "
                     "back to a clean state.\n";
      }
    }
    return ACK::FAIL;
  }

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    ACK::ErrorCode ack =
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    if (ACK::getError(ack))
    {
      std::cout
        << "Error unsubscribing; please restart the drone/FC to get back "
           "to a clean state.\n";
    }
  }

  return ACK::SUCCESS;
}

/*! Position Control By attitudeAndVertPosCtrl. For smoother
    transition and response, convert trajectory to attitude
    setpoints and use attitude control.
!*/
bool moveByattitudeAndVertPosCtrl(Vehicle *vehicle, float xOffsetDesired,
                          float yOffsetDesired, float zOffsetDesired,
                          float yawDesired, float posThresholdInM,
                          float yawThresholdInDeg)
{

  clock_t startTime, endTime0, endTime1, endTime2;
  startTime = clock();//计时开始
  // Set timeout: this timeout is the time you allow the drone to take to finish
  // the
  // mission
  int responseTimeout              = 1;
  int timeoutInMilSec              = 40000;
  int controlFreqInHz              = 50; // Hz
  int cycleTimeInMs                = 1000 / controlFreqInHz;
  int outOfControlBoundsTimeLimit  = 10 * cycleTimeInMs; // 10 cycles
  int withinControlBoundsTimeReqmt = 2 * cycleTimeInMs; // 50 cycles
  int pkgIndex;

  //Parameters for PID.
  float pos_p[3],vel_p[3],vel_i[3],vel_d[3];
  memset(pos_p, 0, sizeof(pos_p));
  memset(vel_p, 0, sizeof(vel_p));
  memset(vel_i, 0, sizeof(vel_i));
  memset(vel_d, 0, sizeof(vel_d));
 
  pos_p[0] = 2.0f;
  vel_p[0] = 0.15f;
  vel_d[0] = 0.5f;

  pos_p[1] = 2.0f;
  vel_p[1] = 0.15f;
  vel_d[1] = 0.5f;

  pos_p[2] = 0.1f;
  vel_p[2] = 0.1f;
  vel_d[2] = 0.0f;

  int VelMax_xy = 2;
  int VelMax_up = 1;
  int VelMax_down = 1;
  float AngleMax_xy = 30;

  //@todo: remove this once the getErrorCode function signature changes
  char func[50];

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    // Telemetry: Verify the subscription
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      return false;
    }

    // Telemetry: Subscribe to quaternion, fused lat/lon and altitude at freq 50
    // Hz
    pkgIndex                  = 0;
    int       freq            = 50;
    TopicName topicList50Hz[] = { TOPIC_QUATERNION, TOPIC_GPS_FUSED };
    int       numTopic = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
    bool      enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
      return pkgStatus;
    }
    subscribeStatus =
      vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      // Cleanup before return
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      return false;
    }

    // Also, since we don't have a source for relative height through subscription,
    // start using broadcast height
    if (!startGlobalPositionBroadcast(vehicle))
    {
      // Cleanup before return
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      return false;
    }
  }

  // Wait for data to come in
  sleep(1);

  // Get data

  // Global position retrieved via subscription
  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originSubscriptionGPS;
  // Global position retrieved via broadcast
  Telemetry::GlobalPosition currentBroadcastGP;
  Telemetry::GlobalPosition originBroadcastGP;

  // Convert position offset from first position to local coordinates
  Telemetry::Vector3f localOffset;

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    originSubscriptionGPS  = currentSubscriptionGPS;
    localOffsetFromGpsOffset(vehicle, localOffset,
                             static_cast<void*>(&currentSubscriptionGPS),
                             static_cast<void*>(&originSubscriptionGPS));

    // Get the broadcast GP since we need the height for zCmd
    currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
  }
  else
  {
    currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
    originBroadcastGP  = currentBroadcastGP;
    localOffsetFromGpsOffset(vehicle, localOffset,
                             static_cast<void*>(&currentBroadcastGP),
                             static_cast<void*>(&originBroadcastGP));
  }

  // Get initial offset. We will update this in a loop later.
  double xOffsetRemaining = xOffsetDesired - localOffset.x;
  double yOffsetRemaining = yOffsetDesired - localOffset.y;
  double zOffsetRemaining = zOffsetDesired - localOffset.z;

  //Get intitial desired velocity by pos_p.
  double xVelocityDesired = pos_p[0] * xOffsetRemaining;
  double yVelocityDesired = pos_p[1] * yOffsetRemaining;
  double zVelocityDesired = pos_p[2] * zOffsetRemaining;

  //! Limit the xy velocity to 8m/s. 
  
  double VelNorm_xy = sqrt(pow(xVelocityDesired, 2) + pow(yVelocityDesired, 2));
  xVelocityDesired = (VelNorm_xy < VelMax_xy) ? xVelocityDesired : 
    xVelocityDesired * VelMax_xy / VelNorm_xy;
  yVelocityDesired = (VelNorm_xy < VelMax_xy) ? yVelocityDesired : 
    yVelocityDesired * VelMax_xy / VelNorm_xy;
  /* Saturate velocity in D-direction */
	zVelocityDesired = (zVelocityDesired < -VelMax_up) ? -VelMax_up : 
    ((zVelocityDesired > VelMax_down) ? VelMax_down : zVelocityDesired);

  // velocity retrieved via subscription
  Telemetry::Vector3f currentVelocity;
  Telemetry::Vector3f originVelocity;
  currentVelocity = vehicle->broadcast->getVelocity();
  originVelocity = currentVelocity;
  double xVelocityOffset = xVelocityDesired - currentVelocity.x;
  double yVelocityOffset = yVelocityDesired - currentVelocity.y;
  double zVelocityOffset = zVelocityDesired - currentVelocity.z;

  // Convert position offset from first position to local coordinates
  //Telemetry::Vector3f velocityOffset;

  // Conversions
  double yawDesiredRad     = DEG2RAD * yawDesired;
  double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;

  //! Get Euler angle
  // Quaternion retrieved via subscription
  Telemetry::TypeMap<TOPIC_QUATERNION>::type subscriptionQ;
  // Quaternion retrieved via broadcast
  Telemetry::Quaternion broadcastQ;
  double yawInRad, rollInRad;
  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
    yawInRad = toEulerAngle((static_cast<void*>(&subscriptionQ))).z ;
  }
  else
  {
    broadcastQ = vehicle->broadcast->getQuaternion();
    //std::cout << "broadcastQ.q00: " << broadcastQ.q0 << std::endl;
    yawInRad   = toEulerAngle((static_cast<void*>(&broadcastQ))).z ;
    rollInRad      = toEulerAngle((static_cast<void*>(&subscriptionQ))).y;
  }

  // No offset in body frame.
  double xVelocityRemaining = xVelocityOffset;
  double yVelocityRemaining = yVelocityOffset;
  double zVelocityRemaining = zVelocityOffset;

  int   elapsedTimeInMs     = 0;
  int   withinBoundsCounter = 0;
  int   outOfBounds         = 0;
  int   brakeCounter        = 0;
  int   speedFactor         = 5;
  float xCmd, yCmd, zCmd;

  double _xVelocityRemaining = xVelocityRemaining;
  double _yVelocityRemaining = yVelocityRemaining;
  double _zVelocityRemaining = zVelocityRemaining;
  float xVelocityint, yVelocityint, zVelocityint;
  Telemetry::Vector3f thr_sp, att_sp;
  xVelocityint = 0;
  yVelocityint = 0;
  zVelocityint = 0;
  xVelocityint += vel_i[0] * xVelocityRemaining;
  yVelocityint += vel_i[1] * yVelocityRemaining;
  zVelocityint += vel_i[2] * zVelocityRemaining;
  
  thr_sp.x = vel_p[0] * xVelocityRemaining + xVelocityint + 
    vel_d[0] * (xVelocityRemaining - _xVelocityRemaining);
  thr_sp.y = vel_p[1] * yVelocityRemaining + yVelocityint + 
    vel_d[1] * (yVelocityRemaining - _yVelocityRemaining);
  thr_sp.z = vel_p[2] * zVelocityRemaining + zVelocityint + 
    vel_d[2] * (zVelocityRemaining - _zVelocityRemaining) - 0.5f;

  thrustToAttitude(thr_sp, att_sp, yawDesiredRad);
  att_sp.x = (fabsf(att_sp.x) < AngleMax_xy * DEG2RAD) ? att_sp.x : 
    AngleMax_xy * DEG2RAD * (att_sp.x > 0 ? 1 : -1);
  att_sp.y = (fabsf(att_sp.y) < AngleMax_xy * DEG2RAD) ? att_sp.y : 
    AngleMax_xy * DEG2RAD * (att_sp.y > 0 ? 1 : -1);

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    zCmd = currentBroadcastGP.height + zOffsetDesired; //Since subscription cannot give us a relative height, use broadcast.
  }
  else
  {
    zCmd = currentBroadcastGP.height + zOffsetDesired;
  }

  //std::cout << "yOffsetRemaining: " << yOffsetRemaining << std::endl;
  
  std::cout << "Px: "     << xOffsetDesired - xOffsetRemaining << std::endl;
  std::cout << "Vx: "     << currentVelocity.x << std::endl;
  std::cout << "Vx_sp: "  << xVelocityDesired << std::endl;
  std::cout << "Py: "     << yOffsetDesired - yOffsetRemaining << std::endl;
  std::cout << "Vy: "     << currentVelocity.y << std::endl;
  std::cout << "Vy_sp: "  << yVelocityDesired << std::endl;
  //std::cout << "rollInDEG: " << rollInRad / DEG2RAD << std::endl;
  std::cout << "Pitch: " << att_sp.x / DEG2RAD << std::endl;
  std::cout << "Roll: " << att_sp.y / DEG2RAD << std::endl;

  int i = 0;
  endTime0 =clock();
  std::cout << "The run time0 is:" <<(double)(endTime0 - startTime) / 1000000 << "s" << std::endl;
  //! Main closed-loop receding setpoint position control
  while (elapsedTimeInMs < timeoutInMilSec)
  {
    
    vehicle->control->attitudeAndVertPosCtrl(att_sp.y / DEG2RAD, att_sp.x / DEG2RAD,
                                      yawDesiredRad / DEG2RAD, zCmd);

    usleep(cycleTimeInMs * 1000);
    //sleep(cycleTimeInMs);
    elapsedTimeInMs += cycleTimeInMs;

    //! Get current position in required coordinates and units
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
      subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
      yawInRad      = toEulerAngle((static_cast<void*>(&subscriptionQ))).z;
      currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
      localOffsetFromGpsOffset(vehicle, localOffset,
                               static_cast<void*>(&currentSubscriptionGPS),
                               static_cast<void*>(&originSubscriptionGPS));

      // Get the broadcast GP since we need the height for zCmd
      currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
    }
    else
    {
      broadcastQ         = vehicle->broadcast->getQuaternion();
      yawInRad           = toEulerAngle((static_cast<void*>(&broadcastQ))).z;
      rollInRad      = toEulerAngle((static_cast<void*>(&subscriptionQ))).y;
      currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
      currentVelocity = vehicle->broadcast->getVelocity();
      localOffsetFromGpsOffset(vehicle, localOffset,
                               static_cast<void*>(&currentBroadcastGP),
                               static_cast<void*>(&originBroadcastGP));
    }

    //! See how much farther we have to go
    xOffsetRemaining = xOffsetDesired - localOffset.x;
    yOffsetRemaining = yOffsetDesired - localOffset.y;
    zOffsetRemaining = zOffsetDesired - localOffset.z;

    xVelocityDesired = pos_p[0] * xOffsetRemaining;
    yVelocityDesired = pos_p[1] * yOffsetRemaining;
    zVelocityDesired = pos_p[2] * zOffsetRemaining;
    VelNorm_xy = sqrt(pow(xVelocityDesired, 2) + pow(yVelocityDesired, 2));
    xVelocityDesired = (VelNorm_xy < VelMax_xy) ? xVelocityDesired : 
      xVelocityDesired * VelMax_xy / VelNorm_xy;
    yVelocityDesired = (VelNorm_xy < VelMax_xy) ? yVelocityDesired : 
      yVelocityDesired * VelMax_xy / VelNorm_xy;
    /* Saturate velocity in D-direction */
	  zVelocityDesired = (zVelocityDesired < -VelMax_up) ? -VelMax_up : 
      ((zVelocityDesired > VelMax_down) ? VelMax_down : zVelocityDesired);

    xVelocityOffset = xVelocityDesired - currentVelocity.x;
    yVelocityOffset = yVelocityDesired - currentVelocity.y;
    zVelocityOffset = zVelocityDesired - currentVelocity.z;

    xVelocityRemaining = xVelocityOffset;
    yVelocityRemaining = yVelocityOffset;
    zVelocityRemaining = zVelocityOffset;

    xVelocityint += vel_i[0] * xVelocityRemaining;
    yVelocityint += vel_i[1] * yVelocityRemaining;
    zVelocityint += vel_i[2] * zVelocityRemaining;

    thr_sp.x = vel_p[0] * xVelocityRemaining + xVelocityint + 
      vel_d[0] * (xVelocityRemaining - _xVelocityRemaining);
    thr_sp.y = vel_p[1] * yVelocityRemaining + yVelocityint + 
      vel_d[1] * (yVelocityRemaining - _yVelocityRemaining);
    thr_sp.z = vel_p[2] * zVelocityRemaining + zVelocityint + 
      vel_d[2] * (zVelocityRemaining - _zVelocityRemaining) - 0.5f;

    thrustToAttitude(thr_sp, att_sp, yawDesiredRad);
    att_sp.x = (fabsf(att_sp.x) < AngleMax_xy * DEG2RAD) ? att_sp.x : 
      AngleMax_xy * DEG2RAD * (att_sp.x > 0 ? 1 : -1);
    att_sp.y = (fabsf(att_sp.y) < AngleMax_xy * DEG2RAD) ? att_sp.y : 
      AngleMax_xy * DEG2RAD * (att_sp.y > 0 ? 1 : -1);
    

    _xVelocityRemaining = xVelocityRemaining;
    _yVelocityRemaining = yVelocityRemaining;
    _zVelocityRemaining = zVelocityRemaining;

    if(i % 20 == 0){
      endTime1 = clock();//计时结束
      std::cout << "The run time1 is:" <<(double)(endTime1 - startTime) / 1000000 << "s" << std::endl;
      std::cout << "Px: " << xOffsetDesired - xOffsetRemaining << std::endl;
      std::cout << "Vx: " << currentVelocity.x<< std::endl;
      std::cout << "Py: " << yOffsetDesired - yOffsetRemaining << std::endl;
      std::cout << "Vy: " << currentVelocity.y<< std::endl;
      //std::cout << "xVelocityDesired: " << xVelocityDesired << std::endl;
      //std::cout << "rollInDEG: " << rollInRad / DEG2RAD << std::endl;
      //std::cout << "att_sp.xDEG: " << att_sp.x / DEG2RAD << std::endl;
      //std::cout << "att_sp.yDEG: " << att_sp.y / DEG2RAD << std::endl;
      
      
    } else if ( i > 200){
        break;
      }
    i ++;
    

    if (vehicle->isM100() && std::abs(xOffsetRemaining) < posThresholdInM &&
        std::abs(yOffsetRemaining) < posThresholdInM &&
        std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad && 
        std::abs(currentVelocity.x) < 0.01f && 
        std::abs(currentVelocity.y) < 0.01f)
    {
      //! 1. We are within bounds; start incrementing our in-bound counter
      withinBoundsCounter += cycleTimeInMs;
      //std::cout << "yVelocityDesired: " << yVelocityDesired << std::endl;
      endTime2 = clock();//计时结束
      std::cout << "The run time2 is:" <<(double)(endTime2 - startTime) / 1000000 << "s" << std::endl;
      std::cout << "Px: " << xOffsetDesired - xOffsetRemaining << std::endl;
      std::cout << "Vx: " << currentVelocity.x<< std::endl;
      std::cout << "Py: " << yOffsetDesired - yOffsetRemaining << std::endl;
      std::cout << "Vy: " << currentVelocity.y<< std::endl;
      //std::cout << "yawInRad - yawDesiredRad: " << yawInRad - yawDesiredRad << std::endl;
      //break;
    }
    /*else if (std::abs(xOffsetRemaining) < posThresholdInM &&
             std::abs(yOffsetRemaining) < posThresholdInM &&
             std::abs(zOffsetRemaining) < posThresholdInM &&
             std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
    {
      //! 1. We are within bounds; start incrementing our in-bound counter
      withinBoundsCounter += cycleTimeInMs;
    }
    else
    {
      if (withinBoundsCounter != 0)
      {
        //! 2. Start incrementing an out-of-bounds counter
        outOfBounds += cycleTimeInMs;
      }
    }
    //! 3. Reset withinBoundsCounter if necessary
    if (outOfBounds > outOfControlBoundsTimeLimit)
    {
      withinBoundsCounter = 0;
      outOfBounds         = 0;
    }
    //! 4. If within bounds, set flag and break
    if (withinBoundsCounter >= withinControlBoundsTimeReqmt)
    {
      //break;
    }*/
  }

  //! Set velocity to zero, to prevent any residual velocity from position
  //! command
  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    while (brakeCounter < withinControlBoundsTimeReqmt)
    {
      vehicle->control->emergencyBrake();
      usleep(cycleTimeInMs * 10);
      brakeCounter += cycleTimeInMs;
    }
  }

  if (elapsedTimeInMs >= timeoutInMilSec)
  {
    std::cout << "Task timeout!\n";
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
      ACK::ErrorCode ack =
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      if (ACK::getError(ack))
      {
        std::cout << "Error unsubscribing; please restart the drone/FC to get "
                     "back to a clean state.\n";
      }
    }
    return ACK::FAIL;
  }

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    ACK::ErrorCode ack =
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    if (ACK::getError(ack))
    {
      std::cout
        << "Error unsubscribing; please restart the drone/FC to get back "
           "to a clean state.\n";
    }
  }

  return ACK::SUCCESS;
}

/*! Monitored Takeoff (Blocking API call). Return status as well as ack.
    This version of takeoff makes sure your aircraft actually took off
    and only returns when takeoff is complete.
    Use unless you want to do other stuff during takeoff - this will block
    the main thread.
!*/
bool
monitoredLanding(Vehicle* vehicle, int timeout)
{
  //@todo: remove this once the getErrorCode function signature changes
  char func[50];
  int  pkgIndex;

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    // Telemetry: Verify the subscription
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(timeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      return false;
    }

    // Telemetry: Subscribe to flight status and mode at freq 10 Hz
    pkgIndex                  = 0;
    int       freq            = 10;
    TopicName topicList10Hz[] = { TOPIC_STATUS_FLIGHT,
                                  TOPIC_STATUS_DISPLAYMODE };
    int  numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
    bool enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
      return pkgStatus;
    }
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, timeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, func);
      // Cleanup before return
      vehicle->subscribe->removePackage(pkgIndex, timeout);
      return false;
    }
  }

  // Start landing
  ACK::ErrorCode landingStatus = vehicle->control->land(timeout);
  if (ACK::getError(landingStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(landingStatus, func);
    return false;
  }

  // First check: Landing started
  int landingNotStarted = 0;
  int timeoutCycles     = 20;

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
             VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
           landingNotStarted < timeoutCycles)
    {
      landingNotStarted++;
      usleep(100000);
    }
  }
  else if (vehicle->isM100())
  {
    while (vehicle->broadcast->getStatus().flight !=
             DJI::OSDK::VehicleStatus::M100FlightStatus::LANDING &&
           landingNotStarted < timeoutCycles)
    {
      landingNotStarted++;
      usleep(100000);
    }
  }

  if (landingNotStarted == timeoutCycles)
  {
    std::cout << "Landing failed. Aircraft is still in the air." << std::endl;
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
      // Cleanup before return
      ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
      if (ACK::getError(ack)) {
        std::cout << "Error unsubscribing; please restart the drone/FC to get "
                     "back to a clean state.\n";
      }
    }
    return false;
  }
  else
  {
    std::cout << "Landing...\n";
  }

  // Second check: Finished landing
  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
           vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
             VehicleStatus::FlightStatus::IN_AIR)
    {
      sleep(1);
    }

    if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
          VehicleStatus::DisplayMode::MODE_P_GPS ||
        vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
          VehicleStatus::DisplayMode::MODE_ATTITUDE)
    {
      std::cout << "Successful landing!\n";
    }
    else
    {
      std::cout
        << "Landing finished, but the aircraft is in an unexpected mode. "
           "Please connect DJI GO.\n";
      ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
      if (ACK::getError(ack))
      {
        std::cout << "Error unsubscribing; please restart the drone/FC to get "
                     "back to a clean state.\n";
      }
      return false;
    }
  }
  else if (vehicle->isLegacyM600())
  {
    while (vehicle->broadcast->getStatus().flight >
           DJI::OSDK::VehicleStatus::FlightStatus::STOPED)
    {
      sleep(1);
    }

    Telemetry::GlobalPosition gp;
    do
    {
      sleep(2);
      gp = vehicle->broadcast->getGlobalPosition();
    } while (gp.altitude != 0);

    if (gp.altitude != 0)
    {
      std::cout
        << "Landing finished, but the aircraft is in an unexpected mode. "
           "Please connect DJI GO.\n";
      return false;
    }
    else
    {
      std::cout << "Successful landing!\n";
    }
  }
  else // M100
  {
    while (vehicle->broadcast->getStatus().flight ==
           DJI::OSDK::VehicleStatus::M100FlightStatus::FINISHING_LANDING)
    {
      sleep(1);
    }

    Telemetry::GlobalPosition gp;
    do
    {
      sleep(2);
      gp = vehicle->broadcast->getGlobalPosition();
    } while (gp.altitude != 0);

    if (gp.altitude != 0)
    {
      std::cout
        << "Landing finished, but the aircraft is in an unexpected mode. "
           "Please connect DJI GO.\n";
      return false;
    }
    else
    {
      std::cout << "Successful landing!\n";
    }
  }

  // Cleanup
  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
    if (ACK::getError(ack))
    {
      std::cout
        << "Error unsubscribing; please restart the drone/FC to get back "
           "to a clean state.\n";
    }
  }

  return true;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates.
    Accurate when distances are small.
!*/
void
localOffsetFromGpsOffset(Vehicle* vehicle, Telemetry::Vector3f& deltaNed,
                         void* target, void* origin)
{
  Telemetry::GPSFused*       subscriptionTarget;
  Telemetry::GPSFused*       subscriptionOrigin;
  Telemetry::GlobalPosition* broadcastTarget;
  Telemetry::GlobalPosition* broadcastOrigin;
  double                     deltaLon;
  double                     deltaLat;

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    subscriptionTarget = (Telemetry::GPSFused*)target;
    subscriptionOrigin = (Telemetry::GPSFused*)origin;
    deltaLon   = subscriptionTarget->longitude - subscriptionOrigin->longitude;
    deltaLat   = subscriptionTarget->latitude - subscriptionOrigin->latitude;
    deltaNed.x = deltaLat * C_EARTH;
    deltaNed.y = deltaLon * C_EARTH * cos(subscriptionTarget->latitude);
    deltaNed.z = subscriptionTarget->altitude - subscriptionOrigin->altitude;
  }
  else
  {
    broadcastTarget = (Telemetry::GlobalPosition*)target;
    broadcastOrigin = (Telemetry::GlobalPosition*)origin;
    deltaLon        = broadcastTarget->longitude - broadcastOrigin->longitude;
    deltaLat        = broadcastTarget->latitude - broadcastOrigin->latitude;
    deltaNed.x      = deltaLat * C_EARTH;
    deltaNed.y      = deltaLon * C_EARTH * cos(broadcastTarget->latitude);
    deltaNed.z      = broadcastTarget->altitude - broadcastOrigin->altitude;
  }
}

/**
* Constructor from quaternion
*
* Instance is initialized from quaternion representing
* coordinate transformation from frame 2 to frame 1.
*
* @param q quaternion to set dcm to
*/
/*void dcmofQ(float dcm[3][3], Quaternion q)
{
    
    float32_t a = q.q0;
    float32_t b = q.q1;
    float32_t c = q.q2;
    float32_t d = q.q3;
    float32_t aa = a * a;
    float32_t ab = a * b;
    float32_t ac = a * c;
    float32_t ad = a * d;
    float32_t bb = b * b;
    float32_t bc = b * c;
    float32_t bd = b * d;
    float32_t cc = c * c;
    float32_t cd = c * d;
    float32_t dd = d * d;
    dcm[0][0] = aa + bb - cc - dd;
    dcm[0][1] = 2 * (bc + ad);
    dcm[0][2] = 2 * (-ac + bd);
    dcm[1][0] = 2 * (bc - ad);
    dcm[1][1] = aa - bb + cc - dd;
    dcm[1][2] = 2 * (cd + ab);
    dcm[2][0] = 2 * (bd + ac);
    dcm[2][1] = 2 * (-ab + cd);
    dcm[2][2] = aa - bb - cc + dd;
}*/

//Caculate length of Vector
float vector3fLength(Telemetry::Vector3f& v)
{
  float a;
  a = sqrt(v.x *v.x + v.y *v.y + v.z *v.z);
  return a;
}

void thrustToAttitude(Telemetry::Vector3f& thr_sp, Telemetry::Vector3f& att_sp, float yaw_sp)
{

	att_sp.z = yaw_sp;

	/* desired body_z axis = -normalize(thrust_vector) */
	Telemetry::Vector3f body_x, body_y, body_z;

  
	if (vector3fLength(thr_sp) > 0.00001f) {
		body_z.x = -thr_sp.x / vector3fLength(thr_sp);
    body_z.y = -thr_sp.y / vector3fLength(thr_sp);
    body_z.z = -thr_sp.z / vector3fLength(thr_sp);

	} else {
		/* no thrust, set Z axis to safe value */
		memset(&body_z, 0, sizeof(body_z));
		body_z.z = 1.0f;
	}

	/* vector of desired yaw direction in XY plane, rotated by PI/2 */
	Telemetry::Vector3f y_C = {-sinf(att_sp.z), cosf(att_sp.z), 0.0f};

	if (fabsf(body_z.z) > 0.000001f) {
		/* desired body_x axis, orthogonal to body_z */
		//body_x = y_C % body_z;
    body_x.x = y_C.y * body_z.z - y_C.z * body_z.y;
    body_x.y = -y_C.x * body_z.z + y_C.z * body_z.x;
    body_x.z = y_C.x * body_z.y - y_C.y * body_z.x;
   

		/* keep nose to front while inverted upside down */
		if (body_z.z < 0.0f) {
			body_x.x = -body_x.x;
      body_x.y = -body_x.y;
      body_x.z = -body_x.z;
		}

		//body_x.normalize();
    body_x.x = body_x.x / vector3fLength(body_x);
    body_x.y = body_x.y / vector3fLength(body_x);
    body_x.z = body_x.z / vector3fLength(body_x);

	} else {
		/* desired thrust is in XY plane, set X downside to construct correct matrix,
		 * but yaw component will not be used actually */
		//body_x.zero();
    memset(&body_x, 0, sizeof(body_x));
		body_x.z = 1.0f;
	}

	/* desired body_y axis */
	//body_y = body_z % body_x;
  body_y.x = body_z.y * body_x.z - body_z.z * body_x.y;
  body_y.y = -body_z.x * body_x.z + body_z.z * body_x.x;
  body_y.z = body_z.x * body_x.y - body_z.y * body_x.x;

	float32_t R_sp[3][3];

	/* fill rotation matrix */
	//for (int i = 0; i < 3; i++) {
	R_sp[0][0] = body_x.x;
  R_sp[1][0] = body_x.y;
  R_sp[2][0] = body_x.z;
	R_sp[0][1] = body_y.x;
  R_sp[1][1] = body_y.y;
  R_sp[2][1] = body_y.z;
	R_sp[0][2] = body_z.x;
  R_sp[1][2] = body_z.y;
  R_sp[2][2] = body_z.z;
	//}

	/* copy quaternion setpoint to attitude setpoint topic */
	Telemetry::Quaternion q;
	q.q0 = sqrt((0.0f < 1.0f + R_sp[0][0] + R_sp[1][1] + R_sp[2][2]) ? 
    1.0f + R_sp[0][0] + R_sp[1][1] + R_sp[2][2] : 0) / 2;
  q.q1 = sqrt((0.0f < 1.0f + R_sp[0][0] - R_sp[1][1] - R_sp[2][2]) ? 
    1.0f + R_sp[0][0] - R_sp[1][1] - R_sp[2][2] : 0) / 2;
  q.q2 = sqrt((0.0f < 1.0f - R_sp[0][0] + R_sp[1][1] - R_sp[2][2]) ? 
    1.0f - R_sp[0][0] + R_sp[1][1] - R_sp[2][2] : 0) / 2;
  q.q3 = sqrt((0.0f < 1.0f - R_sp[0][0] - R_sp[1][1] + R_sp[2][2]) ? 
    1.0f - R_sp[0][0] - R_sp[1][1] + R_sp[2][2] : 0) / 2;
  if(q.q1 * (R_sp[2][1] - R_sp[1][2]) < 0){q.q1 = - q.q1;} 
  else if(q.q1 * (R_sp[2][1] - R_sp[1][2]) == 0){q.q1 = 0;}
  //q.q1 *= sign(q.q1 * (R_sp[2][1] - R_sp[1][2]));
  if(q.q2 * (R_sp[0][2] - R_sp[2][0]) < 0){q.q2 = - q.q2;} 
  else if(q.q2 * (R_sp[0][2] - R_sp[2][0]) == 0){q.q2 = 0;}
  //q.q2 *= sign(q.q2 * (R_sp[0][2] - R_sp[2][0]));
  if(q.q3 * (R_sp[1][0] - R_sp[0][1]) < 0){q.q3 = - q.q3;} 
  else if(q.q3 * (R_sp[1][0] - R_sp[0][1]) == 0){q.q3 = 0;}
  //q.q3 *= sign(q.q3 * (R_sp[1][0] - R_sp[0][1]));
	
	/* calculate euler angles, for logging only, must not be used for control */
	att_sp.x = toEulerAngle((static_cast<void*>(&q))).x;
	att_sp.y = toEulerAngle((static_cast<void*>(&q))).y;
  
  /*std::cout << "body_z.x: " << body_z.x << std::endl;
  std::cout << "body_z.y: " << body_z.y << std::endl;
  std::cout << "body_z.z: " << body_z.z << std::endl;
  std::cout << "body_x.x: " << body_x.x << std::endl;
  std::cout << "body_x.y: " << body_x.y << std::endl;
  std::cout << "body_x.z: " << body_x.z << std::endl;
  std::cout << "body_y.x: " << body_y.x << std::endl;
  std::cout << "body_y.y: " << body_y.y << std::endl;
  std::cout << "body_y.z: " << body_y.z << std::endl;
  std::cout << "q.q0: " << q.q0 << std::endl;
  std::cout << "q.q1: " << q.q1 << std::endl;
  std::cout << "q.q2: " << q.q2 << std::endl;
  std::cout << "q.q3: " << q.q3 << std::endl;
  std::cout << "thr_sp.x: " << thr_sp.x << std::endl;
  
  std::cout << "thr_sp.z: " << thr_sp.z << std::endl;
  std::cout << "att_sp.x: " << att_sp.x << std::endl;
  */

}

Telemetry::Vector3f
toEulerAngle(void* quaternionData)
{
  Telemetry::Vector3f    ans;
  Telemetry::Quaternion* quaternion = (Telemetry::Quaternion*)quaternionData;

  double q2sqr = quaternion->q2 * quaternion->q2;
  double t0    = -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
  double t1 =
    +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
  double t2 =
    -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
  double t3 =
    +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
  double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

  t2 = (t2 > 1.0) ? 1.0 : t2;
  t2 = (t2 < -1.0) ? -1.0 : t2;

  ans.x = asin(t2);
  ans.y = atan2(t3, t4);
  ans.z = atan2(t1, t0);

  return ans;
}

bool startGlobalPositionBroadcast(Vehicle* vehicle)
{
  uint8_t freq[16];

  /* Channels definition for A3/N3/M600
   * 0 - Timestamp
   * 1 - Attitude Quaternions
   * 2 - Acceleration
   * 3 - Velocity (Ground Frame)
   * 4 - Angular Velocity (Body Frame)
   * 5 - Position
   * 6 - GPS Detailed Information
   * 7 - RTK Detailed Information
   * 8 - Magnetometer
   * 9 - RC Channels Data
   * 10 - Gimbal Data
   * 11 - Flight Status
   * 12 - Battery Level
   * 13 - Control Information
   */
  freq[0]  = DataBroadcast::FREQ_HOLD;
  freq[1]  = DataBroadcast::FREQ_HOLD;
  freq[2]  = DataBroadcast::FREQ_HOLD;
  freq[3]  = DataBroadcast::FREQ_HOLD;
  freq[4]  = DataBroadcast::FREQ_HOLD;
  freq[5]  = DataBroadcast::FREQ_50HZ; // This is the only one we want to change
  freq[6]  = DataBroadcast::FREQ_HOLD;
  freq[7]  = DataBroadcast::FREQ_HOLD;
  freq[8]  = DataBroadcast::FREQ_HOLD;
  freq[9]  = DataBroadcast::FREQ_HOLD;
  freq[10] = DataBroadcast::FREQ_HOLD;
  freq[11] = DataBroadcast::FREQ_HOLD;
  freq[12] = DataBroadcast::FREQ_HOLD;
  freq[13] = DataBroadcast::FREQ_HOLD;

  ACK::ErrorCode ack = vehicle->broadcast->setBroadcastFreq(freq, 1);
  if (ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, __func__);
    return false;
  }
  else
  {
    return true;
  }
}
