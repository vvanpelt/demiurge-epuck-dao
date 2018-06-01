#include "ReferenceModel1Dot2.h"

/****************************************/
/****************************************/

ReferenceModel1Dot2::ReferenceModel1Dot2() {
  m_pcRng = CRandom::CreateRNG("argos");
  m_pcRabMessageBuffer = RabMessageBuffer();
  m_pcRabMessageBuffer.SetTimeLife(10);
  m_fMaxVelocity = 12;
  m_fLeftWheelVelocity = 0;
  m_fRightWheelVelocity = 0;
}

/****************************************/
/****************************************/

ReferenceModel1Dot2::~ReferenceModel1Dot2() {}

/****************************************/
/****************************************/

void ReferenceModel1Dot2::Reset() {
  m_fLeftWheelVelocity = 0;
  m_fRightWheelVelocity = 0;
  m_pcRabMessageBuffer.Reset();
}

/****************************************/
/****************************************/

CCI_EPuckProximitySensor::SReading ReferenceModel1Dot2::GetProximityReading() {
  CCI_EPuckProximitySensor::SReading cOutputReading;
  CVector2 cSumProxi(0, CRadians::ZERO);
  for (UInt8 i = 0; i < m_sProximityInput.size(); i++) {
    cSumProxi += CVector2(m_sProximityInput[i].Value, m_sProximityInput[i].Angle.SignedNormalize());
  }

  cOutputReading.Value = (cSumProxi.Length() > 1) ? 1 : cSumProxi.Length();
  cOutputReading.Angle = cSumProxi.Angle().SignedNormalize();

  return cOutputReading;
}

/****************************************/
/****************************************/

void ReferenceModel1Dot2::SetProximityInput(CCI_EPuckProximitySensor::TReadings s_prox_input) {
  m_sProximityInput = s_prox_input;
}

/****************************************/
/****************************************/

CCI_EPuckLightSensor::SReading ReferenceModel1Dot2::GetLightReading() {
  CCI_EPuckLightSensor::SReading cOutputReading;
  CVector2 cSumLight(0, CRadians::ZERO);
	for (UInt8 i = 0; i < m_sLightInput.size(); i++) {
    if (m_sLightInput[i].Value > 0.1) {
      cOutputReading.Value = 1;
    }
		cSumLight += CVector2(m_sLightInput[i].Value, m_sLightInput[i].Angle.SignedNormalize());
	}
  if (cOutputReading.Value == 1) {
    cOutputReading.Angle = cSumLight.Angle().SignedNormalize();
  }
  return cOutputReading;
}

/****************************************/
/****************************************/

void ReferenceModel1Dot2::SetLightInput(CCI_EPuckLightSensor::TReadings s_light_input) {
  m_sLightInput = s_light_input;
}

/****************************************/
/****************************************/

CCI_EPuckGroundSensor::SReadings ReferenceModel1Dot2::GetGroundInput() {
  std::deque<CCI_EPuckGroundSensor::SReadings>::iterator it;
  UInt32 unBlackWhiteCounter[2] = {0,0};  //unBlackWhiteCounter[0] -> Black; unBlackWhiteCounter[1] -> White.
  float fBlackThreshold = 0.03;
  float fWhiteThreshold = 0.85;
  for (it = m_deqGroundInput.begin(); it != m_deqGroundInput.end(); it++) {
    if (it->Left < fBlackThreshold) {
      unBlackWhiteCounter[0] += 1;
    }
    else if (it->Left > fWhiteThreshold) {
      unBlackWhiteCounter[1] += 1;
    }
    if (it->Center < fBlackThreshold) {
      unBlackWhiteCounter[0] +=1;
    }
    else if (it->Center > fWhiteThreshold) {
      unBlackWhiteCounter[1] += 1;
    }
    if (it->Right < fBlackThreshold) {
      unBlackWhiteCounter[0] +=1;
    }
    else if (it->Right > fWhiteThreshold) {
      unBlackWhiteCounter[1] += 1;
    }
  }

  CCI_EPuckGroundSensor::SReadings readings;
  if (unBlackWhiteCounter[0] > 10) {
    readings.Left = 0;
    readings.Center = 0;
    readings.Right = 0;
  }
  else if (unBlackWhiteCounter[1] > 10) {
    readings.Left = 1;
    readings.Center = 1;
    readings.Right = 1;
  }
  else {
    readings.Left = 0.5;
    readings.Center = 0.5;
    readings.Right = 0.5;
  }

  return readings;
}

/****************************************/
/****************************************/

void ReferenceModel1Dot2::SetGroundInput(CCI_EPuckGroundSensor::SReadings s_ground_input) {
  m_deqGroundInput.push_back(s_ground_input);
  if (m_deqGroundInput.size() > 5) {
    m_deqGroundInput.pop_front();
  }
}

/****************************************/
/****************************************/

const UInt8 ReferenceModel1Dot2::GetNumberNeighbors() const {
  return m_unNumberNeighbors;
}

/****************************************/
/****************************************/

void ReferenceModel1Dot2::SetNumberNeighbors(const UInt8& un_number_neighbors){
  m_unNumberNeighbors = un_number_neighbors;
}

/****************************************/
/****************************************/

CVector2 ReferenceModel1Dot2::GetNeighborsCenterOfMass() {
  CCI_EPuckRangeAndBearingSensor::TPackets sRabPackets = m_pcRabMessageBuffer.GetMessages();
  CCI_EPuckRangeAndBearingSensor::TPackets::iterator it;
  CVector2 sRabVectorSum(0,CRadians::ZERO);

  for (it = sRabPackets.begin(); it != sRabPackets.end(); it++) {
    if ((*it)->Data[0] != (UInt8) EpuckDAO::GetRobotIdentifier()) {
      sRabVectorSum += CVector2((*it)->Range,(*it)->Bearing.SignedNormalize());
    }
  }
  sRabVectorSum /= sRabPackets.size();
  return sRabVectorSum;
}

/****************************************/
/****************************************/

std::vector<CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> ReferenceModel1Dot2::GetRangeAndBearingMessages() {
  return m_pcRabMessageBuffer.GetMessages();
}

/****************************************/
/****************************************/

void ReferenceModel1Dot2::SetRangeAndBearingMessages(CCI_EPuckRangeAndBearingSensor::TPackets s_packets) {
  std::map<UInt32, CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> mapRemainingMessages;
  std::map<UInt32, CCI_EPuckRangeAndBearingSensor::SReceivedPacket*>::iterator mapIt;
  CCI_EPuckRangeAndBearingSensor::TPackets::iterator it;
  m_unNumberNeighbors = 0;
  for (it = s_packets.begin(); it < s_packets.end(); ++it) {
    if ((*it)->Data[0] != m_unRobotIdentifier) {
      if (mapRemainingMessages.find((*it)->Data[0]) != mapRemainingMessages.end()) {  // If ID not in map, add message.
        mapRemainingMessages[(*it)->Data[0]] = (*it);
      } else if ((*it)->Bearing != CRadians::ZERO){  // If ID there, overwrite only if the message is valid (correct range and bearing information)
        mapRemainingMessages[(*it)->Data[0]] = (*it);
      }
    }
  }
  for (mapIt = mapRemainingMessages.begin(); mapIt != mapRemainingMessages.end(); ++mapIt) {
    m_pcRabMessageBuffer.AddMessage((*mapIt).second);
    m_unNumberNeighbors += 1;
  }
  m_pcRabMessageBuffer.Update();
}
