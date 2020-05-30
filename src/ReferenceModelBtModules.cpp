#include "ReferenceModelBtModules.h"

/****************************************/
/****************************************/

ReferenceModelBtModules::ReferenceModelBtModules() {
  m_pcRng = CRandom::CreateRNG("argos");
  m_pcRabMessageBuffer = RabMessageBuffer();
  m_pcRabMessageBuffer.SetTimeLife(10);
  m_fMaxVelocity = 12;
  m_fLeftWheelVelocity = 0;
  m_fRightWheelVelocity = 0;
  m_signalToSend = 0;
}

/****************************************/
/****************************************/

ReferenceModelBtModules::~ReferenceModelBtModules() {}

/****************************************/
/****************************************/

void ReferenceModelBtModules::Reset() {
  m_fLeftWheelVelocity = 0;
  m_fRightWheelVelocity = 0;
  m_pcRabMessageBuffer.Reset();
}

/****************************************/
/****************************************/

CCI_EPuckProximitySensor::SReading ReferenceModelBtModules::GetProximityReading() {
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

void ReferenceModelBtModules::SetProximityInput(CCI_EPuckProximitySensor::TReadings s_prox_input) {
  m_sProximityInput = s_prox_input;
}

/****************************************/
/****************************************/

CCI_EPuckLightSensor::SReading ReferenceModelBtModules::GetLightReading() {
  CCI_EPuckLightSensor::SReading cOutputReading;
  CVector2 cSumLight(0, CRadians::ZERO);
	for (UInt8 i = 0; i < m_sLightInput.size(); i++) {
    if (m_sLightInput[i].Value > 0.2) {
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

void ReferenceModelBtModules::SetLightInput(CCI_EPuckLightSensor::TReadings s_light_input) {
  m_sLightInput = s_light_input;
}

/****************************************/
/****************************************/

Real ReferenceModelBtModules::GetGroundReading() {
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

  if (unBlackWhiteCounter[0] > 10) {
    return 0.0f;
  }
  else if (unBlackWhiteCounter[1] > 10) {
    return 1.0f;
  }
  else {
    return 0.5f;
  }
}

/****************************************/
/****************************************/

void ReferenceModelBtModules::SetGroundInput(CCI_EPuckGroundSensor::SReadings s_ground_input) {
  m_deqGroundInput.push_back(s_ground_input);
  if (m_deqGroundInput.size() > 5) {
    m_deqGroundInput.pop_front();
  }
}

/****************************************/
/****************************************/

const UInt8 ReferenceModelBtModules::GetNumberNeighbors() const {
  return m_unNumberNeighbors;
}

/****************************************/
/****************************************/

void ReferenceModelBtModules::SetNumberNeighbors(const UInt8& un_number_neighbors){
  m_unNumberNeighbors = un_number_neighbors;
}

/****************************************/
/****************************************/

CCI_EPuckRangeAndBearingSensor::SReceivedPacket ReferenceModelBtModules::GetAttractionVectorToNeighbors(Real f_alpha_parameter) {
  CCI_EPuckRangeAndBearingSensor::TPackets sRabPackets = m_pcRabMessageBuffer.GetMessages();
  CCI_EPuckRangeAndBearingSensor::TPackets::iterator it;
  CVector2 sRabVectorSum(0,CRadians::ZERO);

  for (it = sRabPackets.begin(); it != sRabPackets.end(); it++) {
    if (((*it)->Data[0] != (UInt32) EpuckDAO::GetRobotIdentifier())) {
      sRabVectorSum += CVector2(f_alpha_parameter/(1 + (*it)->Range),(*it)->Bearing.SignedNormalize());
    }
  }

  sRabVectorSum.Angle() /= sRabPackets.size();

  CCI_EPuckRangeAndBearingSensor::SReceivedPacket cRaBReading;
  cRaBReading.Range = sRabVectorSum.Length();
  cRaBReading.Bearing = sRabVectorSum.Angle().SignedNormalize();

  return cRaBReading;
}


/****************************************/
/****************************************/

std::vector<CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> ReferenceModelBtModules::GetRangeAndBearingMessages() {
  return m_pcRabMessageBuffer.GetMessages();
}

/****************************************/
/****************************************/

void ReferenceModelBtModules::SetRangeAndBearingMessages(CCI_EPuckRangeAndBearingSensor::TPackets s_packets) {
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

/****************************************/
/****************************************/

void ReferenceModelBtModules::SetSignalToSend(UInt8 signal) {
    m_signalToSend = signal;
}

/****************************************/
/****************************************/

void ReferenceModelBtModules::GetRangeAndBearingSignalToSend(CCI_EPuckRangeAndBearingActuator::TData& data)
{
    WriteSignal(m_signalToSend, data);
}

/****************************************/
/****************************************/

UInt32 ReferenceModelBtModules::GetNumberSignalMatchingNeighbors(UInt8 signal)
{
    typedef std::vector<CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> Packets;
    Packets packets = m_pcRabMessageBuffer.GetMessages();
    UInt8 count = 0;

    for (Packets::iterator it = packets.begin(); it != packets.end(); ++it) {
        bool is_not_me = (*it)->Data[0] != EpuckDAO::GetRobotIdentifier();
        bool signal_match = TestSignalMatch((*it)->Data, signal);

        if (is_not_me && signal_match)  {
            ++count;
        }
    }

    return count;
}

/****************************************/
/****************************************/

CCI_EPuckRangeAndBearingSensor::SReceivedPacket
    ReferenceModelBtModules::GetAttractionVectorToSignalMatchingNeighbors(Real f_alpha_parameter, UInt8 signal)
{
    typedef std::vector<CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> Packets;
    Packets packets = m_pcRabMessageBuffer.GetMessages();

    CVector2 vector_sum(0, CRadians::ZERO);

    for (Packets::iterator it = packets.begin(); it != packets.end(); it++) {
        bool is_not_me = (*it)->Data[0] != EpuckDAO::GetRobotIdentifier();
        bool signal_match = TestSignalMatch((*it)->Data, signal);

        if (is_not_me && signal_match) {
            vector_sum += CVector2(f_alpha_parameter/(1 + (*it)->Range),(*it)->Bearing.SignedNormalize());
        }
    }

    vector_sum.Angle() /= packets.size();

    CCI_EPuckRangeAndBearingSensor::SReceivedPacket reading;
    reading.Range = vector_sum.Length();
    reading.Bearing = vector_sum.Angle().SignedNormalize();

    return reading;
}

/****************************************/
/****************************************/

CCI_EPuckRangeAndBearingSensor::SReceivedPacket
    ReferenceModelBtModules::GetSignalMatchingNeighborsCenterOfMass(UInt8 signal)
{
    typedef std::vector<CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> Packets;
    Packets packets = m_pcRabMessageBuffer.GetMessages();

    CVector2 vector_sum(0, CRadians::ZERO);

    for (Packets::iterator it = packets.begin(); it != packets.end(); it++) {
        bool is_not_me = (*it)->Data[0] != EpuckDAO::GetRobotIdentifier();
        bool signal_match = TestSignalMatch((*it)->Data, signal);

        if (is_not_me && signal_match) {
            vector_sum += CVector2((*it)->Range, (*it)->Bearing.SignedNormalize());
        }
    }

    vector_sum.Angle() /= packets.size();

    CCI_EPuckRangeAndBearingSensor::SReceivedPacket reading;
    reading.Range = vector_sum.Length();
    reading.Bearing = vector_sum.Angle().SignedNormalize();

    return reading;
}

/****************************************/
/****************************************/

bool ReferenceModelBtModules::TestSignalMatch(
    CCI_EPuckRangeAndBearingSensor::TData const& data,
    UInt8 signal) const
{
    // valid signals code
    if(signal < 0 || signal > 7) {
        LOGERR << "Got invalid signal " << signal << " (Must be in [0,7])" << std::endl;
        return false;
    }

    // signal is 3 bit number, one byte of data represent one bit
    // - 11111111 for signal bit = 1
    // - 00000000 for signal bit = 0
    // for avoiding errors during transmissions
    // first byte of data is the robot id, ignored

    UInt8 received_signal = 0;

    received_signal += ((data[1] > 127) ? 1 : 0) << 2;
    received_signal += ((data[2] > 127) ? 1 : 0) << 1;
    received_signal += ((data[3] > 127) ? 1 : 0);

    // now match signal with the one in argument
    if(signal == 7) {
        // signal 7 means all signals
        return received_signal != 0;
    }
    // else just check signals are equal
    return received_signal == signal;
}

bool ReferenceModelBtModules::WriteSignal(
    UInt8 signal,
    CCI_EPuckRangeAndBearingActuator::TData& data) const
{
    // valid signals code
    if(signal < 0 || signal > 7) {
        LOGERR << "Got invalid signal " << signal << " (Must be in [0,7])" << std::endl;
        return false;
    }

    // signal 7 cannot be send
    if(signal == 7) {
        LOGERR << "Tried to send special signal 7, which means 'all signals' (cannot be send)" << std::endl;
        return false;
    }

    // write signal

    // first byte is robot id
    data[0] = EpuckDAO::GetRobotIdentifier();

    // other bytes are expanded signals bits
    // if signal bit = 1, write 11111111
    // if signal bit = 0, write 00000000
    data[1] = (signal & 0b100) ? ~0 : 0;
    data[2] = (signal & 0b010) ? ~0 : 0;
    data[3] = (signal & 0b001) ? ~0 : 0;

    return true;
}
