#include "RabMessageBuffer.h"

namespace argos {

  /****************************************/
  /****************************************/

  RabMessageBuffer::RabMessageBuffer() {
    m_unCurrentTime = 0;
  }

  /****************************************/
  /****************************************/

  RabMessageBuffer::~RabMessageBuffer() {}

  /****************************************/
  /****************************************/

  void RabMessageBuffer::SetTimeLife(const UInt32& un_max_time_to_live) {
    m_unMaxTimeToLive = un_max_time_to_live;
  }

  /****************************************/
  /****************************************/

  void RabMessageBuffer::Update() {
    if (m_unCurrentTime >= m_unMaxTimeToLive) {
      UInt32 i = 0;
      while (i < m_vecBufferElements.size()) {
        if (m_vecBufferElements.at(i).second < (m_unCurrentTime - m_unMaxTimeToLive)) {
          m_vecBufferElements.erase(m_vecBufferElements.begin() + i);
        } else {
          i += 1;
        }
      }
    }
    m_unCurrentTime += 1;
  }

  /****************************************/
	/****************************************/

  void RabMessageBuffer::AddMessage(CCI_XPuckRangeAndBearingSensor::SReceivedPacket* c_packet) {
    m_vecBufferElements.push_back(std::make_pair(*c_packet, m_unCurrentTime));
  }

  /****************************************/
  /****************************************/

  std::vector<CCI_XPuckRangeAndBearingSensor::SReceivedPacket*> RabMessageBuffer::GetMessages(){
    std::vector<CCI_XPuckRangeAndBearingSensor::SReceivedPacket*> vecRabMessages;
    std::vector<std::pair<CCI_XPuckRangeAndBearingSensor::SReceivedPacket, UInt32> >::iterator it;
    for (it = m_vecBufferElements.begin(); it != m_vecBufferElements.end(); it++) {
      vecRabMessages.push_back(&(*it).first);
    }
    return vecRabMessages;
  }

  /****************************************/
  /****************************************/

  void RabMessageBuffer::Reset() {
    m_vecBufferElements.clear();
    m_unCurrentTime = 0;
  }

}
