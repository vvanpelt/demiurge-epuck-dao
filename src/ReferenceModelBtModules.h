#pragma once

#include "EpuckDAO.h"
#include "RabMessageBuffer.h"

using namespace argos;

class ReferenceModelBtModules: public EpuckDAO {
  public:
    /*
     *  Class constructor.
     */
    ReferenceModelBtModules();

    /*
     * Class destructor.
     */
    virtual ~ReferenceModelBtModules();

    /*
     * Reset function.
     */
    virtual void Reset();

    /*
     * Getter for the proximity reading.
     */
    CCI_EPuckProximitySensor::SReading GetProximityReading();

    /*
     * Setter for the proximity input.
     */
    void SetProximityInput(CCI_EPuckProximitySensor::TReadings s_prox_input);

    /*
     * Getter for the light reading.
     */
    CCI_EPuckLightSensor::SReading GetLightReading();

    /*
     * Setter for the light input.
     */
    void SetLightInput(CCI_EPuckLightSensor::TReadings s_light_input);

    /*
     * Getter for the ground input.
     */
    Real GetGroundReading();

    /*
     * Setter for the ground input.
     */
    void SetGroundInput(CCI_EPuckGroundSensor::SReadings s_ground_input);

    /*
     * Getter for the number of surrounding robots.
     */
    const UInt8 GetNumberNeighbors() const;

    /*
     * Setter for the number of surrounding robots.
     */
    virtual void SetNumberNeighbors(const UInt8& un_number_neighbors);

    /*
     * Getter for the range-and-bearing input.
     */
    std::vector<CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> GetRangeAndBearingMessages();

    /*
     * Getter for the vector representing the attraction force to the neighbors computed with RaB messages
     */
    CCI_EPuckRangeAndBearingSensor::SReceivedPacket GetAttractionVectorToNeighbors(Real f_alpha_parameter);

    /*
     * Setter for the range-and-bearing input.
     */
    void SetRangeAndBearingMessages(CCI_EPuckRangeAndBearingSensor::TPackets s_packets);

    /*
     * Set the signal to send with this robot
     */
    virtual void SetSignalToSend(UInt8 signal);

    /*
     * Get the signal to send as a RaB packet
     */
    virtual void GetRangeAndBearingSignalToSend(CCI_EPuckRangeAndBearingActuator::TData& data);

    /*
     * Getter for the number of neighbors that have send a signal that
     * match with the one passed as argument
     */
    virtual UInt32 GetNumberSignalMatchingNeighbors(UInt8 signal);

    /*
     * Getter for attraction force to the neighbors that are sending a
     * signal matching with the one passed as argument
     */
    virtual CCI_EPuckRangeAndBearingSensor::SReceivedPacket
        GetAttractionVectorToSignalMatchingNeighbors(Real f_alpha_parameter, UInt8 signal);

	/*
     * Getter for center of mass of the neighbors that are sending a
     * signal matching with the one passed as argument
	 */
	virtual CCI_EPuckRangeAndBearingSensor::SReceivedPacket
        GetSignalMatchingNeighborsCenterOfMass(UInt8 signal);

  protected:
    bool TestSignalMatch(CCI_EPuckRangeAndBearingSensor::TData const& data, UInt8 signal) const;
    bool WriteSignal(UInt8 signal, CCI_EPuckRangeAndBearingActuator::TData& data) const;

  private:
    /*
     * The proximity sensors input.
     */
    CCI_EPuckProximitySensor::TReadings m_sProximityInput;

    /*
     * The light sensors input.
     */
    CCI_EPuckLightSensor::TReadings m_sLightInput;

    /*
     * The ground sensors input.
     */
    std::deque<CCI_EPuckGroundSensor::SReadings> m_deqGroundInput;

    /*
     * The ground sensors input.
     */
    CCI_EPuckGroundSensor::SReadings m_sGroundInput;

    /*
     * The number of surrounding robots.
     */
    UInt8 m_unNumberNeighbors;

    /*
     * Pointer to the range-and-bearing messages buffer.
     */
    RabMessageBuffer m_pcRabMessageBuffer;

    /*
     * Signal to send
     */
    UInt8 m_signalToSend;

};

