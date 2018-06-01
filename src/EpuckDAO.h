/*
 * @file <src/core/EpuckDAO.h>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 *
 * @brief This class represents the status of the robot.
 * 				It contains the input variables (the sensor inputs) and
 * 				the output variables (the values for the wheel actuators),
 * 				as well as the setters and getters to access them.
 *
 * 				Only one object of this class should be instanciated, and
 * 				is to be used as a brigde between the AutoMoDeController and
 * 				the AutoMoDeFiniteStateMachine classes. In AutoMoDeController,
 * 				the variables of the object shall be updated at each time step.
 * 				The different modules of the  AutoMoDeFiniteStateMachine will
 * 				then use the input variables and update the output variables
 * 				accordingly.
 */


#ifndef EPUCK_DAO_H
#define EPUCK_DAO_H

#include <vector>
#include <deque>

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_wheels_actuator.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_rgb_leds_actuator.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_light_sensor.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_ground_sensor.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_omnidirectional_camera_sensor.h>

namespace argos {
	class EpuckDAO {
		public:

			virtual ~EpuckDAO();

			/*
			 * Reset function.
			 */
			virtual void Reset() = 0;

			/*
			 * Setter for the wheels velocity.
			 */
			void SetWheelsVelocity(const Real& un_left_velocity, const Real& un_right_velocity);

			/*
			 * Setter for the wheels velocity.
			 */
			void SetWheelsVelocity(const CVector2& c_velocity_vector);

			/*
			 * Getter for the right wheel velocity.
			 */
			const Real& GetRightWheelVelocity() const;

			/*
			 * Getter for the left wheel velocity.
			 */
			const Real& GetLeftWheelVelocity() const;

			/*
			 * Setter for the robot idientifier.
			 */
			void SetRobotIdentifier(const UInt32& un_robot_id);

			/*
			 * Getter for the robot identifier.
			 */
			const UInt32& GetRobotIdentifier() const;

			/*
			 * Getter for the maximal wheels velocity.
			 */
			const Real& GetMaxVelocity() const;

			/*
			 * Getter for the random number generetor.
			 */
			CRandom::CRNG* GetRandomNumberGenerator() const;


	/*******************/
	/* Virtual classes */
	/*******************/

			/*
			 * Getter for the proximity input.
			 */
			virtual CCI_EPuckProximitySensor::TReadings GetProximityInput() const {
				CCI_EPuckProximitySensor::TReadings emptyReadings;
				return emptyReadings;
			};

			virtual CCI_EPuckProximitySensor::SReading GetProximityReading() {
				return CCI_EPuckProximitySensor::SReading();
			};

			/*
			 * Setter for the proximity input.
			 */
			virtual void SetProximityInput(CCI_EPuckProximitySensor::TReadings s_prox_input) {};

			/*
			 * Getter for the light input.
			 */
			virtual CCI_EPuckLightSensor::TReadings GetLightInput() const {
				CCI_EPuckLightSensor::TReadings emptyReadings;
				return emptyReadings;
			};

			virtual CCI_EPuckLightSensor::SReading GetLightReading() {
				return CCI_EPuckLightSensor::SReading();
			};

			/*
			 * Setter for the light input.
			 */
			virtual void SetLightInput(CCI_EPuckLightSensor::TReadings s_light_input) {};

			/*
			 * Getter for the ground input.
			 */
			virtual CCI_EPuckGroundSensor::SReadings GetGroundInput() {
				CCI_EPuckGroundSensor::SReadings emptyReadings;
				return emptyReadings;
			};

			/*
			 * Setter for the ground input.
			 */
			virtual void SetGroundInput(CCI_EPuckGroundSensor::SReadings s_ground_input) {};

			/*
			 * Getter for the number of surrounding robots.
			 */
			virtual const UInt8 GetNumberNeighbors() const {
				UInt8 unEmptyVariable = 0;
				return unEmptyVariable;
			};

			/*
			 * Getter for the range-and-bearing messages.
			 */
			virtual std::vector<CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> GetRangeAndBearingMessages() {
				std::vector<CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> emptyReadings;
				return emptyReadings;
			};

			/*
			 * Getter for the center of mass of neighbors computed with RaB messages
			 */
			virtual CVector2 GetNeighborsCenterOfMass() {
				CVector2 cEmptyVector;
				return cEmptyVector;
			};

			/*
			 * Setter for the number of surrounding robots.
			 */
			virtual void SetNumberNeighbors(const UInt8& un_number_neighbors) {};

			/*
			 * Setter for the range-and-bearing input.
			 */
			virtual void SetRangeAndBearingMessages(CCI_EPuckRangeAndBearingSensor::TPackets s_packets) {};

		protected:
			/*
			 * The left wheel velocity (output variable).
			 */
			Real m_fLeftWheelVelocity;

			/*
			 * The right wheel velocity (output variable).
			 */
			Real m_fRightWheelVelocity;

			/*
			 * The maximal wheels velocity.
			 */
			Real m_fMaxVelocity;

			/*
			 * The robot identifier.
			 */
			UInt32 m_unRobotIdentifier;

			/*
			 * Pointer to the random number generator.
			 */
			CRandom::CRNG* m_pcRng;

	};
}


#endif
