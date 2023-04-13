#pragma once
#include "Arduino.h"

namespace MN::PWMLib::RefData
{
	/*
		PWMRefData.h

		defines classes that hold PWM reference data as it relates to the three TCC timers and separately for each of the supported pins how they relate to which TCC functions

	*/

	/*-------------------------- START OF TCC definition --------------------------*/

	class TCC
	{
		public:
			TCC ( Tcc *pTcc, RwReg *pCtrlAReg, RwReg *pWaveReg, RwReg *pPerReg, IRQn IRQN, uint32_t counterMax, uint16_t div, uint32_t steps, bool Configured );
			// const PWMTCCData &GetTCCData ();
			void	 SetPrescaler ( uint16_t prescaler );
			uint16_t GetPrescaler ();
			uint32_t GetCounterMax ();
			Tcc		*GetTcc ();
			bool	 IsConfigured ();
			bool	 IsRunning ();
			void	 Clear ();
			void	 Enable ();
			void	 SetConfigured ( bool state );
			void	 SetRunning ( bool state );
			void	 SetPWMType ( uint32_t type );
			void	 SetTop ( uint32_t top );
			void	 Stop ();
			void	 Restart ();
			void	 Start ();
			void	 SetCounter ( uint32_t newValue );
			void	 EnableInterrupts ();
			void	 EnableOverflow ();
			void	 EnableMatch ( uint8_t MCx );

		private:
			const Tcc	  *m_pTcc;
			const RwReg	  *m_pCtrlAReg;
			const RwReg	  *m_pWaveReg;
			const RwReg	  *m_pPerReg;
			const IRQn	   m_IRQn;
			const uint32_t m_counterMax;
			uint32_t	   m_steps;		 // Timer's PWM steps (resolution): range 2 to counterSize
			uint16_t	   m_prescaler;	 // prescaler value in use
			bool		   m_Configured; // Shows if TCCx is configured
			bool		   m_Running;	 // Shows if TCC is running

			void		   SyncWave ();
			void		   SyncPER ();
			void		   SyncCtrlBReg ();
			void		   ResetCount ();
	};

	constexpr uint8_t MAX_TCC = 3; // Max number of TCC units in SAMD21

	class TCCList
	{
		public:
			// TCCList ();
			static void ClearAll ();
			static TCC *GetTCCData ( uint8_t index );

		private:
			static TCC m_list [ MAX_TCC ];
	};

	/*-------------------------- END OF TCC definition --------------------------*/

	/*-------------------------- START OF PWM - Pin  definition --------------------------*/
	/* defines data used to set up PWM for Arduino MKR WIFI 1010 pins D0 to D9 inclusive */
	class PWMPinData
	{
		public:
			PWMPinData ( pin_size_t arduinoPin, int8_t port, uint32_t samd21Pin, uint8_t TCCIndex, RwReg *REG_TCCx_CCBy, uint8_t MCx, uint32_t Mux, uint8_t WOn );
			uint8_t GetTCCIndex ();
			uint8_t GetMC ();
			void	RouteClockToPin ();
			void	EnablePinStateWhenStopped ();
			void	SetSignalWhenStopped ( PinStatus defaultState );
			void	SetDuty ( uint32_t duty );

		private:
			const pin_size_t m_arduinoPin;	  // Arduino pin number
			const int8_t	 m_port;		  // Port of the SAMD21 pin
			const uint32_t	 m_samd21Pin;	  // SAMD21 pin
			const uint8_t	 m_TCCIndex;	  // TCC number used for this pin
			const RwReg		*m_REG_TCCx_CCBy; // Pointer to count register used for this pin
			const uint8_t	 m_MCx;			  // Match counter number
			const uint32_t	 m_Mux;			  // Pin multiplexer for this pin
			const uint8_t	 m_WOn;			  // Waveform output # for this pin
			uint32_t		 m_duty;		  // duty level used with this pin
	};

	/// @brief class that has list of data about supported PWM pins
	constexpr uint8_t MAX_PWM_PINS = 10; // Max number of pins supported in this library

	class PWMPinDataList
	{
		public:
			// PWMPinDataList ();
			static TCC		  *pin2TCCData ( pin_size_t ArduinoPin );
			static PWMPinData *pin2PinData ( pin_size_t ArduinoPin );

		private:
			static PWMPinData m_list [ MAX_PWM_PINS ];
	};

	/*-------------------------- END OF PWM - Pin  definition --------------------------*/
} // namespace MN::PWMLib::RefData