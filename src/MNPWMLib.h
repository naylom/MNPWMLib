#pragma once
#include <Arduino.h>
#include "PWMRefData.h"
#include "MNPrescaler.h"

/*
	MNPWMLib.h

	defines pwm class for mkr wifi 1010

	Supports pins D0 to D9
	Supports Normal PWM (single slope)

	Uses 48Mhz clock (GCLK4)

	When setting up PWM user can optionally provide two functions to be called when duty level is matched and / or counter reaches top
	such routines are called at the interrupt level and should obey normal constraints. Given PWM can run at very high frequency, such
	routines should be minimal
*/
namespace MN ::PWMLib
{

#ifndef pin_size_t
	typedef uint8_t pin_size_t;
#endif
#ifndef NOT_A_PIN
	#define NOT_A_PIN 255
#endif
	/* defines data used to set up PWM for arduino mkr wifi 1010 pins D0 to D9 inclusive */
	class MNPWM
	{
		public:
			MNPWM ( uint16_t ClockDivisor = 1 );
			bool SetPWM ( pin_size_t arduinoPin, uint16_t prescaler, uint32_t duty, uint32_t top, voidFuncPtr OverflowFn = nullptr, voidFuncPtr MatchFn = nullptr );
			bool SetPWM ( pin_size_t arduinoPin, uint16_t prescaler, uint32_t duty, uint32_t top, voidFuncPtrParam OverflowFn = nullptr, voidFuncPtrParam MatchFn = nullptr, void * OverflowParam = nullptr, void * MatchParam = nullptr );
			void StopPWM ();
			void RestartPWM ();
			void StartPWM ();
			bool BestFit ( pin_size_t arduinoPin, uint16_t wantedFreq, uint16_t &prescaler, uint32_t &top );

		private:
			static bool			 bInitialised; // Used to indicate initialisation of TCC done
			pin_size_t			 m_pin = NOT_A_PIN;
			uint32_t			 m_top;
			uint32_t			 m_duty;
			uint16_t			 m_frequency;
			uint16_t			 m_prescaler;
			uint16_t			 m_clockDivisor; // maybe a static data item set once
			bool				 m_bIsRunning;
			RefData::PWMPinData *m_pPinData = nullptr;
			RefData::TCC		*m_pTCCData = nullptr;
	};
} // namespace MN::PWMLib