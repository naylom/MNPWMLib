#include "MNPWMLib.h"
#include "PWMCallback.h"
#include "PWMRefData.h"

namespace MN ::PWMLib
{
	/*

		Implementation of MNPWMLib.h

	*/
	PWMCallback		   pwmCallbacks;
	constexpr uint8_t  GCLOCK_ID	= GCLK_CLKCTRL_GEN_GCLK4_Val; // default to GCLK4
	constexpr uint32_t GCLOCK_SPEED = 48000000;					  // this MUST match the dource used below ie GCLK_GENCTRL_SRC_DFLL48M

	/// @brief Initialises PWM; uses GCLK4 for time source. Configures TCC0, TCC1, TCC2 and TC3 to use it. This is set
	/// @brief on the first object initialisation so once ClockDivisor is set it is the same for all subsequent objects
	/// @param ClockDivisor clock divisor to be used against 48Mhz signal
	MNPWM::MNPWM ( uint16_t ClockDivisor )
	{
		if ( bInitialised == false )
		{
			// SAMD has 9 clock generators , first 4 (ie 0-3) are used by arduino so will use one of 4 to 8
			// see https://www.pointsinfocus.com/learning/diy/arduino-mkr-wifi-1010-custom-pwm-configuration/
			// see https://cdn.sparkfun.com/datasheets/Dev/Arduino/Boards/Atmel-42181-SAM-D21_Datasheet.pdf

			// Step 1, select clock source and divisor
			// Set the divisor for GCLK4.
			REG_GCLK_GENDIV = GCLK_GENDIV_DIV ( ClockDivisor ) | // Set divisor
							  GCLK_GENDIV_ID ( GCLOCK_ID );		 // For GCLK4, divisor is a 8 bit value
			while ( GCLK->STATUS.bit.SYNCBUSY )
				;

			// Enable Generic clock gnerator: Set the clock source, duty cycle, and enable GCLK5
			REG_GCLK_GENCTRL = GCLK_GENCTRL_SRC_DFLL48M |	  // Set 48MHz source
							   GCLK_GENCTRL_IDC |			  // Improve Duty Cycle
							   GCLK_GENCTRL_GENEN |			  // Enable GCLK
							   GCLK_GENCTRL_ID ( GCLOCK_ID ); // For GLCK4
			while ( GCLK->STATUS.bit.SYNCBUSY )
				;

			// Configure Generic clock control : Route GLCK4 to TCC1 & TCC1, and enable the clock.
			REG_GCLK_CLKCTRL = GCLK_CLKCTRL_ID_TCC0_TCC1 | // Route GCLK4 to TCC1 & 1
							   GCLK_CLKCTRL_CLKEN |		   // Enable the clock
							   GCLK_CLKCTRL_GEN_GCLK4;	   // Select GCLK4
			while ( GCLK->STATUS.bit.SYNCBUSY )
				;
			// Configure Generic clock control : Route GLCK4 to TCC2 & TC3, and enable the clock.
			REG_GCLK_CLKCTRL = GCLK_CLKCTRL_ID_TCC2_TC3 | // Route GCLK4 to TCC1 & 1
							   GCLK_CLKCTRL_CLKEN |		  // Enable the clock
							   GCLK_CLKCTRL_GEN_GCLK4;	  // Select GCLK4
			while ( GCLK->STATUS.bit.SYNCBUSY )
				;
			m_clockDivisor = ClockDivisor;
			bInitialised   = true;
			m_bIsRunning   = false;
		}
	}

	/// @brief Called to start PWM on a given arduino pin
	/// @param arduinoPin arduinio pin number D0 - D9 supported
	/// @param prescaler required prescaler setting
	/// @param duty duty value, must be less than top, from 0 to this output is HIGH
	/// @param top value at which output is reset to LOW, min value is 2, max depends on TCC allocated to this pin 24 bit for TCC0 & 1, 16 bit for TCC2
	/// @param OverflowFn optional interrupt level routine to be called when counter matches top
	/// @param MatchFn optional interrupt level routine to be called when counter matches duty level
	/// @return true if successful
	bool MNPWM::SetPWM ( pin_size_t arduinoPin, uint16_t prescaler, uint32_t duty, uint32_t top, voidFuncPtr OverflowFn, voidFuncPtr MatchFn )
	{
		// get pin and related TCC data
		m_pPinData = RefData::PWMPinDataList::pin2PinData ( arduinoPin );
		if ( m_pPinData == nullptr )
		{
			// unsupported pin
			return false;
		}
		m_pTCCData = RefData::PWMPinDataList::pin2TCCData ( arduinoPin );
		if ( m_pTCCData == nullptr )
		{
			// unsupported pin
			m_pPinData = nullptr;
			return false;
		}

		digitalWrite ( m_pin, LOW ); // default to LOW
		m_pin		= arduinoPin;
		// prescalar is associated with TCC object and will be saved there when configured

		m_prescaler = prescaler;
		// ensure duty and top are sanitised before continuing
		m_top		= max ( top, (uint32_t)2 );					   // 2 is the smallest value
		m_top		= min ( m_top, m_pTCCData->GetCounterMax () ); // ensure top does not exceed max counter size of related TCC
		m_duty		= min ( duty, m_top );						   // duty must be smaller than top

		// Step 2, now route the generated clock signal to the TCC (Timer Counter for Control) module
		m_pPinData->RouteClockToPin ();

		m_pPinData->SetSignalWhenStopped ( LOW );

		m_pTCCData->SetPWMType ( TCC_WAVE_WAVEGEN_NPWM ); // Single-slope PWM (aka Fast PWM, aka Normal PWM)

		m_pTCCData->SetTop ( m_top );

		// Set PWM signal to go LOW when PWM stopped
		m_pPinData->SetSignalWhenStopped ( LOW );

		// set prescaler
		m_pTCCData->SetPrescaler ( prescaler );

		// set duty (counter match level)
		m_pPinData->SetDuty ( m_duty );

		// See if user has provided a function to be called when either an overflow or match occurs
		if ( OverflowFn != nullptr || MatchFn != nullptr )
		{
			m_pTCCData->EnableInterrupts ();
			pwmCallbacks.Set ( MatchFn, OverflowFn, m_pPinData->GetTCCIndex (), m_pPinData->GetMC () );
			if ( OverflowFn != nullptr )
			{
				m_pTCCData->EnableOverflow ();
			}
			if ( MatchFn != nullptr )
			{
				m_pTCCData->EnableMatch ( m_pPinData->GetMC () );
			}
		}

		m_pTCCData->Enable ();

		// set TCC state
		m_pTCCData->SetRunning ( true );
		m_pTCCData->SetConfigured ( true );
		return true;
	}

	bool MNPWM::SetPWM ( pin_size_t arduinoPin, uint16_t prescaler, uint32_t duty, uint32_t top, voidFuncPtrParam OverflowFn, voidFuncPtrParam MatchFn, void *OverflowParam, void *MatchParam )
	{
		// get pin and related TCC data
		m_pPinData = RefData::PWMPinDataList::pin2PinData ( arduinoPin );
		if ( m_pPinData == nullptr )
		{
			// unsupported pin
			return false;
		}
		m_pTCCData = RefData::PWMPinDataList::pin2TCCData ( arduinoPin );
		if ( m_pTCCData == nullptr )
		{
			// unsupported pin
			m_pPinData = nullptr;
			return false;
		}

		digitalWrite ( m_pin, LOW ); // default to LOW
		m_pin		= arduinoPin;
		// prescalar is associated with TCC object and will be saved there when configured

		m_prescaler = prescaler;
		// ensure duty and top are sanitised before continuing
		m_top		= max ( top, (uint32_t)2 );					   // 2 is the smallest value
		m_top		= min ( m_top, m_pTCCData->GetCounterMax () ); // ensure top does not exceed max counter size of related TCC
		m_duty		= min ( duty, m_top );						   // duty must be smaller than top

		// Step 2, now route the generated clock signal to the TCC (Timer Counter for Control) module
		m_pPinData->RouteClockToPin ();

		m_pPinData->SetSignalWhenStopped ( LOW );

		m_pTCCData->SetPWMType ( TCC_WAVE_WAVEGEN_NPWM ); // Single-slope PWM (aka Fast PWM, aka Normal PWM)

		m_pTCCData->SetTop ( m_top );

		// Set PWM signal to go LOW when PWM stopped
		m_pPinData->SetSignalWhenStopped ( LOW );

		// set prescaler
		m_pTCCData->SetPrescaler ( prescaler );

		// set duty (counter match level)
		m_pPinData->SetDuty ( m_duty );

		// See if user has provided a function to be called when either an overflow or match occurs
		if ( OverflowFn != nullptr || MatchFn != nullptr )
		{
			m_pTCCData->EnableInterrupts ();
			pwmCallbacks.Set ( MatchFn, OverflowFn, m_pPinData->GetTCCIndex (), m_pPinData->GetMC (), MatchParam, OverflowParam );
			if ( OverflowFn != nullptr )
			{
				m_pTCCData->EnableOverflow ();
			}
			if ( MatchFn != nullptr )
			{
				m_pTCCData->EnableMatch ( m_pPinData->GetMC () );
			}
		}
	}

	/// @brief Disables TCC
	void MNPWM::StopPWM ()
	{
		if ( m_pTCCData != nullptr )
		{
			m_pTCCData->Stop ();
		}
		m_bIsRunning = false;
	}

	/// @brief Restarts the TCC clock
	void MNPWM::RestartPWM ()
	{
		if ( m_pTCCData != nullptr )
		{
			m_pTCCData->Restart ();
		}
		m_bIsRunning = true;
	}

	/// @brief Enables TCC
	void MNPWM::StartPWM ()
	{
		if ( m_pTCCData != nullptr )
		{
			m_pTCCData->Start ();
		}
	}

#define MAX_SCALAR PrescalarValues [ sizeof ( PrescalarValues ) / sizeof ( PrescalarValues [ 0 ] ) - 1 ]

	/// @brief Attempts to find the best prescalar and top for required frequency
	/// @param arduinoPin pin to be used must be between 0 - 9 inclusive
	/// @param wantedFreq the frequency wanted
	/// @param prescalar will hold recommended  prescalar if true returned
	/// @param top will hold the top recommended if true returned
	/// @return false if no match identified
	bool MNPWM::BestFit ( pin_size_t arduinoPin, uint16_t wantedFreq, uint16_t &prescaler, uint32_t &top )
	{
		bool				  result			 = false;
		static const uint16_t PrescalarValues [] = { 1, 2, 4, 8, 16, 64, 256, 1024 }; // must be in ascending order

		// Get clock ticks after divisor applied
		uint32_t			  clock_rate		 = GCLOCK_SPEED / m_clockDivisor;
		// Get max top for the supplied pin
		RefData::TCC		 *pTCCData			 = RefData::PWMPinDataList::pin2TCCData ( arduinoPin );
		if ( pTCCData != nullptr )
		{
			// supported pin
			uint32_t maxtop = pTCCData->GetCounterMax ();
			if ( maxtop != 0 )
			{
				if ( clock_rate / MAX_SCALAR <= maxtop ) // check max prescalar
				{
					// can be calculated with clock divisor of 1
					uint32_t bestDiff = 0xFFFFFFFF;
					for ( uint8_t i = 0; i < sizeof ( PrescalarValues ) / sizeof ( PrescalarValues [ 0 ] ); i++ )
					{
						uint32_t candidate = clock_rate / wantedFreq / PrescalarValues [ i ];
						if ( candidate < maxtop )
						{
							// see how good this is
							uint32_t diff = abs ( int32_t ( clock_rate - wantedFreq * candidate * PrescalarValues [ i ] ) );
							if ( diff < bestDiff )
							{
								bestDiff  = diff;
								top		  = candidate;
								prescaler = PrescalarValues [ i ];
								result	  = true;
								if ( diff == 0 )
								{
									break;
								}
							}
						}
					}
				}
			}
		}
		return result;
	}

	bool MNPWM::bInitialised = false;
} // namespace MN::PWMLib

/// @brief Interrupt Service Routine (ISR) for timer TCC0
void TCC0_Handler ()
{
	if ( TCC0->INTENSET.bit.OVF && TCC0->INTFLAG.bit.OVF )
	{
		// Invoke all user overflow routines for this TCC which only has 2 match counters
		MN::PWMLib::pwmCallbacks.Overflow ( TCC0, REG_TCC0_CCB0 );
		MN::PWMLib::pwmCallbacks.Overflow ( TCC0, REG_TCC0_CCB1 );
		MN::PWMLib::pwmCallbacks.Overflow ( TCC0, REG_TCC0_CCB2 );
		MN::PWMLib::pwmCallbacks.Overflow ( TCC0, REG_TCC0_CCB3 );
		TCC0->INTFLAG.bit.OVF = 1;
	}
	else if ( TCC0->INTENSET.bit.MC0 && TCC0->INTFLAG.bit.MC0 )
	{
		// have we matched the DUTY value?
		MN::PWMLib::pwmCallbacks.Match ( TCC0, REG_TCC0_CCB0 );
		TCC0->INTFLAG.bit.MC0 = 1;
	}
	else if ( TCC0->INTENSET.bit.MC1 && TCC0->INTFLAG.bit.MC1 )
	{
		// have we matched the DUTY value?
		MN::PWMLib::pwmCallbacks.Match ( TCC0, REG_TCC0_CCB1 );
		TCC0->INTFLAG.bit.MC1 = 1;
	}
	else if ( TCC0->INTENSET.bit.MC2 && TCC0->INTFLAG.bit.MC2 )
	{
		// have we matched the DUTY value?
		MN::PWMLib::pwmCallbacks.Match ( TCC0, REG_TCC0_CCB2 );
		TCC0->INTFLAG.bit.MC2 = 1;
	}
	else if ( TCC0->INTENSET.bit.MC3 && TCC0->INTFLAG.bit.MC3 )
	{
		// have we matched the DUTY value?
		MN::PWMLib::pwmCallbacks.Match ( TCC0, REG_TCC0_CCB3 );
		TCC0->INTFLAG.bit.MC3 = 1;
	}
}

/// @brief Interrupt Service Routine (ISR) for timer TCC1
void TCC1_Handler () // Interrupt Service Routine (ISR) for timer TCC1
{
	if ( TCC1->INTENSET.bit.OVF && TCC1->INTFLAG.bit.OVF )
	{
		// Invoke all user overflow routines for this TCC which only has 2 match counters
		MN::PWMLib::pwmCallbacks.Overflow ( TCC1, REG_TCC1_CCB0 );
		MN::PWMLib::pwmCallbacks.Overflow ( TCC1, REG_TCC1_CCB1 );
		TCC1->INTFLAG.bit.OVF = 1;
	}
	else if ( TCC1->INTENSET.bit.MC0 && TCC1->INTFLAG.bit.MC0 )
	{
		// have we matched the DUTY value?
		MN::PWMLib::pwmCallbacks.Match ( TCC1, REG_TCC1_CCB0 );
		TCC1->INTFLAG.bit.MC0 = 1;
	}
	else if ( TCC1->INTENSET.bit.MC1 && TCC1->INTFLAG.bit.MC1 )
	{
		// have we matched the DUTY value?
		MN::PWMLib::pwmCallbacks.Match ( TCC1, REG_TCC1_CCB1 );
		TCC1->INTFLAG.bit.MC1 = 1;
	}
}

/// @brief Interrupt Service Routine (ISR) for timer TCC2
void TCC2_Handler () // Interrupt Service Routine (ISR) for timer TCC2
{
	if ( TCC2->INTENSET.bit.OVF && TCC2->INTFLAG.bit.OVF )
	{
		// Invoke all user overflow routines for this TCC which only has 2 match counters
		MN::PWMLib::pwmCallbacks.Overflow ( TCC2, REG_TCC2_CCB0 );
		MN::PWMLib::pwmCallbacks.Overflow ( TCC2, REG_TCC2_CCB1 );
		TCC2->INTFLAG.bit.OVF = 1;
	}
	else if ( TCC2->INTENSET.bit.MC0 && TCC2->INTFLAG.bit.MC0 )
	{
		// have we matched the DUTY value?
		MN::PWMLib::pwmCallbacks.Match ( TCC2, REG_TCC2_CCB0 );
		TCC2->INTFLAG.bit.MC0 = 1;
	}
	else if ( TCC2->INTENSET.bit.MC1 && TCC2->INTFLAG.bit.MC1 )
	{
		// have we matched the DUTY value?
		MN::PWMLib::pwmCallbacks.Match ( TCC2, REG_TCC2_CCB1 );
		TCC2->INTFLAG.bit.MC1 = 1;
	}
}
