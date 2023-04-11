#include "MNPWMLib.h"
#include "PWMCallback.h"

namespace MN ::PWMLib
{
	/*

		Implementation of MNPWMLib.h

	*/
	PWMCallback pwmCallbacks;

	// Table definition for looking up and storing values for TCCx
	typedef struct
	{
			const Tcc			   *TCCx;			// Pointer to timer
			const RwReg			   *REG_TCCx_CTRLA; // Pointer to timer's CTRLA register
			const RwReg			   *REG_TCCx_WAVE;	// Pointer to timer's WAVE register
			const RwReg			   *REG_TCCx_PER;	// Pointer to timer's PER register
			const IRQn				IRQNx;			// IRQ number for this timer
			const unsigned long int counterSize;	// Timer's counter size: 24 bits for TCC0 and TCC1, 16 bits for TCC2
			unsigned int			TCCDiv;			// Timer's clock divider: 1, 2, 4, 8, 16, 64, 256, or 1024
			unsigned long long int	steps;			// Timer's PWM steps (resolution): 2 to counterSize
			bool					enabled;		// Shows if TCCx should be enabled
	} PWMTCCData;

	// prepopulated table
	static PWMTCCData PWMTimerInfo [] = {
		{TCC0,	&REG_TCC0_CTRLA, &REG_TCC0_WAVE, &REG_TCC0_PER, TCC0_IRQn, 0xFFFFFF, 1, 500000, true},
		{ TCC1, &REG_TCC1_CTRLA, &REG_TCC1_WAVE, &REG_TCC1_PER, TCC1_IRQn, 0xFFFFFF, 1, 500000, true},
		{ TCC2, &REG_TCC2_CTRLA, &REG_TCC2_WAVE, &REG_TCC2_PER, TCC2_IRQn, 0xFFFF,   1, 50000,	 true}
	};
	static const uint8_t timerTableSize = sizeof ( PWMTimerInfo ) / sizeof ( PWMTimerInfo [ 0 ] );

	// prepopulated table of PWM data for arduino pins  *** MUST BE IN ARDUINO PIN NUMBER ORDER WITH NO GAPS ***
	static PWMPinData	 PWMPinInfo []	= {
		{0,	 g_APinDescription [ 0 ].ulPort, g_APinDescription [ 0 ].ulPin, 0, &REG_TCC0_CCB0, 0, PORT_PMUX_PMUXE_F, 4}, // Pin D0
		{ 1, g_APinDescription [ 1 ].ulPort, g_APinDescription [ 1 ].ulPin, 0, &REG_TCC0_CCB1, 1, PORT_PMUX_PMUXO_F, 5}, // Pin D1
		{ 2, g_APinDescription [ 2 ].ulPort, g_APinDescription [ 2 ].ulPin, 1, &REG_TCC1_CCB0, 0, PORT_PMUX_PMUXE_E, 2}, // Pin D2
		{ 3, g_APinDescription [ 3 ].ulPort, g_APinDescription [ 3 ].ulPin, 1, &REG_TCC1_CCB1, 1, PORT_PMUX_PMUXO_E, 3}, // Pin D3
		{ 4, g_APinDescription [ 4 ].ulPort, g_APinDescription [ 4 ].ulPin, 0, &REG_TCC0_CCB0, 0, PORT_PMUX_PMUXE_F, 4}, // Pin D4
		{ 5, g_APinDescription [ 5 ].ulPort, g_APinDescription [ 5 ].ulPin, 0, &REG_TCC0_CCB1, 1, PORT_PMUX_PMUXO_F, 5}, // Pin D5
		{ 6, g_APinDescription [ 6 ].ulPort, g_APinDescription [ 6 ].ulPin, 0, &REG_TCC0_CCB2, 2, PORT_PMUX_PMUXE_F, 6}, // Pin D6
		{ 7, g_APinDescription [ 7 ].ulPort, g_APinDescription [ 7 ].ulPin, 0, &REG_TCC0_CCB3, 3, PORT_PMUX_PMUXO_F, 7}, // Pin D7
		{ 8, g_APinDescription [ 8 ].ulPort, g_APinDescription [ 8 ].ulPin, 2, &REG_TCC2_CCB0, 0, PORT_PMUX_PMUXE_E, 6}, // Pin D8
		{ 9, g_APinDescription [ 9 ].ulPort, g_APinDescription [ 9 ].ulPin, 2, &REG_TCC2_CCB1, 1, PORT_PMUX_PMUXO_E, 7}  // Pin D9
	};
	constexpr uint8_t  PWMPinInfoTableSize = sizeof ( PWMPinInfo ) / sizeof ( PWMPinInfo [ 0 ] );

	constexpr uint8_t  GCLOCK_ID		   = GCLK_CLKCTRL_GEN_GCLK4_Val; // default to GCLK4
	constexpr uint32_t GCLOCK_SPEED		   = 48000000;					 // this MUST match the dource used below ie GCLK_GENCTRL_SRC_DFLL48M

	/// @brief Initialises PWM; uses GCLK4 for time source. Configures TCC0, TCC1, TCC2 and TC3 to use it. This is set
	/// @brief on the first object initialisation so once ClockDivisor is set it is the same for all subsequent objects
	/// @param ClockDivisor clock divisor to be used against 48Mhz signal
	MNPWM::MNPWM ( uint8_t ClockDivisor )
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
		// Check pin is valid
		PWMPinData *pPWMData = GetPinInfo ( arduinoPin );
		if ( pPWMData == nullptr )
		{
			return false;
		}
		m_pin = arduinoPin;
		digitalWrite ( m_pin, LOW ); // default to LOW
		m_prescaler															 = prescaler;
		// validation
		m_top																 = max ( top, (uint32_t)2 );										 // 2 is the smallest value
		m_top																 = min ( m_top, PWMTimerInfo [ pPWMData->timerIndex ].counterSize ); // ensure top does not exceed max counter size
		m_duty																 = min ( duty, m_top );												 // duty must be smaller than top

		// Step 2, now route the generated clock signal to the TCC (Timer Counter for Control) module

		PORT->Group [ pPWMData->port ].PINCFG [ pPWMData->samd21Pin ].reg	 |= PORT_PINCFG_PMUXEN;
		PORT->Group [ pPWMData->port ].PMUX [ pPWMData->samd21Pin >> 1 ].reg |= pPWMData->Mux;

		// Step 3, set up TCC to generate PWN waveform
		// 3 TCC units, TCC1, TCC1, TCC2
		// TCCO has 4 24bit channels (CCn) that map to 8 waveform outouts (WOn)
		// TCC1 has 2 24 bit channels (CCn) that map to 4 waveform outouts (WOn)
		// TCC2 has 2 16 bit channels (CCn) that map to 2 waveform outouts (WOn)

		SetPWMType ( TCC_WAVE_WAVEGEN_NPWM ); // Single-slope PWM (aka Fast PWM, aka Normal PWM)

		SetPWMTop ( m_top );

		// Set PWM signal to go LOW when PWM stopped
		SetPWMWhenStopped ( LOW );

		// set prescaler & enable
		*(RwReg *)PWMTimerInfo [ pPWMData->timerIndex ].REG_TCCx_CTRLA |= PreScalerLookup::value2Ref ( prescaler ) | TCC_CTRLA_ENABLE; // Requires SYNC on CTRLA

		SetDuty ( m_duty, pPWMData );
		m_bIsRunning = true;

		// See if user has provided a function to be called when either an overflow or match occurs
		if ( OverflowFn != nullptr || MatchFn != nullptr )
		{
			// Enable interrupts for this TCCn
			NVIC_DisableIRQ ( PWMTimerInfo [ pPWMData->timerIndex ].IRQNx );
			NVIC_ClearPendingIRQ ( PWMTimerInfo [ pPWMData->timerIndex ].IRQNx );
			NVIC_SetPriority ( PWMTimerInfo [ pPWMData->timerIndex ].IRQNx, 0 ); // Set the Nested Vector Interrupt Controller (NVIC) priority
			NVIC_EnableIRQ ( PWMTimerInfo [ pPWMData->timerIndex ].IRQNx );		 // Connect TCC to Nested Vector Interrupt Controller (NVIC)

			pwmCallbacks.Set ( MatchFn, OverflowFn, pPWMData->timerIndex, pPWMData->MCx );
			if ( OverflowFn != nullptr )
			{
				( (::Tcc *)pin2Tcc ( m_pin ) )->INTENSET.bit.OVF = 1; // Enable Interrupts when time counter reaches TOP
			}
			if ( MatchFn != nullptr )
			{
				( (::Tcc *)pin2Tcc ( m_pin ) )->INTENSET.vec.MC |= 1 << pPWMData->MCx; // Enable interrupts when timer counter matches duty value
			}
		}
		return true;
	}

	/// @brief Controls the state of the pin when PWM is stopped
	/// @param defaultState set to HIGH or LOW as required
	void MNPWM::SetPWMWhenStopped ( PinStatus defaultState )
	{
		PWMPinData *pInfo = GetPinInfo ( m_pin );
		if ( pInfo != nullptr )
		{
			if ( defaultState == HIGH )
			{
				( (::Tcc *)pin2Tcc ( m_pin ) )->DRVCTRL.vec.NRV |= ( 1 << pInfo->WOn );
			}
			else
			{
				( (::Tcc *)pin2Tcc ( m_pin ) )->DRVCTRL.vec.NRV &= ~( 1 << pInfo->WOn );
			}
			EnablePinStateWhenStopped ();
		}
	}

	/// @brief Allows the state of the pin (ie HIGH or LOW) to be defined when PWM clock is stopped
	/// @param pin
	inline void MNPWM::EnablePinStateWhenStopped ()
	{
		PWMPinData *pInfo = GetPinInfo ( m_pin );
		if ( pInfo != nullptr )
		{
			( (::Tcc *)pin2Tcc ( m_pin ) )->DRVCTRL.vec.NRE |= ( 1 << pInfo->WOn ); // enable
		}
	}

	/// @brief Allow the PWM outpin to be undefined when PWM is stopped
	/// @param pin
	inline void MNPWM::DisablePinStateWhenStopped ()
	{
		PWMPinData *pInfo = GetPinInfo ( m_pin );
		if ( pInfo != nullptr )
		{
			( (::Tcc *)pin2Tcc ( m_pin ) )->DRVCTRL.vec.NRE &= ~( 1 << pInfo->WOn ); // enable
		}
	}

	/// @brief Waits for CTRL B register to sync before continuing
	inline void MNPWM::SyncCtrlBReg ()
	{
		while ( ( (::Tcc *)pin2Tcc ( m_pin ) )->SYNCBUSY.bit.CTRLB )
			; // Wait for synchronization
	}

	inline void MNPWM::SetPWMType ( uint32_t PWMType )
	{
		*(RwReg *)PWMTimerInfo [ pin2TCCIndex ( m_pin ) ].REG_TCCx_WAVE |= PWMType;
		while ( ( (::Tcc *)pin2Tcc ( m_pin ) )->SYNCBUSY.bit.WAVE )
			; // Wait for synchronization
	}

	/// @brief Sets the max value for the TCC counter at which point it is reset and generates an overflow interrupt if set
	/// @param PWMTop value for top
	inline void MNPWM::SetPWMTop ( uint32_t PWMTop )
	{
		*(RwReg *)PWMTimerInfo [ pin2TCCIndex ( m_pin ) ].REG_TCCx_PER = PWMTop;
		while ( ( (::Tcc *)pin2Tcc ( m_pin ) )->SYNCBUSY.bit.PER )
			; // Wait for synchronization
	}

	/// @brief Checks if PWM is running
	/// @return true if running, false otherwise
	bool MNPWM::IsRunning ()
	{
		return m_bIsRunning;
	}

	/// @brief Disables TCC
	void MNPWM::StopPWM ()
	{
		// Stops timer so all pins using it will be impacted! Better to set duty = 2 or top?
		( (::Tcc *)pin2Tcc ( m_pin ) )->CTRLBSET.reg = TCC_CTRLBCLR_CMD_STOP; // Stop the timer
		SyncCtrlBReg ();
		m_bIsRunning = false;
	}

	/// @brief Restarts the TCC clock
	void MNPWM::RestartPWM ()
	{
		SetCount ( 0 );															   // reset to 0
		( (::Tcc *)pin2Tcc ( m_pin ) )->CTRLBSET.reg = TCC_CTRLBCLR_CMD_RETRIGGER; // Restart a stopped timer or reset a stopped one
		SyncCtrlBReg ();
		m_bIsRunning = true;
	}

	/// @brief Enables TCC
	void MNPWM::StartPWM ()
	{
		*(RwReg *)PWMTimerInfo [ pin2TCCIndex ( m_pin ) ].REG_TCCx_CTRLA |= TCC_CTRLA_ENABLE;
		while ( pin2Tcc ( m_pin )->SYNCBUSY.bit.ENABLE )
			;
	}

	void MNPWM::SetCount ( uint32_t count )
	{
		// reset the count to 0
		::Tcc *pinTCC	  = (::Tcc *)pin2Tcc ( m_pin );
		pinTCC->COUNT.reg = count & TCC_COUNT_MASK; // max 24 bits
		// TCC0->COUNT.reg = count & TCC_COUNT_MASK; // max 24 bits
		while ( pinTCC->SYNCBUSY.bit.COUNT ) // Wait for synchronization
			;
	}

	/// @brief Gives the port number assocaited with the pin
	/// @param arduinoPin pin number of interest must be 0 - 9 inclusive
	/// @return
	inline int8_t MNPWM::pin2Port ( pin_size_t arduinoPin )
	{
		return PWMPinInfo [ PinData ( arduinoPin ) ].port;
	}

	inline uint32_t MNPWM::pin2SAMD21 ( pin_size_t arduinoPin )
	{
		return PWMPinInfo [ PinData ( arduinoPin ) ].samd21Pin;
	}

	/// @brief Find the TCC allocated to the pin, note there is one TOP value per TCC
	/// @param arduinoPin to check, must be 0 - 9
	/// @return index of TCC ie 0 - 2 inclusive
	inline uint8_t MNPWM::pin2TCCIndex ( pin_size_t arduinoPin )
	{
		return PWMPinInfo [ arduinoPin ].timerIndex;
	}

	/// @brief Gives the TCC register for the pin specified
	/// @param arduinoPin must be 0 - 9 inclusive
	/// @return pointer to TCC register
	inline const ::Tcc *MNPWM::pin2Tcc ( pin_size_t arduinoPin )
	{
		return PWMTimerInfo [ pin2TCCIndex ( arduinoPin ) ].TCCx;
	}

	/// @brief Find the CC allocated to the pin
	/// @param arduinoPin to check, must be 0 - 9
	/// @return index of CCx ie 0 - 3 inclusive
	inline uint8_t MNPWM::pin2CCx ( pin_size_t arduinoPin )
	{
		return PWMPinInfo [ PinData ( arduinoPin ) ].MCx;
	}

	/// @brief Gets the maximum TOP value for a given arduino pin
	/// @param arduinoPin
	/// @return maximum value for TOP, 0 if pin invalid
	inline uint32_t MNPWM::pin2MaxTop ( pin_size_t arduinoPin )
	{
		uint32_t	result	= 0UL;

		// public function so validate parameter
		PWMPinData *pinInfo = GetPinInfo ( arduinoPin );
		if ( pinInfo != nullptr )
		{
			result = PWMTimerInfo [ pinInfo->timerIndex ].counterSize;
		}
		return result;
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
		uint32_t			  maxtop			 = pin2MaxTop ( arduinoPin );
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
		return result;
	}

	inline uint32_t MNPWM::pin2PortMUX ( pin_size_t arduinoPin )
	{
		return PWMPinInfo [ arduinoPin ].Mux;
	}

	/// @brief returns the index for the supplied arduino pin number
	/// @param arduinoPin
	/// @return index for the relevenant data, if pin not found returns 0!
	uint8_t MNPWM::PinData ( uint8_t arduinoPin )
	{
		for ( uint8_t i = 0; i < PWMPinInfoTableSize; i++ )
		{
			if ( PWMPinInfo [ i ].arduinoPin == arduinoPin )
			{
				return i;
			}
		}
		return 0;
	}

	PWMPinData *MNPWM::GetPinInfo ( pin_size_t arduinoPin )
	{
		for ( uint8_t i = 0; i < PWMPinInfoTableSize; i++ )
		{
			if ( PWMPinInfo [ i ].arduinoPin == arduinoPin )
			{
				return &PWMPinInfo [ i ];
			}
		}
		return nullptr;
	}

	void MNPWM::SetDuty ( uint32_t duty, PWMPinData *pData )
	{
		*(RwReg *)pData->REG_TCCx_CCBy = duty;
		while ( pin2Tcc ( m_pin )->SYNCBUSY.vec.CCB )
			;
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
