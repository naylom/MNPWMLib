#include "MNPWMLib.h"

/*

	Implementation of MNPWMLib.h

*/
// Table definition for holding which match and overflow functions are associated with which TCCx and its related match buffer CCBy
typedef struct
{
		voidFuncPtr OverflowFn;
		voidFuncPtr MatchFn;
} PWMPinCallbacks;

constexpr uint8_t MAX_TCC = 3;
constexpr uint8_t MAX_CCB = 4;

class PWMCallback
{
	public:
		PWMCallback ();
		static bool Set ( voidFuncPtr matchFn, voidFuncPtr overflowFn, Tcc *pTCC, RwReg CCB );
		static void Overflow ( Tcc *pTCC, RwReg CCB );
		static void Match ( Tcc *pTCC, RwReg CCB );

	private:
		static PWMPinCallbacks m_PWMcallbackList [ MAX_TCC ][ MAX_CCB ];
		static uint8_t		   ConvertTCC ( Tcc *pTCC );
		static uint8_t		   ConvertCCB ( RwReg CCB );
};

// create instance of PWMCallbacks
PWMCallback		pwmCallbacks;
PWMPinCallbacks PWMCallback::m_PWMcallbackList [ MAX_TCC ][ MAX_CCB ];

PWMCallback::PWMCallback ()
{
	for ( uint8_t i = 0; i < MAX_TCC; i++ )
	{
		for ( uint8_t j = 0; j < MAX_CCB; j++ )
		{
			m_PWMcallbackList [ i ][ j ].MatchFn	= nullptr;
			m_PWMcallbackList [ i ][ j ].OverflowFn = nullptr;
		}
	}
}

bool PWMCallback::Set ( voidFuncPtr matchFn, voidFuncPtr overflowFn, Tcc *pTCC, RwReg CCB )
{
	bool	result	 = false;
	uint8_t indexTCC = ConvertTCC ( pTCC );
	uint8_t indexCCB = ConvertCCB ( CCB );
	if ( m_PWMcallbackList [ indexTCC ][ indexCCB ].MatchFn == nullptr && m_PWMcallbackList [ indexTCC ][ indexCCB ].OverflowFn == nullptr )
	{
		m_PWMcallbackList [ indexTCC ][ indexCCB ].MatchFn	  = matchFn;
		m_PWMcallbackList [ indexTCC ][ indexCCB ].OverflowFn = overflowFn;
		result												  = true;
	}
	return result;
}

void PWMCallback::Overflow ( Tcc *pTCC, RwReg CCB )
{
	uint8_t indexTCC = ConvertTCC ( pTCC );
	uint8_t indexCCB = ConvertCCB ( CCB );
	if ( m_PWMcallbackList [ indexTCC ][ indexCCB ].OverflowFn != nullptr )
	{
		m_PWMcallbackList [ indexTCC ][ indexCCB ].OverflowFn ();
	}
}

void PWMCallback::Match ( Tcc *pTCC, RwReg CCB )
{
	uint8_t indexTCC = ConvertTCC ( pTCC );
	uint8_t indexCCB = ConvertCCB ( CCB );
	if ( m_PWMcallbackList [ indexTCC ][ indexCCB ].MatchFn != nullptr )
	{
		m_PWMcallbackList [ indexTCC ][ indexCCB ].MatchFn ();
	}
}

uint8_t PWMCallback::ConvertTCC ( Tcc *pTCC )
{
	if ( pTCC == TCC0 )
	{
		return 0;
	}
	else if ( pTCC == TCC1 )
	{
		return 1;
	}
	else if ( pTCC == TCC2 )
	{
		return 2;
	}
	else
	{
		return 0;
	}
}

uint8_t PWMCallback::ConvertCCB ( RwReg CCB )
{
	if ( CCB == REG_TCC0_CCB0 || CCB == REG_TCC1_CCB0 || CCB == REG_TCC2_CCB0 )
	{
		return 0;
	}
	else if ( CCB == REG_TCC0_CCB1 || CCB == REG_TCC1_CCB1 || CCB == REG_TCC2_CCB1 )
	{
		return 1;
	}
	else if ( CCB == REG_TCC0_CCB2 )
	{
		return 2;
	}
	else if ( CCB == REG_TCC0_CCB3 )
	{
		return 3;
	}
	else
	{
		return 0;
	}
}

typedef struct PreScalars
{
		uint16_t value;
		uint16_t Ref;
};

PreScalars PreScalarsList [] = {
	{1,	 TCC_CTRLA_PRESCALER_DIV1	  }, //
	{ 2,	 TCC_CTRLA_PRESCALER_DIV2	  }, //
	{ 4,	 TCC_CTRLA_PRESCALER_DIV4	  }, //
	{ 8,	 TCC_CTRLA_PRESCALER_DIV8	  }, //
	{ 64,	  TCC_CTRLA_PRESCALER_DIV64	}, //
	{ 256,  TCC_CTRLA_PRESCALER_DIV256 }, //
	{ 1024, TCC_CTRLA_PRESCALER_DIV1024}  //
};

// Table definition for looking up and storing values for TCCx
typedef struct
{
		const Tcc			   *TCCx;			// Pointer to timer
		const RwReg			   *REG_TCCx_CTRLA; // Pointer to timer's CTRLA register
		const RwReg			   *REG_TCCx_WAVE;	// Pointer to timer's WAVE register
		const RwReg			   *REG_TCCx_PERB;	// Pointer to timer's PERB register
		const IRQn				IRQNx;			// IRQ for this timer
		const unsigned long int counterSize;	// Timer's counter size: 24 bits for TCC0 and TCC1, 16 bits for TCC2
		unsigned int			TCCDiv;			// Timer's clock divider: 1, 2, 4, 8, 16, 64, 256, or 1024
		unsigned long long int	steps;			// Timer's PWM steps (resolution): 2 to counterSize
		bool					enabled;		// Shows if TCCx should be enabled
		voidFuncPtr				OverflowFn;
		voidFuncPtr				MatchFn;
} PWMTimerData;

// prepopulated table
static PWMTimerData PWMTimerInfo [] = {
	{TCC0,	&REG_TCC0_CTRLA, &REG_TCC0_WAVE, &REG_TCC0_PERB, TCC0_IRQn, 0xFFFFFF, 1, 500000, true, nullptr, nullptr},
	{ TCC1, &REG_TCC1_CTRLA, &REG_TCC1_WAVE, &REG_TCC1_PERB, TCC1_IRQn, 0xFFFFFF, 1, 500000, true, nullptr, nullptr},
	{ TCC2, &REG_TCC2_CTRLA, &REG_TCC2_WAVE, &REG_TCC2_PERB, TCC2_IRQn, 0xFFFF,	1, 50000,  true, nullptr, nullptr}
};
static const uint8_t timerTableSize = sizeof ( PWMTimerInfo ) / sizeof ( PWMTimerInfo [ 0 ] );

// prepopulated table of PWM data for arduino pins
static PWMPinData	 PWMPinInfo []	= {
	//{ 2, PORTA, 10, 1, &REG_TCC1_CCB0, PORT_PMUX_PMUXE_E},
	//{ 3, PORTA, 11, 1, &REG_TCC1_CCB1, PORT_PMUX_PMUXO_E},
	//{ 4, PORTB, 10, 0, &REG_TCC0_CCB0, PORT_PMUX_PMUXE_F},
	//{ 5, PORTB, 11, 0, &REG_TCC0_CCB1, PORT_PMUX_PMUXO_F},
	//{ 6, PORTA, 20, 0, &REG_TCC0_CCB2, PORT_PMUX_PMUXE_F},
	//{ 7, PORTA, 21, 0, &REG_TCC0_CCB3, PORT_PMUX_PMUXO_F},
	//{ 8, PORTA, 16, 2, &REG_TCC2_CCB0, PORT_PMUX_PMUXE_E},
	//{ 9, PORTA, 17, 2, &REG_TCC2_CCB1, PORT_PMUX_PMUXO_E}
	{0,	 g_APinDescription [ 0 ].ulPort, g_APinDescription [ 0 ].ulPin, 0, &REG_TCC0_CCB0, 0, PORT_PMUX_PMUXE_F}, // Pin D0
	{ 1, g_APinDescription [ 1 ].ulPort, g_APinDescription [ 1 ].ulPin, 0, &REG_TCC0_CCB1, 1, PORT_PMUX_PMUXO_F}, // Pin D1
	{ 2, g_APinDescription [ 2 ].ulPort, g_APinDescription [ 2 ].ulPin, 1, &REG_TCC1_CCB0, 0, PORT_PMUX_PMUXE_E}, // Pin D2
	{ 3, g_APinDescription [ 3 ].ulPort, g_APinDescription [ 3 ].ulPin, 1, &REG_TCC1_CCB1, 1, PORT_PMUX_PMUXO_E}, // Pin D3
	{ 4, g_APinDescription [ 4 ].ulPort, g_APinDescription [ 4 ].ulPin, 0, &REG_TCC0_CCB0, 0, PORT_PMUX_PMUXE_F}, // Pin D4
	{ 5, g_APinDescription [ 5 ].ulPort, g_APinDescription [ 5 ].ulPin, 0, &REG_TCC0_CCB1, 1, PORT_PMUX_PMUXO_F}, // Pin D5
	{ 6, g_APinDescription [ 6 ].ulPort, g_APinDescription [ 6 ].ulPin, 0, &REG_TCC0_CCB2, 2, PORT_PMUX_PMUXE_F}, // Pin D6
	{ 7, g_APinDescription [ 7 ].ulPort, g_APinDescription [ 7 ].ulPin, 0, &REG_TCC0_CCB3, 3, PORT_PMUX_PMUXO_F}, // Pin D7
	{ 8, g_APinDescription [ 8 ].ulPort, g_APinDescription [ 8 ].ulPin, 2, &REG_TCC2_CCB0, 0, PORT_PMUX_PMUXE_E}, // Pin D8
	{ 9, g_APinDescription [ 9 ].ulPort, g_APinDescription [ 9 ].ulPin, 2, &REG_TCC2_CCB1, 1, PORT_PMUX_PMUXO_E}  // Pin D9
};
constexpr uint8_t PWMPinInfoTableSize = sizeof ( PWMPinInfo ) / sizeof ( PWMPinInfo [ 0 ] );

constexpr uint8_t GCLOCK_ID			  = GCLK_CLKCTRL_GEN_GCLK4_Val; // default to GCLK4

/// @brief Initialises PWM; uses GCLK4 for time source. Configures TCC0, TCC1, TCC2 and TC3 to use it. This is set
/// @brief on the first object initialisation so once ClockDivisor is set it is the same for all subsequent objects
/// @param ClockDivisor clock divisor to be used against 48Mhz signal
MNPWMLib::MNPWMLib ( uint8_t ClockDivisor )
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
	}
}

/// @brief Called to start PWM on a given arduino pin
/// @param arduinoPin arduinio pin number D2 - D9 supported
/// @param frequency required frequency
/// @param prescaler required prescaler setting
/// @param duty duty value, must be less than top, from 0 to this output is HIGH
/// @param top value at which output is reset to LOW, min value is 2, max depends on TCC allocated to this pin 24 bit for TCC0 & 1, 16 bit for TCC2
/// @param OverflowFn optional interrupt level routine to be called when counter matches top
/// @param MatchFn optional interrupt level routine to be called when counter matches duty level
/// @return true if successful
bool MNPWMLib::SetPWM ( pin_size_t arduinoPin, uint16_t frequency, uint16_t prescaler, uint32_t duty, uint32_t top, voidFuncPtr OverflowFn, voidFuncPtr MatchFn )
{
	// Check pin is valid
	PWMPinData *pPWMData = GetPinInfo ( arduinoPin );
	if ( pPWMData == nullptr )
	{
		return false;
	}
	m_pin																 = arduinoPin;
	m_prescaler															 = prescaler;
	// validation
	m_top																 = max ( top, 2 );												   // 2 is the smallest value
	m_top																 = min ( top, PWMTimerInfo [ pPWMData->timerIndex ].counterSize ); // ensure top does not exceed max counter size
	m_duty																 = max ( duty, m_top );											   // duty must be smaller than top

	// Step 2, now route the generated clock signal to the TCC (Timer Counter for Control) module

	// Mkr Wifi 1010 pinout sheet shows PA23 = digital pin 1

	PORT->Group [ pPWMData->port ].PINCFG [ pPWMData->samd21Pin ].reg	 |= PORT_PINCFG_PMUXEN;
	PORT->Group [ pPWMData->port ].PMUX [ pPWMData->samd21Pin >> 1 ].reg |= pPWMData->Mux;

	// Step 3, set up TCC to generate PWN waveform
	// 3 TCC units, TCC1, TCC1, TCC2
	// TCCO has 4 24bit channels (CCn) that map to 8 waveform outouts (WOn)
	// TCC1 has 2 24 bit channels (CCn) that map to 4 waveform outouts (WOn)
	// TCC2 has 2 16 bit channels (CCn) that map to 2 waveform outouts (WOn)

	// REG_TCC1_WAVE														 |= TCC_WAVE_WAVEGEN_NPWM;						 // Single-slope PWM
	*(RwReg *)PWMTimerInfo [ pPWMData->timerIndex ].REG_TCCx_WAVE		 |= TCC_WAVE_WAVEGEN_NPWM; // Single-slope PWM
	while ( PWMTimerInfo [ pPWMData->timerIndex ].TCCx->SYNCBUSY.bit.WAVE )
		;

	// REG_TCC1_PER												   = top;										   // Set TOP
	*(RwReg *)PWMTimerInfo [ pPWMData->timerIndex ].REG_TCCx_PERB |= m_top; // Set TOP
	while ( PWMTimerInfo [ pPWMData->timerIndex ].TCCx->SYNCBUSY.bit.PERB )
		;

	// REG_TCC1_CTRLA												   |= TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_ENABLE; // Requires SYNC on CTRLA
	// convert prescaler value to define
	uint8_t i;
	for ( i = 0; i < sizeof ( PreScalarsList ) / sizeof ( PreScalarsList [ 0 ] ); i++ )
	{
		if ( PreScalarsList [ i ].value == prescaler )
		{
			break;
		}
	}
	if ( i >= sizeof ( PreScalarsList ) / sizeof ( PreScalarsList [ 0 ] ) )
	{
		i = 0;
	}
	*(RwReg *)PWMTimerInfo [ pPWMData->timerIndex ].REG_TCCx_CTRLA |= PreScalarsList [ i ].Ref | TCC_CTRLA_ENABLE; // Requires SYNC on CTRLA

	// while ( TCC1->SYNCBUSY.bit.ENABLE || TCC1->SYNCBUSY.bit.WAVE || TCC1->SYNCBUSY.bit.PER )
	//	;

	// Step 4, Set PWM duty cycle

	*(RwReg *)pPWMData->REG_TCCx_CCBy							   = m_duty;
	while ( PWMTimerInfo [ pPWMData->timerIndex ].TCCx->SYNCBUSY.vec.CCB )
		;
	// while ( PWMTimerInfo [ pPWMData->timerIndex ].TCCx->SYNCBUSY.vec.CCB )
	//	;
	//  REG_TCC1_CCB0 = duty; // Set DUTY
	//  while ( TCC1->SYNCBUSY.bit.CCB0 )
	//;

	// See if user has provided a function to be called when either an overflow or match occurs
	if ( OverflowFn != nullptr || MatchFn != nullptr )
	{
		// Enable interrupts for this TCCn
		NVIC_DisableIRQ ( PWMTimerInfo [ pPWMData->timerIndex ].IRQNx );
		NVIC_ClearPendingIRQ ( PWMTimerInfo [ pPWMData->timerIndex ].IRQNx );
		NVIC_SetPriority ( PWMTimerInfo [ pPWMData->timerIndex ].IRQNx, 0 ); // Set the Nested Vector Interrupt Controller (NVIC) priority
		NVIC_EnableIRQ ( PWMTimerInfo [ pPWMData->timerIndex ].IRQNx );		 // Connect TCC to Nested Vector Interrupt Controller (NVIC)
		// NVIC_DisableIRQ ( TCC1_IRQn );
		// NVIC_ClearPendingIRQ ( TCC1_IRQn );
		// NVIC_SetPriority ( TCC1_IRQn, 0 ); // Set the Nested Vector Interrupt Controller (NVIC) priority
		// NVIC_EnableIRQ ( TCC1_IRQn );	   // Connect TC0 to Nested Vector Interrupt Controller (NVIC)

		if ( OverflowFn != nullptr )
		{
			( (Tcc *)( PWMTimerInfo [ pPWMData->timerIndex ].TCCx ) )->INTENSET.bit.OVF = 1; // Enable Interrupts when time counter reaches TOP
																							 // TCC1->INTENSET.bit.OVF = 1;
		}
		if ( MatchFn != nullptr )
		{
			// needs to change MC0/MC1 based CCB in table
			//( (Tcc *)( PWMTimerInfo [ pPWMData->timerIndex ].TCCx ) )->INTENSET.bit.MC0 = 1 ; // Enable interrupts when timer counter matches duty value
			( (Tcc *)( PWMTimerInfo [ pPWMData->timerIndex ].TCCx ) )->INTENSET.vec.MC = 1 << pPWMData->MCx; // Enable interrupts when timer counter matches duty value
																							 // TCC1->INTENSET.bit.MC0 = 1;
		}
	}
	return true;
}

/// @brief Disables TCC
void MNPWMLib::StopPWM ()
{
	*(RwReg *)PWMTimerInfo [ m_pin ].REG_TCCx_CTRLA &= ~( TCC_CTRLA_ENABLE );
	while ( PWMTimerInfo [ m_pin ].TCCx->SYNCBUSY.bit.ENABLE )
		;
}

void MNPWMLib::RestartPWM ()
{
}

/// @brief Enables TCC
void MNPWMLib::StartPWM ()
{
	*(RwReg *)PWMTimerInfo [ m_pin ].REG_TCCx_CTRLA |= TCC_CTRLA_ENABLE;
	while ( PWMTimerInfo [ m_pin ].TCCx->SYNCBUSY.bit.ENABLE )
		;
}

inline int8_t MNPWMLib::pin2Port ( pin_size_t arduinoPin )
{
	return PWMPinInfo [ PinData ( arduinoPin ) ].port;
}

inline uint32_t MNPWMLib::pin2SAMD21 ( pin_size_t arduinoPin )
{
	return PWMPinInfo [ PinData ( arduinoPin ) ].samd21Pin;
}

inline uint8_t MNPWMLib::pin2TCCx ( pin_size_t arduinoPin )
{
	return PWMPinInfo [ PinData ( arduinoPin ) ].timerIndex;
}

/// @brief Gets the maximum TOP value for a given arduino pin
/// @param arduinoPin
/// @return maximum value for TOP, 0 if pin invalid
inline uint32_t MNPWMLib::pin2MaxTop ( pin_size_t arduinoPin )
{
	uint32_t	result	= 0UL;

	// public function so validate parameter
	PWMPinData *pinData = GetPinInfo ( arduinoPin );
	if ( pinData != nullptr )
	{
		result = PWMTimerInfo [ pinData->timerIndex ].counterSize;
	}
	return result;
}

/// @brief Attempts to find the best prescalar and top for required frequency
/// @param arduinoPin pin to be used must be between 0 - 9 inclusive
/// @param wantedFreq the frequency wanted
/// @param prescalar will hold recommended  prescalar if true returned
/// @param top will hold the top recommended if true returned
/// @return false if no match identified
bool MNPWMLib::BestFit ( pin_size_t arduinoPin, uint16_t wantedFreq, uint16_t &prescaler, uint32_t &top )
{
	bool				  result			 = false;
	static const uint16_t PrescalarValues [] = { 1, 2, 4, 8, 16, 64, 256, 1024 }; // must be in ascending order
#define MAX_SCALAR PrescalarValues [ sizeof ( PrescalarValues ) / sizeof ( PrescalarValues [ 0 ] ) - 1 ]
	// Get max top for the supplied pin
	uint32_t maxtop = pin2MaxTop ( arduinoPin );
	if ( maxtop != 0 )
	{
		if ( F_CPU / MAX_SCALAR <= maxtop ) // check max prescalar
		{
			// can be calculated with clock divisor of 1
			uint32_t bestDiff = 0xFFFFFFFF;
			for ( uint8_t i = 0; i < sizeof ( PrescalarValues ) / sizeof ( PrescalarValues [ 0 ] ); i++ )
			{
				uint32_t candidate = F_CPU / wantedFreq / PrescalarValues [ i ];
				if ( candidate < maxtop )
				{
					// see how good this is
					uint32_t diff = abs ( int32_t ( wantedFreq - ( candidate * F_CPU * PrescalarValues [ i ] ) ) );
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

inline uint32_t MNPWMLib::pin2PortMUX ( pin_size_t arduinoPin )
{
	return PWMPinInfo [ PinData ( arduinoPin ) ].Mux;
}

/// @brief returns the index for the supplied arduino pin number
/// @param arduinoPin
/// @return index for the relevenant data, if pin not found returns 0!
uint8_t MNPWMLib::PinData ( uint8_t arduinoPin )
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

PWMPinData *MNPWMLib::GetPinInfo ( pin_size_t arduinoPin )
{
	PWMPinData *pResult = nullptr;
	for ( uint8_t i = 0; i < PWMPinInfoTableSize; i++ )
	{
		if ( PWMPinInfo [ i ].arduinoPin == arduinoPin )
		{
			return &PWMPinInfo [ i ];
		}
	}
	return nullptr;
}

/// @brief Interrupt Service Routine (ISR) for timer TCC0
void TCC0_Handler ()
{
	if ( TCC0->INTENSET.bit.OVF && TCC0->INTFLAG.bit.OVF )
	{
		pwmCallbacks.Overflow ( TCC0, REG_TCC0_CCB0 );
		TCC0->INTFLAG.bit.OVF = 1;
	}
	else if ( TCC0->INTENSET.bit.MC0 && TCC0->INTFLAG.bit.MC0 )
	{
		// have we matched the DUTY value?
		pwmCallbacks.Match ( TCC0, REG_TCC0_CCB0 );
		TCC0->INTFLAG.bit.MC0 = 1;
	}
	else if ( TCC0->INTENSET.bit.MC1 && TCC0->INTFLAG.bit.MC1 )
	{
		// have we matched the DUTY value?
		pwmCallbacks.Match ( TCC0, REG_TCC0_CCB1 );
		TCC0->INTFLAG.bit.MC1 = 1;
	}
	else if ( TCC0->INTENSET.bit.MC2 && TCC0->INTFLAG.bit.MC2 )
	{
		// have we matched the DUTY value?
		pwmCallbacks.Match ( TCC0, REG_TCC0_CCB2 );
		TCC0->INTFLAG.bit.MC1 = 2;
	}
	else if ( TCC0->INTENSET.bit.MC3 && TCC0->INTFLAG.bit.MC3 )
	{
		// have we matched the DUTY value?
		pwmCallbacks.Match ( TCC0, REG_TCC0_CCB3 );
		TCC0->INTFLAG.bit.MC1 = 3;
	}
}

/// @brief Interrupt Service Routine (ISR) for timer TCC1
void TCC1_Handler () // Interrupt Service Routine (ISR) for timer TCC1
{
	if ( TCC1->INTENSET.bit.OVF && TCC1->INTFLAG.bit.OVF )
	{
		TCC1->INTFLAG.bit.OVF = 1;
	}
	else if ( TCC1->INTENSET.bit.MC0 && TCC1->INTFLAG.bit.MC0 )
	{
		// have we matched the DUTY value?
		pwmCallbacks.Match ( TCC1, REG_TCC0_CCB0 );
		TCC1->INTFLAG.bit.MC0 = 1;
	}
	else if ( TCC1->INTENSET.bit.MC1 && TCC1->INTFLAG.bit.MC1 )
	{
		// have we matched the DUTY value?
		pwmCallbacks.Match ( TCC1, REG_TCC0_CCB1 );
		TCC1->INTFLAG.bit.MC1 = 1;
	}
}

/// @brief Interrupt Service Routine (ISR) for timer TCC2
void TCC2_Handler () // Interrupt Service Routine (ISR) for timer TCC2
{
	if ( TCC2->INTENSET.bit.OVF && TCC2->INTFLAG.bit.OVF )
	{
		TCC2->INTFLAG.bit.OVF = 1;
	}
	else if ( TCC2->INTENSET.bit.MC0 && TCC2->INTFLAG.bit.MC0 )
	{
		// have we matched the DUTY value?
		pwmCallbacks.Match ( TCC2, REG_TCC0_CCB0 );
		TCC2->INTFLAG.bit.MC0 = 1;
	}
	else if ( TCC2->INTENSET.bit.MC1 && TCC2->INTFLAG.bit.MC1 )
	{
		// have we matched the DUTY value?
		pwmCallbacks.Match ( TCC2, REG_TCC0_CCB1 );
		TCC2->INTFLAG.bit.MC1 = 1;
	}
}

bool MNPWMLib::bInitialised = false;