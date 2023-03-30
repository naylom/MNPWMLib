#pragma once
#include <arduino.h>
/*
	MNPWNLib.h

	defines pwm class for mkr wifi 1010

	Supports pins D0 to D9
	Supports Normal PWM (single slope)

	Uses 48Mhz clock (GCLK4)

	When setting up PWM user can optionally provide two functions to be called when duty level is matched and / or counter reaches top
	such routines are called at the interrupt level and should obey normal constraints. Given PWM can run at very high frequency, such
	routines should be minimal
*/
#ifndef pin_size_t
typedef uint8_t pin_size_t;
#endif

/* defines data used to set up PWM for arduino mkr wifi 1010 pins D0 to D9 inclusive */
typedef struct
{
		const pin_size_t arduinoPin;	// Arduino pin number
		const int8_t	 port;			// Port of the SAMD21 pin
		const uint32_t	 samd21Pin;		// SAMD21 pin
		const uint8_t	 timerIndex;	// TimerLookup index used for this pin
		const RwReg		*REG_TCCx_CCBy; // Pointer to count register used for this pin
		const uint8_t	 MCx;			// MCx bit
		const uint32_t	 Mux;			// Pin multiplexer for this pin
} PWMPinData;

class MNPWMLib
{
	public:
		MNPWMLib ( uint8_t ClockDivisor = 1 );
		bool	 SetPWM ( pin_size_t arduinoPin, uint16_t prescaler, uint32_t duty, uint32_t top, voidFuncPtr OverflowFn = nullptr, voidFuncPtr MatchFn = nullptr );
		void	 StopPWM ();
		void	 RestartPWM ();
		void	 StartPWM ();
		uint32_t pin2MaxTop ( pin_size_t arduinoPin );
		uint8_t	 pin2TCCx ( pin_size_t arduinoPin );
		uint8_t	 pin2CCx ( pin_size_t arduinoPin );
		bool	 BestFit ( pin_size_t arduinoPin, uint16_t wantedFreq, uint16_t &prescaler, uint32_t &top );

	private:
		static bool bInitialised; // Used to indicate initialisation of TCC done
		pin_size_t	m_pin;
		uint32_t	m_top;
		uint32_t	m_duty;
		uint16_t	m_frequency;
		uint16_t	m_prescaler;
		uint8_t		m_clockDivisor;
		int8_t		pin2Port ( pin_size_t arduinoPin );
		uint32_t	pin2SAMD21 ( pin_size_t arduinoPin );
		uint32_t	pin2PortMUX ( pin_size_t arduinoPin );
		uint8_t		PinData ( uint8_t arduinoPin );
		PWMPinData *GetPinInfo ( pin_size_t arduinoPin );
		void		SetDuty ( uint32_t duty, PWMPinData *pData );
		void		SyncCtrlBReg ();
		void		SetPWMType ( uint32_t PWMType );
		void		SetPWMTop ( uint32_t PWMTop );
};