#include <MNPWMLib.h>
/*
	Demonstrates:
	 optionally using library to get best values to reduce 48Mhz clock to required frequency
	 optionally using callback routine to something (in this case flash built in led) when duty cycle is up
*/
MNPWMLib	myPWM ();		// Use default clock divisor of 1 

constexpr pin_size_t PWMPin	= D1;	// Must be between D0 and D9
constexpr uint32_t WantedFrequency = 2;	// hertz
constexpr pin_size_t BuiltinLEDPin = 6;

// Optional ISR level routine to do something when the PWM duty limit hit
void PWMDutyCallback ()
{
	static int OnOff = 0;
	digitalWrite ( BuiltinLEDPin, OnOff );
	OnOff == 0 ? 1 : 0;
}

void setup()
{
	uint16_t prescaler;			// Timer prescaler
	uint132_t top;				// desired Frequency

	Serial.begin ( 19200 );
	while ( !Serial );		// wait for serial monitor
	Serial.println ( "MNPWMLib Basic Test example" );
	
	pinMode ( BuiltinLEDPin , OUTPUT  );
	// Calculate best settings to match wanted Frequency, this is optional you can set your own prescaler and top values
	if ( myPWM.BestFit ( PWMPin, WantedFrequency, prescaler, top ) == false )
	{
		Serial.println ( "Unable to automatically determine best settings to get wanted frequency" );
		// stop
		while ( true );
	}

	// start pwm with 50% duty cycle ie top / 2
	if ( myPWM.SetPWM ( PWMPin, prescaler, top / 2, top, nullptr, PWMDutyCallback ) == false )
	{
		Serial.println ( "PWM failed to start" );
		// stop
		while ( true );		
	}

}

void loop()
{
	// do other work whilst signal is generated
}




