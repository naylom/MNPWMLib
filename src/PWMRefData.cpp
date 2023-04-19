#include "PWMRefData.h"
#include "MNPrescaler.h"

/*
	PWMRefData.cpp

	implements classes that hold PWM reference data as it relates to the three TCC timers and separately for each of the supported pins how they relate to which TCC functions

*/

namespace MN::PWMLib::RefData
{
	TCC::TCC ( Tcc *pTcc, RwReg *pCtrlAReg, RwReg *pWaveReg, RwReg *pPerReg, IRQn IRQN, uint32_t counterMax, uint32_t steps, bool Configured ) : m_IRQn ( IRQN ), m_counterMax ( counterMax )
	{
		m_pTcc		 = pTcc;
		m_pCtrlAReg	 = pCtrlAReg;
		m_pWaveReg	 = pWaveReg;
		m_pPerReg	 = pPerReg;
		m_steps		 = steps;
		m_prescaler	 = 0;
		m_Configured = Configured;
		m_Running	 = false;
	}

	void TCC::Clear ()
	{
		if ( IsRunning () )
		{
			Stop ();
		}
		SetConfigured ( false );
		SetRunning ( false );
		m_prescaler = 0;
		m_steps		= 0;
	}

	inline bool TCC::IsRunning ()
	{
		return m_Running;
	}

	inline bool TCC::IsConfigured ()
	{
		return m_Configured;
	}

	void TCC::SetPrescaler ( uint16_t prescaler )
	{
		m_prescaler			  = prescaler;
		*(RwReg *)m_pCtrlAReg |= MN::PWMLib::PreScalerLookup::value2Ref ( prescaler ); // Requires SYNC on CTRLA
	}

	inline uint16_t TCC::GetPrescaler ()
	{
		return m_prescaler;
	}

	void TCC::Enable ()
	{
		*(RwReg *)m_pCtrlAReg |= TCC_CTRLA_ENABLE; // Requires SYNC on CTRLA
	}

	void TCC::EnableInterrupts ()
	{
		// Enable interrupts for this TCCn
		NVIC_DisableIRQ ( m_IRQn );
		NVIC_ClearPendingIRQ ( m_IRQn );
		NVIC_SetPriority ( m_IRQn, 0 ); // Set the Nested Vector Interrupt Controller (NVIC) priority
		NVIC_EnableIRQ ( m_IRQn );		// Connect TCC to Nested Vector Interrupt Controller (NVIC)
	}

	void TCC::EnableOverflow ()
	{
		( (::Tcc *)m_pTcc )->INTENSET.bit.OVF = 1; // Enable Interrupts when time counter reaches TOP
	}

	void TCC::EnableMatch ( uint8_t MCx )
	{
		( (::Tcc *)m_pTcc )->INTENSET.vec.MC |= 1 << MCx; // Enable interrupts when timer counter matches duty value
	}

	uint32_t TCC::GetCounterMax ()
	{
		return m_counterMax;
	}

	Tcc *TCC::GetTcc ()
	{
		return (Tcc *)m_pTcc;
	}

	void TCC::Start ()
	{
		Enable ();
		while ( m_pTcc->SYNCBUSY.bit.ENABLE )
			;
		m_Running = true;
	}

	void TCC::Stop ()
	{
		// Stops timer so all pins using it will be impacted! Better to set duty = 2 or top?
		( (::Tcc *)m_pTcc )->CTRLBSET.reg = TCC_CTRLBCLR_CMD_STOP; // Stop the timer
		SyncCtrlBReg ();
		m_Running = false;
	}

	void TCC::Restart ()
	{
		ResetCount ();
		( (::Tcc *)m_pTcc )->CTRLBSET.reg = TCC_CTRLBCLR_CMD_RETRIGGER; // Stop the timer
		SyncCtrlBReg ();

		m_Running = true;
	}

	void TCC::SetCounter ( uint32_t newValue )
	{
		m_steps = min ( m_counterMax, newValue );
	}

	// should this be per TCC?
	void TCC::SetTop ( uint32_t top )
	{
		top					= max ( top, (uint32_t)2 );
		top					= min ( top, m_counterMax );
		*(RwReg *)m_pPerReg = top;
		SyncPER ();
	}

	void TCC::SyncCtrlBReg ()
	{
		while ( ( (::Tcc *)m_pTcc )->SYNCBUSY.bit.CTRLB )
			; // Wait for synchronization
	}

	void TCC::SyncPER ()
	{
		while ( m_pTcc->SYNCBUSY.bit.PER )
			; // Wait for synchronization
	}

	void TCC::SetConfigured ( bool state )
	{
		m_Configured = state;
	}

	void TCC::SetRunning ( bool State )
	{
		m_Running = State;
	}

	void TCC::SetPWMType ( uint32_t type )
	{
		*(RwReg *)m_pWaveReg |= type;
		SyncWave ();
	}

	void TCC::SyncWave ()
	{
		while ( m_pTcc->SYNCBUSY.bit.WAVE )
			; // Wait for synchronization
	}

	void TCC::ResetCount ()
	{
		// reset the count to 0
		*(RwReg *)m_pTcc->COUNT.reg = 0 & m_counterMax;

		while ( m_pTcc->SYNCBUSY.bit.COUNT ) // Wait for synchronization
			;
	}

	void TCCList::ClearAll ()
	{
		for ( uint8_t i = 0; i < MAX_TCC; i++ )
		{
			m_list [ i ].Clear ();
		}
	}

	TCC *TCCList::GetTCCData ( uint8_t index )
	{
		return &m_list [ index % MAX_TCC ];
	}

	PWMPinData::PWMPinData ( pin_size_t arduinoPin, int8_t port, uint32_t samd21Pin, uint8_t TCCIndex, RwReg *REG_TCCx_CCBy, uint8_t MCx, uint32_t Mux, uint8_t WOn )
		: m_arduinoPin ( arduinoPin ), m_port ( port ), m_samd21Pin ( samd21Pin ), m_TCCIndex ( TCCIndex ), m_REG_TCCx_CCBy ( REG_TCCx_CCBy ), m_MCx ( MCx ), m_Mux ( Mux ), m_WOn ( WOn )
	{
		m_duty = 0UL;
	}

	uint8_t PWMPinData::GetTCCIndex ()
	{
		return m_TCCIndex;
	}

	uint8_t PWMPinData::GetMC ()
	{
		return m_MCx;
	}

	void PWMPinData::SetDuty ( uint32_t duty )
	{
		m_duty					  = duty;
		*(RwReg *)m_REG_TCCx_CCBy = duty;
		while ( TCCList::GetTCCData ( m_TCCIndex )->GetTcc ()->SYNCBUSY.vec.CCB )
			;
	}

	void PWMPinData::RouteClockToPin ()
	{
		PORT->Group [ m_port ].PINCFG [ m_samd21Pin ].reg	 |= PORT_PINCFG_PMUXEN; // set pin to use mux
		PORT->Group [ m_port ].PMUX [ m_samd21Pin >> 1 ].reg |= m_Mux;				// set specific mux for this pin, different mux for odd and even pins based on samd21 pin numbers
	}

	/// @brief Allows the state of the pin (ie HIGH or LOW) to be defined when PWM clock is stopped
	/// @param WOn - wave output number
	inline void PWMPinData::EnablePinStateWhenStopped ()
	{
		TCCList::GetTCCData ( m_TCCIndex )->GetTcc ()->DRVCTRL.vec.NRE |= ( 1 << m_WOn ); // enable
	}

	/// @brief Controls the state of the pin when PWM is stopped
	/// @param defaultState set to HIGH or LOW as required
	void PWMPinData::SetSignalWhenStopped ( PinStatus defaultState )
	{
		if ( defaultState == HIGH )
		{
			TCCList::GetTCCData ( m_TCCIndex )->GetTcc ()->DRVCTRL.vec.NRV |= ( 1 << m_WOn );
		}
		else
		{
			TCCList::GetTCCData ( m_TCCIndex )->GetTcc ()->DRVCTRL.vec.NRV &= ~( 1 << m_WOn );
		}
		EnablePinStateWhenStopped ();
	}

	TCC *PWMPinDataList::pin2TCCData ( pin_size_t ArduinoPin )
	{
		TCC *result = nullptr;
		if ( ArduinoPin < MAX_PWM_PINS )
		{
			result = TCCList::GetTCCData ( m_list [ ArduinoPin ].GetTCCIndex () );
		}
		return result;
	}

	PWMPinData *PWMPinDataList::pin2PinData ( pin_size_t ArduinoPin )
	{
		PWMPinData *result = nullptr;
		if ( ArduinoPin < MAX_PWM_PINS )
		{
			result = &m_list [ ArduinoPin ];
		}
		return result;
	}

	// prepopulated table of TCC data
	TCC TCCList::m_list [ MAX_TCC ] = {
		{TCC0,	&REG_TCC0_CTRLA, &REG_TCC0_WAVE, &REG_TCC0_PER, TCC0_IRQn, (uint32_t)0xFFFFFF, 500000UL, true},
		{ TCC1, &REG_TCC1_CTRLA, &REG_TCC1_WAVE, &REG_TCC1_PER, TCC1_IRQn, (uint32_t)0xFFFFFF, 500000UL, true},
		{ TCC2, &REG_TCC2_CTRLA, &REG_TCC2_WAVE, &REG_TCC2_PER, TCC2_IRQn, (uint32_t)0x00FFFF, 50000UL,	true}
	};

	// prepopulated table of PWM data for arduino pins  *** MUST BE IN ARDUINO PIN NUMBER ORDER WITH NO GAPS ***
	PWMPinData PWMPinDataList::m_list [ MAX_PWM_PINS ] = {
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

} // namespace MN::PWMLib::RefData