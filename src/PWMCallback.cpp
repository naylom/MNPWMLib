#include "PWMCallback.h"

namespace MN::PWMLib
{
	// create instance of PWMCallbacks

	PWMPinCallbacks PWMCallback::m_PWMcallbackList [ MAX_TCC ][ MAX_CCB ];

	PWMCallback::PWMCallback ()
	{
		for ( uint8_t i = 0; i < MAX_TCC; i++ )
		{
			for ( uint8_t j = 0; j < MAX_CCB; j++ )
			{
				m_PWMcallbackList [ i ][ j ].MatchFn	   = nullptr;
				m_PWMcallbackList [ i ][ j ].OverflowFn	   = nullptr;
				m_PWMcallbackList [ i ][ j ].MatchParam	   = nullptr;
				m_PWMcallbackList [ i ][ j ].OverflowParam = nullptr;
			}
		}
	}

	bool PWMCallback::Set ( voidFuncPtr matchFn, voidFuncPtr overflowFn, uint8_t TCC, uint8_t CCB )
	{
		bool result = false;
		// check no callbacks already assigned
		if ( m_PWMcallbackList [ TCC ][ CCB ].MatchFn == nullptr && m_PWMcallbackList [ TCC ][ CCB ].OverflowFn == nullptr )
		{
			m_PWMcallbackList [ TCC ][ CCB ].MatchFn	= matchFn;
			m_PWMcallbackList [ TCC ][ CCB ].OverflowFn = overflowFn;
			result										= true;
		}

		return result;
	}

	bool PWMCallback::Set ( voidFuncPtrParam matchFnParam, voidFuncPtrParam overflowFnParam, uint8_t TCC, uint8_t CCB, void *MatchParam, void *OverflowParam )
	{
		bool result = false;
		// check no callbacks already assigned
		if ( m_PWMcallbackList [ TCC ][ CCB ].MatchFn == nullptr && m_PWMcallbackList [ TCC ][ CCB ].OverflowFn == nullptr )
		{	
			m_PWMcallbackList [ TCC ][ CCB ].MatchFnParam	 = matchFnParam;
			m_PWMcallbackList [ TCC ][ CCB ].MatchParam		 = MatchParam;
			m_PWMcallbackList [ TCC ][ CCB ].OverflowFnParam = overflowFnParam;
			m_PWMcallbackList [ TCC ][ CCB ].OverflowParam	 = OverflowParam;
			result											 = true;
		}
		return result;
	}

	/// @brief Calls the required callback for a given TCC overflow interrupt on CCB counter
	/// @param pTCC TCC that generated the interrupt
	/// @param CCB Counter within TC that matched
	void PWMCallback::Overflow ( ::Tcc *pTCC, RwReg CCB )
	{
		uint8_t indexTCC = ConvertTCC ( pTCC );
		uint8_t indexCCB = ConvertCCB ( CCB );
		// check we have a callback configured
		if ( m_PWMcallbackList [ indexTCC ][ indexCCB ].OverflowFn != nullptr )
		{
			// see if callback expects a parameter or not
			if ( m_PWMcallbackList [ indexTCC ][ indexCCB ].OverflowParam == nullptr )
			{
				m_PWMcallbackList [ indexTCC ][ indexCCB ].OverflowFn ();
			}
			else
			{
				m_PWMcallbackList [ indexTCC ][ indexCCB ].OverflowFnParam ( m_PWMcallbackList [ indexTCC ][ indexCCB ].OverflowParam );
			}
		}
	}

	/// @brief Calls the required callback for a given TCC match interrupt on CCB counter
	/// @param pTCC TCC that generated the interrupt
	/// @param CCB Counter within TC that matched
	void PWMCallback::Match ( ::Tcc *pTCC, RwReg CCB )
	{
		uint8_t indexTCC = ConvertTCC ( pTCC );
		uint8_t indexCCB = ConvertCCB ( CCB );
		// check we have a callback configured
		if ( m_PWMcallbackList [ indexTCC ][ indexCCB ].MatchFn != nullptr )
		{
			// see if callback expects a parameter or not
			if ( m_PWMcallbackList [ indexTCC ][ indexCCB ].MatchParam == nullptr )
			{
				m_PWMcallbackList [ indexTCC ][ indexCCB ].MatchFn ();
			}
			else
			{
				m_PWMcallbackList [ indexTCC ][ indexCCB ].MatchFnParam ( m_PWMcallbackList [ indexTCC ][ indexCCB ].MatchParam );
			}
		}
	}

	uint8_t PWMCallback::ConvertTCC ( ::Tcc *pTCC )
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
} // namespace MN::PWMLib