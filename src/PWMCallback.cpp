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
				m_PWMcallbackList [ i ][ j ].MatchFn	= nullptr;
				m_PWMcallbackList [ i ][ j ].OverflowFn = nullptr;
			}
		}
	}

	bool PWMCallback::Set ( voidFuncPtr matchFn, voidFuncPtr overflowFn, uint8_t TCC, uint8_t CCB )
	{
		bool result = false;
		if ( m_PWMcallbackList [ TCC ][ CCB ].MatchFn == nullptr && m_PWMcallbackList [ TCC ][ CCB ].OverflowFn == nullptr )
		{
			m_PWMcallbackList [ TCC ][ CCB ].MatchFn	= matchFn;
			m_PWMcallbackList [ TCC ][ CCB ].OverflowFn = overflowFn;
			result										= true;
		}

		return result;
	}

	void PWMCallback::Overflow ( ::Tcc *pTCC, RwReg CCB )
	{
		uint8_t indexTCC = ConvertTCC ( pTCC );
		uint8_t indexCCB = ConvertCCB ( CCB );
		if ( m_PWMcallbackList [ indexTCC ][ indexCCB ].OverflowFn != nullptr )
		{
			m_PWMcallbackList [ indexTCC ][ indexCCB ].OverflowFn ();
		}
	}

	void PWMCallback::Match ( ::Tcc *pTCC, RwReg CCB )
	{
		uint8_t indexTCC = ConvertTCC ( pTCC );
		uint8_t indexCCB = ConvertCCB ( CCB );
		if ( m_PWMcallbackList [ indexTCC ][ indexCCB ].MatchFn != nullptr )
		{
			m_PWMcallbackList [ indexTCC ][ indexCCB ].MatchFn ();
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
}