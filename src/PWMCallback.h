#pragma once
#include "Arduino.h"

namespace MN::PWMLib
{
	/*
		PWMCallback.h

		defines class for maintaining callbacks functions invoked when a TCC counter hits it match (ie duty level) or hits its top value (overflow)

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
			static bool Set ( voidFuncPtr matchFn, voidFuncPtr overflowFn, uint8_t TCC, uint8_t CCB );
			static void Overflow ( ::Tcc *pTCC, RwReg CCB );
			static void Match ( ::Tcc *pTCC, RwReg CCB );

		private:
			static PWMPinCallbacks m_PWMcallbackList [ MAX_TCC ][ MAX_CCB ];
			static uint8_t		   ConvertTCC ( ::Tcc *pTCC );
			static uint8_t		   ConvertCCB ( RwReg CCB );
	};
} // namespace MN::PWMLib
