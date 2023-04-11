#include "MNPrescaler.h"

namespace MN::PWMLib ::PreScalerLookup
{
	typedef struct
	{
			uint16_t value; // value of prescalar i.e. how much it divides the clock signal by
			uint16_t Ref;	// the constant used in SAMD21 register to represent this value
	} PreScalers;

	// Note the prescaler does not have 32 divider option
	const PreScalers PreScalersList [ 8 ] = {
		{1,	 TCC_CTRLA_PRESCALER_DIV1	  }, //
		{ 2,	 TCC_CTRLA_PRESCALER_DIV2	  }, //
		{ 4,	 TCC_CTRLA_PRESCALER_DIV4	  }, //
		{ 8,	 TCC_CTRLA_PRESCALER_DIV8	  }, //
		{ 16,	  TCC_CTRLA_PRESCALER_DIV16	}, //
		{ 64,	  TCC_CTRLA_PRESCALER_DIV64	}, //
		{ 256,  TCC_CTRLA_PRESCALER_DIV256 }, //
		{ 1024, TCC_CTRLA_PRESCALER_DIV1024}  //
	};

	uint16_t value2Ref ( uint16_t value )
	{
		uint16_t result = TCC_CTRLA_PRESCALER_DIV1;
		for ( uint8_t i = 0; i < sizeof ( PreScalersList ) / sizeof ( PreScalersList [ 0 ] ); i++ )
		{
			if ( PreScalersList [ i ].value == value )
			{
				result = PreScalersList [ i ].Ref;
				break;
			}
		}
		return result;
	}

	uint16_t ref2Value ( uint16_t ref )
	{
		uint16_t result = 1;
		for ( uint8_t i = 0; i < sizeof ( PreScalersList ) / sizeof ( PreScalersList [ 0 ] ); i++ )
		{
			if ( PreScalersList [ i ].Ref == ref )
			{
				result = PreScalersList [ i ].value;
				break;
			}
		}
		return result;
	}

} // namespace MN::PWMLib::PreScalerLookup