#pragma once
#include <Arduino.h>

namespace MN ::PWMLib ::PreScalerLookup
{
	uint16_t value2Ref ( uint16_t value );
	uint16_t ref2Value ( uint16_t value );
} // namespace MN::PWMLib::PreScalerLookup