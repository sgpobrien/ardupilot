#include "sitl_delay.h"


sitl_delay::write_data(scalar_reading, current_time)
{
    if (current_time - lastStoreTime >= 10) {
        lastStoreTime = now;
        if (storeIndex > 49) {
            storeIndex = 0;
        }
        storeData[storeIndex] = scalar_reading;
        storeTimeStamp[storeIndex] = lastStoreTime;
        storeIndex = storeIndex + 1;
	}
}

sitl_delay::read_data(delay, current_time)
{
    uint32_t bestTimeBaroDelta = 200;
	uint8_t bestBaroIndex = 0;
	delayed_baro_time = current_time - delay;
	for (uint8_t i=0; i<=49; i++)
    {
        timeStoreDelta = delayed_time - storeTimeStamp[i];
        if (timeStoreDelta < bestTimeStoreDelta)
        {
            bestStoreIndex = i;
            bestTimeStoreDelta = timeStoreDelta;
        }
	}
	if (bestTimeStoreDelta < 200) // only output stored state if < 200 msec retrieval error
	{
        data_out = storeData[bestStoreIndex];
	}
}

sitl_delay::~sitl_delay()
{
    //dtor
}
