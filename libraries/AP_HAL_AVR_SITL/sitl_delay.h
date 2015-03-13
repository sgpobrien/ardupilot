#ifndef SITL_DELAY_H
#define SITL_DELAY_H


class sitl_delay
{
    public:
        void write_data(float &scalar_reading, uint32_t &current_time);
        void read_data(uint8_t &delay, uint32_t &current_time);
        virtual ~sitl_delay();
    private:
        uint8_t storeIndex;
        uint32_t lastStoreTime;
        VectorN<uint32_t,50> storeTimeStamp;
        VectorN<float,50> storeData;
        uint32_t timeStoreDelta;
        uint32_t delayed_time;
};

#endif // SITL_DELAY_H
