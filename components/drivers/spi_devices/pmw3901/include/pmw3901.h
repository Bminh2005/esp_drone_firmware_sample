#ifndef PMW3901_H_
#define PMW3901_H_

#include <stdint.h>
#include <stdbool.h> // Thêm thư viện này để dùng kiểu bool

typedef struct motionBurst_s
{
    union
    {
        uint8_t motion;
        struct
        {
            uint8_t frameFrom0 : 1;
            uint8_t runMode : 2;
            uint8_t reserved1 : 1;
            uint8_t rawFrom0 : 1;
            uint8_t reserved2 : 2;
            uint8_t motionOccured : 1;
        };
    };

    uint8_t observation;
    int16_t deltaX;
    int16_t deltaY;

    uint8_t squal; // Tên biến chất lượng bề mặt trong struct gốc là squal

    uint8_t rawDataSum;
    uint8_t maxRawData;
    uint8_t minRawData;

    uint16_t shutter;
} __attribute__((packed)) motionBurst_t;

bool pmw3901Init(uint32_t csPin);
void pmw3901ReadMotion(uint32_t csPin, motionBurst_t *motion);

#endif /* PMW3901_H_ */