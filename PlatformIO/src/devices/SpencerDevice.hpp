#include <Arduino.h>
#include "Device.h"

class SpencerDevice : public Device
{
public:
    SpencerDevice();
    ~SpencerDevice();
    void init() override;

    void setReadMode() override;
    void setWriteMode(int sampleRate, int bitDepth, int numChannels) override;

    void writeAudio(uint8_t *data, size_t size, size_t *bytes_written) override;
    bool readAudio(uint8_t *data, size_t size) override;

    void muteOutput(bool mute) override;

    void setVolume(uint16_t volume) override;

    bool isHotwordDetected() override;

    int readSize = 400;
	int writeSize = 512;
	int width = 2;
	int rate = 16000;

private:
    struct SpencerDeviceImpl;
    SpencerDeviceImpl* impl;
};