#include "SpencerDevice.hpp"
#include <Spencer.h>
#include "AudioGeneratorWAV.h"
#include "AudioOutputI2S.h"
#include "AudioFileSourcePROGMEM.h"


const uint32_t i2sBufferSize = 1600;

struct SpencerDevice::SpencerDeviceImpl {
    //SpencerDeviceImpl();

    I2S i2s;

    void InitI2SSpeakerOrMic(int mode);

    const uint32_t wavBufferSize = sizeof(uint16_t) * i2sBufferSize / 4; // i2sBuffer is stereo by byte, wavBuffer is mono int16
	char* i2sBuffer = static_cast<char*>(malloc(i2sBufferSize));
	uint16_t* wavBuffer = static_cast<uint16_t*>(malloc(wavBufferSize));

    //AudioGeneratorWAV wav;
    //AudioOutputI2S out;
};


// SpencerDevice::SpencerDeviceImpl::SpencerDeviceImpl() : out(0,0,16,0){
// 	out.SetRate(16000);
// 	out.SetPinout(16, 27, 4);
// 	out.SetChannels(1);
// 	out.SetOutputModeMono(1);
// 	float volume = 0.4;
// 	out.SetGain(volume);
// }


SpencerDevice::SpencerDevice() : impl( new SpencerDeviceImpl() )
{
}

SpencerDevice::~SpencerDevice()
{
    free(impl->i2sBuffer);
	free(impl->wavBuffer);
    delete impl;
}

void SpencerDevice::init()
{
    Spencer.begin();
    Spencer.loadSettings();
}

void SpencerDevice::setWriteMode(int sampleRate, int bitDepth, int numChannels)
{
    const int modeSpeaker = 1;
    if (mode != modeSpeaker)
    {
        Serial.println("Init speaker");
        impl->InitI2SSpeakerOrMic(modeSpeaker);
        //impl->out.SetChannels(numChannels);
        //impl->out.SetRate(sampleRate);
        //impl->out.SetBitsPerSample(bitDepth);

        if (sampleRate > 0) {
            i2s_set_clk(I2S_NUM_0, sampleRate, static_cast<i2s_bits_per_sample_t>(bitDepth), static_cast<i2s_channel_t>(numChannels));
        }

        mode = modeSpeaker;
    }
}

void SpencerDevice::setReadMode()
{
    const int modeMic = 0;
    if (mode != modeMic)
    {
        Serial.println("Init mic");
        impl->InitI2SSpeakerOrMic(modeMic);
        mode = modeMic;
    }
}

void SpencerDevice::writeAudio(uint8_t *data, size_t size, size_t *bytes_written)
{
    /**
    // Setup
	AudioFileSourcePROGMEM audioSource(data, size);
	// AudioGeneratorWAV wav;
	// AudioOutputI2S out(0,0,16,0);
	// out.SetRate(16000);
	// out.SetPinout(16, 27, 4);
	// out.SetChannels(1);
	// out.SetOutputModeMono(1);
	// float volume = 0.4;
	// out.SetGain(volume);

	// Run
	Serial.println("Start playback from data");
	impl->wav.begin(&audioSource, &impl->out);
	if(impl->wav.isRunning()){
		impl->wav.loop();
	}
	Serial.println("End playback from data");

    *bytes_written = size;
    */

   i2s_write(I2S_NUM_0, data, size, bytes_written, portMAX_DELAY);
}

void SpencerDevice::SpencerDeviceImpl::InitI2SSpeakerOrMic(int mode)
{
    i2s.stop();
    i2s.begin();
}

bool SpencerDevice::readAudio(uint8_t *data, size_t size)
{
    impl->i2s.Read(impl->i2sBuffer, i2sBufferSize);
    for(int j = 0; j < i2sBufferSize; j += 4){
        uint16_t sample = *(uint16_t*) (&impl->i2sBuffer[j + 2]) /*+ 3705*/;
        impl->wavBuffer[j / 4] = sample;
    }
    memcpy(data, impl->wavBuffer, readSize * width);
    return true;
}

void SpencerDevice::muteOutput(bool mute)
{
}

void SpencerDevice::setVolume(uint16_t volume)
{
}

bool SpencerDevice::isHotwordDetected()
{
    return digitalRead(BTN_PIN) == LOW;
}