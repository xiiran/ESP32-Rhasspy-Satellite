#include "SpencerDevice.hpp"
#include <Spencer.h>
extern "C" {
  #include "speex_resampler.h"
}

const uint32_t i2sBufferSize = 1600;
int err;
SpeexResamplerState *resampler = speex_resampler_init(1, 44100, 44100, 0, &err);


struct SpencerDevice::SpencerDeviceImpl {
    I2S i2s;

    void InitI2SSpeakerOrMic(int mode);

    const uint32_t wavBufferSize = sizeof(uint16_t) * i2sBufferSize / 4; // i2sBuffer is stereo by byte, wavBuffer is mono int16
	char* i2sBuffer = static_cast<char*>(malloc(i2sBufferSize));
	uint16_t* wavBuffer = static_cast<uint16_t*>(malloc(wavBufferSize));
    
    // SetGain and Amplify same as in ESP8266Audio/src/AudioOutput.h
    bool SetGain(float f) { if (f>4.0) f = 4.0; if (f<0.0) f=0.0; gainF2P6 = (uint8_t)(f*(1<<6)); return true; }
    inline int16_t Amplify(int16_t s) {
      int32_t v = (s * gainF2P6)>>6;
      if (v < -32767) return -32767;
      else if (v > 32767) return 32767;
      else return (int16_t)(v&0xffff);
    }
    uint8_t gainF2P6; // Fixed point 2.6

    int sampleRate, bitDepth, numChannels;
};


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
    impl->sampleRate = sampleRate;
	impl->bitDepth = bitDepth;
	impl->numChannels = numChannels;
    const int modeSpeaker = 1;
    if (mode != modeSpeaker)
    {
        Serial.println("Init speaker");
        impl->InitI2SSpeakerOrMic(modeSpeaker);
        mode = modeSpeaker;
    }

    if (sampleRate > 0) {
        int targetSampleRate = ( sampleRate == 44100 ) ? sampleRate / 2 : sampleRate;
        i2s_set_clk(I2S_NUM_0, targetSampleRate, I2S_BITS_PER_SAMPLE_32BIT, static_cast<i2s_channel_t>(numChannels));
    }

    speex_resampler_set_rate(resampler,sampleRate,32000);
	speex_resampler_skip_zeros(resampler);
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
    impl->SetGain(0.05);
    if( impl->sampleRate == 44100 ){
        int16_t mono[size / sizeof(int16_t)];
        // Convert 8 bit to 16 bit
        for (int i = 0; i < size; i += 2) {
            mono[i/2] = impl->Amplify((data[i] & 0xff) | (data[i + 1] << 8));
        }
        i2s_write(I2S_NUM_0, mono, size, bytes_written, portMAX_DELAY);
    }
    else
    {
		uint32_t in_len = size / sizeof(int16_t);
		uint32_t out_len = size * (float)(32000 / impl->sampleRate);
		int16_t output[out_len];
		int16_t input[in_len];
		// Convert 8 bit to 16 bit
		for (int i = 0; i < size; i += 2) {
				input[i/2] = impl->Amplify((data[i] & 0xff) | (data[i + 1] << 8));
		}
   
        speex_resampler_process_int(resampler, 0, input, &in_len, output, &out_len);

        i2s_write(I2S_NUM_0, output, size * 2, bytes_written, portMAX_DELAY);
        *bytes_written = size;
    }
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