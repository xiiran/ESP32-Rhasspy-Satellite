#include "SpencerDevice.hpp"
#include <Spencer.h>
#include "AudioGeneratorWAV.h"
#include "AudioOutputI2S.h"
#include "AudioFileSourcePROGMEM.h"


#define CONFIG_I2S_BCK_PIN 16
#define CONFIG_I2S_LRCK_PIN 27
#define CONFIG_I2S_DATA_PIN 4
#define CONFIG_I2S_DATA_IN_PIN 32

#define SPEAKER_I2S_NUMBER I2S_NUM_0


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

    int sampleRate, bitDepth, numChannels;
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
        i2s_set_clk(I2S_NUM_0, sampleRate / 2, static_cast<i2s_bits_per_sample_t>(2*bitDepth), static_cast<i2s_channel_t>(numChannels));
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

// void interleave(const int16_t * in_L, const int16_t * in_R, int16_t * out, const size_t num_samples)
// {
//     for (size_t i = 0; i < num_samples; ++i)
//     {
//         out[i * 2] = in_L[i];
//         out[i * 2 + 1] = in_R[i];
//     }
// }

void SpencerDevice::writeAudio(uint8_t *data, size_t size, size_t *bytes_written)
{
    //Serial.print("Size: ");
    //Serial.println(size);
    // if( impl->numChannels == 2 ){
    //     int16_t mono[size / sizeof(int16_t)];
    //     // Convert 8 bit to 16 bit
    //     for (int i = 0; i < size; i += 2) {
    //         mono[i/2] = ((data[i] & 0xff) | (data[i + 1] << 8));
    //     }


    //     // const uint32_t bufferSize16bit = sizeof(uint16_t) * size / 4;
    //     // uint16_t buffer16bit[bufferSize16bit];
    //     // for(int j = 0; j < size; j += 4){
    //     //     uint16_t sample = *(uint16_t*) (&data[j + 2]) /*+ 3705*/;
    //     //     buffer16bit[j / 4] = sample;
    //     // }



    //     // int16_t mono[size / sizeof(int16_t)];
    //     // // Convert 8 bit to 16 bit
    //     // for (int i = 0; i < size; i += 2) {
    //     //     mono[i/2] = data[i];
    //     // }



    //     // int16_t mono[size / sizeof(int16_t)];
    //     // int16_t stereo[size];
    //     // //Convert 8 bit to 16 bit
    //     // for (int i = 0; i < size; i += 2) {
    //     //     mono[i/2] = ((data[i] & 0xff) | (data[i + 1] << 8));
    //     // }
    //     // interleave(mono, mono, stereo, size / sizeof(int16_t));


    //     // int16_t mono[size*sizeof(uint16_t)];
    //     // //Convert 8 bit to 16 bit
    //     // for (int i = 0; i < size; ++i) {
    //     //     int16_t sample = data[i];
    //     //     mono[i] = sample * 256;
    //     // }


    //     i2s_write(I2S_NUM_0, mono, size, bytes_written, portMAX_DELAY);
    // }
    // else
    // {
    //     int16_t mono[size / sizeof(int16_t)];
    //     //Convert 8 bit to 16 bit
    //     for (int i = 0; i < size; i += 2) {
    //         mono[i/2] = ((data[i] & 0xff) | (data[i + 1] << 8));
    //     }
    //     i2s_write(I2S_NUM_0, mono, sizeof(mono), bytes_written, portMAX_DELAY);
    // }



    i2s_write(I2S_NUM_0, data, size, bytes_written, portMAX_DELAY);
}

void SpencerDevice::SpencerDeviceImpl::InitI2SSpeakerOrMic(int mode)
{
    // use MODE_MIC from deviceDefinitions
    int MODE_MIC = 0;

    esp_err_t err = ESP_OK;

    i2s_driver_uninstall(SPEAKER_I2S_NUMBER);
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER),
        .sample_rate = 16000,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // spencer library says: could only get it to work with 32bits
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = 0, // Interrupt level 1
        .dma_buf_count = 16,
        .dma_buf_len = 60,
    };
    if (mode == MODE_MIC)
    {
        i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX);
        i2s_config.use_apll = false;
    }
    else
    {
        i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX);
        i2s_config.use_apll = false;
        i2s_config.tx_desc_auto_clear = true;
    }

    err += i2s_driver_install(SPEAKER_I2S_NUMBER, &i2s_config, 0, NULL);
    i2s_pin_config_t tx_pin_config;

    tx_pin_config.bck_io_num = CONFIG_I2S_BCK_PIN;
    tx_pin_config.ws_io_num = CONFIG_I2S_LRCK_PIN;
    tx_pin_config.data_out_num = CONFIG_I2S_DATA_PIN;
    tx_pin_config.data_in_num = CONFIG_I2S_DATA_IN_PIN;

    err += i2s_set_pin(SPEAKER_I2S_NUMBER, &tx_pin_config);
    err += i2s_set_clk(SPEAKER_I2S_NUMBER, 16000, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);

    return;
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