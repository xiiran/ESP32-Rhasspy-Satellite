#ifndef ESP32RHASSPYSATELLITE_DEVICE_H
#define ESP32RHASSPYSATELLITE_DEVICE_H

class Device {
  protected:
    int mode;
  public:
     //You can init your device here if needed
    virtual void init() {};
     //If your device has leds, override these methods to set the colors and brightness
    virtual void updateColors(int colors) {};
    virtual void updateBrightness(int brightness) {};
    //It may be needed to switch between read and write, i.e. if the need the same PIN
    //Override both methods
    virtual void setReadMode() {}; 
    virtual void setWriteMode(int sampleRate, int bitDepth, int numChannels) {}; 
    //Override these two methods to read and write
    virtual void writeAudio(uint8_t *data, size_t size, size_t *bytes_written) {};
    virtual bool readAudio(uint8_t *data, size_t size) {return false;};
    //Some devices cause a noise, even when no sound is played. override this to fix it
    virtual void muteOutput(bool mute) {};
    //Some devices have multiple outputs (jack/speeker)
    virtual void ampOutput(int output) {};
    //Some devices support settings of volume
    virtual void setVolume(uint16_t volume) {};
    //Possiblity to set gain
    virtual void setGain(uint16_t gain) {};
    //You can use this method to activated the hotword state (i.e. a hardware button)
    virtual bool isHotwordDetected() {return false;};
    //
    //You can override these in your device
    int readSize = 256;
    int writeSize = 256;
    int width = 2;
    int rate = 16000;
};

#endif
