#ifndef AUDIO_H

#include <PDM.h>
#include <I2S.h>
#if !DUSTLIB_NO_FFT
// Allow excluding fft support since it was causing crashes in hexa for a while.
// Reminder: PlatformIO build will pull in kissfft even if this include is excluded with preprocessor flags, unless lib_ldf_mode = chain+
#include "kiss_fftr.h"
#endif
#include "util.h"

// TODO: flexible fft datatype/bitwidth for kissfft
// TODO: platform flexibility
// TODO: test with another mics
// FIXME: test i2s

#define DEFAULT_NSAMP 256
#define DEFAULT_SAMPLE_RATE 8000

class AudioProcessing;
class DigitalAudioProcessing;
#if !DUSTLIB_NO_FFT
class FFTProcessing;
static FFTProcessing *sharedFFT = NULL;
#endif

volatile int rawBufferFilled = 0;
volatile int rawSamplesRead = 0;

class AudioProcessing {
  int peakAccum = 0;
public:
  int bufferSize;
  int sampleRate;

  int ignoreSamples = 3; // ignore the first n samples of each read
  int peakFrames = 6; // frames across which to compute peak amplitude
  
  AudioProcessing(int sampleRate) : sampleRate(sampleRate) { }

  virtual void subscribe() = 0;
  virtual void unsubscribe() = 0;

  int processAmplitude(int16_t *buffer, size_t size) {
    if (size > ignoreSamples) {
      int16_t min_sample = INT16_MAX;
		  int16_t max_sample = INT16_MIN;
      for (int s = ignoreSamples; s < size / sizeof(buffer[0]); ++s) {
        // logf("buffer[%i] = %i", s, buffer[s]);
        if (buffer[s] > max_sample) max_sample = buffer[s];
        if (buffer[s] < min_sample) min_sample = buffer[s];
      }
      // logf("min_sample = %i, max_sample = %i", min_sample, max_sample);
      int maxAmplitude = max(abs(min_sample), abs(max_sample));
      peakAccum = (peakFrames * peakAccum + maxAmplitude) / (peakFrames + 1);
    }
    return peakAccum;
  }
  virtual size_t read(int16_t *buffer, size_t size) = 0;
};

class DigitalAudioProcessing : public AudioProcessing {
  int subscribeCount = 0;
protected:
  int dataPin;
  int clockPin;
  virtual void startStreaming() { }
  virtual void stopStreaming() { }
public:
  DigitalAudioProcessing(int dataPin, int clockPin, int sampleRate=DEFAULT_SAMPLE_RATE) 
    : AudioProcessing(sampleRate), dataPin(dataPin), clockPin(clockPin) {
  }

  bool isStreaming() {
    return (subscribeCount > 0);
  }

  void subscribe() {
    if (subscribeCount++ == 0) {
      startStreaming();
    }
  }

  void unsubscribe() {
    assert(subscribeCount > 0, "not subscribed");
    if (--subscribeCount == 0) {
      stopStreaming();
    }
  }

  virtual size_t read(int16_t *buffer, size_t size) = 0;
};

class AudioInputI2S : public DigitalAudioProcessing {
public:
  I2S i2s;
  AudioInputI2S(int dataPin, int clockPin) : DigitalAudioProcessing(dataPin, clockPin) { }
protected:
  virtual void startStreaming() {
    i2s.setDATA(dataPin);
    i2s.setBCLK(clockPin);
    assert(i2s.begin(sampleRate), "Failed to initialize I2S device");
  }
  virtual void stopStreaming() {
    i2s.end();
  }
public:
  virtual size_t read(int16_t *buffer, size_t size) {
    assert(isStreaming(), "can't read unless streaming");
    irq_set_enabled(DMA_IRQ_0, false);
    int hasSamples = i2s.available();
    for (int i = 0; i < min(hasSamples, size); ++i) {
      int32_t l=0,r=0;
      i2s.read32(&l, &r);
      buffer[i] = (l?:r)>>16;
    }
    irq_set_enabled(DMA_IRQ_0, true);
    return min(hasSamples, size);
  }
};

class AudioInputPDM : public DigitalAudioProcessing {
  bool fixSelectHIGH;
public:
  AudioInputPDM(int dataPin, int clockPin, bool fixSelectHIGH=false) : DigitalAudioProcessing(dataPin, clockPin), fixSelectHIGH(fixSelectHIGH) { }
protected:
  virtual void startStreaming() {
    PDM.setDIN(dataPin);
    PDM.setCLK(clockPin);

    if (fixSelectHIGH) {
      // https://github.com/earlephilhower/arduino-pico/issues/3223
      // Our LMD4030 microphone must be sampled >15ns after the CLK 0->1 but before the CLK 0->1 transition
      // Workaround this unusual(?) timing by telling the mic to send data in the HIGH channel
      // framework-arduinopico PDM library only supports mono channel anyway
      pinMode(clockPin+1, OUTPUT);
      digitalWrite(clockPin+1, HIGH);
    }

    assert(1 == PDM.begin(1, sampleRate), "Failed to initialize PDM device");
   }
  virtual void stopStreaming() {
    PDM.end();
  }
public:
  virtual size_t read(int16_t *buffer, size_t size) {
    assert(isStreaming(), "can't read unless streaming");
    int hasBytes = PDM.available();
    size_t bytesRead = PDM.read(buffer, min(hasBytes, size));
    return bytesRead;
  }
};

class AmplitudeReceiver {
  AudioProcessing &audio;
  int16_t samples[DEFAULT_NSAMP];
public:
  AmplitudeReceiver(AudioProcessing &audio) : audio(audio) {
    audio.subscribe();
  }
  ~AmplitudeReceiver() {
    audio.unsubscribe();
  }

  int amplitudeFrame(int smoothing=10) {
    bzero(samples, DEFAULT_NSAMP * sizeof(samples[0]));

    int samplesRead = audio.read(samples, DEFAULT_NSAMP*sizeof(samples[0]));
    // logf("read %i bytes from pdm", samplesRead);
    for (int i = 0; i < samplesRead; ++i){
      // if (samples[i]) {
        // logf(" %4i ", samples[i]);
      //}
    }
    // logf("");
    // logf("end of samples");
    int peak = audio.processAmplitude(samples, samplesRead);

    return peak;
  }
};


#if !DUSTLIB_NO_FFT

struct FFTFrame {
  size_t size;
  FFTFrame(size_t size) : size(size) {}
  int32_t *spectrum = NULL;
  int32_t *smoothSpectrum = NULL;
  int peak = 0;
};

class FFTProcessing {
  int windowSize;
  int numBins; // spectrumSize
  int *fftBinSizes;
  int32_t *spectrum;
  int32_t *spectrumAccum;
  int16_t *samples;
  AudioProcessing &audio;
  FFTFrame dataFrame{0};
  int subscribeCount{0};
  bool initialized{false};
public:
  int ignoreBins = 0; // ignore the first n bins of the fft (was required by ZeroFFT)

  FFTProcessing(AudioProcessing &audio, int numBins, int windowSize=DEFAULT_NSAMP) : audio(audio), numBins(numBins), windowSize(windowSize) { }

  void initialize() {
    assert(fftBinSizes == NULL, "fft double initialize");
    fftBinSizes = new int[numBins];
    spectrum = new int32_t[numBins];
    spectrumAccum = new int32_t[numBins];
    samples = new int16_t[windowSize];
    getFFTBins(numBins, windowSize/2, fftBinSizes);
    initialized = true;
  }

  ~FFTProcessing() {
    delete [] fftBinSizes;
    delete [] samples;
    delete [] spectrum;
    delete [] spectrumAccum;
  }

  void subscribe() {
    if (subscribeCount++ == 0) {
      audio.subscribe();
    }
  }

  void unsubscribe() {
    assert(subscribeCount > 0, "not subscribed");
    if (--subscribeCount == 0) {
      audio.unsubscribe();
    }
  }

  void frameReset() {
    if (initialized) {
      bzero(&dataFrame, sizeof(dataFrame));
    }
  }

  FFTFrame getDataFrame(int smoothSamples=0) {
    if (!initialized) {
      initialize();
    }
    if (dataFrame.size != 0) {
      return dataFrame;
    }

    kiss_fft_scalar fft_in[windowSize];
    kiss_fft_cpx fft_out[windowSize];
    // TIMEIT(kissalloc,
    // TODO: this is taking almost 4ms to alloc on every frame (with window size 128). would be nice to figure out if the structures can be reused.
    kiss_fftr_cfg cfg = kiss_fftr_alloc(windowSize,false,0,0);
// );
  
    bzero(samples, windowSize * sizeof(samples[0]));
    
    int samplesRead = audio.read(samples, windowSize*sizeof(samples[0]));
    int peak = audio.processAmplitude(samples, samplesRead);

    // fill fourier transform input while subtracting DC component
    int64_t sum = 0;
    for (int i = 0; i < windowSize; i++) { sum += samples[i]; }
    int32_t avg = sum/windowSize;
    for (int i = 0; i < windowSize; i++) { fft_in[i] = samples[i] - avg; }
    
    // compute fast fourier transform
    kiss_fftr(cfg, fft_in, fft_out);
    
    // any frequency bin over windowSize/2 is aliased (nyquist sampling theorum)
    for (int b = ignoreBins; b < numBins; b++) {
      int stopIndex = (b < numBins - 1 ? fftBinSizes[b + 1] - 1 : windowSize/2 - 1);
      int64_t powerSum= 0;
      for (int i = fftBinSizes[b]; i <= stopIndex; ++i) {
        int64_t power = fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i;
        powerSum += power;
      }

      // FIXME: specific to one microphone? generalize.
      powerSum /= 16384.;

      spectrum[b] = powerSum;
      if (smoothSamples) {
        spectrumAccum[b] = (spectrumAccum[b] * smoothSamples + spectrum[b]) / (float)(smoothSamples + 1);
      }
    }
    kiss_fft_free(cfg);
    
    FFTFrame frame(numBins - ignoreBins);
    frame.spectrum = spectrum;
    if (smoothSamples) {
      frame.smoothSpectrum = spectrumAccum;
    }
    frame.peak = peak;
    dataFrame = frame;
    return dataFrame;
  }

  void logFrame(FFTFrame frame) {
    for (int x = 0; x < frame.size; ++x) {
      int32_t level = frame.spectrum[x];
      if (level > 0) {
        Serial.printf("%4i ", level);
      } else {
        Serial.print("  -  ");
      }
    }
    Serial.printf(" : (%4i)", frame.peak);
    Serial.println();
  }

private:
  // https://forum.pjrc.com/threads/32677-Is-there-a-logarithmic-function-for-FFT-bin-selection-for-any-given-of-bands
  static float FindE(int bins, int window) {
    float increment = 0.1, eTest, n;
    int b, count, d;

    for (eTest = 1; eTest < window; eTest += increment) {     // Find E through brute force calculations
      count = 0;
      for (b = 0; b < bins; b++) {                         // Calculate full log values
        n = pow(eTest, b);
        d = int(n + 0.5);
        count += d;
      }
      if (count > window) {     // We calculated over our last bin
        eTest -= increment;   // Revert back to previous calculation increment
        increment /= 10.0;    // Get a finer detailed calculation & increment a decimal point lower
      }
      else if (count == window)
        return eTest;
      if (increment < 0.0000001)        // Ran out of calculations. Return previous E. Last bin will be lower than (bins-1)
        return (eTest - increment);
    }
    return 0;
  }

  void getFFTBins(int numBins, int window, int *fftBins) {
    const int binStartOffset = 2; // the first two FFT bins are garbage (DC bins?)
    float e = FindE(numBins + 1, window - binStartOffset);
    if (e) {
      int count = binStartOffset;
      Serial.printf("E = %4.4f\n", e);
      for (int b = 0; b < numBins; b++) {
        float n = pow(e, b+1);
        int d = int(n + 0.5);
        Serial.printf( "%4d ", count);
        fftBins[b] = count;
        count += d - 1;
        Serial.printf( "%4d\n", count);
        ++count;
      }
    } else {
      Serial.println("Error\n");
    }
  }
};

class FFTReceiver {
  FFTProcessing &receiverFftProcessing;
public:
  FFTReceiver(FFTProcessing &fft) : receiverFftProcessing(fft) {
    receiverFftProcessing.subscribe();
  }
  ~FFTReceiver() {
    receiverFftProcessing.unsubscribe();
  }

  FFTFrame spectrumFrame() {
    return receiverFftProcessing.getDataFrame();
  }

  void fftLog() {
    receiverFftProcessing.logFrame(spectrumFrame());
  }
};

#endif

#endif
