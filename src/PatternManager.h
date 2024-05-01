#ifndef PATTERNMANAGER_H
#define PATTERNMANAGER_H

#include <vector>

#include "patterns.h"
#include "ledgraph.h"

template <typename BufferType>
class PatternManager {
  int patternIndex = -1;
  Pattern *activePattern = NULL;
  uint8_t activePatternBrightness = 0xFF;

  bool patternAutoRotate = false; // FIXME: need pattern swapping that plays bits to EOL
  unsigned long patternTimeout = 40*1000;

  std::vector<Pattern * (*)(void)> patternConstructors;

  BufferType &ctx;

  template<class T>
  static Pattern *construct() {
    return new T();
  }

  // Make testIdlePattern in this constructor instead of at global so the Pattern doesn't get made at launch
  Pattern *TestIdlePattern() {
    static Pattern *testIdlePattern = NULL;
    if (testIdlePattern == NULL) {
      testIdlePattern = new TestPattern();
    }
    return testIdlePattern;
  }

public:
  PatternManager(BufferType &ctx) : ctx(ctx) {
    patternConstructors.push_back(&(construct<TestPattern>));
  }

  ~PatternManager() {
    delete activePattern;
  }

  void nextPattern() {
    patternAutoRotate = false;
    patternIndex = addmod8(patternIndex, 1, patternConstructors.size());
    if (!startPatternAtIndex(patternIndex)) {
      nextPattern();
    }
  }

  void previousPattern() {
    patternAutoRotate = false;
    patternIndex = mod_wrap(patternIndex - 1, patternConstructors.size());
    if (!startPatternAtIndex(patternIndex)) {
      previousPattern();
    }
  }

  void enablePatternAutoRotate() {
    logf("Enable pattern autorotate");
    patternAutoRotate = true;
    // patternAutorotateWelcome();
    stopPattern();
  }

  void stopPattern() {
    if (activePattern) {
      activePattern->stop();
      delete activePattern;
      activePattern = NULL;
    }
  }

private:
  bool startPatternAtIndex(int index) {
    stopPattern();
    auto ctor = patternConstructors[index];
    Pattern *nextPattern = ctor();
    if (startPattern(nextPattern)) {
      patternIndex = index;
      return true;
    } else {
      delete nextPattern; // patternConstructors returns retained
      return false;
    }
  }
public:
  bool startPattern(Pattern *pattern) {
    stopPattern();
    if (pattern->wantsToRun()) {
      pattern->start();
      activePattern = pattern;
      return true;
    } else {
      return false;
    }
  }

  void setup() {
  }

  void loop() {
    ctx.leds.fill_solid(CRGB::Black);

    if (activePattern) {
      activePattern->loop();
      activePattern->ctx.blendIntoContext(ctx, BlendMode::blendBrighten, 0xFF);
    }

    // time out idle patterns
    if (patternAutoRotate && activePattern != NULL && activePattern->isRunning() && activePattern->runTime() > patternTimeout) {
      if (activePattern != TestIdlePattern() && activePattern->wantsToIdleStop()) {
        activePattern->stop();
        delete activePattern;
        activePattern = NULL;
      }
    }

    // start a new random pattern if there is none
    if (activePattern == NULL) {
      Pattern *testPattern = TestIdlePattern();
      if (testPattern) {
        startPattern(testPattern);
      } else {
        int choice = (int)random8(patternConstructors.size());
        startPatternAtIndex(choice);
      }
    }
    // controls.update();
  }
};

#endif
