// Please credit chris.keith@gmail.com
// https://opensource.org/licenses/MIT

#include <limits.h>

class SensorData {
private:
  int pin;
  String name;
  bool isAnalog;
  double factor; // apply to get human-readable values, e.g., degrees F

  int lastVal;

public:
  SensorData(int pin, String name, bool isAnalog, double factor) {
    this->pin = pin;
    this->name = name;
    this->isAnalog = isAnalog;
    this->factor = factor;
    this->lastVal = INT_MIN;
    pinMode(pin, INPUT);
  }

  String getName() {
    return name;
  }
  
  String getLastVal() {
    String s(applyFactor(lastVal));
    s.concat("|");
    s.concat(lastVal);
    return s;
  }

  bool sample() {
    int nextVal;
    if (isAnalog) {
      nextVal = analogRead(pin);
    } else {
      nextVal = digitalRead(pin);
    }
    bool changed = ((lastVal != INT_MIN)
        && (applyFactor(lastVal) != applyFactor(nextVal)));
    lastVal = nextVal;
    return true; // change to "changed" to avoid unnecessary data
  }

  int applyFactor(int val) {
    return val * factor;
  }
};

class SensorTestBed {
private:

  SensorData t1[5] = {
    SensorData(A0, "Thermistor A0 sensor", true, 0.036),
    SensorData(A1, "Thermistor A1 sensor", true, 0.036),
    SensorData(A2, "Thermistor A2 sensor", true, 0.036),
    SensorData(A3, "Empty A3 sensor", true, 0.036),
    SensorData(A0, "", true, 1)
  };

  SensorData* getSensors() {
    return t1;
  }

  bool sample() {
    bool changed = false;
    for (SensorData* sensor = getSensors(); !sensor->getName().equals("");
        sensor++) {
      if (sensor->sample()) {
        changed = true;
      }
    }
    return changed;
  }

  void displayValues() {
    bool first = true;
    String s("");
    for (SensorData* sensor = getSensors(); !sensor->getName().equals("");
        sensor++) {
      if (first) {
        first = false;
      } else {
        s.concat("\t");
      }
      s.concat(sensor->getName());
      s.concat("\t");
      s.concat(sensor->getLastVal());
    }
    Serial.println(s);
  }

public:
  SensorTestBed() {
  }

  void firstSample() {
    // publish one right away to verify that things might be working.
    sample();
    displayValues();
  }

  void sampleSensorData() {
    if (sample()) {
      displayValues();
    }
    delay(10000);
  }
};

SensorTestBed sensorTestBed;

void setup() {
  Serial.begin(9600);
  sensorTestBed = SensorTestBed();
  sensorTestBed.firstSample();
}

void loop() {
  sensorTestBed.sampleSensorData();
}

