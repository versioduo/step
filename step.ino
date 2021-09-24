// © Kay Sievers <kay@vrfy.org>, 2020-2021
// SPDX-License-Identifier: Apache-2.0

#include <V2Base.h>
#include <V2Color.h>
#include <V2Device.h>
#include <V2LED.h>
#include <V2Link.h>
#include <V2MIDI.h>
#include <V2Music.h>
#include <V2PowerSupply.h>
#include <V2Stepper.h>

V2DEVICE_METADATA("com.versioduo.step", 8, "versioduo:samd:step");

static constexpr uint8_t n_steppers = 4;
static V2LED::WS2812 LED(n_steppers, PIN_LED_WS2812, &sercom2, SPI_PAD_0_SCK_1, PIO_SERCOM);
static V2Link::Port Plug(&SerialPlug);
static V2Link::Port Socket(&SerialSocket);
static V2Base::Timer::Periodic Timer(2, 200000);

static class Stepper : public V2Stepper::Motor {
public:
  Stepper(const Motor::Config conf, uint8_t index) :
    Motor(conf, &Timer, &SPI, PIN_DRIVER_SELECT + index, PIN_DRIVER_STEP + index),
    _index(index) {}

private:
  const uint8_t _index;

  void handleMovement(Move move) override {
    switch (move) {
      case Move::Forward:
        LED.setHSV(_index, V2Color::Blue, 1, 0.5);
        break;

      case Move::Reverse:
        LED.setHSV(_index, V2Color::Orange, 1, 0.5);
        break;

      case Move::Stop:
        LED.setHSV(_index, V2Color::Green, 1, 0.25);
        break;
    }
  }
} Steppers[n_steppers]{
  Stepper(
    {
      .ampere{0.5},
      .microsteps_shift{3},
      .home{.speed{500}, .stall{0.045}},
      .speed{.min{200}, .max{2000}, .accel{8000}},
    },
    0),
  Stepper(
    {
      .ampere{0.5},
      .microsteps_shift{3},
      .home{.speed{500}, .stall{0.045}},
      .speed{.min{200}, .max{2000}, .accel{8000}},
    },
    1),
  Stepper(
    {
      .ampere{0.5},
      .microsteps_shift{3},
      .home{.speed{500}, .stall{0.045}},
      .speed{.min{200}, .max{2000}, .accel{8000}},
    },
    2),
  Stepper(
    {
      .ampere{0.5},
      .microsteps_shift{3},
      .home{.speed{500}, .stall{0.045}},
      .speed{.min{200}, .max{2000}, .accel{8000}},
    },
    3),
};

static class Power : public V2PowerSupply {
public:
  Power() : V2PowerSupply({.min{6}, .max{26}}, PIN_VOLTAGE_SENSE) {}

  void begin() {
    pinMode(PIN_DRIVER_ENABLE, OUTPUT);
    digitalWrite(PIN_DRIVER_ENABLE, HIGH);
  }

private:
  void handleOn() override {
    digitalWrite(PIN_DRIVER_ENABLE, LOW);
  }

  void handleOff() override {
    digitalWrite(PIN_DRIVER_ENABLE, HIGH);
  }

  void handleNotify(float voltage) override {
    // Power-loss or commands without a power connection show yellow LEDs.
    if (voltage < config.min) {
      LED.splashHSV(0.5, V2Color::Yellow, 1, 0.5);
      return;
    }

    // Over-voltage shows red LEDs.
    if (voltage > config.max) {
      LED.splashHSV(0.5, V2Color::Red, 1, 1);
      return;
    }

    // The number of green LEDs shows the voltage.
    float fraction = voltage / (float)config.max;
    uint8_t n      = ceil((float)n_steppers * fraction);
    LED.splashHSV(0.5, n, V2Color::Green, 1, 0.5);
  }
} Power;

static class Device : public V2Device {
public:
  Device() : V2Device() {
    metadata.vendor      = "Versio Duo";
    metadata.product     = "step";
    metadata.description = "Stepper Motor Controller";
    metadata.home        = "https://versioduo.com/#step";

    system.download  = "https://versioduo.com/download";
    system.configure = "https://versioduo.com/configure";
  }

private:
  struct {
    float rotate;
  } _steppers[n_steppers]{};

  void handleReset() override {
    digitalWrite(PIN_LED_ONBOARD, LOW);
    LED.reset();

    for (uint8_t i = 0; i < n_steppers; i++) {
      _steppers[i] = {};
      Steppers[i].reset();
    }

    Power.off();
  }

  void allNotesOff() {
    reset();
  }

  bool power() {
    bool continuous;
    if (!Power.on(continuous))
      return false;

    if (!continuous) {
      for (uint8_t i = 0; i < n_steppers; i++)
        Steppers[i].reset();
    }

    return true;
  }

  void play(uint8_t channel, uint8_t note, uint8_t velocity) {
    if (channel >= n_steppers)
      return;

    if (note < V2MIDI::C(3) || note > V2MIDI::B(4))
      return;

    if (velocity == 0) {
      Steppers[channel].stop();
      return;
    }

    if (!power())
      return;

    const float range = (float)(note - V2MIDI::C(3)) / 23.f;
    const float speed = (float)velocity / 127.f;
    Steppers[channel].position(range * 200.f * 3.f, powf(speed, 3));
  }

  void handleNote(uint8_t channel, uint8_t note, uint8_t velocity) override {
    play(channel, note, velocity);
  }

  void handleNoteOff(uint8_t channel, uint8_t note, uint8_t velocity) override {
    play(channel, note, 0);
  }

  void handleControlChange(uint8_t channel, uint8_t controller, uint8_t value) override {
    if (channel != 0)
      return;

    switch (controller) {
      case V2MIDI::CC::AllSoundOff:
      case V2MIDI::CC::AllNotesOff:
        allNotesOff();
        break;
    }
  }

  void handlePitchBend(uint8_t channel, int16_t value) override {
    if (channel >= n_steppers)
      return;

    // Negative range, the chosen exponent needs to preserve the sign.
    const float fraction      = (float)value / (value < 0 ? 8192.f : 8191.f);
    _steppers[channel].rotate = powf(fraction, 3);

    if (!power())
      return;

    Steppers[channel].rotate(_steppers[channel].rotate);
  }

  void handleSystemReset() override {
    reset();
  }

  void exportSystem(JsonObject json) override {
    JsonObject json_power       = json.createNestedObject("power");
    json_power["volt"]          = truncf(Power.getVoltage() * 10.f) / 10.f;
    json_power["interruptions"] = Power.getInterruptions();
  }

  void exportInput(JsonObject json) override {
    JsonArray json_channels = json.createNestedArray("channels");
    for (uint8_t ch = 0; ch < n_steppers; ch++) {
      JsonObject json_channel = json_channels.createNestedObject();
      json_channel["number"]  = ch;

      {
        JsonObject json_pitchbend = json_channel.createNestedObject("pitchbend");
        json_pitchbend["value"]   = (int16_t)(_steppers[ch].rotate * (_steppers[ch].rotate < 0 ? 8192.f : 8191.f));
      }

      {
        JsonObject json_chromatic = json_channel.createNestedObject("chromatic");
        json_chromatic["start"]   = V2MIDI::C(3);
        json_chromatic["count"]   = 24;
      }
    }
  }
} Device;

// Dispatch MIDI packets
static class MIDI {
public:
  void loop() {
    if (!Device.usb.midi.receive(&_midi))
      return;

    if (_midi.getPort() == 0) {
      Device.dispatch(&Device.usb.midi, &_midi);

    } else {
      _midi.setPort(_midi.getPort() - 1);
      Socket.send(&_midi);
    }
  }

private:
  V2MIDI::Packet _midi{};
} MIDI;

// Dispatch Link packets
static class Link : public V2Link {
public:
  Link() : V2Link(&Plug, &Socket) {}

private:
  V2MIDI::Packet _midi{};

  // Receive a host event from our parent device
  void receivePlug(V2Link::Packet *packet) override {
    if (packet->getType() == V2Link::Packet::Type::MIDI) {
      packet->receive(&_midi);
      Device.dispatch(&Plug, &_midi);
    }
  }

  // Forward children device events to the host
  void receiveSocket(V2Link::Packet *packet) override {
    if (packet->getType() == V2Link::Packet::Type::MIDI) {
      uint8_t address = packet->getAddress();
      if (address == 0x0f)
        return;

      if (Device.usb.midi.connected()) {
        packet->receive(&_midi);
        _midi.setPort(address + 1);
        Device.usb.midi.send(&_midi);
      }
    }
  }
} Link;

void TC2_Handler() {
  for (uint8_t i = 0; i < n_steppers; i++)
    Steppers[i].tick();

  Timer.clear();
}

void setup() {
  Serial.begin(9600);
  SPI.begin();

  LED.begin();
  LED.setMaxBrightness(0.5);
  Plug.begin();
  Socket.begin();

  // Set the SERCOM interrupt priority, it requires a stable ~300 kHz interrupt
  // frequency. This needs to be after begin().
  setSerialPriority(&SerialPlug, 2);
  setSerialPriority(&SerialSocket, 2);

  Power.begin();
  for (uint8_t i = 0; i < n_steppers; i++)
    Steppers[i].begin();

  // This needs to be lower than the SERCOM priorities.
  Timer.begin();
  Timer.setPriority(3);

  Device.begin();
  Device.reset();
}

void loop() {
  for (uint8_t i = 0; i < n_steppers; i++)
    Steppers[i].loop();

  LED.loop();
  MIDI.loop();
  Link.loop();
  Power.loop();
  Device.loop();

  if (Link.idle() && Device.idle())
    Device.sleep();
}
