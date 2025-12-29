#include <V2Base.h>
#include <V2Buttons.h>
#include <V2Device.h>
#include <V2LED.h>
#include <V2Link.h>
#include <V2MIDI.h>
#include <V2Music.h>
#include <V2PowerSupply.h>
#include <V2Stepper.h>

V2DEVICE_METADATA("com.versioduo.step", 19, "versioduo:samd:step");

namespace {
  constexpr uint8_t       nSteppers{4};
  V2LED::WS2812           LED(nSteppers + 2, PIN_LED_WS2812, &sercom2, SPI_PAD_0_SCK_1, PIO_SERCOM);
  V2Link::Port            Plug(&SerialPlug, PIN_SERIAL_PLUG_TX_ENABLE);
  V2Link::Port            Socket(&SerialSocket, PIN_SERIAL_SOCKET_TX_ENABLE);
  V2Base::Timer::Periodic Timer(2, 200000);
  V2Base::Analog::ADC     ADC(V2Base::Analog::ADC::getID(PIN_VOLTAGE_SENSE));

  class Stepper : public V2Stepper::Motor {
  public:
    Stepper(const Motor::Config conf, uint8_t index) :
      Motor(conf, &Timer, &SPI, PIN_DRIVER_SELECT + index, PIN_DRIVER_STEP + index),
      _index(index) {}

  private:
    const uint8_t _index;

    void handleMovement(Move move) override {
      switch (move) {
        case Move::Forward:
          LED.setHSV(_index, V2Colour::Cyan, 1, 0.4);
          break;

        case Move::Reverse:
          LED.setHSV(_index, V2Colour::Orange, 1, 0.4);
          break;

        case Move::Stop:
          LED.setHSV(_index, V2Colour::Green, 1, 0.2);
          break;
      }
    }
  } Steppers[nSteppers]{
    Stepper(
      {
        .ampere{0.5},
        .microstepsShift{3},
        .home{.speed{500}, .stall{0.045}},
        .speed{.min{200}, .max{2000}, .accel{8000}},
      },
      0),
    Stepper(
      {
        .ampere{0.5},
        .microstepsShift{3},
        .home{.speed{500}, .stall{0.045}},
        .speed{.min{200}, .max{2000}, .accel{8000}},
      },
      1),
    Stepper(
      {
        .ampere{0.5},
        .microstepsShift{3},
        .home{.speed{500}, .stall{0.045}},
        .speed{.min{200}, .max{2000}, .accel{8000}},
      },
      2),
    Stepper(
      {
        .ampere{0.5},
        .microstepsShift{3},
        .home{.speed{500}, .stall{0.045}},
        .speed{.min{200}, .max{2000}, .accel{8000}},
      },
      3),
  };

  class Power : public V2PowerSupply {
  public:
    Power() : V2PowerSupply({.min{4.8}, .max{26}}) {}

    void begin() {
      pinMode(PIN_DRIVER_ENABLE, OUTPUT);
      digitalWrite(PIN_DRIVER_ENABLE, HIGH);
    }

  private:
    float handleMeasurement() override {
      // A voltage 10/100k divider.
      return 36.f * ADC.readChannel(V2Base::Analog::ADC::getChannel(PIN_VOLTAGE_SENSE));
    }

    void handleOn() override {
      digitalWrite(PIN_DRIVER_ENABLE, LOW);
    }

    void handleOff() override {
      digitalWrite(PIN_DRIVER_ENABLE, HIGH);
    }

    void handleNotify(float voltage) override {
      // Power interruption, or commands without a power connection show yellow LEDs.
      if (voltage < config.min) {
        LED.splashHSV(0.5, V2Colour::Yellow, 1, 0.5);
        return;
      }

      // Over-voltage shows red LEDs.
      if (voltage > config.max) {
        LED.splashHSV(0.5, V2Colour::Red, 1, 1);
        return;
      }

      // The number of green LEDs shows the voltage.
      const float fraction{voltage / (float)config.max};
      const float n{ceil((float)nSteppers * fraction)};
      LED.splashHSV(0.5, 0, n, V2Colour::Green, 1, 0.5);
    }
  } Power;

  class Device : public V2Device {
  public:
    Device() : V2Device() {
      metadata.vendor      = "Versio Duo";
      metadata.product     = "V2 step";
      metadata.description = "Stepper Motor Controller";
      metadata.home        = "https://versioduo.com/#step";

      system.download  = "https://versioduo.com/download";
      system.configure = "https://versioduo.com/configure";
    }

  private:
    struct {
      int16_t pitchBend;
      float   rotate;
    } _steppers[nSteppers]{};

    void handleReset() override {
      LED.reset();

      for (uint8_t i = 0; i < nSteppers; i++) {
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

      if (!continuous)
        for (auto& s : Steppers)
          s.reset();

      return true;
    }

    void play(uint8_t channel, uint8_t note, uint8_t velocity) {
      if (channel >= nSteppers)
        return;

      if (note < V2MIDI::C(3) || note > V2MIDI::B(4))
        return;

      if (velocity == 0) {
        Steppers[channel].stop();
        return;
      }

      if (!power())
        return;

      const float range{(float)(note - V2MIDI::C(3)) / 23.f};
      const float speed{(float)velocity / 127.f};
      Steppers[channel].setPosition(range * 200.f * 3.f, powf(speed, 3));
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
      if (channel >= nSteppers)
        return;

      _steppers[channel].pitchBend = value;

      // Negative range, the chosen exponent needs to preserve the sign.
      const float fraction{(float)value / (value < 0 ? 8192.f : 8191.f)};
      _steppers[channel].rotate = powf(fraction, 3);

      if (!power())
        return;

      Steppers[channel].rotate(_steppers[channel].rotate);
    }

    void handleSystemReset() override {
      reset();
    }

    void exportSystem(JsonObject json) override {
      JsonObject jsonPower       = json["power"].to<JsonObject>();
      jsonPower["voltage"]       = serialized(String(Power.getVoltage(), 1));
      jsonPower["interruptions"] = Power.getInterruptions();
    }

    void exportInput(JsonObject json) override {
      JsonArray jsonChannels = json["channels"].to<JsonArray>();
      for (uint8_t ch = 0; ch < nSteppers; ch++) {
        JsonObject jsonChannel = jsonChannels.add<JsonObject>();
        jsonChannel["number"]  = ch;

        {
          JsonObject jsonPitchbend = jsonChannel["pitchbend"].to<JsonObject>();
          jsonPitchbend["value"]   = _steppers[ch].pitchBend;
        }

        {
          JsonObject jsonChromatic = jsonChannel["chromatic"].to<JsonObject>();
          jsonChromatic["start"]   = V2MIDI::C(3);
          jsonChromatic["count"]   = 24;
        }
      }
    }
  } Device;

  // Dispatch MIDI packets.
  class MIDI {
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

  // Dispatch Link packets.
  class Link : public V2Link {
  public:
    Link() : V2Link(&Plug, &Socket) {
      Device.link = this;
    }

  private:
    V2MIDI::Packet _midi{};

    // Receive a host event from our parent device.
    void receivePlug(V2Link::Packet* packet) override {
      if (packet->getType() == V2Link::Packet::Type::MIDI) {
        packet->receive(&_midi);
        Device.dispatch(&Plug, &_midi);
      }
    }

    // Forward children device events to the host.
    void receiveSocket(V2Link::Packet* packet) override {
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

  class Button : public V2Buttons::Button {
  public:
    Button() : V2Buttons::Button(&_config, PIN_BUTTON) {}

  private:
    const V2Buttons::Config _config{.clickUsec{200 * 1000}, .holdUsec{500 * 1000}};

    void handleClick(uint8_t count) override {
      Device.reset();
    }

    void handleHold(uint8_t count) override {
      LED.setHSV(nSteppers + 0, V2Colour::Cyan, 1, 0.25);
      LED.setHSV(nSteppers + 1, V2Colour::Cyan, 1, 0.25);
    }

    void handleRelease() override {}
  } Button;
}

void setup() {
  Serial.begin(9600);
  SPI.begin();

  LED.begin();
  LED.setMaxBrightness(0.5);

  Plug.begin();
  Socket.begin();
  Device.link = &Link;

  // Set the SERCOM interrupt priority, it requires a stable ~300 kHz interrupt
  // frequency. This needs to be after begin().
  setSerialPriority(&SerialPlug, 2);
  setSerialPriority(&SerialSocket, 2);

  Power.begin();
  for (auto& s : Steppers)
    s.begin();

  // The priority needs to be lower than the SERCOM priorities.
  Timer.begin([]() {
    for (auto& s : Steppers)
      s.tick();
  });
  Timer.setPriority(3);

  ADC.begin();
  ADC.addChannel(V2Base::Analog::ADC::getChannel(PIN_VOLTAGE_SENSE));

  Device.begin();
  Button.begin();
  Device.reset();
}

void loop() {
  for (auto& s : Steppers)
    s.loop();

  LED.loop();
  MIDI.loop();
  Link.loop();
  V2Buttons::loop();
  Power.loop();
  Device.loop();

  if (Link.idle() && Device.idle())
    Device.sleep();
}
