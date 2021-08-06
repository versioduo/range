// © Kay Sievers <kay@vrfy.org>, 2020-2021
// SPDX-License-Identifier: Apache-2.0

#include <V2Device.h>
#include <V2LED.h>
#include <V2Link.h>
#include <V2MIDI.h>
#include <V2VL53LX.h>
#include <Wire.h>

V2DEVICE_METADATA("com.versioduo.range", 17, "versioduo:samd:range");

static V2LED LED(1, PIN_LED_WS2812, &sercom2, SPI_PAD_0_SCK_1, PIO_SERCOM);
static V2Link::Port Plug(&SerialPlug);
static V2Link::Port Socket(&SerialSocket);

static class Device : public V2Device {
public:
  Device() : V2Device() {
    metadata.vendor      = "Versio Duo";
    metadata.product     = "range";
    metadata.description = "Proximity Sensor";
    metadata.home        = "https://versioduo.com/#range";

    system.download  = "https://versioduo.com/download";
    system.configure = "https://versioduo.com/configure";

    configuration = {.magic{0x9e020000 | usb.pid}, .size{sizeof(config)}, .data{&config}};
  }

  // Config, written to EEPROM
  struct {
    uint8_t channel;
    uint8_t controller;
    V2VL53LX::Configuration range;
  } config{.channel{},
           .controller{V2MIDI::CC::GeneralPurpose1},
           .range{.n_steps{128}, .min{10}, .max{500}, .detect{600}, .alpha{0.7}, .lag{0.008}}};

  void reset() {
    digitalWrite(PIN_LED_ONBOARD, LOW);
    LED.reset();
  }

  void allNotesOff() {
    reset();
  }

private:
  bool handleSend(V2MIDI::Packet *midi) override {
    usb.midi.send(midi);
    Plug.send(midi);
    return true;
  }

  void handleControlChange(uint8_t channel, uint8_t controller, uint8_t value) override {
    switch (controller) {
      case V2MIDI::CC::AllSoundOff:
      case V2MIDI::CC::AllNotesOff:
        allNotesOff();
        break;
    }
  }

  void handleSystemReset() override {
    reset();
  }

  void exportSettings(JsonArray json) override {
    JsonObject json_midi = json.createNestedObject();
    json_midi["type"]    = "midi";
    json_midi["channel"] = "midi.channel";

    // The object in the configuration record.
    JsonObject json_configuration = json_midi.createNestedObject("configuration");
    json_configuration["path"]    = "midi";
    json_configuration["field"]   = "channel";
  }

  void importConfiguration(JsonObject json) override {
    JsonObject json_midi = json["midi"];
    if (json_midi) {
      if (!json_midi["channel"].isNull()) {
        uint8_t channel = json_midi["channel"];

        if (channel < 1)
          config.channel = 0;
        else if (channel > 16)
          config.channel = 15;
        else
          config.channel = channel - 1;
      }
    }

    if (!json["controller"].isNull()) {
      config.controller = json["controller"];

      if (config.controller > 127)
        config.controller = 127;
    }

    JsonObject json_range = json["range"];
    if (json_range) {
      uint16_t min = json_range["min"];
      if (min == 0 || min > 3000)
        min = 10;

      uint16_t max = json_range["max"];
      if (max < min || max > 3000)
        max = 500;

      uint16_t detect = json_range["detect"];
      if (detect < max || max > 3000)
        detect = 600;

      config.range.min    = min;
      config.range.max    = max;
      config.range.detect = detect;
    }
  }

  void exportConfiguration(JsonObject json) override {
    json["#midi"]         = "The MIDI settings";
    JsonObject json_midi  = json.createNestedObject("midi");
    json_midi["#channel"] = "The channel to send notes and control values to";
    json_midi["channel"]  = config.channel + 1;

    json["#controller"] = "The MIDI controller for pressure values";
    json["controller"]  = config.controller;

    json["#range"]        = "The proximity sensor's parameters";
    JsonObject json_range = json.createNestedObject("range");
    json_range["#min"]    = "The minimum measured distance in millimeters, value = 1";
    json_range["min"]     = config.range.min;
    json_range["#max"]    = "The maximum measured distance in millimeters, value = 127";
    json_range["max"]     = config.range.max;
    json_range["#detect"] = "The maximum accepted distance in millimeters, value = 127";
    json_range["detect"]  = config.range.detect;
  }

  void exportOutput(JsonObject json) override {
    json["channel"] = config.channel;

    JsonArray json_controllers = json.createNestedArray("controllers");
    JsonObject json_controller = json_controllers.createNestedObject();
    json_controller["name"]    = "Proximity";
    json_controller["number"]  = config.controller;
  }
} Device;

// Range/Proximity sensor
static class ProximitySensor : public V2VL53LX::Driver {
public:
  constexpr ProximitySensor(uint8_t pin_reset, uint8_t pin_interrupt) :
    V2VL53LX::Driver(&Device.config.range, &Wire, pin_reset, pin_interrupt){};

  void reset() {
    disable();
    enable();
  }

private:
  V2MIDI::Packet _midi{};

  void handleUpdate(uint32_t step, float fraction, float distance, uint8_t count) override {
    LED.setBrightness(0, fraction);
    Device.send(_midi.setControlChange(Device.config.channel, Device.config.controller, step));
  }
} Range = ProximitySensor(PIN_PROXIMITY_SHUTDOWN, PIN_PROXIMITY_INTERRUPT);

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

void setup() {
  Serial.begin(9600);

  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeout(1);

  LED.begin();
  LED.setMaxBrightness(0.5);
  Plug.begin();
  Socket.begin();

  // Set the SERCOM interrupt priority, it requires a stable ~300 kHz interrupt
  // frequency. This needs to be after begin().
  setSerialPriority(&SerialPlug, 2);
  setSerialPriority(&SerialSocket, 2);

  Range.begin();
  Range.reset();
  Device.begin();
  Device.reset();
}

void loop() {
  LED.loop();
  MIDI.loop();
  Link.loop();
  Range.loop();
  Device.loop();

  if (Link.idle() && Device.idle())
    Device.sleep();
}
