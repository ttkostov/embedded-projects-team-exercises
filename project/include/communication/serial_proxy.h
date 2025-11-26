#pragma once
#include <Arduino.h>

class SerialProxy
{
public:
  static SerialProxy &instance()
  {
    static SerialProxy inst;
    return inst;
  }

  void begin(HardwareSerial &usbPort, HardwareSerial &espPort, unsigned long baud = 9600)
  {
    usb = &usbPort;
    esp = &espPort;
    baud_ = baud;

    usb->begin(baud_);
    esp->begin(baud_);
  }

  void print(const String &s)
  {
    if (forwardToUSB && usb)
      usb->print(s);
    if (forwardToESP && esp)
      esp->print(s);
  }

  void println(const String &s)
  {
    if (forwardToUSB && usb)
      usb->println(s);
    if (forwardToESP && esp)
      esp->println(s);
  }

  String readLine()
  {
    // First check USB
    if (usb && usb->available())
      return usb->readStringUntil('\n');

    // Then ESP
    if (esp && esp->available())
      return esp->readStringUntil('\n');

    return "";
  }

  void setForwardUSB(bool v) { forwardToUSB = v; }
  void setForwardESP(bool v) { forwardToESP = v; }

private:
  SerialProxy() {}

  HardwareSerial *usb = nullptr;
  HardwareSerial *esp = nullptr;

  bool forwardToUSB = true;
  bool forwardToESP = true;
  unsigned long baud_;
};
