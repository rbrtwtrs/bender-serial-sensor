#ifndef BENDER_SERIAL_SENSOR_H
#define BENDER_SERIAL_SENSOR_H

#include "esphome.h"
#include <string>
#include <vector>
#include <map>

namespace bender_serial_sensor {

class BenderSerialSensor : public Component, public esphome::uart::UARTDevice {
 public:
  std::map<int, std::vector<sensor::Sensor *>> bender_sensors;

  BenderSerialSensor() = default;

  void setup() override;
  void loop() override;

 private:
  bool validate_checksum(const std::string &line);
  void process_line(const std::string &line);
  std::string read_line();
};

} // namespace bender_serial_sensor

#endif // BENDER_SERIAL_SENSOR_H
