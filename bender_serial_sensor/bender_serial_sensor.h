#include "esphome.h" // Required for all ESPHome components
#include <sstream> // std::stringstream
#include <vector> 
#include <string>
#include <algorithm>  // std::find
#include <numeric> // Required for std::accumulate
//**************************************************************************************************
// MyCustomSensor
//
// This is a custom component that reads data from a serial interface and parses it to extract
// information about Bender units. The component will publish the status and resistance of each
// Bender unit as sensors. The component will also validate the checksum of the received data.
// The component is designed to be used with the UARTDevice component to read data from a serial
// interface. 
//
// Serial Interface Description
//
// Checksum Calculation:
// - Each message concludes with a CheckSum8 Modulo 256 checksum for data integrity. The checksum is calculated by 
//   summing the ASCII values of all characters in the message preceding the asterisk ('*'). 
//   This sum is then truncated to the two least significant digits and represented in hexadecimal format.
//   Example: BGF 0 1 100024 7291 72 47890 *7F\n
//
// Message Frequency:
// - Bender units report their status approximately twice per second via the serial interface.
//
// Message Formatting:
// - Messages are structured with fields separated by spaces or tabs. The end of a message string 
//   is marked by a null character ('\0'), and typically includes a carriage return ('\r') and/or 
//   a newline ('\n'). The number of spaces or tabs between fields can vary and should be ignored by the parsing routine.
//
// Data Formats in Messages:
// - Bender Data Format: Each Bender unit's data is encapsulated in a string format as follows:
//   BGF n s ppp dcc dcppt gfr *A6\n
//   Example: BGF 0 1 100052 18496 184 6859 *BD\n
//   Where:
//   - n: Single digit (0-3) indicating the Bender number.
//   - s: Single digit status (1 for good, 0 for fault).
//   - ppp: Period of the last PWM cycle in microseconds. Zero indicates a DC level, signifying a short.
//   - dcc: Duration of the high duty cycle pulse within the period, in microseconds.
//   - dcppt: Duty cycle period to total period ratio, expressed in parts per thousand.
//   - gfr: Calculated ground fault resistance in kilo-ohms (kOhms).
//   - Checksum: Two hexadecimal digits preceded by an asterisk. It may be preceded by one or two spaces following the resistance value.
//   - Messages end with a carriage return and/or line feed for readability and are terminated with a null character.
//
// - Analog Input String Format: Generated every 500 milliseconds to report analog inputs:
//   AIN n ain0 ain1 ain2 … *BC\n
//   Example: AIN 4 0 0 0 0 *8C\n 
//   Where:
//   - n: Number of analog inputs being reported.
//   - ain0, ain1, etc.: Represent the analog value on each line in bits (5 volts equals 1023).
//
// - Arbitrary Message Strings: Used primarily for fault indication within regular BGF messages. Only sent when a fault is detected.
//   GFS BENDER 2: 10Hz -- FAULT INDICATED -- 50228 kOhms *37\n
//   Where:
//   - All characters between "GFS" and the asterisk are part of an arbitrary message intended for display to the user. 
//     These can include any ASCII character except for the asterisk. In the example, "2" identifies the referenced Bender unit.
///
// Typicaly a message block of messages will be sent every 500mSec. The message block would look like the following:
// BGF 0 1 100052 18500 184 6859 *AF\n
// BGF 1 1 100056 13508 135 11505 *D3\n
// BGF 2 0 0 0 0 0 *D1\n
// GFS BENDER 2: FLAT LINE LOW -- SHORT -- FAULT INDICATED  *02\n
// AIN 4 0 0 0 0 *8C\n
//
//**************************************************************************************************

class BenderSerialSensor : public Component, public esphome::uart::UARTDevice {
 public:
  std::map<int, std::vector<sensor::Sensor *>> bender_sensors;

  BenderSerialSensor() = default;

  void setup() override {
    ESP_LOGD("custom", "BenderSerialSensor setup complete.");
  }

  void loop() override {
    static std::string line;
    while (available()) {
      char c = read();
      if (c == '\n') {
        if (!line.empty() && validate_checksum(line)) {
          process_line(line);
        } else {
          ESP_LOGW("custom", "Checksum validation failed or line is empty: %s", line.c_str());
        }
        line.clear();
      } else if (c != '\r') {
        line += c;
      }
    }
  }

bool validate_checksum(const std::string &line) {
    size_t asterisk = line.find('*');
    if (asterisk == std::string::npos) return false;

    const std::string data = line.substr(0, asterisk);
    const std::string checksum_received = line.substr(asterisk + 1);

    unsigned int calculated_checksum = 0;
    for (char ch : data) {
        calculated_checksum += static_cast<unsigned char>(ch);
    }
    calculated_checksum &= 0xFF; // Keep only the lower byte

    char calculated_hex[3];
    snprintf(calculated_hex, sizeof(calculated_hex), "%02X", calculated_checksum);

    return checksum_received == calculated_hex;
}


void process_line(const std::string &line) {
    auto start_time = millis();
    size_t asterisk = line.find('*');
    if (asterisk == std::string::npos) return; // Handle error if needed

    std::string data = line.substr(0, asterisk);
    std::vector<std::string> parts;
    std::stringstream ss(data);
    std::string item;
    while (std::getline(ss, item, ' ')) {
        if (!item.empty()) parts.push_back(item);
    }

    if (parts.empty()) return;

    if (parts[0] == "BGF" && parts.size() >= 7) {
        int bender_id = std::stoi(parts[1]);
        if (bender_sensors.count(bender_id) > 0) {
            bender_sensors[bender_id][0]->publish_state(std::stoi(parts[2])); // Status
            bender_sensors[bender_id][1]->publish_state(std::stof(parts[6])); // Resistance
        }
    } else if (parts[0] == "AIN" && parts.size() >= 2) {
        int input_count = std::stoi(parts[1]);
        // Handle analog input data (simply logging here for demonstration)
        for (int i = 0; i < input_count && (i+2) < parts.size(); i++) {
            ESP_LOGD("custom", "Analog Input %d: %s", i, parts[i+2].c_str());
        }
    } else if (parts[0] == "GFS") {
        // Efficient concatenation and logging for fault messages
        std::string fault_message;
        fault_message.reserve(100); // Adjust size based on expected message length to avoid reallocations
        for (auto it = parts.begin() + 1; it != parts.end(); ++it) {
            if (it != parts.begin() + 1) fault_message += " ";
            fault_message += *it;
        }
        ESP_LOGW("custom", "Fault Message: %s", fault_message.c_str());
    } else {
        ESP_LOGW("custom", "Unhandled message type or not enough parts: %s", line.c_str());
    }
    auto end_time = millis();
    ESP_LOGD("custom", "Processing time: %lu ms", end_time - start_time);
}


};
