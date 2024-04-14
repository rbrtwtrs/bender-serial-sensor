//**************************************************************************************************
// 
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
#include "bender_serial_sensor.h"

namespace bender_serial_sensor {

void BenderSerialSensor::setup() {
    // Custom setup can be done here
    ESP_LOGD("custom", "BenderSerialSensor setup complete.");
}

void BenderSerialSensor::loop() {
    while (available()) {
        std::string line = read_line();
        if (!line.empty() && validate_checksum(line)) {
            process_line(line);
        } else {
            ESP_LOGW("custom", "Checksum validation failed or line is empty: %s", line.c_str());
        }
    }
}

bool BenderSerialSensor::validate_checksum(const std::string &line) {
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

void BenderSerialSensor::process_line(const std::string &line) {
    size_t asterisk = line.find('*');
    if (asterisk == std::string::npos) return;

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
    }
}

std::string BenderSerialSensor::read_line() {
    std::string line;
    char c;
    while (available() && (c = read()) != '\n') {
        if (c != '\r') line += c;
    }
    return line;
}

} // namespace bender_serial_sensor
