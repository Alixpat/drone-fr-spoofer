/*
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/*
 * Initial version by Pierre Kancir on May 2020 on funding by Airbot Systems
 * https://github.com/khancyr/droneID_FR
 */
#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>

class droneIDFR {
public:
    droneIDFR() : _droneID("ILLEGAL_DRONE_APPELEZ_POLICE17") {};

    static constexpr uint8_t FRAME_PAYLOAD_LEN_MAX = 251;

    void set_lat_lon(double lat, double lon) {
        _old_latitude = _latitude;
        _old_longitude = _longitude;
        _latitude = (int32_t)(lat * 1e5);
        _longitude = (int32_t)(lon * 1e5);
    }

    void set_altitude(int16_t alt) {
        _altitude = alt;
    }

    void set_heigth(int16_t height) {
        _height = height;
    }

    void set_home_lat_lon(double lat, double lon) {
        _home_latitude = (int32_t)(lat * 1e5);
        _home_longitude = (int32_t)(lon * 1e5);
    }

    void set_ground_speed(uint8_t ground_speed) {
        _ground_speed = ground_speed;
    }

    void set_heading(uint16_t heading) {
        _heading = heading;
    }

    void set_drone_id(const char* id_value) {
        memcpy(_droneID, id_value, TLV_LENGTH[ID_FR]);
    }

    /**
     * Generate the 802.11 Vendor Specific beacon payload
     * @param full_frame beacon frame buffer
     * @param start_from starting offset on the buffer
     * @return total bytes written (end offset)
     */
    uint16_t generate_beacon_frame(uint8_t* full_frame, uint16_t start_from)
    {
        full_frame[start_from] = FRAME_VS;
        start_from++;
        const uint16_t payload_marker = start_from;
        start_from++;

        for (auto i = 0; i < 3; i++) {
            full_frame[start_from] = FRAME_OUI[i];
            start_from++;
        }
        full_frame[start_from] = FRAME_VS_TYPE;
        start_from++;
        const uint8_t payload_size = generate_drone_frame(full_frame, start_from);
        full_frame[payload_marker] = payload_size + 4;  // +OUI + VSTYPE

        return start_from + payload_size;
    }

    uint8_t generate_drone_frame(uint8_t* full_frame, uint16_t start_from) {
        uint8_t count = 0;
        // Protocol version
        full_frame[start_from + count] = PROTOCOL_VERSION;  count++;
        full_frame[start_from + count] = TLV_LENGTH[PROTOCOL_VERSION]; count++;
        full_frame[start_from + count] = FRAME_VS_TYPE; count++;
        // Drone ID FR
        full_frame[start_from + count] = ID_FR; count++;
        full_frame[start_from + count] = TLV_LENGTH[ID_FR]; count++;
        for (auto i = 0; i < TLV_LENGTH[ID_FR]; i++) {
            full_frame[start_from + count] = _droneID[i]; count++;
        }
        // LATITUDE
        full_frame[start_from + count] = LATITUDE; count++;
        full_frame[start_from + count] = TLV_LENGTH[LATITUDE]; count++;
        for (auto i = TLV_LENGTH[LATITUDE] - 1; i >= 0; i--) {
            full_frame[start_from + count] = (get_2_complement(_latitude) >> (8 * i)) & 0xFF; count++;
        }
        // LONGITUDE
        full_frame[start_from + count] = LONGITUDE; count++;
        full_frame[start_from + count] = TLV_LENGTH[LONGITUDE]; count++;
        for (auto i = TLV_LENGTH[LONGITUDE] - 1; i >= 0; i--) {
            full_frame[start_from + count] = (get_2_complement(_longitude) >> (8 * i)) & 0xFF; count++;
        }
        // ALTITUDE
        full_frame[start_from + count] = ALTITUDE; count++;
        full_frame[start_from + count] = TLV_LENGTH[ALTITUDE]; count++;
        for (auto i = TLV_LENGTH[ALTITUDE] - 1; i >= 0; i--) {
            full_frame[start_from + count] = (get_2_complement(_altitude) >> (8 * i)) & 0xFF; count++;
        }
        // HEIGHT
        full_frame[start_from + count] = HEIGTH; count++;
        full_frame[start_from + count] = TLV_LENGTH[HEIGTH]; count++;
        for (auto i = TLV_LENGTH[HEIGTH] - 1; i >= 0; i--) {
            full_frame[start_from + count] = (get_2_complement(_height) >> (8 * i)) & 0xFF; count++;
        }
        // HOME LATITUDE
        full_frame[start_from + count] = HOME_LATITUDE; count++;
        full_frame[start_from + count] = TLV_LENGTH[HOME_LATITUDE]; count++;
        for (auto i = TLV_LENGTH[HOME_LATITUDE] - 1; i >= 0; i--) {
            full_frame[start_from + count] = (get_2_complement(_home_latitude) >> (8 * i)) & 0xFF; count++;
        }
        // HOME LONGITUDE
        full_frame[start_from + count] = HOME_LONGITUDE; count++;
        full_frame[start_from + count] = TLV_LENGTH[HOME_LONGITUDE]; count++;
        for (auto i = TLV_LENGTH[HOME_LONGITUDE] - 1; i >= 0; i--) {
            full_frame[start_from + count] = (get_2_complement(_home_longitude) >> (8 * i)) & 0xFF; count++;
        }
        // GROUND SPEED
        full_frame[start_from + count] = GROUND_SPEED; count++;
        full_frame[start_from + count] = TLV_LENGTH[GROUND_SPEED]; count++;
        full_frame[start_from + count] = _ground_speed; count++;
        // HEADING
        full_frame[start_from + count] = HEADING; count++;
        full_frame[start_from + count] = TLV_LENGTH[HEADING]; count++;
        for (auto i = TLV_LENGTH[HEADING] - 1; i >= 0; i--) {
            full_frame[start_from + count] = (get_2_complement((int16_t)_heading) >> (8 * i)) & 0xFF; count++;
        }
        return count;
    }

private:
    enum DATA_TYPE : uint8_t {
        RESERVED = 0,
        PROTOCOL_VERSION = 1,
        ID_FR = 2,
        ID_ANSI_CTA = 3,
        LATITUDE = 4,
        LONGITUDE = 5,
        ALTITUDE = 6,
        HEIGTH = 7,
        HOME_LATITUDE = 8,
        HOME_LONGITUDE = 9,
        GROUND_SPEED = 10,
        HEADING = 11,
        NOT_DEFINED_END = 12,
    };

    static constexpr uint8_t TLV_LENGTH[] {
        0,  // RESERVED
        1,  // PROTOCOL_VERSION
        30, // ID_FR
        0,  // ID_ANSI_CTA
        4,  // LATITUDE
        4,  // LONGITUDE
        2,  // ALTITUDE
        2,  // HEIGTH
        4,  // HOME_LATITUDE
        4,  // HOME_LONGITUDE
        1,  // GROUND_SPEED
        2,  // HEADING
    };

    static constexpr uint8_t FRAME_VS = 0xDD;
    const uint8_t FRAME_OUI[3] = {0x6A, 0x5C, 0x35};
    static constexpr uint8_t FRAME_VS_TYPE = 1;

    int32_t _latitude = 0;
    int32_t _longitude = 0;
    int16_t _altitude = 0;
    int16_t _height = 0;
    int32_t _home_latitude = 0;
    int32_t _home_longitude = 0;
    uint8_t _ground_speed = 0;
    uint16_t _heading = 0;
    uint8_t _droneID[TLV_LENGTH[ID_FR] + 1];

    int32_t _old_latitude = 0;
    int32_t _old_longitude = 0;

    static inline uint32_t get_2_complement(int32_t value) {
        return value & 0xFFFFFFFF;
    }
    static inline uint16_t get_2_complement(int16_t value) {
        return value & 0xFFFF;
    }
};
