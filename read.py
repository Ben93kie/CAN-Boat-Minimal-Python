import time
import re
import struct
from threading import Thread
from serial import Serial
from datetime import datetime, timedelta
import numpy as np

class Furuno(Thread):
    def __init__(self, port):
        Thread.__init__(self)
        self._port = port
        self._stream = Serial(self._port, 115200, timeout=1)

        # Switch to RAW mode
        self.set_raw_mode()

        self.data_dict = {}

        self.field_decoders = {
            "Heading": self.decode_angle_u16_field,
            "Deviation": self.decode_angle_i16_field,
            "Variation": self.decode_angle_i16_field,
            "Reference": self.decode_lookup_field,
            "SID": self.decode_uint8_field,
            "Date": self.decode_date_field,
            "Time": self.decode_time_field,
            # "Latitude": self.decode_latitude_i64_field,
            "Latitude": self.decode_latitude_i32_field,
            "Longitude": self.decode_longitude_i32_field,
            # "Company": self.decode_company_field,
            "A": self.decode_uint8_field,
            "B": self.decode_uint8_field,
            "Yaw": self.decode_angle_i16_field,
            "Pitch": self.decode_angle_i16_field,
            "Roll": self.decode_angle_i16_field,
            # "Industry": self.decode_industry_field,
        }

        self.prev_position_timestamp = None
        self.prev_attitude_timestamp = None
        self.prev_attitude2_timestamp = None

        self.field_list_gps = [
            {"name": "Latitude", "length": 4},
            {"name": "Longitude", "length": 4},
        ]
        self.field_list_attitude = [
            {"name": "SID", "length": 1},
            {"name": "Yaw", "length": 2},
            {"name": "Pitch", "length": 2},
            {"name": "Roll", "length": 2},
            # {"name": "Reserved", "length": 1},
        ]
        self.field_list_heading = [
            {"name": "SID", "length": 1},
            {"name": "Heading", "length": 2},
            # {"name": "Deviation", "length": 2},
            # {"name": "Variation", "length": 2},
            # {"name": "Reference", "length": 1},
        ]

    def set_raw_mode(self):
        command = "YDNU MODE RAW\r\n"
        self._stream.write(command.encode())
        time.sleep(1)

    def decode_uint8_field(self, data):
        return data[0]

    def decode_date_field(self, data):
        days_since_1970 = struct.unpack('<H', bytes(data[:2]))[0]
        return time.strftime('%Y-%m-%d', time.gmtime(days_since_1970 * 86400))

    def decode_time_field(self, data):
        seconds_since_midnight = struct.unpack('<I', bytes(data[:4]))[0] * 0.0001
        hours = int(seconds_since_midnight // 3600)
        minutes = int((seconds_since_midnight % 3600) // 60)
        seconds = seconds_since_midnight % 60
        return f"{hours:02}:{minutes:02}:{seconds:06.3f}"

    # def decode_latitude_i64_field(self, data):
    #     raw_value = struct.unpack('<q', bytes(data[:8]))[0]
    #     return raw_value * 1e-16
    def decode_longitude_i32_field(self, data):
        raw_value = struct.unpack('<i', bytes(data[:4]))[0]
        return raw_value * 1e-7

    def decode_latitude_i32_field(self, data):
        raw_value = struct.unpack('<i', bytes(data[:4]))[0]
        return raw_value * 1e-7


    def decode_company_field(self, data):
        return struct.unpack('<H', bytes(data[:2]))[0]

    def decode_angle_i16_field(self, data):
        if len(data) != 2:
            print(f"Warning: Insufficient data to decode field. Expected 2 bytes, got {len(data)} bytes.")
            return None
        # Interpret the data as a signed 16-bit integer
        raw_value = struct.unpack('<h', bytes(data))[0]
        # Multiply by the resolution
        return raw_value * 1e-4

    def decode_angle_u16_field(self, data):
        raw_value = struct.unpack('<H', bytes(data))[0]
        return raw_value * 1e-4

    def decode_lookup_field(self, data):
        return struct.unpack('<H', bytes(data))[0]

    def decode_speed_u16_cm_field(self, data):
        raw_value = struct.unpack('<H', bytes(data))[0]
        # Convert cm/s to m/s or knots as required
        return raw_value / 100  # Example: cm/s to m/s

    def decode_manufacturer_field(self, data):
        # Extract 11 bits for manufacturer
        value = (data[0] << 3) | (data[1] >> 5)
        return value

    def decode_industry_field(self, data):
        # Extract 3 bits for industry
        value = data[1] & 0b00000111
        return value

    def get_pgn_from_message(self, msgid):
        # Shift 8 bits to discard the source address
        shifted_msgid = msgid >> 8
        # Mask to get 18 bits representing the PGN
        pgn = shifted_msgid & 0x3FFFF
        return pgn

    def parse_raw_message(self, message):
        parts = message.split()

        # Extracting parts based on the expected structure
        timestamp = parts[0]
        direction = parts[1]
        msgid = int(parts[2], 16)
        pgn = self.get_pgn_from_message(msgid)
        data_bytes = parts[3:]
        int_data_bytes = [int(byte, 16) for byte in data_bytes]

        if pgn == 129025:  # Position, Rapid Update

            start = 0
            # field_list = [
            #     {"name": "Latitude", "length": 4},
            #     {"name": "Longitude", "length": 4},
            # ]
            expected_byte_count = sum(field['length'] for field in self.field_list_gps)
            if len(int_data_bytes) < expected_byte_count:
                print(
                    f"Warning: Message has {len(int_data_bytes)} bytes, but expected {expected_byte_count}. Raw data: {message}")

            for field in self.field_list_gps:
                field_name = field["name"]
                field_length = field["length"]
                decoder = self.field_decoders.get(field_name)
                if not decoder:
                    # print(f"No decoder found for field {field_name}.")
                    continue
                if not field_name == 'SID':
                    try:
                        self.data_dict[field_name] = decoder(int_data_bytes[start:start + field_length])
                        # print(field_name, self.data_dict['data'][field_name])
                    except Exception as e:
                        print(
                            f"Could not decode field {field_name}. Error: {e}. Data: {int_data_bytes[start:start + field_length]}")

                start += field_length
        elif pgn == 129026:  # COG & SOG, Rapid Update
            start = 0
            field_list_cog_sog = [
                {"name": "SID", "length": 1},
                {"name": "COG Reference", "length": 1},  # Combined with Reserved field
                {"name": "COG", "length": 2},
                {"name": "SOG", "length": 2},
                # Add other fields as needed
            ]
            expected_byte_count = sum(field['length'] for field in field_list_cog_sog)
            if len(int_data_bytes) < expected_byte_count:
                print(
                    f"Warning: Incomplete message. Expected {expected_byte_count} bytes, got {len(int_data_bytes)}. Raw data: {message}")

            for field in field_list_cog_sog:
                field_name = field["name"]
                field_length = field["length"]
                decoder = self.field_decoders.get(field_name)
                if not decoder:
                    continue
                try:
                    self.data_dict[field_name] = decoder(int_data_bytes[start:start + field_length])
                except Exception as e:
                    print(
                        f"Could not decode field {field_name}. Error: {e}. Data: {int_data_bytes[start:start + field_length]}")
                start += field_length
        elif pgn == 127257:  # Attitude

            start = 0
            # field_list = [
            #     {"name": "SID", "length": 1},
            #     {"name": "Yaw", "length": 2},
            #     {"name": "Pitch", "length": 2},
            #     {"name": "Roll", "length": 2},
            #     # {"name": "Reserved", "length": 1},
            # ]
            expected_byte_count = sum(field['length'] for field in self.field_list_attitude)
            if len(int_data_bytes) < expected_byte_count:
                print(
                    f"Warning: Message has {len(int_data_bytes)} bytes, but expected {expected_byte_count}. Raw data: {message}")

            for field in self.field_list_attitude:
                # print(field)
                field_name = field["name"]
                field_length = field["length"]
                decoder = self.field_decoders.get(field_name)  # Use get() to safely retrieve the decoder
                if not decoder:
                    # print(f"No decoder found for field {field_name}.")
                    continue
                if not field_name == 'SID':
                    try:
                        self.data_dict[field_name] = 180*decoder(int_data_bytes[start:start + field_length])/np.pi
                        # print(field_name, self.data_dict)
                    except Exception as e:
                        print(
                            f"Could not decode field {field_name}. Error: {e}. Data: {int_data_bytes[start:start + field_length]}")

                start += field_length

        elif pgn == 127250:  # Heading
            # current_timestamp = datetime.strptime(timestamp, "%H:%M:%S.%f")

            # self.prev_heading_timestamp = current_timestamp
            start = 0
            # field_list = [
            #     # Add your heading field structure here, for example:
            #     {"name": "SID", "length": 1},
            #     {"name": "Heading", "length": 2},
            #     # {"name": "Deviation", "length": 2},
            #     # {"name": "Variation", "length": 2},
            #     # {"name": "Reference", "length": 1},
            # ]
            expected_byte_count = sum(field['length'] for field in self.field_list_heading)
            if len(int_data_bytes) < expected_byte_count:
                print(
                    f"Warning: Message has {len(int_data_bytes)} bytes, but expected {expected_byte_count}. Raw data: {message}")

            for field in self.field_list_heading:
                field_name = field["name"]
                field_length = field["length"]
                decoder = self.field_decoders.get(field_name)  # Use get() to safely retrieve the decoder
                if not decoder:
                    # print(f"No decoder found for field {field_name}.")
                    continue
                if not field_name == 'SID':
                    try:
                        self.data_dict[field_name] = 180*decoder(int_data_bytes[start:start + field_length])/np.pi
                        # print(field_name, 360*self.data_dict[field_name]/(2*np.pi))
                    except Exception as e:
                        print(
                            f"Could not decode field {field_name}. Error: {e}. Data: {int_data_bytes[start:start + field_length]}")

                start += field_length
    # else:
    #     print("no match")
    def run(self):
        while True:
            raw_message = self._stream.readline().decode().strip()
            self.parse_raw_message(raw_message)
            # print("self.data_dict", self.data_dict)
            # time.sleep(0.000001)

    def get_data(self):
        return self.data_dict.copy()

gnss_thread_ar20 = Furuno('COM7')
gnss_thread_ar20.start()


while True:
    time.sleep(0.1)
    # print(gnss_thread_ar20.get_data()['Roll'])
    print(gnss_thread_ar20.get_data())

time.sleep(30)
