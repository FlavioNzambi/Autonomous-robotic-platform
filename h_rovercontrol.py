"""
        Program:    RoverController Class for the MiniTHOR system
         Author:    Flaviano Nzambi
    Description:    This Python module defines the RoverController Class for the MiniTHOR system and all the affiliate functions 
           Date:    19th January 2024
"""
from pymavlink import mavutil

class RoverController:
    def __init__(self):
        self.master = None

    def connect(self, address='/dev/ttyACM0', baud=115200):
        """ connects to the pixhawk via the address and baud """
        print('MiniTHOR Communication Test - Connecting')  
        self.master = mavutil.mavlink_connection(address, baud)
        self.master.wait_heartbeat()

        print(f"Connected To Rover at: address={address} Baud={baud}!")
        print("Heartbeat from system(system %u component %u)" %
              (self.master.target_system, self.master.target_component))
        
    def arm(self):
        self.master.arducopter_arm()
        self.master.wait_heartbeat()
        print("Rover Armed")

    def change_mode(self, mode):
        mode_id = self.master.mode_mapping()[mode]

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)

        self.master.wait_heartbeat()
        print("Mode:", mode_id)

    def stop(self):
        self.master.mav.manual_control_send(
            self.master.target_system,
            0,
            0,
            0,
            0,
            0
        )

    def forward_turn(self, forward_value, turn_value):
        self.master.mav.manual_control_send(
            self.master.target_system,
            1,
            turn_value,
            forward_value,
            0,
            0
        )

    def close(self):
        self.master.mav.manual_control_send(
            self.master.target_system,
            0,
            0,
            0,
            0,
            0
        )
        self.master.arducopter_disarm()
        self.master.close()

    def voltage(self):
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)

        msg = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
        battery_voltage = msg.voltage_battery / 1000.0  # Convert to volts
        return battery_voltage
    
    def rc_send(self,rc_id,pwm):
        rc_cha_val = [65535 for _ in range(8)]
        rc_cha_val[rc_id - 1] = pwm
        self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *rc_cha_val)
