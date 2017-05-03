#!/usr/bin/env python3

from argparse import ArgumentParser
from aci import AciCommand
from aci_serial import AciUart
from aci import AciEvent
from queue import Empty
import time
import sensei_cmd
from sensei import *
import datetime
import yaml
from os.path import expanduser
import os.path

class Uploader(object):
    # Synchronize once every minute
    TIME_SYNC_INTERVAL=60

    MESH_HANDLE_MESH_CONTROL = 0x0201

    def __init__(self, sensei_config):
        api_url = sensei_config["server"]["url"] + 'api/v1/'
        self.api = Api(api_url, sensei_config["server"]["username"], sensei_config["server"]["password"])
        self.aci = AciUart.AciUart(port=sensei_config['mesh_network']['serial_path'], baudrate=115200)
        self.classroom_id = sensei_config["classroom_id"]

    def handle_heartbeat(self, hb):
        print(str.format("handling heartbeat: %s" %(hb)))
        if hb.epoch_seconds != hb.received_at:
          print(str.format("Sensor %d clock offset detected; issuing sync_time." %(hb.sensor_id)))
          self.sync_time()

    def get_sensor_updates(self):
        updates = []
        while True:
            try:
                evt = self.aci.events_queue.get_nowait()
                print(str.format("evt = %s" %(evt)))
                if isinstance(evt, AciEvent.AciEventNew) and evt.is_sensor_update():
                    updates.append(evt.sensor_values())
                elif isinstance(evt, AciEvent.AciEventAppEvt) and evt.is_heartbeat():
                    self.handle_heartbeat(evt.heartbeat_msg())
            except Empty:
                break
        return updates

    def run_app_command(self, command):
        data = command.serialize()
        return self.aci.write_aci_cmd(AciCommand.AciAppCommand(data=data,length=len(data)+1))

    def sync_time(self):
        self.last_time_sync = time.time()
        result = self.run_app_command(sensei_cmd.SetTime())

    def get_config(self):
        return self.run_app_command(sensei_cmd.GetConfig())

    def set_mesh_control(self, wake_interval):
        data = struct.pack("<HB", wake_interval, 0)
        self.aci.ValueSet(MESH_HANDLE_MESH_CONTROL, data)

    def radio_obs_from_update(self, update):
        if not update.is_valid:
            return []
        obs = []
        for remote_id, rssi in zip(update.proximity_ids, update.proximity_rssi):
            ob_time = datetime.datetime.utcfromtimestamp(update.valid_time)
            if remote_id > 0 and rssi > 0:
                obs.append(RadioObservation(self.classroom_id, update.sensor_id, remote_id, ob_time, -rssi))
        return obs

    def accelerometer_measurement_from_update(self, update):
        if update.is_valid and (update.accel_x or update.accel_y or update.accel_z):
            ob_time = datetime.datetime.utcfromtimestamp(update.valid_time)
            return AccelerometerObservation(self.classroom_id, update.sensor_id,
                ob_time, update.accel_x, update.accel_y, update.accel_z)

    def run(self):
        # Wait for serial connection to be ready
        time.sleep(3)
        print("Getting config")
        self.get_config()

        # Sync time
        print("Syncing time")
        self.sync_time()

        while True:
            updates = self.get_sensor_updates()
            if len(updates) > 0:
                continue
                obs = [self.radio_obs_from_update(update) for update in updates]
                flattened_obs = [ob for sublist in obs for ob in sublist]
                if len(flattened_obs) > 0:
                    self.api.upload_radio_observations(flattened_obs)
                accelerometer_obs = []
                for update in updates:
                    ob = self.accelerometer_measurement_from_update(update)
                    if ob:
                        accelerometer_obs.append(ob)
                if len(accelerometer_obs) > 0:
                    self.api.upload_accelerometer_observations(accelerometer_obs)
            else:
                time.sleep(0.5)

            if time.time() - self.last_time_sync > Uploader.TIME_SYNC_INTERVAL:
                self.sync_time()


if __name__ == '__main__':

    config_path = expanduser("~") + "/.sensei.yaml"
    if os.path.isfile(config_path):
        with open(config_path, 'r') as stream:
            try:
                sensei_config = yaml.load(stream)
                uploader = Uploader(sensei_config)
                uploader.run()
            except yaml.YAMLError as exc:
                print(exc)
    else:
        print(str.format("Please configure settings in %s" %(config_path)))
        exit(-1)
