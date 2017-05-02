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

    def __init__(self, sensei_config):
        api_url = sensei_config["server"]["url"] + 'api/v1/'
        self.api = Api(api_url, sensei_config["server"]["username"], sensei_config["server"]["password"])
        self.aci = AciUart.AciUart(port=sensei_config['mesh_network']['serial_path'], baudrate=115200)
        self.classroom_id = sensei_config["classroom_id"]

    def get_sensor_updates(self):
        updates = []
        while True:
            try:
                evt = self.aci.events_queue.get_nowait()
                if isinstance(evt, AciEvent.AciEventNew) and evt.is_sensor_update():
                    updates.append(evt.sensor_values())
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

    def radio_obs_from_update(self, update):
        if not update.is_valid:
            return []
        obs = []
        for remote_id, rssi in zip(update.proximity_ids, update.proximity_rssi):
            ob_time = datetime.datetime.utcfromtimestamp(update.valid_time)
            if remote_id > 0 and rssi > 0:
                obs.append(RadioObservation(self.classroom_id, update.sensor_id, remote_id, ob_time, -rssi))
        return obs

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
                obs = [self.radio_obs_from_update(update) for update in updates]
                flattened_obs = [ob for sublist in obs for ob in sublist]
                if len(flattened_obs) > 0:
                    self.api.upload_obs(flattened_obs)
            elif time.time() - self.last_time_sync > Uploader.TIME_SYNC_INTERVAL:
                self.sync_time()
            else:
                time.sleep(0.5)


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
