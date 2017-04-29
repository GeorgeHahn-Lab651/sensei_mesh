#!/usr/bin/env python3

from argparse import ArgumentParser
from aci import AciCommand
from aci_serial import AciUart
from aci import AciEvent
import time
import sensei_cmd
import sensei
import yaml
from os.path import expanduser
import urllib2
import os.path

class Uploader(object):
    def __init__(self, sensei_config):
        api_url = sensei_config["server"]["url"] + 'api/v1/'
        self.api = sensei.Api(api_url, sensei_config["server"]["username"], sensei_config["server"]["password"])
        self.aci = AciUart.AciUart(port=sensei_config['mesh_network']['serial_path'], baudrate=115200)

    def get_sensor_updates(self):
        updates = []
        while True:
            try:
                self.acidev.events_queue.get_nowait()
                if isinstance(evt, AciEvent.AciEventNew) and evt.is_sensor_update():
                    updates.append(evt.sensor_values())
            except Empty:
                break

    def radio_obs_from_sensor_updates(updates):
        

    def run(self):
        # Wait for serial connection to be ready
        time.sleep(2)

        while True:
            updates = self.get_sensor_updates()
            if len(updates) > 0:
                self.api.upload_obs([self.])
            else:
                time.sleep(2)



if __name__ == '__main__':

    config_path = expanduser("~") + "/.sensei.yaml"
    if os.path.isfile(config_path):
        with open(config_path, 'r') as stream:
        try:
            sensei_config = yaml.load(stream)
            uploader = Uploader(sensei_config)
            upload
        except yaml.YAMLError as exc:
            print(exc)
    else:
        print(str.format("Please configure settings in %s" %(config_path)))
        exit(-1)
