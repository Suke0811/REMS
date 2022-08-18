import time

from sim.device import BasicDeviceBase
import subprocess, logging, fnmatch, platform


def os_compatible():
    compatible_os = ['Windows']
    if platform.system() in compatible_os:
        return True
    else:
        return False


class WifiDevice(BasicDeviceBase):
    def __init__(self, ssid):
        self.ssid = ssid
        self.ssid_original = None

    def init(self):
        if not os_compatible():
            logging.warning(f"WifiDevice is not compatible with {platform.system()} yet. Nothing will be executed for this device.")
            return

        self.open()

    def open(self):
        """
        save current ssid -> disconnect, pose 1sec -> search for specified ssid -> connect if exists
        -> connect to the original network if not exists
        :return:
        """
        ssids = self.search()
        if ssids:
            self.ssid_original = ssids[0]
        self.enable(False)
        time.sleep(1)
        ssids = self.search()
        logging.info(f"SSID list: {ssids}")
        matching = fnmatch.filter(ssids, self.ssid)
        if matching:
            self.ssid = matching[0]
        else:
            self.ssid = self.ssid_original
        self.enable(True)


    def close(self):
        if not os_compatible():
            return
        self.ssid = self.ssid_original
        self.enable(True)

    def enable(self, enable):
        if enable:
            output = subprocess.check_output(f'netsh wlan connect {self.ssid}', shell=True)
        else:
            output = subprocess.check_output(f'netsh wlan disconnect', shell=True)
        time.sleep(1)
        logging.info(f'SSID: {self.ssid}: ' + str(output.decode()))


    def search(self):
        results = subprocess.check_output(["netsh", "wlan", "show", "network"])
        results = results.decode("ascii")  # needed in python 3
        results = results.replace("\r", "")
        ls = results.split("\n")
        ls = ls[4:]
        ssids = []
        x = 0
        while x < len(ls):
            if x % 5 == 0:
                ssids.append(ls[x].split(': ')[-1])
            x += 1

        return ssids






if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    w = WifiDevice('ESP*')
    w.init()
    w.close()

