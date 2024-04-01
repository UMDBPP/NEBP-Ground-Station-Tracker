"""
-------------------------------------------------------------------------------
MIT License
Copyright (c) 2023 Jeremy Snyder
Copyright (c) 2021 Ronnel Walton
Modified: Mathew Clutter
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
-------------------------------------------------------------------------------
"""
import sys
import time
from calendar import timegm
import requests
from requests.adapters import HTTPAdapter, Retry
from urllib3.exceptions import MaxRetryError
import csv
from pathlib import Path
import aprs
import multiprocessing
import subprocess
import atexit


class Balloon_Coordinates:
    # Initialize common variables
    def __init__(self, service_type:str) -> None:
        self.latest_time = multiprocessing.Value(typecode_or_type='f', lock=False)
        self.last_time = multiprocessing.Value(typecode_or_type='f', lock=False)
        self.coor_alt = multiprocessing.Array(typecode_or_type='f', size_or_initializer=3, lock=False)

        self.service_type = service_type
        
        self.stop_process = multiprocessing.Event()
        self.process_lock = multiprocessing.Lock()

        atexit.register(self.stop)
        return
    

    # Test whether the position updating is working
    def test(self) -> int:
        if not self.coor_alt_process.is_alive():
            print(str(self.service_type) + " position update process is not running at test start")
            return -1

        # Save the current coor_alt
        old_coor_alt = self.get_coor_alt()

        # Wait for a bit for the position to update
        # Sleeping with a loop like this should stop the program not responding message
        for _ in range(6):
            time.sleep(1)
        
        # Check if coor_alt has changed
        if self.get_coor_alt() != old_coor_alt:
            print(str(self.service_type) + " position update process is receiving updates")
            return 0
        if self.coor_alt_process.is_alive(): # No change in coor_alt, so is the process still running?
            print("Position update process is running, but no positions received yet")
            return 1
        else:
            print(str(self.service_type) + " position update process is not running at test end")
            return -2


    # Start and test the position update process
    def start(self) -> int:
        self.coor_alt_process = multiprocessing.Process(target=self._update_coor_alt, daemon=True)
        self.coor_alt_process.start()

        return self.test()


    # Stop the position update process
    def stop(self):
        if self.coor_alt_process.is_alive():
            self.stop_process.set() # Send signal to stop
            print("Waiting for updater to stop...")
            self.coor_alt_process.join(15) # Pause execution here and wait for the process to stop or 15 seconds
            if self.coor_alt_process.is_alive():
                print("Updater has not stopped in 15 seconds. Terminating updater...")
                self.coor_alt_process.terminate()
                self.coor_alt_process.join(15) # Pause execution here and wait for the process to stop or 15 seconds
                if self.coor_alt_process.is_alive():
                    print("Updater has not terminated in 15 seconds. Killing updater...")
                    self.coor_alt_process.kill()
                    self.coor_alt_process.join(15) # Pause execution here and wait for the process to stop or 15 seconds
                    if self.coor_alt_process.is_alive():
                        print("Error stopping updater")
                        # Should probably throw an error here
                        self.stop_process.clear()
                        return
            print("Updater stopped")
        else:
            print(str(self.service_type) + " position update process is not running at process stop")
        # Clear stop flag after process has stopped
        self.stop_process.clear()
        return


    # Stop then restart process
    def reset(self) -> int:
        self.stop()
        return self.start()


    # Return a list of lat, long, and alt from latest position update
    def get_coor_alt(self):
        with self.process_lock:
            return [self.coor_alt[0], self.coor_alt[1], self.coor_alt[2]]


    # prints the latest lat, long and alt of the balloon
    def print_info(self):
        local_coor_alt = self.get_coor_alt()
        latest_tm = time.gmtime(self.latest_time.value)
        print("Date: {}-{}-{}".format(latest_tm[0], latest_tm[1], latest_tm[2]))
        print("Coordinates: ({}, {})".format(local_coor_alt[0], local_coor_alt[1]))
        print("Altitude: {}".format(local_coor_alt[2]))

        infoStr = "Date: {}-{}-{}".format(latest_tm[0], latest_tm[1], latest_tm[2])
        infoStr += "\n" + "Coordinates: ({}, {}) Altitude: {} m".format(local_coor_alt[0], local_coor_alt[1], local_coor_alt[2])
        infoStr += "\n Balloon Selected!"
        return infoStr


    # finds the difference in time between the latest ping and the one before
    def getTimeDiff(self):
        with self.process_lock:
            print(self.last_time.value)

            print(self.latest_time.value - self.last_time.value)
            return self.latest_time.value - self.last_time.value


    pass



class Balloon_Coordinates_APRS(Balloon_Coordinates):
    def __init__(self, service_type:str, callsign:str) -> None:
        super().__init__(service_type)
        self.callsign = callsign
        if len(self.callsign.split("-")) == 1:
            self.callsign_has_ssid = False
        else:
            self.callsign_has_ssid = True
        return


    # Return a list of callsigns in the APRS_Callsigns.csv file
    @staticmethod
    def list_callsigns():
        # Create path from executing file (this file) directory to CSV file in data directory
        dataPath = Path(__file__).parent / "../data/APRS_Callsigns.csv"
        try:
            with dataPath.open() as file:
                callsignsCSV = csv.reader(file)
                callsignList = []

                for line in callsignsCSV:
                    for callsign in line:
                        callsignList.append(callsign)

                return callsignList
        except IOError:
            print("Could not read file: ", dataPath.name)
            return []


    def print_info(self):
        print("Callsign: ", self.callsign)
        return ("Callsign: " + self.callsign + " " + super().print_info())


    pass



class Balloon_Coordinates_APRS_SDR(Balloon_Coordinates_APRS):
    def __init__(self, service_type:str, callsign:str):
        super().__init__(service_type, callsign)

        self.start()
        return
    

    def start(self):
        self.sdrProc = subprocess.Popen("/usr/bin/rtl_fm -f 144.39M -o 4 - | /srv/NEBP-Ground_Station/NEBP-Ground-Station-Tracker/external/direwolf/build/src/direwolf -c /srv/NEBP-Ground_Station/NEBP-Ground-Station-Tracker/external/direwolf.conf -n 1 -r 24000 -b 16 -",
                                        shell=True,
                                        stdin=subprocess.PIPE,
                                        stdout=subprocess.DEVNULL)
        time.sleep(0.1)
        super().start()


    def stop(self):
        super().stop()
        self.sdrProc.terminate()
        self.sdrProc.wait(10)
        if self.sdrProc.poll() is None:
            print("Subprocess did not terminate in 10 seconds, sending kill signal")
            self.sdrProc.kill()
            self.sdrProc.wait()
        time.sleep(0.1)


    def _update_coor_alt(self):
        def _handle_frame(frame):
            print("\nIncoming frame:")
            print(frame)
            if frame.source.callsign == self.callsign.split("-")[0].encode('UTF-8') and (not self.callsign_has_ssid or frame.source.ssid == int(self.callsign.split("-")[1])):
                print("Matching callsign found")
                with self.process_lock:
                    self.coor_alt[0] = float(frame.info.lat)
                    self.coor_alt[1] = float(frame.info.long)
                    if frame.info.altitude_ft != None:
                        self.coor_alt[2] = float(frame.info.altitude_ft)*0.3048
                    if frame.info.timestamp != None:
                        self.last_time.value = self.latest_time.value
                        self.latest_time.value = float(frame.info.timestamp)
                print(self.get_coor_alt())

        with aprs.TCPKISS(host="localhost", port=8001) as aprs_tcp:
            print("APRS-SDR connection initialized")
            while not self.stop_process.is_set():
                aprs_tcp.read(callback=_handle_frame, min_frames=1)
                
        print("Process stopping...")
        return


    pass



class Balloon_Coordinates_APRS_SerialTNC(Balloon_Coordinates_APRS):
    def __init__(self, service_type:str, callsign:str, port:str, baud_rate:str):
        super().__init__(service_type, callsign)
        self.port = port
        self.baud_rate = baud_rate

        self.start()
        return


    def _update_coor_alt(self):
        def _handle_frame(frame):
            print("\nIncoming frame:")
            print(frame)
            if frame.source.callsign == self.callsign.split("-")[0].encode('UTF-8') and (not self.callsign_has_ssid or frame.source.ssid == int(self.callsign.split("-")[1])):
                print("Matching callsign found")
                with self.process_lock:
                    self.coor_alt[0] = float(frame.info.lat)
                    self.coor_alt[1] = float(frame.info.long)
                    if frame.info.altitude_ft != None:
                        self.coor_alt[2] = float(frame.info.altitude_ft)*0.3048
                    if frame.info.timestamp != None:
                        self.last_time.value = self.latest_time.value
                        self.latest_time.value = float(frame.info.timestamp)
                print(self.get_coor_alt())

        with aprs.SerialKISS(port=self.port, speed=self.baud_rate) as aprs_tnc:
            print("APRS serial TNC connection initialized")
            while not self.stop_process.is_set():
                aprs_tnc.read(callback=_handle_frame, min_frames=1)
                
        print("Process stopping...")
        return


    pass



class Balloon_Coordinates_APRS_IS(Balloon_Coordinates_APRS):
    def __init__(self, service_type:str, callsign:str):
        super().__init__(service_type, callsign)

        self.start()
        return
    

    def _update_coor_alt(self):
        aprsFilter = "filter p/" + self.callsign
        # aprsFilter = "filter r/39.0/-76.9/100"
        # aprsFilter = "filter t/p"

        def __handle_frame(frame):
            print("\nIncoming APRS-IS frame:")
            print(frame)
            with self.process_lock:
                self.coor_alt[0] = float(frame.info.lat)
                self.coor_alt[1] = float(frame.info.long)
                if frame.info.altitude_ft != None:
                    self.coor_alt[2] = float(frame.info.altitude_ft)*0.3048
                if frame.info.timestamp != None:
                    self.last_time.value = self.latest_time.value
                    self.latest_time.value = float(frame.info.timestamp)
            print(self.get_coor_alt())

        with aprs.TCP(host="noam.aprs2.net", port=14580, command=aprsFilter) as aprs_tcp:
            print("APRS-IS connection initialized")
            while not self.stop_process.is_set():
                aprs_tcp.read(callback=__handle_frame, min_frames=1)
                
        print("Process stopping...")
        return


    pass



class Balloon_Coordinates_APRS_fi(Balloon_Coordinates_APRS):
    def __init__(self, service_type:str, callsign:str, apikey:str):
        super().__init__(service_type, callsign)
        self.apikey = apikey

        self.APRS_fi_sess = requests.Session()
        self.APRS_fi_sess.headers.update({
            "User-Agent": "UMDBPP_NEBP_GS_Tracker/0.1 (+https://github.com/UMDBPP/NEBP-Ground-Station-Tracker)"
        })
        self.APRS_fi_sess.mount("https://", HTTPAdapter(max_retries=Retry(
            total=3,
            backoff_factor=0.2,
            status_forcelist=[500, 502, 503, 504]
        )))

        reqTest = Balloon_Coordinates_APRS_fi._req_packet(self)
        if(isinstance(reqTest, str)):
            print("\nPossible network connection problem or API failure response.\nExiting...")
            SystemExit(reqTest)

        self.start()
        return
    

    # Returns the json object of the latest APRS packet.
    def _req_packet(self):
        # Send request and deal with standard network/connection errors.
        # At the moment, any exceptions here will just print the error and return an empty list,
        # but they could (and should) be handled separately later.
        try:
            # Get latest APRS packet
            req = self.APRS_fi_sess.get("https://api.aprs.fi/api/get?name={}&what=loc&apikey={}&format=json".format(self.callsign, self.apikey), timeout=2)
            req.raise_for_status() # Raise an HTTPError exception for bad HTTP status codes
        except requests.exceptions.ConnectionError as e:
            # Some kind of network problem
            print("Request Connection Error:")
            print(e)
            return e.strerror
        except requests.exceptions.Timeout as e:
            # Request timed out
            print("Request Timeout:")
            print(e)
            return e.strerror
        except MaxRetryError as e:
            # Too many retries
            print("Too many retries:")
            print(e)
            return str(e.reason) # Not sure what to do with the nested exception
        except requests.exceptions.HTTPError as e:
            # Some kind of HTTP error (e.g. 404)
            print("Request HTTP Error:")
            print(e)
            return e.strerror
        except requests.exceptions.RequestException as e:
            # Some other request problem
            print("Request Exception:")
            print(e)
            return e.strerror
                
        # Presumably, some kind of proper response from the server at this point
        reqData = req.json()

        # Handle APRS.fi API-specific errors
        if(reqData["result"] == "fail"):
            print("APRS.fi API error: ", reqData["description"])
            return reqData["description"]
        
        return reqData
        

    # Position update process
    def _update_coor_alt(self):
        # Checks for updated position every 5 seconds
        timer = time.time() - 5
        while not self.stop_process.is_set():
            if (time.time() - timer) > 5:
                timer = time.time()
            # Request packet
            reqData = Balloon_Coordinates_APRS_fi._req_packet(self)

            # Save last time
            self.last_time.value = self.latest_time.value
            print(reqData)
            # Record current location
            with self.process_lock:
                self.coor_alt = [float(reqData["entries"][0]["lat"]),
                                float(reqData["entries"][0]["lng"]),
                                float(reqData["entries"][0]["altitude"])]
                # Record the last time this position was reported
                self.latest_time.value = int(reqData["entries"][0]["lasttime"])
                print(self.coor_alt)
            return


    pass



class Balloon_Coordinates_Borealis(Balloon_Coordinates):
    BOREALIS_EPOCH = 1357023600

    def __init__(self, service_type:str, imei):
        super().__init__(service_type)
        self.imei = imei

        try:
            # Grab and Define IMEI's Latest Flight
            req = requests.get("https://borealis.rci.montana.edu/meta/flights?imei={}".format(self.imei))
        except requests.exceptions.RequestException:  # does not catch if there is no internet
            print("No internet connection detected")
            # sys.exit(-1)

        self.latest_flight = req.json()[-1]

        # Define UID
        flightTime = timegm(time.strptime(self.latest_flight, "%Y-%m-%d")) - Balloon_Coordinates_Borealis.BOREALIS_EPOCH
        self.uid = (int(flightTime) << 24) | int(self.imei[8:])
        print("UID: " + str(self.uid))

        self.start()
        return


    @staticmethod
    def list_IMEI():
    # grab the list of all of the imei's on the borealis server

        IMEIs = []

       # Request IMEI List
        try:
            req = requests.get('https://borealis.rci.montana.edu/meta/imeis')
            data = req.json()
            for imei in data:
                IMEIs.append(imei)

        except requests.exceptions.RequestException:
            print("couldn't connect to internet")
            print("Please connect to internet and relaunch")
            # sys.exit(-1)

        return IMEIs


    # Returns the json object of the latest Borealis Iridium packet.
    def _req_packet(self):
        # Send request and deal with standard network/connection errors.
        # At the moment, any exceptions here will just print the error and return an empty list,
        # but they could (and should) be handled separately later.
        try:
            # Get latest APRS packet
            req = requests.get("https://borealis.rci.montana.edu/flight?uid={}".format(self.uid))
            print("https://borealis.rci.montana.edu/flight?uid={}".format(self.uid))
            req.raise_for_status() # Raise an HTTPError exception for bad HTTP status codes
        except requests.exceptions.ConnectionError as e:
            # Some kind of network problem
            print("Request Connection Error:")
            print(e)
            return e.strerror
        except requests.exceptions.Timeout as e:
            # Request timed out
            print("Request Timeout:")
            print(e)
            return e.strerror
        except MaxRetryError as e:
            # Too many retries
            print("Too many retries:")
            print(e)
            return str(e.reason) # Not sure what to do with the nested exception
        except requests.exceptions.HTTPError as e:
            # Some kind of HTTP error (e.g. 404)
            print("Request HTTP Error:")
            print(e)
            return e.strerror
        except requests.exceptions.RequestException as e:
            # Some other request problem
            print("Request Exception:")
            print(e)
            return e.strerror
                
        # Presumably, some kind of proper response from the server at this point
        return req.json()


    # Position update process
    def _update_coor_alt(self):
        # Checks for updated position every 5 seconds
        timer = time.time() - 5
        while not self.stop_process.is_set():
            if (time.time() - timer) > 5:
                timer = time.time()
                # Request packet
                reqData = Balloon_Coordinates_Borealis._req_packet(self)

                # Save last time
                self.last_time.value = self.latest_time.value
                # print(reqData)
                # Record current location
                with self.process_lock:
                    self.coor_alt[0] = float(reqData['data'][-1][3])
                    self.coor_alt[1] = float(reqData['data'][-1][4])
                    self.coor_alt[2] = float(reqData['data'][-1][5])
                    # Record the last time this position was reported
                    self.latest_time.value = int(reqData['data'][-1][2])
                print(self.get_coor_alt())

        print("Borealis update process stopping...")
        return


    def print_info(self):
        print("IMEI: ", self.imei)
        return ("IMEI: " + self.imei + " " + super().print_info())    


    pass