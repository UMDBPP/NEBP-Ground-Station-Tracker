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

class Balloon_Coordinates_APRS:
    def __init__(self, callsign:str, apikey:str):

        self.callsign = callsign
        self.apikey = apikey

        self.latest_time = 0
        self.last_time = 0
        self.coor_alt = [0, 0, 0]
        self.last_coor_alt = [0, 0, 0]

        self.APRSfi_sess = requests.Session()
        self.APRSfi_sess.headers.update({
            "User-Agent": "UMDBPP_NEBP_GS_Tracker/0.1 (+https://github.com/UMDBPP/NEBP-Ground-Station-Tracker)"
        })
        self.APRSfi_sess.mount("https://", HTTPAdapter(max_retries=Retry(
            total=3,
            backoff_factor=0.2,
            status_forcelist=[500, 502, 503, 504]
        )))

        reqTest = Balloon_Coordinates_APRS.__req_packet__(self)
        if(isinstance(reqTest, str)):
            print("\nPossible network connection problem or API failure response.\nExiting...")
            SystemExit(reqTest)

        print("Balloon_Coordinates_APRS successfully initialized")
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
    

    # Returns the json object of the latest APRS packet.
    def __req_packet__(self):
        # Send request and deal with standard network/connection errors.
        # At the moment, any exceptions here will just print the error and return an empty list,
        # but they could (and should) be handled separately later.
        try:
            # Get latest APRS packet
            req = self.APRSfi_sess.get("https://api.aprs.fi/api/get?name={}&what=loc&apikey={}&format=json".format(self.callsign, self.apikey), timeout=2)
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
        

    # Return a list of lat, long, and alt from latest APRS packet
    def get_coor_alt(self):
        # Request packet
        reqData = Balloon_Coordinates_APRS.__req_packet__(self)

        if(reqData == []):
            # Some kind of problem with getting the packet
            return []

        # Save last location and time
        self.last_coor_alt = self.coor_alt
        self.last_time = self.latest_time

        # Record current location
        self.coor_alt = [float(reqData["entries"][0]["lat"]),
                         float(reqData["entries"][0]["lng"]),
                         float(reqData["entries"][0]["altitude"])]
        # Record the last time this position was reported
        self.latest_time = int(reqData["entries"][0]["lasttime"])
        print(self.coor_alt)

        return self.coor_alt  # [lat, long, altitude]


    def print_info(self):
        # prints the latest lat, long and alt of the balloon
        self.get_coor_alt()
        latest_tm = time.gmtime(self.latest_time)
        print("Callsign: ", self.callsign)
        print("Date: {}-{}-{}".format(latest_tm[0], latest_tm[1], latest_tm[2]))
        print("Coordinates: ({}, {})".format(self.coor_alt[0], self.coor_alt[1]))
        print("Altitude: {}".format(self.coor_alt[2]))

        infoStr = "Callsign: " + self.callsign + " Date: {}-{}-{}".format(latest_tm[0], latest_tm[1], latest_tm[2])
        infoStr += "\n" + "Coordinates: ({}, {}) Altitude: {}".format(self.coor_alt[0], self.coor_alt[1], self.coor_alt[2])
        infoStr += "\n Balloon Selected!"
        return infoStr


    def getTimeDiff(self):
        # finds the difference in time between the latest ping and the one before
        print(self.last_time)

        print(self.latest_time - self.last_time);
        return self.latest_time - self.last_time

    pass


class Balloon_Coordinates_Borealis:
    BOREALIS_EPOCH = 1357023600

    def __init__(self, imei):

        self.imei = imei

        try:
            # Grab and Define IMEI's Latest Flight
            req = requests.get("https://borealis.rci.montana.edu/meta/flights?imei={}".format(self.imei))
        except requests.exceptions.RequestException:  # does not catch if there is no internet
            print("No internet connection detected")
            sys.exit(-1)

        self.latest_flight = req.json()[-1]

        # Define UID
        flightTime = timegm(time.strptime(self.latest_flight, "%Y-%m-%d")) - Balloon_Coordinates_Borealis.BOREALIS_EPOCH
        self.uid = (int(flightTime) << 24) | int(self.imei[8:])

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
    

    def get_coor_alt(self):
        # returns a list containing the lat, long, and alt of the latest ping from the selected IMEI
        try:
            req = requests.get("https://borealis.rci.montana.edu/flight?uid={}".format(self.uid))
            print("https://borealis.rci.montana.edu/flight?uid={}".format(self.uid))
        except requests.exceptions.RequestException:
            print("couldn't get updated position (no internet probably)")
            return []

        data = req.json()
        # print(data) # for debugging server problem, will print a lot

        # print(data['data'][-1][3])

        # Lat, Long, Alt
        self.coor_alt = [data['data'][-1][3], data['data'][-1][4], data['data'][-1][5]]
        print(self.coor_alt)

        return self.coor_alt  # [lat, long, altitude]

    def print_info(self):
        # prints the latest lat, long and alt of the balloon
        self.get_coor_alt()
        print("IMEI: ", self.imei)
        print("Date:", self.latest_flight)
        print("Coordinates: (", self.coor_alt[0], ", ", self.coor_alt[1], ")")
        print("Altitude: ", self.coor_alt[2])

        infoStr = "IMEI: " + self.imei + " Date: " + self.latest_flight
        infoStr += "\n" + "Coordinates: (" + str(self.coor_alt[0]) + ", " + str(self.coor_alt[1]) + ")" + " Altitude: " + str(self.coor_alt[2])
        infoStr += "\n Balloon Selected!"
        return infoStr

    def getTimeDiff(self):
        # finds the difference in time between the latest ping and the one before
        req = requests.get("https://borealis.rci.montana.edu/flight?uid={}".format(self.uid))
        data = req.json()

        lastTime = [data['data'][-1][2], data['data'][-2][2]]

        print(lastTime[0])

        print(lastTime[0] - lastTime[1])
        # return lastTime[0] - lastTime[1]
        return lastTime[0] - lastTime[1]

    pass

