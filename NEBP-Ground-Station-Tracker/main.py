"""
-------------------------------------------------------------------------------
MIT License
Copyright (c) 2023 Jeremy Snyder
Copyright (c) 2021 Mathew Clutter
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

Note that this project uses PQt5, which is licensed under GPL v3
https://pypi.org/project/PyQt5/
-------------------------------------------------------------------------------
"""

from PyQt5 import QtWidgets
from PyQt5.QtCore import QThread, QObject, pyqtSignal, Qt, pyqtSlot
from PyQt5.QtWidgets import QCompleter, QApplication, QDesktopWidget
from Ground_Station_GUI import Ui_MainWindow
import sys
from Balloon_Coordinates import Balloon_Coordinates_Borealis, Balloon_Coordinates_APRS_fi, Balloon_Coordinates_APRS_IS, Balloon_Coordinates_APRS_SDR, Balloon_Coordinates_APRS_SerialTNC
from satelliteTrackingMath import trackMath
from Ground_Station_Arduino import Ground_Station_Arduino
import serial.tools.list_ports
import time
#from pylab import *
from sunposition import sunpos
from datetime import datetime
import csv
# import statistics
# import numpy as np

# todo: clean up this code and better document


class Window(QtWidgets.QMainWindow, Ui_MainWindow):
    # This class connects functionality to buttons in the GUI
    # The "main" class
    def __init__(self):
        super(Window, self).__init__()

        self.setupUi(self)

        self.IMEIList = Balloon_Coordinates_Borealis.list_IMEI()
        self.Borealis_comboBox_IMEI.addItem("")
        if self.IMEIList == []:
            print("Unable to get IMEI list")
            self.statusBox.setPlainText("Unable to get IMEI list")
        else:
            for i in range(len(self.IMEIList)):
                self.Borealis_comboBox_IMEI.addItem(self.IMEIList[i])
        self.refreshCallsigns()

        self.arduinoConnected = False
        self.IMEIAssigned = False
        self.callsignAssigned = False
        self.APRSfiKeyAssigned = False
        self.radioConnected = False
        self.GSLocationSet = False
        self.calibrated = False

        self.tracking = False

        self.GSArduino = None  # classes will be instantiated later
        self.Balloon = None

        self.trackThread = None
        self.worker = None

        self.GSLat = 0
        self.GSLong = 0
        self.GSAlt = 0

        self.startingAzimuth = 0
        self.startingElevation = 0

        completerIMEI = QCompleter(self.IMEIList)
        completerIMEI.setFilterMode(Qt.MatchContains)
        self.Borealis_comboBox_IMEI.setEditable(True)
        self.Borealis_comboBox_IMEI.setCompleter(completerIMEI)

        completerCallsign = QCompleter(self.callsignList)
        completerCallsign.setFilterMode(Qt.MatchContains)
        self.APRS_fi_comboBox_Callsign.setEditable(True)
        self.APRS_fi_comboBox_Callsign.setCompleter(completerCallsign)
        self.APRS_IS_comboBox_Callsign.setEditable(True)
        self.APRS_IS_comboBox_Callsign.setCompleter(completerCallsign)
        self.APRS_Radio_comboBox_Callsign.setEditable(True)
        self.APRS_Radio_comboBox_Callsign.setCompleter(completerCallsign)

        self.ports = None
        self.portNames = []
        self.comPortCounter = 0
        self.refreshCOMPortLists()

        self.Borealis_button_RefreshIMEI.clicked.connect(self.refreshIMEI)
        self.Borealis_button_ConfirmIMEI.clicked.connect(self.assignIMEI)
        self.APRS_fi_button_ConfirmCallsign.clicked.connect(self.assignCallsign_APRS_fi)
        self.APRS_fi_button_ConfirmAPIKey.clicked.connect(self.assignAPRSfiKey)
        self.APRS_IS_button_ConfirmCallsign.clicked.connect(self.assignCallsign_APRS_IS)
        self.APRS_Radio_button_ConfirmCallsign.clicked.connect(self.assignCallsign_APRS_Radio)
        self.APRS_Radio_button_refreshCOMPorts.clicked.connect(self.refreshCOMPortLists)
        self.APRS_Radio_button_ConnectRadio.clicked.connect(self.connectRadio_APRS_Radio)
        self.APRS_Radio_comboBox_COMPort.setCurrentIndex(self.comPortCounter - 1)

        self.Tracking_button_Refresh.clicked.connect(self.refreshTrackingStatus)
        self.Tracking_button_Reset.clicked.connect(self.resetTrackingStatus)

        self.GPSRequestButton.clicked.connect(self.getGSLocation)
        self.confirmGSLocationButton.clicked.connect(self.setGSLocation)

        self.calibrateButton.clicked.connect(self.calibrate)

        self.Arduino_button_refreshCOMPorts.clicked.connect(self.refreshCOMPortLists)
        self.Arduino_button_ConnectArduino.clicked.connect(self.connectToArduino)

        self.degreesPerClickBox.setCurrentIndex(1)
        self.Arduino_comboBox_COMPort.setCurrentIndex(self.comPortCounter - 1)
        self.tiltUpButton.clicked.connect(self.tiltUp)
        self.tiltDownButton.clicked.connect(self.tiltDown)
        self.panCounterClockwiseButton.clicked.connect(self.panClockwise)
        self.panClockwiseButton.clicked.connect(self.panCounterClockwise)

        self.calculateStartingPosButton.clicked.connect(self.getStartingPos)

        self.backToSunButton.clicked.connect(self.returnToSun)
        self.backToZeroButton.clicked.connect(self.returnToZero)

        self.startButton.clicked.connect(self.checkIfReady)
        self.stopButton.clicked.connect(self.stopTracking)
        self.EStopButton.clicked.connect(self.EStop)

        self.predictionStartButton.clicked.connect(self.setPredictTrack)

        font = self.font()
        font.setPointSize(11)  # can adjust for sizing
        QApplication.instance().setFont(font)

        # self.showMaximized()
        # self.showFullScreen()

        self.predictingTrack = False


    def refreshIMEI(self):
        # Refresh the Borealis_comboBox_IMEI IMEI list from the Borealis website
        print("Refreshing IMEI list")
        self.statusBox.setPlainText("Refreshing IMEI list")
        self.IMEIList = Balloon_Coordinates_Borealis.list_IMEI()
        self.Borealis_comboBox_IMEI.clear()
        self.Borealis_comboBox_IMEI.addItem("")
        if self.IMEIList == []:
            print("Unable to get IMEI list")
            self.statusBox.setPlainText("Unable to get IMEI list")
        else:
            for i in range(len(self.IMEIList)):
                self.Borealis_comboBox_IMEI.addItem(self.IMEIList[i])
        return


    def refreshTrackingStatus(self):
        if type(self.Balloon) is not type(None):
            testStr = self.Balloon.print_info()
            self.statusBox.setPlainText(testStr)
        return
    

    def resetTrackingStatus(self):
        if type(self.Balloon) is not type(None):
            status = self.Balloon.reset()
            if status == 0:
                self.statusBox.setPlainText("Position update process successfully restarted and receiving")
            elif status == 1:
                self.statusBox.setPlainText("Position update process successfully restarted, but no positions have been received yet")
            elif status == -1:
                self.statusBox.setPlainText("Position update process is not running")
            elif status == -2:
                self.statusBox.setPlainText("Position update process stopped, but failed to restart")
            else:
                self.statusBox.setPlainText("Unexpected status received from reset function: " + str(status))
        return
        

    def assignIMEI(self):
        # this function checks if an IMEI has been selected
        # if an IMEI has been selected, it creates an instance of the balloon coordinates class using the IMEI
        # if an IMEI has not been selected, it simply returns
        if self.Borealis_comboBox_IMEI.currentIndex() != 0:  # SHOULD BE != FOR BOREALIS WEBSITE!
            self.IMEIAssigned = True
            print(self.Borealis_comboBox_IMEI.currentText())        
            if type(self.Balloon) is not type(None):
                self.Balloon.stop()
                time.sleep(1)
            del self.Balloon
            self.Balloon = Balloon_Coordinates_Borealis(service_type="Borealis",
                                                        imei=self.Borealis_comboBox_IMEI.currentText())
            testStr = self.Balloon.print_info()
            self.statusBox.setPlainText(testStr)
            # self.Balloon.getTimeDiff()
        else:
            print("select a balloon ")
            self.statusBox.setPlainText("Please select a balloon IMEI")
            self.IMEIAssigned = False
        return


    def refreshCallsigns(self):
        self.callsignList = Balloon_Coordinates_APRS_IS.list_callsigns()

        self.APRS_fi_comboBox_Callsign.clear()
        self.APRS_IS_comboBox_Callsign.clear()
        self.APRS_Radio_comboBox_Callsign.clear()

        self.APRS_fi_comboBox_Callsign.addItem("")
        self.APRS_IS_comboBox_Callsign.addItem("")
        self.APRS_Radio_comboBox_Callsign.addItem("")
        
        for idx, callsign in enumerate(self.callsignList):
            if len(callsign.split("-")) == 1:
                self.APRS_IS_comboBox_Callsign.addItem(callsign)
                self.APRS_Radio_comboBox_Callsign.addItem(callsign)
            else:
                if self.APRS_IS_comboBox_Callsign.findText(callsign.split("-")[0]) == -1:
                    self.APRS_IS_comboBox_Callsign.addItem(callsign.split("-")[0])
                    self.APRS_Radio_comboBox_Callsign.addItem(callsign.split("-")[0])
                
                self.APRS_IS_comboBox_Callsign.addItem(callsign)
                self.APRS_Radio_comboBox_Callsign.addItem(callsign)
                self.APRS_fi_comboBox_Callsign.addItem(callsign)


    def assignCallsign_APRS_fi(self):
        self.assignCallsign("APRS.fi")
        return


    def assignCallsign_APRS_IS(self):
        self.assignCallsign("APRS-IS")
        return
    

    def assignCallsign_APRS_Radio(self):
        if self.APRS_Radio_radioButton_SDR.isChecked():
            self.assignCallsign("APRS_Radio_SDR")
        elif self.APRS_Radio_radioButton_Serial.isChecked():
            self.assignCallsign("APRS_Radio_Serial")
        else:
            print("assignCallsign_APRS_Radio: This message should not be seen")
    

    def assignCallsign(self, service:str):
        # this function checks if a callsign has been selected
        if service == "APRS.fi":
            # if a callsign has been selected and an API key has been entered, it creates an instance of the balloon coordinates APRS class
            # if a callsign has not been selected or an API key has not been entered, it simply returns
            if self.APRS_fi_comboBox_Callsign.currentIndex() != 0:
                self.callsignAssigned = True
                print("Assigned callsign: " + str(self.APRS_fi_comboBox_Callsign.currentText()))
                if(self.APRSfiKeyAssigned):
                    if type(self.Balloon) is not type(None):
                        self.Balloon.stop()
                        time.sleep(1)
                    del self.Balloon
                    self.Balloon = Balloon_Coordinates_APRS_fi(service_type="APRS.fi",
                                                               callsign=self.APRS_fi_comboBox_Callsign.currentText(),
                                                               apikey=self.APRS_fi_lineEdit_APIKey.text())
                    testStr = self.Balloon.print_info()
                    self.statusBox.setPlainText(testStr)
            else:
                print("Select a balloon callsign from the list in APRS_Callsigns.csv")
                self.statusBox.setPlainText("Please select a balloon callsign from those listed in APRS_Callsigns.csv")
                self.callsignAssigned = False
        elif service == "APRS-IS":
            if self.APRS_IS_comboBox_Callsign.currentIndex() != 0:
                self.callsignAssigned = True
                print("Assigned callsign: " + str(self.APRS_IS_comboBox_Callsign.currentText()))
                if type(self.Balloon) is not type(None):
                    self.Balloon.stop()
                    time.sleep(1)
                del self.Balloon
                self.Balloon = Balloon_Coordinates_APRS_IS(service_type="APRS-IS",
                                                           callsign=self.APRS_IS_comboBox_Callsign.currentText())
                testStr = self.Balloon.print_info()
                self.statusBox.setPlainText(testStr)
            else:
                print("Select a balloon callsign from the list in APRS_Callsigns.csv")
                self.statusBox.setPlainText("Please select a balloon callsign from those listed in APRS_Callsigns.csv")
                self.callsignAssigned = False
        elif service == "APRS_Radio_SDR":
            if self.APRS_Radio_comboBox_Callsign.currentIndex() != 0:
                self.callsignAssigned = True
                print("Assigned callsign: " + str(self.APRS_Radio_comboBox_Callsign.currentText()))
                if type(self.Balloon) is not type(None):
                    self.Balloon.stop()
                    time.sleep(1)
                del self.Balloon
                self.Balloon = Balloon_Coordinates_APRS_SDR(service_type="APRS-SDR",
                                                           callsign=self.APRS_Radio_comboBox_Callsign.currentText())
                testStr = self.Balloon.print_info()
                self.statusBox.setPlainText(testStr)
            else:
                print("Select a balloon callsign from the list in APRS_Callsigns.csv")
                self.statusBox.setPlainText("Please select a balloon callsign from those listed in APRS_Callsigns.csv")
                self.callsignAssigned = False
        elif service == "APRS_Radio_Serial":
            if self.APRS_Radio_comboBox_Callsign.currentIndex() != 0:
                self.callsignAssigned = True
                print("Assigned callsign: " + str(self.APRS_Radio_comboBox_Callsign.currentText()))
                if self.radioConnected and self.baudrate > 0:
                    if type(self.Balloon) is not type(None):
                        self.Balloon.stop()
                        time.sleep(1)
                    del self.Balloon
                    self.Balloon = Balloon_Coordinates_APRS_SerialTNC(service_type="APRS Serial TNC",
                                                                      callsign=self.APRS_Radio_comboBox_Callsign.currentText(),
                                                                      port=self.radio_port,
                                                                      baudrate=self.baud_rate)
                    testStr = self.Balloon.print_info()
                    self.statusBox.setPlainText(testStr)
            else:
                print("Select a balloon callsign from the list in APRS_Callsigns.csv")
                self.statusBox.setPlainText("Please select a balloon callsign from those listed in APRS_Callsigns.csv")
                self.callsignAssigned = False
        else:
            print("assignCallsign: This message should not be seen")

        return


    def assignAPRSfiKey(self):
        # this function checks if an APRS.fi API key has been entered
        # if an API key has been entered and a callsign has been selected, it creates an instance of the balloon coordinates APRS class
        # if an API key has not been entered or a callsign has not been selected, it simply returns

        if self.APRS_fi_lineEdit_APIKey.text() != "":
            self.APRSfiKeyAssigned = True
            print("Assigned APRS.fi API key: " + str(self.APRS_fi_lineEdit_APIKey.text()))
            if(self.callsignAssigned):
                if type(self.Balloon) is not type(None):
                    self.Balloon.stop()
                    time.sleep(1)
                del self.Balloon
                self.Balloon = Balloon_Coordinates_APRS_fi(service_type="APRS.fi",
                                                           callsign=self.APRS_fi_comboBox_Callsign.currentText(),
                                                           apikey=self.APRS_fi_lineEdit_APIKey.text())
                testStr = self.Balloon.print_info()
                self.statusBox.setPlainText(testStr)
        else:
            print("Enter an APRS.fi API key")
            self.statusBox.setPlainText("Please enter an APRS.fi API key")
            self.APRSfiKeyAssigned = False
        return
    
    def connectRadio_APRS_Radio(self):
        if self.APRS_Radio_comboBox_COMPort.currentText() != 0:
            if self.APRS_Radio_lineEdit_Baudrate.text() != 0 and str(self.APRS_Radio_lineEdit_Baudrate.text()).isdigit():
                self.radio_port = "/dev/" + str(self.APRS_Radio_comboBox_COMPort.currentText())
                self.baud_rate = int(self.APRS_Radio_lineEdit_Baudrate.text())
                self.radioConnected = True
                print("Radio connected")
                self.statusBox.setPlainText("Radio connected")
                if self.callsignAssigned:
                    print("Starting balloon position updater")
                    self.statusBox.setPlainText("Starting balloon position updater")
                    if type(self.Balloon) is not type(None):
                        self.Balloon.stop()
                        time.sleep(1)
                    del self.Balloon
                    self.Balloon = Balloon_Coordinates_APRS_SerialTNC(service_type="APRS Serial TNC",
                                                                      callsign=self.APRS_Radio_comboBox_Callsign.currentText(),
                                                                      port=self.radio_port,
                                                                      baud_rate=self.baud_rate)
                    testStr = self.Balloon.print_info()
                    self.statusBox.setPlainText(testStr)
            else:
                print("Enter a valid baud rate for the connected radio")
                self.statusBox.setPlainText("Enter a valid baud rate for the connected radio")
                self.radioConnected = False
        else:
            print("Select the device for the connected radio")
            self.statusBox.setPlainText("Select the device for the connected radio")
            self.radioConnected = False


    def refreshCOMPortLists(self):
        # this function searches the list of COM ports, and adds devices that it finds to the COM port comboboxes
        self.Arduino_comboBox_COMPort.clear()
        self.APRS_Radio_comboBox_COMPort.clear()
        self.ports = serial.tools.list_ports.comports()
        self.portNames = []
        self.comPortCounter = 0
        for port, desc, hwid in sorted(self.ports):
            # self.Arduino_comboBox_COMPort.addItem("[{}] {}: {}".format(i, port, desc))
            self.Arduino_comboBox_COMPort.addItem(desc)
            self.APRS_Radio_comboBox_COMPort.addItem(desc)
            self.portNames.append("{}".format(port))
            self.comPortCounter += 1


    def connectToArduino(self):
        # checks if arduino is selected, and if the connection is not already made, instantiates an instance of
        # the Ground_Station_Arduino class
        # if an arduino is connected, or one is not selected, the function returns
        if not self.arduinoConnected and self.Arduino_comboBox_COMPort.currentText():
            self.GSArduino = Ground_Station_Arduino(self.portNames[self.Arduino_comboBox_COMPort.currentIndex()], 9600)
            self.statusBox.setPlainText("connected to arduino!")
            self.arduinoConnected = True
        elif self.arduinoConnected:
            print("Arduino already connected")
            self.statusBox.setPlainText("Arduino already connected")
        else:
            self.statusBox.setPlainText("Unable to connect to Arduino")

        return


    def tiltUp(self):
        # if an arduino is connected, uses GSArduino to adjust the tilt up
        if self.arduinoConnected:
            self.GSArduino.adjustTiltUp(self.degreesPerClickBox.currentText())
            self.statusBox.setPlainText("adjusting tilt up " + self.degreesPerClickBox.currentText() + " degrees")
        else:
            print("Unable to connect to ground station motors")
            self.statusBox.setPlainText("Not connected to ground station motors")

        return


    def tiltDown(self):
        # if an arduino is connected, uses GSArduino to adjust the tilt down
        if self.arduinoConnected:
            self.GSArduino.adjustTiltDown(self.degreesPerClickBox.currentText())
            self.statusBox.setPlainText("adjusting tilt down " + self.degreesPerClickBox.currentText() + " degrees")
        else:
            print("Unable to connect to ground station motors")
            self.statusBox.setPlainText("Not connected to ground station motors")

        return


    def panCounterClockwise(self):
        # if an arduino is connected, uses GSArduino to adjust the pan counter-clockwise
        if self.arduinoConnected:
            self.GSArduino.adjustPanNegative(self.degreesPerClickBox.currentText())
            self.statusBox.setPlainText("adjusting pan " + self.degreesPerClickBox.currentText() + " degrees negative")
        else:
            print("Unable to connect to ground station motors")
            self.statusBox.setPlainText("Not connected to ground station motors")

        return


    def panClockwise(self):
        # if an arduino is connected, uses GSArduino to adjust the pan clockwise
        if self.arduinoConnected:
            self.GSArduino.adjustPanPositive(self.degreesPerClickBox.currentText())
            self.statusBox.setPlainText("adjusting pan " + self.degreesPerClickBox.currentText() + " degrees positive")
        else:
            print("Unable to connect to ground station motors")
            self.statusBox.setPlainText("Not connected to ground station motors")

        return


    def getGSLocation(self):
        # if the arduino is connected, this uses a GPS shield to request the coordinates of the ground station
        # it includes a check to ensure that the shield has a gps lock
        # if all conditions are met, the location of the ground station is set
        # if something fails, the function returns and nothing is set
        if self.arduinoConnected:
            check = self.GSArduino.warm_start()
            if not check:  # if the coords cannot be retrieved, return
                print("Failed to get GPS coords, please try again")
                self.statusBox.setPlainText("Failed to get GPS coordinates, please try again")
                return
            time.sleep(.25)

            GSCoords = self.GSArduino.req_GPS()
            self.GSArduino.print_GPS()
            self.GSLat = GSCoords[0]
            self.GSLong = GSCoords[1]
            self.GSAlt = GSCoords[2]

            self.GSLatBox.setText(str(self.GSLat))
            self.GSLongBox.setText(str(self.GSLong))
            self.GSAltBox.setText(str(self.GSAlt))
        else:
            print("arduino not connected")
            self.statusBox.setPlainText("Arduino not connected")

        return


    def setGSLocation(self):
        # this ensures that the arduino is connected, and valid text is present in the gs location text boxes
        # if the values present can be converted to floats, the starting location of the gs is set
        try:
            if self.arduinoConnected:
                latStr = self.GSLatBox.text()
                latStr = latStr.strip()
                self.GSLat = float(latStr)

                print(self.GSLat)

                longStr = self.GSLongBox.text()
                self.GSLong = float(longStr)
                print(self.GSLong)

                altStr = self.GSAltBox.text()
                self.GSAlt = float(altStr)
                print(self.GSAlt)

                self.statusBox.setPlainText("Ground station location entered successfully!")
                self.GSLocationSet = True
            else:
                self.statusBox.setPlainText("Please connect arduino")
                self.GSLocationSet = False
        except ValueError:
            print("numbers only for GPS location (decimal degrees)")
            self.statusBox.setPlainText("Invalid GPS location entered. Please only enter numbers")


    def getStartingPos(self):
        # this makes a call to sunposition to calculate the azimuth and elevation of the sun at the current location
        # of the ground station
        # it populates the starting aziumth and elevation boxes
        if self.GSLocationSet:
            now = datetime.utcnow()
            az, elev = sunpos(now, self.GSLat, self.GSLong, self.GSAlt)[:2]  # discard RA, dec, H

            self.startingAzimuth = az
            self.startingElevation = elev

            self.startingAzimuthBox.setText(str(az))
            self.startingElevationBox.setText(str(elev))

        else:
            self.statusBox.setPlainText("Please set ground station location "
                                              "and point at the sun using solar sight")

        return


    def calibrate(self):
        # sends the GSArduino class the starting azimuth and elevation
        if self.arduinoConnected:
            try:
                startingAzimuthStr = self.startingAzimuthBox.text()
                startingAzimuth = float(startingAzimuthStr)
                print(startingAzimuth)

                startingElevationStr = self.startingElevationBox.text()
                startingElevation = float(startingElevationStr)
                print(startingElevation)

                self.GSArduino.calibrate(startingAzimuth, startingElevation)
                self.calibrated = True
                self.statusBox.setPlainText("Successfully calibrated!")
            except ValueError:
                print("numbers only for initial azimuth and elevation")
                self.statusBox.setPlainText("Invalid input for initial azimuth and elevation")
        else:
            print("not connected to arduino")
            self.statusBox.setPlainText("Not connected to arduino")

        return


    def returnToSun(self):
        if self.arduinoConnected and self.GSLocationSet and self.calibrated:
            now = datetime.utcnow()
            az, elev = sunpos(now, self.GSLat, self.GSLong, self.GSAlt)[:2]  # discard RA, dec, H

            self.GSArduino.move_position(az, elev)

            self.startingAzimuth = az
            self.startingElevation = elev

            self.startingAzimuthBox.setText(str(self.startingAzimuth))
            self.startingElevationBox.setText(str(self.startingElevation))
            self.statusBox.setPlainText("at new sun position")

        else:
            self.statusBox.setPlainText("Ensure that arduino is connected, GS location is set and calibration is set")
            print("Cannot point back at the sun")

        return
    

    def returnToZero(self):
        if self.arduinoConnected and self.GSLocationSet and self.calibrated:
            self.GSArduino.move_position(0, 0)

            self.statusBox.setPlainText("at zero position")

        else:
            self.statusBox.setPlainText("Ensure that arduino is connected, GS location is set and calibration is set")
            print("Cannot point back at zero position")

        return


    def setPredictTrack(self):
        # sets the predict track bool variable
        # then calls the checkIfReady function to ensure all conditions to track have been met
        self.predictingTrack = True
        self.checkIfReady()
        return


    def checkIfReady(self):
        # this function ensures that all conditions to track have been met
        # if they have been, it calls the appropriate function to either start tracking with/without predictions
        if self.calibrated:
            print("Calibrated!")
        else:
            print("starting position not set")
            self.statusBox.setPlainText("Please set staring azimuth and elevation")

        if self.GSLocationSet:
            print("Ground station location set!")
        else:
            print("Ground Station Location not assigned")
            self.statusBox.setPlainText("Ground Station location not assigned")

        if self.IMEIAssigned:
            print("IMEI assigned")
        elif self.callsignAssigned:
            print("Callsign assigned")
            if type(self.Balloon) is Balloon_Coordinates_APRS_fi and self.APRSfiKeyAssigned:
                print("APRS.fi API key assigned")
            elif type(self.Balloon) is Balloon_Coordinates_APRS_IS:
                pass
            elif type(self.Balloon) is Balloon_Coordinates_APRS_SDR:
                pass
            elif type(self.Balloon) is Balloon_Coordinates_APRS_SerialTNC:
                pass
            else:
                print("APRS.fi API key not assigned")
                self.statusBox.setPlainText("Please set an APRS.fi API key to use APRS")
        else:
            print("Neither IMEI nor Callsign assigned")
            self.statusBox.setPlainText("Neither IMEI nor Callsign assigned")

        if self.arduinoConnected:
            print("Arduino connected!")
        else:
            print("Please connect to the Arduino")
            self.statusBox.setPlainText("Please connect to the Arduino")

        print("\n")

        if(   (self.arduinoConnected and self.IMEIAssigned and self.calibrated and self.GSLocationSet)
           or (self.arduinoConnected and self.callsignAssigned and self.APRSfiKeyAssigned and self.calibrated and self.GSLocationSet)
           or (self.arduinoConnected and self.callsignAssigned and type(self.Balloon) is Balloon_Coordinates_APRS_IS and self.calibrated and self.GSLocationSet)
           or (self.arduinoConnected and self.callsignAssigned and type(self.Balloon) is Balloon_Coordinates_APRS_SDR and self.calibrated and self.GSLocationSet)
           or (self.arduinoConnected and self.callsignAssigned and type(self.Balloon) is Balloon_Coordinates_APRS_SerialTNC and self.calibrated and self.GSLocationSet)):
            
            if self.predictingTrack:
                self.statusBox.setPlainText("Starting tracking with predictions!")
                self.callPredictTrack()
            else:
                self.statusBox.setPlainText("Starting tracking!")
                print("starting tracking!")
                self.callTrack()
                return True
        else:
            print("not ready to track yet")
            return False


    def callTrack(self):
        # sets up the qt thread to start tracking, and starts the thread
        self.tracking = True
        self.statusBox.setPlainText("Tracking!")
        self.trackThread = QThread()
        self.worker = Worker()

        self.worker.moveToThread(self.trackThread)

        self.trackThread.started.connect(self.worker.track)

        self.worker.finished.connect(self.trackThread.quit)  # pycharm has bug, this is correct
        self.worker.finished.connect(self.worker.deleteLater)  # https://youtrack.jetbrains.com/issue/PY-24183?_ga=2.240219907.1479555738.1625151876-2014881275.1622661488
        self.trackThread.finished.connect(self.trackThread.deleteLater)

        self.startButton.setEnabled(False)
        self.predictionStartButton.setEnabled(False)
        self.calibrateButton.setEnabled(False)

        self.trackThread.start()


    def callPredictTrack(self):
        # sets up the qt thread to start tracking with predictions and starts the thread
        self.statusBox.setPlainText("Tracking with predictions!")
        print("In predictTrack call")
        self.tracking = True
        self.trackThread = QThread()
        self.worker = Worker()

        self.worker.moveToThread(self.trackThread)

        self.trackThread.started.connect(self.worker.predictTrack)

        self.worker.finished.connect(self.trackThread.quit)  # pycharm has bug, this is correct
        self.worker.finished.connect(self.worker.deleteLater)  # https://youtrack.jetbrains.com/issue/PY-24183?_ga=2.240219907.1479555738.1625151876-2014881275.1622661488
        self.trackThread.finished.connect(self.trackThread.deleteLater)

        self.startButton.setEnabled(False)
        self.predictionStartButton.setEnabled(False)
        self.calibrateButton.setEnabled(False)

        self.trackThread.start()


    def stopTracking(self):
        # this stops the tracking thread, thus stopping the tracking
        if self.tracking:
            self.tracking = False
            self.predictingTrack = False
            self.startButton.setEnabled(True)
            self.predictionStartButton.setEnabled(True)
            self.calibrateButton.setEnabled(True)
            self.statusBox.setPlainText("tracking stopped")
        return


    def EStop(self):
        if self.arduinoConnected:
            self.GSArduino.sendEStop()
            self.stopTracking()

            self.statusBox.setPlainText("E-Stop triggered \n Please recalibrate before starting again")
            print("E-Stopped must recalibrate before starting tracking")

        return


    def displayCalculations(self, distance, azimuth, elevation):
        # this displays the outputs from the tracking threads on the GUI
        self.distanceDisplay.setText(str(distance))
        self.azimuthDisplay.setText(str(azimuth))
        self.elevationDisplay.setText(str(elevation))
        return



class Worker(QObject):
    # worker class to track without making the GUI hang
    finished = pyqtSignal()

    calcSignal = pyqtSignal(float, float, float)

    i = 0

    def track(self):
        # basic tracking algorithm
        # checks for updated position every second
        # if a new position has been found, calculate the azimuth and elevation to point at the new location
        # send the motors a command to move to the new position
        last_Balloon_Coor = [0, 0, 0]

        while MainWindow.tracking:
            Balloon_Coor = MainWindow.Balloon.get_coor_alt()
            if Balloon_Coor == last_Balloon_Coor:
                time.sleep(1)
                continue
            # note that trackMath takes arguments as long, lat, altitude
            Tracking_Calc = trackMath(MainWindow.GSLong, MainWindow.GSLat, MainWindow.GSAlt, 
                                        Balloon_Coor[1], Balloon_Coor[0], Balloon_Coor[2])

            distance = Tracking_Calc.distance
            newElevation = Tracking_Calc.elevation()
            newAzimuth = Tracking_Calc.azimuth()

            print(str(self.i) + " Distance " + str(distance) + " Azimuth: " + str(newAzimuth) + ", Elevation: " + str(newElevation))

            self.calcSignal.connect(MainWindow.displayCalculations)  # this seems to happen a lot for some reason
            self.calcSignal.emit(distance, newAzimuth, newElevation)

            MainWindow.GSArduino.move_position(newAzimuth, newElevation)

            self.i += 1
            last_Balloon_Coor = Balloon_Coor

        print("All done!")
        print(str(self.i) + " location updates processed")
        self.finished.emit()  # same pycharm bug as above
        return


    def predictTrack(self):
        # check for new location from server
        # if the new location is still the same, go to prediction
        # if there is new location, go to that latest location

        # find the difference between the latest lat/long location and the one before the last one and time
        # using last vertical velocity/altitude, find difference between altitudes/new altitude after ~1 second
        # find the amount that the position is changing each ~second
        # input the new predicted lat/long/alt into math equations to get new azimuth/elevation
        # go to the predicted elevation/azimuth

        print("In predictTrack")

        timer = time.time()
        newestLocation = MainWindow.Balloon.get_coor_alt()
        oldLocation = MainWindow.Balloon.get_coor_alt()
        i = 1

        calculations = open("predictedOutput.csv", "w")
        csvWriter = csv.writer(calculations)
        calcFields = ["Distance", "Azimuth", "Elevation", "r/p"]
        csvWriter.writerow(calcFields)

        azimuthList = []
        elevationList = []

        while MainWindow.predictingTrack:
            if (time.time() - timer) > 1:
                timer = time.time()
                currData = MainWindow.Balloon.get_coor_alt()

                if newestLocation == currData:
                    # need to predict!
                    print("predicted output")
                    timeDelta = MainWindow.Balloon.getTimeDiff()
                    print("The time delta is: " + str(timeDelta))
                    latStep = (newestLocation[0] - oldLocation[0]) / timeDelta
                    longStep = (newestLocation[1] - oldLocation[1]) / timeDelta
                    altStep = (newestLocation[2] - oldLocation[2]) / timeDelta

                    Tracking_Calc = trackMath(MainWindow.GSLong, MainWindow.GSLat, MainWindow.GSAlt,
                                              newestLocation[1] + (i * longStep), newestLocation[0] + (i * latStep), newestLocation[2] + (i * altStep))

                    distance = Tracking_Calc.distance
                    newElevation = Tracking_Calc.elevation()
                    newAzimuth = Tracking_Calc.azimuth()

                    elevationList.append(newElevation)
                    azimuthList.append(newAzimuth)

                    # keep average of azimuth/elevations
                    # if new calculation is outlier, throw it out, don't go to new spot
                    # reset average between pings
                    # alternatively, implement some type of filter (savitzky golay, kalman, etc)

                    """
                    if newElevation > np.mean(elevationList) + (2 * np.std(elevationList)) or newElevation < np.mean(elevationList) - (2 * np.std(elevationList)) \
                            or newAzimuth > np.mean(azimuthList) + (2 * np.std(azimuthList)) or newAzimuth < np.mean(azimuthList) - (2 * np.std(azimuthList)):
                        print("outlier detected! ")
                        pass
                    else:
                        print("distance: " + str(distance))
                        print("elevation: " + str(newElevation))
                        print("azimuth: " + str(newAzimuth) + "\n")
                    """
                    self.calcSignal.connect(MainWindow.displayCalculations)
                    self.calcSignal.emit(distance, newAzimuth, newElevation)

                    row = [distance, newAzimuth, newElevation, "p"]
                    csvWriter.writerow(row)

                    MainWindow.GSArduino.move_position(newAzimuth, newElevation)

                    i += 1

                else:
                    # go to the new actual spot
                    oldLocation = newestLocation
                    newestLocation = currData

                    # note that trackMath takes arguments as long, lat, altitude
                    Tracking_Calc = trackMath(MainWindow.GSLong, MainWindow.GSLat, MainWindow.GSAlt, currData[1],
                                              currData[0], currData[2])

                    distance = Tracking_Calc.distance
                    newElevation = Tracking_Calc.elevation()
                    newAzimuth = Tracking_Calc.azimuth()

                    self.calcSignal.connect(MainWindow.displayCalculations)  # this seems to happen a lot for some reason
                    self.calcSignal.emit(distance, newAzimuth, newElevation)

                    print("Got new real ping!")
                    print("distance: " + str(distance))
                    print("elevation: " + str(newElevation))
                    print("azimuth: " + str(newAzimuth) + "\n")

                    MainWindow.GSArduino.move_position(newAzimuth, newElevation)

                    row = [distance, newAzimuth, newElevation, "r"]
                    csvWriter.writerow(row)

                    i = 1
                    azimuthList = []
                    elevationList = []

        print("All done tracking with predictions! :)")
        calculations.close()
        self.finished.emit()
        return



if __name__ == "__main__":

    # standard pyqt5 main
    # sets up and shows the window

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = Window()
    # MainWindow.show()
    MainWindow.showMaximized()

    sys.exit(app.exec_())
