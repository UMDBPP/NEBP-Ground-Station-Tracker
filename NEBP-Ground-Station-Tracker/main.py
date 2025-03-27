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
from PyQt5.QtCore import QThread, QObject, pyqtSignal, Qt, QUrl
from PyQt5.QtWidgets import QCompleter, QApplication, QTreeWidgetItem
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEngineSettings
from Ground_Station_GUI import Ui_MainWindow
import sys
from Balloon_Coordinates import Balloon_Coordinates_Borealis, Balloon_Coordinates_APRS_fi, Balloon_Coordinates_APRS_IS, Balloon_Coordinates_APRS_SDR, Balloon_Coordinates_APRS_SerialTNC, Balloon_Coordinates_Test
from satelliteTrackingMath import trackMath
from Ground_Station_Arduino import Ground_Station_Arduino
import serial.tools.list_ports
import time
from sunposition import sunpos
from datetime import datetime
import csv

# todo: clean up this code and better document

# ======================================
import numpy as np

from matplotlib.backends.backend_qtagg import FigureCanvas
# from matplotlib.backends.backend_qtagg import \
#     NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

from matplotlib.dates import DateFormatter
from pathlib import Path
# ======================================
# Better to use a proper HTTP server with actual security, but this keeps things self-contained and is trivial to set up and run
import http.server
import socketserver

import atexit

import faulthandler

faulthandler.enable(all_threads=True)


class Window(QtWidgets.QMainWindow, Ui_MainWindow):
    # This class connects functionality to buttons in the GUI
    # The "main" class
    def __init__(self):
        super(Window, self).__init__()

        self.setupUi(self)

        # self.refreshModems()
        self.refreshCallsigns()

        self.arduinoConnected = False
        self.modemAssigned = False
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

        self.ports = None
        self.portNames = []
        self.comPortCounter = 0
        self.refreshCOMPortLists()

        self.latest_update = {}
        self.updateThread = None
        self.updateWorker = None
        self.updating = False


        self.Borealis_button_RefreshModem.clicked.connect(self.refreshModems)
        self.Borealis_button_ConfirmModem.clicked.connect(self.assignModem)
        self.APRS_fi_button_ConfirmCallsign.clicked.connect(self.assignCallsign_APRS_fi)
        self.APRS_fi_button_ConfirmAPIKey.clicked.connect(self.assignAPRSfiKey)
        self.APRS_IS_button_ConfirmCallsign.clicked.connect(self.assignCallsign_APRS_IS)
        self.APRS_Radio_button_ConfirmCallsign.clicked.connect(self.assignCallsign_APRS_Radio)
        self.APRS_Radio_button_refreshCOMPorts.clicked.connect(self.refreshCOMPortLists)
        self.APRS_Radio_button_ConnectRadio.clicked.connect(self.connectRadio_APRS_Radio)
        self.APRS_Radio_comboBox_COMPort.setCurrentIndex(self.comPortCounter - 1)
        self.Test_Source_button_Confirm.clicked.connect(self.init_test_source)

        self.Tracking_button_Refresh.clicked.connect(self.refreshTrackingStatus)
        self.Tracking_button_Test.clicked.connect(self.testTrackingStatus)
        self.Tracking_button_Reset.clicked.connect(self.resetTrackingStatus)
        self.Tracking_button_Stop.clicked.connect(self.stopTrackingStatus)

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

        self.serve_http = False
        self.http_port = 8000
        self.map_view = None
        self.map_view_loaded = False

        # Set the HTTP server to start and initialize the map the first time the Location/Orientation tab is opened
        self.tabWidget.currentChanged.connect(lambda index: self._start_http_server() or self._initialize_map() or self.tabWidget.currentChanged.disconnect() if index == 2 else None)

        self._initialize_plot()

        atexit.register(self.cleanup)


    def cleanup(self):
        # Stop embedded map's HTTP server
        self._stop_http_server()
        if self.httpServerThread and self.httpServerThread.isRunning():
            print("Waiting for HTTP server to stop")
            # Give the thread 5 seconds to stop itself
            if not self.httpServerThread.wait(5000) and self.httpServerThread.isRunning():
                # If it hasn't stopped in 5 seconds, try manually quiting the thread
                print("HTTP server hasn't stopped. Manually sending quit command...")
                self.httpServerThread.quit()
                if not self.httpServerThread.wait(5000) and self.httpServerThread.isRunning():
                    # If it hasn't stopped in 5 seconds, terminate the thread
                    print("HTTP server hasn't stopped. Terminating...")
                    self.httpServerThread.terminate()
                    # Give it another 5 seconds to terminate
                    if not self.httpServerThread.wait(5000) and self.httpServerThread.isRunning():
                        print("Error stopping HTTP server. HTTP server still running at program exit")
        
        # Stop tracking thread
        self.stopTracking()
        if self.trackThread and self.trackThread.isRunning():
            print("Waiting for tracking thread to stop")
            # Give the thread 5 seconds to stop itself
            if not self.trackThread.wait(5000) and self.trackThread.isRunning():
                # If it hasn't stopped in 5 seconds, try manually quiting the thread
                print("Tracking thread hasn't stopped. Manually sending quit command...")
                self.trackThread.quit()
                if not self.trackThread.wait(5000) and self.trackThread.isRunning():
                    # If it hasn't stopped in 5 seconds, terminate the thread
                    print("Tracking thread hasn't stopped. Terminating...")
                    self.trackThread.terminate()
                    # Give it another 5 seconds to terminate
                    if not self.trackThread.wait(5000) and self.trackThread.isRunning():
                        print("Error stopping tracking thread. Tracking thread still running at program exit")
        
        # Stop updates thread
        self._stop_updating()
        if self.updateThread and self.updateThread.isRunning():
            print("Waiting for position update thread to stop")
            # Give the thread 5 seconds to stop itself
            if not self.updateThread.wait(5000) and self.updateThread.isRunning():
                # If it hasn't stopped in 5 seconds, try manually quiting the thread
                print("Position update thread hasn't stopped. Manually sending quit command...")
                self.updateThread.quit()
                if not self.updateThread.wait(5000) and self.updateThread.isRunning():
                    # If it hasn't stopped in 5 seconds, terminate the thread
                    print("Position update thread hasn't stopped. Terminating...")
                    self.updateThread.terminate()
                    # Give it another 5 seconds to terminate
                    if not self.updateThread.wait(5000) and self.updateThread.isRunning():
                        print("Error stopping position update thread. Position update thread still running at program exit")
        return


    # Debug test source replaying logged balloon positions
    def init_test_source(self):
        self.Balloon = Balloon_Coordinates_Test("TEST", int(self.Test_Source_spinBox_Period.value()))
        testStr = self.Balloon.print_info()
        self.statusBox.setPlainText(testStr)
        self._start_updating()
        return
    

    def _map_onLoadStarted(self):
        self.map_view_loaded = False


    def _map_onLoadFinished(self, ok):
        if ok:
            self.map_view_loaded = True


    # Initialize position map and altitude graph
    def _initialize_map(self, gs_lon:float=-77.5, gs_lat:float=39.5):
        # If the map_view hasn't been created yet, make it
        if not self.map_view:
            # Create web view for Leaflet map
            self.map_view = QWebEngineView()
            self.map_view.settings().setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True)
            self.visualization_map_grid.addWidget(self.map_view)
            self.map_view.setUrl(QUrl("http://127.0.0.1:" + str(self.http_port)))

            # self.map_view_dev = QWebEngineView()
            # self.visualization_map_grid.addWidget(self.map_view_dev)
            # self.map_view.page().setDevToolsPage(self.map_view_dev.page())

            self.map_view.loadStarted.connect(self._map_onLoadStarted)
            self.map_view.loadFinished.connect(self._map_onLoadFinished)
        
        # If the map is loaded,
        if self.map_view_loaded:
            # Center map on ground station location
            self.map_view.page().runJavaScript("setGroundStationLocation(" + str(gs_lat) + ", " + str(gs_lon) + ");")
            self.map_view.page().runJavaScript("MAP.flyTo(new L.LatLng(" + str(gs_lat) + ", " + str(gs_lon) + "));")

        return


    def _initialize_plot(self):
        # Create figure canvas for map and altitude graph
        self.plot_canvas = FigureCanvas(Figure(layout="constrained"))
        self.visualization_altitude_graph_grid.addWidget(self.plot_canvas)

        # Set altitude graph options
        self.altitude_ax = self.plot_canvas.figure.subplots(1, 1)
        self.altitude_ax.xaxis.set_major_formatter(DateFormatter("%H:%M:%S"))
        self.altitude_ax.minorticks_on()
        self.altitude_ax.grid()
        self.altitude_ax.set_ylabel("Altitude (m)")
        self.altitude_ax.set_xlabel("Time")

        # Draw figure
        self.plot_canvas.figure.canvas.draw()
        return


    # Refresh the Borealis_comboBox_modem modem list from the Borealis website
    def refreshModems(self):
        print("Refreshing modem list")
        self.statusBox.setPlainText("Refreshing modem list")
        self.modemList = Balloon_Coordinates_Borealis.list_modems()

        self.Borealis_comboBox_modem.clear()
        self.Borealis_comboBox_modem.addItem("")

        if self.modemList == []:
            print("Unable to get modem list")
            self.statusBox.setPlainText("Unable to get modem list")
        else:
            for i in range(len(self.modemList)):
                self.Borealis_comboBox_modem.addItem(self.modemList[i])

        completerModem = QCompleter(self.modemList)
        completerModem.setFilterMode(Qt.MatchContains)
        completerModem.setCaseSensitivity(Qt.CaseInsensitive)
        self.Borealis_comboBox_modem.setEditable(True)
        self.Borealis_comboBox_modem.setCompleter(completerModem)
        return


    def refreshTrackingStatus(self):
        print("Refreshing position status")
        self.statusBox.setPlainText("Refreshing position status")
        if type(self.Balloon) is not type(None):
            testStr = self.Balloon.print_info()
            self.statusBox.setPlainText(testStr)
        return
    

    def testTrackingStatus(self):
        print("Testing position update")
        self.statusBox.setPlainText("Testing position update")
        if type(self.Balloon) is not type(None):
            status = self.Balloon.test()
            if status == 0:
                print("Position update process is receiving updates")
                self.statusBox.setPlainText("Position update process is receiving updates")
            elif status == 1:
                print("Position update process is running, but no positions have been received yet")
                self.statusBox.setPlainText("Position update process is running, but no positions have been received yet")
            elif status == -1:
                print("Position update process is not running at test start")
                self.statusBox.setPlainText("Position update process is not running at test start")
            elif status == -2:
                print("Position update process is not running at test end")
                self.statusBox.setPlainText("Position update process is not running at test end")
            else:
                print("Unexpected status received from reset function: " + str(status))
                self.statusBox.setPlainText("Unexpected status received from reset function: " + str(status))
        return

    
    def resetTrackingStatus(self):
        print("Reseting position update process")
        self.statusBox.setPlainText("Reseting position update process")
        if type(self.Balloon) is not type(None):
            status = self.Balloon.reset()
            if status == 0:
                print("Position update process successfully restarted and receiving")
                self.statusBox.setPlainText("Position update process successfully restarted and receiving")
            elif status == 1:
                print("Position update process successfully restarted, but no positions have been received yet")
                self.statusBox.setPlainText("Position update process successfully restarted, but no positions have been received yet")
            elif status == -1:
                print("Position update process is not running")
                self.statusBox.setPlainText("Position update process is not running")
            elif status == -2:
                print("Position update process stopped, but failed to restart")
                self.statusBox.setPlainText("Position update process stopped, but failed to restart")
            else:
                print("Unexpected status received from reset function: " + str(status))
                self.statusBox.setPlainText("Unexpected status received from reset function: " + str(status))
        return
        

    def stopTrackingStatus(self):
        print("Stopping position update process")
        self.statusBox.setPlainText("Stopping position update process")
        if type(self.Balloon) is not type(None):
            self._stop_updating()
            self.Balloon.stop()
            status = self.Balloon.test()
            if status == 0:
                print("Position update process still running and receiving after post-stop test")
                self.statusBox.setPlainText("Position update process still running and receiving after post-stop test")
            elif status == 1:
                print("Position update process still running after post-stop test")
                self.statusBox.setPlainText("Position update process still running after post-stop test")
            elif status == -1 or status == -2:
                print("Position update process stopped")
                self.statusBox.setPlainText("Position update process stopped")
            else:
                print("Unexpected status received from reset function: " + str(status))
                self.statusBox.setPlainText("Unexpected status received from reset function: " + str(status))
        return
    

    def assignModem(self):
        # this function checks if an modem has been selected
        # if an modem has been selected, it creates an instance of the balloon coordinates class using the modem
        # if an modem has not been selected, it simply returns
        if self.Borealis_comboBox_modem.currentIndex() != 0:  # SHOULD BE != FOR BOREALIS WEBSITE!
            self.modemAssigned = True
            print(self.Borealis_comboBox_modem.currentText())        
            if type(self.Balloon) is not type(None):
                self.Balloon.stop()
                time.sleep(1)
            del self.Balloon
            self.Balloon = Balloon_Coordinates_Borealis(service_type="Borealis",
                                                        modem=str(self.Borealis_comboBox_modem.currentText()).split()[0])
            testStr = self.Balloon.print_info()
            self.statusBox.setPlainText(testStr)
            self._start_updating()
            # self.Balloon.getTimeDiff()
        else:
            print("select a balloon ")
            self.statusBox.setPlainText("Please select a balloon modem")
            self.modemAssigned = False
        return


    def refreshCallsigns(self):
        print("Refreshing callsign list")
        self.statusBox.setPlainText("Refreshing callsign list")
        self.callsignList = Balloon_Coordinates_APRS_IS.list_callsigns()
        tmpList = list(self.callsignList)

        self.APRS_fi_comboBox_Callsign.clear()
        self.APRS_IS_comboBox_Callsign.clear()
        self.APRS_Radio_comboBox_Callsign.clear()

        self.APRS_fi_comboBox_Callsign.addItem("")
        self.APRS_IS_comboBox_Callsign.addItem("")
        self.APRS_Radio_comboBox_Callsign.addItem("")
        
        for idx, callsign in enumerate(self.callsignList):
            if len(callsign.split("-")) == 1 and self.APRS_IS_comboBox_Callsign.findText(callsign.split("-")[0]) == -1:
                self.APRS_IS_comboBox_Callsign.addItem(callsign)
                self.APRS_Radio_comboBox_Callsign.addItem(callsign)
            else:
                if self.APRS_IS_comboBox_Callsign.findText(callsign.split("-")[0]) == -1:
                    self.APRS_IS_comboBox_Callsign.addItem(callsign.split("-")[0])
                    self.APRS_Radio_comboBox_Callsign.addItem(callsign.split("-")[0])
                    tmpList.append(callsign.split("-")[0])
                self.APRS_IS_comboBox_Callsign.addItem(callsign)
                self.APRS_Radio_comboBox_Callsign.addItem(callsign)
                self.APRS_fi_comboBox_Callsign.addItem(callsign)

        completerCallsign = QCompleter(tmpList)
        completerCallsign.setFilterMode(Qt.MatchContains)
        completerCallsign.setCaseSensitivity(Qt.CaseInsensitive)
        self.APRS_fi_comboBox_Callsign.setEditable(True)
        self.APRS_fi_comboBox_Callsign.setCompleter(completerCallsign)
        self.APRS_IS_comboBox_Callsign.setEditable(True)
        self.APRS_IS_comboBox_Callsign.setCompleter(completerCallsign)
        self.APRS_Radio_comboBox_Callsign.setEditable(True)
        self.APRS_Radio_comboBox_Callsign.setCompleter(completerCallsign)
        return


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
                    self._start_updating()
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
                self._start_updating()
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
                self._start_updating()
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
                    self._start_updating()
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
                self._start_updating()
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
                    self._start_updating()
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
        for port, desc, hwid in sorted(self.ports, reverse=True):
            # self.Arduino_comboBox_COMPort.addItem("[{}] {}: {}".format(i, port, desc))
            self.Arduino_comboBox_COMPort.addItem(port)
            self.APRS_Radio_comboBox_COMPort.addItem(port)
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
        # this ensures that valid text is present in the gs location text boxes
        # if the values present can be converted to floats, the starting location of the gs is set
        try:
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

            self._initialize_map(self.GSLong, self.GSLat)
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

        if self.modemAssigned:
            print("Modem assigned")
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
        elif type(self.Balloon) is Balloon_Coordinates_Test:
            pass
        else:
            print("Neither Modem nor Callsign assigned")
            self.statusBox.setPlainText("Neither Modem nor Callsign assigned")

        if self.arduinoConnected:
            print("Arduino connected!")
        else:
            print("Please connect to the Arduino")
            self.statusBox.setPlainText("Please connect to the Arduino")

        print("\n")

        if(   (self.arduinoConnected and self.modemAssigned and self.calibrated and self.GSLocationSet)
           or (self.arduinoConnected and self.callsignAssigned and self.APRSfiKeyAssigned and self.calibrated and self.GSLocationSet)
           or (self.arduinoConnected and self.callsignAssigned and type(self.Balloon) is Balloon_Coordinates_APRS_IS and self.calibrated and self.GSLocationSet)
           or (self.arduinoConnected and self.callsignAssigned and type(self.Balloon) is Balloon_Coordinates_APRS_SDR and self.calibrated and self.GSLocationSet)
           or (self.arduinoConnected and self.callsignAssigned and type(self.Balloon) is Balloon_Coordinates_APRS_SerialTNC and self.calibrated and self.GSLocationSet)
           or (self.arduinoConnected and type(self.Balloon) is Balloon_Coordinates_Test and self.calibrated and self.GSLocationSet)):
            
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
        '''self.trackThread = QThread()
        self.worker = Worker_tracking()

        self.worker.moveToThread(self.trackThread)

        self.trackThread.started.connect(self.worker.track)

        self.worker.finished.connect(self.trackThread.quit)  # pycharm has bug, this is correct
        self.worker.finished.connect(self.worker.deleteLater)  # https://youtrack.jetbrains.com/issue/PY-24183?_ga=2.240219907.1479555738.1625151876-2014881275.1622661488
        self.trackThread.finished.connect(self.trackThread.deleteLater)'''

        self.startButton.setEnabled(False)
        self.predictionStartButton.setEnabled(False)
        self.calibrateButton.setEnabled(False)
        '''
        self.trackThread.start()'''


    def callPredictTrack(self):
        # sets up the qt thread to start tracking with predictions and starts the thread
        self.statusBox.setPlainText("Tracking with predictions!")
        print("In predictTrack call")
        self.tracking = True
        self.trackThread = QThread()
        self.worker = Worker_tracking()

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


    # Function to update visualizations of received data (plot received coordinates on the position map and altitude graph)
    def _update_visualizations(self, latest_time:float, lon:float, lat:float, alt:float):
        # Record balloon locations and plot on map
        # xpt,ypt = self.basemap(lon, lat)
        # self.xpt.append(xpt)
        # self.ypt.append(ypt)
        # self.basemap.plot(self.xpt,self.ypt,'bo-')
        # self.basemap.plot(self.xpt[-1],self.ypt[-1],'yo')

        # Plot latest altitude on graph
        self.altitude_ax.plot(datetime.fromtimestamp(latest_time), alt, "bo-")
        # Draw figure
        self.plot_canvas.figure.canvas.draw()


    # Function to update receivedUpdates tree widget
    def _update_receivedUpdates_tree(self, update_time:str, time_diff:str, lat:str, long:str, alt:str, comment:str):
        self.receivedUpdates_treeWidget.scrollToItem(QTreeWidgetItem(self.receivedUpdates_treeWidget, [update_time, time_diff, lat, long, alt, comment]))


    # Function to update local updates dict and run other GUI updates
    def _receive_updates(self, updateData:dict):
        self.latest_update = updateData
        latest_tm = time.localtime(updateData['time'])
        time_str = "{:02.0f}:{:02.0f}:{:02.0f}".format(latest_tm[3], latest_tm[4], latest_tm[5])
        time_diff = self.Balloon.getTimeDiff()
        
        self._update_receivedUpdates_tree(time_str, "{:.0f}".format(time_diff), "{:.2f}".format(updateData['lat']), "{:.2f}".format(updateData['long']), "{:.1f}".format(updateData['alt']), str(updateData['comment']))
        self._update_visualizations(updateData['time'], updateData['long'], updateData['lat'], updateData['alt'])

        if self.tracking:
            # note that trackMath takes arguments as long, lat, altitude
            Tracking_Calc = trackMath(self.GSLong, self.GSLat, self.GSAlt, 
                                        updateData['long'], updateData['lat'], updateData['alt'])

            distance = Tracking_Calc.distance
            newElevation = Tracking_Calc.elevation()
            newAzimuth = Tracking_Calc.azimuth()

            print("Distance " + str(distance) + " Azimuth: " + str(newAzimuth) + ", Elevation: " + str(newElevation))

            self.displayCalculations(distance, newAzimuth, newElevation)

            self.GSArduino.move_position(newAzimuth, newElevation)


    # Start QThread for getting position updates
    def _start_updating(self):
        # sets up the qt thread to start updating, and starts the thread
        self.updating = True
        self.statusBox.setPlainText("Updating from tracking source")
        self.updateThread = QThread()
        self.updateWorker = Worker_updates()

        self.updateWorker.moveToThread(self.updateThread)

        self.updateThread.started.connect(self.updateWorker.receive_updates)

        self.updateWorker.finished.connect(self.updateThread.quit)  # pycharm has bug, this is correct
        self.updateWorker.finished.connect(self.updateWorker.deleteLater)  # https://youtrack.jetbrains.com/issue/PY-24183?_ga=2.240219907.1479555738.1625151876-2014881275.1622661488
        self.updateThread.finished.connect(self.updateThread.deleteLater)

        self.updateThread.start()


    # Stop Qthread for getting position updates
    def _stop_updating(self):
        if self.updating:
            self.updating = False
        return
    

    # Start QThread for serving HTTP requests for the embedded map
    def _start_http_server(self):
        # sets up the qt thread, and starts the thread
        self.serve_http = True
        self.statusBox.setPlainText("Starting embedded map's HTTP server")
        print("Starting embedded map's HTTP server")
        self.httpServerThread = QThread()
        self.httpServerWorker = Worker_http()

        self.httpServerWorker.moveToThread(self.httpServerThread)

        self.httpServerThread.started.connect(self.httpServerWorker.serve_requests)

        self.httpServerWorker.finished.connect(self.httpServerThread.quit)  # pycharm has bug, this is correct
        self.httpServerWorker.finished.connect(self.httpServerWorker.deleteLater)  # https://youtrack.jetbrains.com/issue/PY-24183?_ga=2.240219907.1479555738.1625151876-2014881275.1622661488
        self.httpServerThread.finished.connect(self.httpServerThread.deleteLater)

        self.httpServerThread.start()
        return


    # Stop Qthread for getting position updates
    def _stop_http_server(self):
        if self.serve_http:
            self.serve_http = False
        return
    


class Worker_http(QObject):
    # worker class to run the HTTP server for the embedded map without making the program hang
    finished = pyqtSignal()

    # Create handler class to set top-level directory to embedded_map
    class Handler(http.server.SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=str(Path(__file__).parent / "../embedded_map/"), **kwargs)

    def serve_requests(self):
        with socketserver.TCPServer(("127.0.0.1", MainWindow.http_port), self.Handler) as httpd:
            # Set timeout for waiting on incoming requests
            httpd.timeout = 1
            httpd.handle_timeout
            print("Serving local requests at port", MainWindow.http_port)
            while MainWindow.serve_http:
                httpd.handle_request()
                # print("tmp")

        print("Local HTTP server stopped")
        self.finished.emit()  # same pycharm bug as above
        return



class Worker_updates(QObject):
    # worker class to receive updates without making the GUI hang
    finished = pyqtSignal()

    update_signal = pyqtSignal(dict)

    i = 0

    def receive_updates(self):
        # checks for updated position every second
        # if a new position has been found, append to updates queue in main window class
        last_Balloon_Update = {}

        self.update_signal.connect(MainWindow._receive_updates)

        while MainWindow.updating:
            Balloon_Update = MainWindow.Balloon.get_latest_update()
            if last_Balloon_Update == Balloon_Update:
                time.sleep(1)
                continue
            
            self.update_signal.emit({'count':self.i} | Balloon_Update)

            self.i += 1
            last_Balloon_Update = Balloon_Update

            #============================


        print("All done!")
        print(str(self.i) + " location updates processed")
        self.finished.emit()  # same pycharm bug as above
        return
    pass



class Worker_tracking(QObject):
    # worker class to track without making the GUI hang
    finished = pyqtSignal()

    calcSignal = pyqtSignal(float, float, float)
    coor_signal = pyqtSignal(float, float, float, float)

    i = 0

    def track(self):
        # basic tracking algorithm
        # checks for updated position every second
        # if a new position has been found, calculate the azimuth and elevation to point at the new location
        # send the motors a command to move to the new position
        last_Balloon_Coor = [0, 0, 0]

        self.calcSignal.connect(MainWindow.displayCalculations)
        self.coor_signal.connect(MainWindow._update_visualizations)

        while MainWindow.tracking:
            Balloon_Coor = [MainWindow.latest_update["lat"], MainWindow.latest_update["long"], MainWindow.latest_update["alt"]]
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

            self.calcSignal.emit(distance, newAzimuth, newElevation)
            self.coor_signal.emit(MainWindow.Balloon.get_latest_timestamp(), Balloon_Coor[1], Balloon_Coor[0], Balloon_Coor[2])

            MainWindow.GSArduino.move_position(newAzimuth, newElevation)

            self.i += 1
            last_Balloon_Coor = Balloon_Coor

            #============================


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
