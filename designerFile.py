# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '.\QtDesignerUI.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(945, 638)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.tiltUpButton = QtWidgets.QPushButton(self.centralwidget)
        self.tiltUpButton.setGeometry(QtCore.QRect(660, 440, 91, 41))
        self.tiltUpButton.setObjectName("tiltUpButton")
        self.tiltDownButton = QtWidgets.QPushButton(self.centralwidget)
        self.tiltDownButton.setGeometry(QtCore.QRect(650, 510, 111, 51))
        self.tiltDownButton.setObjectName("tiltDownButton")
        self.panLeftButton = QtWidgets.QPushButton(self.centralwidget)
        self.panLeftButton.setGeometry(QtCore.QRect(550, 460, 91, 51))
        self.panLeftButton.setObjectName("panLeftButton")
        self.panRightButton = QtWidgets.QPushButton(self.centralwidget)
        self.panRightButton.setGeometry(QtCore.QRect(770, 450, 71, 71))
        self.panRightButton.setObjectName("panRightButton")
        self.IMEIComboBox = QtWidgets.QComboBox(self.centralwidget)
        self.IMEIComboBox.setGeometry(QtCore.QRect(0, 0, 281, 121))
        self.IMEIComboBox.setObjectName("IMEIComboBox")
        self.IMEIComboBox.addItem("")
        self.COMPortComboBox = QtWidgets.QComboBox(self.centralwidget)
        self.COMPortComboBox.setGeometry(QtCore.QRect(0, 170, 241, 101))
        self.COMPortComboBox.setObjectName("COMPortComboBox")
        self.COMPortComboBox.addItem("")
        self.startButton = QtWidgets.QPushButton(self.centralwidget)
        self.startButton.setGeometry(QtCore.QRect(20, 420, 131, 61))
        self.startButton.setObjectName("startButton")
        self.connectToArduinoButton = QtWidgets.QPushButton(self.centralwidget)
        self.connectToArduinoButton.setGeometry(QtCore.QRect(250, 230, 101, 41))
        self.connectToArduinoButton.setObjectName("connectToArduinoButton")
        self.stopTrackingButton = QtWidgets.QPushButton(self.centralwidget)
        self.stopTrackingButton.setGeometry(QtCore.QRect(20, 500, 131, 51))
        self.stopTrackingButton.setObjectName("stopTrackingButton")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(480, 20, 131, 41))
        self.label.setObjectName("label")
        self.confirmGSLocationButton = QtWidgets.QPushButton(self.centralwidget)
        self.confirmGSLocationButton.setGeometry(QtCore.QRect(840, 130, 75, 23))
        self.confirmGSLocationButton.setObjectName("confirmGSLocationButton")
        self.GSLatBox = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.GSLatBox.setGeometry(QtCore.QRect(620, 20, 151, 31))
        self.GSLatBox.setObjectName("GSLatBox")
        self.GSLongBox = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.GSLongBox.setGeometry(QtCore.QRect(620, 70, 161, 31))
        self.GSLongBox.setObjectName("GSLongBox")
        self.GSAltBox = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.GSAltBox.setGeometry(QtCore.QRect(620, 120, 161, 31))
        self.GSAltBox.setObjectName("GSAltBox")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(470, 70, 131, 21))
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(480, 120, 131, 21))
        self.label_3.setObjectName("label_3")
        self.startingAzimuthBox = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.startingAzimuthBox.setGeometry(QtCore.QRect(610, 210, 141, 31))
        self.startingAzimuthBox.setObjectName("startingAzimuthBox")
        self.startingElevationBox = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.startingElevationBox.setGeometry(QtCore.QRect(610, 260, 141, 31))
        self.startingElevationBox.setObjectName("startingElevationBox")
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setGeometry(QtCore.QRect(510, 210, 81, 31))
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setGeometry(QtCore.QRect(510, 260, 91, 41))
        self.label_5.setObjectName("label_5")
        self.calibrateButton = QtWidgets.QPushButton(self.centralwidget)
        self.calibrateButton.setGeometry(QtCore.QRect(790, 260, 121, 31))
        self.calibrateButton.setObjectName("calibrateButton")
        self.GPSRequestButton = QtWidgets.QPushButton(self.centralwidget)
        self.GPSRequestButton.setGeometry(QtCore.QRect(810, 80, 111, 41))
        self.GPSRequestButton.setObjectName("GPSRequestButton")
        self.confirmIMEIButton = QtWidgets.QPushButton(self.centralwidget)
        self.confirmIMEIButton.setGeometry(QtCore.QRect(290, 80, 91, 41))
        self.confirmIMEIButton.setObjectName("confirmIMEIButton")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 945, 21))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.tiltUpButton.setText(_translate("MainWindow", "Tilt Up"))
        self.tiltDownButton.setText(_translate("MainWindow", "Tilt Down"))
        self.panLeftButton.setText(_translate("MainWindow", "Pan Left"))
        self.panRightButton.setText(_translate("MainWindow", "Pan Right"))
        self.IMEIComboBox.setItemText(0, _translate("MainWindow", "Select IMEI"))
        self.COMPortComboBox.setItemText(0, _translate("MainWindow", "com port"))
        self.startButton.setText(_translate("MainWindow", "start tracking"))
        self.connectToArduinoButton.setText(_translate("MainWindow", "connect to arduino"))
        self.stopTrackingButton.setText(_translate("MainWindow", "Stop Tracking"))
        self.label.setText(_translate("MainWindow", "Ground Station Latitude: "))
        self.confirmGSLocationButton.setText(_translate("MainWindow", "set coords"))
        self.label_2.setText(_translate("MainWindow", "Ground Station Longitude"))
        self.label_3.setText(_translate("MainWindow", "Ground Station Altitude"))
        self.label_4.setText(_translate("MainWindow", "Starting Azimuth"))
        self.label_5.setText(_translate("MainWindow", "Starting Elevation"))
        self.calibrateButton.setText(_translate("MainWindow", "Set Starting Position"))
        self.GPSRequestButton.setText(_translate("MainWindow", "Request GPS coords"))
        self.confirmIMEIButton.setText(_translate("MainWindow", "Confirm IMEI"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())