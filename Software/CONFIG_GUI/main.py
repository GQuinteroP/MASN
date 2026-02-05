#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jul  4 11:49:16 2023

@author: ubuntu
"""

# from IPython import get_ipython
# get_ipython().magic('reset -sf')

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.uic import loadUi
import sys
import serial
from serial.tools.list_ports import comports
import numpy as np
import random
import struct 
from PyQt5.QtWidgets import QApplication
import os
from datetime import datetime
import csv
import time
from bitstring import BitStream, BitArray
import pandas as pd
import matplotlib.pyplot as plt

sys.path.insert(1, '../IoT_CoAP')
from Decode_data import decode_data_new, postgresql, decode_data

PID = 22336

EEPROM_PAGE_SIZE = 512
EEPROM_N_PAGES = 8191
#EEPROM_READ_BLOCK_LEN = 496

n_bytes_header = 15
n_bytes_payload	= 33
block_payloads	= 7

EEPROM_READ_BLOCK_LEN = 2*(n_bytes_header + n_bytes_payload*block_payloads)


HEADER = ['Leq', 'Date/Time', 'LAT', 'LON', 'Q', 'SAT #', #'25', '31.5', '40', '50', 
          '63', '80', '100', '125', '160', '200', '250', '315', '400', 
          '500', '630', '800', '1000', '1250', '1600', '2000', '2500', '3150', 
          '4000', '5000', '6300', '8000', '10000']

HEADER_PLOT = ['Leq', #'25', '31.5', '40', '50', 
               '63', '80', '100', '125', '160', '200', '250', '315', '400', 
          '500', '630', '800', '1000', '1250', '1600', '2000', '2500', '3150', 
          '4000', '5000', '6300', '8000', '10000']

class Window(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super().__init__()
        loadUi("main.ui", self)
        self.active_COM = False
        self.pBar_read.setMinimum(0)
        self.pBar_read.setValue(0)
        now = datetime.now()
        self.dTEdit_time.setDateTime(now)
        self.tEdit_file.setText(os.getcwd() + '/' + now.strftime("%d_%m_%Y-%H_%M_%S") + '.csv')
        self.lEdit_cmd.setText('AT+CPIN?')
        self.postgresql_handle = None
        self.plot = None
        self.connect_signals()
        self.show()
        self.activateWindow()
        
    def connect_signals(self):
        self.pB_connect.clicked.connect(self.pB_connect_clicked)
        self.pB_read_data.clicked.connect(self.pB_read_data_clicked)
        self.pB_clear_mem.clicked.connect(self.pB_clear_mem_clicked)
        self.pB_path.clicked.connect(self.pB_path_clicked)
        self.gBox_text.toggled.connect(self.gBox_text_checked)
        self.gBox_dB.toggled.connect(self.gBox_dB_checked)
        self.tWidget_config.currentChanged.connect(self.tW_config_changed)
        self.pButton_send_cmd.clicked.connect(self.pB_send_cmd_clicked)
        self.pButton_save_config.clicked.connect(self.send_config)
        
        self.gBox_text.clicked.connect(self.gBox_text_toggled)
        self.gBox_dB.clicked.connect(self.gBox_dB_toggled)
        self.cBox_plot.toggled.connect(self.cBox_plot_toggled)
        
        self.pButton_set_datetime.clicked.connect(self.pButton_set_datetime_clicked)
    
        self.pButton_restart_lpwa.clicked.connect(self.pButton_restart_lpwa_clicked)
    
    def pButton_restart_lpwa_clicked(self):
        if(self.active_COM):
            self.COM_handle.flushOutput()
            self.COM_handle.flushInput()
            output = bytes.fromhex('080000')
            self.COM_handle.write(output)
        
    def pB_clear_mem_clicked(self):
        if(self.active_COM):
            self.COM_handle.flushOutput()
            self.COM_handle.flushInput()
            output = bytes.fromhex('070000')
            self.COM_handle.write(output)
    
    def pButton_set_datetime_clicked(self):
        now = datetime.now()
        self.dTEdit_time.setDateTime(now)
        output = bytes.fromhex('06' + now.strftime("%H%M%S") + "%02d"%(now.weekday()+1) 
                                   + now.strftime("%y%m%d"))
                    
        try: 
            self.COM_handle.flushOutput()
            self.COM_handle.flushInput()
            self.COM_handle.write(output)  
            time.sleep(0.01)
        except TypeError as e:
            print("Failed to set datetime! ", e)
        
        print(output.hex())
        
    def gBox_text_toggled(self, status):
        self.cBox_plot.toggled.disconnect()
        self.gBox_text.clicked.disconnect()
        self.gBox_dB.clicked.disconnect()

        self.gBox_dB.setChecked(False)
        self.cBox_plot.setChecked(False)
        self.gBox_text.setChecked(True)
        if (self.postgresql_handle is not None):
            self.postgresql_handle.disconnect()
            self.postgresql_handle = None
        print('gBox_text')
  
        self.gBox_text.clicked.connect(self.gBox_text_toggled)
        self.gBox_dB.clicked.connect(self.gBox_dB_toggled)
        self.cBox_plot.toggled.connect(self.cBox_plot_toggled)
    
    def gBox_dB_toggled(self, status):
        self.cBox_plot.toggled.disconnect()
        self.gBox_text.clicked.disconnect()
        self.gBox_dB.clicked.disconnect()

        self.gBox_text.setChecked(False)
        self.cBox_plot.setChecked(False)
        self.gBox_dB.setChecked(True)
        if (self.postgresql_handle is None):
            self.postgresql_handle = postgresql(host=self.lEdit_host.text(), db=self.lEdit_db.text(), user=self.lEdit_user.text(), 
                                                passw=self.lEdit_pass.text(), port = int(self.lEdit_port.text()))
        print('gBox_dB')  

        self.gBox_text.clicked.connect(self.gBox_text_toggled)
        self.gBox_dB.clicked.connect(self.gBox_dB_toggled)
        self.cBox_plot.toggled.connect(self.cBox_plot_toggled)
        
    def cBox_plot_toggled(self, status):
        self.cBox_plot.toggled.disconnect()
        self.gBox_text.clicked.disconnect()
        self.gBox_dB.clicked.disconnect()
        
        self.cBox_plot.setChecked(True)
        self.gBox_text.setChecked(False)
        self.gBox_dB.setChecked(False)
        print('cBox_plot')
            
        self.gBox_text.clicked.connect(self.gBox_text_toggled)
        self.gBox_dB.clicked.connect(self.gBox_dB_toggled)
        self.cBox_plot.toggled.connect(self.cBox_plot_toggled)
        
    def send_config(self):
        output = BitArray(bytes = struct.pack('>f', self.dSpinBox_cal.value()))
        output.append(BitArray(bool = self.rButton_gps.isChecked()))
        output.append(BitArray(bool =self.rButton_lpwa.isChecked()))
        output.append(BitArray(bool =self.rButton_tob.isChecked()))
        output.append(BitArray(int =self.sBox_rect.value(), length = 13))
        output.append(BitArray(int =self.sBox_leqt.value(), length = 12))
        output.append(BitArray(int = 0, length = 4))  #Padding
        #print("BitArray: ", output.tobytes().hex())
        output.byteswap()
        output.prepend(BitArray(hex = '05'))
        
        self.COM_handle.flushOutput()
        self.COM_handle.flushInput()
        output = output.tobytes()
        print("send_config: ", output.hex())
        self.COM_handle.write(output)  
        time.sleep(0.1) #To let the uC perform changes
        self.get_config_pars()
        
    def pB_send_cmd_clicked(self):
        self.tEdit_response.setText(' ')
        tout = int(self.dSpinBox_tout.value()*1000)
        text = self.lEdit_cmd.text().split(';') 
        if len(text)>1:
            cmds = ''
            for ii in range(len(text)):
                print('Send[%02d]'%ii)
                cmds +=  text[ii] + '\r'                
            self.tEdit_response.setText(self.LPWA_send_cmd(cmds , tout))
        else:
            self.tEdit_response.setText(self.LPWA_send_cmd(text[0] + '\r', tout))
        
    def tW_config_changed(self,tab):
        #TODO: Command to load APN and network configuration
        if(tab == 1):
            CMD = "AT+CGDCONT?"
            res = self.LPWA_send_cmd(CMD, 200)
            APN = res.split('\n')
            APN = [x for x in APN if x]
            print("\r\ntW_config_changed(CGDCONT):", res, ' - ', APN)
            if(len(APN)>3):
                self.lEdit_apn.setText(APN[1].split(',')[2][1:-1])
            else:
                self.lEdit_apn.setText('Not available')
                
            time.sleep(0.1)
            
            CMD = "AT+COPS?"
            res = self.LPWA_send_cmd(CMD, 200)
            APN = res.split('\n')
            print("tW_config_changed(COPS):", res, ' - ', APN)
            if(len(APN)>1):
                self.lEdit_network.setText(APN[1][1:])
            else:
                self.lEdit_network.setText('Not available')
            
    def LPWA_send_cmd(self, cmd, timeout):
        
        try: 
            self.COM_handle.flushOutput()
            self.COM_handle.flushInput()

            tout_b = struct.pack('<H', timeout)
            output = bytes.fromhex('03') + tout_b + cmd.encode() + '\r'.encode()
            print("LPWA_Tx:", output.hex())
            self.COM_handle.write(output)
            # time.sleep(0.15)     
            raw_bytes = self.COM_handle.read_until(expected=serial.to_bytes([0]))
            print("LPWA_Rx:", raw_bytes)
            return raw_bytes[:-1].decode("utf-8")
        except TypeError as e:
            print("Failed to receive data! ", e)

        return -1
    
    def gBox_text_checked(self, status):
        if(status):
            self.gBox_dB.setChecked(False)
        
    def gBox_dB_checked(self, status):
        if(status):
            self.gBox_text.setChecked(False)
            
    def gBox_dB_checked(self, status):
        if(status):
            self.gBox_text.setChecked(False)
        
    def pB_path_clicked(self, status):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        
        fileName, _= QtWidgets.QFileDialog.getSaveFileName(
            None,
            "Select output file",
            ".csv",
            "Comma sepparated values (*.csv)",
            options=options,
        )
        print(fileName)
        self.tEdit_file.setText(fileName)
        
    def pB_read_data_clicked(self, status):
        if(self.active_COM):    #TODO: Add this to the rest of the functions
            self.COM_handle.flushOutput()
            self.COM_handle.flushInput()
            output = bytes.fromhex('010000')
            self.COM_handle.write(output)
            try:           
                #Receive data len
                raw_bytes = self.COM_handle.read_until(size = 2)
                print(raw_bytes.hex())
                data_len = struct.unpack('<H', raw_bytes.ljust(2, b'\0'))[0]
                self.pBar_read.setMaximum(data_len - 1)
                print("data_len:", data_len)
                
                if(self.gBox_text.isChecked()):
                    with open(self.tEdit_file.toPlainText(), 'w', encoding='UTF8') as f:
                        writer = csv.writer(f, delimiter='\t')
                        writer.writerow(HEADER)
                        for curr_add in list(range(data_len-1, -1, -1)):#range(data_len):
                            #print(curr_add)
                            output = bytes.fromhex('02') + struct.pack('<H', curr_add)
                            self.COM_handle.write(output)
                            raw_bytes = bytearray(EEPROM_READ_BLOCK_LEN)
                            self.COM_handle.readinto(raw_bytes)
                            #print(raw_bytes.hex(),'\r\n')
             
                            for ii in range(2):
                                # DATA = decode_data(raw_bytes.hex()[EEPROM_READ_BLOCK_LEN*ii:EEPROM_READ_BLOCK_LEN*(ii+1)])
                                DATA = decode_data_new(raw_bytes.hex()[EEPROM_READ_BLOCK_LEN*ii:EEPROM_READ_BLOCK_LEN*(ii+1)])
                                #print(DATA)
                                
                                for sample_time in DATA['samples'].keys():
                                    row = []
                                    row.append(DATA['samples'][sample_time]['Leq'])
                                    row.append('\"' + str(sample_time) + '\"')
                                    row.append(DATA['samples'][sample_time]['lat'])
                                    row.append(DATA['samples'][sample_time]['lon'])
                                    row.append(DATA['samples'][sample_time]['q'])
                                    row.append(DATA['samples'][sample_time]['sat_n'])
                                    
                                    for tob in DATA['samples'][sample_time]['TOB'].keys():
                                        row.append(DATA['samples'][sample_time]['TOB'][tob])    
                                    
                                    writer.writerow(row)   
                                    
                            self.pBar_read.setValue(curr_add)
                            QApplication.processEvents()
                
                #### BINARY FILE OUTPUT
                # row = bytearray([])
                # if(self.gBox_text.isChecked()):
                #     with open(self.tEdit_file.toPlainText(), 'wb') as f:
                #         for curr_add in range(data_len):
                #             #print(curr_add)
                #             output = bytes.fromhex('02') + struct.pack('<H', curr_add)
                #             self.COM_handle.write(output)
                #             raw_bytes = bytearray(EEPROM_READ_BLOCK_LEN)
                #             self.COM_handle.readinto(raw_bytes)
                #             #print(raw_bytes.hex(),'\r\n')
                #             #time.sleep(0.1) 
                #             row.extend(raw_bytes)
                                    
                #             self.pBar_read.setValue(curr_add)
                #             QApplication.processEvents()
                    
                #         f.write(row)
                
                elif(self.gBox_dB.isChecked()):
                    for curr_add in range(data_len):
                        output = bytes.fromhex('02') + struct.pack('<H', curr_add)
                        self.COM_handle.write(output)
                        raw_bytes = bytearray(EEPROM_READ_BLOCK_LEN)
                        self.COM_handle.readinto(raw_bytes)
                        for ii in range(2):
                            decoded = decode_data(raw_bytes.hex()[EEPROM_READ_BLOCK_LEN*ii:EEPROM_READ_BLOCK_LEN*(ii+1)])
                            self.postgresql_handle.insert_data(decoded)
                                
                        self.pBar_read.setValue(curr_add)
                        QApplication.processEvents()
                
                elif(self.cBox_plot.isChecked()):
                    df = pd.DataFrame(columns=HEADER_PLOT)

                    for curr_add in range(data_len):
                        print('Curr_add:', curr_add)
                        output = bytes.fromhex('02') + struct.pack('<H', curr_add)
                        self.COM_handle.write(output)
                        # time.sleep(0.05) 
                        raw_bytes = bytearray(EEPROM_READ_BLOCK_LEN)
                        self.COM_handle.readinto(raw_bytes)
                        #print(raw_bytes.hex(),'\r\n')
                        
                        for ii in range(2):
                            DATA = decode_data_new(raw_bytes.hex()[EEPROM_READ_BLOCK_LEN*ii:EEPROM_READ_BLOCK_LEN*(ii+1)])
                            #DATA = decode_data(raw_bytes.hex()[EEPROM_READ_BLOCK_LEN*ii:EEPROM_READ_BLOCK_LEN*(ii+1)])
                            # print(raw_bytes.hex()[EEPROM_READ_BLOCK_LEN*ii:EEPROM_READ_BLOCK_LEN*(ii+1)])
                            print(DATA)
                            for sample_time in DATA['samples'].keys():
                                row = []
                                row.append(DATA['samples'][sample_time]['Leq'])                                
                                for tob in DATA['samples'][sample_time]['TOB'].keys():
                                    row.append(DATA['samples'][sample_time]['TOB'][tob])    

                                df.loc[sample_time] = row
                                
                        self.pBar_read.setValue(curr_add)
                        QApplication.processEvents()    
                    
                    df = df.iloc[1:]
                    Leqs = df.apply(self.Leq)
                    self.fig, self.ax1 = plt.subplots(figsize=(10, 10))
                    self.ax1.grid()
                    #self.ax1.step(Leqs, 'r')
                    self.ax1.bar(HEADER_PLOT,np.asarray(Leqs), color ='maroon',
                                     width = 0.4)
                    plt.xticks(rotation = 45)
                    print(np.asarray(Leqs))
                
            except TypeError as e:
                print("Failed to send data! ", e)
    
    def Leq(self, data):
        data_np = np.asarray(data)
        return 10*np.log10((1/len(data_np)) * sum(10**(data_np/10)))
        
    def get_config_pars(self):
        print("Get config pars")
        self.COM_handle.flushOutput()
        self.COM_handle.flushInput()
        output = bytes.fromhex('040000')
        self.COM_handle.write(output)
        try:           
            #Receive bytes
            raw_bytes = self.COM_handle.read_until(size = 8)
            tmp = struct.unpack('I', raw_bytes[:4])[0]
            data_bin = BitArray(hex=hex(tmp))
            
            self.dSpinBox_cal.setValue(struct.unpack('f', raw_bytes[4:])[0])
            self.rButton_gps.setChecked(data_bin[0])
            self.rButton_lpwa.setChecked(data_bin[1])
            self.rButton_tob.setChecked(data_bin[2])
            self.sBox_rect.setValue(int(data_bin[3:16].bin, 2))
            self.sBox_leqt.setValue(int(data_bin[17:28].bin, 2))

            #print('cal:', cal, ' - gps:', gps, ' - lpwa:', lpwa, ' - octave:', octave, ' - RecTime:', RecTime, ' - LeqTime:', LeqTime)
            print("get_config_pars:", raw_bytes.hex())
        except TypeError as e:
            print("Failed to get_config_pars! ", e)
        
    def pB_connect_clicked(self, checked):
        self.pB_connect.clicked.disconnect()
        if(checked):
            self.pB_connect.setChecked(False)
            self.active_COM = False
            if self.COM_conn()  :
                if self.send_conn_string():
                   self.pB_connect.setStyleSheet('QPushButton {background-color: #43A047; color: white;}')
                   self.pB_connect.setText('Disconnect')
                   self.active_COM = True
                   # time.sleep(0.1)
                   self.get_config_pars()
        else:
            self.COM_close()
            self.pB_connect.setText('Connect')
            self.pB_connect.setStyleSheet('QPushButton {background-color: rgb(237, 51, 59); color: red;}')
            self.active_COM = False    
        self.pB_connect.clicked.connect(self.pB_connect_clicked)
    
    def send_conn_string(self):
        #Send random bytes
        self.COM_handle.flushOutput()
        self.COM_handle.flushInput()
        rand_bytes = 'ff%08x' % random.getrandbits(32)
        print('Conn string: ', rand_bytes)
        output = bytes.fromhex(rand_bytes)
        self.COM_handle.write(output)
        try:           
            #Receive random bytes
            raw_bytes = self.COM_handle.read_until(size = 5)
            print("received:", raw_bytes.hex(), '-', rand_bytes)
            if(raw_bytes.hex() == rand_bytes):
                return True
        except TypeError as e:
            print("Failed to send data! ", e)
            
        return False
    
    def COM_conn(self):
        self.port = None
        self.baud = 115200
        self.COM_handle = None
        #self.timeout = 1
        for port in comports():
            print('PID:', port.pid)
            if port.pid == PID:
                self.port = port.device
            print(port)  
        try:
            self.COM_handle = serial.Serial(self.port, self.baud, timeout=10)
            print('Connected to ' + str(self.port) + ' at ' + str(self.baud) + ' BAUD.')
            return True
        except:
            print("Failed to connect!")
            return False
            
    def COM_close(self):
        if (self.COM_handle is not None):
            try:
                self.COM_handle.close()
                print("Port " + self.port + " disconnected!")
            except:
                print("Failed to disconnect!")
        
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = Window()
    win.show()
    sys.exit(app.exec())