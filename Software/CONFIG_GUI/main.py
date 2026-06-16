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
from scipy.interpolate import UnivariateSpline

sys.path.insert(1, '../IoT_CoAP')
from Decode_data import decode_delta_data, postgresql, decode_data

PID = 22336

EEPROM_PAGE_SIZE = 512
EEPROM_N_PAGES = 8191
#EEPROM_READ_BLOCK_LEN = 496

n_bytes_header = 15
n_bytes_payload = 33
n_bytes_payload_delta = 27
block_payloads = 8

# 2 blocks of 237 bytes each (15 + 33 + 7*27 = 237) = 474 bytes total
EEPROM_READ_BLOCK_LEN = 2 * (n_bytes_header + n_bytes_payload + ((block_payloads - 1) * n_bytes_payload_delta))


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
        if(self.active_COM):    
            self.COM_handle.reset_input_buffer()
            self.COM_handle.flushOutput()
            output = bytes.fromhex('010000')
            self.COM_handle.write(output)
            
            try:           
                # Receive data len
                raw_bytes = self.COM_handle.read_until(size = 2)
                data_len = struct.unpack('<H', raw_bytes.ljust(2, b'\0'))[0]
                self.pBar_read.setMaximum(data_len - 1)
                print("data_len:", data_len)
                
                # Check if the EEPROM contains no data to prevent processing crashes across all modes
                if data_len == 0:
                    QtWidgets.QMessageBox.warning(self, "Empty Memory", "The EEPROM is completely empty. No data available to retrieve.")
                    return
                
                # (15 + 33 + 7*27 = 237)
                block_len = 237 
                leq_time = int(self.sBox_leqt.value())
                
                if(self.gBox_text.isChecked()):
                    all_rows = []
                    
                    # 1. READ AND ACCUMULATE
                    for curr_add in list(range(data_len-1, -1, -1)):
                        self.COM_handle.reset_input_buffer()
                        
                        output = bytes.fromhex('02') + struct.pack('<H', curr_add)
                        self.COM_handle.write(output)
                        
                        raw_bytes = bytearray(EEPROM_READ_BLOCK_LEN)
                        self.COM_handle.readinto(raw_bytes)
         
                        for ii in range(2):
                            block_bytes = raw_bytes[block_len * ii : block_len * (ii + 1)]
                            DATA = decode_delta_data(block_bytes.hex(), leq_time)
                            
                            sample_keys = list(DATA['samples'].keys())
                            num_samples = len(sample_keys)
                            
                            for idx, sample_time in enumerate(sample_keys):
                                # GNSS position is taken at the last sample of the block
                                is_fix = (idx == num_samples - 1)
                                
                                row_data = {
                                    'Leq': DATA['samples'][sample_time]['Leq'],
                                    'Date/Time': sample_time,
                                    'LAT': DATA['samples'][sample_time]['lat'],
                                    'LON': DATA['samples'][sample_time]['lon'],
                                    'Q': DATA['samples'][sample_time]['q'],
                                    'SAT #': DATA['samples'][sample_time]['sat_n'],
                                    'is_fix': is_fix
                                }
                                
                                for tob in DATA['samples'][sample_time]['TOB'].keys():
                                    row_data[str(tob)] = DATA['samples'][sample_time]['TOB'][tob]
                                
                                all_rows.append(row_data)
                                
                        self.pBar_read.setValue(data_len - curr_add)
                        QApplication.processEvents()
                    
                    # 2. PROCESSING AND SPLINE INTERPOLATION
                    df = pd.DataFrame(all_rows)
                    
                    # Parse strings using 'mixed' format to handle exact seconds and fractional seconds
                    df['Date/Time'] = pd.to_datetime(df['Date/Time'], format='mixed')
                    df = df.drop_duplicates(subset=['Date/Time']).sort_values('Date/Time').reset_index(drop=True)
                    
                    df_interp = df.copy()
                    df_interp.loc[~df_interp['is_fix'], ['LAT', 'LON']] = np.nan
                    
                    valid_mask = df_interp['is_fix'] & df_interp['LAT'].notna() & df_interp['LON'].notna()
                    
                    if valid_mask.sum() >= 4:
                        x_all = df_interp['Date/Time'].astype(np.int64) // 10**9
                        x_valid = x_all[valid_mask]
                        
                        _, unique_indices = np.unique(x_valid, return_index=True)
                        x_valid = x_valid.iloc[unique_indices]
                        y_lat_valid = df_interp.loc[valid_mask, 'LAT'].iloc[unique_indices]
                        y_lon_valid = df_interp.loc[valid_mask, 'LON'].iloc[unique_indices]
                        
                        # Define the maximum number of GNSS valid points per spline segment
                        # 20 points * 8 seconds = 160 seconds window
                        max_spline_points = 6
                        
                        if len(x_valid) >= 4:
                            # # spline_lat = UnivariateSpline(x_valid, y_lat_valid)
                            # # spline_lon = UnivariateSpline(x_valid, y_lon_valid)
                            
                            # # df_interp['LAT'] = spline_lat(x_all)
                            # # df_interp['LON'] = spline_lon(x_all)
                            
                            # LINEAR INTERPOLATION (DEBUG)
                            df_interp.set_index('Date/Time', inplace=True)                        
                            df_interp['LAT'] = df_interp['LAT'].interpolate(method='time')
                            df_interp['LON'] = df_interp['LON'].interpolate(method='time')
                            df_interp.reset_index(inplace=True)
                            
                            time_prev_fix = df_interp['Date/Time'].where(valid_mask).ffill()
                            time_next_fix = df_interp['Date/Time'].where(valid_mask).bfill()
                            
                            gap_seconds = (time_next_fix - time_prev_fix).dt.total_seconds()
                            
                            # Remove interpolations trapped in gaps larger than 60s
                            df_interp.loc[gap_seconds > 60, ['LAT', 'LON']] = np.nan
                            df_interp.loc[time_prev_fix.isna() | time_next_fix.isna(), ['LAT', 'LON']] = np.nan
                            
                            # Fill the initial orphaned samples by copying the first valid position
                            first_valid_idx = df_interp['LAT'].first_valid_index()
                            if first_valid_idx is not None and first_valid_idx <= 8:
                                df_interp.loc[:first_valid_idx, 'LAT'] = df_interp.loc[first_valid_idx, 'LAT']
                                df_interp.loc[:first_valid_idx, 'LON'] = df_interp.loc[first_valid_idx, 'LON']
                            
                            #### Chunk processing
                            # df_interp['LAT'] = np.nan
                            # df_interp['LON'] = np.nan
                            
                            # # Process the route in smaller chunks to prevent static data from flattening the curves
                            # for i in range(0, len(x_valid), max_spline_points):
                                
                            #     end_idx = min(i + max_spline_points, len(x_valid))
                                
                            #     # Add a 2-point overlap with the next chunk to ensure curve continuity at boundaries
                            #     if end_idx < len(x_valid):
                            #         end_idx += 2 
                                    
                            #     x_chunk = x_valid.iloc[i:end_idx]
                            #     lat_chunk = y_lat_valid.iloc[i:end_idx]
                            #     lon_chunk = y_lon_valid.iloc[i:end_idx]
                                
                            #     # Minimum 4 points required for a cubic spline (degree 3)
                            #     if len(x_chunk) >= 4:
                            #         spline_lat = UnivariateSpline(x_chunk, lat_chunk)
                            #         spline_lon = UnivariateSpline(x_chunk, lon_chunk)
                                    
                            #         # Identify all interpolated timestamps belonging to this specific time window
                            #         start_time = x_chunk.iloc[0]
                            #         end_time = x_chunk.iloc[-1]
                                    
                            #         chunk_mask = (x_all >= start_time) & (x_all <= end_time)
                                    
                            #         # Apply the calculated spline only to this specific segment
                            #         df_interp.loc[chunk_mask, 'LAT'] = spline_lat(x_all[chunk_mask])
                            #         df_interp.loc[chunk_mask, 'LON'] = spline_lon(x_all[chunk_mask])
                    
                    # 3. FINAL CSV GENERATION
                    # Include microseconds (%f) to preserve 125ms resolution in the text file
                    df_interp['Date/Time'] = '"' + df_interp['Date/Time'].dt.strftime('%Y-%m-%d %H:%M:%S.%f') + '"'
                    df_interp.to_csv(self.tEdit_file.toPlainText(), sep='\t', index=False, columns=HEADER)
                    print("Data extraction, UnivariateSpline interpolation, and export completed.")
                    
                    # Notify the user about successful file creation
                    QtWidgets.QMessageBox.information(self, "Export Success", "Data extraction and CSV generation completed successfully.")
                
                elif(self.gBox_dB.isChecked()):
                    for curr_add in range(data_len):
                        # LIMPIEZA CRÍTICA
                        self.COM_handle.reset_input_buffer()
                        output = bytes.fromhex('02') + struct.pack('<H', curr_add)
                        self.COM_handle.write(output)
                        
                        raw_bytes = bytearray(EEPROM_READ_BLOCK_LEN)
                        self.COM_handle.readinto(raw_bytes)
                        
                        for ii in range(2):
                            block_bytes = raw_bytes[block_len * ii : block_len * (ii + 1)]
                            DATA = decode_delta_data(block_bytes.hex(), leq_time)
                            self.postgresql_handle.insert_data(DATA)
                                
                        self.pBar_read.setValue(curr_add)
                        QApplication.processEvents()
                
                elif(self.cBox_plot.isChecked()):
                    df = pd.DataFrame(columns=HEADER_PLOT)

                    for curr_add in range(data_len):
                        print('Curr_add:', curr_add)
                        # LIMPIEZA CRÍTICA
                        self.COM_handle.reset_input_buffer()
                        output = bytes.fromhex('02') + struct.pack('<H', curr_add)
                        self.COM_handle.write(output)
                        
                        raw_bytes = bytearray(EEPROM_READ_BLOCK_LEN)
                        self.COM_handle.readinto(raw_bytes)
                        
                        for ii in range(2):
                            block_bytes = raw_bytes[block_len * ii : block_len * (ii + 1)]
                            DATA = decode_delta_data(block_bytes.hex(), leq_time)
                            
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
                    self.ax1.bar(HEADER_PLOT,np.asarray(Leqs), color ='maroon', width = 0.4)
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
            self.active_COM = False
            
            # Attempt to open the port
            if self.COM_conn():
                # Attempt to verify the synchronization string
                if self.send_conn_string():
                    self.pB_connect.setStyleSheet('QPushButton {background-color: #43A047; color: white;}')
                    self.pB_connect.setText('Disconnect')
                    self.active_COM = True
                    self.get_config_pars()
                else:
                    # Synchronization failed: close port, restore button state, and show alert
                    print("Error: Device synchronization failed.")
                    self.COM_close()
                    self.pB_connect.setChecked(False)
                    QtWidgets.QMessageBox.critical(self, "Connection Error", "Device synchronization failed. Please verify the firmware status.")
            else:
                # Failed to open port: restore button state and show alert
                self.pB_connect.setChecked(False)
                QtWidgets.QMessageBox.critical(self, "Connection Error", "Could not open the serial port. Verify that the device is properly connected.")
        else:
            # Manual disconnection triggered by the user
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
        
        # Search for the specific port using PID
        for port in comports():
            print('PID:', port.pid)
            if port.pid == PID:
                self.port = port.device
                break # Exit loop if device is found
            print(port)
            
        # Validate port existence before attempting to open
        if self.port is None:
            print("Error: Device not found (PID mismatch).")
            return False
            
        try:
            self.COM_handle = serial.Serial(self.port, self.baud, timeout=10)
            print('Connected to ' + str(self.port) + ' at ' + str(self.baud) + ' BAUD.')
            return True
        except Exception as e:
            print(f"Error connecting to port {self.port}: {e}")
            self.COM_handle = None
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