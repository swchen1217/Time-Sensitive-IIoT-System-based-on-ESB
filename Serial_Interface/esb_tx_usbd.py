import threading
import time
from time import sleep
import serial
import sys
import queue
import pandas as pd
import csv
import sys
import struct
import chn_map_process as cmp

CDC_ACM_DATA = 0
CDC_ACM_CHN_UPDATE = 1
URLLC_DATA_INTERVAL = 0.1                   #sleep URLLC_DATA_INTERVAL(seconds) between sending data(sync or urllc) to comport
CHA_MAP_UPDATE_INTERVAL = 2
CHN_MAP_UPDATE_OFFSET   = 0.2
CDC_ACM_DATA_MAX_SIZE = 256                 #maximum data bytes that can tranfer each time 



com_list = ['com12']
#com_list = ['com10','com16']
#com_list = ['com10','com9']
#com_list = ['com10','com9','com12']
#com_list = ['com10','com9','com12','com21','com14']
#chn_map = [20, 34, 13, 24, 8, 17, 11, 2, 27, 25, 35, 29, 5, 31, 3, 10, 14, 23, 1, 28, 9, 18, 0, 6, 7, 15, 16, 21, 19, 33, 12, 4, 32, 26, 22, 30]
com_threads = {}
com_lock = threading.Lock()  # Lock for synchronizing access to com_queue
 
"""write cdc_acm_data or chn_map to related serial port
   Args:
        com:assign comport number to write data to
"""
def write_data(com,com_queue):
    while(True):
        if not com_queue.empty():
            q_data = com_queue.get()
            data_type = q_data['type']
            if data_type == CDC_ACM_DATA:
                val = q_data['data']
                valBytes = str(val).encode() 
                length = len(valBytes)
                seq_number = q_data['seq_num']
                tlv_data_header = struct.pack('BB',data_type,length+4)
                tlv_data = struct.pack("I",seq_number)+valBytes
                com.write(tlv_data_header)
                com.write(tlv_data)

            elif data_type == CDC_ACM_CHN_UPDATE:
                chn_map = q_data['chn_map']
                print(chn_map)
                chn_map_bytes = struct.pack('B' * len(chn_map), *chn_map)
                length = len(chn_map_bytes)
                tlv_data_header = struct.pack('BB',data_type,length)
                com.write(tlv_data_header)
                com.write(chn_map_bytes)

"""read data from related serial port
   Args:
        com:assign comport number to write data to
        q:put data into this queue
"""
def read_data(ser,q):
    while(True):
        queue = ser.inWaiting()
        if queue > 0 :
            data = ser.readline()
            print(data)

"""open comports and assigned thread task to them  
"""           
def com_port_init():
    com_queue = queue.Queue(1000)
    write_thread_assigned_list = []
    #open com and assign thread
    for com_name in com_list:
        try:
            opened_com = serial.Serial(com_name, 115200, timeout=0.5)
            write_thread = threading.Thread(target=write_data, args=(opened_com, com_queue))
            write_thread_assigned_list.append(write_thread)
            com_threads[com_name] = {'thread': write_thread, 'queue': com_queue}
            #assigned read_data task to thread for relative comport and start it immediately 
            read_thread = threading.Thread(target=read_data, args=(opened_com, com_queue))
            read_thread.start()
        except serial.SerialException as e:
            print(f"Failed to open COM port {com_name}. Error: {str(e)}")
    #start assigned thread
    for assigned_thread in write_thread_assigned_list:
        assigned_thread.start()

def update_chn_map():
    while True:
        current_chn_map = cmp.get_current_chn_map()
        with com_lock:
            for com_info in com_threads.values():
                com_queue = com_info['queue']
                com_queue.put({'type': CDC_ACM_CHN_UPDATE, 'chn_map': current_chn_map})
                sleep(CHN_MAP_UPDATE_OFFSET)
        sleep(CHA_MAP_UPDATE_INTERVAL)
        
'''generate CDC_ACM data packets with a sequence number, sends them to the queue of each COM port
'''
def generate_cdc_acm_data():
    seq_number = 0
    while True:
        with com_lock:
            for com_info in com_threads.values():
                com_queue = com_info['queue']
                com_queue.put({'type': CDC_ACM_DATA, 'seq_num': seq_number, 'data': str(seq_number)})
        seq_number += 1
        sleep(URLLC_DATA_INTERVAL)

def main():
    
    # chn_update_thread = threading.Thread(target=update_chn_map)
    # chn_update_thread.start()
    com_port_init()
    # cmp.open_qos_device()
    generate_cdc_acm_data()
    
if __name__ =='__main__':
    main() 
