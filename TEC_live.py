"""
Use the pyubx2 library to compute the live value of vertical TEC
Make the TEC value computed availiable via websocket
"""

import asyncio
import math
import datetime as dt
from threading import Event, Thread
from queue import Queue
from serial import Serial
from pyubx2 import UBXReader, UBXMessage, NMEA_PROTOCOL, UBX_PROTOCOL, RTCM3_PROTOCOL
import websockets
import os
import time


def determineFrequency(gnssId, sigId):
    # ...copy from ubxreader.py...
    if gnssId == 0 and sigId == 0:
        return 1575.42e6 # L1C/A
    elif gnssId == 0 and sigId == 3:
        return 1227.6e6 # L2CL
    elif gnssId == 0 and sigId == 4:
        return 1227.6e6 # L2CM
    elif gnssId == 0 and sigId == 6:
        return 1176.45e6 # L5 I
    elif gnssId == 0 and sigId == 7:
        return 1176.45e6 #L5 Q
    elif gnssId == 1 and sigId == 0:
        return 1575.42e6 # SBAS L1C/A
    elif gnssId == 2 and sigId == 0:
        return 1575.42e6 # GALI E1 C
    elif gnssId == 2 and sigId == 1:
        return 1207.14e6 # E1 B
    elif gnssId == 2 and sigId == 3:
        return 1176.45e6 # E5a 
    elif gnssId == 2 and sigId == 4:
        return 1176.45e6 #E5a
    elif gnssId == 2 and sigId == 5:
        return 1207.14e6 # E5b
    elif gnssId == 2 and sigId == 6:
        return 1207.14e6 # E5b
    elif gnssId == 3 and sigId == 0:
        return 1561.091e6 # B1I D1
    elif gnssId == 3 and sigId == 1:
        return 1561.091e6 # B1I D2
    elif gnssId == 3 and sigId == 2:
        return 1207.14e6 # B2I D1
    elif gnssId == 3 and sigId == 3:
        return 1207.14e6 # B2I D2
    elif gnssId == 3 and sigId == 5:
        return 1575.42e6 # B1C
    elif gnssId == 3 and sigId == 7:
        return 1176.45e6 #B2a
    elif gnssId == 5 and sigId == 0:
        return 1575.42e6 # QZSS L1C/A 
    elif gnssId == 5 and sigId == 1:
        return 1575.42e6 # QZSS L1S
    elif gnssId == 5 and sigId == 4:
        return 1227.6e6 # QZSS L2 CM
    elif gnssId == 6 and sigId == 0:
        return 1598.0625e6 #GLONASS L1
    elif gnssId == 6 and sigId == 2:
        return 1242.9375e6 #GLONASS L2
    elif gnssId == 7 and sigId == 0:
        return 1176.45e6 #NAVIC L5

def calc_tec(f1:float, f2:float, pseudo1:float, pseudo2:float) -> float:
    term1 = (1/(40.3))
    term2 = (f1*f2)/(f1-f2)
    term3 = (pseudo2 - pseudo1)
    return abs(term1*term2*term3)

def verticalIntegration(tec, angle):
    height = 350
    Re = 6371e3
    cos_inner = math.asin((Re*math.cos(angle))/(Re+height))
    return tec*math.cos(cos_inner)

def findMatchers(gnssIdBlock, svIdBlock):
    for i in range(len(gnssIdBlock)):
        for j in range(len(gnssIdBlock)):
            if (gnssIdBlock[i] != gnssIdBlock[j]) and (svIdBlock[i] == svIdBlock[j]):   
                return i,j
    return None, None 

class TECReader:
    def __init__(self, port="/dev/ttyACM0", baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.latest_vtec = None
        self._stop_event = Event()
        self._thread = Thread(target=self._read_loop, daemon=True)
        self.ubx_buffer = []
        self.last_flush_time = time.time()
        self.flush_interval = 300  # seconds (5 minutes)
        self._file_thread = Thread(target=self._file_writer_loop, daemon=True)
        self._file_stop_event = Event()
        
        # In the init function, we can send messages to control the device (i.e. make it produce UBX messages!)
        stream = Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
        cfg_data = []
        enable = True  # Set to False to disable NMEA and UBX messages
        for port_type in ("USB", "UART1"):
            #To be able to generate RINEX with convbin we need RXM-RAWX, NAV-SAT, NAV-TIMEGPS, and NAV-PVT
            #the others are there in case they might be useful but I don't think they are necessary
            cfg_data.append((f"CFG_{port_type}OUTPROT_NMEA", not enable))
            cfg_data.append((f"CFG_{port_type}OUTPROT_UBX", enable))
            cfg_data.append((f"CFG_MSGOUT_UBX_RXM_RAWX_{port_type}", enable * 4))
            cfg_data.append((f"CFG_MSGOUT_UBX_NAV_PVT_{port_type}", enable))
            cfg_data.append((f"CFG_MSGOUT_UBX_NAV_SAT_{port_type}", enable * 4))
            cfg_data.append((f"CFG_MSGOUT_UBX_NAV_DOP_{port_type}", enable * 4))
            cfg_data.append((f"CFG_MSGOUT_UBX_RXM_RTCM_{port_type}", enable))
            cfg_data.append((f"CFG_MSGOUT_UBX_NAV_TIMEGPS_{port_type}", enable))  # <--- Add this line
        msg = UBXMessage.config_set(1,0,cfg_data)
        stream.write(msg.serialize())
        
    
    def start(self):
        self._thread.start()
        self._file_thread.start()

    def stop(self):
        self._stop_event.set()
        self._file_stop_event.set()
        self._thread.join()
        self._file_thread.join()

    def _file_writer_loop(self):
        while not self._file_stop_event.is_set():
            now = time.time()
            if self.ubx_buffer and (now - self.last_flush_time >= self.flush_interval):
                # Write buffer to file
                ts = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"observation_{ts}.ubx"
                filepath = os.path.join(os.path.dirname(__file__), filename)
                try:
                    with open(filepath, "wb") as f:
                        for raw_bytes in self.ubx_buffer:
                            f.write(raw_bytes)
                    print(f"Wrote {len(self.ubx_buffer)} UBX messages to {filename}")
                except Exception as e:
                    print(f"Error writing UBX file: {e}")
                self.ubx_buffer.clear()
                self.last_flush_time = now
            time.sleep(1)

    def _read_loop(self):
        stream = Serial(self.port, self.baudrate, timeout=self.timeout)
        ubr = UBXReader(stream, protfilter=(NMEA_PROTOCOL | UBX_PROTOCOL | RTCM3_PROTOCOL))
        gnssIdBlock = svIdBlock = sigIdBlock = doMesBlock = psuedorangeBlock = elvIdBlock = None
        while not self._stop_event.is_set():
            try:
                if stream.in_waiting:
                    raw_data, parsed_data = ubr.read()
                    # --- Buffer raw UBX messages for file output ---
                    if raw_data and isinstance(raw_data, (bytes, bytearray)):
                        self.ubx_buffer.append(raw_data)
                    print(f"Received data at {dt.datetime.now()}")  # Debug print statement
                    print(f"Raw data identity: {parsed_data.identity if parsed_data else 'None'}")  # Debug print statement
                    if parsed_data and hasattr(parsed_data, "identity"):
                        if parsed_data.identity == 'RXM-RAWX':
                            #print(f"Received RXM-RAWX\n{parsed_data}")  # Debug print statement
                            try:
                                psuedorangeBlock = [getattr(parsed_data, f"prMes_{i:02d}") for i in range(1,33)]
                                gnssIdBlock = [getattr(parsed_data, f"gnssId_{i:02d}") for i in range(1,33)]
                                svIdBlock = [getattr(parsed_data, f"svId_{i:02d}") for i in range(1,33)]
                                doMesBlock = [getattr(parsed_data, f"doMes_{i:02d}") for i in range(1,33)]
                                sigIdBlock = [getattr(parsed_data, f"sigId_{i:02d}") for i in range(1,33)]
                            except AttributeError:
                                print("AttributeError: One or more blocks not found in parsed_data")
                                continue
                        if parsed_data.identity == 'NAV-SAT':
                            #print(f"Received NAV-SAT\n{parsed_data}")  # Debug print statement
                            try:
                                elvIdBlock = [getattr(parsed_data, f"elev_{i:02d}") for i in range(1,33)]
                            except AttributeError:
                                print("AttributeError: elvIdBlock not found in parsed_data")
                                continue
                            if gnssIdBlock and svIdBlock and sigIdBlock and doMesBlock and psuedorangeBlock and elvIdBlock:
                                #print("All blocks are populated, proceeding with TEC calculation")
                                i, j = findMatchers(gnssIdBlock, svIdBlock)
                                if i is not None and j is not None:
                                    f1 = determineFrequency(gnssIdBlock[i], sigIdBlock[i]) + doMesBlock[i]
                                    f2 = determineFrequency(gnssIdBlock[j], sigIdBlock[j]) + doMesBlock[j]
                                    tec = calc_tec(f1, f2, psuedorangeBlock[i], psuedorangeBlock[j])
                                    # Elevation is in degrees, convert to radians
                                    elev_rad = math.radians(elvIdBlock[i])
                                    vtec = verticalIntegration(tec, elev_rad)
                                    self.latest_vtec = vtec
                                    #convert to TECU
                                    slant_tecu = tec / 1e14  # Convert to TECU
                                    print(f"Slant TEC: {slant_tecu} TECU")
            except Exception as e:
                continue

async def tec_websocket_server(reader, host="0.0.0.0", port=8765):
    async def handler(websocket, path):
        while True:
            vtec = reader.latest_vtec
            if vtec is not None:
                await websocket.send(str(vtec))
            await asyncio.sleep(1)
    async with websockets.serve(handler, host, port):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    #may require adjustment based on host system
    #use portfinder.py to find the correct port 
    reader = TECReader(port="/dev/ttyACM0", baudrate=38400, timeout=3)
    reader.start()
    try:
        asyncio.run(tec_websocket_server(reader))
    except KeyboardInterrupt:
        reader.stop()
