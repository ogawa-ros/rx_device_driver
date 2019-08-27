#!/usr/bin/env python3

from NASCORX_System.device import FSW_0020
import rospy
import std_msgs.msg

class fsw_0020(object):
    '''
    DESCRIPTION
    ================
    This class cntrols the FSW0020.

    ARGUMENTS
    ================
    1. IP: IP address of the FSW0020
        Type: string
        Default: '192.168.100.1'
    2. port: port number of the FSW0020
        Type: int
        Default: 10001
    '''

    def __init__(self, IP='192.168.100.1', port=10001):
        self.IP = IP
        self.port = port
        self.com = pymeasure.ethernet(self.IP, self.port)
        self.sg = pymeasure.Phasematrix.FSW0020(self.com)
        
        def set_freq(self, freq):
        """
        DESCRIPTION
        ================
        This function sets the frequency of the SG.

        ARGUMENTS
        ================
        1. freq: CW frequency [GHz]
            Number: 0.5-20
            Type: float
            Default: nothing

        RETURNS
        ================
        Nothing.
        """
        if 0.5<=freq<=20.0:
            self.sg.freq_set(freq, 'GHz')
        else:
            print('!!!!ERROR!!!!')
            print('invalid freq: '+str(freq))
            print('available freq: 0.5-20 [GHz]')
            return
        
        def query_freq(self):
        """
        DESCRIPTION
        ================
        This function queries the frequency of the SG.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. freq: CW frequency [GHz]
            Type: float
        """
        ret = self.sg.freq_query()
        freq = float(ret)/1e+12
        return freq
    
    def set_power(self, power):
        """
        DESCRIPTION
        ================
        This function sets the power of the SG.

        ARGUMENTS
        ================
        1. power: CW power [dBm]
            Number: -10-+13
            Type: float
            Default: nothing

        RETURNS
        ================
        Nothing.
        """
        if -10.0<=power<=13.0:
            self.sg.power_set(power, 'dBm')
        else:
            print('!!!!ERROR!!!!')
            print('invalid power: '+str(power))
            print('available power: -10 - +13 [dBm]')
            return
        
        def query_power(self):
        """
        DESCRIPTION
        ================
        This function sets the power of the SG.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. power: CW power [dBm]
            Type: float
        """
        ret = self.sg.power_query()
        power = float(ret)
        return power

    
    def set_output(self, onoff=0):
        """
        DESCRIPTION
        ================
        This function switches the RF output.

        RETURNS
        ================
        Nothing.
        """
        if onoff==0:
            self.sg.output_set('OFF')
        elif onoff==1:
            self.sg.output_set('ON')
        else:
            print('!!!!ERROR!!!!')
            print('invalid onoff: '+str(onoff))
            print('available onoff: 0 or 1')
        return
        
    def query_output(self):
        """
        DESCRIPTION
        ================
        This function queries the output status.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        3. onoff: RF ON/OFF status
            Type: int(1: ON, 0: OFF)
        """
        ret = self.sg.output_query()
        switch = ret.startswith('OFF')
        if switch==True:
            onoff = 0
        else:
            onoff = 1

        def set_ref(self, source='INT'):
        """
        DESCRIPTION
        ================
        This function selects the reference source.

        ARGUMENTS
        ================
        1. source: reference source
            Number: 'INT' or 'EXT'
            Type: string
            Default: 'INT'

        RETURNS
        ================
        Nothing.
        """
        if source=='EXT':
            self.com.send('ROSC:SOUR EXT')
        else:
            self.com.send('ROSC:SOUR INT')
            return
        
        def query_ref(self):
        """
        DESCRIPTION
        ================
        This function queries the reference sources.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. ref: reference source INTERNAL or EXTERNAL.
            Type: string (INT/EXT)
        """
        self.com.send('ROSC:SOUR?')
        ret = self.com.readline()
        ref = ret.startswith('EXT')
        if ref == 1:
            ref = 'EXT'
        else:
            ref = 'INT'
            return ref
        
        def set_refout(self, onoff=0):
        """
        DESCRIPTION
        ================
        This function turns on or off the reference signal output.

        ARGUMENTS
        ================
        1. onoff: reference signal on or off
            Number: 1 or 0
            Type: int
            Default: 0

        RETURNS
        ================
        Nothing.
        """
        if onoff==1:
            self.com.send('OUTP:ROSC:STAT ON')
        else:
            self.com.send('OUTP:ROSC:STAT OFF')
            return
        
        def query_refout(self):
        """
        DESCRIPTION
        ================
        This function queries the reference signal output state.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. onoff: reference signal output state
            Type: int (1 or 0)
        """
        self.com.send('OUTP:ROSC:STAT?')
        ret = self.com.readline()
        frag = ret.startswith('OFF')
        if frag==1:
            onoff = 0
        elif frag==0:
            onoff = 1
        else:
            print('!!!!ERROR!!!!')
            print('_query_refout_() has invalid value.')
            return onoff
        
        def query_temp(self):
        """
        DESCRIPTION
        ================
        This function queries the temperature.
        !!!!WARNING!!!!
            *Operating temperature is 0 to 55 deg C.
            *You must use heat sink to operate.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. temp: temperature [deg C]
            Type: float
        """
        self.com.send('DIAG:MEAS? 21')
        ret = self.com.readline()
        temp = float(ret)
        return temp                               



class FSW_0020(object):

    def __init__(self):
        
