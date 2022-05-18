#!/usr/bin/env python

import sys, os
import rospy
import  Rbk2Adf24Tx2Rx8 as Rbk2Adf24Tx2Rx8
import  time as time
import  numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from PIL import Image
# from    pyqtgraph.Qt import QtGui, QtCore
# import  pyqtgraph as pg


def talker():
        
    # Configure script
    Disp_FrmNr = 1
    Disp_TimSig = 0     # display time signals
    Disp_RP = 0         # display range profile
    Disp_RD = 1         # display range-Doppler map


    c0 = 299792458

    #--------------------------------------------------------------------------
    # Setup Connection
    #--------------------------------------------------------------------------
    #Brd     =   Rbk2Adf24Tx2Rx8.Rbk2Adf24Tx2Rx8('RadServe', '127.0.0.1', 8000, '192.168.1.1')
    Brd = Rbk2Adf24Tx2Rx8.Rbk2Adf24Tx2Rx8('PNet','192.168.1.1')

    # Verify if sampling framework is installed
    Brd.BrdChkSocSysId()

    Brd.BrdRst()
    Brd.BrdPwrEna()

    # Use 40 MHz clock for Range Doppler processing. The standard 20 MHz can
    # cause problems with the synchronisation of the sampling and the ramps
    Brd.BrdSetRole('Ms', 40e6)

    #--------------------------------------------------------------------------
    # Configure Receiver and Transmitters (static setup with single Tx turned on)
    #--------------------------------------------------------------------------
    Brd.RfRxEna()
    Brd.RfTxEna(1, 60)

    #--------------------------------------------------------------------------
    # Configure AFE5801
    #--------------------------------------------------------------------------
    Brd.Set('AfeIntDcCoupling',1)
    Brd.Set('AfeGaindB',20)

    #--------------------------------------------------------------------------
    # Configure Up-Chirp
    #--------------------------------------------------------------------------
    dCfg = dict()
    dCfg['fStrt'] = 24.0e9         
    dCfg['fStop'] = 24.25e9
    dCfg['TRampUp'] = 128e-6
    dCfg['TInt'] = 100e-3
    dCfg['N'] = 512
    dCfg['IniTim'] = 500e-3
    dCfg['IniEve'] = 0
    dCfg['Np'] = 64                     # number of chirps for range doppler
    dCfg['Tp'] = 0.25e-3                # chirp repetition interval

    #--------------------------------------------------------------------------
    # Configure DMA Transfer to copy Cfg.NLoop frames simultaneously.
    # Required to achiev maximal data transfer between FPGA and Soc 
    Brd.Set('DmaMult', dCfg['Np'])
    # Copy only the data of a single channel. RX1. The data of the residual channels
    # is sampled but not transfered
    Brd.Set('NrChn',1)

    #--------------------------------------------------------------------------
    # Use FPGA triggered measurement mode:
    # FPGA generates timing and triggers RCC1010
    # The timing is generated in multiples of ADC clock cycles. The default
    # clock is set to 20 MHz.
    #--------------------------------------------------------------------------
    Brd.RfMeas('ExtTrigUp',dCfg)

    #--------------------------------------------------------------------------
    # Read configured values; 
    # In the FPGA the CIC sampling rate reduction filter is configured
    # automatically; only an integer R reduction value can be configured;
    # Readback the actual sampling frequency
    #--------------------------------------------------------------------------
    fs = Brd.Get('fs')
    N = int(Brd.Get('N'))
    Np = int(dCfg['Np'])
    NrChn = Brd.Get('NrChn')
    #--------------------------------------------------------------------------
    # Check TCP/IP data rate:
    # 16 Bit * Number of Enabled Channels * Number of Samples are measureed in
    # the interval TInt. If the data rate is too high, than frames can be losed
    #--------------------------------------------------------------------------
    DataRate = 16*NrChn*N*dCfg['Np']/dCfg['TInt']
    print('DataRate: ', (DataRate/1e6), 'MBit/s')
    DataRate = 16*NrChn*N/dCfg['Tp']
    print('DataRateInt: ', (DataRate/1e6), 'MBit/s')
    # Configure Signal Processing
    #--------------------------------------------------------------------------
    # Processing of range profile
    Win2D = Brd.hanning(N-1,Np)
    ScaWin = sum(Win2D[:,0])
    NFFT = 2**10
    NFFTVel = 2**9
    kf = (dCfg['fStop'] - dCfg['fStrt'])/dCfg['TRampUp']
    vRange = np.arange(NFFT//2)/NFFT*fs*c0/(2*kf)
    fc = (dCfg['fStop'] + dCfg['fStrt'])/2

    RMin = 1
    RMax = 100
    RMinIdx = np.argmin(np.abs(vRange - RMin))
    RMaxIdx = np.argmin(np.abs(vRange - RMax))
    vRangeExt = vRange[RMinIdx:RMaxIdx]

    WinVel2D = Brd.hanning(int(dCfg['Np']), len(vRangeExt))
    ScaWinVel = sum(WinVel2D[:,0])
    WinVel2D = WinVel2D.transpose()

    vFreqVel = np.arange(-NFFTVel//2,NFFTVel//2)/NFFTVel*(1/dCfg['Tp'])
    vVel = vFreqVel*c0/(2*fc)

    ChnSel = 0
    pub_data = rospy.Publisher('radarbook/data', numpy_msg(Floats),queue_size=10)
    pub_RP = rospy.Publisher('radarbook/RP', numpy_msg(Floats),queue_size=10)
    pub_RDMap = rospy.Publisher('radarbook/RDMap', numpy_msg(Floats), queue_size=10)
    rospy.init_node('radarbook_pub', anonymous=True)
    r = rospy.Rate(10) # 10hz
    for Cycles in range(0, 1000):
        if rospy.is_shutdown():
            break
        Data = Brd.BrdGetData(dCfg['Np'])
        print(">>> Data.shape: ", Data.shape)
        if Disp_FrmNr  > 0:
            print(Data[0,:])

        # Reshape measurement data for range doppler processing
        # 2D array with [N, NLoop]
        MeasChn = np.reshape(Data[:,ChnSel],(N, Np), order='F')

        # Calculate range profiles and display them
        RP = 2*np.fft.fft(MeasChn[1:,:]*Win2D, n=NFFT, axis=0)/ScaWin*Brd.FuSca
        RP = RP[RMinIdx:RMaxIdx,:]
        print(">>> RP.shape: ", RP.shape)

        if Disp_RD > 0:
            # Display range doppler map
            RD = np.fft.fftshift(np.fft.fft(RP*WinVel2D, NFFTVel, axis=1)/ScaWinVel, axes=1)
            RDMap = 20*np.log10(np.abs(RD))
            flat_arr = RDMap.flatten().astype(np.float32)
            print("RDMap.shape: ", RDMap.shape)
            print("flat_arr.shape: ", flat_arr.shape)
            pub_RDMap.publish(flat_arr)

        pub_data.publish(Data.astype(np.float32))
        pub_RP.publish(RP.astype(np.float32))
        r.sleep()
    Brd.BrdRst()
    Brd.BrdPwrDi()

    del Brd




if __name__ == '__main__':
    print("haha")
    talker()
