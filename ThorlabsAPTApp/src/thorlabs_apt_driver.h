/*
	EPICS asyn driver for Thorlabs APT motor controllers
	June 2018, M. W. Bruker
*/

#ifndef _THORLABS_APT_DRIVER_H
#define _THORLABS_APT_DRIVER_H

#include "asynPortDriver.h"
#include "stdint.h"

class asynUser;

#define P_NumEvents_String "NUM_EVENTS"
#define P_LastEvent_String "LAST_EVENT"
#define P_LastEventNotes_String "LAST_EVENT_NOTES"

#define P_SerialNumber_String "SERIAL_NUMBER"
#define P_ModelNumber_String "MODEL_NUMBER"
#define P_FirmwareVersionMinor_String "FIRMWARE_VERSION_MINOR"
#define P_FirmwareVersionInterim_String "FIRMWARE_VERSION_INTERIM"
#define P_FirmwareVersionMajor_String "FIRMWARE_VERSION_MAJOR"
#define P_Notes_String "NOTES"
#define P_HardwareVersion_String "HARDWARE_VERSION"
#define P_ModState_String "MOD_STATE"
#define P_NumberChannels_String "NUMBER_CHANNELS"

#define P_ChEnabled_Format "%s_ENABLED"
#define P_MinVelocity_Format "%s_MIN_VELOCITY"
#define P_Acceleration_Format "%s_ACCELERATION"
#define P_MaxVelocity_Format "%s_MAX_VELOCITY"
#define P_Backlash_Format "%s_BACKLASH"
#define P_CurrentPosition_Format "%s_CURRENT_POSITION"
#define P_CurrentVelocity_Format "%s_CURRENT_VELOCITY"
#define P_StatusBits_Format "%s_STATUS_BITS"

#define P_MoveAbsolute_Format "%s_MOVE_ABSOLUTE"
#define P_MoveStop_Format "%s_MOVE_STOP"
#define P_MoveHome_Format "%s_MOVE_HOME"

class ThorlabsAPTDriver : public asynPortDriver {
public:
    ThorlabsAPTDriver(const char *portName, const char *serialPortName, int n_channel);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    void pollThread();
protected:
    struct ioPvt {
        asynOctet *pasynOctet;
        void *octetPvt;
    };
    asynStatus sendPacket(unsigned char *dataToSend, size_t sendLen);
    asynStatus sendLongCommand(int chd, unsigned short int commandId, unsigned char *extraData, size_t extraDataLen);
    asynStatus sendShortCommand(int chd, unsigned short int commandId, unsigned char p1 = 0, unsigned char p2 = 0);
    unsigned short int recvPacket(char **extraData, size_t *extraDataLen);
    void processUnsolicitedMessage(unsigned short int messageId, unsigned char *extraData, size_t extraDataLen);
    asynStatus waitForReply(unsigned short int expectedReplyId, char **extraData = 0, size_t *extraDataLen = 0);
    
    char *formatChString(const char*, int);

    void requestStatusUpdate(int ch);
    asynStatus setVelocityParams(int ch);
    
    static const unsigned char deviceAddress = 0x50;
    static const int MAX_CHANNELS = 4;
    
    int nCh;

    int P_NumEvents;
    int P_LastEvent;
    int P_LastEventNotes;
    
    int P_SerialNumber;
    int P_ModelNumber;
    int P_FirmwareVersionMinor;
    int P_FirmwareVersionInterim;
    int P_FirmwareVersionMajor;
    int P_Notes;
    int P_HardwareVersion;
    int P_ModState;
    int P_NumberChannels;

    // The following quantities are defined for each motor channel.

    int P_ChEnabled[MAX_CHANNELS];
    int P_MinVelocity[MAX_CHANNELS];
    int P_Acceleration[MAX_CHANNELS];
    int P_MaxVelocity[MAX_CHANNELS];
    int P_Backlash[MAX_CHANNELS];
    int P_CurrentPosition[MAX_CHANNELS];
    int P_CurrentVelocity[MAX_CHANNELS];
    int P_StatusBits[MAX_CHANNELS];
    
    int P_MoveAbsolute[MAX_CHANNELS];
    int P_MoveStop[MAX_CHANNELS];
    int P_MoveHome[MAX_CHANNELS];

    asynUser *asynUserSerial;

    char *channelPrefixes[MAX_CHANNELS] = { const_cast<char *>("Ch1"),
                                 const_cast<char *>("Ch2"),
                                 const_cast<char *>("Ch3"),
                                 const_cast<char *>("Ch4") };
};


#endif

