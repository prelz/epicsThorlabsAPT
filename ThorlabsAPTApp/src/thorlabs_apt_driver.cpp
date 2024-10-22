/*
	EPICS asyn driver for Thorlabs APT motor controllers
	June 2018, M. W. Bruker

    Multi-channel hardware is currently unsupported because I don't have any.
    Support for it could be added easily, though, and additions to the code are welcome.
*/

#include "thorlabs_apt_driver.h"
#include <asynDriver.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <epicsThread.h>
#include <asynOctetSyncIO.h>
#include <stdint.h>
#include <string.h>
#include <cantProceed.h>

#define MGMSG_HW_REQ_INFO               0x0005
#define MGMSG_HW_GET_INFO               0x0006
#define MGMSG_HW_NO_FLASH_PROGRAMMING   0x0018
#define MGMSG_HW_RESPONSE               0x0080
#define MGMSG_HW_RICHRESPONSE           0x0081
#define MGMSG_MOD_SET_CHANENABLESTATE   0x0210
#define MGMSG_MOD_REQ_CHANENABLESTATE   0x0211
#define MGMSG_MOD_GET_CHANENABLESTATE   0x0212
#define MGMSG_MOT_SET_VELPARAMS         0x0413
#define MGMSG_MOT_REQ_VELPARAMS         0x0414
#define MGMSG_MOT_GET_VELPARAMS         0x0415
#define MGMSG_MOT_SET_GENMOVEPARAMS     0x043A
#define MGMSG_MOT_REQ_GENMOVEPARAMS     0x043B
#define MGMSG_MOT_GET_GENMOVEPARAMS     0x043C
#define MGMSG_MOT_MOVE_HOME             0x0443
#define MGMSG_MOT_SET_MOVEABSPARAMS	0x0450
#define MGMSG_MOT_REQ_MOVEABSPARAMS	0x0451
#define MGMSG_MOT_GET_MOVEABSPARAMS	0x0452
#define MGMSG_MOT_MOVE_ABSOLUTE         0x0453
#define MGMSG_MOT_MOVE_STOP             0x0465
#define MGMSG_MOT_REQ_DCSTATUSUPDATE    0x0490
#define MGMSG_MOT_GET_DCSTATUSUPDATE    0x0491

// yet to be implemented:
// S. 55
#define MGMSG_MOT_MOVE_HOMED            0x0444
// S. 58
#define MGMSG_MOT_MOVE_COMPLETED        0x0464
// S. 59
// S. 63
#define MGMSG_MOT_MOVE_STOPPED          0x0466
// S. 97
//#define MGMSG_MOT_GET_STATUSUPDATE      0x0481
//#define MGMSG_MOT_REQ_STATUSUPDATE      0x0480
// S. 100
#define MGMSG_MOT_ACK_DCSTATUSUPDATE    0x0492


void ThorlabsAPTPollThreadC(void *drvPvt)
{
    ((ThorlabsAPTDriver *) drvPvt)->pollThread();
}
char *
ThorlabsAPTDriver::formatChString(const char *fmt, int ch)
{
    int n_alloc = snprintf(NULL, 0, fmt, channelPrefixes[ch]) + 1;
    char *ret = (char *)malloc(n_alloc);
    snprintf(ret, n_alloc, fmt, channelPrefixes[ch]);
    return ret;
}

ThorlabsAPTDriver::ThorlabsAPTDriver(const char *portName, const char *serialPortName, int n_channels=1)
   : asynPortDriver(portName,
                    1, /* maxAddr */
                    asynInt32Mask | asynUInt32DigitalMask | asynOctetMask | asynDrvUserMask,
                    asynInt32Mask | asynUInt32DigitalMask | asynOctetMask,
                    ASYN_CANBLOCK, /* asynFlags */
                    1, /* autoConnect */
                    0, /* default priority */
                    0), /* default stack size */
    nCh(n_channels)
{
    asynStatus status;
    asynInterface *pasynInterface;
    struct ioPvt *pioPvt;
    
    if (nCh > MAX_CHANNELS) nCh = MAX_CHANNELS - 1;
    if (nCh <= 0) nCh = 1;

    pioPvt = (struct ioPvt *) callocMustSucceed(1, sizeof(struct ioPvt), "ThorlabsAPT");
    asynUserSerial = pasynManager->createAsynUser(0, 0);
    asynUserSerial->userPvt = pioPvt;
    status = pasynManager->connectDevice(asynUserSerial, serialPortName, 0);
    if (status != asynSuccess) {
        printf("Cannot connect to port %s: %s\n", serialPortName, asynUserSerial->errorMessage);
        return;
    }
    pasynInterface = pasynManager->findInterface(asynUserSerial, asynOctetType, 1);
    if (!pasynInterface) {
        printf("%s interface not supported\n", asynOctetType);
        return;
    }
    pioPvt->pasynOctet = (asynOctet *) pasynInterface->pinterface;
    pioPvt->octetPvt = pasynInterface->drvPvt;

    pasynManager->lockPort(asynUserSerial);

    sendShortCommand(MGMSG_HW_NO_FLASH_PROGRAMMING);

    unsigned char *data;
    size_t dataLen;
    sendShortCommand(MGMSG_HW_REQ_INFO);
    status = waitForReply(MGMSG_HW_GET_INFO, (char **) &data, &dataLen);
    if (status == asynTimeout) {
        pasynManager->unlockPort(asynUserSerial);
        printf("ThorlabsAPT: timeout waiting for message MGMSG_HW_GET_INFO\n");
        return;
    }
    if (dataLen != 84) {
        pasynManager->unlockPort(asynUserSerial);
        printf("ThorlabsAPT: malformed message MGMSG_HW_GET_INFO\n");
        return;
    }
    
    createParam(P_NumEvents_String, asynParamInt32, &P_NumEvents);
    setIntegerParam(P_NumEvents, 0);
    createParam(P_LastEvent_String, asynParamInt32, &P_LastEvent);
    createParam(P_LastEventNotes_String, asynParamOctet, &P_LastEventNotes);
    
    unsigned long int serialNumber = data[3] << 24 | data[2] << 16 | data[1] << 8 | data[0];
    createParam(P_SerialNumber_String, asynParamInt32, &P_SerialNumber);
    setIntegerParam(P_SerialNumber, serialNumber);

    char modelNumber[9];
    memcpy(modelNumber, data + 4, 8);
    modelNumber[8] = 0;
    createParam(P_ModelNumber_String, asynParamOctet, &P_ModelNumber);
    setStringParam(P_ModelNumber, modelNumber);
    
    createParam(P_FirmwareVersionMinor_String, asynParamInt32, &P_FirmwareVersionMinor);
    setIntegerParam(P_FirmwareVersionMinor, data[14]);
    createParam(P_FirmwareVersionInterim_String, asynParamInt32, &P_FirmwareVersionInterim);
    setIntegerParam(P_FirmwareVersionInterim, data[15]);
    createParam(P_FirmwareVersionMajor_String, asynParamInt32, &P_FirmwareVersionMajor);
    setIntegerParam(P_FirmwareVersionMajor, data[16]);

    char notes[49];
    memcpy(notes, data + 18, 48);
    notes[48] = 0;
    createParam(P_Notes_String, asynParamOctet, &P_Notes);
    setStringParam(P_Notes, notes);

    unsigned short int hardwareVersion = (data[79] << 8) | data[78];
    createParam(P_HardwareVersion_String, asynParamInt32, &P_HardwareVersion);
    setIntegerParam(P_HardwareVersion, hardwareVersion);
    
    unsigned short int modState = (data[81] << 8) | data[80];
    createParam(P_ModState_String, asynParamInt32, &P_ModState);
    setIntegerParam(P_ModState, modState);

    unsigned short int numberChannels = (data[83] << 8) | data[82];
    createParam(P_NumberChannels_String, asynParamInt32, &P_NumberChannels);
    setIntegerParam(P_NumberChannels, numberChannels);

    free(data);

    printf("ThorlabsAPT: model %s; fw ver %u.%u.%u; hw ver %u; %u channels\n", modelNumber, data[16], data[15], data[14], hardwareVersion, numberChannels);

    for (int ch=0; ch < nCh; ++ch) {

        char *chSt = formatChString(P_MoveAbsolute_Format, ch);
        if (chSt == NULL) {
            printf("ThorlabsAPT: Out of memory.\n");
            return;
        }
        createParam(chSt, asynParamInt32, &(P_MoveAbsolute[ch]));
        free(chSt);

        chSt = formatChString(P_MoveStop_Format, ch);
        if (chSt == NULL) {
            printf("ThorlabsAPT: Out of memory.\n");
            return;
        }
        createParam(chSt, asynParamInt32, &(P_MoveStop[ch]));
        free(chSt);

        chSt = formatChString(P_MoveHome_Format, ch);
        if (chSt == NULL) {
            printf("ThorlabsAPT: Out of memory.\n");
            return;
        }
        createParam(chSt, asynParamInt32, &(P_MoveHome[ch]));
        free(chSt);
        
        // Get enabled state for current channel
        sendShortCommand(MGMSG_MOD_REQ_CHANENABLESTATE, ch+1, 0);
        status = waitForReply(MGMSG_MOD_GET_CHANENABLESTATE, (char **) &data, &dataLen);
        if (status == asynTimeout) {
            pasynManager->unlockPort(asynUserSerial);
            printf("ThorlabsAPT: timeout waiting for message MGMSG_MOD_GET_CHANENABLESTATE (ch #%d)\n", ch+1);
            return;
        }
        if (dataLen != 2) {
            pasynManager->unlockPort(asynUserSerial);
            printf("ThorlabsAPT: malformed message MGMSG_MOD_GET_CHANENABLESTATE (ch #%d)\n", ch+1);
            return;
        }
        chSt = formatChString(P_ChEnabled_Format, ch);
        if (chSt == NULL) {
            printf("ThorlabsAPT: Out of memory.\n");
            return;
        }
        createParam(chSt, asynParamInt32, &(P_ChEnabled[ch]));
        free(chSt);
        setIntegerParam(P_ChEnabled[ch], data[1] == 1 ? 1 : 0);
        free(data);

        // Get velocity parameters for current channel
        sendShortCommand(MGMSG_MOT_REQ_VELPARAMS, ch+1, 0);
        status = waitForReply(MGMSG_MOT_GET_VELPARAMS, (char **) &data, &dataLen);
        if (status == asynTimeout) {
            pasynManager->unlockPort(asynUserSerial);
            printf("ThorlabsAPT: timeout waiting for message MGMSG_MOT_GET_VELPARAMS (ch #%d)\n", ch+1);
            return;
        }
        if (dataLen != 14) {
            pasynManager->unlockPort(asynUserSerial);
            printf("ThorlabsAPT: malformed message MGMSG_MOT_GET_VELPARAMS (ch #%d)\n", ch+1);
            return;
        }

        chSt = formatChString(P_MinVelocity_Format, ch);
        if (chSt == NULL) {
            printf("ThorlabsAPT: Out of memory.\n");
            return;
        }
        createParam(chSt, asynParamInt32, &(P_MinVelocity[ch]));
        setIntegerParam(P_MinVelocity[ch], (data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2]);
        free(chSt);

        chSt = formatChString(P_Acceleration_Format, ch);
        if (chSt == NULL) {
            printf("ThorlabsAPT: Out of memory.\n");
            return;
        }
        createParam(chSt, asynParamInt32, &(P_Acceleration[ch]));
        setIntegerParam(P_Acceleration[ch], (data[9] << 24) | (data[8] << 16) | (data[7] << 8) | data[6]);
        free(chSt);

        chSt = formatChString(P_MaxVelocity_Format, ch);
        if (chSt == NULL) {
            printf("ThorlabsAPT: Out of memory.\n");
            return;
        }
        createParam(chSt, asynParamInt32, &(P_MaxVelocity[ch]));
        setIntegerParam(P_MaxVelocity[ch], (data[13] << 24) | (data[12] << 16) | (data[11] << 8) | data[10]);
        free(chSt);
        free(data);
        
        // Get destination of last absolute move command for current channel
        sendShortCommand(MGMSG_MOT_REQ_MOVEABSPARAMS, ch+1, 0);
        status = waitForReply(MGMSG_MOT_GET_MOVEABSPARAMS, (char **) &data, &dataLen);
        if (status == asynTimeout) {
            pasynManager->unlockPort(asynUserSerial);
            printf("ThorlabsAPT: timeout waiting for message MGMSG_MOT_GET_MOVEABSPARAMS (ch #%d)\n", ch+1);
            return;
        }
        if (dataLen != 6) {
            pasynManager->unlockPort(asynUserSerial);
            printf("ThorlabsAPT: malformed message MGMSG_MOT_GET_MOVEABSPARAMS (ch #%d)\n", ch+1);
            return;
        }
        setIntegerParam(P_MoveAbsolute[ch], (data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2]);
        free(data);
    
        // Get general move parameters (backlash) for current channel
        sendShortCommand(MGMSG_MOT_REQ_GENMOVEPARAMS, ch+1, 0);
        status = waitForReply(MGMSG_MOT_GET_GENMOVEPARAMS, (char **) &data, &dataLen);
        if (status == asynTimeout) {
            pasynManager->unlockPort(asynUserSerial);
            printf("ThorlabsAPT: timeout waiting for message MGMSG_MOT_GET_GENMOVEPARAMS (ch #%d)\n", ch+1);
            return;
        }
        if (dataLen != 6) {
            pasynManager->unlockPort(asynUserSerial);
            printf("ThorlabsAPT: malformed message MGMSG_MOT_GET_GENMOVEPARAMS (ch #%d)\n", ch+1);
            return;
        }
        chSt = formatChString(P_Backlash_Format, ch);
        if (chSt == NULL) {
            printf("ThorlabsAPT: Out of memory.\n");
            return;
        }
        createParam(chSt, asynParamInt32, &(P_Backlash[ch]));
        setIntegerParam(P_Backlash[ch], (data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2]);
        free(chSt);
        free(data);

        chSt = formatChString(P_CurrentPosition_Format, ch);
        if (chSt == NULL) {
            printf("ThorlabsAPT: Out of memory.\n");
            return;
        }
        createParam(chSt, asynParamInt32, &(P_CurrentPosition[ch]));
        free(chSt);

        chSt = formatChString(P_CurrentVelocity_Format, ch);
        if (chSt == NULL) {
            printf("ThorlabsAPT: Out of memory.\n");
            return;
        }
        createParam(chSt, asynParamInt32, &(P_CurrentVelocity[ch]));
        free(chSt);

        chSt = formatChString(P_StatusBits_Format, ch);
        if (chSt == NULL) {
            printf("ThorlabsAPT: Out of memory.\n");
            return;
        }
        createParam(chSt, asynParamUInt32Digital, &(P_StatusBits[ch]));
        free(chSt);
    }
    
    pasynManager->unlockPort(asynUserSerial);
    
    for (int ch=0; ch < nCh; ++ch) requestStatusUpdate(ch);
    
    status = (asynStatus) (epicsThreadCreate("ThorlabsAPTPollThread",
                                             epicsThreadPriorityMedium,
                                             epicsThreadGetStackSize(epicsThreadStackMedium),
                                             (EPICSTHREADFUNC) ThorlabsAPTPollThreadC,
                                             this) == NULL);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "epicsThreadCreate failed\n");
    }
}

void ThorlabsAPTDriver::pollThread()
{
    for (;;) {
        for (int ch=0; ch < nCh; ++ch) requestStatusUpdate(ch);
        epicsThreadSleep(0.1);
    }
}

asynStatus ThorlabsAPTDriver::sendPacket(unsigned char *dataToSend, size_t sendLen)
{
    asynStatus status = asynSuccess;
    size_t numBytes = 0;
    struct ioPvt *pioPvt = (struct ioPvt *) asynUserSerial->userPvt;

    // send message to device and read reply
    asynUserSerial->timeout = 1.0;
    status = pioPvt->pasynOctet->write(pioPvt->octetPvt, asynUserSerial, (char *) dataToSend, sendLen, &numBytes);

    return status;
}

asynStatus ThorlabsAPTDriver::sendLongCommand(unsigned short int commandId, unsigned char *extraData, size_t extraDataLen)
{
    unsigned char message[256];
    unsigned char *pMessage = message;

    if (extraDataLen > 250)
        return asynError;

    // command id, little endian
    *pMessage++ = commandId;
    *pMessage++ = commandId >> 8;

    // size of extra data
    *pMessage++ = extraDataLen;
    *pMessage++ = extraDataLen >> 8;

    // device address
    *pMessage++ = extraDataLen == 0 ? deviceAddress : (deviceAddress | 0x80);
    
    // source address
    *pMessage++ = 0x01;

    // extra data
    for (unsigned char i = 0; i < extraDataLen; ++i)
        *pMessage++ = extraData[i];
    
    return sendPacket(message, extraDataLen + 6);
}

asynStatus ThorlabsAPTDriver::sendShortCommand(unsigned short int commandId, unsigned char p1, unsigned char p2)
{
    unsigned char message[6];
    message[0] = commandId;
    message[1] = commandId >> 8;
    message[2] = p1;
    message[3] = p2;
    message[4] = deviceAddress;
    message[5] = 0x01;
    
    return sendPacket(message, 6);
}

unsigned short int ThorlabsAPTDriver::recvPacket(char **extraData, size_t *extraDataLen)
{
    size_t numBytes = 0, totalBytes = 0;
    struct ioPvt *pioPvt = (struct ioPvt *) asynUserSerial->userPvt;
    unsigned char message[50];
    
    // Find out if there is data to be read. If so, read the whole message, otherwise return immediately.
    // This is done to prevent the polling thread from locking the port unnecessarily.
    asynUserSerial->timeout = 0.0;
    pioPvt->pasynOctet->read(pioPvt->octetPvt, asynUserSerial, (char *) message, 6, &numBytes, 0);
    if (numBytes == 0)
        return 0;

    totalBytes = numBytes;
    while (totalBytes < 6) {
        asynUserSerial->timeout = 1.0;
        pioPvt->pasynOctet->read(pioPvt->octetPvt, asynUserSerial, (char *) &message[totalBytes], 6 - totalBytes, &numBytes, 0);

        if (!numBytes) {
            printf("ThorlabsAPT: timeout after reading %lu/6 bytes; message incomplete\n", totalBytes);
            return 0;
        }
        totalBytes += numBytes;
    }
    
    unsigned short int messageId = message[1] << 8 | message[0];
    
    size_t _extraDataLen = 0;
    char *_extraData = 0;
    if (message[4] & 0x80) {
        _extraDataLen = message[3] << 8 | message[2];
        _extraData = (char *) malloc(_extraDataLen < 2 ? 2 : _extraDataLen);
        
        totalBytes = 0;
        while (totalBytes < _extraDataLen) {
            asynUserSerial->timeout = 1.0;
            pioPvt->pasynOctet->read(pioPvt->octetPvt, asynUserSerial, _extraData + totalBytes, _extraDataLen - totalBytes, &numBytes, 0);
    
            if (!numBytes) {
                printf("ThorlabsAPT: message id 0x%x: timeout after reading %lu/%lu bytes of extra data; message incomplete\n", messageId, totalBytes, _extraDataLen);
                free(_extraData);
                return 0;
            }
            totalBytes += numBytes;
        }
    } else {
        _extraData = (char *) malloc(2);
        _extraData[0] = message[2];
        _extraData[1] = message[3];
        _extraDataLen = 2;
    }
    if (extraData)
        *extraData = _extraData;
    else
        free(_extraData);
    if (extraDataLen)
        *extraDataLen = _extraDataLen;

    return messageId;
}

void ThorlabsAPTDriver::processUnsolicitedMessage(unsigned short int messageId, unsigned char *extraData, size_t extraDataLen)
{
    switch (messageId) {
        case MGMSG_HW_RESPONSE: {
            int numEvents;
            getIntegerParam(P_NumEvents, &numEvents);
            setIntegerParam(P_NumEvents, numEvents + 1);
            setIntegerParam(P_LastEvent, extraData[1] << 8 | extraData[0]);
            callParamCallbacks();
            break;
        }
        case MGMSG_HW_RICHRESPONSE: {
            int numEvents;
            getIntegerParam(P_NumEvents, &numEvents);
            setIntegerParam(P_NumEvents, numEvents + 1);
            setIntegerParam(P_LastEvent, extraData[2] << 8 | extraData[1]);
            char buffer[65];
            buffer[64] = 0;
            memcpy(buffer, extraData + 4, 64);
            setStringParam(P_LastEventNotes, buffer);
            callParamCallbacks();
            break;
        }
        case MGMSG_MOT_MOVE_HOMED: {
            int ch = extraData[0] - 1;
            if ((ch >= 0) && (ch < nCh)) {
                setIntegerParam(P_MoveAbsolute[ch], 0);
            }
            break;
        }
        default: {
        }
    }
}

asynStatus ThorlabsAPTDriver::waitForReply(unsigned short int expectedReplyId, char **extraData, size_t *extraDataLen)
{
    unsigned short int replyId = 0;
    char *_extraData;
    size_t _extraDataLen;
    int timeoutCounter = 0;

    for (;;) {
        replyId = recvPacket(&_extraData, &_extraDataLen);
        for (timeoutCounter = 0; (timeoutCounter < 10) && !replyId; ++timeoutCounter) {
            epicsThreadSleep(0.05);
            replyId = recvPacket(&_extraData, &_extraDataLen);
        }
        if (timeoutCounter == 10)
            return asynTimeout;
        
        if (replyId == expectedReplyId) {
            *extraData = _extraData;
            *extraDataLen = _extraDataLen;
            return asynSuccess;
        }
    
        processUnsolicitedMessage(replyId, (unsigned char *) _extraData, _extraDataLen);
        free(_extraData);
    }
}

void ThorlabsAPTDriver::requestStatusUpdate(int ch=0)
{
    asynStatus status;
    unsigned char *data;
    size_t dataLen;
    
    lock();
    pasynManager->lockPort(asynUserSerial);
    sendShortCommand(MGMSG_MOT_REQ_DCSTATUSUPDATE, ch+1, 0);
    status = waitForReply(MGMSG_MOT_GET_DCSTATUSUPDATE, (char **) &data, &dataLen);
    pasynManager->unlockPort(asynUserSerial);
    
    if (status == asynTimeout) {
    	unlock();
        printf("ThorlabsAPT: timeout waiting for message MGMSG_MOT_GET_DCSTATUSUPDATE (ch #%d)\n", ch+1);
        return;
    }
    if (dataLen != 14) {
    	unlock();
        free(data);
        printf("ThorlabsAPT: malformed message MGMSG_MOT_GET_DCSTATUSUPDATE (ch #%d)\n", ch+1);
        return;
    }
    setIntegerParam(P_CurrentPosition[ch], (data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2]);
    setIntegerParam(P_CurrentVelocity[ch], (data[7] << 8) | data[6]);
    setUIntDigitalParam(P_StatusBits[ch], (data[13] << 24) | (data[12] << 16) | (data[11] << 8) | data[10], 0xffffffff);
    callParamCallbacks(0, 0);
    unlock();
    free(data);
}

asynStatus ThorlabsAPTDriver::setVelocityParams(int ch=0)
{
    unsigned char data[14];
    uint32_t value;
    
    // channel id
    data[0] = ch+1;
    data[1] = 0;
    
    // min velocity
    getIntegerParam(P_MinVelocity[ch], (epicsInt32 *) &value);
    data[2] = value;
    data[3] = value >> 8;
    data[4] = value >> 16;
    data[5] = value >> 24;

    // acceleration
    getIntegerParam(P_Acceleration[ch], (epicsInt32 *) &value);
    data[6] = value;
    data[7] = value >> 8;
    data[8] = value >> 16;
    data[9] = value >> 24;

    // max velocity
    getIntegerParam(P_MaxVelocity[ch], (epicsInt32 *) &value);
    data[10] = value;
    data[11] = value >> 8;
    data[12] = value >> 16;
    data[13] = value >> 24;
    
    return sendLongCommand(MGMSG_MOT_SET_VELPARAMS, data, 14);
}

asynStatus ThorlabsAPTDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    
    for (int ch = 0; ch < nCh; ++ch) {
        if (function == P_MoveAbsolute[ch]) {
            setIntegerParam(P_MoveAbsolute[ch], value);
            unsigned char data[6];
            data[0] = ch+1;
            data[1] = 0;
            data[2] = value;
            data[3] = value >> 8;
            data[4] = value >> 16;
            data[5] = value >> 24;
            sendLongCommand(MGMSG_MOT_SET_MOVEABSPARAMS, data, 6);
            return sendShortCommand(MGMSG_MOT_MOVE_ABSOLUTE);
        } else if (function == P_MoveStop[ch]) {
        	return sendShortCommand(MGMSG_MOT_MOVE_STOP, ch+1, 2);
        } else if (function == P_MoveHome[ch]) {
            return sendShortCommand(MGMSG_MOT_MOVE_HOME, ch+1, 0);
        } else if (function == P_ChEnabled[ch]) {
            setIntegerParam(P_ChEnabled[ch], value);
            return sendShortCommand(MGMSG_MOD_SET_CHANENABLESTATE, ch+1, value);
        } else if (function == P_MinVelocity[ch]) {
            setIntegerParam(P_MinVelocity[ch], value);
            return setVelocityParams(ch);
        } else if (function == P_Acceleration[ch]) {
            setIntegerParam(P_Acceleration[ch], value);
            return setVelocityParams(ch);
        } else if (function == P_MaxVelocity[ch]) {
            setIntegerParam(P_MaxVelocity[ch], value);
            return setVelocityParams(ch);
        } else if (function == P_Backlash[ch]) {
            setIntegerParam(P_Backlash[ch], value);
            unsigned char data[6];
            data[0] = ch+1;
            data[1] = 0;
            data[2] = value;
            data[3] = value >> 8;
            data[4] = value >> 16;
            data[5] = value >> 24;
            return sendLongCommand(MGMSG_MOT_SET_GENMOVEPARAMS, data, 6);
        }
    }
    return asynPortDriver::writeInt32(pasynUser, value);
}


extern "C" {

int ThorlabsAPTConfigure(const char *portName, const char *serialPortName, int n_channels=1)
{
    new ThorlabsAPTDriver(portName, serialPortName, n_channels); // scary but apparently the usual way
    return asynSuccess;
}

static const iocshArg initArg0 = { "portName", iocshArgString };
static const iocshArg initArg1 = { "serialPortName", iocshArgString };
static const iocshArg initArg2 = { "numChannels", iocshArgInt };

static const iocshArg * const initArgs[] = {&initArg0, &initArg1, &initArg2};
static const iocshFuncDef initFuncDef = { "ThorlabsAPTConfigure", 3, initArgs };
static void initCallFunc(const iocshArgBuf *args)
{
    ThorlabsAPTConfigure(args[0].sval, args[1].sval, args[2].ival);
}

void ThorlabsAPTDriverRegister()
{
    iocshRegister(&initFuncDef, initCallFunc);
}

epicsExportRegistrar(ThorlabsAPTDriverRegister);

}

