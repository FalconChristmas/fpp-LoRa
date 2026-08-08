#include <fpp-pch.h>

#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <cstring>

#include <unistd.h>
#include <termios.h>
#include <chrono>
#include <thread>

// HttpAppFramework.h must come before fpphttp.h: fpphttp.h undefines the
// trantor LOG_* macros, but drogon's own headers need them while compiling.
#include "fpphttp.h"
#include "common.h"
#include "settings.h"
#include "MultiSync.h"
#include "Plugin.h"
#include "Plugins.h"
#include "Sequence.h"
#include "log.h"

#include "channeloutput/serialutil.h"
#include "fppversion_defines.h"


enum {
    SET_SEQUENCE_NAME = 1,
    SET_MEDIA_NAME    = 2,

    START_SEQUENCE    = 3,
    START_MEDIA       = 4,
    STOP_SEQUENCE     = 5,
    STOP_MEDIA        = 6,
    SYNC              = 7,

    BLANK             = 9
};

// The port name comes out of the plugin's settings, which the API lets a client
// write, so it isn't necessarily the bare device name the UI dropdown offers.
// Only accept the serial devices content.php actually lists - that keeps a value
// like "../../etc/passwd" from turning "/dev/" + device into an arbitrary path.
static bool isAllowedSerialPort(const std::string &device) {
    static const char * const PREFIXES[] = {"ttyUSB", "ttyACM", "ttyAMA", "ttyS", "ttyO"};
    for (const char *prefix : PREFIXES) {
        size_t plen = strlen(prefix);
        if (device.compare(0, plen, prefix) == 0
            && device.length() > plen
            && device.find_first_not_of("0123456789", plen) == std::string::npos) {
            return true;
        }
    }
    LogWarn(VB_PLUGIN, "LoRa: refusing to use '%s' - not a recognized serial port name\n", device.c_str());
    return false;
}

class LoRaMultiSyncPlugin : public MultiSyncPlugin {
public:
    
    LoRaMultiSyncPlugin() {}
    virtual ~LoRaMultiSyncPlugin() {
        if (devFile >= 0) {
            SerialClose(devFile);
            devFile = -1;
        }
    }
    void setupPacket(char *buf, const std::string &modType) {
        buf[0] = 0xC0;
        if (modType == "E32-433T20D" || modType == "E32-915T30D") {
            // nothing else
        } else {
            buf[1] = 0x00;
            buf[2] = 0x07;
        }
    }
    void addMA(int MA, char *buf, const std::string &modType) {
        if (modType == "E32-433T20D" || modType == "E32-915T30D") {
            buf[1] = (MA >> 8) & 0xFF;
            buf[2] = MA & 0xFF;
        } else {
            buf[3] = (MA >> 8) & 0xFF;
            buf[4] = MA & 0xFF;
            buf[5] = 0x00; // NETID
        }
    }
    void addUBR(int UBR, char *buf, const std::string &modType) {
        if (modType == "E32-433T20D" || modType == "E32-915T30D") {
            char &f = buf[3];
            f &= 0b11000111;
            switch (UBR) {
                case 1200: f |= 0b00000000; break;
                case 2400: f |= 0b00001000; break;
                case 4800: f |= 0b00010000; break;
                case 9600: f |= 0b00011000; break;
                case 19200: f |= 0b00100000; break;
                case 38400: f |= 0b00101000; break;
                case 57600: f |= 0b00110000; break;
                case 115200: f |= 0b00111000; break;
            }
        } else {
            char &f = buf[6];
            f &= 0b00011111;
            switch (UBR) {
                case 1200: f |= 0b00000000; break;
                case 2400: f |= 0b00100000; break;
                case 4800: f |= 0b01000000; break;
                case 9600: f |= 0b01100000; break;
                case 19200: f |= 0b10000000; break;
                case 38400: f |= 0b10100000; break;
                case 57600: f |= 0b11000000; break;
                case 115200: f |= 0b11100000; break;
            }
        }
    }
    void addADR(int ADR, char *buf, const std::string &modType) {
        if (modType == "E32-433T20D" || modType == "E32-915T30D") {
            char &f = buf[3];
            // clear the last 3 bits
            f &= 0b11111000;
            switch (ADR) {
                case 300: f |= 0b00000000; break;
                case 1200: f |= 0b00000001; break;
                case 2400: f |= 0b00000010; break;
                case 4800: f |= 0b00000011; break;
                case 9600: f |= 0b00000100; break;
                case 19200: f |= 0b00000101; break;
            }
        } else if (modType == "E22-230T22U") {
            char &f = buf[6];
            // clear the last 3 bits
            f &= 0b11111000;
            switch (ADR) {
                case 2400:  f |= 0b00000010; break;
                case 4800:  f |= 0b00000100; break;
                case 9600:  f |= 0b00000101; break;
                case 15600: f |= 0b00000110; break;
            }
        } else if (modType == "E22-400T22U" || modType == "E22-900T22U") {
            char &f = buf[6];
            // clear the last 3 bits
            f &= 0b11111000;
            switch (ADR) {
                case 2400:  f |= 0b00000010; break;
                case 4800:  f |= 0b00000011; break;
                case 9600:  f |= 0b00000100; break;
                case 19200: f |= 0b00000101; break;
                case 38400: f |= 0b00000110; break;
                case 62500: f |= 0b00000111; break;
            }
        }
    }
    void addFLAGS(int FEC, int TXP, char *buf, const std::string &modType) {
        if (modType == "E32-433T20D" || modType == "E32-915T30D") {
            buf[5] = 0b0100'0000; //transparent transmission mode and pull ups
            if (FEC == 1) {
                buf[5] |= 0b0000'0100;
            }
            if (TXP == 1) {
                buf[5] |= 0b0000'0011;
            } else if (TXP == 2) {
                buf[5] |= 0b0000'0010;
            } else if (TXP == 3) {
                buf[5] |= 0b0000'0001;
            }
        } else {
            buf[7] = 0;
            if (TXP == 1) {
                buf[7] |= 0b0000'0011;
            } else if (TXP == 2) {
                buf[7] |= 0b0000'0010;
            } else if (TXP == 3) {
                buf[7] |= 0b0000'0001;
            }
        }
    }
    void addCH(float ch, char *buf, const std::string &modType) {
        if (modType == "E32-433T20D") {
            char &f = buf[4];
            int chi = ch - 410;
            f = chi;
        } else if (modType == "E32-915T30D") {
            char &f = buf[4];
            int chi = ch - 900;
            f = chi;
        } else if (modType == "E22-230T22U") {
            char &f = buf[8];
            ch *= 4;
            int chi = ch - (220 * 4);
            f = chi;
        } else if (modType == "E22-400T22U") {
            char &f = buf[8];
            int chi = ch - 410;
            f = chi;
        } else if (modType == "E22-900T22U") {
            char &f = buf[8];
            int chi = ch - 850;
            f = chi;
        }
    }
    void setupQuery(char buf[256], const std::string &modType, int &rl) {
        if (modType == "E32-433T20D" || modType == "E32-915T30D") {
            buf[0] = 0xC1;
            buf[1] = 0xC1;
            buf[2] = 0xC1;
            rl = 6;
        } else {
            buf[0] = 0xC1;
            buf[1] = 0;
            buf[2] = 7;
            rl = 10;
        }
    }
    void printBuf(const char *buf, const char *pfx, int len) {
        char out[256];
        snprintf(out, sizeof(out), "%s (%d): ", pfx, len);
        for (int x = 0; x < len; x++) {
            int l = strlen(out);
            snprintf(out + l, sizeof(out) - l,  "%02X ", buf[x]);
        }
        LogDebug(VB_PLUGIN, "%s\n", out);
    }
    int sendCommand(int sdevFile, char *buf, int sendLen, int expRead) {
        int w = write(sdevFile, buf, sendLen);
        //printf("Wrote %d bytes\n", w);
        tcdrain(sdevFile);
        int i = read(sdevFile, buf, expRead);
        int count = 0;
        int total = i;
        while (i >= 0 && count < 1000 && total < expRead) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            count++;
            i = read(sdevFile, &buf[total], expRead - total);
            if (i > 0) {
                total += i;
            }
        }
        if (total == 0) {
            // didn't respond, we'll resend the command and see if that works
            w = write(sdevFile, buf, sendLen);
            //printf("Wrote %d bytes (attempt 2)\n", w);
            tcdrain(sdevFile);
            i = read(sdevFile, buf, expRead);
            count = 0;
            total = i;
            while (i >= 0 && count < 1000 && total < expRead) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                count++;
                i = read(sdevFile, &buf[total], expRead - total);
                if (i > 0) {
                    total += i;
                }
            }
        }
        return total;
    }

    // Drogon handler for the "/LoRa" route (reached via the web UI's POST to
    // api/plugin-apis/LoRa, which Apache proxies to localhost:32322/LoRa).
    // Reconfigures the attached LoRa module from the posted JSON settings.
    void handleConfigRequest(const drogon::HttpRequestPtr &req,
                             std::function<void(const drogon::HttpResponsePtr &)> &&callback) {
        bool reopen = false;
        if (devFile >= 0) {
            SerialClose(devFile);
            devFile = -1;
            reopen = true;
        }
        Json::Value json;
        std::string content(req->body());
        LoadJsonFromString(content, json);
        std::string modType = json["LoRaDeviceType"].asString();

        if (!startsWith(modType, "Waveshare")) {
            device = json["LoRaDevicePort"].asString();
            int MA = json["MA"].asInt();
            int UBR = json["UBR"].asInt();
            int ADR = json["ADR"].asInt();
            int FEC = json["FEC"].asInt();
            int TXP = json["TXP"].asInt();
            float CH = json["CH"].asFloat();

            // Validate before building the path, not after - nothing should
            // construct a /dev path out of an unchecked request value.  Only
            // ttyUSB/ttyACM/ttyAMA/ttyS/ttyO names are accepted; see
            // isAllowedSerialPort() above.  Note we can't early-return here:
            // this is a drogon handler and the response callback at the end of
            // the function still has to run.
            std::string devFileName;
            int sdevFile = -1;
            if (isAllowedSerialPort(device)) {
                devFileName = "/dev/" + device;
                sdevFile = SerialOpen(devFileName.c_str(), 9600, "8N1", true);
                if (sdevFile < 0) {
                    LogWarn(VB_PLUGIN, "Could not open %s to configure LoRa module\n", devFileName.c_str());
                }
            }
            if (sdevFile >= 0) {
                char buf[256];
                memset(buf, 0, sizeof(buf));
                int packetLen;
                setupQuery(buf, modType, packetLen);
                int w = sendCommand(sdevFile, buf, 3, packetLen);
                printBuf(buf, "C1", w);

                setupPacket(buf, modType);
                addMA(MA, buf, modType);
                addUBR(UBR, buf, modType);
                addADR(ADR, buf, modType);
                addCH(CH, buf, modType);
                addFLAGS(FEC, TXP, buf, modType);
                printBuf(buf, "C0S", packetLen);
                w = sendCommand(sdevFile, buf, packetLen, packetLen);
                printBuf(buf, "C0E", w);
                if (w == 0) {
                    w = sendCommand(sdevFile, buf, packetLen, packetLen);
                    printBuf(buf, "C0E", w);
                }

                memset(buf, 0, sizeof(buf));
                setupQuery(buf, modType, packetLen);
                w = sendCommand(sdevFile, buf, 3, packetLen);
                printBuf(buf, "C1E", w);

                SerialClose(sdevFile);
                LogInfo(VB_PLUGIN, "LoRa Module Configured: %s\n", devFileName.c_str());
            }
        }
        loadSettings();
        if (reopen) {
            Init();
        }

        auto resp = drogon::HttpResponse::newHttpResponse();
        resp->setStatusCode(drogon::k200OK);
        resp->setContentTypeString("text/plain");
        resp->setBody("OK");
        callback(resp);
    }
    void writeWS(const char *buf, bool resp = true) {
        if (devFile >= 0) {
            write(devFile, buf, strlen(buf));
            tcdrain(devFile);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (resp) {
            char buf2[256];
            memset(buf2, 0, sizeof(buf2));
            int r = read(devFile, buf2, 256);
            //printBuf(buf2, "C1", r);
        }
    }
    void setupWaveshare() {
        char buf[256];
        writeWS("\r\n", false); //  Go into AT mode
        writeWS("+++\r\n"); //  Go into AT mode
        writeWS("AT+SF=7\r\n"); // 1 Spreading Factor = 7
        snprintf(buf, sizeof(buf), "AT+BW=0\r\n", ADR == 500 ? 2 : ((ADR == 250) ? 1 : 0)); // 2 Bandwidth = 125KHz
        writeWS(buf);
        writeWS("AT+CR=1\r\n"); // 3 Encoding Rate = 4/5
        int pwr = 22;
        if (TXP == 1) {
            pwr = 10;
        } else if (TXP == 2) {
            pwr = 13;
        } else if (TXP == 3) {
            pwr = 17;
        }
        snprintf(buf, sizeof(buf), "AT+PWR=%d\r\n", pwr); // 4 Set Power to Low Value
        writeWS(buf);
        writeWS("AT+NETID=0\r\n"); // 5 Network ID = 0

        snprintf(buf, sizeof(buf), "AT+ADDR=%d\r\n", MA); // 6 Address = 0
        writeWS(buf);
        writeWS("AT+LBT=0\r\n"); // 7 Disable LBT
        writeWS("AT+MODE=1\r\n"); // 8 Stream Mode

        int ch = CH;
        if (modType == "Waveshare USB-TO-LoRa-HF") {
            ch -= 850;
        } else {
            ch -= 410;
        }
        snprintf(buf, sizeof(buf), "AT+TXCH=%d\r\n", ch); // 9 Transmit Channel
        writeWS(buf);
        snprintf(buf, sizeof(buf), "AT+RXCH=%d\r\n", ch); // 10 Receive Channel
        writeWS(buf);
        writeWS("AT+RSSI=0\r\n"); // 11 Set the RSSI enabled
        writeWS("AT+PORT=3\r\n"); // 12 Set Port to RS232
        snprintf(buf, sizeof(buf), "AT+BAUD=%d\r\n",baud); // 13 Set baudrate to 115200
        writeWS(buf);
        writeWS("AT+COMM=\"8N1\"\r\n"); // 14 Set Com port parameters 8N1
        writeWS("AT+KEY=0\r\n"); // 15 Disable encryption
        writeWS("AT+EXIT\r\n"); // 16 Exit AT mode  
    }

    bool Init() {
        // Only ttyUSB/ttyACM/ttyAMA/ttyS/ttyO names are accepted here; see
        // isAllowedSerialPort() above.  Checked before the path is built.
        if (!isAllowedSerialPort(device)) {
            return false;
        }
        std::string devFileName = "/dev/" + device;
        devFile = SerialOpen(devFileName.c_str(), baud, "8N1", getFPPmode() != REMOTE_MODE);
        if (devFile < 0) {
            LogWarn(VB_PLUGIN, "Could not open %s\n", devFileName.c_str());
            return false;
        } else {
            LogDebug(VB_PLUGIN, "LoRa Configured - %s    Baud: %d\n", devFileName.c_str(), baud);
        }
        if (startsWith(modType, "Waveshare")) {
            // Waveshare modules need a different setup
            setupWaveshare();
        }
        return true;
    }
    virtual void ShutdownSync(void) override {
        if (devFile >= 0) {
            SerialClose(devFile);
            devFile = -1;
        }
    }

    void send(char *buf, int len) {
        if (devFile >= 0) {
            write(devFile, buf, len);
            tcdrain(devFile);
        }
    }

    // A name packet is one type byte, the name, then a null terminator, so the
    // name itself has to fit in sizeof(buf) - 2.  Truncate an over-long name
    // instead of running off the end of the stack buffer - the remote just ends
    // up with a shortened name, which is far better than corrupting our stack.
    void sendNamePacket(char type, const std::string &filename) {
        char buf[256];
        size_t len = filename.length();
        if (len > sizeof(buf) - 2) {
            len = sizeof(buf) - 2;
            LogWarn(VB_SYNC, "LoRa: name '%s' too long for a sync packet, truncating to %d chars\n",
                    filename.c_str(), (int)len);
        }
        buf[0] = type;
        memcpy(&buf[1], filename.c_str(), len);
        buf[len + 1] = 0;
        send(buf, len + 2);
    }

    void SendSync(uint32_t frames, float seconds) {
        int diff = frames - lastSentFrame;
        float diffT = seconds - lastSentTime;
        bool sendSync = false;
        if (diffT > 0.5) {
            sendSync = true;
        } else if (!frames) {
            // no need to send the 0 frame
        } else if (frames < 32) {
            //every 8 at the start
            if (frames % 8 == 0) {
                sendSync = true;
            }
        } else if (diff == 16) {
            sendSync = true;
        }
        
        if (sendSync) {
            char buf[120];
            buf[0] = SYNC;
            memcpy(&buf[1], &frames, 4);
            memcpy(&buf[5], &seconds, 4);
            send(buf, 9);

            lastSentFrame = frames;
            lastSentTime = seconds;
        }
        lastFrame = frames;
    }

    virtual void SendSeqOpenPacket(const std::string &filename) override {
        sendNamePacket(SET_SEQUENCE_NAME, filename);
        lastSequence = filename;
        lastFrame = -1;
        lastSentTime = -1.0f;
        lastSentFrame = -1;
    }
    virtual void SendSeqSyncStartPacket(const std::string &filename) override {
        if (filename != lastSequence) {
            SendSeqOpenPacket(filename);
        }
        char buf[2];
        buf[0] = START_SEQUENCE;
        send(buf, 1);
        lastFrame = -1;
        lastSentTime = -1.0f;
        lastSentFrame = -1;
    }
    virtual void SendSeqSyncStopPacket(const std::string &filename) override {
        char buf[2];
        buf[0] = STOP_SEQUENCE;
        send(buf, 1);
        lastSequence = "";
        lastFrame = -1;
        lastSentTime = -1.0f;
        lastSentFrame = -1;
    }
    virtual void SendSeqSyncPacket(const std::string &filename, int frames, float seconds) override {
        if (filename != lastSequence) {
            SendSeqSyncStartPacket(filename);
        }
        SendSync(frames, seconds);
    }
    
    virtual void SendMediaOpenPacket(const std::string &filename) override {
        if (sendMediaSync) {
            sendNamePacket(SET_MEDIA_NAME, filename);
            lastMedia = filename;
            lastFrame = -1;
            lastSentTime = -1.0f;
            lastSentFrame = -1;
        }
    }
    virtual void SendMediaSyncStartPacket(const std::string &filename) override {
        if (sendMediaSync) {
            if (filename != lastMedia) {
                SendSeqOpenPacket(filename);
            }
            char buf[2];
            buf[0] = START_MEDIA;
            send(buf, 1);
            lastFrame = -1;
            lastSentTime = -1.0f;
            lastSentFrame = -1;
        }
    }
    virtual void SendMediaSyncStopPacket(const std::string &filename) override {
        if (sendMediaSync) {
            char buf[2];
            buf[0] = STOP_MEDIA;
            send(buf, 1);
            lastMedia = "";
            lastFrame = -1;
            lastSentTime = -1.0f;
            lastSentFrame = -1;
        }
    }
    virtual void SendMediaSyncPacket(const std::string &filename, float seconds) override {
        if (sendMediaSync) {
            if (filename != lastMedia) {
                SendMediaSyncStartPacket(filename);
            }
            SendSync(lastFrame > 0 ? lastFrame : 0, seconds);
        }
    }
    
    virtual void SendBlankingDataPacket(void) override {
        char buf[2];
        buf[0] = BLANK;
        send(buf, 1);
    }
    
    bool fullCommandRead(int &commandSize) {
        if (curPosition == 0) {
            return false;
        }
        switch (readBuffer[0]) {
        case SET_SEQUENCE_NAME:
        case SET_MEDIA_NAME:
            //need null terminated string
            for (commandSize = 0; commandSize < curPosition; commandSize++) {
                if (readBuffer[commandSize] == 0) {
                    commandSize++;
                    return true;
                }
            }
            return false;
        case SYNC:
            commandSize = 9;
            return curPosition >= 9;
        case START_SEQUENCE:
        case START_MEDIA:
        case STOP_SEQUENCE:
        case STOP_MEDIA:
        case BLANK:
            commandSize = 1;
            break;
        default:
            commandSize = 1;
            return false;
        }
        return true;
    }
    
    void addControlCallbacks(std::map<int, std::function<bool(int)>> &callbacks) {
        std::function<bool(int)> fn = [this](int d) {
            int i = read(devFile, &readBuffer[curPosition], 255-curPosition);
            if (i) {
                //printf("CB %d\n", i);
                curPosition += i;
                int commandSize = 0;
                while (fullCommandRead(commandSize)) {
                    if (readBuffer[0] == SYNC) {
                        LogExcess(VB_SYNC, "LoRa Callback - %d   (%d bytes of %d)\n", readBuffer[0], commandSize, curPosition);
                    } else {
                        LogDebug(VB_SYNC, "LoRa Callback - %d   (%d bytes of %d)\n", readBuffer[0], commandSize, curPosition);
                    }
                    switch (readBuffer[0]) {
                        case SET_SEQUENCE_NAME:
                            lastSequence = &readBuffer[1];
                            multiSync->OpenSyncedSequence(&readBuffer[1]);
                            if (bridgeToLocal) {
                                multiSync->SendSeqOpenPacket(&readBuffer[1]);
                            }
                            break;
                        case SET_MEDIA_NAME:
                            lastMedia = &readBuffer[1];
                            multiSync->OpenSyncedMedia(&readBuffer[1]);
                            if (bridgeToLocal) {
                                multiSync->SendMediaOpenPacket(&readBuffer[1]);
                            }
                            break;
                        case START_SEQUENCE:
                            if (lastSequence != "") {
                                multiSync->StartSyncedSequence(lastSequence.c_str());
                                multiSync->SyncSyncedSequence(lastSequence.c_str(), 0, 0);
                                if (bridgeToLocal) {
                                    multiSync->SendSeqSyncStartPacket(lastSequence);
                                    multiSync->SendSeqSyncPacket(lastSequence, 0, 0);
                                }
                            }
                            break;
                        case START_MEDIA:
                            if (lastMedia != "") {
                                multiSync->StartSyncedMedia(lastMedia.c_str());
                                multiSync->SyncSyncedMedia(lastMedia.c_str(), 0, 0);
                                if (bridgeToLocal) {
                                    multiSync->SendMediaSyncStartPacket(lastMedia);
                                    multiSync->SendMediaSyncPacket(lastMedia, 0);
                                }
                            }
                            break;
                        case STOP_SEQUENCE:
                            if (lastSequence != "") {
                                multiSync->StopSyncedSequence(lastSequence.c_str());
                                lastSequence = "";
                                if (bridgeToLocal) {
                                    multiSync->SendSeqSyncStopPacket(lastSequence);
                                }
                            }
                            break;
                        case STOP_MEDIA:
                            if (lastMedia != "") {
                                multiSync->StopSyncedMedia(lastMedia.c_str());
                                lastMedia = "";
                                if (bridgeToLocal) {
                                    multiSync->SendMediaSyncStopPacket(lastMedia);
                                }
                            }
                            break;
                        case SYNC: {
                                int frame;
                                memcpy(&frame, &readBuffer[1], 4);
                                float time;
                                memcpy(&time, &readBuffer[5], 4);
                            
                                if (lastSequence != "") {
                                    multiSync->SyncSyncedSequence(lastSequence.c_str(), frame, time);
                                    if (bridgeToLocal) {
                                        multiSync->SendSeqSyncPacket(lastSequence, frame, time);
                                    }
                                }
                                if (lastMedia != "") {
                                    multiSync->SyncSyncedMedia(lastMedia.c_str(), frame, time);
                                    if (bridgeToLocal) {
                                        multiSync->SendMediaSyncPacket(lastMedia, time);
                                    }
                                }
                            }
                            break;
                        case BLANK:
                            sequence->SendBlankingData();
                            if (bridgeToLocal) {
                                multiSync->SendBlankingDataPacket();
                            }
                            break;
                        default:
                            LogWarn(VB_SYNC, "Unknown command   cmd: %d    (%d bytes)\n", readBuffer[0], curPosition);
                            break;
                    }
                    if (commandSize < curPosition) {
                        memcpy(readBuffer, &readBuffer[commandSize],  curPosition - commandSize);
                        curPosition -= commandSize;
                    } else {
                        curPosition = 0;
                    }
                    commandSize = 0;
                }
            } else {
                LogExcess(VB_SYNC, "LoRa Callback -  no data read: %d   (%d bytes)\n", readBuffer[0], curPosition);
            }
            return false;
        };
        callbacks[devFile] = fn;
    }

    bool loadSettings() {
        bool enabled = false;
        if (FileExists(FPP_DIR_CONFIG("/plugin.fpp-LoRa"))) {
            std::ifstream infile(FPP_DIR_CONFIG("/plugin.fpp-LoRa"));
            std::string line;
            while (std::getline(infile, line)) {
                std::istringstream iss(line);
                std::string a, b, c;
                if (!(iss >> a >> b >> c)) { break; } // error
                
                c.erase(std::remove( c.begin(), c.end(), '\"' ), c.end());
                if (a == "LoRaEnable") {
                    enabled = (c == "1");
                } else if (a == "LoRaBridgeEnable") {
                    bridgeToLocal = (c == "1");
                } else if (a == "LoRaDevicePort") {
                    device = c;
                } else if (a == "LoRaDeviceSpeed") {
                    baud = std::stoi(c);
                } else if (a == "LoRaMediaEnable") {
                    sendMediaSync = std::stoi(c) != 0;
                } else if (a == "LoRaDeviceType") {
                    modType = line;
                    modType.erase(0, modType.find_first_of('\"'));
                    modType.erase(std::remove( modType.begin(), modType.end(), '\"' ), modType.end());
                } else if (a == "MA") {
                    MA = std::stoi(c);
                } else if (a == "ADR") {
                    ADR = std::stoi(c);
                } else if (a == "TXP") {
                    TXP = std::stoi(c);
                } else if (a == "CH") {
                    CH = std::stoi(c);
                }
            }
        }
        return enabled;
    }
    
    int devFile = -1;
    std::string device = "ttyUSB0";
    int baud = 9600;
    bool bridgeToLocal = false;
    std::string modType = "E32-915T30D";

    std::string lastSequence;
    std::string lastMedia;
    bool sendMediaSync = true;
    int lastFrame = -1;
    
    float lastSentTime = -1.0f;
    int lastSentFrame = -1;
    
    char readBuffer[256];
    int curPosition = 0;

    int CH = 915;
    int ADR = 2500;
    int MA = 0;
    int TXP = 4;
};


class LoRaFPPPlugin : public  FPPPlugins::Plugin, public FPPPlugins::APIProviderPlugin {
public:
    LoRaMultiSyncPlugin *plugin = new LoRaMultiSyncPlugin();
    bool enabled = false;
    
    // Name matches the directory the plugin installs into. It used to be
    // "LoRa", which is what everything inside fppd keyed on while the Plugin
    // Manager, the install/uninstall scripts and the load/unload endpoints all
    // use the directory name - so this plugin could not be found to be unloaded.
    // It also lines the inherited settings map up with the file loadSettings()
    // actually reads, config/plugin.fpp-LoRa.
    LoRaFPPPlugin() : FPPPlugins::Plugin("fpp-LoRa", true), FPPPlugins::APIProviderPlugin() {
        enabled = plugin->loadSettings();
    }
    virtual ~LoRaFPPPlugin() {
        delete plugin;
        plugin = nullptr;
    }
    
    // Called by FPP when config/plugin.fpp-LoRa changes.
    //
    // Only the two flags below can be applied where they stand. The rest -
    // enabling the plugin, and the serial port and its speed - decide which
    // device is open and whether its descriptor is in the epoll loop, and that
    // descriptor was handed to FPP through addControlCallbacks(); swapping it
    // underneath FPP is not something this plugin can do safely, so those still
    // want a restart and their settings still say so.
    //
    // The radio parameters (module type, air rate, power, channel, FEC, UART
    // rate) are not here on purpose: they are programmed into the module by the
    // UI posting to the /LoRa route, which is why they never needed a restart.
    virtual void settingChanged(const std::string &key, const std::string &value) override {
        if (key == "LoRaMediaEnable") {
            plugin->sendMediaSync = (value == "1");
            LogInfo(VB_PLUGIN, "LoRa: media sync %s\n", plugin->sendMediaSync ? "enabled" : "disabled");
        } else if (key == "LoRaBridgeEnable") {
            plugin->bridgeToLocal = (value == "1");
            LogInfo(VB_PLUGIN, "LoRa: local bridging %s\n", plugin->bridgeToLocal ? "enabled" : "disabled");
            if (plugin->bridgeToLocal) {
                // Needed to forward what arrives over the radio; opening them
                // again when they already are is harmless.
                multiSync->OpenControlSockets();
            }
        }
    }

    virtual void registerApis() override {
        //at this point, most of FPP is up and running, we can register our MultiSync plugin
        if (enabled && plugin->Init()) {
            if (getFPPmode() == PLAYER_MODE) {
                //only register the sender for master mode
                multiSync->addMultiSyncPlugin(plugin);
            }
        } else {
            enabled = false;
        }
        // Registered through FPP rather than drogon::app() directly: drogon has
        // no route removal, so a handler registered straight with it could never
        // be withdrawn - it would pin this plugin in memory for the life of
        // fppd, and a rebuilt copy could never take the path back over.
        // Capturing the plugin pointer is safe because unregisterPluginApi()
        // does not return until no request is inside the handler AND the
        // handler has been destroyed.
        LoRaMultiSyncPlugin *p = plugin;
        FPPPlugins::registerPluginApi("/LoRa",
            [p](const HttpRequestPtr &req, HttpCallback &&cb) {
                p->handleConfigRequest(req, std::move(cb));
            },
            { drogon::Post });
    }
    virtual void unregisterApis() override {
        // Disarms the route and destroys the handler before returning; after
        // this the path answers 410 rather than dispatching into this library.
        FPPPlugins::unregisterPluginApi("/LoRa");
    }

    // Teardown belongs here rather than in unregisterApis(): FPP calls this once
    // the route is disarmed and while the plugin is still whole. The MultiSync
    // registration is the one that matters - its callbacks run from the
    // sequence/media path, so leaving it in place while the object is destroyed
    // is a call into a half-destroyed plugin. Nothing is asynchronous, so no
    // readiness predicate is needed; the serial port is closed by
    // ~LoRaMultiSyncPlugin() in the destructor below.
    virtual std::function<bool()> shutdown() override {
        if (enabled) {
            plugin->ShutdownSync();
            multiSync->removeMultiSyncPlugin(plugin);
        }
        return nullptr;
    }

    virtual void addControlCallbacks(std::map<int, std::function<bool(int)>> &callbacks) override {
        if (enabled && getFPPmode() == REMOTE_MODE) {
            plugin->addControlCallbacks(callbacks);
            if (plugin->bridgeToLocal) {
                //if we're bridging the multisync, we need to have the control sockets open
                multiSync->OpenControlSockets();
            }
        }
    }
};


// Safe to dlclose() on unload: no threads of its own, no timers, no CurlManager
// requests, no commands and no drogon client objects. The route goes through
// registerPluginApi() and comes back in unregisterApis(); shutdown() withdraws
// the MultiSync registration; FPP takes the serial descriptor out of its epoll
// loop, and ~LoRaMultiSyncPlugin() closes it before the library is unmapped.
FPP_PLUGIN_SUPPORTS_UNLOAD()

extern "C" {
    FPPPlugins::Plugin *createPlugin() {
        return new LoRaFPPPlugin();
    }
}
