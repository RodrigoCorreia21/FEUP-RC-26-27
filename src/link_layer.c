// RCOM 2026/2027
//
// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <stdlib.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

// Debug flag: set to 1 to enable verbose frame dumps, 0 to disable
#define DEBUG_FRAMES 1

// Supervision frame fields
#define FLAG       0x7E
#define A_SENDER   0x03  // frames sent by Sender or answers from Receiver
#define A_RECEIVER 0x01  // frames sent by Receiver or answers from Sender
#define C_SET      0x03
#define C_UA       0x07

#define ESC          0x7D
#define ESC_XOR      0x20
#define C_N0         0x00 // Trama I0
#define C_N1         0x80 // Trama I1
#define C_RR0        0xAA // Receiver Ready
#define C_RR1        0xAB // Receiver Ready 
#define C_REJ0       0x54 // Reject pacote 0
#define C_REJ1       0x55 // Reject pacote 1
#define C_DISC       0x0B // Disconnect

static int txSequenceNumber = 0; // N(s)
static int rxSequenceNumber = 0; // N(r)

static int timeoutValue = 0;
static int maxRetransmissions = 0;

// Helper macros for hexadecimal debug printing.
#if DEBUG_FRAMES
#define PRINT_HEX(label, var) \
    printf("%s = 0x%02X\n", label, (unsigned int)((var) & 0xFF))
#else
#define PRINT_HEX(label, var) ((void)0)
#endif

// ---------------------------------------------------------------------------
// Alarm handling for retransmission timeout (Lab 2)
// ---------------------------------------------------------------------------
static volatile sig_atomic_t alarmCount = 0;
static volatile sig_atomic_t timeoutOccurred = FALSE;

void alarmHandler(int signal)
{
    alarmCount++;
    timeoutOccurred = TRUE;
}

static int installAlarmHandler(void)
{
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        return -1;
    }
    return 0;
}

// Helper: write exactly nBytes to the serial port
static int writeAll(const unsigned char *buf, int nBytes)
{
    int written = 0;

    while (written < nBytes)
    {
        int res = writeBytesSerialPort(buf + written, nBytes - written);
        if (res < 0)
            return -1;
        if (res == 0)
            continue; // nothing written, try again
        written += res;
    }

    return written;
}

// ---------------------------------------------------------------------------
// Wait for a supervision frame of the form
//   FLAG | A | C | BCC | FLAG
// Returns 1 if a valid frame was received, 0 on timeout, -1 on error.
// If timeoutSec > 0, an alarm is set and the function returns 0 on timeout.
// If timeoutSec == 0, the function blocks until a frame or error.
// ---------------------------------------------------------------------------
static int waitForFrame(unsigned char a, unsigned char c, unsigned char bcc, int timeoutSec)
{
    typedef enum
    {
        STATE_START,
        STATE_FLAG_RCV,
        STATE_A_RCV,
        STATE_C_RCV,
        STATE_BCC_OK
    } State;

    State state = STATE_START;
    int useAlarm = (timeoutSec > 0);

    if (useAlarm)
    {
        timeoutOccurred = FALSE;
        alarm(timeoutSec);
    }

    while (1)
    {
        if (useAlarm && timeoutOccurred)
        {
            alarm(0);
            printf("Alarm #%d received\n", alarmCount);
            return 0; // timeout
        }

        unsigned char byte;
        int res = readByteSerialPort(&byte);

        if (res < 0)
        {
            if (errno == EINTR)
            {
                if (useAlarm && timeoutOccurred)
                {
                    alarm(0);
                    printf("Alarm #%d received\n", alarmCount);
                    return 0; // timeout
                }
                continue; // interrupted by other signal, retry
            }
            if (useAlarm)
                alarm(0);
            return -1;
        }

        if (res == 0)
            continue; // no byte (should not happen with VMIN=1)

        // State machine
        switch (state)
        {
        case STATE_START:
            if (byte == FLAG) state = STATE_FLAG_RCV;
            break;

        case STATE_FLAG_RCV:
            if (byte == a) state = STATE_A_RCV;
            else if (byte == FLAG) state = STATE_FLAG_RCV;
            else state = STATE_START;
            break;

        case STATE_A_RCV:
            if (byte == c) state = STATE_C_RCV;
            else if (byte == FLAG) state = STATE_FLAG_RCV;
            else state = STATE_START;
            break;

        case STATE_C_RCV:
            if (byte == bcc) state = STATE_BCC_OK;
            else if (byte == FLAG) state = STATE_FLAG_RCV;
            else state = STATE_START;
            break;

        case STATE_BCC_OK:
            if (byte == FLAG) return 1; // STOP
            state = STATE_START;
            break;
        }
    }
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
    if (installAlarmHandler() < 0)
        return -1;

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    timeoutValue = llParameters.timeout;
    maxRetransmissions = llParameters.nRetransmissions;

    printf("Serial port %s opened (Tx)\n", llParameters.serialPort);

    // SET frame: FLAG | A=0x03 | C=0x03 | BCC=0x03^0x03 | FLAG
    unsigned char setFrame[5] = {
        FLAG,
        A_SENDER,
        C_SET,
        A_SENDER ^ C_SET, // BCC = 0x00
        FLAG
    };

    // Expected UA frame from receiver: FLAG | A=0x03 | C=0x07 | BCC=0x03^0x07 | FLAG
    unsigned char expectedA   = A_SENDER; // answer from Receiver uses 0x03
    unsigned char expectedC   = C_UA;
    unsigned char expectedBcc = expectedA ^ expectedC; // 0x04

    for (int attempt = 0; attempt <= llParameters.nRetransmissions; attempt++)
    {

        if (writeAll(setFrame, 5) < 0)
        {
            closeSerialPort();
            return -1;
        }

        int res = waitForFrame(expectedA, expectedC, expectedBcc, llParameters.timeout);

        if (res == 1)
        {
            printf("Connection established (Tx received UA)\n");
            return 0;
        }

        if (res < 0)
        {
            closeSerialPort();
            return -1;
        }
    }

    closeSerialPort();
    return -1;
}

int llOpenRx(LinkLayer llParameters)
{
    if (installAlarmHandler() < 0)
        return -1;

    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    timeoutValue = llParameters.timeout;
    maxRetransmissions = llParameters.nRetransmissions;

    printf("Serial port %s opened (Rx)\n", llParameters.serialPort);

    // UA frame to send after correct SET: FLAG | A=0x03 | C=0x07 | BCC=0x03^0x07 | FLAG
    unsigned char uaFrame[5] = {
        FLAG,
        A_SENDER,
        C_UA,
        A_SENDER ^ C_UA, // BCC = 0x04
        FLAG
    };

    // Expected SET frame from transmitter: FLAG | A=0x03 | C=0x03 | BCC=0x03^0x03 | FLAG
    unsigned char expectedA   = A_SENDER;
    unsigned char expectedC   = C_SET;
    unsigned char expectedBcc = expectedA ^ expectedC; // 0x00

    while (1)
    {
        // For the receiver we wait indefinitely for a SET frame (timeout = 0).
        int res = waitForFrame(expectedA, expectedC, expectedBcc, 0);

        if (res < 0)
        {
            closeSerialPort();
            return -1;
        }

        if (res == 1)
        {
            if (writeAll(uaFrame, 5) < 0)
            {
                closeSerialPort();
                return -1;
            }

            printf("Connection established (Rx sent UA)\n");
            return 0;
        }

        // res == 0 cannot happen because timeout is 0
    }
}

static int sendSupervisionFrame(unsigned char a, unsigned char c) {
    unsigned char frame[5] = {FLAG, a, c, a ^ c, FLAG};
    return writeAll(frame, 5);
}

static unsigned char readControlResponse(unsigned char expectedA, int timeoutSec) {
    int state = 0;
    unsigned char cField = 0;
    
    timeoutOccurred = FALSE;
    alarm(timeoutSec);

    while (1) {
        if (timeoutOccurred) {
            alarm(0);
            return 0; // Timeout
        }

        unsigned char byte;
        int res = readByteSerialPort(&byte);
        
        if (res < 0) {
            if (errno == EINTR) {
                if (timeoutOccurred) { alarm(0); return 0; }
                continue;
            }
            alarm(0); return -1;
        }
        if (res == 0) continue;

        switch (state) {
            case 0: if (byte == FLAG) state = 1; break;
            case 1: if (byte == expectedA) state = 2; else if (byte != FLAG) state = 0; break;
            case 2:
                if (byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1 || byte == C_DISC) {
                    cField = byte; state = 3;
                } else if (byte == FLAG) state = 1; else state = 0; break;
            case 3:
                if (byte == (expectedA ^ cField)) state = 4;
                else if (byte == FLAG) state = 1; else state = 0; break;
            case 4:
                if (byte == FLAG) { alarm(0); return cField; }
                else state = 0; break;
        }
    }
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize)
{
    unsigned char bcc2 = buf[0];
    for (int i = 1; i < bufSize; i++) {
        bcc2 ^= buf[i];
    }

    int maxFrameSize = (bufSize * 2) + 6; 
    unsigned char *frame = (unsigned char *) malloc(maxFrameSize);
    
    frame[0] = FLAG;
    frame[1] = A_SENDER;
    frame[2] = (txSequenceNumber == 0) ? C_N0 : C_N1;
    frame[3] = frame[1] ^ frame[2]; 

    int j = 4;
    for (int i = 0; i < bufSize; i++) {
        if (buf[i] == FLAG || buf[i] == ESC) {
            frame[j++] = ESC;
            frame[j++] = buf[i] ^ ESC_XOR;
        } else {
            frame[j++] = buf[i];
        }
    }

    if (bcc2 == FLAG || bcc2 == ESC) {
        frame[j++] = ESC;
        frame[j++] = bcc2 ^ ESC_XOR;
    } else {
        frame[j++] = bcc2;
    }

    frame[j++] = FLAG; 
    int frameSize = j;

    unsigned char expectedRR = (txSequenceNumber == 0) ? C_RR1 : C_RR0;
    unsigned char expectedREJ = (txSequenceNumber == 0) ? C_REJ0 : C_REJ1;
    
    int attempt = 0;
    while (attempt <= maxRetransmissions) {
        
        writeAll(frame, frameSize); 
        
        unsigned char response = readControlResponse(A_SENDER, timeoutValue);

        if (response == expectedRR) {
            txSequenceNumber = (txSequenceNumber + 1) % 2; 
            free(frame);
            return bufSize; 
        } 
        else if (response == expectedREJ) {
            continue; 
        }
        
        attempt++; // Timeout
    }

    free(frame);
    return -1;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    int state = 0; 
    unsigned char cField = 0;
    int packetIndex = 0;

    while (1) {
        unsigned char byte;
        int res = readByteSerialPort(&byte);
        if (res <= 0) continue;

        switch (state) {
            case 0:
                if (byte == FLAG) state = 1;
                break;
            case 1:
                if (byte == A_SENDER) state = 2; 
                else if (byte != FLAG) state = 0;
                break;
            case 2:
                if (byte == C_N0 || byte == C_N1) { 
                    cField = byte;
                    state = 3;
                }
                else if (byte == FLAG) state = 1;
                else state = 0;
                break;
            case 3:
                if (byte == (A_SENDER ^ cField)) state = 4; 
                else if (byte == FLAG) state = 1;
                else state = 0; 
                break;
            case 4:
                if (byte == FLAG) state = 1; 
                else if (byte == ESC) state = 6;
                else {
                    packet[packetIndex++] = byte;
                    state = 5;
                }
                break;
            case 5: 
                if (byte == ESC) {
                    state = 6; 
                }
                else if (byte == FLAG) { 
                    if (packetIndex < 1) { state = 0; packetIndex = 0; break; }
                    
                    int dataSize = packetIndex - 1;
                    unsigned char receivedBcc2 = packet[dataSize]; 
                    
                    unsigned char calcBcc2 = packet[0];
                    for (int i = 1; i < dataSize; i++) calcBcc2 ^= packet[i];

                    int isDuplicate = ((cField == C_N0 && rxSequenceNumber == 1) || 
                                       (cField == C_N1 && rxSequenceNumber == 0));

                    if (receivedBcc2 != calcBcc2) {
                        unsigned char rejC = (rxSequenceNumber == 0) ? C_REJ0 : C_REJ1; 
                        sendSupervisionFrame(A_SENDER, rejC);
                        return -1; 
                    } 
                    else {
                        unsigned char rrC = (rxSequenceNumber == 0) ? C_RR1 : C_RR0; 
                        sendSupervisionFrame(A_SENDER, rrC); 
                        
                        if (!isDuplicate) {
                            rxSequenceNumber = (rxSequenceNumber + 1) % 2; 
                            return dataSize; 
                        } else {
                            state = 0;
                            packetIndex = 0;
                        }
                    }
                }
                else {
                    packet[packetIndex++] = byte; 
                }
                break;
            case 6: 
                state = 5;
                if (byte == (FLAG ^ ESC_XOR) || byte == (ESC ^ ESC_XOR)) {
                    packet[packetIndex++] = byte ^ ESC_XOR; 
                } else {
                    state = 0; 
                    packetIndex = 0;
                }
                break;
        }
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx()
{
    // TODO: Implement this function
    // Remember to call closeSerialPort() here when the connection ends.

    return 0;
}

int llCloseRx()
{
    // TODO: Implement this function
    // Remember to call closeSerialPort() here when the connection ends.

    return 0;
}