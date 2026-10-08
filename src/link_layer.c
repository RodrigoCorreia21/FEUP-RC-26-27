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

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
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