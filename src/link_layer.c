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
static volatile sig_atomic_t alarmEnabled = FALSE;
static volatile sig_atomic_t alarmCount = 0;
static volatile sig_atomic_t timeoutOccurred = FALSE;

void alarmHandler(int signal)
{
    (void)signal;
    alarmEnabled = FALSE;
    alarmCount++;
    timeoutOccurred = TRUE;
    printf("Alarm #%d received\n", alarmCount);
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

// Helper: dump a 5-byte supervision frame in hexadecimal
static void dumpFrame(const char *tag, const unsigned char *frame)
{
#if DEBUG_FRAMES
    printf("%s: FLAG=0x%02X A=0x%02X C=0x%02X BCC=0x%02X FLAG=0x%02X\n",
           tag,
           (unsigned int)(frame[0] & 0xFF),
           (unsigned int)(frame[1] & 0xFF),
           (unsigned int)(frame[2] & 0xFF),
           (unsigned int)(frame[3] & 0xFF),
           (unsigned int)(frame[4] & 0xFF));
#else
    (void)tag;
    (void)frame;
#endif
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
        STATE_A,
        STATE_C,
        STATE_BCC,
        STATE_FLAG
    } State;

    State state = STATE_START;
    int useAlarm = (timeoutSec > 0);

    if (useAlarm)
    {
        timeoutOccurred = FALSE;
        alarmEnabled = TRUE;
        alarm(timeoutSec);
    }

    while (1)
    {
        if (useAlarm && timeoutOccurred)
        {
            alarm(0);
            alarmEnabled = FALSE;
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
                    alarmEnabled = FALSE;
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

#if DEBUG_FRAMES
        PRINT_HEX("  Rx byte", byte);
#endif

        // State machine
        switch (state)
        {
        case STATE_START:
            if (byte == FLAG)
                state = STATE_A;
            break;

        case STATE_A:
            if (byte == a)
                state = STATE_C;
            else if (byte == FLAG)
                state = STATE_A;
            else
                state = STATE_START;
            break;

        case STATE_C:
            if (byte == c)
                state = STATE_BCC;
            else if (byte == FLAG)
                state = STATE_A;
            else
                state = STATE_START;
            break;

        case STATE_BCC:
            if (byte == bcc)
                state = STATE_FLAG;
            else if (byte == FLAG)
                state = STATE_A;
            else
                state = STATE_START;
            break;

        case STATE_FLAG:
            if (byte == FLAG)
            {
                if (useAlarm)
                {
                    alarm(0);
                    alarmEnabled = FALSE;
                }
                return 1; // complete valid frame
            }
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

#if DEBUG_FRAMES
    printf("--- Building SET frame ---\n");
    PRINT_HEX("FLAG", setFrame[0]);
    PRINT_HEX("A   ", setFrame[1]);
    PRINT_HEX("C   ", setFrame[2]);
    PRINT_HEX("BCC ", setFrame[3]);
    PRINT_HEX("FLAG", setFrame[4]);
#endif
    dumpFrame("SET (Tx -> Rx)", setFrame);

    // Expected UA frame from receiver: FLAG | A=0x03 | C=0x07 | BCC=0x03^0x07 | FLAG
    unsigned char expectedA   = A_SENDER; // answer from Receiver uses 0x03
    unsigned char expectedC   = C_UA;
    unsigned char expectedBcc = expectedA ^ expectedC; // 0x04

#if DEBUG_FRAMES
    printf("--- Expecting UA frame ---\n");
    PRINT_HEX("Expected A  ", expectedA);
    PRINT_HEX("Expected C  ", expectedC);
    PRINT_HEX("Expected BCC", expectedBcc);
#endif

    for (int attempt = 0; attempt <= llParameters.nRetransmissions; attempt++)
    {
#if DEBUG_FRAMES
        printf("Sending SET (attempt %d/%d)\n",
               attempt + 1, llParameters.nRetransmissions + 1);
#endif

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

        // timeout -> retry
#if DEBUG_FRAMES
        printf("Timeout waiting for UA, retrying...\n");
#endif
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

#if DEBUG_FRAMES
    printf("--- Building UA frame ---\n");
    PRINT_HEX("FLAG", uaFrame[0]);
    PRINT_HEX("A   ", uaFrame[1]);
    PRINT_HEX("C   ", uaFrame[2]);
    PRINT_HEX("BCC ", uaFrame[3]);
    PRINT_HEX("FLAG", uaFrame[4]);
#endif
    dumpFrame("UA (Rx -> Tx)", uaFrame);

    // Expected SET frame from transmitter: FLAG | A=0x03 | C=0x03 | BCC=0x03^0x03 | FLAG
    unsigned char expectedA   = A_SENDER;
    unsigned char expectedC   = C_SET;
    unsigned char expectedBcc = expectedA ^ expectedC; // 0x00

#if DEBUG_FRAMES
    printf("--- Expecting SET frame ---\n");
    PRINT_HEX("Expected A  ", expectedA);
    PRINT_HEX("Expected C  ", expectedC);
    PRINT_HEX("Expected BCC", expectedBcc);
#endif

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
            // Correct SET received -> answer with UA
#if DEBUG_FRAMES
            printf("Valid SET received, sending UA\n");
#endif
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