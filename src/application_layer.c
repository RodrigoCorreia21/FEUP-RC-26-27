// RCOM 2026/2027
//
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // ----------------------------------------------------
    // TODO: Adapt and extend this code according to the specifications of the project.
    // ----------------------------------------------------

    LinkLayer llParameters = {
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout,
    };
    strcpy(llParameters.serialPort, serialPort);

    if (strcmp(role, "tx") == 0)
    {
        printf("[Tx] Initiating connection establishment...\n");
        if (llOpenTx(llParameters) < 0) {
            printf("[ERROR]: llOpenTx failed.\n");
            return;
        }

        FILE *file = fopen(filename, "rb");
        if (file == NULL) {
            printf("[ERROR]: Could not open file %s\n", filename);
            llCloseTx();
            return;
        }

        fseek(file, 0, SEEK_END);
        long int fileSize = ftell(file);
        fseek(file, 0, SEEK_SET);

        unsigned char startPacket[MAX_PAYLOAD_SIZE];
        startPacket[0] = 1; // C = 1 (START)
        
        //(T = 0)
        startPacket[1] = 0; 
        startPacket[2] = sizeof(long int); // L
        memcpy(&startPacket[3], &fileSize, sizeof(long int)); // V
        int pos = 3 + sizeof(long int);
        
        //(T = 1)
        startPacket[pos++] = 1; 
        int nameLen = strlen(filename);
        startPacket[pos++] = nameLen; // L
        memcpy(&startPacket[pos], filename, nameLen); // V
        pos += nameLen;

        if (llSend(startPacket, pos) < 0) {
            printf("[ERROR]: Failed to send START packet.\n");
            fclose(file);
            llCloseTx();
            return;
        }

        unsigned char dataBuf[MAX_PAYLOAD_SIZE - 4];
        unsigned char dataPacket[MAX_PAYLOAD_SIZE];
        int bytesRead;
        long int currentBytes = 0; // For the progress bar

        printf("\n[Tx] Starting data transfer...\n");

        while ((bytesRead = fread(dataBuf, 1, sizeof(dataBuf), file)) > 0)
        {
            dataPacket[0] = 2; // C = 2 (DATA)
            dataPacket[1] = bytesRead / 256; // L2
            dataPacket[2] = bytesRead % 256; // L1
            memcpy(&dataPacket[3], dataBuf, bytesRead); 

            if (llSend(dataPacket, bytesRead + 3) < 0) {
                fclose(file);
                llCloseTx();
                return;
            }
            currentBytes += bytesRead;

            // PROGRESS BAR (TX)
            int progress = (int)((currentBytes * 100.0) / fileSize);
            printf("\r[Tx] Progress: [%3d%%] (%ld/%ld bytes)", progress, currentBytes, fileSize);
            fflush(stdout); // Forces the terminal to update the current line
        }

        printf("\n[Tx] File transmitted at 100%%!\n");

        unsigned char endPacket[MAX_PAYLOAD_SIZE];
        endPacket[0] = 3; 
        memcpy(&endPacket[1], &startPacket[1], pos - 1); 
        
        llSend(endPacket, pos);

        fclose(file);

        printf("\n[Tx] Closing connection...\n");
        llCloseTx();
        printf("[SUCCESS] Tx operation completed successfully!\n");
    }
    else if (strcmp(role, "rx") == 0)
    {
        printf("[Rx] Listening for connection requests...\n");
        if (llOpenRx(llParameters) < 0) {
            printf("[ERROR]: llOpenRx failed.\n");
            return;
        }

        unsigned char packet[MAX_PAYLOAD_SIZE + 5]; 
        FILE *file = NULL;
        long int fileSize = 0;
        long int currentBytes = 0;

        while (1)
        {
            int bytesRead = llReceive(packet);
            
            if (bytesRead < 0) continue; 
            if (bytesRead == 0) continue; 

            unsigned char c = packet[0];

            if (c == 1) { 
                file = fopen(filename, "wb");
                if (file == NULL) {
                    printf("[ERROR] Could not create file %s\n", filename);
                    break;
                }
                // Extract file size (assuming the first parameter T=0 is the size)
                if (packet[1] == 0 && packet[2] == sizeof(long int)) {
                    memcpy(&fileSize, &packet[3], sizeof(long int));
                }
                printf("[Rx] START packet received. Expected size: %ld bytes.\n", fileSize);
                printf("[Rx] Starting data reception...\n");
            }
            else if (c == 2) { 
                if (file == NULL) continue;
                
                int k = 256 * packet[1] + packet[2];
                fwrite(&packet[3], 1, k, file);

                currentBytes += k;

                // PROGRESS BAR (RX)
                if (fileSize > 0) {
                    int progress = (int)((currentBytes * 100.0) / fileSize);
                    printf("\r[Rx] Progress: [%3d%%] (%ld/%ld bytes)", progress, currentBytes, fileSize);
                    fflush(stdout);
                }
            }
            else if (c == 3) { 
                printf("\n[Rx] END packet received. Reception at 100%%!\n");
                break; 
            }
        }

        if (file != NULL) fclose(file);

        printf("\n[Rx] Closing connection...\n");
        llCloseRx();
        printf("[SUCCESS] Rx operation completed successfully!\n");
    }
    else
    {
        printf("Invalid role: %s. Must be 'tx' or 'rx'.\n", role);
    }
}
