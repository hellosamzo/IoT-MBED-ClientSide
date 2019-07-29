//Minimal imports for assessment 2
#include "mbed.h"
#include "LM75B.h"

//Recommended libraries to simplify the UDP connection
#include "easy-connect.h"

//Additional libraries
#include "C12832.h"

/*
 * Author: Sam M (hellosamzo)
 *
 * Functionality: A packet is sent every 10 seconds to the UDP server
 * and every 60 seconds an acknowledgement packet is sent. If no 
 * acknowledgement packet is sent back from the server, another 
 * acknowledgement is sent with a retry flag 120sec+ later. Every retry packet that is sent
 * increments a retry counter which makes the backoff time longer between each
 * retry to create a backoff with random jitter.
 *
 * Each packet contains the temperature from the temperature sensor on the shield,
 * and contains any button presses which are buffered between packets.
 *
 * Each packet also has its own unique sequence number and is verified using
 * a CRC-8-CCITT checksum.
 
 * For the CRC-8-CCITT checksum I used a public domain implementation
 * from 3dbrew at https://3dbrew.org/wiki/CRC-8-CCITT
 *
 */

// shield temperature sensor and screen
C12832 screen(D11, D13, D12, D7, D10);
LM75B tempSensor(D14, D15);

// buttons
InterruptIn up(A2);
InterruptIn down(A3);
InterruptIn left(A4);
InterruptIn right_btn(A5);
InterruptIn fire(D4);
InterruptIn sw2(SW2);
InterruptIn sw3(SW3);

// networking
UDPSocket sock;
const char *IP = "example ip";
const int PORT = 123;

// ticker for acknowledgements
Ticker ack;
// ticker for general packets to be sent
Ticker pack;
// timeout for checking acks have been receieved
Timeout checker;

// global int flags
volatile int upFlag = 0;
volatile int downFlag = 0;
volatile int leftFlag = 0;
volatile int rightFlag = 0;
volatile int fireFlag = 0;
volatile int sw2Flag = 0;
volatile int sw3Flag = 0;
volatile int ackFlag = 0;
volatile int checkAckFlag = 0;
volatile int packetFlag = 0;

// global receiving ack packet array
char recPacket[3];

// retry counter
int retryCount = 0;

// packet variables
uint8_t seqnumber = 0;
uint8_t packetoption;
uint8_t btnpress;
uint8_t checksum;

// this is where the referenced code from https://3dbrew.org/wiki/CRC-8-CCITT begins
static const uint8_t CRC_TABLE[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3};

uint8_t crc8ccitt(const void *data, size_t size)
{
    uint8_t val = 0;

    uint8_t *pos = (uint8_t *)data;
    uint8_t *end = pos + size;

    while (pos < end)
    {
        val = CRC_TABLE[val ^ *pos];
        pos++;
    }

    return val;
}
// this is where the referenced code from https://3dbrew.org/wiki/CRC-8-CCITT ends

// continuation of my code
// union for my senderid, to split the 16bit int into two 8 bits
union senderunion {
    uint16_t id;
    char bytes[2];
} senderid;

// union for the temperature, to split the 16bit int into two 8 bits
union tempunion {
    uint16_t tmp;
    uint8_t bytes[2];
} temp;

// set a bit in the packet option variable at a certain position
void bitPosPacket(uint8_t *packetoption, int position)
{
    *packetoption |= 1UL << position;
}

// sets a bit for packetoptions or btnpress at a certain position
void bitPosBtn(uint8_t *btnpress, int position)
{
    *btnpress |= 1UL << position;
}

// if sw2 is pressed set the flag to 1
void trigger_sw2()
{
    sw2Flag = 1;
}

// if sw3 is pressed set the flag to 1
void trigger_sw3()
{
    sw3Flag = 1;
}

// if joystick middle btn is pressed set the flag to 1
void trigger_fire()
{
    fireFlag = 1;
}

// if joystick is pointed up set the flag to 1
void trigger_up()
{
    upFlag = 1;
}

// if joystick is pointed down set the flag to 1
void trigger_down()
{
    downFlag = 1;
}

// if joystick is pointed left set the flag to 1
void trigger_left()
{
    leftFlag = 1;
}

// if joystick is pointed right set the flag to 1
void trigger_right()
{
    rightFlag = 1;
}

// if ticker executes, set flag to 1
void setPacketFlag()
{
    packetFlag = 1;
}

// if ticker executes, set flag to 1
void setAckFlag()
{
    ackFlag = 1;
}

// if timer executes, set flag to 1
void setCheckAckFlag()
{
    checkAckFlag = 1;
}

// check to see if an ack has been received from the UDP server
void checkAck()
{
    // check if ack packet has been received and the senderid matches
    if (recPacket[1] == senderid.bytes[0] && recPacket[0] == senderid.bytes[1])
    {
        screen.locate(0, 1);
        screen.printf("got ack");
        // clear receive array in preparation for next ack
        recPacket[0] = 0;
        recPacket[1] = 0;
        recPacket[2] = 0;
        // if last check was a retry then clear retry counter
        // and start sending packets at normal rate again
        if (retryCount > 0)
        {
            retryCount = 0;
            pack.detach();
            ack.detach();
            ack.attach(setAckFlag, 60);
            pack.attach(setPacketFlag, 10);
        }
    }
    // if no ack received or ack meant for different senderid
    else
    {
        retryCount++;
        bitPosPacket(&packetoption, 2);
        screen.cls();
        screen.locate(0, 1);
        screen.printf("%d", retryCount);
        // start sending only acks at a longer time each retry
        // as a way of backing off
        pack.detach();
        ack.detach();
        int jitterTime = (retryCount * 60);
        ack.attach(setAckFlag, (60 + jitterTime));
        pack.attach(setPacketFlag, (60 + jitterTime));
    }
}

// verifying there is a connection by receiving a 3 byte(senderid and seqnumbr) packet from the UDP server
void recvPacket()
{
    while (1)
    {
        sock.recvfrom(NULL, recPacket, sizeof(recPacket));
    }
}

// send an 8 byte long packet which is built up from an array
void sendPacket()
{
    // add sequence number by 1 each time a new packet is about to be sent
    seqnumber++;

    // build temp packet for CCITT
    char packetTemp[] = {senderid.bytes[1], senderid.bytes[0], seqnumber, packetoption, temp.bytes[0], temp.bytes[1], btnpress};
    checksum = crc8ccitt(packetTemp, sizeof(packetTemp));

    // final packet with CCITT included
    char packet[] = {senderid.bytes[1], senderid.bytes[0], seqnumber, packetoption, temp.bytes[0], temp.bytes[1], btnpress, checksum};
    btnpress = 00000000;
    packetoption = 00000000;
    if (0 > sock.sendto(IP, PORT, packet, sizeof(packet)))
    {
        screen.locate(0, 1);
        screen.printf("sending packet failed");
    }
}

int main()
{
    // set CCITT flag to true
    bitPosPacket(&packetoption, 1);
    // set the timeout for a packet to be 3 seconds
    sock.set_timeout(3000);
    // sender id from union (split into two 8 bits)
    senderid.id = 61414;
    // set acks to be sent every 60 sec
    ack.attach(setAckFlag, 60);
    // set packets to be sent every 10 sec
    pack.attach(setPacketFlag, 10);
    sw2.fall(trigger_sw2);
    sw3.fall(trigger_sw3);
    fire.fall(trigger_fire);
    up.fall(trigger_up);
    down.fall(trigger_down);
    left.fall(trigger_left);
    right_btn.fall(trigger_right);

    // connect mbed to the internet via ethernet
    NetworkInterface *network = easy_connect(true);
    if (!network)
    {
        screen.locate(0, 1);
        screen.printf("Network failure");
        return 1;
    }

    //open udp socket
    sock.open(network);

    // thread for receiving packets from the UDP server
    // at any moment (could change often due to backoff)
    Thread receiveThread;
    receiveThread.start(&recvPacket);

    while (1)
    {
        // polling the sensor (temp) for its data
        temp.tmp = tempSensor.read();

        if (ackFlag)
        {
            ackFlag = 0;
            bitPosPacket(&packetoption, 0);
            // check an ack has been received
            checker.attach(setCheckAckFlag, 5);
        }
        if (packetFlag)
        {
            packetFlag = 0;
            bitPosPacket(&packetoption, 1);
            sendPacket();
        }
        if (sw2Flag)
        {
            sw2Flag = 0;
            bitPosBtn(&btnpress, 0);
        }
        if (sw3Flag)
        {
            sw3Flag = 0;
            bitPosBtn(&btnpress, 1);
        }
        if (fireFlag)
        {
            fireFlag = 0;
            bitPosBtn(&btnpress, 6);
        }
        if (upFlag)
        {
            upFlag = 0;
            bitPosBtn(&btnpress, 2);
        }
        if (downFlag)
        {
            downFlag = 0;
            bitPosBtn(&btnpress, 3);
        }
        if (leftFlag)
        {
            leftFlag = 0;
            bitPosBtn(&btnpress, 4);
        }
        if (rightFlag)
        {
            rightFlag = 0;
            bitPosBtn(&btnpress, 5);
        }
        if (checkAckFlag)
        {
            checkAckFlag = 0;
            checkAck();
        }
    }
}
