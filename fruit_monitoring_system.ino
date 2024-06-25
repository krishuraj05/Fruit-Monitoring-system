#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include <SD.h>
#include "memorysaver.h"

const int CS = 7;
#define SD_CS 9
#define FRAMES_NUM 0x06

bool is_header = false;
int total_time = 0;
ArduCAM myCAM(OV2640, CS);

const int analogsensor = A0; // Analog pin connected to AO
const int methaneThreshold = 32; // Methane concentration threshold
const unsigned long captureDuration = 2000; // Capture duration in milliseconds

void setup() {
    pinMode(analogsensor, INPUT); // Set AO pin as input
    Serial.begin(115200); // Initialize serial communication

    uint8_t vid, pid;
    uint8_t temp;

    #if defined(__SAM3X8E__)
    Wire1.begin();
    #else
    Wire.begin();
    #endif

    Serial.println(F("ArduCAM Start!"));
    pinMode(CS, OUTPUT);
    digitalWrite(CS, HIGH);
    SPI.begin();
    
    myCAM.write_reg(0x07, 0x80);
    delay(100);
    myCAM.write_reg(0x07, 0x00);
    delay(100);

    while (1) {
        myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
        temp = myCAM.read_reg(ARDUCHIP_TEST1);
        if (temp != 0x55) {
            Serial.println(F("SPI interface Error!"));
            delay(1000);
            continue;
        } else {
            Serial.println(F("SPI interface OK."));
            break;
        }
    }

    while (1) {
        myCAM.wrSensorReg8_8(0xff, 0x01);
        myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
        myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
        if ((vid != 0x26) && ((pid != 0x41) || (pid != 0x42))) {
            Serial.println(F("ACK CMD Can't find OV2640 module!"));
            delay(1000);
            continue;
        } else {
            Serial.println(F("ACK CMD OV2640 detected."));
            break;
        }
    }

    while (!SD.begin(SD_CS)) {
        Serial.println(F("SD Card Error!"));
        delay(1000);
    }

    Serial.println(F("SD Card detected."));
    myCAM.set_format(JPEG);
    myCAM.InitCAM();
    myCAM.clear_fifo_flag();
    myCAM.write_reg(ARDUCHIP_FRAMES, FRAMES_NUM);
}

void loop() {
    int methaneConcentration = analogRead(analogsensor);
    Serial.print("Methane Concentration: ");
    Serial.println(methaneConcentration);

    if (methaneConcentration > methaneThreshold) {
        Serial.println("Methane concentration exceeds threshold. Capturing image...");
        captureForDuration();
    }
    delay(1000); // Delay for stability and to control the loop rate
}

void captureForDuration() {
    unsigned long startTime = millis();
    while (millis() - startTime < captureDuration) {
        myCAM.flush_fifo();
        myCAM.clear_fifo_flag();
        myCAM.OV2640_set_JPEG_size(OV2640_1600x1200);
        myCAM.start_capture();
        Serial.println(F("Start capture."));
        total_time = millis();
        while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
        Serial.println(F("CAM Capture Done."));
        total_time = millis() - total_time;
        Serial.print(F("Capture total time used (in milliseconds):"));
        Serial.println(total_time, DEC);
        total_time = millis();
        read_fifo_burst(myCAM);
        total_time = millis() - total_time;
        Serial.print(F("Save capture total time used (in milliseconds):"));
        Serial.println(total_time, DEC);
        myCAM.clear_fifo_flag();
    }
    Serial.println("Image capture complete. Resuming methane concentration monitoring.");
}

uint8_t read_fifo_burst(ArduCAM myCAM) {
    uint8_t temp = 0, temp_last = 0;
    uint32_t length = 0;
    static int i = 0;
    static int k = 0;
    char str[16];
    File outFile;
    byte buf[256];
    length = myCAM.read_fifo_length();
    Serial.print(F("The fifo length is :"));
    Serial.println(length, DEC);
    if (length >= MAX_FIFO_SIZE) { //8M
        Serial.println("Over size.");
        return 0;
    }
    if (length == 0 ) { //0 kb
        Serial.println(F("Size is 0."));
        return 0;
    }
    myCAM.CS_LOW();
    myCAM.set_fifo_burst(); // Set FIFO burst mode
    i = 0;
    while ( length-- ) {
        temp_last = temp;
        temp =  SPI.transfer(0x00);
        // Read JPEG data from FIFO
        if ((temp == 0xD9) && (temp_last == 0xFF)) { // If find the end, break while loop
            buf[i++] = temp;  // Save the last 0xD9
            // Write the remaining bytes in the buffer
            myCAM.CS_HIGH();
            outFile.write(buf, i);
            outFile.close();
            Serial.println(F("OK"));
            is_header = false;
            myCAM.CS_LOW();
            myCAM.set_fifo_burst();
            i = 0;
        }
        if (is_header == true) {
            // Write image data to buffer if not full
            if (i < 256)
                buf[i++] = temp;
            else {
                // Write 256 bytes image data to file
                myCAM.CS_HIGH();
                outFile.write(buf, 256);
                i = 0;
                buf[i++] = temp;
                myCAM.CS_LOW();
                myCAM.set_fifo_burst();
            }
        }
        else if ((temp == 0xD8) && (temp_last == 0xFF)) {
            is_header = true;
            myCAM.CS_HIGH();
            // Create a jpg file
            k = k + 1;
            itoa(k, str, 10);
            strcat(str, ".jpg");
            // Open the new file
            outFile = SD.open(str, O_WRITE | O_CREAT | O_TRUNC);
            if (! outFile) {
                Serial.println(F("File open failed"));
                while (1);
            }
            myCAM.CS_LOW();
            myCAM.set_fifo_burst();
            buf[i++] = temp_last;
            buf[i++] = temp;
        }
    }
    myCAM.CS_HIGH();
    return 1;
}

