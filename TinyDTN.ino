#include <Arduino.h>
#include <SPI.h>
#include "LoRaSem.h"
#include <tinycbor.h>

#define LORABUFLENGTH 255
#define SERIALTIMEOUT 100
#define FREQUENCY 434E6

uint64_t currentUnixTime = 1700000000; // Default in seconds since Unix epoch (Jan 1, 1970)

const int csPin = 8;
const int resetPin = 4;
const int irqPin = 3;
uint8_t outgoing[LORABUFLENGTH];
uint8_t incoming[LORABUFLENGTH];
uint8_t tempBuffer[LORABUFLENGTH];

struct IPNEndpointId {
    uint64_t nodeId;
    uint64_t serviceId;
};

class DTNBundle {
public:
    DTNBundle(IPNEndpointId source, IPNEndpointId destination, uint64_t unix_timestamp, uint64_t sequence_number, uint64_t lifetime);
    void setPayload(const char* payloadString);
    size_t encodeBundle(uint8_t* outputBuffer, size_t bufferSize);

    IPNEndpointId source;
    IPNEndpointId destination;
    uint64_t creationTimestamp; // DTN time in seconds since DTN epoch (Jan 1, 2000)
    uint64_t sequenceNumber;
    uint64_t lifetime; // Bundle lifetime in milliseconds
    const uint8_t* payload;
    size_t payloadLength;

private:
    void encodePrimaryBlock(CborEncoder* encoder, uint16_t calculatedCRC);
    void encodePayloadBlock(CborEncoder* encoder, uint16_t calculatedCRC);
    uint16_t calculateCRC16(const uint8_t* data, size_t length);
};

DTNBundle::DTNBundle(IPNEndpointId source, IPNEndpointId destination, uint64_t unix_timestamp, uint64_t sequence_number, uint64_t lifetime) 
    : source(source), destination(destination), creationTimestamp(unix_timestamp - 946684800), sequenceNumber(sequence_number), lifetime(lifetime), payload(nullptr), payloadLength(0) {}

void DTNBundle::setPayload(const char* payloadString) {
    this->payload = reinterpret_cast<const uint8_t*>(payloadString);
    this->payloadLength = strlen(payloadString);
}

uint16_t DTNBundle::calculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0x8408;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc ^ 0xFFFF;
}

void DTNBundle::encodePrimaryBlock(CborEncoder* encoder, uint16_t calculatedCRC) {
    CborEncoder primaryBlock;
    cbor_encoder_create_array(encoder, &primaryBlock, 9); // Primary block should have 9 elements when using CRC

    cbor_encode_uint(&primaryBlock, 7); // Bundle protocol version (BPv7)
    cbor_encode_uint(&primaryBlock, 64); // Bundle processing flags (example value)
    cbor_encode_uint(&primaryBlock, 1); // CRC type (CRC16)

    // Encode Destination EID
    CborEncoder destinationEID;
    cbor_encoder_create_array(&primaryBlock, &destinationEID, 2);
    cbor_encode_uint(&destinationEID, 2); // Scheme code for IPN
    CborEncoder destinationServiceNr;
    cbor_encoder_create_array(&destinationEID, &destinationServiceNr, 2);
    cbor_encode_uint(&destinationServiceNr, destination.nodeId);
    cbor_encode_uint(&destinationServiceNr, destination.serviceId);
    cbor_encoder_close_container(&destinationEID, &destinationServiceNr);
    cbor_encoder_close_container(&primaryBlock, &destinationEID);

    // Encode Source EID
    CborEncoder sourceEID;
    cbor_encoder_create_array(&primaryBlock, &sourceEID, 2);
    cbor_encode_uint(&sourceEID, 2); // Scheme code for IPN
    CborEncoder sourceServiceNr;
    cbor_encoder_create_array(&sourceEID, &sourceServiceNr, 2);
    cbor_encode_uint(&sourceServiceNr, source.nodeId);
    cbor_encode_uint(&sourceServiceNr, source.serviceId);
    cbor_encoder_close_container(&sourceEID, &sourceServiceNr);
    cbor_encoder_close_container(&primaryBlock, &sourceEID);

    // Encode Report-to EID
    CborEncoder reportToEID;
    cbor_encoder_create_array(&primaryBlock, &reportToEID, 2);
    cbor_encode_uint(&reportToEID, 2); // Scheme code for IPN
    CborEncoder reportToServiceNr;
    cbor_encoder_create_array(&reportToEID, &reportToServiceNr, 2);
    cbor_encode_uint(&reportToServiceNr, destination.nodeId); // Using destination nodeId as an example
    cbor_encode_uint(&reportToServiceNr, destination.serviceId); // Using destination serviceId as an example
    cbor_encoder_close_container(&reportToEID, &reportToServiceNr);
    cbor_encoder_close_container(&primaryBlock, &reportToEID);

    // Creation timestamp and sequence number
    CborEncoder creationTimestampArray;
    cbor_encoder_create_array(&primaryBlock, &creationTimestampArray, 2);
    cbor_encode_uint(&creationTimestampArray, creationTimestamp); // DTN time in seconds since DTN epoch (Jan 1, 2000)
    cbor_encode_uint(&creationTimestampArray, sequenceNumber);
    cbor_encoder_close_container(&primaryBlock, &creationTimestampArray);

    // Lifetime
    cbor_encode_uint(&primaryBlock, lifetime); // Lifetime in milliseconds

    // Write CRC value to the output
    uint8_t crcBytes[2];
    crcBytes[0] = static_cast<uint8_t>(calculatedCRC >> 8); // MSB first
    crcBytes[1] = static_cast<uint8_t>(calculatedCRC); // LSB last
    cbor_encode_byte_string(&primaryBlock, crcBytes, 2);

    // Close the primary block container
    cbor_encoder_close_container(encoder, &primaryBlock);
}

void DTNBundle::encodePayloadBlock(CborEncoder* encoder, uint16_t calculatedCRC) {
    CborEncoder payloadBlock;
    cbor_encoder_create_array(encoder, &payloadBlock, 5); // Payload block with 5 elements

    cbor_encode_uint(&payloadBlock, 1); // Block type code: Bundle Payload Block
    cbor_encode_uint(&payloadBlock, 1); // Block number
    cbor_encode_uint(&payloadBlock, 1); // Block processing control flags
    cbor_encode_uint(&payloadBlock, 1); // CRC type (CRC16)
    cbor_encode_byte_string(&payloadBlock, payload, payloadLength); // Block-specific data (payload)

    // Write CRC value to the output
    uint8_t crcBytes[2];
    crcBytes[0] = static_cast<uint8_t>(calculatedCRC >> 8); // MSB first
    crcBytes[1] = static_cast<uint8_t>(calculatedCRC); // LSB last
    cbor_encode_byte_string(&payloadBlock, crcBytes, 2);

    // Close the payload block container
    cbor_encoder_close_container(encoder, &payloadBlock);
}

size_t DTNBundle::encodeBundle(uint8_t* outputBuffer, size_t bufferSize) {
    Serial.println("Encoding DTN Bundle...");
    CborEncoder encoder;

    // Step 1: Encode the Primary Block with zero CRC to calculate CRC
    cbor_encoder_init(&encoder, tempBuffer, sizeof(tempBuffer), 0);
    encodePrimaryBlock(&encoder, 0);
    size_t primaryBlockSize = cbor_encoder_get_buffer_size(&encoder, tempBuffer);

    // Set the last two bytes in the buffer to zero before CRC calculation
    if (primaryBlockSize >= 2) {
        tempBuffer[primaryBlockSize - 2] = 0; // CRC byte 1
        tempBuffer[primaryBlockSize - 1] = 0; // CRC byte 2
    }

    uint16_t primaryCRC = calculateCRC16(tempBuffer, primaryBlockSize);

    // Step 2: Encode the Payload Block with zero CRC to calculate CRC
    cbor_encoder_init(&encoder, tempBuffer, sizeof(tempBuffer), 0);
    encodePayloadBlock(&encoder, 0);
    size_t payloadBlockSize = cbor_encoder_get_buffer_size(&encoder, tempBuffer);

    // Set the last two bytes in the buffer to zero before CRC calculation
    if (payloadBlockSize >= 2) {
        tempBuffer[payloadBlockSize - 2] = 0; // CRC byte 1
        tempBuffer[payloadBlockSize - 1] = 0; // CRC byte 2
    }

    uint16_t payloadCRC = calculateCRC16(tempBuffer, payloadBlockSize);

    // Step 3: Encode the entire bundle with calculated CRCs
    cbor_encoder_init(&encoder, outputBuffer, bufferSize, 0);
    CborEncoder bundle;
    cbor_encoder_create_array(&encoder, &bundle, CborIndefiniteLength); // Bundle with primary and payload blocks

    // Encode Primary Block with actual CRC
    encodePrimaryBlock(&bundle, primaryCRC);

    // Encode Payload Block with actual CRC
    encodePayloadBlock(&bundle, payloadCRC);

    // Close the bundle container
    cbor_encoder_close_container(&encoder, &bundle);

    size_t encodedLength = cbor_encoder_get_buffer_size(&encoder, outputBuffer);
    Serial.print("Encoded Bundle Length: ");
    Serial.println(encodedLength);
    Serial.print("Encoded Bundle (Hex): ");
    for (size_t i = 0; i < encodedLength; i++) {
        if (outputBuffer[i] < 16) Serial.print("0");
        Serial.print(outputBuffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    return encodedLength;
}

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(SERIALTIMEOUT);
    while (!Serial);
    Serial.println("LoRa ION Demo Arduino Sketch");

    LoRa.setPins(csPin, resetPin, irqPin);
    if (!LoRa.begin(FREQUENCY)) {
        Serial.println("LoRa init failed. Check your pin settings.");
        while (true);
    }
    Serial.println("LoRa init succeeded.");

    LoRa.setSpreadingFactor(9);
    LoRa.setSignalBandwidth(250E3);
    LoRa.setCodingRate4(5);
    LoRa.setTxPower(20);
    LoRa.enableCrc();
    LoRa.receive();

    // Example usage of DTNBundle
    IPNEndpointId source = {268484888, 2};
    IPNEndpointId destination = {268484612, 2};
    uint64_t lifetime = 3600000; // 1 hour in milliseconds
    DTNBundle bundle(source, destination, currentUnixTime, 1, lifetime);
    const char* payloadString = "Hello DTN world!";
    bundle.setPayload(payloadString);

    uint8_t encodedBundle[LORABUFLENGTH];
    size_t bundleLength = bundle.encodeBundle(encodedBundle, sizeof(encodedBundle));

    // Send bundle to LoRa
    LoRa.beginPacket();
    LoRa.write(encodedBundle, bundleLength);
    LoRa.endPacket();
    Serial.print("Bundle sent over LoRa. Length: ");
    Serial.println(bundleLength);
}

void loop() {
    handleSerialInput();
    handleLoRaReception();
}

void handleSerialInput() {
    if (Serial.available()) {
        int readBytes = Serial.readBytes((char*)incoming, LORABUFLENGTH);
        LoRa.beginPacket();
        LoRa.write(incoming, readBytes);
        LoRa.endPacket();
    }
}

void handleLoRaReception() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        int i = 0;
        while (LoRa.available() && (i < LORABUFLENGTH)) {
            outgoing[i] = LoRa.read();
            i++;
        }
        Serial.print("Received Bundle Length: ");
        Serial.println(i);
        Serial.print("Received Bundle (Hex): ");
        for (int j = 0; j < i; j++) {
            if (outgoing[j] < 16) Serial.print("0");
            Serial.print(outgoing[j], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}
