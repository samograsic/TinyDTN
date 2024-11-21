# TinyDTN Implementation

TinyDTN is a minimalistic implementation of the Bundle Protocol v7 (BPv7) designed specifically for Arduino-based platforms. It enables LoRa communication using DTN bundles, making it suitable for small, resource-constrained devices. The library intentionally focuses only on sending DTN bundles to maintain a minimal footprint, ensuring compatibility with even the simplest Arduino devices.

## Features

- Bundle Protocol v7 (BPv7) compliance
- Minimal implementation suitable for microcontrollers
- Optional CRC calculation
- Tested on Adafruit LoRa M0 development board

## Requirements

- Adafruit LoRa M0 development board or compatible
- Arduino IDE
- LoRa library (such as LoRaSem)
- tinyCBOR library for encoding CBOR

## Getting Started



You can purchase the Adafruit LoRa M0 module from [Adafruit's official store](https://www.adafruit.com/product/3178). Please note that you will also need to add an appropriate antenna to the module for optimal performance.



### Hardware Setup

Ensure your development board is properly connected, and the pins are configured according to the following setup:

- **CS Pin**: Pin 8
- **Reset Pin**: Pin 4
- **IRQ Pin**: Pin 3

The default LoRa frequency is set to **434 MHz**. Please adjust the frequency to match your local regulations.

### Installation

1. Clone this repository:
   ```sh
   git clone https://github.com/samograsic/TinyDTN.git
   ```
2. Open the Arduino IDE and load the `ArduinoDTN.ino` sketch from the `TinyDTN` folder.

### Usage

1. Adjust the **frequency** if needed in the code:
   ```cpp
   #define FREQUENCY 434E6 // Set your local frequency here
   ```
2. Set the **source** and **destination** endpoint IDs as required.
3. If no accurate time is available, use **0** for `currentUnixTime` and increase the `sequenceNumber` on every transmission to ensure unique identifiers.

### Example

Below is an example of how to create and send a DTN bundle:

```cpp
IPNEndpointId source = {268484888, 2};
IPNEndpointId destination = {268484612, 2};
uint64_t lifetime = 3600000; // 1 hour in milliseconds
DTNBundle bundle(source, destination, currentUnixTime, 1, lifetime, true); // Set useCRC to true or false as needed
const char* payloadString = "Hello DTN world!";
bundle.setPayload(payloadString);

uint8_t encodedBundle[LORABUFLENGTH];
size_t bundleLength = bundle.encodeBundle(encodedBundle, sizeof(encodedBundle));

// Send bundle to LoRa
LoRa.beginPacket();
LoRa.write(encodedBundle, bundleLength);
LoRa.endPacket();
```

### Notes

- This project was tested on the **Adafruit LoRa M0** development board. Other compatible boards should work as well but may require adjustments.
- The **CRC calculation** is optional. You can disable it by setting `useCRC` to `false` in the `DTNBundle` constructor.
- This implementation is focused only on **sending bundles**, and reception functionality is kept to a minimal example to reduce the library's size and complexity.
- On the receiving side, the **IONe DTN** implementation was used with the UART Convergence Layer. You can find more details here: [IONe DTN UART LoRa](https://github.com/samograsic/ION-DTN-Uart-LoRa/tree/main).

## Contributing

Contributions are welcome! If you have suggestions for improvements or would like to contribute to this project, please open an issue or submit a pull request.

## License

This project is licensed under the MIT License.

## Contact

For more information, please contact Samo Grasic at [[samo@grasic.net](mailto\:samo@grasic.net)].

## Project Information

This work was conducted as part of the [IPNSIG Project Working Group](https://ipnsig.org/).

