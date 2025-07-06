<img src="/img/icon.png" alt="icon" height=100/>

### SteadyHaystack
SteadyHaystack is an ESP32-C6 firmware based on https://github.com/liucoj/OHS-Arduino/blame/main/OpenHS.ino to implement a motion sensor and make a stealthy OpenHaystack-based bicycle tracker. https://github.com/seemoo-lab/openhaystack

## But why?
Idea came to me after a tweet from Pavel Zhovner, the CEO of Flipper devices, in which he describes an "undetectable" airtag with a motion sensor, that activates after a not moving for 5 hours. I've decided to make a similar device, but instead use a cheap microcontroller and OpenHaystack.

## Parts
This project uses a ESP32-C6 super mini board with a built-in battery controller. It also uses a MPU6050 motion sensor to detect motion. Nothing special.

## Build and configure
To build and upload firmware you need PlatformIO. To configure pins and delays, use the defines in ```src/main.cpp``` file. Default activation delay is set to 1 minute and default sleep delay is set to 5 seconds.
