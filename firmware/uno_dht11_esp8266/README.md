# UNO DHT11 + ESP8266 server telemetry

The sketch sends one DHT11 reading every 10 seconds to the existing local
controller server at `POST /api/readings`.

## Required wiring

| Device pin | UNO R3 pin | Notes |
| --- | --- | --- |
| DHT11 DATA | D2 | Add a 4.7–10 kOhm pull-up to 5 V if this is a bare DHT11, not a module. |
| ESP8266 TX | D4 | 3.3 V output is readable as HIGH by the Uno. |
| ESP8266 RX | D3 | **Use a 5 V-to-3.3 V divider or level shifter. Do not connect directly.** |
| ESP8266 VCC / EN | regulated 3.3 V | Use a supply capable of at least 500 mA. The Uno 3.3 V pin is usually not sufficient. |
| ESP8266 GND | UNO GND | The regulator, ESP8266, DHT11 and Uno must share ground. |

For normal ESP8266 boot, keep EN/CH_PD high, GPIO0 high and GPIO2 high. The
sketch assumes an ESP8266 AT firmware baud rate of **9600**. If your module is
still at the common 115200 baud, set it to 9600 first; `SoftwareSerial` on an
Uno is not reliable at 115200.

## Server setup

1. Connect this PC and the ESP8266 to the same Wi-Fi network.
2. Start the server: `py server.py` in the `server` folder.
3. Find the PC's LAN IPv4 address (for example `192.168.x.x`).
4. Set `SERVER_HOST` in local-only `secrets.h` to that IP, then compile and
   upload the sketch.
5. Open `http://PC_LAN_IP:8000` to monitor the latest reading.

`secrets.h` is local configuration and contains the Wi-Fi credential. Keep it
out of any public repository or presentation material.
