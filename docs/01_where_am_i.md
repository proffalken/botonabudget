# 01. Where Am I?

The first task I want to tackle is to make sure the Autonomous Guided Vehicle knows where it is at all times.

To do this, we're going to use two simple components:

   * A Raspberry Pi Pico W
   * A [U-BLOX 6M-NEO](https://www.u-blox.com/en/product/neo-6-series) GPS Breakout board

The Pi Pico W will connect to both the GPS module via a Serial connection and our ROS2 Framework via WiFi in order to send the location data into the control system.

## What are we collecting?

The data we're collecting is as follows:

   1. Location (Latitude/Longitude in degrees)
   2. Altitude (Metres above sea level)
   3. Signal Status (Fixed / Lost)
   4. Covariance (How confident are we in the location's accuracy?)

This gives us enough data to report back on our location with confidence and for the control system to know when we've reached our goal.

## What do we need to do?

### Preparing Micro Ros to run via PlatformIO

The first thing we need to do is setup our PlatformIO installation so we can use it with MicroROS.

For the purposes of this AGV I'm going to be using the Arduino framework rather than the RP2040 SDK as there are a wider range of libraries available for us to capitalise on.

To do this, we need a `platformio.ini` file with the following content:

```ini
; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:rpipicow]
platform = raspberrypi ; The hardware platform we're building for
board = rpipicow       ; The board we are targeting
framework = arduino    ; The framework to use - NOTE: This is the Arduino Framework and *NOT* the RP2040 SDK


lib_deps = 
    https://github.com/micro-ROS/micro_ros_platformio ; Pull the LATEST micro_ros_platformio directly from GitHub (HEAD), this includes the microros_arduino framework
    adafruit/Adafruit GPS Library@^1.7.5              ; Add the Adafruit GPS Library to get the data from the module

; Use ROS 2 Kilted and Wi-Fi transport.  If we don't add this, messages may be incompatible and it will default to serial communications
board_microros_distro = kilted
board_microros_transport = wifi

build_flags = 
    -D WIFI_SSID=\"MyTravelRouter\"
    -D WIFI_PASS=\"MyRouterPassword\"
    -D ROS_AGENT_IP=\"My.IP.Of.Controller\"
    -D ROS_AGENT_PORT=8888
;
; run our custom SCons script before linking the final firmware
extra_scripts = remove_atomic.py

```

!!! Warning "Updating the build flags"
    You MUST update the `WIFI_SSID`, `WIFI_PASS`, and `ROS_AGENT_IP` details with the ones for your environment otherwise this will never work!

At this point I'd recommend creating an "empty" arduino sketch at `src/main.cpp` with the following content:

```cpp
#include <Arduino.h>

void setup(){}

void loop(){}
```

and then running `platformio run`, before going off and having a screen break whilst PlatformIO installs the Micro ROS framework.  This can take some minutes, but only needs to happen on the first install, so best to do it when there isn't any code in the repo to upset it!

### Creating the tracker

Remember how we said we were going to follow the Unix philosphy of "Do one thing and do it well"? This is the first proof of that - the *only* purpose of this module is to tell us where we are (and whether we're the right way up, but that's a later addition!).

We need to create code that will do the following:

   1. Import any libraries necessary
   2. Create some helper functions to read the data and output it to both serial and ROS2 Topics
   3. Connect to the WiFi network, ROS2 Agent, and GPS module
   4. Read the GPS data from the module and publish it on the ROS2 topic for something else to read

And here is that code:

```cpp
// We're using the Arduino Framework
#include <Arduino.h>

// We need the platformio version of micro-ros
#include <micro_ros_platformio.h>

// Now bring in the core ROS2 code for 
// nodes and publishers
#include <rcl/rcl.h>
#include <rclc/rclc.h>

// Include the message types we're going to send
#include <std_msgs/msg/string.h>

// Include our standard string functions
#include <rosidl_runtime_c/string_functions.h>

// Bring in the GPS message formats
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/nav_sat_status.h>

// Add the library files required to sync time with the micro ros agent
#include <rmw_microros/rmw_microros.h>

/******* WIFI SETUP ********/

// Set the variables from the build flags
char ssid[] = WIFI_SSID;
char psk[]  = WIFI_PASS;

/******* Agent ********/
// Create the object, but don't set it until the setup() function
IPAddress agent_ip;
const uint16_t agent_port = ROS_AGENT_PORT;

/******* GPS ********/
#include <Adafruit_GPS.h>

// ---- Hardware serial port connected to GPS ----
#define GPSSerial Serial1
// Set the pins for the Serial connection to the GPS module
// You can override these with build-flags if you decide to use different pins
#ifndef GPS_TX_PIN
  #define GPS_TX_PIN 0   // MCU TX -> GPS RX
#endif
#ifndef GPS_RX_PIN
  #define GPS_RX_PIN 1   // MCU RX -> GPS TX
#endif

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console.
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO false

// NMEA baud (most GPS modules ship at 9600)
#define NMEA_BAUD 9600

// Timers
uint32_t gps_print_timer = 0;
uint32_t ros_pub_timer   = 0;

/******* Micro ROS Setup ********/
// We declare these early to avoid issues around assigning/using them before they are ready 
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Create two publishers - the first will be used for string messages, the second for GPS Data
rcl_publisher_t pub;
rcl_publisher_t gps_pub;

/******* Helpers ********/

// Print the current GPS status to the serial output (Usually connected to USB)
static inline void print_gps_snapshot()
{
  Serial.print("\nTime: ");
  if (GPS.hour < 10)   Serial.print('0');
  Serial.print(GPS.hour, DEC); Serial.print(':');
  if (GPS.minute < 10) Serial.print('0');
  Serial.print(GPS.minute, DEC); Serial.print(':');
  if (GPS.seconds < 10) Serial.print('0');
  Serial.print(GPS.seconds, DEC); Serial.print('.');
  if      (GPS.milliseconds < 10)   Serial.print("00");
  else if (GPS.milliseconds < 100)  Serial.print('0');
  Serial.println(GPS.milliseconds);

  Serial.print("Date: ");
  Serial.print(GPS.day, DEC); Serial.print('/');
  Serial.print(GPS.month, DEC); Serial.print("/20");
  Serial.println(GPS.year, DEC);

  Serial.print("Fix: ");     Serial.print((int)GPS.fix);
  Serial.print("  quality: "); Serial.println((int)GPS.fixquality);

  if (GPS.fix) {
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 4);  Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
    Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    Serial.print("Angle: ");         Serial.println(GPS.angle);
    Serial.print("Altitude: ");      Serial.println(GPS.altitude);
    Serial.print("Satellites: ");    Serial.println((int)GPS.satellites);
    Serial.print("Antenna status: ");Serial.println((int)GPS.antenna);
  }
}


// Update the ROS2 message with the latest values and send it to the agent
static inline void update_ros_gps()
{
  // Create the message type
  sensor_msgs__msg__NavSatFix fix;
  // Initialise it
  sensor_msgs__msg__NavSatFix__init(&fix);

  // Make sure the header timestamp matches the one we sync'd from the agent
  int64_t now_ns = rmw_uros_epoch_nanos();           // 0 if not initialised/synced
  fix.header.stamp.sec     = (int32_t)(now_ns / 1000000000LL);
  fix.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

  // Only send the message if we have a GPS fix.
  // Note that we don't really care how good that fix is, because we use the 
  // covariance to tell the control platform how much it can trust the location
  if (GPS.fix) {
    // Set the status of the GPS FIX
    fix.status.status  = sensor_msgs__msg__NavSatStatus__STATUS_FIX;
    // Tell us which service we're using (GPS, Glonas etc)
    fix.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;

    // Get the lat/long in degrees and append it to the message
    fix.latitude  = GPS.latitudeDegrees;
    fix.longitude = GPS.longitudeDegrees;
    // Add the Altitude
    fix.altitude  = GPS.altitude;

    /**** SET THE COVARIANCE SO WE KNOW HOW WELL WE CAN "TRUST" GPS */
    const float uere = (fix.status.status >= sensor_msgs__msg__NavSatStatus__STATUS_SBAS_FIX)
                       ? 2.5f   // SBAS / good multi-constellation
                       : 5.0f;  // plain GPS
    
    // Pull DOPs from your Adafruit GPS parser
    // (For Arduino Adafruit_GPS, fields are typically GPS.HDOP and maybe GPS.VDOP; if not present, set NAN)
    float hdop = GPS.HDOP;                    // unitless
    float vdop = (isnan(GPS.VDOP) ? NAN : GPS.VDOP);
    
    if (!(hdop > 0.0f)) { // if missing/zero, fall back safely
      hdop = 1.5f;
    }
    
    const float sigma_h = hdop * uere;                           // metres (1-sigma)
    const float sigma_u = (vdop > 0.0f ? vdop : 1.9f*hdop)*uere; // metres (1-sigma)
    
    // Fill ENU covariance (row-major), metres^2
    double *C = fix.position_covariance;
    C[0] = (double)(sigma_h * sigma_h); // EE
    C[1] = 0.0;                         // EN
    C[2] = 0.0;                         // EU
    C[3] = 0.0;                         // NE
    C[4] = (double)(sigma_h * sigma_h); // NN
    C[5] = 0.0;                         // NU
    C[6] = 0.0;                         // UE
    C[7] = 0.0;                         // UN
    C[8] = (double)(sigma_u * sigma_u); // UU
    
    fix.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_APPROXIMATED;

  }

  // Send the message using the appropriate publisher
  rcl_publish(&gps_pub, &fix, NULL);

  // Clean up our message
  sensor_msgs__msg__NavSatFix__fini(&fix);

}

void setup() {
  // Start the serial port for debugging
  Serial.begin(115200);
  delay(10);

  // ----- GPS initialisation -----
  #if defined(ARDUINO_ARCH_RP2040)
    // Ensure pins are mapped before begin()
    GPSSerial.setTX(GPS_TX_PIN);
    GPSSerial.setRX(GPS_RX_PIN);
  #endif

  // 9600 NMEA is the default baud rate for Adafruit GPS modules (some older ones are 4800)
  GPS.begin(NMEA_BAUD);

  // Only the essentials while debugging: RMC (date/time/speed) + GGA (fix/alt)
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // 1 Hz update to give the parser ample time
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  // Antenna status (optional; some chipsets ignore)
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(500);

  // Ask for firmware/version (module will respond on NMEA stream)
  GPSSerial.println(PMTK_Q_RELEASE);

  Serial.println("[init] Serial initialised for debugging");
  Serial.println("[init.ros] Setting up WiFi Transport");

  // NOTE: We use the built-in Ros WiFi transport, this removes the need for the
  // WiFi library in the Arduino Framework to be configured directly, but also means
  // that if this is not configured correctly then you will not get WiFi.

  // ----- Micro-ROS transport -----
  // Convert the ROS Agent IP Address from the build flag value
  agent_ip.fromString(ROS_AGENT_IP);
  // Connect to WiFi and the ROS2 Agent using the WiFi and Agent settings
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  // Set the date/time from the agent so we're in sync
  rmw_uros_sync_session(1000);  // try to sync epoch with the Agent (NTP-like)

  // ----- Minimal ROS context -----
  Serial.println("[init.ros] Setting up default micro-ros alloc");
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  Serial.println("[init.ros] Setting up micro-ros node");
  rclc_node_init_default(&node, "pico_w_minimal_talker", "", &support);

  Serial.println("[init.ros] Setting up micro-ros publisher");
  rclc_publisher_init_default(
      &pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "debug"
  );

  Serial.println("[init.ros] Setting up micro-ros GPS publisher");
  rclc_publisher_init_default(
      &gps_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
      "/gps/fix"
  );

  

  // Send an initial test message so we know the communication with the agent is working
  {
    std_msgs__msg__String msg;
    rosidl_runtime_c__String__init(&msg.data);
    rosidl_runtime_c__String__assign(&msg.data, "hello from pico w");
    Serial.println("[init.ros] Sending a test message");
    rcl_publish(&pub, &msg, NULL);
    rosidl_runtime_c__String__fini(&msg.data);
  }

  // Create a couple of timers so we can send the data at different intervals
  gps_print_timer = millis();
  ros_pub_timer   = millis();
}

void loop() {
  // ---- Drain GPS bytes as fast as possible ----
  // This tight loop prevents buffer overruns / garbled sentences.
  while (GPS.available()) {
    char c = GPS.read();
    if (GPSECHO && c) {
      Serial.write(c);
    }

    if (GPS.newNMEAreceived()) {
      // Parse once per complete sentence
      char *nmea = GPS.lastNMEA();
      if (!GPS.parse(nmea)) {
        // Bad checksum or incomplete sentence; skip
        continue;
      }
      // On success, GPS.* fields are updated
    }
  }

  // ---- Publish a ROS debug heartbeat at 1 Hz (unchanged in spirit) ----
  if (millis() - ros_pub_timer >= 1000) {
    ros_pub_timer += 1000;
    std_msgs__msg__String msg;
    rosidl_runtime_c__String__init(&msg.data);
    rosidl_runtime_c__String__assign(&msg.data, "hello from pico w");
    rcl_publish(&pub, &msg, NULL);
    rosidl_runtime_c__String__fini(&msg.data);
  }

  // ---- Print a human snapshot and update ROS with the location every ~2 seconds ----
  if (millis() - gps_print_timer >= 2000) {
    gps_print_timer += 2000;
    print_gps_snapshot();
    update_ros_gps();
  }
}

```

If you copy that code into your `src/main.cpp`, update your build flags in `platformio.ini` and then upload and monitor the pi pico (`platformio run -t upload && platformio device monitor`), you should see the following output on the serial console:

```
Time: 12:02:56.000
Date: 22/9/2025
Fix: 1  quality: 1
Location: <LAT,LNG>
Speed (knots): 0.05
Angle: 237.75
Altitude: 137.50
Satellites: 7
Antenna status: 0

Time: 12:02:58.000
Date: 22/9/2025
Fix: 1  quality: 1
Location: <LAT,LNG>
Speed (knots): 0.00
Angle: 237.75
Altitude: 135.80
Satellites: 6
Antenna status: 0

```
## How do we know it worked?

The simplest way to check it's worked is to listen to the ROS2 topic.

Depending on how you installed ROS2, you need to run `ros2 topic list /gps/fix` and you should see the same data (in a slightly different format!) arriving on the topic.

Once you see that data, you can start to visualise it in your UI of choice, and I'll cover that in a later section.

### Checking in Foxglove

If you're running Foxglove, you can add a `Map` panel and select the `/gps/fix` topic as the source - you should see your device turn up on the map at it's current location, although you may need to zoom in to see it properly!
