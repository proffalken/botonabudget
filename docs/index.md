# Bot on a budget

Welcome to Bot on a Budget, where I'm documenting my attempts to build a fully autonomous guided vehicle with the minimal amount of spending money.

A lot of the components I'm using are ones that I've had lying around on my workbench for various failed projects for years, but whatever else I buy, I'm going to try and keep the cost for each component to under £20 (GBP) unless absolutely unavoidable.

## What's the end goal?

By the time I'm finished, I want to have an Autonomously Guided Vehicle (AGV) with the following features:

   1. Fully managed and controlled via [ROS2](https://docs.ros.org/en/rolling/index.html)
   2. Knows where it is based on GPS
   3. Can understand whether it is upside down or not using a basic 9DoF IMU
   4. Can "see" what's infront of it via a variety of means:
      * HC-SR04 distance sensors
      * LIDAR
      * Video Processing
   5. Can accept simple commands such as "forward", "back" etc. and move accordingly
   6. Can proceed to a location, avoiding obstacles, without needing further instructions

To make it even more challenging, I've decided to use the [Raspberry Pi Pico W](https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html#picow-technical-specification) for most of the sensors/actuators,
meaning that I need to connect the messages from various boards together as a single Pico W will not be powerful enough to run the logic for the entire system.

## What about standards and approaches?

I'm a stickler for standardising things to try and make things easier, so I'll be doing my best to follow these approaches and standards:

* [OpenTelemetry (OTEL)](https://opentelemetry.io) - OTEL is rapidly becoming the industry standard for logs, metrics, traces, and profiling web applications, so I'm going to apply it to robotics as well!
* [The Unix Philosophy](https://en.wikipedia.org/wiki/Unix_philosophy) - Often summarised as "Do one thing and do it well", I'll be making the AGV as modular as possible so I can upgrade in future.  Using ROS2 as the control platform will make this a lot easier as I'll be able to upgrade modules to more powerful hardware in future whilst retaining message compatibility and avoiding a rewrite of the entire framework.


