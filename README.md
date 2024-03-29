# LIRIC

Source code for the Liverpool Infra Red Imaging Camera (LIRIC) instrument.

This is a near infra-red imaging instrument, based around a Raptor Photonics Ninox 640 camera, with a shortwave indium gallium arsenide (InGaAs) detector.

The instrument also has a starlight express filter wheel. There is also an offsetting mechanism (called the nudgematic) which moves the
camera body slightly in x and y in the image plane, enabling us to do sky offsets quickly without moving the telescope. 

## Directory Structure

* **fmt** Raptor Photonics Ninox 640 configuration files
* **c** This contains the source code for the C layer, that is sent commands from the Java software and controls the mechanisms.
* **include** The header files for the C layer source code.
* **scripts** Deployment and engineering scripts.
* **java** This contains the source code for the robotic layer, which receives commands from the LT robotic control system.
* **detector** This is a C library that uses the Raptor SDK to provide a library to control the Raptor Ninox 640 detector.
* **filter_wheel** Starlight Express filter wheel control library
* **nudgematic** This contains code for a C library/test programs to move the nudgematic offseting mechanism. This is done by sending text commands over a USB link to an Arduino Mega, which has a motor controller board to control two motors moving the offset stage, and analogue inputs to read the motor output shaft encoders to determine the offset stage's position in x and y.
* **usb_pio** Library to communicate with the BMCM USB-PIO controller, used to control the BMCM OR8 IO board. This is no longer used by LIRIC.

The Makefile.common file is included in Makefile's to provide common root directory information.

## Dependencies / Prerequisites

* The Raptor SDK must be installed
* The eSTAR config repo/package must be installed.
* The log_udp repo/package must be installed: https://github.com/LivTel/log_udp
* The ngatastro repo/package must be installed.
* The commandserver repo/package must be installed: https://github.com/LivTel/commandserver
* The ngat repo/package must be installed: https://github.com/LivTel/ngat
* The software can only be built from within an LT development environment

* The ics_gui package should be installed on the machine you wish to control the instrument from: https://github.com/LivTel/ics_gui
