
# Project with ESP32 #1

	This project uses an ESP32 and a MPU6050 module.
The ESP32 configures the MPU to sets the DMP (Digital Motion Procesor), it also configures WiFi and Mqtt to connect to a broker.
	The project uses a basic of FreeRTOS to uses the two cores of the ESP32 and improve the functionality of the system.
	There are some functions to set the schedule for data sending, a function to join data in a JSON object, and  a frequency vibration calculator function.
	The code is well comment.

## Acquired knowledge

The following was the knowledge acquired by me in this project:

* JSON object library
* Mqtt protocol
* WiFi connection
* Algorithms
* Digital Filters
* Real Time Operating System (FreeRTOS)
