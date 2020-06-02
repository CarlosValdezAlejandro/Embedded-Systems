
#Project with ESP32 #1

	This project use an ESP32 and a MPU6050 module.
The ESP32 configure the MPU to set the DMP (Digital Motion Procesor), also configure WiFi and Mqtt to connect to a broker.
	The project uses a basic of FreeRTOS to use the two cores of the ESP32 and improve the functionality of the system.
	There are some functions to set the schedule for data sending, a function to join data in a JSON object, and  a frequency vibration calculator function.
	The code is well comment.

##Acquired knowledge

The following was the knowledge acquired for me in this project:

* JSON object library
* Mqtt protocol
* WiFi connection
* Algorithms
* Digital Filters
* Real Time Operating System (FreeRTOS)
