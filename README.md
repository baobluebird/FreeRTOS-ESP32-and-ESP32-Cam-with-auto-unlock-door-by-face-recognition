# FreeRTOS-ESP32-and-ESP32-Cam-with-auto-unlock-door-by-face-recognition

This is the ESP32 code responsible for receiving data from the HCSR04 sensor to send it to Blynk. Blynk will then send a signal to the ESP32 CAM to take a photo. If the person is identified as family, the ESP32 will unlock the door; if not, it will trigger a warning sound at the door and on the Blynk app.

Here is the English translation of your text:

"For the FreeRTOS part, I use a Binary Semaphore to close the door with a servo when someone passes the second HCSR04 sensor (closeDoorTask). I use a Mutex to manage two tasks, distanceOutDoorTask and distanceInDoorTask. When the door is closed, the distanceOutDoorTask will run to check if anyone is near the door. If someone is near, it will send the distance data to a Queue, allowing the processDistanceTask to run and trigger Blynk to take a photo. If the photo shows family, the ESP32 will receive a signal from Blynk to command the door to open. When the door is open, the distanceInDoorTask will run to check if someone passes through. If someone does pass through, the door will be closed again. vTaskSuspend and vTaskResume are used to allow distanceInDoorTask to run only when the door is open.

xTaskCreatePinnedToCore is used to allocate resources, ensuring that the Blynk task always runs on a separate core so that other tasks do not interfere.

The distanceInDoorTask has the highest priority to ensure that it will run and not be preempted by other tasks.

The ESP32 CAM mainly takes photos and sends them to the AI server. Counting Semaphore is primarily used to limit the number of photos taken."
