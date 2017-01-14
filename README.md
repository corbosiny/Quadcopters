# Quadcopters
Arduino libraries for the quadcopters of the feromone robotics team

This folder should/will contain:

-All code directly involved in maneuvering the drone                   <-- WHAT WE ARE WORKING ON RIGHT NOW!!!
    
    1. The motor controller code(for ESCs)
    
    2. IMU https://www.youtube.com/watch?v=4BoIE8YQwM8(code) <-Video on the IMU code
    
    3. The PID Code for stabalizing the drone
    
    4. Any sensors whos ouptut contribute to flight adjustment:
        -GPS
        -Accel/Gyro
        -Barometer
        -magnetometer
    
    5. Kalman filter for signal filtering and state estimation
    
    6.Overall flight controller
    
    7. Test/callibration code for making sure the drone is functioning properly

GPS related code for tracking/planning flight

    -We are using the adafruit ultimate GPS module

Sensors for measurments the drone needs to take
    
    -Almost all are contained within the code of the modules they work with(EX: accel/gyro in IMU code)

Code setting up the basic communication between drones
    
    -ESP8266 will be used

Support repositories or repositories that build off of this:

    -Image Processing
    -AI/Machine Learning
    -Mathmatics
//Ask me if you need or want into these folders
