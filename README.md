# Agni Robot

* Agni is a robot powered by [Arduino Mega 2560](https://github.com/LasithaAmarasinghe/Agni-Robot/blob/main/Arduino%20Mega%202560%20Board.pdf), capable of doing various tasks in robotics competitions.
* This is developed for the semester 3 project under EN2533: Robot Design and Competition.
* The robot had to overcome several [tasks](https://github.com/LasithaAmarasinghe/Agni-Robot/blob/main/Robotics_Task.pdf) in this competition, which was held in an [arena](https://github.com/LasithaAmarasinghe/Agni-Robot/blob/main/Competition%20Arena.png) prepared in the ENTC building. 

![IMG-20231209-WA00](https://github.com/LasithaAmarasinghe/Agni-Robot/assets/106037441/81d65c23-b77b-41e4-ac23-0e065fe6b50b)

## Functionalities

🛤️ Precise line following  
🚧 Wall avoidance during navigation  
🏞️ Navigating ramps (20 degrees)  
🤖 Object interaction with a mechanical arm  
🎵 Sound sensitivity   
🌈 Colour detection  
🌀 Maze-traversing capabilities  
🛑 Obstacle avoidance, including guard robots  

## Game Plan

### Task 1: Line Following
* For the initial task of the competition, the robot needs to follow the white line on the black background. To detect the line with better accuracy we used an IR array with 8 IR modules. For smoothing the movement of the robot we added a PID controller. The PID control processes the IR sensor data and makes real-time adjustments to the robot's motor output, ensuring that it stays on the line with precision.

### Task 2: Wall Following
* In this task, the white line is disturbed by walls at some points and the robot needs to follow the white line, avoiding the walls. To detect walls we used ultrasonic sensors. The robot manages to keep a constant distance from the walls in disturbed areas and comes back to the white line after that according to the algorithm.

### Task 3: Ramp Navigation
* To go through the ramp we focused on what needed to be done by the mechanical design side. We calculated the height that the chassis needs to be from the ground and where the tyres should be situated relative to the chassis. The PID controller is adjusted for this task to slow down the robot when it comes near the ramp, so the robot can smoothly enter the ramp.

### Task 4: Box Manipulation
* To tackle the box manipulation challenge, we added a robotic arm with stepper motors equipped with joints for precise movements. We used ultrasonic sensors to detect the presence of the box. After the box is detected, the arm grabs the box according to the code.

### Task 5: Coloured Line Following
* For this task, the robot needs to identify the colour of the box and follow the path with the same colour as the box. First, the box is picked up by the arm, and then using the colour sensor, the box's colour is detected. According to the colour of the box, the robot follows the line according to the right-hand rule to reach the destination.

### Task 6: Sound Detection
* In this part, there is a sound-emitting tower. The robot should freeze when a sound is emitted from the tower and should move only when it's silent. With the sound sensor, the robot detects the sound and moves according to the algorithm. 

### Task 7: Guard Bot Avoidance
* In the final task, the robot encounters a guard robot blocking its path. Ultrasound sensors are strategically positioned on the front sides of the robot to detect the movements of the guard bot. When the bot moves away from our destination, our robot successfully evades the guard robot and reaches the final destination.

## Hardware Specifications

* [Arduino Mega 2560](https://github.com/LasithaAmarasinghe/Agni-Robot/blob/main/Arduino%20Mega%202560%20Board.pdf)
* [Sensors](https://github.com/LasithaAmarasinghe/Agni-Robot/blob/main/AGNI%20sensors%20.pdf)
* Buck Converter
* 2 Li-ion Batteries
* 2 N20 50 rpm Motors
* Wheels

## Software Specifications

* Arduino IDE
* Solid Works

![Arduino](https://img.shields.io/badge/-Arduino-00979D?logo=Arduino&logoColor=white)
![Solidworks](https://img.shields.io/badge/Solid_Works_-red)

## Sensors

![image](https://github.com/LasithaAmarasinghe/Agni-Robot/assets/106037441/e7ea8e2a-3c1e-45ae-9f9a-de4333612ad0)

## Chassie Design

<img src="https://github.com/LasithaAmarasinghe/Agni-Robot/assets/106037441/6b88fad9-39b2-48c4-ba49-6703d3684168" alt="Bottom Chassie" width="700" height="500"/>

## Team

![image](https://github.com/LasithaAmarasinghe/Agni-Robot/assets/106037441/28c360bd-b7ae-4253-b2ec-855202097034)
<p align="center">
  <img src="https://github.com/LasithaAmarasinghe/Agni-Robot/assets/106037441/b11320d2-f724-4cf0-9a87-100cba223529" alt="Image Description">
</p>


## License
 
 * This project is licensed under the MIT License. See the [LICENSE](MIT-LICENSE.txt) file for details.

## For More Information - [Project Report](https://github.com/LasithaAmarasinghe/Agni-Robot/blob/main/Agni%20final%20report.pdf)
