# Swarm GUI

## Model-Viewer-Controller framework GUI Design using PyQt
The specific controller handles the entire requests from the user and will tell the rest of the GUI what to do. Once the controller receives the request, it ask the model module for information based on the request. The model is responsible to handle the data logic of a user input which means model interacts with the swarm directly and reciprocally via swarm control algorithm while also handle all data collection. After series of communications between controller module and model module, the controller will send all information obtained to the viewer for visual presentation.
![image](https://user-images.githubusercontent.com/72159394/141874461-cb3960df-b33a-4b2e-86ed-32e0132c8f73.png)
### Controller.py	
Connect GUI elements with corresponding functions in Model.py and Viewer.py. Handel the user interaction. Receive and interpret mouse and keyboard input from operator and update GUI display accordingly. Controller will record which button is clicked by the user. Such information from Controller file will then be used to inform Model.py to make appropriate change.  	
### Model.py
Stores all swarm related speed position, and formation data. Communicate with local robot control algorithm, manipulate data according to controller output and send back to ROS and controller which will later be sent to Viewer.py to make corresponding movement and change in map display. 
### Viewer.py	
Connect controller with GUI map display, controller send results from model.py to View.py to update map. Format symbols and map appearance including grid, axis, landmark. Format font and color of text displayed in the message box. Identify and report cursor position to model for motion planning. Report time and display before each command and prompt. 	
### Ui_SwarmUI.py
Qt designer file; Handles the GUI layout. (GUI elements placements) Any new element can be added using this file in Qt designer.
### Rest are Robot Swarm controlling files

## Setup
Compile Controller.py to run the GUI
