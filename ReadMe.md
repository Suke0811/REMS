# REMS (Robotics Educational Middleware System)
Currently, in early beta (v0.2.*)

## PreRequirements 
- Platform Ubuntu, Windows, Mac
- Python 3.8/3.9/ (3.10)
- IDE: Pycharm(recommended) or Visual Studio
- git

Python 3.10 may not work on Windows environments.
Generally speaking, Windows causes more problems than the others. 

# Install
### Pip install from repo (as library)
```shell
pip install git+https://github.com/Suke0811/REMS.git@main
```
This installs Rems currently from a branch. 

### Pip install from repo (as an edible library)
```shell
pip install -e git+https://github.com/Suke0811/REMS.git@main
```
This installs Rems currently from a branch as edible. 

### Alternative methods (no pip)

```bash
clone https://github.com/Suke0811/REMS.git
pip install -r requirements.txt
```
This installs all requirements for you.


## How to Run
- Run.py has an example of how to run it
- You can pick one of input options: FileInput, KeyboardInput, or JoystickInput
- You can have outputs as you want: FileOutput, AnimationOutputs
- You can add as many robot as you want: KinematicsModel, WebotsModel, NOPModel
- If you use FileOutput, you can reproduce the result with FileInput


## Notes
- Input space
- Joystick
  - JoystickInput requires a joystick controller (not required if you don't have one), or throws an error
  - We are using pygame to deal with joysticks which suppose to work cross platform and on nearly all controllers
  - Depending on the vendor of the controllers, key mapping could be different (Defined in constants.py, class JOYSTICK)
  - We tested with Xbox and Switch controllers with no issues (Plug and Play)
  - Your controller should have 2 joysticks at least.
- matplotlib is for figures


## Webots and aruco maker
- Please update Webots Woodbot motor maxVelocity to 20.(HingeJoint > Device > Motor > maxVelocity)
- Aruco Marker system is added. 
- Aruco maker detection is another robot. you need to add the ArucoMaker().
  - aruco_track = ArucoMaker(track_id=1, display=False,camera_id=1)
  - camera id could be 0 or 1 if you have more than 1 camera
  - track_id is maker id on the robot
  - display True will slow down everything. Even false, you get woodbot.avi video output.

- If you like to use your own setup, you need: 
  - ID 255 (right top), 254, 253, 252 (right bottom) 5by5 50mm marker at https://chev.me/arucogen/.
  - Camera calibration board and follow advices in https://aliyasineser.medium.com/opencv-camera-calibration-e9a48bdd1844
  - To run calibration, change your picture names to image0, image1, image2, ..., and then run ArucoHelper.py. You may need to change the box size (currently 0.022m)

___
## Debugging
### With Operator
You can do two ways:
- breakpoint()
- Operator(debug_mode=True)

Adding breakpoint() works for all situations, but not as convenient as IDE debuggers.

Operator(debug_mode=True) lets you use IDE debuggers (Pycarm_pydev), but this won't run robots in multi-process. 

### With LiteOperator
- No special methods are required with LiteOperator
- LiteOperator is NOT appropriate for hardware
- A multi-agent Webots simulation is NOT possible, but 1 Webots robot works Ok.

## Process system
- ProcessSystem allows you to conduct a process after every timesteps
- returning a job at def process() creates a background job. (Either thread or multi process)
- you can set callback to know when the background task is done


## Files
```
kinematics
│   README.md
│   run_example.py                    (An example code)   
│
└───doc
│   │   function.md               (Software architecture)
│   │   summary.md                (Lab summary)
│
└───sim
    │   fromulation.py            (You need to modify)
    │   constants.py
    │   Operator.py 
    │
    └───inputs
    │     │   InputSystem         (Base class)
    │     │   FileInput.py
    │     │   Keyboard.py
    │     │   JoystickInput.py 
    │
    └───outputs
    │     │   OutputSystem        (Base class)
    │     │   FileOutput.py
    │     │   GraphOutput.py      (You need to implement)
    │     │   AnimationOutput.py  (You need to implement)
    │
    └───robots
    │     │   RobotSystem.py      (Base class)
    │     │   KinematicsModel.py  (Kinematics Model of differential drive)
    │     │   NOPModel.py         (Do nothing)
    │     │   WebotsModel.py      Webots implementation of differntial drive
    │     │   WebsocketConnection.py  Differential drive robot hardware 
    │ 
    └───PROCESS
          │   ProcessSystem.py    (Base class)
          │   TestProcess.py      (example showing how to create, submit and get callback from a job)

```  
