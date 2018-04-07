# Brown IEEE Robotics Olympiad Software
Team:
  - Theo Guerin
  - Peter Huson
  - Camden Baer
  - Noah Rubin
  - Lina Lim

#### In order to connect to the Raspberry Pi, follow these steps:
  - Plug the Pi into a power source (USB has worked so far)
  - Connect to the "defaultdrone" network (wait 60 seconds or so until the Pi has booted)
  - The password is [ask Peter]
  - Open a terminal prompt and type the following command:
    
```sh
$ ssh pi@defaultdrone.local
```
    
  - Type in the password "bigbubba"
  - You are now connected to the Pi!

#### In order to run the script on startup: 
   - Paste the following code 
```sh
exec 2> /tmp/rc.local.log   # Send stderr to a log file
exec 1>&2                   # Send stdout to the same log file

pigpiod                                 # Starts gpio daemon
python /home/pi/A-Maze/turnon.py        # Flashes LEDs
python /home/pi/A-Maze/main/robot.py    # Runs main program
```
