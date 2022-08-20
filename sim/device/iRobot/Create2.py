from  pycreate2 import Create2
import time

# Create a Create2.
port = "COM7"  # where is your serial port?
bot = Create2(port)

# Start the Create 2
bot.start()

# Put the Create2 into 'safe' mode so we can drive it
# This will still provide some protection
bot.safe()

# You are responsible for handling issues, no protection/safety in
# this mode ... becareful
bot.full()

# directly set the motor speeds ... move forward
bot.drive_direct(100, 100)
time.sleep(2)

# turn in place
bot.drive_direct(200,-200)  # inputs for motors are +/- 500 max
time.sleep(2)

# Stop the bot
bot.drive_stop()

# query some sensors
sensors = bot.get_sensors()  # returns all data
print(sensors.light_bumper_left)

# Close the connection
# bot.close()
