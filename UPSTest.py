##Test for the UPS system in code. Python script that periodically
##Calls the upsc lcbUps command, parses its output, and then checks
##Specific fields such as ups.status.
import subprocess
import time

def get_ups_data():
    # Run the command and capture its output
    result = subprocess.run(["upsc", "lcbUps"], capture_output=True, text=True)
    data = {}
    # Parse each line into a key-value pair
    for line in result.stdout.splitlines():
        if ':' in line:
            key, value = line.split(":", 1)
            data[key.strip()] = value.strip()
    return data

previous_status = None

while True:
    ups_data = get_ups_data()
    current_status = ups_data.get("ups.status", "Unknown")
    if current_status != previous_status:
        print("UPS status changed to:", current_status)
        previous_status = current_status
    time.sleep(5)  # Poll every 5 seconds
