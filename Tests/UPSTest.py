import subprocess
import time

def get_ups_data():
    # Run the 'upsc lcbUps' command and capture its output
    result = subprocess.run(["upsc", "lcbUps"], capture_output=True, text=True)
    data = {}
    # Parse the output into a dictionary of key/value pairs
    for line in result.stdout.splitlines():
        if ':' in line:
            key, value = line.split(":", 1)
            data[key.strip()] = value.strip()
    return data

# Initialize previous values to track changes
previous_status = None
previous_charge = None
previous_runtime = None

while True:
    ups_data = get_ups_data()
    current_status = ups_data.get("ups.status", "Unknown")
    current_charge = ups_data.get("battery.charge", "Unknown")
    current_runtime = ups_data.get("battery.runtime", "Unknown")

    # Check if any key value has changed
    if (current_status != previous_status or 
        current_charge != previous_charge or 
        current_runtime != previous_runtime):
        
        print("UPS Status:", current_status)
        print("Battery Charge:", current_charge, "%")
        print("Battery Runtime:", current_runtime, "seconds")
        print("-" * 30)
        
        previous_status = current_status
        previous_charge = current_charge
        previous_runtime = current_runtime
        
    time.sleep(5)  # Poll every 5 seconds
