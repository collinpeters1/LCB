#Made by Jonah Hartley
import subprocess

def get_ups_data():
    """
    Run the 'upsc lcbUps' command and parse its output into a dictionary.
    Run:
    watch -n 0.2 'date +%T.%3N; upsc lcbUPS | egrep "ups.status|battery.charge|battery.runtime"'
    if you need constant feed
    """
    result = subprocess.run(["upsc", "lcbUps"], capture_output=True, text=True)
    data = {}
    for line in result.stdout.splitlines():
        if ':' in line:
            key, value = line.split(":", 1)
            data[key.strip()] = value.strip()
    return data
