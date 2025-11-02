import subprocess

def get_ups_data():
    """
    Run the 'upsc lcbUps' command and parse its output into a dictionary.
    """
    result = subprocess.run(["upsc", "lcbUps"], capture_output=True, text=True)
    data = {}
    for line in result.stdout.splitlines():
        if ':' in line:
            key, value = line.split(":", 1)
            data[key.strip()] = value.strip()
    return data
