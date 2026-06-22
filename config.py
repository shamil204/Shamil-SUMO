import os
import sys

# Validate SUMO tools environment pathing
if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
else:
    raise ImportError("SUMO_HOME environment variable not found. Please set it in your system variables.")

WORKING_DIR = os.path.dirname(os.path.abspath(__file__))
NET_FILE = os.path.join(WORKING_DIR, "grid.net.xml")
ROUTES_FILE = os.path.join(WORKING_DIR, "grid_routes.rou.xml")

# Global variables
SIMULATION_END_TIME = 7200  
MAX_TAXI_CAPACITY = 4

def setup_limited_fleet():
    """Generates the initial route file using native taxi devices for automated control."""
    routes_xml = """<routes>
        <vType id=\"taxi\" vClass=\"taxi\" personCapacity=\"4\">
            <param key=\"has.taxi.device\" value=\"true\"/>
            <param key=\"device.taxi.pickUpDuration\" value=\"15\"/>
            <param key=\"device.taxi.dropOffDuration\" value=\"15\"/>
        </vType>
        <trip id=\"taxi_0\" depart=\"0.00\" type=\"taxi\" from=\"A0B0\" to=\"D0D1\"/>
        <trip id=\"taxi_1\" depart=\"0.00\" type=\"taxi\" from=\"C2B2\" to=\"A0A1\"/>
        <trip id=\"taxi_2\" depart=\"0.00\" type=\"taxi\" from=\"A2B2\" to=\"D3C3\"/>
    </routes>"""
    with open(ROUTES_FILE, "w") as f:
        f.write(routes_xml.strip())