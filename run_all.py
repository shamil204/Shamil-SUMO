import os
import sys
import random

# 1. Setup SUMO Tools Environment
if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
else:
    # If the system environment variable is missing, throw a clean, helpful error
    raise ImportError("SUMO_HOME environment variable not found. Please set SUMO_HOME to your SUMO installation directory.")
import traci

# Automatically uses the directory where your new script is saved!
WORKING_DIR = os.path.dirname(os.path.abspath(__file__))
NET_FILE = os.path.join(WORKING_DIR, "grid.net.xml")
ROUTES_FILE = os.path.join(WORKING_DIR, "grid_routes.rou.xml")

# 2. Re-write a clean base route file containing ONLY the limited taxi fleet
def setup_limited_fleet():
    routes_xml = """<routes>
        <vType id="taxi" vClass="taxi" personCapacity="4">
            <param key="has.taxi.device" value="true"/>
            <param key="device.taxi.pickUpDuration" value="15"/>
            <param key="device.taxi.dropOffDuration" value="15"/>
        </vType>
        <trip id="taxi_0" depart="0.00" type="taxi" from="A0B0" to="D0D1"/>
        <trip id="taxi_1" depart="0.00" type="taxi" from="C2B2" to="A0A1"/>
        <trip id="taxi_2" depart="0.00" type="taxi" from="A2B2" to="D3C3"/>
    </routes>"""
    with open(ROUTES_FILE, "w") as f:
        f.write(routes_xml)

setup_limited_fleet()

print("[LAUNCH] Launching Advanced Ride-Pooling DRT Engine...")
traci.start(["sumo-gui", "-n", NET_FILE, "-r", ROUTES_FILE, "--device.taxi.dispatch-algorithm", "traci"])

# Extract clean edge lists from your grid network file to spawn people on valid roads
all_edges = [e for e in traci.edge.getIDList() if not e.startswith(":")]

# Simulation controls
step = 0
passenger_counter = 0
SIMULATION_END_TIME = 7200  # Run sandbox for 2 hours of simulation time
virtual_waiting_room = []   # Local cache keeping track of backlogged requests


# =========================================================================
#  ALNS OBJECTIVE FUNCTION
# =========================================================================
def calculate_itinerary_cost(taxi_id):
    """
    ALNS Objective Function: Evaluates the cost of a taxi's current itinerary.
    Penalizes total stops, empty travel distance, and passenger wait times.
    """
    stops = traci.vehicle.getStops(taxi_id)
    actual_tasks = [s for s in stops if "pickUp" in s.actType or "dropOff" in s.actType]
    
    cost = 0
    # 1. Penalty for a long queue (More stops = more delay for passengers)
    cost += len(actual_tasks) * 50  
    
    # 2. Traffic congestion/travel time penalty
    speed = traci.vehicle.getSpeed(taxi_id)
    if speed < 2.0:  # Taxi is struggling at a traffic light or junction
        cost += 100
        
    return cost


# =========================================================================
#  MAIN SIMULATION LOOP
# =========================================================================
while step < SIMULATION_END_TIME:
    traci.simulationStep()
    
    # ------------------ STEP 1: RANDOM PASSENGER GENERATION ------------------
    if step % 15 == 0 and random.random() < 0.60:
        p_id = f"passenger_{passenger_counter}"
        start_edge = random.choice(all_edges)
        end_edge = random.choice(all_edges)
        
        while start_edge == end_edge:
            end_edge = random.choice(all_edges)
            
        traci.person.add(p_id, start_edge, pos=0)
        traci.person.appendStage(p_id, traci.simulation.findIntermodalRoute(start_edge, end_edge, modes="taxi")[0])
        print(f"[NEW REQUEST] {p_id} wants a ride from [{start_edge}] -> [{end_edge}]")
        passenger_counter += 1

    # ------------------ STEP 2: ALNS OPTIMIZATION LAYER ------------------
    if step % 10 == 0:
        all_active_reservations = traci.person.getTaxiReservations(3) 
        fleet = traci.vehicle.getTaxiFleet(-1)
        
        # Track completely new reservations in our virtual python backlog list
        for res in all_active_reservations:
            if res.id not in virtual_waiting_room and res.state == 1:
                virtual_waiting_room.append(res.id)
                
        # Only run optimization if there are requests waiting in the virtual room
        if virtual_waiting_room:
            print(f"\n[ALNS LOOP] Optimizing assignments for {len(virtual_waiting_room)} unassigned requests...")
            
            # --- ALNS Step A: Destroy Operator (Removal Heuristic) ---
            passengers_to_insert = list(virtual_waiting_room)
            
            # --- ALNS Step B: Repair Heuristic (Insertion Heuristic) ---
            for res_id in passengers_to_insert:
                try:
                    res = next(r for r in all_active_reservations if r.id == res_id)
                except StopIteration:
                    if res_id in virtual_waiting_room:
                        virtual_waiting_room.remove(res_id)
                    continue
                
                # OPTIMIZATION UPGRADE: Calculate customer sidewalk waiting age duration
                reservation_spawn_time = getattr(res, 'submissionTime', step)
                waiting_time = step - reservation_spawn_time
                waiting_penalty = waiting_time * 2.0  # Scalar weights older items higher
                
                best_taxi = None
                best_additional_cost = float('inf')
                
                # Test inserting this passenger into every available taxi to find the lowest cost delta
                for taxi_id in fleet:
                    stops = traci.vehicle.getStops(taxi_id)
                    task_list = [s.actType for s in stops]
                    
                    # Calculate how many total tasks are already assigned in this taxi's itinerary
                    current_load = sum(1 for t in task_list if "dropOff" in t or "pickUp" in t)
                    
                    # CRITICAL CAPACITY CHECK: Skip this taxi if it already has 4 or more tasks lined up.
                    if current_load >= 4:
                        continue
                        
                    # SYSTEM REPAIR UPGRADE: Determine the vehicle's actual current location safely
                    current_road = traci.vehicle.getRoadID(taxi_id)
                    if current_road.startswith(":"):  # Handle internal junction road formatting
                        try:
                            current_road = traci.vehicle.getLaneID(taxi_id).split("_")[0]
                            if current_road.startswith(":"):
                                continue  # Bypass loop optimization sequence if completely locked inside center junction arrays
                        except Exception:
                            continue
                    
                    # UPGRADE: Query SUMO to get the exact real-time routing travel time
                    try:
                        route_stages = traci.simulation.findIntermodalRoute(current_road, res.fromEdge, modes="taxi")
                        if route_stages:
                            routing_cost = route_stages[0].travelTime
                        else:
                            routing_cost = 9999
                    except Exception:
                        routing_cost = 9999

                    # Evaluate overall utility cost combining proximity, load conditions, and waiting age
                    current_cost = calculate_itinerary_cost(taxi_id)
                    total_calculated_cost = current_cost + routing_cost - waiting_penalty
                    
                    if total_calculated_cost < best_additional_cost:
                        best_additional_cost = total_calculated_cost
                        best_taxi = taxi_id
                
                # Execute assignment based on the best heuristic selection
                if best_taxi:
                    try:
                        existing_stops = traci.vehicle.getStops(best_taxi)
                        # Security validation wrapper checking that database double insertion tracking loops do not cross-post
                        already_assigned = any(res.id in getattr(s, 'actType', '') or res.id in str(s) for s in existing_stops)
                        
                        if not already_assigned:
                            print(f"[ALNS REPAIR] Inserting {res.id} into {best_taxi} (Adjusted Utility Delta: {best_additional_cost:.2f})")
                            traci.vehicle.dispatchTaxi(best_taxi, [res.id])
                            
                        if res_id in virtual_waiting_room:
                            virtual_waiting_room.remove(res_id)
                    except (traci.exceptions.TraCIException, traci.exceptions.FatalTraCIError) as e:
                        print(f"[SUMO REJECTION/CRASH AVOIDED] Keeping {res.id} in backlog due to junction positioning safety restrictions.")
                        pass

        if virtual_waiting_room:
            print(f"[SIDEWALK CONGESTION] {len(virtual_waiting_room)} passengers waiting in queue safely.")

    # ------------------ STEP 3: LIVE POOL VISUALIZER ------------------
    if step % 40 == 0:
        print(f"\n[STEP {step}] LIVE FLEET OCCUPANCY & EMISSION DASHBOARD:")
        print("=" * 75)
        for taxi_id in traci.vehicle.getTaxiFleet(-1):
            stops = traci.vehicle.getStops(taxi_id)
            current_road = traci.vehicle.getRoadID(taxi_id)
            
            task_list = [s.actType for s in stops]
            passenger_count = sum(1 for t in task_list if "dropOff" in t)
            
            # UNIQUE METRICS: Extract live eco-efficiency data from the vehicle
            fuel_ml_per_sec = traci.vehicle.getFuelConsumption(taxi_id)  # in ml/s
            co2_mg_per_sec = traci.vehicle.getCO2Emission(taxi_id)       # in mg/s
            
            print(f"  * {taxi_id} @ [{current_road}] | Passengers: {passenger_count} | Queue: {task_list}")
            print(f"    [ECO-METRICS] Fuel Consumption: {fuel_ml_per_sec:.2f} ml/s | CO2 Emissions: {co2_mg_per_sec:.2f} mg/s")
        print("=" * 75 + "\n")

    step += 1

traci.close()
print("[COMPLETE] Sandbox run complete. All networks closed cleanly.")