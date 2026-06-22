import random
import traci
import config
import engine

# 1. Initialize fleet configuration layout
config.setup_limited_fleet()

print("[LAUNCH] Launching Consolidated Native Ride-Pooling DRT Engine...")
traci.start([
    "sumo-gui",
    "-n", config.NET_FILE,
    "-r", config.ROUTES_FILE,
    "--time-to-teleport", "-1",  # Force physical compliance
    # REQUIRED for traci.vehicle.dispatchTaxi() to work at all. Without this,
    # SUMO loads its own built-in dispatcher instead, which silently assigns
    # reservations on its own (one passenger at a time) and rejects every
    # dispatchTaxi() call from this script with:
    #   "device.taxi.dispatch-algorithm 'traci' has not been loaded"
    "--device.taxi.dispatch-algorithm", "traci",
])

# 2. Reset per-taxi commitment tracking now that we have a live connection
engine.initialize_fleet_states()

# 3. Extract edge networks
all_edges = [e for e in traci.edge.getIDList() if not e.startswith(":")]
step = 0
passenger_counter = 0

# 4. Main Step Execution Pipeline Loop
while step < config.SIMULATION_END_TIME:
    traci.simulationStep()

    # Randomly spawn passenger demands directly inside the active simulation step
    if step % 20 == 0 and random.random() < 0.65:
        p_id = f"passenger_{passenger_counter}"

        # Find a valid start->end pair with a real, driveable route between
        # them of at least 100m. This filters out two failure modes that cause
        # passengers to get permanently stuck in currentCustomers:
        #   1. Trips where start or end edge is currently occupied by a taxi's
        #      routing (SUMO can't place a stop on an edge it already passed).
        #   2. Trivially short or same-direction trips where the stop insertion
        #      fails silently and the passenger never gets served.
        # We try up to 10 random pairs before giving up for this step.
        start_edge = None
        end_edge = None
        for _ in range(10):
            s = random.choice(all_edges)
            e = random.choice(all_edges)
            if s == e:
                continue
            try:
                route = traci.simulation.findRoute(s, e, vType="taxi")
                if route.length >= 100.0:
                    start_edge = s
                    end_edge = e
                    break
            except traci.exceptions.TraCIException:
                continue

        if start_edge is None:
            step += 1
            continue

        try:
            # Add person into the map environment, standing on start_edge
            traci.person.add(p_id, start_edge, pos=0)

            # Send directly into our native reservation pool pipeline
            engine.register_passenger(p_id, start_edge, end_edge)
            print(f"[NEW DEMAND] {p_id} requested transit: [{start_edge}] -> [{end_edge}]")
            passenger_counter += 1
        except traci.exceptions.TraCIException as e:
            print(f"[SPAWN ERROR] Could not add {p_id}: {e}")

    # Reconcile our local taxi-plan tracking with SUMO's actual reservation
    # table (drops fully-completed reservations, collapses picked-up ones
    # from two occurrences to one) BEFORE we try to dispatch anything new -
    # otherwise dispatchTaxi() gets sent stale reservation ids and SUMO
    # rejects the whole call.
    engine.sync_reservation_state()

    # Run native dispatcher calculations
    engine.match_and_dispatch_fleet(max_wait=1800)

    # Print status every 40 steps
    if step % 40 == 0:
        engine.print_dashboard_metrics()

    step += 1

traci.close()