import traci
import config

# Reservation ids that we have personally dispatched to a taxi at some point.
# Used only to avoid re-dispatching the same reservation a second time as if
# it were brand new; NOT used to determine what's currently pending (that
# comes straight from SUMO, see _build_taxi_plan_from_ground_truth below).
_ever_dispatched = set()

# Maps reservation id -> tuple of person ids, refreshed each step from
# traci.person.getTaxiReservations(0). Needed because device.taxi.
# currentCustomers reports PERSON ids, but dispatchTaxi() needs RESERVATION
# ids - this dict bridges the two.
_reservation_persons = {}
_person_reservation = {}  # reverse lookup: person id -> reservation id

# Reservation ids whose person has already been picked up (state bit 8 in
# the last getTaxiReservations(0) snapshot). Needed to know whether a
# reservation needs ONE occurrence (dropoff only) or TWO occurrences
# (pickup + dropoff) when rebuilding a dispatch plan.
_picked_up_reservations = set()


def initialize_fleet_states():
    """Reset all tracking. Call once after traci.start()."""
    _ever_dispatched.clear()
    _reservation_persons.clear()
    _person_reservation.clear()
    _picked_up_reservations.clear()


def register_passenger(p_id, start_edge, end_edge):
    """Books a passenger directly into SUMO's native taxi reservation system.

    There is no traci.person.appendTaxiStage() - that method does not exist
    in the TraCI API. The real, documented way to book a taxi ride for a
    person is appendDrivingStage(personID, toEdge, lines), where
    lines="taxi" tells SUMO's taxi device to treat this as a taxi
    reservation instead of a regular ride on a named vehicle/line.

    We do NOT need a manual walking stage to the pickup edge - the person is
    spawned with traci.person.add() already standing on start_edge, so a
    single appendDrivingStage call both registers the reservation with the
    dispatcher AND defines the person's trip.
    """
    try:
        traci.person.appendDrivingStage(p_id, end_edge, lines="taxi")
    except traci.exceptions.TraCIException as e:
        print(f"[RESERVATION ERROR] Failed to register {p_id}: {e}")


def _get_current_customer_persons(taxi_id):
    """Ground-truth list of person ids this taxi is currently responsible
    for (still to be picked up OR already onboard awaiting drop-off), read
    directly from SUMO via 'device.taxi.currentCustomers'."""
    try:
        raw = traci.vehicle.getParameter(taxi_id, "device.taxi.currentCustomers")
    except traci.exceptions.TraCIException:
        return []
    if not raw:
        return []
    return raw.split()


def _current_customer_count(taxi_id):
    return len(_get_current_customer_persons(taxi_id))


def sync_reservation_state():
    """Call once per step, BEFORE match_and_dispatch_fleet(). Refreshes the
    reservation-id <-> person-id mapping from SUMO's reservation table.

    IMPORTANT DESIGN NOTE (after two earlier failed approaches):
    This function no longer tries to maintain a parallel "_taxi_plan"
    dictionary that mirrors what each taxi is supposedly carrying. Earlier
    versions did that, and real test runs proved it drifts out of sync with
    SUMO's actual internal state - reservations that were fully served kept
    reappearing in re-dispatch calls because completion wasn't being
    detected correctly, causing taxis to look permanently full
    (currentCustomers climbing to 4/4 and never coming back down) even
    though Onboard counts showed only 1 person physically in the car at a
    time.

    The fix is to NOT maintain a separate plan at all. Instead,
    match_and_dispatch_fleet() rebuilds each taxi's dispatch list FRESH,
    every single call, directly from device.taxi.currentCustomers (ground
    truth from the vehicle itself). This function's only remaining job is
    to keep the reservation-id <-> person-id lookup table current, since
    currentCustomers gives us person ids but dispatchTaxi() needs
    reservation ids.
    """
    try:
        all_reservations = traci.person.getTaxiReservations(0)
    except traci.exceptions.TraCIException:
        return

    for res in all_reservations:
        _reservation_persons[res.id] = tuple(res.persons)
        for p in res.persons:
            _person_reservation[p] = res.id
        if res.state & 8:  # picked up
            _picked_up_reservations.add(res.id)
        else:
            _picked_up_reservations.discard(res.id)


def _build_dispatch_plan(taxi_id):
    """Builds the correct, minimal dispatchTaxi() argument list for a taxi
    FROM SCRATCH, based on device.taxi.currentCustomers (ground truth for
    WHICH reservations are still active) combined with each reservation's
    pickup state (for WHETHER it needs one or two occurrences).

    Per the official SUMO re-dispatch rules:
      - A reservation not yet picked up needs to appear TWICE (pickup +
        dropoff) for SUMO to schedule both stops correctly.
      - A reservation already picked up (state bit 8 set) needs to appear
        ONCE (only the pending dropoff remains).

    We get the SET of currently-active reservations from
    device.taxi.currentCustomers (ground truth - this is what guarantees we
    never resend a stale/already-completed reservation id, which was the
    bug in the previous version). We get the per-reservation picked-up FLAG
    from traci.person.getTaxiReservations() state bits, refreshed each step
    by sync_reservation_state().
    """
    persons = _get_current_customer_persons(taxi_id)
    plan = []
    seen = set()
    for p in persons:
        rid = _person_reservation.get(p)
        if rid is None or rid in seen:
            continue
        seen.add(rid)
        if rid in _picked_up_reservations:
            plan.append(rid)          # already picked up: only dropoff remains
        else:
            plan.append(rid)
            plan.append(rid)          # still waiting: pickup AND dropoff remain
    return plan


def match_and_dispatch_fleet(max_wait=1800):
    """Looks at unassigned reservations and dispatches/extends taxi plans.

    Dispatch lists are rebuilt fresh from ground truth every call (see
    _build_dispatch_plan) rather than maintained incrementally, which
    eliminates the drift bug from earlier versions.
    """
    try:
        # state 3 = NEW (1) + RETRIEVED (2) - catches reservations regardless
        # of whether sync_reservation_state() already "touched" them this step.
        reservations = [r for r in traci.person.getTaxiReservations(3)
                        if r.id not in _ever_dispatched]
        if not reservations:
            return

        fleet = traci.vehicle.getIDList()
        taxi_ids = [v for v in fleet if traci.vehicle.getTypeID(v) == "taxi"]

        for res in reservations:
            best_taxi = None
            best_distance = float("inf")

            for taxi_id in taxi_ids:
                # Skip taxis that are already fully booked, using SUMO's own
                # ground-truth customer count rather than local tracking.
                if _current_customer_count(taxi_id) >= config.MAX_TAXI_CAPACITY:
                    continue

                current_edge = traci.vehicle.getRoadID(taxi_id)
                if current_edge.startswith(":"):
                    # vehicle is on an internal junction edge, route lookup will fail
                    continue

                try:
                    route_check = traci.simulation.findRoute(current_edge, res.fromEdge)
                except traci.exceptions.TraCIException:
                    continue

                if route_check.length < best_distance:
                    best_distance = route_check.length
                    best_taxi = taxi_id

            if best_taxi is None:
                continue

            # Build the taxi's CURRENT real plan from ground truth, then add
            # the new reservation as a fresh pickup+dropoff pair at the end.
            existing_plan = _build_dispatch_plan(best_taxi)
            new_plan = existing_plan + [res.id, res.id]

            try:
                traci.vehicle.dispatchTaxi(best_taxi, new_plan)
                _ever_dispatched.add(res.id)
                print(f"[NATIVE DISPATCH] Assigned reservation {res.id} "
                      f"(persons {res.persons}) to {best_taxi} "
                      f"[plan sent: {new_plan}]")
            except traci.exceptions.TraCIException as e:
                print(f"[DISPATCH ERROR] Could not assign {res.id} to {best_taxi}: {e}")

    except traci.exceptions.TraCIException:
        pass


def print_dashboard_metrics():
    """Outputs clear system tracking logs directly from SUMO's native runtime telemetry."""
    try:
        fleet = traci.vehicle.getIDList()
        taxis = [v for v in fleet if traci.vehicle.getTypeID(v) == "taxi"]

        print("\n===========================================================================")
        for taxi_id in taxis:
            pos = traci.vehicle.getRoadID(taxi_id)
            status = "STOPPED/BOARDING" if traci.vehicle.isStopped(taxi_id) else "DRIVING"

            try:
                onboard = traci.vehicle.getPersonIDList(taxi_id)
            except traci.exceptions.TraCIException:
                onboard = []

            fuel_ml_per_sec = traci.vehicle.getFuelConsumption(taxi_id)
            co2_mg_per_sec = traci.vehicle.getCO2Emission(taxi_id)
            current_customers = _get_current_customer_persons(taxi_id)

            print(f" * {taxi_id} @ [{pos}] | {status} | Onboard: {len(onboard)}/{config.MAX_TAXI_CAPACITY} "
                  f"| Current customers (picked-up + pending pickup): {len(current_customers)}/{config.MAX_TAXI_CAPACITY} {current_customers}")
            print(f"   [ECO-METRICS] Fuel: {fuel_ml_per_sec:.2f} ml/s | CO2: {co2_mg_per_sec:.2f} mg/s")
        print("===========================================================================")
    except traci.exceptions.TraCIException:
        pass