# State of one vehicle in the city.
#
# Published by a recruit's vehicle node, drawn by transit_sim. The vehicle does
# not send screen coordinates: it says which numbered lane it is on and how far
# along that lane it is, and the sim converts that to a position using the lane
# polyline it owns. See SIMULATION_PLAN.md sections 3 and 6.

string  vehicle_id     # Unique per publisher, e.g. "car_anbit". New ids appear automatically.
uint16  lane_id        # Which numbered lane, as printed on the map.
float64 progress       # How far along the lane, 0.0 (start) .. 1.0 (end).
float64 velocity       # Speed in m/s. Informational only, the sim does not integrate it.
string  color          # Colour name, e.g. "red". The sim maps the name to RGBA.
bool    moving         # False parks the vehicle, e.g. waiting at a red light.
