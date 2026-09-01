# State of one traffic light.
#
# A light is identified by the lane it governs, which reuses the lane numbering
# from SIMULATION_PLAN.md section 5 as the shared vocabulary between recruits
# and the sim. The sim places the light at that lane's stop line, offset to the
# right of the direction of travel (European convention, section 4).

uint8 RED=0
uint8 YELLOW=1
uint8 GREEN=2

string signal_id       # Unique per publisher, e.g. "junction1_north".
uint16 lane_id         # The incoming lane this signal governs.
uint8  state           # One of RED / YELLOW / GREEN.
