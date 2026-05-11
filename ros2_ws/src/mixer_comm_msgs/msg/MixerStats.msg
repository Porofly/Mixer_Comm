# Per-round Mixer statistics parsed from the dongle's serial console.
# Source line example:
#   # ID:1 round=100 rank=8 dec=8 !dec=0 weak=0 wrong=0

std_msgs/Header header

uint16 node_id        # Physical Mixer node id (TOS_NODE_ID)
uint32 round          # Round counter
uint32 rank           # Rank reached this round
uint32 decoded        # Messages successfully decoded
uint32 not_decoded    # Messages missing at round end
uint32 weak           # Messages flagged as weak zero
uint32 wrong          # Decoded but failed payload sanity check
