# A single decoded Mixer slot payload (RX) or a payload to enqueue for a slot (TX).
#
# RX direction (/mixer/rx):
#   node_id: the *local* dongle's TOS_NODE_ID (which dongle observed this)
#   slot   : Mixer slot index (0..MX_GENERATION_SIZE-1)
#   data   : MX_PAYLOAD_SIZE bytes verbatim from mixer_read()
#
# TX direction (/mixer/tx):
#   node_id: 0 (ignored — TX is always sent from the dongle this node talks to)
#   slot   : Mixer slot to enqueue into. Must satisfy
#            payload_distribution[slot] == this dongle's TOS_NODE_ID,
#            otherwise the firmware drops the frame silently.
#   data   : Up to MX_PAYLOAD_SIZE bytes. Shorter payloads are zero-padded
#            on the host side before framing.

std_msgs/Header header

uint16 node_id
uint8  slot
uint8[] data
