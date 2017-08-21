#!/bin/bash
# start server on pod
ssh mithl@$1 -t 'bash -ic "\${MIT_HL_ROOT}/tools/net/restart_tunnel.sh server"'

# start tunnel here
${MIT_HL_ROOT}/tools/net/restart_tunnel.sh $1

# Starts a deputy on the ODroid
ssh mithl@$1 -t 'bash -ic "\${MIT_HL_ROOT}/tools/net/restart_deputies.sh odroid"'

# Starts a deputy on the pilot laptop
${MIT_HL_ROOT}/tools/net/restart_deputies.sh pilot

# Start a procman sheriff with a script on the pilot computer.
echo Deputy restart complete.