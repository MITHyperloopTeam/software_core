# set root to oot of software directory
export MIT_HL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)"

# Primary software build directoy for externals
export PATH=$MIT_HL_ROOT/build/bin:$PATH
export PATH=$MIT_HL_ROOT/tools/net:$PATH
export PATH=$MIT_HL_ROOT/scripts:$PATH

export LCM_DEFAULT_URL=udpm://239.255.76.67:62237?ttl=0

export LIBRARY_PATH=$MIT_HL_ROOT/build/lib:$LIBRARY_PATH
export LD_LIBRARY_PATH=$MIT_HL_ROOT/build/lib:$LD_LIBRARY_PATH
export CLASSPATH=$CLASSPATH:$MIT_HL_ROOT/build/share/java/lcmtypes_mithl.jar
export CLASSPATH=$CLASSPATH:$MIT_HL_ROOT/communication/lcmtypes/built_types/lcmgen_j/
export CLASSPATH=$CLASSPATH:$MIT_HL_ROOT/build/share/java/lcmtypes_bot2-core.jar

# add libserial
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MIT_HL_ROOT/externals/libserial/build/lib
# add pyassimp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MIT_HL_ROOT/build/lib/python2.7/site-packages/pyassimp

# add lcm libs to pythonpath
export PYTHONPATH=$MIT_HL_ROOT/build/lib/python2.7/site-packages:$PYTHONPATH
export PYTHONPATH=$MIT_HL_ROOT/build/lib/python2.7/dist-packages:$PYTHONPATH
export PYTHONPATH=$MIT_HL_ROOT/communication/lcmtypes/built_types/lcmgen_p:$PYTHONPATH
export PYTHONPATH=$MIT_HL_ROOT/odroid/:$PYTHONPATH

# add common utils for python
export PYTHONPATH=$MIT_HL_ROOT/common:$PYTHONPATH

alias MITHL_SPY="lcm-spy -l udpm://239.255.76.67:62237"
export MITHL_PORT=62237
export MITHL_ODROID_IP=192.168.0.15
alias MITHL_BOT_TUNNEL_SERVER='bot-lcm-tunnel -l udpm://239.255.76.67:62237 -S "(SIM_).*" -R "(SIM_).*"'
alias MITHL_BOT_TUNNEL_SERVER_NO_ANALOG='bot-lcm-tunnel -l udpm://239.255.76.67:62237 -S "(SIM_|_AF_|_AR).*" -R "(SIM_|_AF_|_AR).*"'
alias MITHL_BOT_TUNNEL_CLIENT="bot-lcm-tunnel -l udpm://239.255.76.67:62237 192.168.0.15 -R SIM_.* -S SIM_.*"
alias MITHL_LOGGER="lcm-logger --lcm-url=udpm://239.255.76.67:62237"
alias MITHL_LOGPLAYER="lcm-logplayer-gui --lcm-url=udpm://239.255.76.67:62237"
alias MITHL_PROCMAN_SHERIFF="bot-procman-sheriff ${MIT_HL_ROOT}/config/pod.pmd"

alias cdmithl="cd ${MIT_HL_ROOT}"
alias cdmit="cdmithl"
alias cdmhl="cdmithl"
