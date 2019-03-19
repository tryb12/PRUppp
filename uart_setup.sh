export PRUN=0
export TARGET=main
echo PRUN=$PRUN
echo TARGET=$TARGET

# Configure tx
config-pin P9_24 pru_uart
# Configure rx
config-pin P9_26 pru_uart
