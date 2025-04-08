 #! /usr/bin/bash

seed=$RANDOM
verbose=""
capture=""
fulltrace=""
nomipi=""


for i in "$@"; do
    case $i in
        --seed=*)
            seed="${i#*=}"
            shift
            ;;
        -v)
            verbose="-v"
            shift
            ;;
        -s)
            capture="-s"
            shift
            ;;
        --full-trace)
            fulltrace="--full-trace"
            shift
            ;;
        --nomipi)
            nomipi="--nomipi"
            shift
            ;;
        *)
            ;;
    esac
done


function run_mipi_tests {
    if [ "${nomipi}" == "--nomipi" ]; then
        echo "skipping mipi tests"
    else
        sudo systemctl stop frontend
        sleep 5
        if pidof frontend ; then
            sudo killall frontend
            sleep 3
        fi
        sudo /usr/sbin/frontend -o /run/lumotive/cobra -r 1000 &
        sleep 1

        pytest test_mipi.py --random-order-seed=${seed} ${verbose} ${capture} --integration ${fulltrace} --maxfail=1

        sudo killall frontend
        sleep 3
        rm -f /run/lumotive/cobra*.bin
    fi
}

echo "Using ${verbose} ${capture} ${fulltrace} --integration seed=${seed} "

pushd ~/cobra_system_control/unittests/
sudo mkdir -p -m 777 /run/lumotive
sudo chmod -R 777 /run/lumotive

git lfs pull
rm -f /run/lumotive/cobra*.bin
sudo systemctl stop remote
sudo systemctl stop cb_api

run_mipi_tests

sudo systemctl start frontend
sleep 3
sudo systemctl restart frontend
sleep 3
# Only run a few of the unittests for intake just to make sure things are nominally working
pytest test_sensor_head.py::test_itof_fpga_mipi --random-order-seed=${seed} ${verbose} ${capture} --integration ${fulltrace} 

rm -f /run/lumotive/cobra_*.bin

sudo systemctl restart frontend
sudo systemctl start remote
popd
