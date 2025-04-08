#! /usr/bin/bash

seed=$RANDOM
verbose=""
capture=""
fulltrace=""
integration=""
nomipi=""


for i in "$@"; do
    case $i in
        --seed=*)
            seed="${i#*=}"
            shift
            ;;
        -v)
            verbose="-vv"
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
        --integration)
            integration="--integration"
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
    elif [ "${integration}" == "--integration" ]; then
        sudo systemctl stop frontend
        sleep 5
        if pidof frontend ; then
            sudo killall frontend
            sleep 3
        fi
        sudo /usr/sbin/frontend -o /run/lumotive/cobra -r 1000 &
        sleep 5
        fectrl -d 1
        sleep 1
        rm -f /run/lumotive/cobra*.bin
        pytest test_mipi.py --random-order-seed=${seed} ${verbose} ${capture} ${integration} ${fulltrace}
        rm -f /run/lumotive/cobra*.bin
        sudo killall frontend
        sleep 3
        rm -f /run/lumotive/cobra*.bin
    else
        echo "skipping test_mipi since only mocked tests remove this in run_clean_pytest if you add mocked tests to test_mipi"
    fi
}

echo "Using ${verbose} ${capture} ${fulltrace} ${integration} ${NX} seed=${seed} include=${include}"

sudo mkdir -p -m 777 /run/lumotive
sudo chmod -R 777 /run/lumotive

git lfs pull
rm -f /run/lumotive/cobra*.bin
sudo systemctl stop remote
sudo systemctl stop cb_api

run_mipi_tests

sudo systemctl restart frontend
rm -f /run/lumotive/cobra_*.bin
sleep 3
