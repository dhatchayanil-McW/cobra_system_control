#! /usr/bin/bash

seed=$RANDOM
verbose=""
capture=""
fulltrace=""
integration=""
nomipi=""
nomcs=""
include=""

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
        --nomcs)
            nomcs="--ignore=test_mcs_updater.py"
            shift
            ;;
        --include=*)
            include="${i#*=}"
            shift
            ;;
        *)
            ;;
    esac
done

echo "Using ${verbose} ${capture} ${fulltrace} ${integration} ${NX} seed=${seed} include=${include}"

sudo mkdir -p -m 777 /run/lumotive
sudo chmod -R 777 /run/lumotive

git lfs pull
rm -f /run/lumotive/cobra*.bin
sudo systemctl stop remote
sudo systemctl stop cb_api


sh run_mipi_tests.sh ${integration} ${nomipi} --seed=${seed} -v ${capture} ${fulltrace}

sudo systemctl start frontend
sleep 3
sudo systemctl restart frontend
sleep 3

if [ "${include}" == "" ]; then
    pytest . --ignore=test_mipi.py --ignore=test_rowcal.py $nomcs --random-order-seed=${seed} ${verbose} ${capture} ${integration} ${fulltrace}
else
    pytest ${include} --ignore=test_mipi.py --ignore=test_rowcal.py $nomcs --random-order-seed=${seed} ${verbose} ${capture} ${integration} ${fulltrace}
fi

rm -f /run/lumotive/cobra_*.bin

sudo systemctl restart frontend
sleep 3
sh run_mipi_tests.sh ${integration} ${nomipi} --seed=${seed} ${verbose} ${capture} ${fulltrace}

sudo systemctl start frontend
