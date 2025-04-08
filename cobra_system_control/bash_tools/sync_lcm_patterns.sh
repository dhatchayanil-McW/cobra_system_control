#
# file: sync_lcm_patterns.sh
#
# Copyright (C) 2024 Lumotive, Inc. All rights reserved.
#
# Syncs new LCM pattern MCS to the system control software.
# Copies MCS to system control repo
# Creates BIN from MCS
# Gets MD5 sums for the MCS and the BIN
# Writes MD5 sums to python file for use by system control software.
#

MD5FILEBASE="lcm_collateral_md5.py"

safepopd () {
    # check the directory stack and popd
    if [ $(dirs -p | wc -l) -gt 1 ]; then
        popd "$@" > /dev/null
    fi
}

pushd () {
    command pushd "$@" > /dev/null
}

if [ "x${1}" == "x" ]; then
    echo "usage -- . ./sync_lcm_patterns.sh <path_to_lcm_pattern_mcs>"
    echo "           You must run this script from the bash_scripts directory"
    return 22
fi

if [ "$#" -gt 1 ]; then
    echo "warning -- Check for spaces in your input directory and escape them as needed"
    return 22
fi

if ! which hex2bin.py >/dev/null; then
    echo "intelhex not installed, please install with: pip install intelhex"
    exit 22
fi

# Check the input path
inputfile="$1"
# is the path to the MCS or a folder?
if [ -d "${inputfile}" ]; then
    echo "Please provide the path to the MCS file, not the directory"
    exit 1
fi

# Get the filename and directory
if [ -f "${inputfile}" ]; then
    basedir=$(dirname "$inputfile")
    basename=$(basename "$inputfile")
    echo $basedir
    echo $basename
else
    echo "${inputfile} was not determined to be a file. Please check"
    exit 1
fi

# Check where we're running this from since it uses PWD calls.
startingdir="$PWD"
if [ "${startingdir##*/}" != "bash_tools" ]; then
    echo "Please run this script from inside the bash_tools directory"
    exit 1
fi

# Get more folder information
cobradir=$(dirname "$PWD")
resourcesdir="${cobradir}/resources"

# Start the file for the md5 information of the artifact
md5file="${cobradir}/${MD5FILEBASE}"
rm -f "$md5file"

cat > "$md5file" <<EOF
"""Constants related to LCM pattern artifacts
"""
EOF

artifactlist=(
    fpga_dual_boot.mcs
)

# Copy over the MCS as binary and as MCS
if [ -d "$basedir" ]; then
    pushd "$basedir"
    if [ -f "$basename" ]; then
        pattern_base_name="lm10_voltage_patterns"

        echo "Moving MCS to correct folder"
        cp "$inputfile" "${resourcesdir}/${pattern_base_name}.mcs"
        echo "${pattern_base_name}.mcs copied"

        pushd "$resourcesdir"
        echo $PWD

        echo "Generating binary file from MCS file"
        hex2bin.py --pad FF --range 0: ${pattern_base_name}.mcs ${pattern_base_name}.bin

        if [ $? == 1 ]; then
            echo "Failed to generate binary from MCS file"
            exit 2
        fi

        mcs_md5=($(md5sum "${pattern_base_name}.mcs"))
        bin_md5=($(md5sum "${pattern_base_name}.bin"))
        echo "MCS_MD5 = \"${mcs_md5}\"" >> "$md5file"
        echo "BIN_MD5 = \"${bin_md5}\"" >> "$md5file"
        echo "Copied MD5 sums to ${md5file}"

    else
        echo "pattern file not found"
        exit 2
    fi
    safepopd
else
    echo "base directory folder not found"
    exit 2
fi

safepopd

return 0
