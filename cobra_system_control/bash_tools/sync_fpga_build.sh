#
# file: sync_fpga_build.sh
#
# Copyright (C) 2023-2024 Lumotive, Inc. All rights reserved.
#
# Syncs FPGA collateral to the system control software.
# Copies MCS from FPGA build folder to system control repo
# Copies yaml memory maps from FPGA build folder to system control repo
# Generates BIN version from MCS
# Gets MD5 sums for the MCS and the BIN
# Writes MD5 sums to python file for use by system control software.
#
# Warning, the FPGA Golden Sha is current hardcoded. This will need to
# be updated if the golden image is changed in the jump partition.
#
#

MD5FILEBASE="fpga_collateral_md5.py"

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
    echo "usage -- . ./sync_fpga_build.sh <path_to_fpga_build_dir>"
    echo "           You must run this script from the bash_scripts directory"
    echo "           The fpga build directory name must start with the project name, e.g., \"m30\""
    return 22
fi

if [ "$#" -gt 1 ]; then
    echo "warning -- Check for spaces in your input directory and escape them as needed"
    return 22
fi

if ! which hex2bin.py >/dev/null; then
    echo "intelhex not installed, please install with: pip install intelhex"
    return 22
fi

# Check the input directory name and formattigng
inputbuilddir="$1"
# is there a slash at the end of the dir string?
if [ "x${inputbuilddir##*/}" == "x" ]; then
    len="${#inputbuilddir}"
    inputbuilddir="${inputbuilddir::$len-1}"
fi

# Check where we're running this from since it uses PWD calls.
startingdir="$PWD"
if [ "${startingdir##*/}" != "bash_tools" ]; then
    echo "Please run this script from inside the bash_tools directory"
    return 1
fi

# Get more folder information
cobradir=$(dirname "$PWD")
resourcesdir="${cobradir}/resources"


# Was the top level or impl or yaml directory provided?
if [ "${inputbuilddir##*/}" == "impl_1" ]; then
    basedir=$(dirname "$inputbuilddir")
elif [ "${inputbuilddir##*/}" == "yaml" ]; then
    basedir=$(dirname "$inputbuilddir")
else
    basedir="$inputbuilddir"
fi

# Get the project name from the file name of the fpga mcs
foldername="${basedir##*/}"
projectname="${foldername::3}"


# Start the file for the md5 information of the artifact
md5file="${cobradir}/${projectname}_${MD5FILEBASE}"
rm -f "$md5file"

cat > "$md5file" <<EOF
"""Constants related to FPGA build artifacts
"""
EOF

# Write the fpga git sha to the md5file
verdatfile="${basedir}/${projectname}_fpga.ver.dat"
gsha=$(cat "${verdatfile}" | sed 's/.*sha://' | sed 's/,.*//')
echo "FPGA_RELEASED_SHA = 0x${gsha}" >> "$md5file"

# TODO currently hardcoded.
echo "FPGA_GOLDEN_SHA = 0x7d4f5353" >> "$md5file"

artifactlist=(
    fpga_dual_boot.mcs
    fpga_map.yml
    spi_flash_map.yml
)

subfolderlist=(
    impl_1
    yaml
    yaml
)

# Copy over the MCS as binary and as MCS
mcsdir="${basedir}/impl_1"
if [ -d "$mcsdir" ]; then
    pushd "$mcsdir"

    fname="${projectname}_fpga_dual_boot"
    if [ -f "${fname}.mcs" ]; then
        echo "Generating binary file from MCS file"
        hex2bin.py --pad FF --range 0: ${fname}.mcs ${fname}.bin
        if [ $? == 1 ]; then
            echo "Failed to generate binary from MCS file"
            return 2
        fi

        mcs_md5=($(md5sum "${fname}.mcs"))
        bin_md5=($(md5sum "${fname}.bin"))
        echo "MCS_MD5 = \"${mcs_md5}\"" >> "$md5file"
        echo "BIN_MD5 = \"${bin_md5}\"" >> "$md5file"
        cp "${fname}.mcs" "${fname}.bin" "$resourcesdir"
        echo "${fname}.mcs ${fname}.bin copied"

        # Assert the copy worked by comparing md5
        cpmd5=($(md5sum "${resourcesdir}/${fname}.mcs"))
        if [ $mcs_md5 != $cpmd5 ]; then
            echo "Copied MCS file md5 does not match copied md5!"
            return 5
        fi

        cpmd5=($(md5sum "${resourcesdir}/${fname}.bin"))
        if [ $bin_md5 != $cpmd5 ]; then
            echo "Copied bin file md5 does not match copied md5!"
            return 5
        fi
    else
        echo "mcs file not found"
        return 2
    fi
    safepopd
else
    echo "impl_1 folder not found"
    return 2
fi


# Copy over the YAMLs
yamldir="${basedir}/yaml"
if [ -d "$yamldir" ]; then
    pushd "$yamldir"
    # Copy all the yamls
    cp -R . "$resourcesdir"
    echo "Yamls copied"
else
    echo "yaml folder not found"
    return 2
fi
safepopd

# Cat together the spi flash info
pushd "$resourcesdir"
mv "${projectname}_spi_flash_map.yml" temp.yml
cat temp.yml "${projectname}_spi_flash_extras.yml" >> "${projectname}_spi_flash_map.yml"
rm temp.yml

# Replace the metadata buffer info with the M30 specific data
cp metadata_map.yml ./periphs/*meta_ram.yml

safepopd

return 0
