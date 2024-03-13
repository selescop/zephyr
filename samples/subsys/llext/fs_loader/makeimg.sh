#!/bin/sh

# Create a FAT12 disk image to flash in memory
# This tool requires the following to be available on the host system:
#
# - dosfstools
# - mtools

OUTPUT=$1
INPUTS=${@:2}
LOGICAL_SECTOR_SIZE=512
DISK_SIZE_KB=128

if [ $# -lt 2 ]; then
    echo -e "Not enough arguments.\nUsage:\n\t`basename $0` output.img input1 [input2] [...]"
    exit 1
fi

echo -ne "Creating empty '`basename $OUTPUT`' image..."
dd if=/dev/zero of="$OUTPUT" bs=1k count=${DISK_SIZE_KB} status=none
if [ $? -ne 0 ]; then
    echo "failed."
    exit 1
fi
echo -ne "done\nCreating FAT partition image..."
mkfs.fat -F12 -S"$LOGICAL_SECTOR_SIZE" "$OUTPUT" >/dev/null
if [ $? -ne 0 ]; then
    echo "failed."
    exit 1
fi
echo -ne "done\nCopying input files..."
mcopy -i "$OUTPUT" $INPUTS "::/"
if [ $? -ne 0 ]; then
    echo "failed."
    exit 1
fi
echo "done"
