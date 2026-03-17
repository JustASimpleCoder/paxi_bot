#!/usr/bin/bash





# On my machine Product ID is 23a3 and vendor Id is 067b -> check with lusb and looking for 
# "prolific usb" and check the ID xxxx:xxxx part, e.g. 
# $ lsusb
# Bus 002 Device 002: ID 0bda:0420 Realtek Semiconductor Corp. 4-Port USB 3.0 Hub
# Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
# Bus 001 Device 006: ID fccf:f100 SLAMTEC SLAMWARELC
# Bus 001 Device 005: ID 10c4:ea60 Silicon Labs CP210x UART Bridge
# Bus 001 Device 004: ID 067b:23a3 Prolific Technology, Inc. USB-Serial Controller 
# Bus 001 Device 003: ID 0bda:5420 Realtek Semiconductor Corp. 4-Port USB 2.0 Hub
# Bus 001 Device 002: ID 13d3:3549 IMC Networks Bluetooth Radio
# Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub


# Subsystem neets to ge tty for serial port class to get terminal attributes

VENDOR_ID="067b"
PRODUCT_ID="23a3"

print_help() {
  echo "Usage:"
  echo "  $0                          Use default Vendor/Product IDs"
  echo "  $0 <VendorID> <ProductID>   Use custom IDs (4-digit hex)"
}



OPTIND=1

while getopts "h" opt; do
  case "$opt" in 
  h)
    print_help
    exit 0
    ;;
  *)
    #echo "Flag not recognized use [-h] for more info"
    echo "try $0 -h"
    exit 1
    ;;
  esac
done

if ! [[ "$#" -eq 0 || "$#" -eq 2 ]]; then 
  echo "Invalid arguements given, please pass no arguement or two arguemnets, see option [-h] for help"
  exit 1
fi


shift $((OPTIND - 1))

if [[ "$#" -eq 2 ]]; then 
  VENDOR_ID=$1
  PRODUCT_ID=$2

  if ! [[ "$PRODUCT_ID" =~ ^[0-9a-fA-F]{4}$ && "$VENDOR_ID" =~ ^[0-9a-fA-F]{4}$ ]]; then
    echo "Error: IDs must be 4-digit hexadecimal values."
    exit 1
  fi
fi


echo "Adding udev rule for Vendor ID [$VENDOR_ID] and Product ID [$VENDOR_ID]"

UDEV_RULE="SUBSYSTEM==\"tty\", ATTRS{idProduct}==\"${PRODUCT_ID}\", ATTRS{idVendor}==\"${VENDOR_ID}\", GROUP=\"users\", MODE=\"0666\", SYMLINK+=\"hoverboard_serial\""
sudo echo "$UDEV_RULE" | sudo tee /etc/udev/rules.d/99-${VENDOR_ID}-${PRODUCT_ID}.rules

sudo udevadm control --reload
sudo udevadm trigger

echo "Udev rule installed sucessfully!\n". 
echo " - Check if you can find the link with symbolic name: ls /dev/hoverboard_serial, should return /dev/hoverboard_serial"
echo " - Check if rule was added: ls /etc/udev/rules.d/, should find 99-${PRODUCT_ID}-${VENDOR_ID}.rules"


