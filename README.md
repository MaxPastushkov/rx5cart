# RX5Cart
A replacement for the Yamaha RAM4 cartridge, based on [GliGli's RX5USB](https://github.com/gligli/rx5usb)

**WARNING: This project is currently incomplete. Do not use the board yet! The PCB and firmware have not been tested!**

* `board_sch` - board and schematic files for KiCad
* `rx5cart.ino` - firmware for the teensy
* `transfer.py` - software for interfacing to the RX5Cart

### Loading the firmware ###
You will need Teensyduino. Download and instructions are [here](https://www.pjrc.com/teensy/td_download.html).

Open the `rx5cart.ino` file in the Arduino software, plug in the teensy, and press Upload.

### Using the transfer software ###
* You will need Python 3 and the `pyserial` package.
* To read RAM into a file, use `python transfer.py -r file.bin`
* To write to RAM from a file, use `python transfer.py -w file.bin`
* By default it uses the device `/dev/ttyACM0`. You can use the `-d` flag to change it.
