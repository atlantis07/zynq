compile linux kernel
1. Get source code
git clone http://192.168.1.79/platform/linux.git
cd linux

2. Setup some parameters
export CROSS_COMPILE=arm-linux-gnueabihf-
export ARCH=arm

3. checkout the tagged version for Vivado 2015.4
git checkout xilinx-v2015.4

4. Configure the kernel options
make xilinx_zynq_defconfig

5. Configure the kernel options by menuconfig

make menuconfig

1.Set the drivers space size
	Devices Drivers --> Generic Driver Options --> Size in Mega Bytes = 256

2. Kernel Configuration iic Options for Driver
	Devices Drivers --> I2C support --> I2C Hardware Bus support --> Xilinx I2C Controller = *

3. Kernel Configuration ATHEROS AR8035 Options for Driver
	Devices Drivers --> Network device support --> PHY Device support and infrastructure --> Drivers for Atheros AT803X PHYs = *
	Devices Drivers --> Network device support --> Ethernet driver support --> Atheros devices = *
	Devices Drivers --> Network device support --> Ethernet driver support --> Atheros L2 Fast Ethernet support = *
	Devices Drivers --> Network device support --> Ethernet driver support --> Atheros/Attansic L1 Gigabit Ethernet support = *
	Devices Drivers --> Network device support --> Ethernet driver support --> Atheros L1C Gigabit Ethernet support = *

4. Kernel Configuration usb serial for Driver  
	Device Drivers --> USB support --> USB Modem (CDC ACM) support = *
	Device Drivers --> USB support --> USB Serial Converter support --> USB FTDI Single Port Serial Driver = *

5.support UVC
	Device Drivers  --->
		<*> Multimedia support  --->
			[*] Media USB Adapters  --->
				<*> USB Video Class (UVC)
				[*] UVC input events device support (NEW)	


6. Build the kernel image
make uImage LOADADDR=0X00008000 -j100

7. Copy uImage to somewhere 
cp arch/arm/boot/uImage /opt/BOOT/


if excute make menuconfig error
sudo apt-get install libncurses5-dev

if note that not hanve mkimage,location need mkimage tool
sudo apt-get install u-boot-tools

