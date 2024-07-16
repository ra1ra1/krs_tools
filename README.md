# Setting
'''
sudo su
modprobe ftdi-sio
echo 165C 0008 > /sys/bus/usb-serial/drivers/ftdi_sio/new_id
exit
'''

# References
https://kondo-robot.com/faq/usb_adapter_for_linux_2019
https://kondo-robot.com/faq/krs_tutorial_py1
