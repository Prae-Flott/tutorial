# How to clone SD Cards so that ros2 still works
- Create a image and copy it to the new card or use clone directly
- Edit the /etc/hostname file: sudo nano /etc/hostname

- Replace the current hostname with the new one.

-Edit the /etc/hosts file: sudo nano /etc/hosts

- Update the line that references the old hostname to the new one. For example: 127.0.1.1    oldhostname

- Change oldhostname to your new hostname.

- Reboot the system: sudo reboot
- Then you have to rebuild all ros packages
