#!/bin/sh

gcc -o serialrelay serialrelay.c
sudo rm /usr/local/bin/serialrelay
sudo cp serialrelay /usr/local/bin/
sudo chmod u+s /usr/local/bin/serialrelay  # necessary to allow chroot. internal setuid drops process priv to nobody after chroot is done.
