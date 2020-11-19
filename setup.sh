# To enable systemd boot start-up.
sudo systemctl enable systemd-networkd

# To start wpa_supplicant@wlan0 service
sudo systemctl start wpa_supplicant@wlan0.service

# Debug
# To stop all the current wpa_supplicant processes.
sudo killall wpa_supplicant
