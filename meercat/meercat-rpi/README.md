# meercat-rpi

This is the repository hosting the Raspberry Pi code for the Meercat module to be deployed with the capstone project. Follow the directions contained within to get the project building on a Raspberry Pi Zero 2W.

## Setup

1. Install Ubuntu 2022 onto the Raspberry Pi
2. Upon first setup, complete the following steps to setup a static IP on the Pi (convenient for development over SSH)
   1. Add (or create the file if not present) `network: {config: disabled}` to `/etc/cloud/cloud.cfg.d/99-disable-network-config.cfg` to disable automatic overwriting of the network configuration upon boot
   2. Modify `/etc/netplan/50-cloud-init.yaml` to add network configuration code. The following is provided as an example.

```yaml
network:
    version: 2
    wifis:
        renderer: networkd
        wlan0:
            access-points:
                wifi-name:
                    password: <base64-encoded-password>
            addresses:
              - 10.0.0.57/24
            routes:
              - to: default
                via: 10.0.0.1
            nameservers:
              addresses: [1.1.1.1, 8.8.8.8, 4.4.4.4]
            dhcp4: false
            optional: true
```
3. (Optional) Setup auto verification of SSH to allow use of VS Code with the Raspberry Pi acting as the remote hos or to genreally make SSH development work more convenient. Run the command `ssh-copy-id -i ~/.ssh/id_ed25519.pub user@host` but replace the appropriate public key and user/host of the Pi.
4. Update I2C on the Pi to use a clock rate of 400 kHz by adding `dtparam=i2c_arm_baudrate=400000` to `/boot/config.txt`. Reboot system for changes to take effect.
5. Install the following dependencies on the Pi

```bash
sudo apt-get install -y python3-pip
```

## Troubleshooting/Warnings

If you see the warning `LC_ALL: cannot change locale (en_US.UTF-8)` when running the deploy script, run the following on the Pi to fix the issue: `sudo locale-gen en_US.UTF-8`



apt install `python3.11` `python3.11-venv`
use `python3.11 -m venv venv`

```
Match User sastreet
  AcceptEnv VENV_PATH
```

sudo apt-get install -y python3-pip python3-venv python3.11-dev

```
dtparam=i2c_arm=on

[cm5]
dtoverlay=dwc2,dr_mode=host

[all]
enable_uart=1
dtoverlay=i2c-gpio,bus=3
```

install protobuf-compiler