#!/bin/sh
printf "lxd:$(id -u):1\nroot:$(id -u):1\n" | sudo tee -a /etc/subuid
printf "lxd:$(id -g):1\nroot:$(id -g):1\n" | sudo tee -a /etc/subgid
sudo systemctl restart lxc
