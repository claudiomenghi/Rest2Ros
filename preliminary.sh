sudo groupadd docker
sudo gpasswd -a $USER docker
sudo service docker stop
sudo usermod -a -G docker $USER
sudo reboot
