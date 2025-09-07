SERVICE_NAME = serial-udp-middleware
SERVICE_FILE = /etc/systemd/system/$(SERVICE_NAME).service

.PHONY: install start stop restart status enable disable clean

install:
	sudo cp $(SERVICE_NAME).service $(SERVICE_FILE)
	sudo systemctl daemon-reload

start:
	sudo systemctl start $(SERVICE_NAME)

stop:
	sudo systemctl stop $(SERVICE_NAME)

restart: stop start

status:
	sudo systemctl status $(SERVICE_NAME)

enable:
	sudo systemctl enable $(SERVICE_NAME)

disable:
	sudo systemctl disable $(SERVICE_NAME)

clean:
	sudo systemctl stop $(SERVICE_NAME)
	sudo systemctl disable $(SERVICE_NAME)
	sudo rm -f $(SERVICE_FILE)
	sudo systemctl daemon-reload
