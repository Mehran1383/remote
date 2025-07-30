#!/usr/bin/env python3

# Copyright 2022-2024 u-blox AG
# SPDX-License-Identifier: Apache-2.0

# Version test_only 20240315_125555

"""
    u-blox PointPerfect NTRIP client

    Run with -h (or --help) to see supported command line arguments

    Typical command line examples:

    Using auto-detected closest mountpoint and default server/port:
    python pointperfect_ntrip_client.py -P <port> -u <user> -p <password>

    Using a specific mountpoint (EU in this case):
    python pointperfect_ntrip_client.py -P <port> -u <user> -p <password> -m EU

    Using a secure (TLS) connection:
    python pointperfect_ntrip_client.py -P <port> -u <user> -p <password> --tls

    <user> and <password> can be found in the u-blox Thingstream portal:
    "Thingstream > Location Services > Location Thing > credentials"

    <port> is the serial port of the u-blox GNSS receiver with SPARTN support,
    e.g. /dev/ttyACM0 or COM3. Optionally with baudrate, e.g. /dev/ttyACM0@115200.
"""

import argparse
import base64
from dataclasses import dataclass
import logging
from math import radians, cos, pi
import re
import sys
import socket
import ssl
from threading import Thread
import time
from typing import NamedTuple

# pip install pyserial
import serial

SOCKET_RECONNECT_DELAY = 10    # seconds of delay before retry after socket error
SERVER_RECONNECT_DELAY = 60    # seconds of delay before retry after server error
SOCKET_TIMEOUT = 5             # socket timeout in seconds
SOCKET_MAX_RECV_TIMEOUTS = 12  # number of timeouts before reconnecting

DEFAULT_NTRIP_SERVER = "ppntrip.services.u-blox.com"
DEFAULT_NTRIP_PORT = 2101
DEFAULT_NTRIP_TLS_PORT = 2102


class MountPointInfo(NamedTuple):
    name: str
    identifier: str
    format: str
    format_details: str
    carrier: str
    nav_system: str
    network: str
    country: str
    lat: float
    lon: float


class NtripClient:
    def __init__(
        self, host, port=DEFAULT_NTRIP_PORT, user=None, password=None, tls=False
    ):
        self.host = host
        self.port = port
        self.user = user
        self.password = password
        self.thread = None
        self.streaming = False
        self.sock = None
        self.tls = tls

    def __make_request(self, mountpoint_name):
        auth_str = base64.b64encode(f"{self.user}:{self.password}".encode()).decode()
        request = (
            f"GET /{mountpoint_name} HTTP/1.1\r\n"
            f"Host: {self.host}:{self.port}\r\n"
            "User-Agent: NTRIP Python Client\r\n"
            "Accept: */*\r\n"
            f"Authorization: Basic {auth_str}\r\n"
            "Connection: close\r\n"
            "\r\n"
        )
        return request.encode()

    def get_mountpoints(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if self.tls:
            logging.info("Using secure connection")
            context = ssl.create_default_context()
            sock = context.wrap_socket(sock, server_hostname=self.host)
        sock.settimeout(5)
        try:
            sock.connect((self.host, self.port))
            sock.send(self.__make_request(""))
            response = sock.recv(1024)
            data = ""
            while response:
                data += response.decode()
                response = sock.recv(1024)
        except socket.timeout:
            return []
        finally:
            if sock:
                sock.close()

        mountpoints = []
        for line in data.splitlines():
            if line.startswith("STR;"):
                cols = line.split(";")
                mountpoint = MountPointInfo(
                    name=cols[1],
                    identifier=cols[2],
                    format=cols[3],
                    format_details=cols[4],
                    carrier=cols[5],
                    nav_system=cols[6],
                    network=cols[7],
                    country=cols[8],
                    lat=float(cols[9]),
                    lon=float(cols[10]),
                )
                mountpoints.append(mountpoint)
                logging.debug("mountpoint %s", mountpoint)
            elif line.startswith("CAS;"):
                logging.debug(line)
        return mountpoints

    def start_stream(self, mountpoint, callback):
        if self.streaming:
            self.stop_stream()
        self.thread = Thread(target=self.stream_data, args=(mountpoint, callback))
        self.thread.start()

    def stop_stream(self):
        self.streaming = False
        if self.thread:
            self.thread.join()

    def send_gga(self, gga):
        if self.sock:
            try:
                self.sock.send(gga.encode())
            except OSError:
                logging.warning("Error sending GGA")
                return False
            return True
        return False

    def stream_data(self, name, callback):
        self.streaming = True
        delay = 0
        while self.streaming:
            try:
                # create a socket connection to the host and port
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                if self.tls:
                    logging.info("Using secure connection")
                    context = ssl.create_default_context()
                    sock = context.wrap_socket(sock, server_hostname=self.host)
                sock.settimeout(SOCKET_TIMEOUT)
                sock.connect((self.host, self.port))

                # send the request
                sock.send(self.__make_request(name))
                # read a chunk of data, expecting to get the status line and headers
                response = sock.recv(1024)

                response_lines = response.split(b"\r\n")
                if len(response_lines) < 2:
                    logging.error("Invalid response from server: %s", response)
                status = response_lines[0].decode().split(" ")
                logging.debug(response_lines[0].decode())
                if status[1] != "200":
                    logging.error("HTTP Error: %s, retrying in %d seconds",
                                response_lines[0].decode(), SERVER_RECONNECT_DELAY)
                    sock.close()
                    time.sleep(SERVER_RECONNECT_DELAY)
                    continue
                for line in range(1, len(response_lines)):
                    if response_lines[line] == b"":
                        # empty line, end of headers
                        # the rest of the response contains data
                        response = b"\r\n".join(response_lines[line + 1 :])
                        break
                    logging.debug(response_lines[line])

                # make the socket available for sending GGA
                self.sock = sock

                if response:
                    callback(response)

                timeouts = 0
                while self.streaming and timeouts < SOCKET_MAX_RECV_TIMEOUTS:
                    try:
                        response = sock.recv(2048)
                        if response:
                            timeouts = 0
                            callback(response)
                        else:
                            delay = SERVER_RECONNECT_DELAY
                            logging.warning(
                                "Connection closed by server, reconnecting in %d seconds", delay
                            )
                            break
                    except TimeoutError:
                        timeouts += 1

            except (socket.herror, socket.gaierror):
                delay = SOCKET_RECONNECT_DELAY
                logging.warning("Error connecting to server %s:%d, retrying in %d seconds",
                                self.host, self.port, delay)
            except TimeoutError:
                delay = SOCKET_RECONNECT_DELAY
                logging.warning("Connection timeout, retrying in %d seconds", delay)

            # close the socket
            self.sock = None
            try:
                sock.close()
            except OSError:
                pass
            time.sleep(delay)


QUALITIES = ("NOFIX", "GNSS", "DGNSS", "PPS", "FIXED", "FLOAT", "DR", "MAN", "SIM")

STATS = 100  # logging level for stats


class NmeaParser:
    """
    Parse NMEA sentences from bytes and invoke callbacks for matching sentences.
    Strips newlines before passing the sentence to the callback. Errors are silently
    ignored and the parser is robust to malformed sentences or UBX, RTCM, SPARTN, etc.
    """

    def __init__(self, callbacks):
        """
        Initialize the parser with a dictionary of callbacks for matching sentences.

        Parameters:
            callbacks (dict): Dictionary of compiled regular expressions objects mapped
                              to callbacks.
        """
        self.callbacks = callbacks
        self.buffer = None

    def parse(self, data):
        """Parse the given bytes and invoke callbacks for matching sentences."""
        for byte in data:
            if byte == ord("$"):
                self.buffer = bytearray([byte])
            elif self.buffer is not None:
                if (
                    byte in range(ord("A"), ord("Z") + 1)
                    or byte in range(ord("0"), ord("9") + 1)
                    or byte in (ord(","), ord("."), ord("-"), ord("*"))
                ):
                    self.buffer.append(byte)
                elif byte == 0x0D:  # CR
                    if len(self.buffer) > 3 and self.buffer[-3] == ord("*"):
                        try:
                            chksum_received = int(self.buffer[-2:], 16)
                        except ValueError:
                            chksum_received = -1  # will never match below
                        chksum = 0
                        for i in self.buffer[1:-3]:
                            chksum ^= i
                        if chksum == chksum_received:
                            for regexp in self.callbacks.keys():
                                if regexp.match(self.buffer):
                                    data = self.buffer.decode(encoding="ascii")
                                    # invoke callback with data
                                    self.callbacks[regexp](data)
                        else:
                            logging.warning(
                                "chksum error: %02x != %02x", chksum_received, chksum
                            )
                    self.buffer = None
                else:
                    self.buffer = None


@dataclass
class Statistics:
    """Accumulator for statistics."""

    # number of epochs for each quality
    epochs: list[int]
    # total number of epochs
    total: int
    # interval for printing statistics
    interval: int


class PointPerfectClient:
    """
    u-blox PointPerfect NTRIP client

    Subscribes to the PointPerfect NTRIP service and sends corrections to
    a u-blox receiver. Monitors the receiver's position in order to connect to
    the appropriate mountpoint.
    """

    EARTH_CIRCUMFERENCE = 6371000 * 2 * pi

    def __init__(
        self,
        gnss,
        ntrip_client,
        mountpoint="",
        gga_interval=0,
        distance=50000,
        epochs=float("inf"),
        ubxfile=None,
        stats_interval=0,
    ):
        self.gnss = gnss
        self.ntrip_client = ntrip_client
        self.distance = distance
        self.epochs = epochs
        self.ubxfile = ubxfile
        self.lat = 0  # lat at which node selection was last performed
        self.lon = 0  # lon at which node selection was last performed
        self.epoch_count = 0  # number of epochs since last node selection
        self.dlat_threshold = distance * 360 / self.EARTH_CIRCUMFERENCE
        self.dlon_threshold = 0  # will be set in process_position()
        self.mountpoints = None  # cached tile data
        self.mountpoint = mountpoint
        self.lastgga = 0
        self.gga_interval = gga_interval

        if stats_interval > 0:
            self.stats = Statistics(
                epochs=[0] * len(QUALITIES), total=0, interval=stats_interval
            )
        else:
            self.stats = None

        handlers = {re.compile(b"^\\$G[A-Z]GGA,"): self.handle_nmea_gga}
        self.nmea_parser = NmeaParser(handlers)

        if self.mountpoint:
            self.ntrip_client.start_stream(self.mountpoint, self.handle_ntrip_data)
        else:
            while not self.mountpoints:
                self.mountpoints = self.ntrip_client.get_mountpoints()
                if not self.mountpoints:
                    logging.warning(
                        "No mountpoints available, retrying in %d seconds", SOCKET_RECONNECT_DELAY
                    )
                    time.sleep(SOCKET_RECONNECT_DELAY)
            if self.mountpoint == "":
                print("Available mountpoints:")
                for point in self.mountpoints:
                    print(f"{point.name:8s}: {point.format}, {point.country},"
                          f"({point.lat:> 7.3f},{point.lon:> 8.3f})")
                sys.exit(0)

    def handle_ntrip_data(self, data):
        """Callback for handling NTRIP data."""
        logging.debug("NTRIP data: %d bytes", len(data))
        # send to receiver as is
        self.gnss.write(data)

    def loop_forever(self):
        """Main loop of the client."""
        try:
            buffer = bytearray(100)
            while True:
                bytes_read = self.gnss.readinto(buffer)
                if bytes_read:
                    if self.ubxfile:
                        self.ubxfile.write(buffer[0:bytes_read])
                    # parse the bytes and invoke matching handlers
                    self.nmea_parser.parse(buffer[0:bytes_read])
        finally:
            self.ntrip_client.stop_stream()

    def handle_nmea_gga(self, sentence):
        """Process an NMEA-GGA sentence passed in as a string."""
        logging.info(sentence)
        fields = sentence.split(",")
        quality = int(fields[6] or 0)
        f_lat = float(fields[2] or 0)
        lat = int(f_lat / 100) + (f_lat % 100) / 60
        if fields[3] == "S":
            lat *= -1
        f_lon = float(fields[4] or 0)
        lon = int(f_lon / 100) + (f_lon % 100) / 60
        if fields[5] == "W":
            lon *= -1

        if self.stats:
            self.stats.epochs[quality] += 1
            self.stats.total += 1
            if self.stats.total % self.stats.interval == 0:
                pct = [
                    f"{QUALITIES[i]}: {self.stats.epochs[i] / self.stats.total * 100:.1f}%"
                    for i in range(len(QUALITIES))
                    if self.stats.epochs[i]
                ]
                logging.log(STATS, ", ".join(pct))

        if quality not in (0, 6):  # no fix or estimated
            self.process_position(lat, lon)
            if self.gga_interval > 0:
                if self.lastgga == 0 or time.time() - self.lastgga > self.gga_interval:
                    if self.ntrip_client.send_gga(sentence + "\r\n"):
                        logging.debug("GGA sent")
                        self.lastgga = time.time()

    def process_position(self, lat, lon):
        """Handle position from the receiver. If needed, select a new mountpoint."""
        self.epoch_count += 1
        # Only record new position if it changed significantly since the last calculation
        if (
            abs(lat - self.lat) > self.dlat_threshold
            or abs(lon - self.lon) > self.dlon_threshold
            or self.epoch_count > self.epochs
        ):
            logging.debug("updating position: %f, %f", lat, lon)
            self.lat = lat
            self.lon = lon
            self.epoch_count = 0
            self.dlon_threshold = self.dlat_threshold * cos(radians(self.lat))
            new_mountpoint = self.get_mountpoint(self.lat, self.lon)
            if new_mountpoint and new_mountpoint.name != self.mountpoint:
                logging.info("Switching mountpoint to %s", new_mountpoint.name)
                logging.debug("%s", new_mountpoint)
                self.ntrip_client.start_stream(
                    new_mountpoint.name, self.handle_ntrip_data
                )
                self.mountpoint = new_mountpoint.name

    def get_mountpoint(self, lat, lon):
        """Select the closest mountpoint to the current position."""
        if not self.mountpoints:
            # not yet ready to select a mountpoint, as we don't have tile data
            return None
        # Rather than calculate distance in meters, calculate a value that grows with
        # the distance, since all we care about is finding the closest.
        # As an approximation, use the sum of the lat and lon difference
        # squared, but scale the latitude difference by cos(lon) to make it the
        # same scale as longitude.
        factor_lon = cos(radians(lat))
        min_dist_scaled = float("inf")
        nearest_mountpoint = None
        # TODO: add protocol filters, e.g. for receivers not supporting SPARTN
        for mountpoint in self.mountpoints:
            # longitude difference is proportional to distance along NS
            # latitude difference is proportional to distance along EW,
            # but scale by cos(lon) to make it the same scale as lon
            dist_scaled = (mountpoint.lat - lat) ** 2 + (
                (mountpoint.lon - lon) * factor_lon
            ) ** 2
            if dist_scaled < min_dist_scaled:
                min_dist_scaled = dist_scaled
                nearest_mountpoint = mountpoint
        logging.debug("Nearest mountpoint: %s", nearest_mountpoint)
        return nearest_mountpoint


def main():
    """Main program."""
    argp = argparse.ArgumentParser()
    argp.add_argument("-V", "--version", action="version", version="BLD23-v1.11.6")
    argp.add_argument(
        "-P",
        "--port",
        required=True,
        help="Serial port[@baudrate] of u-blox GNSS receiver with SPARTN support",
    )

    o_group = argp.add_argument_group("Output options")
    time_stamp = time.strftime("%Y%m%d_%H%M%S")
    o_group.add_argument(
        "--ubx",
        nargs="?",
        type=argparse.FileType("wb"),
        const=f"pointperfect_log_{time_stamp}.ubx",
        help="Write all GNSS receiver output to a UBX file",
    )
    o_group.add_argument(
        "--log",
        nargs="?",
        type=argparse.FileType("w"),
        const=f"pointperfect_log_{time_stamp}.txt",
        help="Write all program output to a text file in addition to stdout",
    )
    o_group.add_argument(
        "--stats",
        type=int,
        nargs="?",
        const=5,
        default=0,
        help="Print statistics every N epochs (default: off, 5 if no argument given)",
    )
    o_group.add_argument(
        "--trace",
        choices=("CRITICAL", "ERROR", "WARNING", "INFO", "DEBUG"),
        default="INFO",
        help="Trace level: CRITICAL, ERROR, WARNING, INFO, DEBUG (default: INFO)",
    )

    cgroup = argp.add_argument_group(
        "NTRIP settings", description="Options controlling the NTRIP connection"
    )
    cgroup.add_argument(
        "-s",
        "--server",
        default=argparse.SUPPRESS,
        help="NTRIP server address, with optional port " +
             f"(default: {DEFAULT_NTRIP_SERVER}:{DEFAULT_NTRIP_PORT}",
    )
    cgroup.add_argument("-u", "--user", required=True, help="NTRIP user name")
    cgroup.add_argument("-p", "--password", required=True, help="NTRIP password")
    cgroup.add_argument(
        "-t", "--tls", action="store_true", help="Use secure (TLS) connection"
    )
    cgroup.add_argument(
        "-m",
        "--mountpoint",
        nargs="?",
        default=None,
        const="",
        help="Specify NTRIP mountpoint or use without argument to list available mountpoints",
    )
    cgroup.add_argument(
        "-g",
        "--ggainterval",
        default=0,
        type=int,
        help="GGA interval in seconds (default: 0, no GGA sent to server)",
    )

    lgroup = argp.add_argument_group(
        "Localization options",
        description="Options controlling selection of the mountpoint",
    )
    lgroup.add_argument(
        "--distance",
        default=50000,
        type=int,
        help="The distance threshold [m] for recalculating tile and node (default: 50000)",
    )
    lgroup.add_argument(
        "--epochs",
        default=float("inf"),
        type=float,
        help="The maximum number of epochs between recalculating tile and node (default: infinite)",
    )
    args = argp.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.trace),
        format="%(levelname)s %(message)s",
        stream=sys.stdout,
    )
    if args.log:
        logging.getLogger().addHandler(logging.FileHandler(args.log.name))
    logging.info(" ".join(sys.argv))  # log the command line arguments
    logging.addLevelName(STATS, "STATS")

    default_port = DEFAULT_NTRIP_TLS_PORT if args.tls else DEFAULT_NTRIP_PORT
    if not hasattr(args, "server"):
        ntrip_server = DEFAULT_NTRIP_SERVER
        ntrip_port = default_port
    else:
        ntrip_addr = args.server.split(":")
        if len(ntrip_addr) == 2:
            (ntrip_server, ntrip_port) = (ntrip_addr[0], int(ntrip_addr[1]))
        elif len(ntrip_addr) > 2:
            argp.error("Invalid server address")
        else:
            (ntrip_server, ntrip_port) = (ntrip_addr[0], default_port)
    ntrip_client = NtripClient(
        ntrip_server, ntrip_port, args.user, args.password, args.tls
    )

    if args.ubx:
        logging.info("Writing all receiver data to %s", args.ubx.name)

    serial_params = args.port.split("@")  # split optional baudrate from port argument
    if len(serial_params) == 2:
        (port, baud) = (serial_params[0], int(serial_params[1]))
    else:
        (port, baud) = (serial_params[0], 115200)
    gnss = serial.Serial(port=port, baudrate=baud, timeout=0.1)

    try:
        pp_client = PointPerfectClient(
            gnss,
            ntrip_client,
            mountpoint=args.mountpoint,
            gga_interval=args.ggainterval,
            distance=args.distance,
            epochs=args.epochs,
            ubxfile=args.ubx,
            stats_interval=args.stats,
        )
        pp_client.loop_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if args.ubx:
            args.ubx.close()
        gnss.close()


if __name__ == "__main__":
    main()
