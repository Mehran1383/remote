#include "GpsSerialHandler.h"
#include <QDebug>
#include <QSerialPort>
#include <QCoreApplication>
#include <QDir>


GpsSerialHandler::GpsSerialHandler(QObject *parent, const QString &portName):
    QThread(parent)
    ,m_targetPortName(portName)
{
    udpManager = new UdpThreadManager(this);
    serialPort = new QSerialPort(this);

    connect(serialPort, &QSerialPort::readyRead, this, &GpsSerialHandler::readData);
    connect(serialPort, &QSerialPort::errorOccurred, this, &GpsSerialHandler::handleError);

    serialPortTimer = new QTimer(this);
    connect(serialPortTimer, &QTimer::timeout, this, &GpsSerialHandler::tryReconnect);
    serialPortTimer->setInterval(5000);
    serialPortTimer->start();

    sampleRateTimer = new QTimer(this);
    connect(sampleRateTimer, &QTimer::timeout, this, &GpsSerialHandler::updateSampleRate);
    sampleRateTimer->setInterval(1000);
    sampleRateTimer->start();

    const QString settingsPath =
        QDir(QCoreApplication::applicationDirPath()).filePath("settings.ini");

    QSettings setting(settingsPath, QSettings::IniFormat);
    usingRtk = setting.value("GPS/serial/RTK/use_rtk", false).toBool();
    if (usingRtk) {
        client = new NtripClient(this);
        connect(client, &NtripClient::streamBytes, this, &GpsSerialHandler::onNtripBytes);
        if (client->setZtpParameters(setting.value("GPS/serial/RTK/ztp_token").toString(),
                                     setting.value("GPS/serial/RTK/thing_name").toString(),
                                     setting.value("GPS/serial/RTK/product-id").toString())) {

            client->startStream();
        } else {
            qWarning() << "Problem with ZTP parameters";
            usingRtk = false;
        }
    }
}

GpsSerialHandler::~GpsSerialHandler()
{
    if(serialPort)
    {
        if(serialPort->isOpen())
        {
            serialPort->close();
            emit Info_ComPort("Close Port");
        }
        delete serialPort;
        serialPort = nullptr;
    }

    if(serialPortTimer)
    {
        serialPortTimer->stop();
        delete serialPortTimer;
        serialPortTimer = nullptr;
    }

    if (client != nullptr)
        delete client;
}

void GpsSerialHandler::readData()
{
    QByteArray data = serialPort->readAll();

    QList<QByteArray> lines = data.split('\n');
    for (const QByteArray &line : lines){
        parseLine(line);
        samples++;
    }
}

void GpsSerialHandler::onNtripBytes(const QByteArray& bytes)
{
    if (serialPort->isOpen())
        serialPort->write(bytes);
}

void GpsSerialHandler::handleError(QSerialPort::SerialPortError error)
{
    // if (error == QSerialPort::ResourceError || error == QSerialPort::OpenError || error == QSerialPort::NotOpenError)
    if (error == QSerialPort::NoError) {
        return; // No action needed if there's no error
    }

    serialPortTimer->stop();
    qDebug() << "Serial port error: " << serialPort->errorString();
    emit Info_ComPort("Close Port");
    if (serialPort->isOpen()){
        serialPort->close();
    }

    delete serialPort;
    serialPort = nullptr;
    serialPort = new QSerialPort(this);
    connect(serialPort, &QSerialPort::readyRead, this, &GpsSerialHandler::readData);
    connect(serialPort, &QSerialPort::errorOccurred, this, &GpsSerialHandler::handleError);

    QThread::sleep(1);

    tryReconnect();
    serialPortTimer->start();
}

void GpsSerialHandler::tryReconnect()
{
    if (!serialPort->isOpen())
    {
        qDebug() << "Attempting to reconnect to GPS...";
        emit Info_ComPort("Close Port");
        tryConnect();
    }
    else{
        emit Info_ComPort(serialPort->portName());
    }
}

void GpsSerialHandler::tryConnect()
{
    if (serialPort->isOpen())
        return;

    if (m_targetPortName.isEmpty()) {
        qWarning() << "No target port name specified for GPS handler.";
        emit Info_ComPort("Port Not Set");
        return;
    }

    qDebug() << "Trying to connect to port:" << m_targetPortName;

    // Configure the serial port
    serialPort->setPortName(m_targetPortName);
    serialPort->setBaudRate(115200);
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);

    if (serialPort->open(QIODevice::ReadWrite))
    {
        qDebug() << "Connected to serial port:" << serialPort->portName();
        emit Info_ComPort(serialPort->portName());
    }
    else
    {
        qWarning() << "Failed to connect to serial port:"
                   << serialPort->errorString();
        emit Info_ComPort("Error");
        QThread::msleep(1000);
    }
}

void GpsSerialHandler::parseLine(const QString &line)
{
    static double lastSpeed = 0.0;
    static double lastLat = 0.0;
    static double lastLon = 0.0;
    static int lastFixQuality = 0;
    static int lastNumSats = 0;
    static double lastHdop = 99.0;

    if (line.startsWith("$GPRMC") || line.startsWith("$GNRMC")) {
        auto [speed, _] = processRMC(line);
        lastSpeed = speed;
    } else if (line.startsWith("$GPGGA") || line.startsWith("$GNGGA")) {
        auto [lat, lon, fixQuality, numSats, hdop] = processGGA(line);

        if (usingRtk) {
            if (fixQuality != 0 && fixQuality != 6) { // no fix or estimated
                if (!lastGgaTimer.isValid() || lastGgaTimer.elapsed() > ggaIntervalSec * 1000) {
                    QByteArray out = line.toUtf8();
                    if (client->sendGGA(out)) { // Send GGA to NTRIP server for keeping connection
                        qDebug() << "Sent GGA to NTRIP server";
                        lastGgaTimer.restart();
                    }
                }
            }
        }

        lastLat = lat;
        lastLon = lon;
        lastFixQuality = fixQuality;
        lastNumSats = numSats;
        lastHdop = hdop;

        QJsonObject json;
        json["latitude"] = lastLat;
        json["longitude"] = lastLon;
        json["speed_kmh"] = lastSpeed;
        json["fix_quality"] = lastFixQuality;
        json["num_satellites"] = lastNumSats;
        json["hdop"] = lastHdop;
        json["timestamp"] = (QTime::currentTime().second() * 1000) + QTime::currentTime().msec();
        json["eph"] = 0.056;
        json["epv"] = 0.043;
        json["heading_motion_valid"] = true;
        json["heading_motion"] = 252.43755;
        json["heading_vehicle_valid"] = true;
        json["heading_vehicle"] = 252.43753;

        QByteArray datagram = QJsonDocument(json).toJson(QJsonDocument::Compact);

        // Send the datagram through UDP
        QSettings settings("settings.ini", QSettings::IniFormat);
        udpManager->send(datagram, QHostAddress("127.0.0.1"), settings.value("GPS/UDP/port", 30000).toInt());
    }
}

std::tuple<double, bool> GpsSerialHandler::processRMC(const QString &line)
{
    QStringList parts = line.split(',');
    if (parts.size() < 8) return {0.0, false};

    QString speed = parts[7];
    double speedKnots = speed.toDouble();
    double speedKmh = speedKnots * 1.852;

    return {speedKmh, true};
}

std::tuple<double, double, int, int, double> GpsSerialHandler::processGGA(const QString &line)
{
    QStringList parts = line.split(',');
    if (parts.size() < 9) return {-1.0, -1.0, 0, 0, 99.0};

    double lat = convertToDecimal(parts[2], parts[3], true);
    double lon = convertToDecimal(parts[4], parts[5], false);
    int fixQuality = parts[6].toInt();
    int numSats = parts[7].toInt();
    double hdop = parts[8].toDouble();

    return {lat, lon, fixQuality, numSats, hdop};
}

double GpsSerialHandler::convertToDecimal(const QString &coordinate, const QString &direction, bool isLatitude)
{
    int degreeLength = isLatitude ? 2 : 3;
    bool ok1, ok2;
    double degrees = coordinate.left(degreeLength).toDouble(&ok1);
    double minutes = coordinate.mid(degreeLength).toDouble(&ok2);

    if (!ok1 || !ok2) {
        qWarning() << "Invalid coordinate string format.";
        return 0.0; // Return a default value or handle error appropriately
    }

    double decimal = degrees + (minutes / 60.0);
    if (direction == "S" || direction == "W") {
        decimal = -decimal;
    }
    return decimal;
}

void GpsSerialHandler::updateSampleRate()
{
    emit Info_SampleRate(QString::number(samples));
    samples = 0;
}
