#include "ReceiverSerialHandler.h"

ReceiverSerialHandler::ReceiverSerialHandler(QObject *parent, const QString &portName):
    QThread(parent)
    ,m_targetPortName(portName)
{
    udpManager = new UdpThreadManager(this);

    serialPort = new QSerialPort(this);

    connect(serialPort, &QSerialPort::readyRead, this, &ReceiverSerialHandler::readData);
    connect(serialPort, &QSerialPort::errorOccurred, this, &ReceiverSerialHandler::handleError);

    serialPortTimer = new QTimer(this);
    connect(serialPortTimer, &QTimer::timeout, this, &ReceiverSerialHandler::tryReconnect);
    serialPortTimer->setInterval(5000);
    serialPortTimer->start();

    sampleRateTimer = new QTimer(this);
    connect(sampleRateTimer, &QTimer::timeout, this, &ReceiverSerialHandler::updateSampleRate);
    sampleRateTimer->setInterval(1000);
    sampleRateTimer->start();
}

void ReceiverSerialHandler::readData()
{
    QByteArray data = serialPort->readAll();
    if(data.size() >= 37){
        separator_data(data);
        samples++;
    } else {
        serialPort->clear();
    }
}

void ReceiverSerialHandler::tryReconnect()
{
    if (serialPort->isOpen())
    {
        QByteArray testCommand = "PING";  // A small harmless test command
        qint64 bytesWritten = serialPort->write(testCommand);

        if (bytesWritten == -1)
        {
            qDebug() << "Failed to write to port, possibly disconnected.";
            serialPort->close();
        }
    }

    if (!serialPort->isOpen())
    {
        qDebug() << "Attempting to reconnect...";
        emit Info_ComPort("Close Port");
        tryConnect();
    }
    else{
        emit Info_ComPort(serialPort->portName());
    }
}

void ReceiverSerialHandler::handleError(QSerialPort::SerialPortError error)
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
    connect(serialPort, &QSerialPort::readyRead, this, &ReceiverSerialHandler::readData);
    connect(serialPort, &QSerialPort::errorOccurred, this, &ReceiverSerialHandler::handleError);

    QThread::sleep(1);

    tryReconnect();
    serialPortTimer->start();
}

void ReceiverSerialHandler::tryConnect()
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

void ReceiverSerialHandler::updateSampleRate()
{
    emit Info_SampleRate(QString::number(samples));
    samples = 0;
}

void ReceiverSerialHandler::closeSerialPort_Recicer()
{
    if(serialPort->isOpen()){
        serialPort->close();
        emit Info_ComPort("Close Port");
    }
}

void ReceiverSerialHandler::separator_data(QByteArray buf)
{
    QJsonObject root;
    QJsonObject accel, battery;
    QJsonArray a1, a2, a3, a4, a5;

    auto timestampMs = (QTime::currentTime().second() * 1000) + QTime::currentTime().msec();
    root["timestamp"] = timestampMs;

    auto extractXYZ = [&](int offset, QJsonArray &arr) {
        int16_t x = assemble_16bit((uint8_t)buf[offset], (uint8_t)buf[offset + 1]);
        int16_t y = assemble_16bit((uint8_t)buf[offset + 2], (uint8_t)buf[offset + 3]);
        int16_t z = assemble_16bit((uint8_t)buf[offset + 4], (uint8_t)buf[offset + 5]);
        arr.append(x);
        arr.append(y);
        arr.append(z);
    };

    extractXYZ(2, a1);
    extractXYZ(8, a2);
    extractXYZ(14, a3);
    extractXYZ(20, a4);
    extractXYZ(26, a5);

    accel["sensor1"] = a1;
    accel["sensor2"] = a2;
    accel["sensor3"] = a3;
    accel["sensor4"] = a4;
    accel["sensor5"] = a5;

    root["acceleration"] = accel;

    uint16_t pantoBank_V_A = 0, pantoBank_V_B = 0;
    uint16_t pantoBank_C_A = 0, pantoBank_C_B = 0;
    uint16_t pantoBank_T_A = 0, pantoBank_T_B = 0;
    uint16_t pantoBank_BP_A = 0, pantoBank_BP_B = 0;

    // Battery parsing
    if ((uint8_t)buf[32] == 0xe1) {
        if ((uint8_t)buf[33] == 0x0a) {
            pantoBank_V_A = ((uint8_t)buf[34] | (uint8_t)buf[35] << 8);
            pantoBank_C_A = (uint8_t)buf[36];
            pantoBank_T_A = (uint8_t)buf[37];
            pantoBank_BP_A = calculateBatteryPercentage(pantoBank_V_A);

            QJsonObject bankA;
            bankA["voltage"] = pantoBank_V_A;
            bankA["current"] = pantoBank_C_A;
            bankA["temp"] = pantoBank_T_A;
            bankA["percent"] = pantoBank_BP_A;
            battery["bank_A"] = bankA;
        }
        else if ((uint8_t)buf[33] == 0x0b) {
            pantoBank_V_B = ((uint8_t)buf[34] | (uint8_t)buf[35] << 8);
            pantoBank_C_B = (uint8_t)buf[36];
            pantoBank_T_B = (uint8_t)buf[37];
            pantoBank_BP_B = calculateBatteryPercentage(pantoBank_V_B);

            QJsonObject bankB;
            bankB["voltage"] = pantoBank_V_B;
            bankB["current"] = pantoBank_C_B;
            bankB["temp"] = pantoBank_T_B;
            bankB["percent"] = pantoBank_BP_B;
            battery["bank_B"] = bankB;
        }
        buf[32] = 0;
    }

    root["battery"] = battery;

    QByteArray payload = QJsonDocument(root).toJson(QJsonDocument::Compact);

    // 🚀 Send via UDP
    QSettings settings("settings.ini", QSettings::IniFormat);
    udpManager->send(payload, QHostAddress("127.0.0.1"), settings.value("ACC/UDP/port", 20000).toInt());
}

signed int ReceiverSerialHandler::assemble_16bit(uint8_t Msb, uint8_t Lsb)
{
    int16_t D;

    D=(signed int)(Msb|Lsb<<8);

    //if(D<0){D=(65535-D)*(-1);}
    D=Division(D,2140)*1000;
    return D ;
}

float ReceiverSerialHandler::Division(int16_t num, int16_t den)
{
    float buf=0;
    buf=(num*1.0)/den;
    return buf;
}

double ReceiverSerialHandler::calculateBatteryPercentage(int millivolts)
{
    const int minVoltage = 2500; // 2.5V (0% charge)
    const int maxVoltage = 4200; // 4.2V (100% charge)

    // Calculate the percentage based on the voltage range
    double percentage = static_cast<double>(millivolts - minVoltage) / (maxVoltage - minVoltage) * 100.0;

    // Clamp the percentage between 0 and 100
    percentage = qMax(0.0, qMin(100.0, percentage));

    return percentage;
}
