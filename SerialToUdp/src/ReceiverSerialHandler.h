#ifndef ReceiverSerialHandler_H
#define ReceiverSerialHandler_H

#include <QObject>
#include <QCoreApplication>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include <QDebug>
#include <QSettings>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>
#include <QThread>

#include "udpthreadmanager.h"

class ReceiverSerialHandler : public QThread
{
    Q_OBJECT
public:
    explicit ReceiverSerialHandler(QObject *parent = nullptr, const QString &portName = "");

private:
    QThread *processThread;

    QSerialPort *serialPort;
    UdpThreadManager *udpManager;

    QTimer *serialPortTimer;

    void separator_data(QByteArray buf);
    signed int assemble_16bit(uint8_t Msb, uint8_t Lsb);
    float Division(int16_t num, int16_t den);
    void closeSerialPort_Recicer();

    uint16_t reciver_Timer;

    int samples = 0;
    QTimer *sampleRateTimer;

    double calculateBatteryPercentage(int millivolts);
    QString m_targetPortName = "";

private slots:
    void readData();
    void tryReconnect();
    void handleError(QSerialPort::SerialPortError error);
    void tryConnect();

    void updateSampleRate();

signals:
    void Info_ComPort(QString);
    void Info_SampleRate(QString);
};

#endif // ReceiverSerialHandler_H
