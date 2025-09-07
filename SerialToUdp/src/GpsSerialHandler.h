#ifndef GpsSerialHandler_H
#define GpsSerialHandler_H

#include <QThread>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QSettings>
#include <QTime>
#include <QJsonArray>
#include <QTimer>
#include <QMutex>
#include <QJsonObject>
#include <QJsonDocument>

#include "udpthreadmanager.h"
#include "ntripclient.h"

#define DEBUG true

class GpsSerialHandler : public QThread
{
    Q_OBJECT
public:
    explicit GpsSerialHandler(QObject *parent = nullptr, const QString &portName = "");
    ~GpsSerialHandler();

private:
    QTimer *serialPortTimer;
    QSerialPort *serialPort;
    UdpThreadManager *udpManager;
    NtripClient *client;

    static constexpr unsigned int ggaIntervalSec = 10;
    int samples = 0;
    bool usingRtk;
    QTimer *sampleRateTimer;
    QElapsedTimer lastGgaTimer;

    std::tuple<double, bool> processRMC(const QString &line);
    std::tuple<double, double, int, int, double> processGGA(const QString &line);
    double convertToDecimal(const QString &coordinate, const QString &direction, bool isLatitude);
    void parseLine(const QString &line);
    QString m_targetPortName = "";

private slots:
    void readData();
    void handleError(QSerialPort::SerialPortError error);
    void tryReconnect();
    void tryConnect();
    void onNtripBytes(const QByteArray& bytes);
    void updateSampleRate();

signals:
    void Info_ComPort(QString);
    void Info_SampleRate(QString);
};

#endif // GpsSerialHandler_H
