#ifndef UDPWORKER_H
#define UDPWORKER_H

#include <QObject>
#include <QUdpSocket>
#include <QHostAddress>

class UdpWorker : public QObject
{
    Q_OBJECT

public:
    explicit UdpWorker(QObject *parent = nullptr);
    void sendData(const QByteArray &data, const QHostAddress &address, quint16 port);

private:
    QUdpSocket *udpSocket;
};

#endif // UDPWORKER_H
