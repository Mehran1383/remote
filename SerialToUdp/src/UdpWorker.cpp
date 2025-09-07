#include "UdpWorker.h"
#include <QDebug>

UdpWorker::UdpWorker(QObject *parent)
    : QObject(parent)
{
    udpSocket = new QUdpSocket(this);  // No bind needed
}

void UdpWorker::sendData(const QByteArray &data, const QHostAddress &address, quint16 port)
{
    qint64 bytesSent = udpSocket->writeDatagram(data, address, port);
    if (bytesSent == -1) {
        qWarning() << "Failed to send datagram:" << udpSocket->errorString();
    } else {
        qDebug() << "Sent" << bytesSent << "bytes to" << address.toString() << ":" << port;
    }
}
