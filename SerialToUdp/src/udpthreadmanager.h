#ifndef UDPTHREADMANAGER_H
#define UDPTHREADMANAGER_H

#include <QObject>
#include <QThread>
#include "UdpWorker.h"

class UdpThreadManager : public QObject
{
    Q_OBJECT

public:
    explicit UdpThreadManager(QObject *parent = nullptr);
    ~UdpThreadManager();

    void send(const QByteArray &data, const QHostAddress &address, quint16 port);

private:
    QThread workerThread;
    UdpWorker *worker;
};

#endif // UDPTHREADMANAGER_H
