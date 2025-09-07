#include "udpthreadmanager.h"

UdpThreadManager::UdpThreadManager(QObject *parent)
    : QObject(parent)
{
    worker = new UdpWorker();
    worker->moveToThread(&workerThread);

    connect(&workerThread, &QThread::finished, worker, &QObject::deleteLater);
    workerThread.start();
}

UdpThreadManager::~UdpThreadManager()
{
    workerThread.quit();
    workerThread.wait();
}

void UdpThreadManager::send(const QByteArray &data, const QHostAddress &address, quint16 port)
{
    QMetaObject::invokeMethod(worker, [=]() {
        worker->sendData(data, address, port);
    }, Qt::QueuedConnection);
}
