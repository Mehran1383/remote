#ifndef NTRIPCLIENT_H
#define NTRIPCLIENT_H

#include <QObject>
#include <QTcpSocket>
#include <QTimer>
#include <QNetworkAccessManager>

class NtripClient : public QObject
{
    Q_OBJECT
public:
    explicit NtripClient(QObject *parent = nullptr);
    ~NtripClient();

    void startStream();
    bool sendGGA(QByteArray& gga);
    bool setZtpParameters(const QString token, const QString name, const QString id);

signals:
    void streamBytes(const QByteArray& bytes);

private slots:
    void onReadyRead();
    void onConnected();
    void onSocketError(QAbstractSocket::SocketError);
    void onSocketTimeout();
    void onResponseTimeout();
    void onReplyFinished(QNetworkReply* reply);

private:
    void makeRequest();
    void parseHeaders();
    bool saveCredentials(const QJsonObject& credentials);
    bool loadCredentials();
    bool areCredentialsValid(const QJsonObject& credentials);
    bool isCacheValid();
    void startStreamWithCredentials();
    void fetchCredentials();
    void updateDelayTime();

    QString userName, password, server, mount, ztpToken, thingName, productId;
    quint16 httpPort;
    QTcpSocket socket;
    QByteArray buffer;
    QTimer reconnectTimer, responseTimer;
    QNetworkAccessManager networkManager;

    bool readingHeaders, credentialsExist;
    int delay;
};

#endif // NTRIPCLIENT_H
