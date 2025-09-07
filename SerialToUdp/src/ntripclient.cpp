#include "ntripclient.h"

#include <QFile>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QStandardPaths>
#include <QDir>
#include <QNetworkReply>

#define SOCKET_RECONNECT_DELAY 10 // seconds
#define SERVER_RECONNECT_DELAY 60 // seconds
#define MAX_RESTART_DELAY 300 // seconds
#define MIN_RESTART_DELAY 5 // seconds
#define CACHE_FILENAME "credentials.json"
#define ZTP_API_URL "https://api.thingstream.io/ztp/pointperfect/credentials"

NtripClient::NtripClient(QObject *parent)
    : QObject{parent},
    readingHeaders(true),
    credentialsExist(false),
    delay(0)
{
    reconnectTimer.setSingleShot(true);
    responseTimer.setSingleShot(true);

    connect(&reconnectTimer, &QTimer::timeout,
            this, &NtripClient::onSocketTimeout);

    connect(&responseTimer, &QTimer::timeout,
            this, &NtripClient::onResponseTimeout);

    connect(&socket, &QTcpSocket::readyRead,
            this, &NtripClient::onReadyRead);

    connect(&socket, &QTcpSocket::connected,
            this, &NtripClient::onConnected);

    connect(&socket, &QTcpSocket::errorOccurred,
            this, &NtripClient::onSocketError);

    connect(&networkManager, &QNetworkAccessManager::finished,
            this, &NtripClient::onReplyFinished);
}

NtripClient::~NtripClient()
{
    if (socket.isOpen())
        socket.disconnectFromHost();

    responseTimer.stop();
    reconnectTimer.stop();
}

void NtripClient::startStream()
{
    if (isCacheValid() && loadCredentials()) {
        credentialsExist = true;
        startStreamWithCredentials();
        return;
    }

    fetchCredentials();
}

void NtripClient::startStreamWithCredentials()
{
    if (socket.isOpen())
        socket.disconnectFromHost();

    buffer.clear();
    readingHeaders = true;
    socket.connectToHost(server, httpPort);
    reconnectTimer.start(SOCKET_RECONNECT_DELAY * 1000);
}

void NtripClient::makeRequest()
{
    QByteArray req;
    QByteArray auth = (userName + ":" + password).toUtf8().toBase64();

    req += "GET /" + mount.toUtf8() + " HTTP/1.1\r\n";
    req += "Host: " + server.toUtf8() + "\r\n";
    req += "User-Agent: NTRIP Client\r\n";
    req += "Accept: */*\r\n";
    req += "Authorization: Basic " + auth + "\r\n";
    req += "Connection: close\r\n\r\n";

    if (socket.write(req) > 0) {
        responseTimer.start(SERVER_RECONNECT_DELAY * 1000);
        delay = 0;
    } else {
        updateDelayTime();
        QTimer::singleShot(delay * 1000, this, &NtripClient::startStreamWithCredentials);
    }
}

bool NtripClient::isCacheValid()
{
    QFile cacheFile(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + "/" + CACHE_FILENAME);
    if (!cacheFile.exists())
        return false;

    if (!cacheFile.open(QIODevice::ReadOnly))
        return false;


    QByteArray cacheData = cacheFile.readAll();
    cacheFile.close();

    QJsonParseError parserError;
    QJsonDocument doc = QJsonDocument::fromJson(cacheData, &parserError);
    if (parserError.error != QJsonParseError::NoError || !doc.isObject())
        return false;

    return true;
}

bool NtripClient::loadCredentials()
{
    QString cachePath = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + "/" + CACHE_FILENAME;
    QFile cacheFile(cachePath);

    qDebug() << cachePath;
    if (!cacheFile.open(QIODevice::ReadOnly)) {
        qWarning() << "Failed to open cache file: " << cachePath;
        return false;
    }

    QByteArray cacheData = cacheFile.readAll();
    cacheFile.close();

    QJsonParseError parserError;
    QJsonDocument doc = QJsonDocument::fromJson(cacheData, &parserError);
    if (parserError.error != QJsonParseError::NoError) {
        qWarning() << "Cache JSON parser error: " << parserError.errorString();
        return false;
    }

    if (!doc.isObject()) {
        qWarning() << "Invalid cache JSON structure";
        return false;
    }

    QJsonObject cache = doc.object();
    QJsonObject credentials = cache["ntripCredentials"].toObject();

    if (!areCredentialsValid(credentials)) {
        qWarning() << "Cached credentials are invalid";
        return false;
    }

    userName = credentials["userName"].toString();
    password = credentials["password"].toString();
    server = credentials["endpoint"].toString();
    httpPort = credentials["httpPort"].toString().toInt();
    mount = credentials["mountPoint"].toString();

    return true;
}

bool NtripClient::areCredentialsValid(const QJsonObject& credentials)
{
    return !credentials["userName"].toString().isEmpty() &&
           !credentials["password"].toString().isEmpty() &&
           !credentials["endpoint"].toString().isEmpty() &&
           credentials["httpPort"].toString().toInt() > 0 &&
           !credentials["mountPoint"].toString().isEmpty();
}

bool NtripClient::saveCredentials(const QJsonObject& credentials)
{
    QString appDataDir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    QDir dir(appDataDir);
    if (!dir.exists()) {
        if (!dir.mkpath(".")) {
            qWarning() << "Failed to create app data directory: " << appDataDir;
            return false;
        }
    }

    QString cachePath = appDataDir + "/" + CACHE_FILENAME;
    QFile cacheFile(cachePath);

    if (!cacheFile.open(QIODevice::WriteOnly)) {
        qWarning() << "Failed to open cache file for writing: " << cachePath;
        return false;
    }

    QJsonObject cache;
    cache["ntripCredentials"] = credentials;

    QJsonDocument doc(cache);
    qint64 bytesWritten = cacheFile.write(doc.toJson());
    cacheFile.close();

    if (bytesWritten == -1) {
        qWarning() << "Failed to write cache file";
        return false;
    }

    return true;
}

void NtripClient::fetchCredentials()
{
    QUrl url(ZTP_API_URL);
    QNetworkRequest request(url);

    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    QJsonObject jsonPayload;
    jsonPayload["token"] = ztpToken;
    jsonPayload["givenName"] = thingName;
    jsonPayload["hardwareId"] = productId;

    QJsonDocument jsonDoc(jsonPayload);
    QByteArray postData = jsonDoc.toJson();

    networkManager.post(request, postData);
    responseTimer.start(SERVER_RECONNECT_DELAY * 1000);
}

bool NtripClient::setZtpParameters(const QString token, const QString name, const QString id)
{
    if (token.isEmpty() || name.isEmpty() || id.isEmpty())
        return false;

    ztpToken = token;
    thingName = name;
    productId = id;

    return true;
}

void NtripClient::onReplyFinished(QNetworkReply* reply)
{
    responseTimer.stop();

    if (reply->error() != QNetworkReply::NoError) {
        qWarning() << "ZTP API error:" << reply->errorString();
        reply->deleteLater();

        if (credentialsExist) {
            startStreamWithCredentials();
        } else {
            updateDelayTime();
            QTimer::singleShot(delay * 1000, this, &NtripClient::startStream);
        }
        return;
    }

    QByteArray response = reply->readAll();
    reply->deleteLater();

    QJsonParseError parserError;
    QJsonDocument doc = QJsonDocument::fromJson(response, &parserError);

    if (parserError.error != QJsonParseError::NoError) {
        qWarning() << "ZTP JSON parser error:" << parserError.errorString();

        if (credentialsExist) {
            startStreamWithCredentials();
        } else {
            updateDelayTime();
            QTimer::singleShot(delay * 1000, this, &NtripClient::startStream);
        }
        return;
    }

    if (!doc.isObject()) {
        qWarning() << "Invalid ZTP JSON structure";

        if (credentialsExist) {
            startStreamWithCredentials();
        } else {
            updateDelayTime();
            QTimer::singleShot(delay * 1000, this, &NtripClient::startStream);
        }
        return;
    }

    QJsonObject credentials = doc["rtcmCredentials"].toObject();
    if (credentials.contains("userName"))
        userName = credentials["userName"].toString();

    if (credentials.contains("password"))
        password = credentials["password"].toString();

    if (credentials.contains("endpoint"))
        server = credentials["endpoint"].toString();

    if (credentials.contains("httpPort"))
        httpPort = credentials["httpPort"].toString().toInt();

    if (credentials.contains("mountPoint"))
        mount = credentials["mountPoint"].toString();

    if (userName.isEmpty() || password.isEmpty() || server.isEmpty() || mount.isEmpty() || httpPort == 0) {
        qWarning() << "Incomplete ZTP credentials received";

        if (credentialsExist) {
            startStreamWithCredentials();
        } else {
            updateDelayTime();
            QTimer::singleShot(delay * 1000, this, &NtripClient::startStream);
        }
        return;
    }

    credentialsExist = true;
    delay = 0;

    if (!saveCredentials(credentials)) {
        qWarning() << "Failed to save credentials to cache";
    }

    startStreamWithCredentials();
}

void NtripClient::onConnected()
{
    qInfo() << QString("Connected to %1:%2").arg(server).arg(httpPort);
    reconnectTimer.stop();
    delay = 0;
    makeRequest();
}

void NtripClient::onSocketTimeout()
{
    qWarning() << "Socket connection time out";
    updateDelayTime();
    QTimer::singleShot(delay * 1000, this, &NtripClient::startStreamWithCredentials);
}

void NtripClient::onResponseTimeout()
{
    qWarning() << "Response time out";
    updateDelayTime();
    QTimer::singleShot(delay * 1000, this, &NtripClient::startStream);
}

void NtripClient::onSocketError(QAbstractSocket::SocketError)
{
    qWarning() << QString("Socket error: %1").arg(socket.errorString());

    if (!readingHeaders) {
        responseTimer.stop();
        reconnectTimer.stop();
        updateDelayTime();
        QTimer::singleShot(delay * 1000, this, &NtripClient::startStreamWithCredentials);
    }
}

void NtripClient::onReadyRead()
{
    responseTimer.stop();
    delay = 0;
    buffer += socket.readAll();

    if (readingHeaders) {
        parseHeaders();
    } else {
        emit streamBytes(buffer);
        buffer.clear();
    }

    if (!readingHeaders)
        responseTimer.start(SERVER_RECONNECT_DELAY * 1000);
}

void NtripClient::parseHeaders()
{
    qsizetype idx = buffer.indexOf("\r\n\r\n");
    if (idx < 0)
        return; // wait for full headers

    QByteArray headers = buffer.left(idx);
    QByteArray rest = buffer.mid(idx + 4);

    // Status line
    QList<QByteArray> lines = headers.split('\n');
    if (lines.isEmpty()) {
        qWarning() << "Invalid HTTP response";
        updateDelayTime();
        QTimer::singleShot(delay * 1000, this, &NtripClient::startStream);
        return;
    }
    QByteArray statusLine = lines.first().trimmed();

    QList<QByteArray> parts = statusLine.split(' ');
    if (parts.size() < 2 || parts.at(1) != "200") {
        qWarning() << QString("HTTP Error: %1").arg(QString::fromUtf8(statusLine));
        if (parts.at(1) == "401") { // Unauthorized
            credentialsExist = false;
            updateDelayTime();
            QTimer::singleShot(delay * 1000, this, &NtripClient::fetchCredentials);
            return;
        }
        updateDelayTime();
        QTimer::singleShot(delay * 1000, this, &NtripClient::startStream);
        return;
    }

    // the rest of the response contains data
    if (!rest.isEmpty())
        emit streamBytes(rest);

    readingHeaders = false;
    buffer.clear();
    delay = 0;
}

bool NtripClient::sendGGA(QByteArray& gga)
{
    if (!gga.endsWith("\r\n"))
        gga.append("\r\n");

    if (socket.state() == QAbstractSocket::ConnectedState) {
        qint64 size = socket.write(gga);
        return size == gga.size();
    }
    return false;
}

void NtripClient::updateDelayTime()
{
    delay = qMin(delay + MIN_RESTART_DELAY, MAX_RESTART_DELAY);
}


