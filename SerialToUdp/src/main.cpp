#include <QCoreApplication>
#include <QThread>
#include <QSettings>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QSerialPortInfo>
#include <QDebug>

#include "ReceiverSerialHandler.h"
#include "GpsSerialHandler.h"

// Helper: find first serial port matching USB vendor/product IDs.
static QString findPortByUsbIds(quint16 vendorId, quint16 productId) {
    for (const QSerialPortInfo &info : QSerialPortInfo::availablePorts()) {
        if (info.hasVendorIdentifier() && info.hasProductIdentifier()) {
            if (info.vendorIdentifier() == vendorId && info.productIdentifier() == productId) {
                return info.portName();
            }
        }
    }
    return {};
}

// Helper: read an ID with fallback for "vender-id" typo.
static quint16 readIdWithFallback(QSettings &s, const QString &key, quint16 def = 0) {
    QVariant v = s.value(key);
    if (!v.isValid()) {
        QString alt = key;
        alt.replace("vendor-id", "vender-id");
        v = s.value(alt);
    }
    return v.isValid() ? static_cast<quint16>(v.toUInt()) : def;
}

// Helper: hex string like "0x1546"
static QString hex16(quint16 x) {
    return QString("0x%1").arg(QString::number(x, 16).rightJustified(4, '0'));
}

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    // Build an absolute path next to the executable.
    const QString settingsPath =
        QDir(QCoreApplication::applicationDirPath()).filePath("settings.ini");

    qInfo().noquote() << "Settings path:" << settingsPath;

    // Ensure the directory exists.
    QFileInfo finfo(settingsPath);
    if (!QDir().mkpath(finfo.absolutePath())) {
        qCritical().noquote() << "Failed to create directory for settings:"
                              << finfo.absolutePath();
        return 1;
    }

    // Create defaults if missing (spelling fixed to "vendor-id").
    if (!QFile::exists(settingsPath)) {
        QSettings init(settingsPath, QSettings::IniFormat);

        init.beginGroup("ACC");
            init.beginGroup("serial");
                init.setValue("vendor-id",  0x0483);
                init.setValue("product-id", 0x5740);
            init.endGroup();
            init.beginGroup("UDP");
                init.setValue("port", 30000);
            init.endGroup();
        init.endGroup();

        init.beginGroup("GPS");
            init.beginGroup("serial");
                init.setValue("module", "RTK");   // "RTK" or "NORMAL"
                init.beginGroup("RTK");
                    init.setValue("vendor-id",  0x1546);
                    init.setValue("product-id", 0x01a9);
                    init.setValue("use_rtk", false);
                    init.setValue("ztp_token", QString());
                    init.setValue("thing_name", QString());
                init.endGroup();
                init.beginGroup("NORMAL");
                    init.setValue("vendor-id",  0x1546);
                    init.setValue("product-id", 0x01a8);
                init.endGroup();
            init.endGroup();
            init.beginGroup("UDP");
                init.setValue("port", 20000);
            init.endGroup();
        init.endGroup();

        init.sync();
        if (init.status() != QSettings::NoError) {
            qCritical() << "Failed to write settings:" << init.status()
                        << "- path:" << settingsPath;
            return 1;
        } else {
            qInfo() << "Created settings file.";
        }
    }

    // Load settings (use the same absolute path).
    QSettings settings(settingsPath, QSettings::IniFormat);
    if (settings.status() != QSettings::NoError) {
        qCritical() << "Failed to open settings:" << settings.status()
                    << "- path:" << settingsPath;
        return 1;
    }

    // ACC IDs
    const quint16 accVid = readIdWithFallback(settings, "ACC/serial/vendor-id");
    const quint16 accPid = readIdWithFallback(settings, "ACC/serial/product-id");

    // GPS IDs chosen by module
    const QString gpsModule = settings.value("GPS/serial/module", "RTK").toString();
    const QString gpsBase   = QString("GPS/serial/%1").arg(gpsModule);
    const quint16 gpsVid = readIdWithFallback(settings, gpsBase + "/vendor-id");
    const quint16 gpsPid = readIdWithFallback(settings, gpsBase + "/product-id");

    // Resolve port names
    const QString accPort = findPortByUsbIds(accVid, accPid);
    const QString gpsPort = findPortByUsbIds(gpsVid, gpsPid);

    if (accPort.isEmpty()) {
        qWarning().noquote() << QString("ACC device not found (VID=%1 PID=%2).")
                                .arg(hex16(accVid)).arg(hex16(accPid));
    } else {
        qInfo() << "ACC port:" << accPort;
    }

    if (gpsPort.isEmpty()) {
        qWarning().noquote() << QString("GPS device not found (module=%1, VID=%2 PID=%3).")
                                .arg(gpsModule).arg(hex16(gpsVid)).arg(hex16(gpsPid));
    } else {
        qInfo() << "GPS port:" << gpsPort;
    }

    QThread *receiverThread = new QThread();
    ReceiverSerialHandler* receiverHandler = new ReceiverSerialHandler(nullptr, accPort);
    receiverHandler->moveToThread(receiverThread);

    receiverHandler->start();
    receiverThread->start();

    QThread *gpsThread = new QThread();
    GpsSerialHandler* gpsHandler = new GpsSerialHandler(nullptr, gpsPort);
    gpsHandler->moveToThread(gpsThread);

    gpsThread->start();

    return a.exec();
}
