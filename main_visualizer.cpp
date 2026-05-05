#include <QGuiApplication>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DExtras/QForwardRenderer>
#include <Qt3DRender/QCamera>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QConeMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QStringList>
#include <iostream>
#include <math.h>

#include "rocket_attitude.hpp"

using namespace std;

// 独自QuaternionからQt3DのQQuaternionへの変換ヘルパー
QQuaternion toQQuaternion(const Quaternion& q) {
    return QQuaternion((float)q.w, (float)q.x, (float)q.y, (float)q.z);
}

// ロケットの姿勢情報（Transform）を外から操作できるように参照を受け取る形に変更
Qt3DCore::QEntity* createScene(Qt3DCore::QTransform*& outRocketTransform) {
    Qt3DCore::QEntity *rootEntity = new Qt3DCore::QEntity;

    // --- ロケットのルートエンティティ ---
    Qt3DCore::QEntity *rocketEntity = new Qt3DCore::QEntity(rootEntity);
    Qt3DCore::QTransform *rocketTransform = new Qt3DCore::QTransform();
    outRocketTransform = rocketTransform; // コントロール用に参照を渡す
    rocketEntity->addComponent(rocketTransform);

    // ロケット全体の色（赤色）
    Qt3DExtras::QPhongMaterial *material = new Qt3DExtras::QPhongMaterial();
    material->setDiffuse(QColor(Qt::red));

    // --- ロケットの胴体（円柱） ---
    Qt3DCore::QEntity *bodyEntity = new Qt3DCore::QEntity(rocketEntity);
    Qt3DExtras::QCylinderMesh *bodyMesh = new Qt3DExtras::QCylinderMesh();
    bodyMesh->setRadius(0.2f);
    bodyMesh->setLength(2.0f);
    Qt3DCore::QTransform *bodyTransform = new Qt3DCore::QTransform();
    bodyTransform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), -90.0f));
    bodyEntity->addComponent(bodyMesh);
    bodyEntity->addComponent(material);
    bodyEntity->addComponent(bodyTransform);

    // --- ロケットの先端（円錐） ---
    Qt3DCore::QEntity *noseEntity = new Qt3DCore::QEntity(rocketEntity);
    Qt3DExtras::QConeMesh *noseMesh = new Qt3DExtras::QConeMesh();
    noseMesh->setBottomRadius(0.4f);
    noseMesh->setLength(1.0f);
    Qt3DCore::QTransform *noseTransform = new Qt3DCore::QTransform();
    noseTransform->setTranslation(QVector3D(1.5f, 0.0f, 0.0f));
    noseTransform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), -90.0f));
    noseEntity->addComponent(noseMesh);
    noseEntity->addComponent(material);
    noseEntity->addComponent(noseTransform);

    return rootEntity;
}

int main(int argc, char *argv[]) {
    QGuiApplication app(argc, argv);
    Qt3DExtras::Qt3DWindow view;
    view.defaultFrameGraph()->setClearColor(QColor(QRgb(0x4d4d4f)));

    std::cout << "--- リアルタイム姿勢ビジュアライザ ---" << std::endl;

    // シーンを作成し、ロケットの姿勢を操作するためのTransformオブジェクトを受け取る
    Qt3DCore::QTransform *rocketTransform = nullptr;
    Qt3DCore::QEntity *scene = createScene(rocketTransform);

    // 機体の姿勢オブジェクト
    RocketAttitude rocket_attitude;

    // ===== シリアル通信の設定 =====
    QSerialPort serial;
    serial.setBaudRate(115200);

    // 利用可能なシリアルポートを探して自動接続（Macに繋いだESP32などを想定）
    bool portFound = false;
    for (const QSerialPortInfo &info : QSerialPortInfo::availablePorts()) {
        QString name = info.portName();
        if (name.contains("usb", Qt::CaseInsensitive) || 
            name.contains("SLAB", Qt::CaseInsensitive) ||
            name.contains("wch", Qt::CaseInsensitive)) {
            
            serial.setPort(info);
            if (serial.open(QIODevice::ReadOnly)) {
                std::cout << "[成功] シリアルポートに接続しました: " << name.toStdString() << " (115200 bps)" << std::endl;
                portFound = true;
                break;
            }
        }
    }

    if (!portFound) {
        std::cout << "[警告] マイコンらしきシリアルポートが見つかりませんでした。" << std::endl;
        std::cout << "USBケーブルが接続されているか確認してください。" << std::endl;
    }

    // データ受信時のイベント（リアルタイム処理）
    QObject::connect(&serial, &QSerialPort::readyRead, [&serial, rocketTransform, &rocket_attitude]() {
        while (serial.canReadLine()) {
            QByteArray lineBytes = serial.readLine().trimmed();
            if (lineBytes.isEmpty()) continue;
            
            QString lineStr(lineBytes);
            lineStr.replace(",", " ");
            QStringList parts = lineStr.split(" ", Qt::SkipEmptyParts);
            
            if (parts.size() >= 3) {
                bool ok1, ok2, ok3;
                double yaw   = parts[0].toDouble(&ok1);
                double pitch = parts[1].toDouble(&ok2);
                double roll  = parts[2].toDouble(&ok3);
                
                // 数値変換に成功した場合のみ姿勢を更新
                if (ok1 && ok2 && ok3) {
                    double yaw_rad   = yaw * M_PI / 180.0;
                    double pitch_rad = pitch * M_PI / 180.0;
                    double roll_rad  = roll * M_PI / 180.0;
                    
                    // オブジェクト指向で姿勢を更新
                    rocket_attitude.setFromEulerZYX(yaw_rad, pitch_rad, roll_rad);
                    
                    // 3Dモデルをリアルタイムに回転
                    rocketTransform->setRotation(toQQuaternion(rocket_attitude.getQuaternion()));
                }
            }
        }
    });

    // カメラの設定（全体が見えるように斜め上から見下ろす）
    Qt3DRender::QCamera *camera = view.camera();
    camera->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    camera->setPosition(QVector3D(5.0f, 5.0f, 5.0f));
    camera->setViewCenter(QVector3D(0, 0, 0));

    // カメラコントローラー
    Qt3DExtras::QOrbitCameraController *camController = new Qt3DExtras::QOrbitCameraController(scene);
    camController->setLinearSpeed(50.0f);
    camController->setLookSpeed(180.0f);
    camController->setCamera(camera);

    view.setRootEntity(scene);
    view.setTitle("Real-time Rocket Visualizer");
    view.resize(800, 600);
    view.show();

    return app.exec();
}
