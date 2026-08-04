#include "IntegratedDemoController.h"
#include "ShapeModelDebugViewModel.h"
#include "VisionDisplay/VisionDisplayItem.h"
#include "shape_match/pipeline_v3/WorkerPoolV3.h"

#include <QCoreApplication>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
    ShapeMatch::WorkerPoolV3::initializeShared();

    IntegratedDemoController integratedController;
    ShapeModelDebugViewModel shapeModelDebug;

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty(QStringLiteral("integratedController"), &integratedController);
    engine.rootContext()->setContextProperty(QStringLiteral("shapeModelDebug"), &shapeModelDebug);
    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);
    engine.loadFromModule("VisionDisplayApp", "Main");

    return app.exec();
}
