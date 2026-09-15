#include "CoatingAnalysisInfoPanel.h"
#include "CoatingAnalysisLanguage.h"

#include <SprayThicknessPrediction/ThicknessPrediction.h>

#include <QLabel>
#include <QStringList>
#include <QVBoxLayout>

namespace robot_qt_viewer
{
    namespace
    {
        constexpr double kRadToDeg = 57.295779513082320876798154814105;

        QString angleText(double radians)
        {
            return QStringLiteral("%1 deg").arg(radians * kRadToDeg, 0, 'g', 6);
        }
    }

    CoatingAnalysisInfoPanel::CoatingAnalysisInfoPanel(QWidget* parent)
        : QWidget(parent)
    {
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(10, 8, 10, 8);
        layout->setSpacing(4);

        m_modelSection = addSection(layout, QStringLiteral("Model"));
        m_modelReadout = addReadout(layout);
        m_trajectorySection = addSection(layout, QStringLiteral("Trajectory"));
        m_trajectoryReadout = addReadout(layout);
        addSection(layout, QStringLiteral("Deposition"));
        m_depositionReadout = addReadout(layout);
        addSection(layout, QStringLiteral("Thermal History"));
        m_historyReadout = addReadout(layout);
        addSection(layout, QStringLiteral("Thickness"));
        m_thicknessReadout = addReadout(layout);
        addSection(layout, QStringLiteral("Computation"));
        m_computationReadout = addReadout(layout);
        m_validationSection = addSection(layout, QStringLiteral("Validation"));
        m_validationReadout = addReadout(layout);
        m_simulationSection = addSection(layout, QStringLiteral("Simulation"));
        m_simulationReadout = addReadout(layout);
        layout->addStretch(1);

        {
            const spraythickness::PaperGaussianParameters parameters;
            m_depositionReadout->setText(
                QStringLiteral("Amplitude    : %1 mm\n"
                               "Rotation     : %2\n"
                               "Phi offset   : %3\n"
                               "Psi offset   : %4\n"
                               "Sigma phi    : %5\n"
                               "Sigma psi    : %6\n"
                               "Ref distance : %7 mm\n"
                               "Ref angle    : %8 deg\n"
                               "Ref exposure : %9 s")
                    .arg(parameters.amplitudeMillimeters, 0, 'g', 6)
                    .arg(angleText(parameters.rotationRadians))
                    .arg(angleText(parameters.phiOffsetRadians))
                    .arg(angleText(parameters.psiOffsetRadians))
                    .arg(angleText(parameters.sigmaPhiRadians))
                    .arg(angleText(parameters.sigmaPsiRadians))
                    .arg(parameters.referenceDistanceMeters * 1000.0, 0, 'g', 6)
                    .arg(parameters.referenceAngleDegrees, 0, 'g', 6)
                    .arg(parameters.referenceExposureSeconds, 0, 'g', 6));
        }
        {
            const spraythickness::HeatHistoryParameters parameters;
            m_historyReadout->setText(
                QStringLiteral("Correction amp: %1\n"
                               "History scale : %2 s\n"
                               "Ref history   : %3 s\n"
                               "Cooling time  : %4 s\n"
                               "Activity thr  : %5")
                    .arg(parameters.correctionAmplitude, 0, 'g', 6)
                    .arg(parameters.historyScaleSeconds, 0, 'g', 6)
                    .arg(parameters.referenceHistorySeconds, 0, 'g', 6)
                    .arg(parameters.coolingTimeSeconds, 0, 'g', 6)
                    .arg(parameters.activityThresholdRatio, 0, 'g', 6));
        }
        setLanguageCode(QStringLiteral("en"));
    }

    void CoatingAnalysisInfoPanel::setLanguageCode(const QString& languageCode)
    {
        m_languageCode = languageCode.toLower().startsWith(QStringLiteral("zh"))
            ? QStringLiteral("zh-CN") : QStringLiteral("en");
        for(QLabel* label : findChildren<QLabel*>()) {
            label->setText(coatingAnalysisTranslate(m_languageCode, label->text()));
        }
    }

    QLabel* CoatingAnalysisInfoPanel::addSection(QVBoxLayout* layout, const QString& title)
    {
        auto* label = new QLabel(title, this);
        label->setStyleSheet(QStringLiteral("font-weight: bold;"));
        layout->addWidget(label);
        return label;
    }

    QLabel* CoatingAnalysisInfoPanel::addReadout(QVBoxLayout* layout)
    {
        auto* label = new QLabel(QStringLiteral("-"), this);
        label->setTextInteractionFlags(Qt::TextSelectableByMouse);
        label->setWordWrap(true);
        label->setStyleSheet(QStringLiteral("font-family: Consolas, monospace;"));
        layout->addWidget(label);
        return label;
    }

    void CoatingAnalysisInfoPanel::applyInfo(const CoatingAnalysisInfoView& view)
    {
        const bool simulation = view.simulationActive;
        if(m_modelSection != nullptr) {
            m_modelSection->setText(simulation
                ? QStringLiteral("Simulation plate")
                : QStringLiteral("Model"));
        }
        if(m_trajectorySection != nullptr) {
            m_trajectorySection->setText(simulation
                ? QStringLiteral("Simulation trajectory")
                : QStringLiteral("Trajectory"));
        }
        if(m_simulationSection != nullptr) {
            m_simulationSection->setText(simulation
                ? QStringLiteral("Simulation status")
                : QStringLiteral("Simulation"));
        }
        if(m_validationSection != nullptr) {
            m_validationSection->setVisible(!simulation);
        }
        if(m_validationReadout != nullptr) {
            m_validationReadout->setVisible(!simulation);
        }

        if(view.hasModel) {
            const CoatingAnalysisModelInfo& info = view.modelInfo;
            m_modelReadout->setText(
                QStringLiteral("Submeshes : %1\nVertices  : %2\nTriangles : %3\n"
                               "Size      : %4 x %5 x %6 mm")
                    .arg(static_cast<qulonglong>(info.subMeshCount))
                    .arg(static_cast<qulonglong>(info.vertexCount))
                    .arg(static_cast<qulonglong>(info.triangleCount))
                    .arg(info.sizeXMeters * 1000.0, 0, 'f', 2)
                    .arg(info.sizeYMeters * 1000.0, 0, 'f', 2)
                    .arg(info.sizeZMeters * 1000.0, 0, 'f', 2));
        } else {
            m_modelReadout->setText(QStringLiteral("No model loaded."));
        }

        if(view.hasTrajectory) {
            const CoatingAnalysisTrajectoryInfo& info = view.trajectoryInfo;
            m_trajectoryReadout->setText(
                QStringLiteral("Poses      : %1\nDuration   : %2 s\n"
                               "Path length: %3 mm\nAvg speed  : %4 mm/s")
                    .arg(static_cast<qulonglong>(info.pointCount))
                    .arg(info.durationSeconds, 0, 'f', 3)
                    .arg(info.pathLengthMeters * 1000.0, 0, 'f', 2)
                    .arg(info.averageSpeedMetersPerSecond * 1000.0, 0, 'f', 2));
        } else {
            m_trajectoryReadout->setText(QStringLiteral("No trajectory loaded."));
        }

        if(view.hasThickness) {
            const spraythickness::ThicknessMetrics& metrics = view.thicknessMetrics;
            m_thicknessReadout->setText(
                QStringLiteral("Min   : %1 um\n"
                               "Max   : %2 um\n"
                               "Average: %3 um\n"
                               "Compute: %4 s\n"
                               "Coverage: %5%\n"
                               "Under : %6%  Over : %7%")
                    .arg(metrics.minThickness * 1.0e6, 0, 'f', 1)
                    .arg(metrics.maxThickness * 1.0e6, 0, 'f', 1)
                    .arg(metrics.averageThickness * 1.0e6, 0, 'f', 1)
                    .arg(view.predictionElapsedSeconds, 0, 'f', 3)
                    .arg(metrics.coverageRatio * 100.0, 0, 'f', 1)
                    .arg(metrics.underCoatedRatio * 100.0, 0, 'f', 1)
                    .arg(metrics.overCoatedRatio * 100.0, 0, 'f', 1));

            const spraythickness::ThicknessPredictionTiming& timing =
                view.predictionTiming;
            QStringList lines;
            lines << QStringLiteral("Wall time : %1 s")
                         .arg(view.predictionElapsedSeconds, 0, 'f', 3);
            if(simulation) {
                lines << QStringLiteral("Volume    : %1 mm3")
                             .arg(view.simulationThicknessVolumeCubicMillimeters,
                                 0, 'f', 6);
            }
            if(timing.valid) {
                lines << QStringLiteral("Backend   : %1 ms")
                             .arg(timing.backendTotalMilliseconds, 0, 'f', 1)
                    << QStringLiteral("BVH       : %1 ms")
                             .arg(timing.bvhMilliseconds, 0, 'f', 1)
                    << QStringLiteral("Upload    : %1 ms")
                             .arg(timing.uploadMilliseconds, 0, 'f', 1)
                    << QStringLiteral("Dispatch  : %1 ms")
                             .arg(timing.dispatchMilliseconds, 0, 'f', 1)
                    << QStringLiteral("Pure GPU  : %1 ms")
                             .arg(timing.pureGpuMilliseconds, 0, 'f', 1)
                    << QStringLiteral("Readback  : %1 ms")
                             .arg(timing.readbackMilliseconds, 0, 'f', 1)
                    << QStringLiteral("Batch     : %1")
                             .arg(static_cast<qulonglong>(timing.adaptiveBatchSize))
                    << QStringLiteral("Inputs    : %1 vertices x %2 spray points")
                             .arg(static_cast<qulonglong>(timing.predictionVertexCount))
                             .arg(static_cast<qulonglong>(timing.sprayPointCount));
                if(timing.periodicMappingMilliseconds > 0.0) {
                    lines << QStringLiteral("Mapping   : %1 ms")
                                 .arg(timing.periodicMappingMilliseconds, 0, 'f', 1);
                }
                if(timing.axisymmetricMappingMilliseconds > 0.0) {
                    lines << QStringLiteral("Axis map  : %1 ms (%2)")
                                 .arg(timing.axisymmetricMappingMilliseconds, 0, 'f', 1)
                                 .arg(timing.axisymmetricMappingCacheHit
                                     ? QStringLiteral("cache hit")
                                     : QStringLiteral("built"))
                         << QStringLiteral("Axis bind : %1 active / %2")
                                 .arg(static_cast<qulonglong>(
                                     timing.axisymmetricActiveBindingCount))
                                 .arg(static_cast<qulonglong>(
                                     timing.axisymmetricBindingCount))
                         << QStringLiteral("Axis zero : %1  nonzero: %2")
                                 .arg(static_cast<qulonglong>(
                                     timing.axisymmetricMappedZeroCount))
                                 .arg(static_cast<qulonglong>(
                                     timing.axisymmetricMappedNonzeroCount))
                         << QStringLiteral("Axis invalid: %1")
                                 .arg(static_cast<qulonglong>(
                                     timing.axisymmetricMappedInvalidCount))
                         << QStringLiteral("Axis parts: %1 segments / %2 paths")
                                 .arg(static_cast<qulonglong>(
                                     timing.axisymmetricActiveSegmentCount))
                                 .arg(static_cast<qulonglong>(
                                     timing.axisymmetricActiveProfilePathCount));
                }
                if(timing.spatialFiltering) {
                    lines << QStringLiteral("Grid      : %1 ms")
                                 .arg(timing.spatialGridMilliseconds, 0, 'f', 1)
                        << QStringLiteral("Grid mode : %1")
                                 .arg(timing.manualSpatialGridCellSize
                                     ? QStringLiteral("manual")
                                     : QStringLiteral("automatic"))
                        << QStringLiteral("Grid size : %1 mm")
                                 .arg(timing.spatialGridCellSizeMeters * 1000.0, 0, 'f', 3)
                        << QStringLiteral("Grid dim  : %1 x %2 x %3")
                                 .arg(timing.spatialGridDimensionX)
                                 .arg(timing.spatialGridDimensionY)
                                 .arg(timing.spatialGridDimensionZ)
                        << QStringLiteral("Grid cells: %1")
                                 .arg(static_cast<qulonglong>(timing.spatialGridCellCount))
                        << QStringLiteral("Cell refs : %1")
                                 .arg(static_cast<qulonglong>(
                                     timing.spatialGridCandidateCellPairs))
                        << QStringLiteral("Vertex refs: %1")
                                 .arg(static_cast<qulonglong>(
                                     timing.spatialGridCandidateVertexPairs))
                        << QStringLiteral("GPU vertices: %1 / %2")
                                 .arg(static_cast<qulonglong>(
                                     timing.predictionVertexCount))
                                 .arg(static_cast<qulonglong>(
                                     timing.spatialInputVertexCount))
                        << QStringLiteral("Skipped zero: %1")
                                 .arg(static_cast<qulonglong>(
                                     timing.spatialSkippedVertexCount))
                        << QStringLiteral("Grid cache: %1")
                                 .arg(timing.spatialGridCacheHit
                                     ? QStringLiteral("hit")
                                     : QStringLiteral("built"));
                }
            }
            m_computationReadout->setText(lines.join(QStringLiteral("\n")));
        } else {
            m_thicknessReadout->setText(simulation
                ? QStringLiteral("No simulation thickness result yet.")
                : QStringLiteral("No thickness result yet."));
            m_computationReadout->setText(simulation
                ? QStringLiteral("Waiting for simulation...")
                : QStringLiteral("No computation data yet."));
        }
        m_validationReadout->setText(view.validationDetails);
        m_simulationReadout->setText(view.simulationActive
            ? view.simulationDetails
            : QStringLiteral("Simulation inactive."));
        setLanguageCode(m_languageCode);
    }
}
