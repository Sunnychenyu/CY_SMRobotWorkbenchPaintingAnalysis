#include "DynamicSurfaceDisplayAdapter.h"
#include "CoatingAnalysisSession.h"
#include <AssetCore/ModelDesc.h>
#include <AssetCore/SubMeshDesc.h>
#include <QSaveFile>
#include <QDataStream>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <limits>

namespace robot_qt_viewer
{
    DynamicSurfaceView makeDynamicSurfaceView(const QString& objectId,
        const spraythickness::DynamicSurfaceSnapshot& s,const Eigen::Isometry3d& transform,
        const DynamicSurfaceDisplay& display)
    {
        DynamicSurfaceView out; out.model=std::make_shared<assetcore::ModelDesc>();
        out.overlay.objectId=objectId.toStdString(); out.overlay.unit="m";
        out.overlay.quantityName=display.coloring==1?"Displacement":"Thickness";
        out.overlay.range={0,0};
        const auto inverse=transform.inverse();
        auto append=[&](bool substrate) {
            assetcore::SubMeshDesc mesh; mesh.name=substrate?"Original substrate":"Evolved surface";
            mesh.geometry.indices=s.substrate->triangleIndices;
            smrobot::visualization::SurfaceScalarSubMesh scalar;
            std::vector<std::size_t> binding;
            for(std::size_t i=0;i<s.positions.size();++i) {
                const auto& original=s.substrate->samples[i];
                mesh.geometry.positions.push_back((inverse*(substrate?original.position:s.positions[i])).cast<float>());
                mesh.geometry.normals.push_back((inverse.linear()*(substrate?original.normal:s.normals[i])).cast<float>());
                double value=display.coloring==1?(s.positions[i]-original.position).norm():s.thicknessMeters[i];
                if(substrate) value=0;
                scalar.values.push_back(value); binding.push_back(i);
                out.overlay.range.maximum=std::max(out.overlay.range.maximum,value);
            }
            out.model->addSubMesh(mesh);out.overlay.subMeshes.push_back(std::move(scalar));out.binding.sampleIndicesBySubMesh.push_back(std::move(binding));
        };
        append(display.view==1); if(display.view==2) append(true);
        if(display.coloring==2 || display.view==1) {
            const auto color=display.view==1?QColor(160,170,180):display.color;
            Eigen::Vector3f rgb(static_cast<float>(color.redF()),static_cast<float>(color.greenF()),static_cast<float>(color.blueF()));
            out.overlay.colorMap=smrobot::visualization::ScalarColorMap({{0,rgb},{1,rgb}});
        }
        return out;
    }

    bool saveDynamicSurface(const QString& path,const CoatingAnalysisSession& session,bool ply,QString* error)
    {
        auto fail=[&](const QString& message){if(error)*error=message;return false;};
        if(!session.prediction.dynamicSurface) return fail(QStringLiteral("没有可保存的动态表面结果。"));
        const auto& s=*session.prediction.dynamicSurface;
        QSaveFile file(path); if(!file.open(QIODevice::WriteOnly)) return fail(file.errorString());
        QDataStream data(&file); data.setVersion(QDataStream::Qt_5_15); data.setByteOrder(QDataStream::LittleEndian); data.setFloatingPointPrecision(QDataStream::DoublePrecision);
        const auto count=s.positions.size();
        if(ply) {
            const QByteArray header=QStringLiteral("ply\nformat binary_little_endian 1.0\ncomment units meters; world coordinates\ncomment thickness is accumulated normal deposition; displacement is distance from substrate\nelement vertex %1\nproperty double x\nproperty double y\nproperty double z\nproperty double nx\nproperty double ny\nproperty double nz\nproperty double thickness\nproperty double displacement\nproperty double original_x\nproperty double original_y\nproperty double original_z\nelement face %2\nproperty list uchar uint vertex_indices\nend_header\n").arg(static_cast<qulonglong>(count)).arg(static_cast<qulonglong>(s.substrate->triangleIndices.size()/3)).toLatin1();
            file.write(header);
        } else {
            using Json=nlohmann::json; const auto& p=session.resultDepositionParameters;
            auto curve=[](const spraythickness::CalibratedEfficiencyCurve& c){return Json{{"kind",static_cast<int>(c.kind)},{"parameters",c.parameters},{"x",c.x},{"y",c.y},{"lowerRampEnd",c.lowerRampEnd}};};
            auto history=[](const spraythickness::HeatHistoryParameters& h){return Json{{"correctionAmplitude",h.correctionAmplitude},{"continuousInput",h.continuousInput},{"referencePeakRateMillimetersPerSecond",h.referencePeakRateMillimetersPerSecond},{"historyScaleSeconds",h.historyScaleSeconds},{"referenceHistorySeconds",h.referenceHistorySeconds},{"coolingTimeSeconds",h.coolingTimeSeconds},{"activityThresholdRatio",h.activityThresholdRatio}};};
            Json metadata={{"format","RS2026.DynamicSurface"},{"version",1},{"units","meters/seconds"},
                {"modelName",session.modelName.toStdString()},{"sourcePath",session.sourcePath.toStdString()},
                {"simulationSeconds",s.simulationSeconds},{"elapsedSeconds",s.elapsedSeconds},
                {"acceptedSteps",s.acceptedSteps},{"rejectedSteps",s.rejectedSteps},
                {"maximumGrowthMeters",s.options.maximumGrowthMeters},{"maximumNormalChangeDegrees",s.options.maximumNormalChangeDegrees},{"maximumIntervalSeconds",s.options.maximumIntervalSeconds},
                {"bvhOcclusion",session.dynamicResultOptions.enableBvhOcclusion},{"heatHistory",session.dynamicResultOptions.enableHistoryCorrection},
                {"history",history(session.dynamicResultOptions.history)},
                {"initialHistorySeconds",session.dynamicResultOptions.initialHistorySeconds},
                {"shadowBiasMeters",session.dynamicResultOptions.shadowBiasMeters},
                {"trajectoryTimeStepSeconds",std::min(std::max(1e-5,session.dynamicResultOptions.base.timeStep),s.options.maximumIntervalSeconds)},
                {"deposition",{{"calibrationModelId",p.calibrationModelId},{"amplitudeMillimeters",p.amplitudeMillimeters},
                    {"rotationRadians",p.rotationRadians},{"phiOffsetRadians",p.phiOffsetRadians},{"psiOffsetRadians",p.psiOffsetRadians},
                    {"sigmaPhiRadians",p.sigmaPhiRadians},{"sigmaPsiRadians",p.sigmaPsiRadians},{"referenceDistanceMeters",p.referenceDistanceMeters},
                    {"referenceAngleDegrees",p.referenceAngleDegrees},{"referenceExposureSeconds",p.referenceExposureSeconds},
                    {"useCalibratedHistory",p.useCalibratedHistory},{"calibratedHistory",history(p.calibratedHistory)},
                    {"useCalibratedEfficiencies",p.useCalibratedEfficiencies},{"efficiencyIncludesGeometry",p.efficiencyIncludesGeometry},
                    {"distanceEfficiency",curve(p.distanceEfficiency)},{"angleEfficiency",curve(p.angleEfficiency)},
                    {"useCalibratedDensity",p.useCalibratedDensity},{"distanceDensity",curve(p.distanceDensity)},{"angleDensity",curve(p.angleDensity)},
                    {"densityDistanceMinMm",p.densityDistanceMinMm},{"densityDistanceMaxMm",p.densityDistanceMaxMm},
                    {"densityAngleMinDegrees",p.densityAngleMinDegrees},{"densityAngleMaxDegrees",p.densityAngleMaxDegrees}}}};
            metadata["process"]={{"id",session.dynamicResultProcess.id},{"flowRate",session.dynamicResultProcess.flowRate},
                {"atomizationFactor",session.dynamicResultProcess.atomizationFactor},{"materialScale",session.dynamicResultProcess.materialScale}};
            metadata["vertexLayout"]={"x","y","z","nx","ny","nz","thickness","displacement","original_x","original_y","original_z","original_nx","original_ny","original_nz","historySeconds","lastUpdateSeconds","historyFactor"};
            metadata["vertexEncoding"]="little-endian float64";
            metadata["triangleEncoding"]="3 little-endian uint32 indices";
            for(const auto& g:p.gaussianComponents) metadata["deposition"]["gaussianComponents"].push_back({g.coefficient,g.phi0,g.psi0,g.sigmaPhi,g.sigmaPsi});
            auto vector=[](const Eigen::Vector3d& v){return Json::array({v.x(),v.y(),v.z()});};
            for(int row=0;row<4;++row)for(int col=0;col<4;++col) {
                metadata["worldFromModel"].push_back(session.dynamicWorldFromModel.matrix()(row,col));
                metadata["linkFromTool"].push_back(session.dynamicResultTool.T_link_tool.matrix()(row,col));
            }
            metadata["sprayDirectionLocal"]=vector(session.dynamicResultTool.sprayDirectionLocal);
            metadata["powderFeedDirectionLocal"]=vector(session.dynamicResultTool.powderFeedDirectionLocal);
            for(const auto& point:session.trajectory.flattenedPoints()) {
                Json pose=Json::array();for(int row=0;row<4;++row)for(int col=0;col<4;++col)pose.push_back(point.tcpPose.matrix()(row,col));
                metadata["trajectory"].push_back({{"time",point.time},{"pose",pose},{"sprayEnabled",point.sprayEnabled},{"processId",point.processId}});
            }
            data.writeRawData("DSURF001",8); const auto text=metadata.dump(); data<<QByteArray(text.data(),static_cast<int>(text.size()));
            data<<static_cast<quint64>(count)<<static_cast<quint64>(s.substrate->triangleIndices.size()/3);
        }
        auto writeVector=[&](const Eigen::Vector3d& v){data<<v.x()<<v.y()<<v.z();};
        for(std::size_t i=0;i<count;++i) {
            writeVector(s.positions[i]);writeVector(s.normals[i]);data<<s.thicknessMeters[i]<<(s.positions[i]-s.substrate->samples[i].position).norm();writeVector(s.substrate->samples[i].position);
            if(!ply) {writeVector(s.substrate->samples[i].normal);data<<static_cast<double>(s.historySeconds.at(i))<<static_cast<double>(s.lastUpdateSeconds.at(i))<<static_cast<double>(s.historyFactors.at(i));}
        }
        for(std::size_t f=0;f<s.substrate->triangleIndices.size();f+=3) {
            if(ply)data<<quint8(3);
            for(int c=0;c<3;++c)data<<static_cast<quint32>(s.substrate->triangleIndices[f+c]);
        }
        if(data.status()!=QDataStream::Ok) return fail(QStringLiteral("写入失败，原文件未被覆盖。"));
        if(!file.commit()) return fail(file.errorString());
        return true;
    }
}
