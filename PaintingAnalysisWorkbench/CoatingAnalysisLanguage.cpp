#include "CoatingAnalysisLanguage.h"

#include <SprayThicknessPrediction/AlgorithmReproduction.h>

#include <QVector>

#include <algorithm>

namespace robot_qt_viewer
{
    namespace
    {
        struct TranslationEntry
        {
            const char* english;
            const char* chinese;
        };

        const QVector<TranslationEntry>& entries()
        {
            static const QVector<TranslationEntry> value = {
                {"Thickness Prediction", "厚度预测"},
                {"Thickness Simulation", "厚度仿真"},
                {"Algorithm Reproduction", "算法复现"},
                {"Online Thickness Prediction", "在线厚度预测"},
                {"Method principle", "算法原理"},
                {"Scene", "场景"},
                {"Scene source", "场景来源"},
                {"Imported model", "已导入模型"},
                {"Generated plate stack", "生成多层平板"},
                {"Plate count", "平板数量"},
                {"Z spacing", "Z 向间距"},
                {"Generate scene", "生成场景"},
                {"Generate trajectory", "生成轨迹"},
                {"Apply and generate recommended setup", "应用推荐设置并生成"},
                {"Run benchmark (1 warm-up + 5 runs)", "批量效率实验（预热1次＋正式5次）"},
                {"Apply recommended parameters", "应用推荐参数"},
                {"Recommended scene: side", "推荐场景：边长"},
                {"Recommended line scan for generated plates", "生成平板的推荐线扫轨迹"},
                {"Recommended trajectory: no tested preset in this calibration file.", "推荐轨迹：此标定文件没有经过测试的预设。"},
                {"Recommended trajectory unavailable: ", "无法读取推荐轨迹："},
                {"Stand-off:", "喷涂距离："},
                {"incidence:", "入射角："},
                {"speed:", "扫描速度："},
                {"Point interval:", "轨迹点间隔："},
                {"overrun:", "越界长度："},
                {"round trips:", "往返遍数："},
                {"Test scene: side", "测试场景：边长"},
                {"; count ", "；平板数 "},
                {"; spacing ", "；间距 "},
                {"; cell ", "；网格单元 "},
                {"Local test preset; not a published optimum.", "本地测试预设，并非论文给出的最优工况。"},
                {"Custom-file preset; calculation not verified.", "自定义标定文件的预设，尚未验证计算结果。"},
                {"Scene differs from the test scene; result is not guaranteed.", "当前场景与测试场景不同，不能保证结果。"},
                {"Trajectory source", "轨迹来源"},
                {"Imported trajectory", "已导入轨迹"},
                {"Generated point spray", "生成点喷涂轨迹"},
                {"Generated line scan", "生成线扫轨迹"},
                {"Target span", "目标区域宽度"},
                {"Scan start XY", "扫描起点 XY"},
                {"Scan end XY", "扫描终点 XY"},
                {"Generated multi-layer plate mesh", "生成的多层平板网格"},
                {"Generated timed spray trajectory", "生成的定时喷涂轨迹"},
                {"Published algorithm", "已发表算法"},
                {"Calibration file", "标定文件"},
                {"Model input", "模型输入"},
                {"Trajectory input", "轨迹输入"},
                {"Required paper-specific JSON calibration", "需要论文专用 JSON 标定文件"},
                {"Browse...", "浏览..."},
                {"Create template...", "创建模板..."},
                {"Select STL Surface", "选择 STL 表面"},
                {"Select Substrate Model", "选择基底模型"},
                {"Select Surface Mesh", "选择表面网格"},
                {"Select Pose Sequence", "选择位姿序列"},
                {"Export result", "导出结果"},
                {"Current BVH-GPU method", "当前 BVH-GPU 方法"},
                {"Tzinava et al. (2020)", "Tzinava 等（2020）"},
                {"Wu et al. (2020)", "Wu 等（2020）"},
                {"Fuke et al. (2005)", "Fuke 等（2005）"},
                {"Vanerio et al. (2021)", "Vanerio 等（2021）"},
                {"Dynamic surface evolution (2026)", "动态表面演化（2026）"},
                {"Triangulated workpiece surface", "三角化工件表面"},
                {"Timed robot spray poses", "带时间的机器人喷涂位姿"},
                {"Triangulated STL surface with beam-based subdivision", "采用喷束细分的 STL 三角表面"},
                {"Gun trajectory with adaptive internal time stepping", "采用内部自适应时间步的喷枪轨迹"},
                {"CAD or mesh collision surface with deposited cylinders", "CAD/网格碰撞表面与离散沉积圆柱"},
                {"Original nozzle poses without internal resampling", "不进行内部重采样的原始喷嘴位姿"},
                {"Normal height (display only)", "法向高度（仅显示）"},
                {"Apply", "应用"},
                {"Surface-meshed CAD polygons evaluated at centroids", "在面心计算的 CAD 表面多边形网格"},
                {"Relative vapor-source/workpiece pose sequence", "蒸发源与工件的相对位姿序列"},
                {"Remeshed evolving STL triangular surface", "动态重网格化的 STL 三角表面"},
                {"Nozzle trajectory for interval-wise surface growth", "用于逐时间区间表面生长的喷嘴轨迹"},
                {"STL surface with deposited cylinders and point-cloud reconstruction", "包含沉积圆柱与点云重建的 STL 表面"},
                {"Nozzle trajectory processed in deposition batches", "按沉积批次处理的喷嘴轨迹"},
                {"Open workpiece model", "打开工件模型"},
                {"Open target-point source model", "打开目标点源模型"},
                {"Open Tzinava STL surface", "打开 Tzinava STL 表面"},
                {"Open Wu substrate model", "打开 Wu 基底模型"},
                {"Open Fuke surface mesh", "打开 Fuke 表面网格"},
                {"Open Vanerio STL surface", "打开 Vanerio STL 表面"},
                {"Open dynamic-surface STL model", "打开动态表面 STL 模型"},
                {"Open timed spray-gun trajectory", "打开带时间的喷枪轨迹"},
                {"Open Tzinava gun trajectory", "打开 Tzinava 喷枪轨迹"},
                {"Open Wu nozzle trajectory", "打开 Wu 喷嘴轨迹"},
                {"Open Fuke relative-pose trajectory", "打开 Fuke 相对位姿轨迹"},
                {"Open Vanerio nozzle trajectory", "打开 Vanerio 喷嘴轨迹"},
                {"Open dynamic-surface nozzle trajectory", "打开动态表面喷嘴轨迹"},
                {"Reference peak rate", "参考峰值厚度速率"},
                {"Reference distance", "参考距离"},
                {"Sigma X", "X 方向标准差"},
                {"Sigma Y", "Y 方向标准差"},
                {"Beam radius", "喷束半径"},
                {"Distance exponent", "距离指数"},
                {"Angle/plume exponent", "角度/羽流指数"},
                {"Shadow occlusion", "阴影遮挡"},
                {"Run reproduction", "运行复现"},
                {"No reproduction result yet.", "尚无算法复现结果。"},
                {"Select a published algorithm and run its reproduction.", "请选择已发表算法并运行复现。"},
                {"Current method (GPU)", "本文方法（GPU）"},
                {"Algorithm reproduction mode active.", "算法复现模式已激活。"},
                {"Exited algorithm reproduction mode.", "已退出算法复现模式。"},
                {"Canceling algorithm reproduction...", "正在取消算法复现..."},
                {"Algorithm reproduction failed.", "算法复现失败。"},
                {"Algorithm reproduction canceled.", "算法复现已取消。"},
                {"Reproduction inputs changed. Run the selected method again.", "复现输入已更改，请重新运行所选方法。"},
                {"Scene: not generated.", "场景：尚未生成。"},
                {"Trajectory: not generated.", "轨迹：尚未生成。"},
                {"Configure the Scene tab and generate it.", "请在“场景”页完成设置并生成场景。"},
                {"Configure the Trajectory tab and generate it.", "请在“轨迹”页完成设置并生成轨迹。"},
                {"Scene settings changed. Generate the scene again.", "场景设置已更改，请重新生成场景。"},
                {"Trajectory settings changed. Generate the trajectory again.", "轨迹设置已更改，请重新生成轨迹。"},
                {"The generated trajectory depends on the scene. Generate the trajectory again.", "生成轨迹依赖当前场景，请重新生成轨迹。"},
                {"Scene model changed. Generate the scene again.", "场景模型已更改，请重新生成场景。"},
                {"Trajectory data changed. Generate the trajectory again.", "轨迹数据已更改，请重新生成轨迹。"},
                {"Generate the scene and trajectory before running an algorithm.", "请先分别生成场景和轨迹，再运行算法。"},
                {"Scene ready:", "场景已就绪："},
                {"Trajectory ready:", "轨迹已就绪："},
                {"generated points.", "个生成轨迹点。"},
                {"Load an analysis model or select the generated plate stack.", "请加载分析模型或选择生成多层平板。"},
                {"Load a spray trajectory or select a generated trajectory.", "请加载喷涂轨迹或选择生成轨迹。"},
                {"The viewport is unavailable.", "视口不可用。"},
                {"Failed to display the generated plate stack.", "无法显示生成的多层平板。"},
                {"Plate side, grid cell, plate count and spacing must be positive.", "平板边长、网格单元、平板数量和间距必须为正数。"},
                {"The plate stack exceeds the 20,000,000 vertex safety limit. Increase the grid cell size or reduce the plate count.", "多层平板超过 20,000,000 个顶点的安全上限，请增大网格单元或减少平板数量。"},
                {"Generated trajectory", "生成轨迹"},
                {"Calibration template created. Replace every null value before running.", "标定模板已创建。运行前请填写所有 null 参数。"},
                {"Spray Simulation", "喷涂仿真"},
                {"Result Validation", "结果验证"},
                {"Local Debug Display", "局部调试显示"},
                {"Deposition Model", "沉积模型"},
                {"Spray direction", "喷涂方向"},
                {"Powder feed direction", "送粉方向"},
                {"Deposition", "沉积"},
                {"Trajectory Sampling", "轨迹采样"},
                {"Thermal History", "热历史"},
                {"Computation", "计算"},
                {"Total (first frame)", "总耗时（首帧显示）"},
                {"Preparation", "任务准备"},
                {"Algorithm", "算法核心"},
                {"Conversion", "结果转换"},
                {"Overlay setup", "云图提交"},
                {"First-frame wait", "首帧等待"},
                {"Scheduling/other", "调度及其他"},
                {"Input vertices", "输入顶点数"},
                {"Input triangles", "输入三角面片数"},
                {"Display vertices", "显示顶点数"},
                {"Display triangles", "显示三角面片数"},
                {"Pending", "等待中"},
                {"Spray samples", "喷涂采样数"},
                {"Evaluated elements", "计算单元数"},
                {"Method candidates", "方法候选数"},
                {"Visibility queries", "可见性查询数"},
                {"Occluded elements", "遮挡单元数"},
                {"Analysis", "分析"},
                {"Simulation", "仿真"},
                {"Prediction Input", "预测输入"},
                {"Prediction", "预测"},
                {"Workpiece", "工件"},
                {"Visualize", "可视化"},
                {"Model", "模型"},
                {"Models", "模型列表"},
                {"Trajectory", "轨迹"},
                {"Waypoints", "轨迹点列表"},
                {"Spray Points", "喷涂点"},
                {"Thickness Cloud", "厚度云图"},
                {"Thickness Pick", "厚度拾取"},
                {"Cylinder surface", "圆柱面"},
                {"Rotation axis", "旋转轴"},
                {"Local sector", "局部扇区"},
                {"Profile line", "母线"},
                {"Calculation spray points", "参与计算的喷涂点"},
                {"Complete - all spray points", "完整模型 - 全部喷涂点"},
                {"Local - all spray points", "局部模型 - 全部喷涂点"},
                {"Complete - spatial filtering", "完整模型 - 空间筛选"},
                {"Local - spatial filtering", "局部模型 - 空间筛选"},
                {"Axisymmetric profile - spatial filtering", "轴对称母线 - 空间筛选"},
                {"Complete - candidate vertices", "完整模型 - 候选顶点"},
                {"Adaptive mesh - candidate filtering", "自适应网格 - 候选筛选"},
                {"Local candidates - full BVH", "局部候选顶点 - 完整 BVH"},
                {"Complete model + all spray points", "完整模型 + 全部喷涂点"},
                {"Local model + all spray points", "局部模型 + 全部喷涂点"},
                {"Complete model + spatial-filtered spray points", "完整模型 + 空间筛选喷涂点"},
                {"Local model + spatial-filtered spray points", "局部模型 + 空间筛选喷涂点"},
                {"Axisymmetric profile samples + spatial-filtered spray points", "轴对称母线采样点 + 空间筛选喷涂点"},
                {"Complete model + spatial-filtered spray points + candidate vertices", "完整模型 + 空间筛选喷涂点 + 候选顶点"},
                {"Dense selected region + sparse outside mesh + candidate spray points/vertices", "选定区域密集网格 + 外部稀疏网格 + 候选喷涂点/顶点"},
                {"Selected local vertices + candidate spray points/vertices + complete-model occlusion BVH", "选定局部顶点 + 候选喷涂点/顶点 + 完整模型遮挡 BVH"},
                {"Paper Gaussian (GPU)", "论文高斯模型（GPU）"},
                {"Online Thickness", "在线厚度预测"},
                {"Scene submission rate:", "场景提交帧率："},
                {"Viewport draw rate:", "视口绘制帧率："},
                {"Cloud submission rate:", "云图提交帧率："},
                {"Frame interval P95:", "帧间隔 P95："},
                {"Longest frame interval:", "最长帧间隔："},
                {"Screen refresh rate:", "屏幕刷新率："},
                {"Unlimited", "不限制"},
                {"Display FPS limit", "显示帧率上限"},
                {"Show deposition influence", "显示沉积作用域"},
                {"Influence display threshold", "作用域显示阈值"},
                {"Percentage of the angular Gaussian peak. Display only; does not change deposition. Preview remains visible after stopping powder.",
                    "相对于角度高斯峰值的百分比。仅影响显示，不改变沉积计算；停粉后仍可预览作用域。"},
                {"Cloud presentation rate:", "有效云图呈现帧率："},
                {"Compute completion rate:", "计算完成帧率："},
                {"Average frame interval:", "平均帧间隔："},
                {"Frame interval P99:", "帧间隔 P99："},
                {"Compute backlog:", "计算积压："},
                {"Preparing online model and GPU resources...", "正在准备在线模型和 GPU 资源……"},
                {"Timing detection:not started", "耗时检测：尚未开始"},
                {"Timing detection:running", "耗时检测：运行中"},
                {"Timing detection:stopped", "耗时检测：已结束"},
                {"Timing anomalies:", "耗时异常数："},
                {"Waiting for timing samples...", "等待耗时采样…"},
                {"Startup / warm-up:", "启动／预热耗时："},
                {"Last long frame:", "最近异常帧间隔："},
                {"Current wait:", "当前持续等待："},
                {"Suspected stage:", "疑似耗时环节："},
                {"; recent ", "；近期平均 "},
                {"Diagnostic CSV:", "诊断 CSV："},
                {"Diagnostic write failed:", "诊断文件写入失败："},
                {"Dropped diagnostic rows:", "未写入的诊断行数："},
                {"Input sampling / submission", "输入采样／提交"},
                {"Input backlog / worker queue", "输入积压／后台排队"},
                {"Backend CPU preparation", "后台 CPU 准备"},
                {"GPU input upload", "GPU 输入上传"},
                {"Compute submission", "计算指令提交"},
                {"GPU compute", "GPU 计算"},
                {"GPU wait / thickness readback", "GPU 等待／厚度回读"},
                {"GPU backend accumulation / readback", "GPU 后台累计／回读"},
                {"Thickness conversion", "厚度结果转换"},
                {"GPU timer result wait", "GPU 计时结果等待"},
                {"Uniformity statistics", "均匀性统计"},
                {"Cloud mapping", "云图顶点映射"},
                {"GUI event delivery", "界面事件投递"},
                {"Previous frame presentation", "上一帧呈现等待"},
                {"Display pacing", "显示节奏等待"},
                {"GUI result handling", "界面结果接收／处理"},
                {"Pose submission", "位姿提交"},
                {"Cloud conversion / upload / copy", "云图转换／上传／复制"},
                {"Information / legend update", "信息栏／标尺更新"},
                {"Repaint scheduling wait", "重绘调度等待"},
                {"Scene update", "场景更新"},
                {"Draw submission", "绘制指令提交"},
                {"Qt swap / compositor wait", "Qt 帧交换／窗口合成等待"},
                {"GUI event loop delay", "界面事件循环延迟"},
                {"Model / GUI startup", "模型／界面启动准备"},
                {"Compute context startup", "计算上下文创建"},
                {"BVH / initial GPU preparation", "BVH／GPU 初始数据准备"},
                {"Unmeasured scheduling / input wait", "未归属的调度／输入等待"},
                {"Suspected stages are timing evidence, not a verified root cause. GPU compute overlaps readback wait; do not add all stage times together.",
                    "疑似环节由耗时推断，仍需结合日志确认原因。GPU 计算与回读等待有重叠，各阶段耗时不能直接相加。"},
                {"Rates count Qt window submissions and draw calls, not physical screen frames. Frame intervals reveal pauses hidden by average rates.", "帧率统计的是 Qt 窗口提交和绘制次数，并非屏幕实际显示帧数。帧间隔可反映平均帧率掩盖的停顿。"},
                {"First cloud frame:", "首张云图耗时："},
                {"Waiting for first cloud frame...", "等待首张云图…"},
                {"Not started", "尚未开始"},
                {"Load the debug model in Thickness Prediction first.",
                    "请先在厚度预测中加载调试模型。"},
                {"Pose source", "位姿来源"},
                {"Virtual validation", "虚拟验证"},
                {"Live RWS", "实机 RWS"},
                {"Rotate workpiece", "零件旋转"},
                {"Reciprocate spray gun", "喷枪往返移动"},
                {"Spray point movement", "喷涂点移动"},
                {"Workpiece rotation", "模型旋转"},
                {"Start moving", "开始移动"},
                {"Stop moving", "停止移动"},
                {"Start rotating", "开始旋转"},
                {"Stop rotating", "停止旋转"},
                {"Virtual motion active; spraying is stopped.", "虚拟运动中；喷涂已停止。"},
                {"Virtual motion paused; current poses retained.", "虚拟运动已暂停，保持当前位姿。"},
                {"Movement direction (world)", "移动方向（世界坐标）"},
                {"One-way distance", "单程距离"},
                {"Motion parameters remain editable. Press Enter or finish editing to apply. Speed and axis changes preserve the current pose. Direction/distance changes move toward the new target before reciprocating. Editing Start repositions the gun.",
                    "运动参数可实时修改，按回车或完成编辑后生效。修改速度、转速或旋转轴时保持当前位姿；修改方向或行程后先到达新目标，再沿新路径往返；修改起点坐标会重新定位喷涂点。"},
                {"Moves between the start and target at constant speed; reverses immediately. Gun orientation stays fixed. Can run together with workpiece rotation.",
                    "在起点与目标位置之间匀速往返，到端点立即换向；喷枪姿态不变，可与零件旋转同时启用。"},
                {"Virtual motion", "虚拟运动"},
                {"Fixed gun / rotating workpiece", "喷枪固定 / 零件旋转"},
                {"Fixed workpiece / moving gun", "零件固定 / 喷枪移动"},
                {"Rotation axis", "旋转轴"},
                {"Rotation speed", "旋转速度"},
                {"Random (changing axis)", "随机（轴持续变化）"},
                {"Axis direction changes smoothly every 2 s.", "轴方向每 2 秒平滑转向新的随机方向。"},
                {"Current axis: ", "当前轴方向："},
                {"Random seed: ", "随机种子："},
                {"Start X", "起点 X"},
                {"Start Y", "起点 Y"},
                {"Start Z", "起点 Z"},
                {"End X", "终点 X"},
                {"End Y", "终点 Y"},
                {"End Z", "终点 Z"},
                {"Gun speed", "喷枪速度"},
                {"Reset thickness", "重置厚度"},
                {"Start spraying", "开始喷涂"},
                {"Stop spraying", "停止喷涂"},
                {"Controls thickness accumulation only; no equipment command is sent.",
                    "仅控制软件厚度累计，不向实际设备发送命令。"},
                {"Ready to start online prediction.", "已准备开始在线厚度预测。"},
                {"Start powder (software)", "开始送粉（仅软件）"},
                {"Stop powder (software)", "停止送粉（仅软件）"},
                {"Waiting for live RWS poses.", "等待 RWS 实时位姿。"},
                {"Online accumulation active.", "在线厚度累计中。"},
                {"Online accumulation paused.", "在线厚度累计已暂停。"},
                {"Spraying stopped; finalizing submitted results.", "已停止喷涂，正在完成已提交批次的结果。"},
                {"Simulated time: ", "已计算仿真时长："},
                {"Unsimulated wall time: ", "未补算的现实时间："},
                {"Live RWS pose stream timed out; online accumulation paused.",
                    "RWS 实时位姿已超时，在线厚度累计已暂停。"},
                {"Deposition directions changed. Run prediction to apply them.", "沉积方向已更改，请重新运行厚度预测。"},
                {"Original Points", "原始点"},
                {"Resample by Time Step", "按时间步重采样"},
                {"BVH shadow occlusion", "BVH 阴影遮挡"},
                {"Thermal exposure history", "热历史修正"},
                {"Override grid cell size", "覆盖网格单元尺寸"},
                {"Grid cell (mm)", "网格单元（毫米）"},
                {"Outside target vertex ratio", "目标区域外顶点比例"},
                {"Rotation axis: Not configured", "旋转轴：未配置"},
                {"Sectors", "扇区数量"},
                {"Profile samples", "母线采样点"},
                {"Pick surface and fit axis", "拾取圆柱面并拟合轴"},
                {"Select profile prediction region", "选择母线预测区域"},
                {"Preview local prediction inputs", "预览局部预测输入"},
                {"Refresh local input preview", "刷新局部输入预览"},
                {"Select prediction region", "选择预测区域"},
                {"Select Profile Prediction Region", "选择母线预测区域"},
                {"Draw a closed freeform boundary over the profile region. The selected boundary is predicted; outside it, final thickness is set to 0.", "请在母线区域上绘制闭合自由曲线。选中边界内参与预测，边界外最终厚度设为 0。"},
                {"Select entire profile", "选择整个母线"},
                {"Clear selection", "清空选择"},
                {"Horizontal: radius (mm)   Vertical: axis coordinate (mm)", "横轴：半径（毫米）   纵轴：轴向坐标（毫米）"},
                {"Waypoint", "轨迹点"},
                {"Index", "索引"},
                {"Time", "时间"},
                {"Duration next", "到下一点时长"},
                {"Position", "位置"},
                {"Orientation", "姿态"},
                {"Spray", "喷涂"},
                {"Process ID", "工艺 ID"},
                {"Target dist.", "目标距离"},
                {"Target normal", "目标法向"},
                {"Region ID", "区域 ID"},
                {"On", "开启"},
                {"Off", "关闭"},
                {"(last waypoint)", "（最后一个轨迹点）"},
                {"(none)", "（无）"},
                {"Point spray", "点喷涂"},
                {"Line scan", "线扫喷涂"},
                {"Experiment", "实验类型"},
                {"Plate side", "平板边长"},
                {"Grid cell", "网格单元"},
                {"Spray distance", "喷涂距离"},
                {"Trajectory overrun", "轨迹超出距离"},
                {"Incidence angle", "入射角"},
                {"Azimuth", "方位角"},
                {"Tool roll (local Z)", "喷枪滚转角（局部 Z）"},
                {"Point duration", "点喷涂停留时间"},
                {"Scan speed", "扫描速度"},
                {"Scan passes (round trips)", "扫描遍数（往返次数）"},
                {"Entry speed", "进入速度"},
                {"Exit speed", "离开速度"},
                {"Trajectory point interval", "轨迹点时间间隔"},
                {"Simulation plate", "仿真平板"},
                {"Simulation trajectory", "仿真轨迹"},
                {"Simulation status", "仿真状态"},
                {"No simulation thickness result yet.", "尚无仿真厚度结果。"},
                {"Waiting for simulation...", "等待仿真..."},
                {"Scan start X", "扫描起点 X"},
                {"Scan start Y", "扫描起点 Y"},
                {"Scan end X", "扫描终点 X"},
                {"Scan end Y", "扫描终点 Y"},
                {"Run simulation", "运行仿真"},
                {"Export CSV", "导出 CSV"},
                {"Target", "目标"},
                {"Data", "数据"},
                {"Debug Model", "调试模型"},
                {"Debug Trajectory", "调试轨迹"},
                {"Load Selected Trajectory", "加载所选轨迹"},
                {"Load Saved Trajectory", "加载已保存轨迹"},
                {"Load the configured debug spray trajectory.", "加载预设的调试喷涂轨迹。"},
                {"Load the selected planned trajectory or the selected dual/three-trajectory optimization data for the workpiece.", "加载所选工件的规划轨迹或双/三轨迹优化数据。"},
                {"Choose a workpiece model file.", "选择工件模型文件。"},
                {"Choose a spray trajectory file.", "选择喷涂轨迹文件。"},
                {"Load the saved rotation-body trajectory for the selected workpiece.", "加载当前工件已保存的回转体轨迹。"},
                {"Select Model File", "选择模型文件"},
                {"Select Trajectory File", "选择轨迹文件"},
                {"Sampling", "采样方式"},
                {"Time step", "时间步长"},
                {"Apply Sampling", "应用采样"},
                {"Original trajectory points selected for prediction.", "已选择原始轨迹点用于预测。"},
                {"Sampling parameters changed. Click Apply Sampling to update the trajectory preview.", "采样参数已更改，请点击“应用采样”更新轨迹预览。"},
                {"Trajectory loaded. Click Apply Sampling to build the resampled preview.", "轨迹已加载，请点击“应用采样”生成重采样预览。"},
                {"Failed to resample the trajectory.", "轨迹重采样失败。"},
                {"Start Prediction", "开始预测"},
                {"Cancel", "取消"},
                {"Set as Reference", "设为基准"},
                {"Clear Reference", "清空基准"},
                {"Check Against Reference", "与基准对比"},
                {"Reference: not set", "基准：未设置"},
                {"Waypoint Info...", "轨迹点信息..."},
                {"Set as Prediction Workpiece", "设为预测工件"},
                {"Show / Hide Model", "显示/隐藏模型"},
                {"Delete Model", "删除模型"},
                {"Delete Trajectory", "删除轨迹"},
                {"Delete this model from the current project and 3D scene?", "确定从当前项目和三维场景中删除该模型吗？"},
                {"Delete the loaded trajectory and its prediction result?", "确定删除已加载轨迹及其预测结果吗？"},
                {"Clear Result", "清空结果"},
                {"No model loaded.", "尚未加载模型。"},
                {"No trajectory loaded.", "尚未加载轨迹。"},
                {"No thickness result yet.", "尚无厚度结果。"},
                {"No computation data yet.", "尚无计算数据。"},
                {"Simulation inactive.", "仿真未激活。"},
                {"Enter simulation mode before running it.", "请先进入仿真模式。"},
                {"Build the simulation plate before running it.", "请先生成仿真平板。"},
                {"A thickness prediction is already running.", "厚度预测任务正在运行。"},
                {"The simulation trajectory is empty. Rebuild the plate.", "仿真轨迹为空，请重新生成平板。"},
                {"Failed to sample the simulation trajectory.", "仿真轨迹采样失败。"},
                {"Plate side, cell size and trajectory point interval must be positive.", "平板边长、网格单元和轨迹点时间间隔必须为正数。"},
                {"No reference result.", "无基准结果。"},
                {"Rotation axis: ", "旋转轴："},
                {"Manual axis", "手动轴"},
                {"Fitted axis", "拟合轴"},
                {"Amplitude", "幅值"},
                {"Rotation", "旋转"},
                {"Phi offset", "Phi 偏移"},
                {"Psi offset", "Psi 偏移"},
                {"Sigma phi", "Sigma Phi"},
                {"Sigma psi", "Sigma Psi"},
                {"Ref distance", "参考距离"},
                {"Ref angle", "参考角度"},
                {"Ref exposure", "参考曝光"},
                {"Correction amp", "修正幅值"},
                {"History scale", "历史尺度"},
                {"Ref history", "参考历史"},
                {"Cooling time", "冷却时间"},
                {"Activity thr", "活跃阈值"},
                {"Wall time", "墙钟时间"},
                {"Volume", "体积"},
                {"Backend", "后端"},
                {"BVH", "BVH"},
                {"Upload", "上传"},
                {"Dispatch", "调度"},
                {"Pure GPU", "纯 GPU"},
                {"Readback", "回读"},
                {"Batch", "批次"},
                {"Inputs", "输入"},
                {"Mapping", "映射"},
                {"Axis map", "轴映射"},
                {"Axis bind", "轴绑定"},
                {"Axis zero", "轴零值"},
                {"Axis invalid", "轴无效"},
                {"Axis parts", "轴分段"},
                {"Grid", "网格"},
                {"Grid mode", "网格模式"},
                {"Grid size", "网格尺寸"},
                {"Grid dim", "网格维度"},
                {"Grid cells", "网格单元数"},
                {"Cell refs", "单元引用"},
                {"Vertex refs", "顶点引用"},
                {"GPU vertices", "GPU 顶点"},
                {"Skipped zero", "跳过零值"},
                {"Grid cache", "网格缓存"},
                {"cache hit", "命中缓存"},
                {"automatic", "自动"},
                {"manual", "手动"},
                {"built", "已构建"},
                {"hit", "命中"},
                {"Vertex", "顶点"},
                {"Thickness", "厚度"},
                {"Thickness display range", "厚度显示范围"},
                {"Double-click the color bar to edit its range.", "双击颜色条可修改显示范围。"},
                {"Minimum", "最小值"},
                {"Maximum", "最大值"},
                {"Apply", "应用"},
                {"Restore automatic range", "恢复自动范围"},
                {"Close", "关闭"},
                {"Minimum must be less than maximum.", "最小值必须小于最大值。"},
                {"Range", "范围"},
                {"Mean", "平均值"},
                {"Variance", "方差"},
                {"Std. dev.", "标准差"},
                {"CV", "变异系数"},
                {"No vertices in the display range.", "显示范围内没有顶点。"},
                {"Relative error", "相对误差"},
                {"Reference ready", "基准已就绪"},
                {"Vertices", "顶点数"},
                {"Active", "有效数"},
                {"Min", "最小"},
                {"Max", "最大"},
                {"Average", "平均"},
                {"Coverage", "覆盖率"},
                {"Under-coated", "欠涂"},
                {"Over-coated", "过涂"},
                {"Points", "点数"},
                {"Duration", "时长"},
                {"Path length", "路径长度"},
                {"Avg speed", "平均速度"},
                {"Poses", "位姿数"},
                {"No robot", "未选择机器人"},
                {"Load a model and trajectory to begin.", "请加载模型和轨迹以开始。"},
                {"Run a prediction before setting a reference.", "请先运行预测，再设置基准。"},
                {"Set a reference result before checking this prediction.", "请先设置基准结果，再检查当前预测。"},
                {"Run prediction before checking it against the reference.", "请先运行预测，再与基准对比。"},
                {"Current prediction stored as the reference result.", "当前预测已保存为基准结果。"},
                {"Reference result cleared.", "基准结果已清空。"},
                {"Model loaded. Load a trajectory and run thickness prediction.", "模型已加载。请加载轨迹并运行厚度预测。"},
                {"Trajectory loaded. Ready for GPU thickness prediction.", "轨迹已加载，可以进行 GPU 厚度预测。"},
                {"Preparing GPU thickness prediction...", "正在准备 GPU 厚度预测..."},
                {"Canceling GPU thickness prediction...", "正在取消 GPU 厚度预测..."},
                {"GPU thickness prediction completed in %1 s.", "GPU 厚度预测完成，用时 %1 秒。"},
                {"GPU thickness prediction failed.", "GPU 厚度预测失败。"},
                {"GPU thickness prediction produced no result.", "GPU 厚度预测未产生结果。"},
                {"Failed to start the GPU thickness prediction task.", "无法启动 GPU 厚度预测任务。"},
                {"Thickness result cleared.", "厚度结果已清空。"},
                {"The model has no mesh vertices.", "模型没有网格顶点。"},
                {"The analysis model or viewport is no longer available.", "分析模型或视口已不可用。"},
                {"The analysis model, trajectory, or viewport is unavailable.", "分析模型、轨迹或视口不可用。"},
                {"Model file was not found: ", "未找到模型文件："},
                {"Spray trajectory was not found: ", "未找到喷涂轨迹："},
                {"Existing model reused. Run thickness prediction.", "已复用现有模型。请运行厚度预测。"},
                {"Click a cylindrical side face in the viewport.", "请在视口中点击圆柱侧面。"},
                {"Rotation axis changed. Select the profile prediction region again.", "旋转轴已改变，请重新选择母线预测区域。"},
                {"Rotation axis changed. Select the dense prediction region again.", "旋转轴已改变，请重新选择密集预测区域。"},
                {"Draw a closed freeform profile region before accepting.", "请先绘制闭合的自由曲线母线区域，再确认。"},
                {"Running spray simulation...", "正在运行喷涂仿真..."},
                {"Failed to start spray simulation.", "无法启动喷涂仿真。"},
                {"Simulation mode active.", "仿真模式已激活。"},
                {"Exited spray simulation mode.", "已退出喷涂仿真模式。"},
                {"Validation completed for %1 vertices.", "已完成 %1 个顶点的验证。"},
                {"Validation stopped: no active current vertices.", "验证已停止：当前没有有效顶点。"},
                {"Validation stopped: vertex index mapping differs.", "验证已停止：顶点索引映射不一致。"},
                {"Validation stopped: complete-model vertex counts differ.", "验证已停止：完整模型顶点数不一致。"},
                {"Validation unavailable", "无法验证"}
                ,{"Adaptive mesh cache hit; simplification skipped.", "自适应网格缓存命中，跳过简化。"}
                ,{"Spatial grid cache hit:", "空间网格缓存命中："}
                ,{"Spatial grid ready:", "空间网格已就绪："}
                ,{"Failed to load the spray trajectory.", "喷涂轨迹加载失败。"}
                ,{"Failed to display the thickness result.", "厚度结果显示失败。"}
                ,{"Failed to re-apply the thickness overlay.", "重新应用厚度覆盖层失败。"}
                ,{"Failed to apply the thickness display range.", "应用厚度显示范围失败。"}
                ,{"Failed to restore the automatic thickness range.", "恢复自动厚度范围失败。"}
                ,{"Thickness display range updated.", "厚度显示范围已更新。"}
                ,{"Automatic thickness display range restored.", "已恢复自动厚度显示范围。"}
                ,{"The thickness overlay is unavailable.", "厚度覆盖层不可用。"}
                ,{"Profile extraction failed: ", "母线提取失败："}
                ,{"Profile region preparation failed: ", "母线区域准备失败："}
                ,{"Local input preview failed: ", "局部输入预览失败："}
                ,{"Cylindrical fit failed: ", "圆柱拟合失败："}
                ,{"The picked object is not a workpiece.", "拾取的对象不是工件。"}
                ,{"Simulation export failed: ", "仿真导出失败："}
                ,{"Simulation thickness exported to ", "仿真厚度已导出到 "}
                ,{"Failed to start spray simulation.", "无法启动喷涂仿真。"}
                ,{"Failed to re-apply the thickness overlay.", "重新应用厚度覆盖层失败。"}
            };
            return value;
        }
    }

    QString coatingAnalysisTranslate(const QString& languageCode, const QString& text)
    {
        const bool chinese = languageCode.toLower().startsWith(QStringLiteral("zh"));
        QString result = text;
        // Build and order the immutable dictionary once, not on every UI reading.
        static const auto ordered = [] {
            QVector<const TranslationEntry*> pointers;
            for(const TranslationEntry& entry : entries()) pointers.push_back(&entry);
            std::sort(pointers.begin(), pointers.end(), [](const auto* left, const auto* right) {
                return std::char_traits<char>::length(left->english)
                    > std::char_traits<char>::length(right->english);
            });
            QVector<QPair<QString, QString>> strings;
            strings.reserve(pointers.size());
            for(const auto* entry : pointers) {
                strings.push_back({ QString::fromUtf8(entry->english),
                    QString::fromUtf8(entry->chinese) });
            }
            return strings;
        }();
        for(const auto& entry : ordered) {
            const QString& source = chinese ? entry.first : entry.second;
            const QString& target = chinese ? entry.second : entry.first;
            if(!source.isEmpty()) {
                result.replace(source, target);
            }
        }
        return result;
    }

    QString coatingAnalysisReproductionDescription(
        const QString& languageCode,
        spraythickness::ReproductionAlgorithmKind algorithm)
    {
        using Kind = spraythickness::ReproductionAlgorithmKind;
        const bool chinese = languageCode.toLower().startsWith(QStringLiteral("zh"));
        switch(algorithm) {
        case Kind::CurrentMethod:
            return chinese ? QString::fromUtf8(R"(计算对象
对完整工件网格的顶点逐点预测厚度。输入为顶点位置、法向量及带时间的喷枪位姿；喷涂方向和送粉方向确定喷枪局部坐标轴。

沉积模型
把顶点相对喷枪的位置投影到局部主、副轴，使用两个角度方向上的椭圆高斯分布计算喷束内的沉积强度。角度坐标由横向偏移与轴向距离共同决定，因此同一喷束在不同距离和倾斜表面上的落点形状会随几何关系变化。

几何与时间修正
每个喷涂点的贡献乘以时间权重、距离平方几何因子、表面入射角投影，以及标定得到的距离效率和角度效率。复现对比固定使用 BVH 射线遮挡：喷枪与顶点之间有其他面片阻挡时，该次贡献为零。可选热历史修正：根据先前有效喷涂和冷却时间调整后续沉积。

累计与适用范围
计算着色器按批次累加轨迹中各喷涂点对顶点的贡献，得到厚度场。该选项在算法复现页使用当前 GPU 方法及内置沉积参数，不读取论文标定 JSON。遮挡始终开启，热历史开关会改变结果；复杂曲面仍受原始网格分辨率与法向质量影响。)")
                : QStringLiteral(R"(Evaluation target
Thickness is predicted at the vertices of the complete workpiece mesh. Inputs are vertex positions and normals plus timed gun poses. The spray and powder-feed directions define the gun-local axes.

Deposition model
Each vertex is projected onto the gun-local major and minor axes. An elliptical Gaussian in two angular coordinates gives the beam intensity. The angles depend on lateral offset and axial distance, so the footprint changes geometrically with stand-off and surface inclination.

Geometry and time corrections
Each pose contributes according to its duration, inverse-square distance geometry, incidence projection, and calibrated distance and angle efficiency curves. BVH ray occlusion is always enabled for reproduction and rejects contributions blocked by another triangle. Optional thermal-history correction adjusts later deposition according to previous exposure and cooling.

Accumulation and scope
A GPU compute shader accumulates spray-pose contributions in batches into a vertex thickness field. In this reproduction tab the current method uses built-in deposition parameters, not a paper-specific JSON file. Occlusion stays enabled while the history switch can change the result; fine surface features still depend on mesh and normal quality.)");
        case Kind::Tzinava2020:
            return chinese ? QString::fromUtf8(R"(计算对象
输入 STL 三角表面与喷枪轨迹。首先按喷束尺度细分过长的三角形边，再以细分后的面片为厚度计算单元，而非直接在原始顶点上计算。

喷束与候选面片
逐时间步插值喷枪位置、方向和速度；仅将面心落在喷束内的面片列为候选，并计算喷涂距离、入射角、径向位置及喷斑相对移动速度。遮挡遵循论文第 2.2 节：将候选三角形正交投影到垂直喷轴的平面，若较近面片的投影与较远面片相交，则后者不累计厚度。这里不是面心射线遮挡。时间步长按第 2.5 节自适应选择。

沉积与累计
基础厚度由距离和入射角二维标定表插值得到。可选径向高斯因子进一步调整喷斑内的分布；运动喷涂按论文式 (9) 的喷斑速度函数修正，同一次连续可见覆盖只计一次；停驻喷涂按持续时间与标定停驻时间之比累计。论文没有说明连续覆盖期间选用哪个采样位置，程序取该次覆盖的最大贡献，这是明确的数值实现约定。

实现边界
标定文件包含喷束几何、厚度查表、速度系数和时间步参数。速度系数取自论文式 (8)，但随附的距离-角度厚度表是本地估计，不是论文的原始实验表；定量比较需要替换为相应实验数据。最终结果以细分面片厚度为基础呈现。)")
                : QStringLiteral(R"(Evaluation target
Inputs are an STL triangle surface and a gun trajectory. Long mesh edges are subdivided relative to beam size. Thickness is evaluated on the resulting faces rather than directly on original vertices.

Beam and candidate faces
Gun position, axis and velocity are interpolated in time. Only faces whose centroids lie inside the beam are candidates. Section 2.2 shadowing orthographically projects candidate triangles onto a plane normal to the spray axis; a nearer overlapping projected triangle hides a farther one. This is not a centroid ray test. Time steps follow Section 2.5.

Deposition and accumulation
A two-dimensional calibration table provides base thickness by distance and impact angle. An optional radial Gaussian shapes the footprint. Moving deposition uses the Eq. (9) spot-speed correction once per continuous visible pass; stationary spraying scales with dwell time. The paper does not specify which sample represents a continuous pass, so this implementation retains the largest contribution during that pass.

Implementation boundary
The JSON contains beam geometry, the thickness table, speed coefficients and stepping parameters. The speed coefficients follow Eq. (8), but the bundled distance-angle thickness table is a local estimate, not the paper's original measurements. Quantitative comparison requires corresponding experimental data.)");
        case Kind::Wu2020:
            return chinese ? QString::fromUtf8(R"(计算对象
在原始基底网格上叠加离散沉积圆柱，模拟逐步增长的涂层几何。仅使用输入的原始喷涂点，不再进行内部轨迹插值；每个有效喷枪位姿发射一组覆盖喷束角度范围的射线。

几何命中与遮挡
每条射线只取基底或此前喷涂点沉积圆柱上的最近命中点。同一喷涂点的所有射线先完成查询，再统一提交新圆柱，因此不会在同一位姿内自堆叠。前方几何自然截断射线，后续喷涂点可以沉积到已有圆柱上。

沉积高度
命中位置的径向距离进入高斯函数；喷涂角、喷涂距离和喷枪横移速度分别进入标定的相对沉积效率或峰值修正函数。每个喷涂点的高度按到下一点的时间间隔与标定参考时长之比缩放，末点不额外沉积；计算出的高度与给定圆柱半径共同形成一个新的沉积圆柱。

实现边界
这里输出的核心是离散圆柱及其高度，不是简单的固定网格顶点逐点求和。算法复现生成平板使用更密射线和按喷涂距离匹配的物理圆柱半径；法向高度放大只改变显示，不改变真实厚度。结果仍受输入轨迹密度与经验标定曲线影响。)")
                : QStringLiteral(R"(Evaluation target
Discrete deposited cylinders are added to an initial substrate mesh to represent growing coating geometry. Only the supplied nozzle poses are used, without internal trajectory interpolation. Each active pose emits rays over the configured angular range.

Hits and shadowing
Each ray uses its nearest hit on the substrate or cylinders deposited by earlier poses. All rays at one pose are evaluated before that pose's cylinders are committed, preventing self-stacking within a pose. Front geometry blocks farther surfaces, while later poses can hit the growing deposit.

Deposited height
Radial distance enters a Gaussian profile. Spray angle, spray distance and traverse speed enter calibrated relative-deposition-efficiency or peak-correction functions. Each pose's height is scaled by its interval to the next pose relative to the calibration duration; the final pose adds no deposition. The resulting height and configured radius define a new deposited cylinder.

Implementation boundary
The native result is a set of cylinders and heights, not a simple per-vertex sum on a fixed mesh. Generated plate scenes use denser rays and a physical radius matched to the spray distance. Normal-height scaling affects only display geometry, not the physical thickness. Input pose density and empirical calibration still affect the outcome.)");
        case Kind::Fuke2005:
            return chinese ? QString::fromUtf8(R"(计算对象
把工件表面划分为多边形，以多边形面心作为厚度评价位置。输入为蒸发源与工件之间带时间的相对位姿；程序按位姿更新面心和法向。

通量模型
蒸发源的方向性用源轴与射出方向夹角余弦的幂次表示；传播到表面的强度再按距离平方反比衰减，并乘以面法向的入射投影。参考厚度速率、参考距离和羽流指数由 JSON 提供。

可见性与时间累计
仅当前向发射且表面迎向蒸发源时才有贡献。对每个面心，从源点进行射线可见性检查；若更近的其他面片挡住射线，该面本次不沉积。可见面的速率乘以相邻位姿时间差并累加。

实现边界
该方法表达的是定向蒸发源通量，不是喷枪椭圆高斯喷斑。计算单元是面心，所以很小的局部特征需要足够细的多边形网格才能分辨。)")
                : QStringLiteral(R"(Evaluation target
The workpiece is divided into polygons, with thickness evaluated at polygon centroids. Timed relative source/workpiece poses update each centroid and its normal.

Flux model
Source directionality is a power of the cosine between the source axis and emission direction. Flux at the surface falls with inverse-square distance and is multiplied by the surface incidence projection. The JSON supplies reference rate, reference distance and plume exponent.

Visibility and time accumulation
Only forward emission onto a facing surface contributes. A ray from the source to each centroid checks whether another triangle lies closer; blocked polygons receive no deposition for that interval. Visible rates are multiplied by pose duration and accumulated.

Implementation boundary
This is a directional vapor-source model, not an elliptical Gaussian spray-gun footprint. Centroid evaluation requires sufficiently fine polygons to resolve small local features.)");
        case Kind::Vanerio2021:
            return chinese ? QString::fromUtf8(R"(计算对象
使用可演化的 STL 三角表面。计算前先把超过最大边长的网格边二分，以便后续涂层生长能够改变局部表面形状。

候选与遮挡
每个喷枪位姿先把喷束中的顶点投影到横向网格单元，选取各单元最靠近喷枪的顶点，再将其相邻面片视为可见候选。这是离散阴影网格近似，而不是对每个面心执行精确射线求交。

沉积与表面生长
对候选面片计算面心、法向、喷涂距离、倾斜度与径向距离。径向分布随距离标定的拉伸量改变，同时乘以角度效率、距离效率和生长速率；乘以时间步长得到面片增量。相邻面片增量平均到顶点，并沿生长方向移动顶点；必要时再次细分过长的边。

实现边界
该方法直接改变网格形状，因此与固定网格厚度求和的结果定义不同。随附 JSON 的材料效率与喷斑拉伸曲线是本地估计，需用目标材料的实测值替换；论文也没有给出完整重网格实现，当前程序使用最长边二分。)")
                : QStringLiteral(R"(Evaluation target
An evolving STL triangle surface is used. Before prediction, edges exceeding the configured maximum length are bisected so later coating growth can change local shape.

Candidates and shadowing
For each pose, beam-region vertices are projected into transverse grid cells. The nearest vertex in each cell marks adjacent faces as visible candidates. This is a discrete shadow-grid approximation, not an exact ray test for every face centroid.

Deposition and surface growth
Candidate faces are evaluated at their centroids using normal, distance, inclination and radial offset. A distance-dependent stretch changes the radial profile, which is multiplied by angle efficiency, distance efficiency, growth rate and time step. Neighboring face increments are averaged onto vertices and displace them along the growth direction. Long edges are bisected again when needed.

Implementation boundary
This method changes the mesh itself, so its output differs in meaning from a sum on a fixed mesh. The bundled material-efficiency and footprint-stretch curves are local estimates requiring measurements for the chosen material. The paper does not publish a complete remesher; this implementation uses longest-edge bisection.)");
        case Kind::DynamicSurface2026:
            return chinese ? QString::fromUtf8(R"(计算对象
从初始 STL 表面出发，以等效颗粒和沉积圆柱表示涂层增长。带时间的喷枪轨迹按设定批次处理，每批更新一次表面，而不只在原始顶点上记录厚度。

颗粒投射
在喷枪局部坐标中按极角分布生成加权的射出方向，并按给定随机种子抽取颗粒。射线与当前表面求最近交点；未命中的颗粒丢弃。随附标定在论文实验的 0–40° 范围内使用 Rbu(θ)=1−1.482×10⁻⁴θ²（θ 以度计）调整沉积圆柱高度；超出该范围不外推。

重叠与重建
同批圆柱的重叠体积按本实现的规则分配；随后提取圆柱顶端和表面点，进行点云降采样、法向估计、区域识别和局部表面重建。重建后的网格成为下一批颗粒的碰撞表面，因此前一批沉积会影响后续命中。

实现边界
这是计算和内存开销较高的动态几何方法。论文未公开全部重叠求解、孔洞修补与平滑数值细节，程序采用可复现的近似实现；结果还受随机种子、批次长度和点云重建参数影响。)")
                : QStringLiteral(R"(Evaluation target
Starting from an STL surface, coating growth is represented by equivalent particles and deposited cylinders. A timed nozzle path is processed in batches, updating the surface after each batch rather than only storing thickness on original vertices.

Particle projection
Weighted ejection directions are generated in gun-local polar coordinates and sampled with a configured random seed. Each ray takes the nearest hit on the current surface; misses are discarded. The bundled calibration uses Rbu(theta)=1-1.482e-4*theta^2 (theta in degrees) over the paper's measured 0-40 degree range to set cylinder height; it does not extrapolate beyond that range.

Overlap and reconstruction
Overlapping volume among same-batch cylinders is redistributed by this implementation. Cylinder tips and surface points feed point-cloud downsampling, normal estimation, region detection and local surface reconstruction. The rebuilt mesh becomes the collision surface for later particles, so earlier deposition affects subsequent hits.

Implementation boundary
This dynamic-geometry approach is computationally and memory intensive. The paper does not specify all overlap, hole-repair and smoothing numerics, so the program uses reproducible approximations. Random seed, batch duration and reconstruction settings also affect the result.)");
        }
        return {};
    }
}
