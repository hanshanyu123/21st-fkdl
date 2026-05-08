# 智能车工程修改记录

## v0 - 2026-05-08 修改前基线

- 视觉入口为 `code/camera.c`。
- 摄像头使用 MT9V03X 1 路，分辨率 `188 x 120`。
- 图像处理流程为：复制一帧灰度图 -> Otsu 阈值 -> 单阈值二值化 -> 从上一行中点向左右搜索线段 -> 计算 `track_offset`。
- 当前设置为识别白线：`TRACK_LINE_IS_BLACK 0`。
- 屏幕上方显示摄像头图像，下方显示控制状态。
- IMU 初始化在 `user/cpu0_main.c` 中通过 `USE_IMU_MODULE 0` 临时关闭，避免未接 IMU 时日志占屏。
- 已知问题：视觉调试信息不足，只能看到 `Lost/Off`，无法直接判断阈值、曝光和白点分布是否正常。

## v1 - 2026-05-08 视觉第一步升级

- 在 `code/camera.c` 增加视觉诊断数据：
  - `camera_threshold_base`
  - `camera_threshold_near`
  - `camera_threshold_mid`
  - `camera_threshold_far`
  - `white_num_row[]`
  - `white_num_col[]`
- 二值化阶段改为按近/中/远三段阈值处理：
  - 顶部三分之一使用远景阈值。
  - 中部三分之一使用中景阈值。
  - 底部三分之一使用近景阈值。
- 当前三个阈值偏移量先全部设为 `0`，行为等价于原单阈值，方便后续逐步调参。
- 新增视觉数据接口：
  - `Camera_GetThreshold()`
  - `Camera_GetThresholdNear()`
  - `Camera_GetThresholdMid()`
  - `Camera_GetThresholdFar()`
  - `Camera_GetWhiteRow(row)`
  - `Camera_GetWhiteCol(col)`
- 在 `code/control.c` 屏幕状态最后一行显示：
  - `Th`：当前 Otsu 基础阈值。
  - `Wr`：图像中下部一行的白点数量。
  - `Wc`：图像中间列的白点数量。
- 未执行编译、烧录或硬件测试。
## v1.1 - 2026-05-08 远处过曝阈值初调

- 针对白线识别时远处画面过曝、背景容易被误识别为白线的问题，调整三段阈值偏移：
  - `CAMERA_CLOSE_THRESHOLD_OFFSET 5`
  - `CAMERA_MID_THRESHOLD_OFFSET 10`
  - `CAMERA_FAR_THRESHOLD_OFFSET 25`
- 远处阈值提高最多，目的是减少顶部区域亮背景误判。
- 未执行编译、烧录或硬件测试。
## v1.2 - 2026-05-08 摄像头过曝源头处理

- 远处仍然严重过曝，判断仅靠三段阈值不足，需要先降低摄像头原始灰度亮度。
- 调整 `libraries/zf_device/zf_device_mt9v03x_double.h` 中摄像头 1 参数：
  - `MT9V03X_1_EXP_TIME_DEF`：`512` -> `180`
  - `MT9V03X_1_GAIN_DEF`：`32` -> `16`
- 保持自动曝光关闭：`MT9V03X_1_AUTO_EXP_DEF 0`。
- 未执行编译、烧录或硬件测试。

## v1.3 - 2026-05-08 视觉抗割裂与白线误判收紧

- 参考 `草莽-缩微光电代码` 的图像处理思路，当前白线二值化不再只看单个像素是否超过阈值。
- 在 `code/camera.c` 中加入 3x3 邻域一致性判断：周围 9 个点中至少 6 个满足白线阈值，才保留为白点。
- 目的：减少反光、噪点、局部亮斑导致的整片误识别，让 `white_num_row/white_num_col` 更接近真实白线分布。
- 修正摄像头帧交接逻辑：`libraries/zf_device/zf_device_mt9v03x_double.c` 在单摄模式下帧完成后不立即重启 DMA，而是等 `mt9v03x_finish_flag_1` 被主循环复制并清除后，再开始下一帧采集。
- 在 `code/camera.c` 中保证先复制稳定帧，再清除 `mt9v03x_finish_flag_1`，降低屏幕图像割裂和视觉处理读到半帧数据的概率。
- 继续降低摄像头 1 原始曝光：`MT9V03X_1_EXP_TIME_DEF` 当前为 `80`，`MT9V03X_1_GAIN_DEF` 保持 `16`，自动曝光保持关闭。
- 未执行编译、烧录或硬件测试。

## v1.4 - 2026-05-08 修复摄像头图像割裂

- 回滚上一版中“延迟 DMA 重启”的采集时序改动，恢复逐飞驱动原本的 VSYNC/DMA 状态机，避免出现两幅图像拼接在同一屏的问题。
- 在 `libraries/zf_device/zf_device_mt9v03x_double.c` 中改为帧完成后、DMA 重启前复制一份完整图像。
- 当前工程只初始化 `mt9v03x_1` 单摄，因此复用未启用的 `mt9v03x_image_2` 作为稳定帧缓存，不额外增加 RAM 占用。
- `code/camera.c` 改为从稳定帧缓存读取图像，不再直接读取正在被 DMA 写入的 `mt9v03x_image_1`。
- 未执行编译、烧录或硬件测试。
