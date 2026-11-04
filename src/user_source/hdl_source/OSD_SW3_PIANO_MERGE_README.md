# OSD + SW3 琴键测试模式合并版

基线：`lab_ex5_i2s_step7_c4_osd.zip`

本版本只做“已有 OSD 分支 + 已验证琴键测试分支”的合并，不加入 Piano Roll / 多音 / 真实钢琴采样。

## 开关与按键

### SW3 = 0：正常模式
- OSD 关闭。
- SW1=1：KEY1/KEY2 保持原图片控制。
- SW1=0：KEY1=BPM+1，KEY2=BPM-1，KEY4=拍数+1。
- UI click 保持原行为。
- SW2 仍控制节拍器。

### SW3 = 1：琴键测试 + OSD
- 底部 OSD 开启，继续显示 BPM、拍号、当前拍。
- KEY1 = C4（MIDI 60）
- KEY2 = D4（MIDI 62）
- KEY4 = E4（MIDI 64）
- 当前仍为单音模式，优先级：C4 > D4 > E4。
- 原图片翻页/BPM/拍数按键功能在此模式下屏蔽。
- 琴键不再叠加 UI click。
- SW2 节拍器继续运行，并可与琴音同时混音；OSD 当前拍高亮继续跟随。
- SW1 仍保持原显示/数码管模式选择，但不会抢占琴键。
- SW4 仍只预留给后续 Piano Roll / 音轨编辑器，本版本不使用。

## Note Event
50MHz 控制域现在产生：
- `note_evt_toggle`
- `note_evt_on`
- `note_evt_note[6:0]`
- `note_evt_velocity[6:0]`

SW3 从 1 切回 0 时，如果琴音正在响，会自动产生 NOTE_OFF，避免卡音。

后续接 25 键矩阵时，只需要把板载 KEY1/KEY2/KEY4 的事件源替换为矩阵扫描事件，后端声音链无需重写。

## 音频
在原 `hdmi_audio_tone_i2s_64fs` 中增加第三路单音 DDS：
- C4 = 261.6256 Hz
- D4 = 293.6648 Hz
- E4 = 329.6276 Hz
- 方波幅度 = 2^21（沿用之前增大后的验证音量）

Mixer 现在为：
`metronome + UI click + piano test voice`

原 I2S 时序、strong/weak/click ROM、TF、SDRAM、视频双缓冲都未重构。

## OSD
沿用本基线已有的 `music_score_osd`：
- SW3 为 OSD 总开关。
- 底部区域显示 BPM、拍号和当前拍进度。
- OSD 只在视频输出路径叠加，不写回 SDRAM。

本版本暂时没有把 C4/D4/E4 的音符块画到 OSD；那作为下一步单独验证。

## 修改文件
- `src/user_source/hdl_source/top_tf_hdmi_audio.v`
- `src/user_source/hdl_source/hdmi_audio_tone_i2s_64fs.v`
- 新增本说明文件

注意：基线里已有 `piano_c4_dds.v`，但当前工程项目文件没有把它加入综合列表。本合并版继续使用现有 `hdmi_audio_tone_i2s_64fs` 作为唯一 I2S/Mixer 路径，因此没有启用那个独立实验模块。
