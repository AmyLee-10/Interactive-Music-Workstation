# Step7 C4电子琴最小链路

新增：
- OSD计划缩小为48px版本
- 增加piano_c4_dds模块

当前目标：
KEY1 -> note_on -> C4 -> HDMI Audio

第一版：
- DDS生成C4
- 带简单release衰减
- 不占BRAM

后续：
DDS替换为钢琴采样PCM
+ ADSR包络
+ 多音符事件
